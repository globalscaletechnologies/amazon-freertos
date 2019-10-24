/** @file apds9301.c
 *
 *  @brief This file provides driver for Ambient Light Sensor(APDS-9301)
 *
 *  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
 *
 */

#include <wmstdio.h>
#include <stdbool.h>
#include <mdev_gpio.h>
#include <mdev_i2cbus.h>
#include <apds9301.h>
#include <wm_os.h>
#include <lowlevel_drivers.h>

#define DIV_ROUND_CLOSEST(x,y) ((x + (y / 2)) / y)

typedef struct _mdev_apds9301_priv_data {
	mdev_t *dev;
	mdev_t *i2cbus;
	mdev_t *gpiod;

	int irq;
	uint8_t exit;
	apds9301_irq_cb callback;

	uint8_t power_state;
	uint8_t gain;
	uint8_t integ_time;
	uint8_t intr_enable;
	uint8_t intr_persist;
	uint16_t low_threshold;
	uint16_t high_threshold;

	os_mutex_t rw_mutex;
	os_semaphore_t event_sem;
	os_semaphore_t exit_sem;
	os_thread_t irq_thread;
} mdev_apds9301_priv_data_t;

static os_thread_stack_define(apds9301_thread_stack, 1024);

static mdev_t mdev_apds9301;
static mdev_apds9301_priv_data_t apds9301_priv_dara;

/* Lux calculation */

/* Calculated values 1000 * (CH1/CH0)^1.4 for CH1/CH0 from 0 to 0.52 */
static const uint16_t apds9301_lux_ratio[] = {
	0, 2, 4, 7, 11, 15, 19, 24, 29, 34, 40, 45, 51, 57, 64, 70, 77, 84, 91,
	98, 105, 112, 120, 128, 136, 144, 152, 160, 168, 177, 185, 194, 203,
	212, 221, 230, 239, 249, 258, 268, 277, 287, 297, 307, 317, 327, 337,
	347, 358, 368, 379, 390, 400,
};

static inline void *mdev_to_priv(mdev_t *dev)
{
	return (dev)? (void *)(dev->private_data) : NULL;
}

static int apds9301_getReg(mdev_t *dev, APDS9301_Register_t reg)
{
	int word = 0, ret, val = 0;
	uint8_t cmd = 0;
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	switch (reg) {
		case rControl:
		case rTiming:
		case rInterrupt:
		case rID:
			word = 0;
		break;
		case rThresholdLow:
		case rThresholdHigh:
		case rData0:
		case rData1:
			word = 1;
			cmd |= APDS9301_WORD;
		break;
		default:
		return -WM_FAIL;
	}

	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	cmd |= APDS9301_CMD | reg;
	if (!word) {
		ret = i2cbus_drv_read_byte_data(privdata->i2cbus, &val, cmd, 1);
	} else {
		ret = i2cbus_drv_read_word_data(privdata->i2cbus, &val, cmd, 1);
	}
	os_mutex_put(&privdata->rw_mutex);

	if (ret > 0) {
		return (int)val;
	}
	return -WM_FAIL;
}

static int apds9301_setReg(mdev_t *dev, APDS9301_Register_t reg,
						   int val)
{
	int word = 0, ret;
	uint8_t cmd = 0;
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	switch (reg) {
		case rTiming:
		case rID:
			word = 0;
		break;
		case rControl:
		case rInterrupt:
			word = 0;
			/* force to clear interrupt flag */
			cmd |= APDS9301_CLEAR;
		break;
		case rThresholdLow:
		case rThresholdHigh:
		case rData0:
		case rData1:
			word = 1;
			cmd |= APDS9301_WORD;
		break;
		default:
		return -WM_FAIL;
	}

	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	cmd |= APDS9301_CMD | reg;

	if (!word) {
		ret = i2cbus_drv_write_byte_data(privdata->i2cbus, &val, cmd, 1);
	} else {
		ret = i2cbus_drv_write_word_data(privdata->i2cbus, &val, cmd, 1);
	}
	os_mutex_put(&privdata->rw_mutex);

	if (ret > 0) {
		return WM_SUCCESS;
	}
	return -WM_FAIL;
}

static int apds9301_clearInterrupt(mdev_apds9301_priv_data_t *privdata)
{
	int ret = -WM_FAIL;
	uint8_t command = APDS9301_CMD | APDS9301_CLEAR;

	if (privdata) {
		os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
		ret = i2cbus_drv_write_bytes(privdata->i2cbus, &command, 1);
		os_mutex_put(&privdata->rw_mutex);
	}
	return ret;
}

int apds9301_powerOn(mdev_t *dev)
{
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	if (privdata->power_state) {
		return WM_SUCCESS;
	}

	if (!apds9301_setReg(dev, rControl, APDS9301_POWER_ON)) {
		privdata->power_state = 1;
		return WM_SUCCESS;
	}
	return -WM_FAIL;
}

int apds9301_powerOff(mdev_t *dev)
{
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	if (privdata->power_state == 0) {
		return WM_SUCCESS;
	}
	if (!apds9301_setReg(dev, rControl, APDS9301_POWER_OFF)) {
		privdata->power_state = 0;
		return WM_SUCCESS;
	}
	return -WM_FAIL;
}

int apds9301_isPowerOn(mdev_t *dev)
{
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	return privdata->power_state;
}

int apds9301_setLowThreshold(mdev_t *dev, uint16_t threshold)
{
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata || !privdata->power_state) {
		return -WM_FAIL;
	}

	if (!apds9301_setReg(dev, rThresholdLow, threshold)) {
		privdata->low_threshold = threshold;
		return WM_SUCCESS;
	}
	return -WM_FAIL;
}

int apds9301_setHighThreshold(mdev_t *dev, uint16_t threshold)
{
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata || !privdata->power_state) {
		return -WM_FAIL;
	}

	if (!apds9301_setReg(dev, rThresholdHigh, threshold)) {
		privdata->high_threshold = threshold;
		return WM_SUCCESS;
	}
	return -WM_FAIL;
}

int apds9301_setADCGain(mdev_t *dev, uint8_t gain)
{
	int reg;
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata || !privdata->power_state || gain > 1) {
		return -WM_FAIL;
	}

	if ((reg = apds9301_getReg(dev, rTiming)) == -WM_FAIL) {
		goto f_gain;
	}
	if (gain) {
		reg |= APDS9301_GAIN;
	} else {
		reg &= ~APDS9301_GAIN;
	}

	if (!apds9301_setReg(dev, rTiming, reg)) {
		privdata->gain = gain;
		return WM_SUCCESS;
	}
f_gain:
	return -WM_FAIL;
}

int apds9301_getADCGain(mdev_t *dev)
{
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	return privdata->gain;
}

int apds9301_setIntegrationTime(mdev_t *dev, uint8_t time)
{
	int reg;
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata || !privdata->power_state) {
		return -WM_FAIL;
	}

	if ((reg = apds9301_getReg(dev, rTiming)) == -WM_FAIL) {
		goto f_integ;
	}

	switch (time) {
		case APDS9301_INTEG_LOW:
		case APDS9301_INTEG_MIDIUM:
		case APDS9301_INTEG_HIGH:
			reg = (reg & ~APDS9301_INTEG) | time;
		break;
		default:
		goto f_integ;
	}
	if (!apds9301_setReg(dev, rTiming, reg)) {
		privdata->integ_time = time;
		return WM_SUCCESS;
	}
f_integ:
	return -WM_FAIL;
}

int apds9301_getIntegrationTime(mdev_t *dev)
{
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	return privdata->integ_time;
}

int apds9301_setInterruptOn(mdev_t *dev)
{
	int reg;
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata || !privdata->power_state) {
		return -WM_FAIL;
	}
	if (privdata->intr_enable) {
		return WM_SUCCESS;
	}

	if ((reg = apds9301_getReg(dev, rInterrupt)) == -WM_FAIL) {
		goto f_intr;
	}
	reg = (reg & ~APDS9301_INTR_ENABLE_MASK) | APDS9301_INTR_ENABLE;
	if (!apds9301_setReg(dev, rInterrupt, reg)) {
		privdata->intr_enable = 1;
		return WM_SUCCESS;
	}
f_intr:
	return -WM_FAIL;
}

int apds9301_setInterruptOff(mdev_t *dev)
{
	int reg;
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata || !privdata->power_state) {
		return -WM_FAIL;
	}
	if (!privdata->intr_enable) {
		return WM_SUCCESS;
	}

	if ((reg = apds9301_getReg(dev, rInterrupt)) == -WM_FAIL) {
		goto f_intr;
	}
	reg &= ~APDS9301_INTR_ENABLE_MASK;
	if (!apds9301_setReg(dev, rInterrupt, reg)) {
		privdata->intr_enable = 0;
		return WM_SUCCESS;
	}
f_intr:
	return -WM_FAIL;
}

int apds9301_getInterruptState(mdev_t *dev)
{
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	return privdata->intr_enable;
}

int apds9301_setGenInterruptEveryADCCycle(mdev_t *dev)
{
	int reg;
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata || !privdata->power_state) {
		return -WM_FAIL;
	}

	if ((reg = apds9301_getReg(dev, rInterrupt)) == -WM_FAIL) {
		goto f_persist;
	}
	reg &= ~APDS9301_INTR_PERSIST_MASK;
	if (!apds9301_setReg(dev, rInterrupt, reg)) {
		privdata->intr_persist = 0;
		return WM_SUCCESS;
	}
f_persist:
	return -WM_FAIL;
}

int apds9301_setGenInterruptPeriodOfOutRange(mdev_t *dev, int period)
{
	int reg;
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata || !privdata->power_state) {
		return -WM_FAIL;
	}

	if (period < 1 || period > 15) {
		goto f_persist;
	}
	if ((reg = apds9301_getReg(dev, rInterrupt)) == -WM_FAIL) {
		goto f_persist;
	}
	reg = (reg & ~APDS9301_INTR_PERSIST_MASK) | period;
	if (!apds9301_setReg(dev, rInterrupt, reg)) {
		privdata->intr_persist = period;
		return WM_SUCCESS;
	}
f_persist:
	return -WM_FAIL;
}

int apds9301_getGenerateInterruptMode(mdev_t *dev)
{
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	return privdata->intr_persist;
}

int apds9301_getADCChannel(mdev_t *dev, int channel)
{
	int adc;
	APDS9301_Register_t reg;
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata || !privdata->power_state) {
		return -WM_FAIL;
	}

	if (channel < 0 || channel > 1) {
		goto f_adc;
	}
	reg = (channel == 0)? rData0 : rData1;

	if ((adc = apds9301_getReg(dev, reg)) == -WM_FAIL) {
		goto f_adc;
	}
	return adc;
f_adc:
	return -WM_FAIL;
}

/*
	CH1/CH0			        Sensor Lux Formula
	========================================================================================
	0 < CH1/CH0 ≤ 0.50      Sensor Lux = (0.0304 x CH0) – (0.062 x CH0 x ((CH1/CH0)^1.4))
	0.50 < CH1/CH0 ≤ 0.61	Sensor Lux = (0.0224 x CH0) – (0.031 x CH1)
	0.61 < CH1/CH0 ≤ 0.80	Sensor Lux = (0.0128 x CH0) – (0.0153 x CH1)
	0.80 < CH1/CH0 ≤ 1.30	Sensor Lux = (0.00146 x CH0) – (0.00112 x CH1)
	CH1/CH0>1.30            Sensor Lux = 0
	========================================================================================
 */
float apds9301_getLux(mdev_t *dev)
{
	int adc0, adc1, gain, integ, idx;
	float ch0, ch1, ratio, lux = 0.0;
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata || !privdata->power_state) {
		return -1.0;
	}

	adc0 = apds9301_getADCChannel(dev, 0);
	adc1 = apds9301_getADCChannel(dev, 1);

	if (adc0 <= 0 || adc1 <= 0) {
		return -1.0;
	}

	ch0 = (float) adc0;
	ch1 = (float) adc1;
	ratio = ch1 / ch0;

	idx = DIV_ROUND_CLOSEST(100 * adc1, adc0);

	integ = apds9301_getIntegrationTime(dev);
	switch (integ) {
		case APDS9301_INTEG_LOW:
		if (adc0 >= 5047 || adc1 >= 5047) {
			return -1.0;
		}
		ch0 *= 1.0 / 0.034;
		ch1 *= 1.0 / 0.034;
		break;
		case APDS9301_INTEG_MIDIUM:
		if (adc0 >= 37177 || adc1 >= 37177) {
			return -1.0;
		}
		ch0 *= 1.0 / 0.252;
		ch1 *= 1.0 / 0.252;
		break;
		case APDS9301_INTEG_HIGH:
		if (adc0 >= 65535 || adc1 >= 65535) {
			return -1.0;
		}
		ch0 *= 1.0;
		ch1 *= 1.0;
		break;
	}

	gain = apds9301_getADCGain(dev);
	if (gain == 0) {
		ch0 *= 16.0;
		ch1 *= 16.0;
	}

	if (ratio <= 0.5) {
		lux = (0.0304 * ch0) -
			   ((0.062 * ch0) * (float)apds9301_lux_ratio[idx] / 1000.0);
	} else if (ratio <= 0.61) {
		lux = (0.0224 * ch0) - (0.031 * ch1);
	} else if (ratio <= 0.8) {
		lux = (0.0128 * ch0) - (0.0153 * ch1);
	} else if (ratio <= 1.3) {
		lux = (0.00146 * ch0) - (0.00112 * ch1);
	} else {
		lux = 0.0;
	}
	return lux;
}

int apds9301_getPartNum(mdev_t *dev)
{
	int id;
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	id = apds9301_getReg(dev, rID);
	return (id == -WM_FAIL)? 0: (id & 0xF0) >> 4;
}

int apds9301_getRevNum(mdev_t *dev)
{
	int id;
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	id = apds9301_getReg(dev, rID);
	return (id == -WM_FAIL)? 0: (id & 0xF);
}

static void apds9301_irq_handler(int pin, void *data)
{
	mdev_apds9301_priv_data_t *privdata = data;

	if (privdata) {
		os_semaphore_put(&privdata->event_sem);
	}
	return;
}

int apds9301_setInterruptCallback(mdev_t *dev, apds9301_irq_cb cb)
{
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}

	privdata->callback = cb;
	return WM_SUCCESS;
}

static void apds9301_irq_thread(os_thread_arg_t data)
{
	mdev_apds9301_priv_data_t *privdata = data;

	do {
		os_semaphore_get(&privdata->event_sem, OS_WAIT_FOREVER);
		if (privdata->exit) {
			break;
		}
		if (privdata->callback) {
			privdata->callback(privdata->dev);
		}
		apds9301_clearInterrupt(privdata);
	} while (1);

	os_semaphore_put(&privdata->exit_sem);
	os_thread_self_complete(NULL);
}

mdev_t *apds9301_drv_open(const char *name)
{
	mdev_t *dev = mdev_get_handle(name);

	if (dev == NULL) {
		return NULL;
	}
	return dev;
}

int apds9301_drv_close(mdev_t *dev)
{
	mdev_apds9301_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	if (privdata->power_state) {
		apds9301_powerOff(dev);
	}
	return 0;
}

static int apds9301_drv_setup(I2C_ID_Type id, uint16_t addr, int gpio)
{
	int ret, control, timing, interrupt;
	mdev_t *dev = &mdev_apds9301;
	mdev_apds9301_priv_data_t *privdata = &apds9301_priv_dara;

	dev->port_id = 0;
	dev->name = MDEV_APDS9301;
	dev->pNextMdev = NULL;
	dev->private_data = (uint32_t)privdata;

	privdata->dev = dev;

	ret = os_mutex_create(&privdata->rw_mutex, "apds9301_rw",
						  OS_MUTEX_INHERIT);
	if (ret != WM_SUCCESS) {
		return -WM_FAIL;
	}

	ret = os_semaphore_create(&privdata->event_sem, "apds9301_irq");
	if (ret != WM_SUCCESS) {
		goto fail_sem_create;
	}
	os_semaphore_get(&privdata->event_sem, OS_WAIT_FOREVER);

	ret = os_semaphore_create(&privdata->exit_sem, "apds9301_exit");
	if (ret != WM_SUCCESS) {
		goto fail_sem_create_1;
	}
	os_semaphore_get(&privdata->exit_sem, OS_WAIT_FOREVER);

	ret = os_thread_create(&privdata->irq_thread, "apds9301_irq_thread",
						   apds9301_irq_thread, privdata,
						   &apds9301_thread_stack, OS_PRIO_2);
	if (ret != WM_SUCCESS) {
		goto fail_create_thread;
	}

	ret = i2cbus_drv_init(id);
	if (ret != WM_SUCCESS) {
		goto fail_create_thread;
	}

	privdata->i2cbus = i2cbus_drv_open(id, addr, 0);
	if (!privdata->i2cbus) {
		goto fail_create_thread;
	}

	privdata->irq = gpio;
	privdata->power_state = 0;
	privdata->gain = 0;
	privdata->integ_time = APDS9301_INTEG_HIGH;
	privdata->intr_enable = 0;
	privdata->intr_persist = 0;
	privdata->low_threshold = 0;
	privdata->high_threshold = 0;
	privdata->callback = NULL;

	/* initialize apds9301 register */
	control =
		(privdata->power_state)? APDS9301_POWER_ON : APDS9301_POWER_OFF;
	timing = ((privdata->gain)? APDS9301_GAIN : 0) |
			 privdata->integ_time;
	interrupt = ((privdata->intr_enable)? APDS9301_INTR_ENABLE : 0) |
				privdata->intr_persist;

	apds9301_setReg(dev, rControl, control);
	apds9301_setReg(dev, rTiming, timing);
	apds9301_setReg(dev, rInterrupt, interrupt);
	apds9301_setReg(dev, rThresholdLow, privdata->low_threshold);
	apds9301_setReg(dev, rThresholdHigh, privdata->high_threshold);

	gpio_drv_init();
	privdata->gpiod = gpio_drv_open("MDEV_GPIO");
	gpio_drv_setdir(privdata->gpiod, gpio, GPIO_INPUT);
	ret = gpio_drv_set_cb(privdata->gpiod, gpio, GPIO_INT_FALLING_EDGE,
						  privdata, apds9301_irq_handler);
	if (ret != WM_SUCCESS) {
		goto fail_gpio_init;
	}
	apds9301_clearInterrupt(privdata);
	return WM_SUCCESS;

fail_gpio_init:
	gpio_drv_close(privdata->gpiod);
	privdata->gpiod = NULL;

	i2cbus_drv_close(privdata->i2cbus);
	privdata->i2cbus = NULL;
fail_create_thread:
	os_semaphore_delete(&privdata->exit_sem);
fail_sem_create_1:
	os_semaphore_delete(&privdata->event_sem);
fail_sem_create:
	os_mutex_delete(&privdata->rw_mutex);
	return ret;
}

int apds9301_drv_init(I2C_ID_Type id, uint16_t addr, int gpio)
{
	if (mdev_get_handle(MDEV_APDS9301) != NULL) {
		return WM_SUCCESS;
	}
	if (apds9301_drv_setup(id, addr, gpio) != WM_SUCCESS) {
		return -WM_FAIL;
	}
	return mdev_register(&mdev_apds9301);
}

int apds9301_drv_deinit(I2C_ID_Type id)
{
	mdev_apds9301_priv_data_t *privdata = &apds9301_priv_dara;

	if (!mdev_get_handle(MDEV_APDS9301)) {
		return WM_SUCCESS;
	}
	if (privdata->power_state) {
		apds9301_powerOff(privdata->dev);
	}

	privdata->exit = 1;

	os_semaphore_get(&privdata->exit_sem, os_msec_to_ticks(1000));

	gpio_drv_set_cb(privdata->gpiod, privdata->irq, GPIO_INT_DISABLE,
					NULL, NULL);
	gpio_drv_close(privdata->gpiod);

	i2cbus_drv_close(privdata->i2cbus);

	os_semaphore_delete(&privdata->event_sem);
	os_mutex_delete(&privdata->rw_mutex);
	mdev_deregister(MDEV_APDS9301);
	return WM_SUCCESS;
}
