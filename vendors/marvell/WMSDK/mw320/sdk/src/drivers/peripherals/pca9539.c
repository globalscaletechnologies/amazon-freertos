/*
 * mdev_pca9539.c: mdev driver for GPIO expander (pca9539)
 */
#include <wmstdio.h>
#include <mdev_gpio.h>
#include <mdev_i2cbus.h>
#include <pca9539.h>
#include <wm_os.h>
#include <lowlevel_drivers.h>

#define PCA9539_PINS					16

#define PCA9539_INT_DISABLE				(0x0)
#define PCA9539_INT_FALLING_EDGE		(0x1)
#define PCA9539_INT_RISING_EDGE			(0x2)
#define PCA9539_INT_BOTH_EDGE			(0x3)

#define PCA9539_INPUT_REG				0x00
#define PCA9539_OUTPUT_REG				0x02
#define PCA9539_POLARITY_REG			0x04
#define PCA9539_CONFIG_REG				0x06

typedef struct _pca9539_gpio_irq{
	void *data;
	gpio_irq_cb gpio_cb;
} pca9539_gpio_irq_t;

typedef struct _pca9539_priv {
	mdev_t *dev;
	mdev_t *i2cbus;
	mdev_t *gpiod;

	int irq;
	int pins;
	uint16_t config;
	uint16_t polarity;
	uint16_t output;
	uint16_t status;
	uint32_t irq_type;
	uint8_t exit;
	pca9539_gpio_irq_t callbacks[PCA9539_PINS];

	os_mutex_t open_mutex;
	os_mutex_t rw_mutex;
	os_semaphore_t event_sem;
	os_semaphore_t exit_sem;
	os_thread_t irq_thread;
} pca9539_priv_t;

static os_thread_stack_define(irq_thread_stack, 1024);

static mdev_t mdev_pca9539;
static pca9539_priv_t pca9539_priv_data;

static inline void *mdev_to_priv(mdev_t *dev)
{
	return (dev)? (void *)(dev->private_data) : NULL;
}

static int pca9539_getReg(mdev_t *dev, uint8_t reg, uint16_t *val)
{
	int ret = -WM_FAIL;
	pca9539_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata || !val) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	switch (reg) {
		case PCA9539_INPUT_REG:
		case PCA9539_POLARITY_REG:
		case PCA9539_CONFIG_REG:
			ret = i2cbus_drv_read_word_data(privdata->i2cbus, val, reg,
											1);
		break;
	}
	os_mutex_put(&privdata->rw_mutex);

	return (ret > 0)? WM_SUCCESS : -WM_FAIL;
}

static int pca9539_setReg(mdev_t *dev, uint8_t reg, uint16_t val)
{
	int ret = -WM_FAIL;
	pca9539_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	switch (reg) {
		case PCA9539_OUTPUT_REG:
		case PCA9539_POLARITY_REG:
		case PCA9539_CONFIG_REG:
			ret = i2cbus_drv_write_word_data(privdata->i2cbus, &val,
											 reg, 1);
		break;
	}
	if (reg == PCA9539_OUTPUT_REG && ret > 0) {
		privdata->output = val;
	}
	os_mutex_put(&privdata->rw_mutex);

	return (ret > 0)? WM_SUCCESS : -WM_FAIL;
}

static int pca9539_updateReg(mdev_t *dev, uint8_t reg, uint16_t val,
							 uint16_t mask)
{
	int ret = -WM_FAIL;
	uint16_t data = 0, *p;
	pca9539_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	switch (reg) {
		case PCA9539_OUTPUT_REG:
		p = &privdata->output;
		data = (privdata->output & ~mask) | (val & mask);
		break;
		case PCA9539_POLARITY_REG:
		p = &privdata->polarity;
		data = (privdata->polarity & ~mask) | (val & mask);
		break;
		case PCA9539_CONFIG_REG:
		p = &privdata->config;
		data = (privdata->config & ~mask) | (val & mask);
		break;
		default:
		goto f_exit;
	}
	data = (*p & ~mask) | (val & mask);
	ret = i2cbus_drv_write_word_data(privdata->i2cbus, &data, reg, 1);
	if (ret > 0) {
		*p = data;
	}
f_exit:
	os_mutex_put(&privdata->rw_mutex);

	return (ret > 0)? WM_SUCCESS : -WM_FAIL;
}

static void pca9539_irq_handler(int pin, void *data)
{
	pca9539_priv_t *privdata = data;

	os_semaphore_put(&privdata->event_sem);
	return;
}

int pca9539_drv_get_pins(mdev_t *dev)
{
	pca9539_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	return privdata->pins;
}

int pca9539_drv_set_cb(mdev_t *dev, int pin, GPIO_Int_Type type,
					   void *data, gpio_irq_cb gpio_cb)
{
	pca9539_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata || pin < 0 || pin >= privdata->pins) {
		return -WM_FAIL;
	}

	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	privdata->irq_type &= ~(PCA9539_INT_BOTH_EDGE << (pin * 2));
	switch (type) {
		case GPIO_INT_RISING_EDGE:
		privdata->irq_type |= (PCA9539_INT_RISING_EDGE) << (pin * 2);
		break;
		case GPIO_INT_FALLING_EDGE:
		privdata->irq_type |= (PCA9539_INT_FALLING_EDGE) << (pin * 2);
		break;
		case GPIO_INT_BOTH_EDGES:
		privdata->irq_type |= (PCA9539_INT_BOTH_EDGE) << (pin * 2);
		break;
		case GPIO_INT_DISABLE:
		privdata->irq_type |= (PCA9539_INT_DISABLE) << (pin * 2);
		break;
	}
	if (gpio_cb) {
		privdata->callbacks[pin].data = data;
		privdata->callbacks[pin].gpio_cb = gpio_cb;
	}
	os_mutex_put(&privdata->rw_mutex);
	return WM_SUCCESS;
}

int pca9539_drv_set_input(mdev_t *dev, int pin)
{
	uint16_t mask;
	pca9539_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata || pin < 0 || pin >= privdata->pins) {
		return -WM_FAIL;
	}
	mask = 1 << pin;
	return pca9539_updateReg(dev, PCA9539_CONFIG_REG, mask, mask);
}

int pca9539_drv_set_output(mdev_t *dev, int pin, int value)
{
	uint16_t data, mask;
	pca9539_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata || pin < 0 || pin >= privdata->pins) {
		return -WM_FAIL;
	}
	data = (value > 0)? 1 << pin : 0;
	mask = 1 << pin;
	pca9539_updateReg(dev, PCA9539_OUTPUT_REG, data, mask);
	return pca9539_updateReg(dev, PCA9539_CONFIG_REG, ~mask, mask);
}

int pca9539_drv_get(mdev_t *dev, int pin)
{
	uint16_t reg;
	int ret;
	pca9539_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata || pin < 0 || pin >= privdata->pins) {
		return -WM_FAIL;
	}

	ret = pca9539_getReg(privdata->dev, PCA9539_INPUT_REG, &reg);
	if (ret < 0) {
		return -WM_FAIL;
	}
	return !!(reg & (1 << pin));
}

int pca9539_drv_set(mdev_t *dev, int pin, int value)
{
	return pca9539_drv_set_output(dev, pin, value);
}

static void pca9539_irq_thread(os_thread_arg_t data)
{
	int i, ret;
	uint16_t status, changed;
	uint32_t edge;
	pca9539_priv_t *privdata = (pca9539_priv_t *)data;

	do {
		os_semaphore_get(&privdata->event_sem, OS_WAIT_FOREVER);
		if (privdata->exit) {
			break;
		}
		ret = pca9539_getReg(privdata->dev, PCA9539_INPUT_REG, &status);
		if (ret < 0) {
			/* can't read io expander status, drop this event */
			continue;
		}
		os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
		status ^= privdata->polarity;
		changed = privdata->status ^ status;

		privdata->status = status;
		os_mutex_put(&privdata->rw_mutex);

		for (i = 0; i < privdata->pins; i++) {
			if (changed & (1 << i)) {
				if (status & (1 << i)) {
					edge = (PCA9539_INT_RISING_EDGE) << (2 * i);
				} else {
					edge = (PCA9539_INT_FALLING_EDGE) << (2 * i);
				}
				if ((edge & privdata->irq_type) &&
					privdata->callbacks[i].gpio_cb) {
					privdata->callbacks[i].gpio_cb(i,
								privdata->callbacks[i].data);
				}
			}
		}
	} while (1);
	os_semaphore_put(&privdata->exit_sem);
	os_thread_self_complete(NULL);
}

mdev_t *pca9539_drv_open(const char *name)
{
	int ret;
	mdev_t *dev = mdev_get_handle(name);
	pca9539_priv_t *privdata;

	if (dev == NULL) {
		return NULL;
	}
	privdata = mdev_to_priv(dev);

	if (!privdata) {
		return NULL;
	}
	ret = os_mutex_get(&privdata->open_mutex, OS_WAIT_FOREVER);
	if (ret == -WM_FAIL) {
		return NULL;
	}
	return dev;
}

int pca9539_drv_close(mdev_t *dev)
{
	pca9539_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	return os_mutex_put(&privdata->open_mutex);
}

static int pca9539_drv_setup(I2C_ID_Type id, uint16_t addr, int gpio)
{
	int ret;
	mdev_t *dev = &mdev_pca9539;
	pca9539_priv_t *privdata = &pca9539_priv_data;

	dev->port_id = 0;
	dev->name = MDEV_PCA9539;
	dev->pNextMdev = NULL;
	dev->private_data = (uint32_t)privdata;

	privdata->dev = dev;
	privdata->irq = gpio;
	privdata->pins = PCA9539_PINS;
	privdata->config = 0xFFFF;
	privdata->polarity = 0;
	privdata->output = 0;
	privdata->status = 0;
	privdata->irq_type = 0;

	ret = os_mutex_create(&privdata->open_mutex, "pca9539_drv",
						  OS_MUTEX_INHERIT);
	if (ret != WM_SUCCESS) {
		return -WM_FAIL;
	}
	ret = os_mutex_create(&privdata->rw_mutex, "pca9539_rw",
						  OS_MUTEX_INHERIT);
	if (ret != WM_SUCCESS) {
		goto fail_sem_create;
	}

	ret = os_semaphore_create(&privdata->event_sem, "pca9539_irq");
	if (ret != WM_SUCCESS) {
		goto fail_sem_create_1;
	}
	os_semaphore_get(&privdata->event_sem, OS_WAIT_FOREVER);

	ret = os_semaphore_create(&privdata->exit_sem, "pca9539_exit");
	if (ret != WM_SUCCESS) {
		goto fail_sem_create_2;
	}
	os_semaphore_get(&privdata->exit_sem, OS_WAIT_FOREVER);

	ret = i2cbus_drv_init(id);
	if (ret != WM_SUCCESS) {
		goto fail_i2cbus_init;
	}

	privdata->i2cbus = i2cbus_drv_open(id, addr, 0);
	if (!privdata->i2cbus) {
		goto fail_i2cbus_init;
	}

	/* initialize pca9539 registers */
	pca9539_setReg(dev, PCA9539_CONFIG_REG, privdata->config);
	pca9539_setReg(dev, PCA9539_POLARITY_REG, privdata->polarity);
	pca9539_getReg(dev, PCA9539_INPUT_REG, &privdata->status);

	ret = os_thread_create(&privdata->irq_thread, "pca9539_irq_thread",
						   pca9539_irq_thread, privdata,
						   &irq_thread_stack, OS_PRIO_1);
	if (ret != WM_SUCCESS) {
		goto fail_thread_create;
	}
	gpio_drv_init();
	privdata->gpiod = gpio_drv_open("MDEV_GPIO");
	gpio_drv_setdir(privdata->gpiod, gpio, GPIO_INPUT);
	ret = gpio_drv_set_cb(privdata->gpiod, gpio, GPIO_INT_FALLING_EDGE,
						  privdata, pca9539_irq_handler);
	if (ret != WM_SUCCESS) {
		goto fail_gpio_init;
	}
	return WM_SUCCESS;

fail_gpio_init:
	gpio_drv_close(privdata->gpiod);
	os_thread_delete(&privdata->irq_thread);
fail_thread_create:
	i2cbus_drv_close(privdata->i2cbus);
	privdata->i2cbus = NULL;
fail_i2cbus_init:
	os_semaphore_delete(&privdata->exit_sem);
fail_sem_create_2:
	os_semaphore_delete(&privdata->event_sem);
fail_sem_create_1:
	os_mutex_delete(&privdata->rw_mutex);
fail_sem_create:
	os_mutex_delete(&privdata->open_mutex);
	return ret;
}

int pca9539_drv_init(I2C_ID_Type id, uint16_t addr, int gpio)
{
	if (mdev_get_handle(MDEV_PCA9539) != NULL) {
		return WM_SUCCESS;
	}

	if (pca9539_drv_setup(id, addr, gpio) != WM_SUCCESS) {
		return -WM_FAIL;
	}

	return mdev_register(&mdev_pca9539);
}

int pca9539_drv_deinit(I2C_ID_Type id)
{
	int i;
	pca9539_priv_t *privdata = &pca9539_priv_data;

	if (!mdev_get_handle(MDEV_PCA9539)) {
		return WM_SUCCESS;
	}

	privdata->exit = 1;
	os_semaphore_get(&privdata->exit_sem, os_msec_to_ticks(1000));

	privdata->irq_type = 0;
	gpio_drv_set_cb(privdata->gpiod, privdata->irq, GPIO_INT_DISABLE,
					NULL, NULL);
	gpio_drv_close(privdata->gpiod);
	for (i = 0; i < privdata->pins; i++) {
		privdata->callbacks[i].data = NULL;
		privdata->callbacks[i].gpio_cb = NULL;
	}
	i2cbus_drv_close(privdata->i2cbus);
	privdata->config = 0;
	privdata->polarity = 0;

	os_thread_delete(&privdata->irq_thread);
	os_semaphore_delete(&privdata->event_sem);
	os_mutex_delete(&privdata->rw_mutex);
	mdev_deregister(MDEV_PCA9539);
	return WM_SUCCESS;
}
