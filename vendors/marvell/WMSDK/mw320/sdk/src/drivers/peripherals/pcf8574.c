/*
 * mdev_pcf8574.c: mdev driver for GPIO expander (pcf8574)
 */
#include <wmstdio.h>
#include <mdev_gpio.h>
#include <mdev_i2cbus.h>
#include <wm_os.h>
#include <lowlevel_drivers.h>
#include <pcf8574.h>

#define PCF8574_PINS					8

#define PCF8574_INT_DISABLE				(0x0)
#define PCF8574_INT_FALLING_EDGE		(0x1)
#define PCF8574_INT_RISING_EDGE			(0x2)
#define PCF8574_INT_BOTH_EDGE			(0x3)

typedef struct _pcf8574_gpio_irq{
	void *data;
	gpio_irq_cb gpio_cb;
} pcf8574_gpio_irq_t;

typedef struct _mdev_pcf8574_priv_data {
	mdev_t *dev;
	mdev_t *i2cbus;
	mdev_t *gpiod;

	int irq;
    int pins;
	uint8_t output;
	uint8_t status;
	uint16_t irq_type;
	uint8_t exit;
	pcf8574_gpio_irq_t irq_cb_list[PCF8574_PINS];

	os_mutex_t rw_mutex;
	os_semaphore_t event_sem;
	os_semaphore_t exit_sem;
	os_thread_t irq_thread;
} mdev_pcf8574_priv_data_t;

static os_thread_stack_define(irq_thread_stack, 1024);

static mdev_t mdev_pcf8574;
static mdev_pcf8574_priv_data_t pcf8574_priv_dara;

static inline void *mdev_to_priv(mdev_t *dev)
{
	return (dev)? (void *)(dev->private_data) : NULL;
}

static void pcf8574_irq_handler(int pin, void *data)
{
	mdev_pcf8574_priv_data_t *privdata = data;

	os_semaphore_put(&privdata->event_sem);
	return;
}

int pcf8574_drv_get_pins(mdev_t *dev)
{
	mdev_pcf8574_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	return privdata->pins;
}

int pcf8574_drv_set_cb(mdev_t *dev, int pin, GPIO_Int_Type type,
					   void *data, gpio_irq_cb gpio_cb)
{
	mdev_pcf8574_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	if (pin < 0 || pin >= privdata->pins) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	privdata->irq_type &= ~(PCF8574_INT_BOTH_EDGE << (pin * 2));
	switch (type) {
		case GPIO_INT_RISING_EDGE:
		privdata->irq_type |= (PCF8574_INT_RISING_EDGE) << (pin * 2);
		break;
		case GPIO_INT_FALLING_EDGE:
		privdata->irq_type |= (PCF8574_INT_FALLING_EDGE) << (pin * 2);
		break;
		case GPIO_INT_BOTH_EDGES:
		privdata->irq_type |= (PCF8574_INT_BOTH_EDGE) << (pin * 2);
		break;
		case GPIO_INT_DISABLE:
		privdata->irq_type |= (PCF8574_INT_DISABLE) << (pin * 2);
		break;
	}
	if (gpio_cb) {
		privdata->irq_cb_list[pin].data = data;
		privdata->irq_cb_list[pin].gpio_cb = gpio_cb;
	}
	os_mutex_put(&privdata->rw_mutex);
	return WM_SUCCESS;
}

int pcf8574_drv_set_input(mdev_t *dev, int pin)
{
	uint8_t reg;
	int ret;
	mdev_pcf8574_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	if (pin < 0 || pin >= privdata->pins) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);

	reg = privdata->output;
	reg |= 1 << pin;
	ret = i2cbus_drv_write_bytes(privdata->i2cbus, &reg, 1);
	if (ret > 0) {
		privdata->output = reg;
		ret = WM_SUCCESS;
	}
	os_mutex_put(&privdata->rw_mutex);
	return ret;
}

int pcf8574_drv_set_output(mdev_t *dev, int pin, int value)
{
	uint8_t reg;
	int ret;
	mdev_pcf8574_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	if (pin < 0 || pin >= privdata->pins) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);

	reg = privdata->output;
	if (value)
		reg |= 1 << pin;
	else
		reg &= ~(1 << pin);
	ret = i2cbus_drv_write_bytes(privdata->i2cbus, &reg, 1);
	if (ret > 0) {
		privdata->output = reg;
		ret = WM_SUCCESS;
	}
	os_mutex_put(&privdata->rw_mutex);
	return ret;
}

int pcf8574_drv_get(mdev_t *dev, int pin)
{
	uint8_t reg;
	int ret;
	mdev_pcf8574_priv_data_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}

	if (pin < 0 || pin >= privdata->pins) {
		return -WM_FAIL;
	}

	ret = i2cbus_drv_read_bytes(privdata->i2cbus, &reg, 1);
	if (ret <= 0) {
		ret = -WM_FAIL;
	}
	return !!(reg & (1 << pin));
}

int pcf8574_drv_set(mdev_t *dev, int pin, int value)
{
	return pcf8574_drv_set_output(dev, pin, value);
}

static void pcf8574_irq_thread(os_thread_arg_t data)
{
	int i;
	uint8_t status, change;
	uint16_t edge;
	mdev_pcf8574_priv_data_t *privdata = (mdev_pcf8574_priv_data_t *)data;

	do {
		os_semaphore_get(&privdata->event_sem, OS_WAIT_FOREVER);
		if (privdata->exit) {
			break;
		}
		status = 0;
		if (i2cbus_drv_read_bytes(privdata->i2cbus, &status, 1) < 0) {
			/* can't read io expander status */
			break;
		}

		os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
		change = (privdata->status ^ status);
		privdata->status = status;
		os_mutex_put(&privdata->rw_mutex);

		for (i = 0; i < privdata->pins; i++) {
			if (change & (1 << i)) {
				if (status & (1 << i)) {
					edge = (PCF8574_INT_RISING_EDGE) << (2 * i);
				} else {
					edge = (PCF8574_INT_FALLING_EDGE) << (2 * i);
				}

				if ((edge & privdata->irq_type) &&
					privdata->irq_cb_list[i].gpio_cb) {
					privdata->irq_cb_list[i].gpio_cb(i,
								privdata->irq_cb_list[i].data);
				}
			}
		}
	} while (1);
	os_semaphore_put(&privdata->exit_sem);
	os_thread_self_complete(NULL);
}

mdev_t *pcf8574_drv_open(const char *name)
{
	mdev_t *dev = mdev_get_handle(name);

	if (dev == NULL) {
		return NULL;
	}
	return dev;
}

int pcf8574_drv_close(mdev_t *dev)
{
	return 0;
}

static int pcf8574_drv_mdev_init(I2C_ID_Type id, uint16_t addr, int gpio)
{
	int ret;
	mdev_t *dev = &mdev_pcf8574;
	mdev_pcf8574_priv_data_t *privdata = &pcf8574_priv_dara;

	dev->port_id = 0;
	dev->name = MDEV_PCF8574;
	dev->pNextMdev = NULL;
	dev->private_data = (uint32_t)privdata;

	privdata->irq = gpio;
	privdata->output = 0;
	privdata->status = 0;
	privdata->irq_type = 0;
    privdata->pins = PCF8574_PINS;

	ret = os_mutex_create(&privdata->rw_mutex, "pcf8574_rw",
						  OS_MUTEX_INHERIT);
	if (ret != WM_SUCCESS) {
		return -WM_FAIL;
	}

	ret = os_semaphore_create(&privdata->event_sem, "pcf8574_irq");
	if (ret != WM_SUCCESS) {
		goto fail_sem_create;
	}
	os_semaphore_get(&privdata->event_sem, OS_WAIT_FOREVER);

	ret = os_semaphore_create(&privdata->exit_sem, "pcf8574_exit");
	if (ret != WM_SUCCESS) {
		goto fail_sem_create_1;
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
	/* read initial pin state */
	i2cbus_drv_read_bytes(privdata->i2cbus, &privdata->status, 1);

	ret = os_thread_create(&privdata->irq_thread, "pcf85774_irq_thread",
						   pcf8574_irq_thread, privdata,
						   &irq_thread_stack, OS_PRIO_1);
	if (ret != WM_SUCCESS) {
		goto fail_thread_create;
	}
	gpio_drv_init();
	privdata->gpiod = gpio_drv_open("MDEV_GPIO");
	gpio_drv_setdir(privdata->gpiod, gpio, GPIO_INPUT);
	ret = gpio_drv_set_cb(privdata->gpiod, gpio, GPIO_INT_FALLING_EDGE,
						  privdata, pcf8574_irq_handler);
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
fail_sem_create_1:
	os_semaphore_delete(&privdata->event_sem);
fail_sem_create:
	os_mutex_delete(&privdata->rw_mutex);
	return ret;
}

int pcf8574_drv_init(I2C_ID_Type id, uint16_t addr, int gpio)
{
	if (mdev_get_handle(MDEV_PCF8574) != NULL) {
		return WM_SUCCESS;
	}
	if (pcf8574_drv_mdev_init(id, addr, gpio) != WM_SUCCESS) {
		return -WM_FAIL;
	}
	return mdev_register(&mdev_pcf8574);
}

int pcf8574_drv_deinit(I2C_ID_Type id)
{
	int i;
	mdev_pcf8574_priv_data_t *privdata = &pcf8574_priv_dara;

	if (!mdev_get_handle(MDEV_PCF8574)) {
		return WM_SUCCESS;
	}

	privdata->exit = 1;
	os_semaphore_get(&privdata->exit_sem, os_msec_to_ticks(1000));

	privdata->irq_type = 0;
	gpio_drv_set_cb(privdata->gpiod, privdata->irq, GPIO_INT_DISABLE,
					NULL, NULL);
	gpio_drv_close(privdata->gpiod);
	for (i = 0; i < privdata->pins; i++) {
		privdata->irq_cb_list[i].data = NULL;
		privdata->irq_cb_list[i].gpio_cb = NULL;
	}
	i2cbus_drv_close(privdata->i2cbus);
	privdata->status = 0;
	privdata->output = 0;

	os_thread_delete(&privdata->irq_thread);
	os_semaphore_delete(&privdata->event_sem);
	os_mutex_delete(&privdata->rw_mutex);
	mdev_deregister(MDEV_PCF8574);
	return WM_SUCCESS;
}
