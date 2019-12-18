/*
 * pcf8574_expander_wrapper.c: gpio wrapper for PCF8574 GPIO expander
 */

#include <wmstdio.h>
#include <wm_os.h>
#include <mdev_gpio.h>
#include <pcf8574.h>
#include <io_expander.h>

static gpio_expander_t pcf8574_expander = {};

static int pcf8574_expander_get_pins(mdev_t *dev)
{
	return pcf8574_drv_get_pins(dev);
}

static int pcf8574_expander_set_cb(mdev_t *dev,
								   int pin,
								   GPIO_Int_Type type,
								   void *data,
								   gpio_irq_cb gpio_cb)
{
	return pcf8574_drv_set_cb(dev, pin, type, data, gpio_cb);
}

static int pcf8574_expander_write(mdev_t *dev, int pin, int value)
{
	return pcf8574_drv_set(dev, pin, value);
}

static int pcf8574_expander_read(mdev_t *dev, int pin, int *value)
{
	int state = 0;

	state = pcf8574_drv_get(dev, pin);
	if (state < 0) {
		return -WM_FAIL;
	}
	*value = state;
	return WM_SUCCESS;
}

static int pcf8574_expander_setdir(mdev_t *dev,
								   int pin,
								   GPIO_Dir_Type dir)
{
	if (dir == GPIO_INPUT) {
		return pcf8574_drv_set_input(dev, pin);
	} else {
		return pcf8574_drv_set_output(dev, pin, 0);
	}
}

static mdev_t* pcf8574_expander_open(void)
{
	int ret, i, value;
	mdev_t *dev;
	uint32_t pinctrl = board_io_expander_pinctrl();

	ret = pcf8574_drv_init(board_peripheral_i2c_id(),
						   board_io_expander_i2c_address(),
						   board_io_expander_irq());
	if (ret < 0) {
		return NULL;
	}
	dev = pcf8574_drv_open(MDEV_PCF8574);
	if (!dev) {
		pcf8574_drv_deinit(board_peripheral_i2c_id());
		return NULL;
	}
	for (i = 0; i < pcf8574_expander_get_pins(dev); i++) {
		if (pinctrl & (1 << (16 + i))) {
			/* GPIO_OUTPUT */
			value = (pinctrl & (1 << i))? GPIO_IO_HIGH : GPIO_IO_LOW;
			pcf8574_drv_set_output(dev, i, value);
		} else {
			/* GPIO_INPUT */
			pcf8574_drv_set_input(dev, i);
		}
	}
	return dev;
}

static int pcf8574_expander_close(mdev_t *dev)
{
	if (!dev) {
		return -WM_FAIL;
	}
	pcf8574_drv_close(dev);
	pcf8574_drv_deinit(board_peripheral_i2c_id());
	return WM_SUCCESS;
}

int add_pcf8574_io_expander(gpio_expander_t **ioexp, int start)
{
	int num = 0;
	mdev_t *dev = NULL;
	gpio_expander_t *expander = &pcf8574_expander;

	wm_printf("[pcf8574]%s: start:%d\r\n", __func__, start);
	dev = pcf8574_expander_open();
	if (dev == NULL) {
		return -WM_FAIL;
	}
	num = pcf8574_expander_get_pins(dev);

	if (num < 0) {
		return pcf8574_expander_close(dev);
	}

	/* fill the io expander structure */
	expander->dev = dev;
	expander->pins = num;
	expander->start = start;
	expander->ops.open = pcf8574_expander_open;
	expander->ops.setdir = pcf8574_expander_setdir;
	expander->ops.read = pcf8574_expander_read;
	expander->ops.write = pcf8574_expander_write;
	expander->ops.set_cb = pcf8574_expander_set_cb;
	expander->ops.get_pins = pcf8574_expander_get_pins;
	expander->ops.close = pcf8574_expander_close;

	*ioexp = expander;
	return WM_SUCCESS;
}
