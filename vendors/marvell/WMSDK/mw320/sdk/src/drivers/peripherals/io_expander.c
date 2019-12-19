/*
 * io_expander.c: gpio wrapper for GPIO expander
 */

#include <wmstdio.h>
#include <wm_os.h>
#include <mdev_gpio.h>
#include <io_expander.h>
#include <pcf8574_expander_wrapper.h>
#include <pca9539_expander_wrapper.h>

int add_io_expander(gpio_expander_t **ioexp, int start)
{
	int result = WM_SUCCESS;
	int id = board_io_expander_id();

	switch (id) {
		case PCF8574_IDNUM:
		result = add_pcf8574_io_expander(ioexp, start);
		break;
		case PCA9539_IDNUM:
		result = add_pca9539_io_expander(ioexp, start);
		break;
		default:
		result = -WM_FAIL;
		break;
	}
	return result;
}
