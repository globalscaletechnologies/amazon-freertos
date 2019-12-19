#ifndef _IO_EXPANDER_H_
#define _IO_EXPANDER_H_

#include <mdev.h>
#include <wmlog.h>
#include <wmerrno.h>
#include <lowlevel_drivers.h>
#include <mdev_gpio.h>

#define GPIO_IOP_N(n)		((GPIO_MaxNo + 1) + n)

typedef mdev_t* (*gpio_expander_open_t) (void);
typedef int (*gpio_expander_setdir_t) (mdev_t *dev,
									   int pin,
									   GPIO_Dir_Type dir);
typedef int (*gpio_expander_read_t) (mdev_t *dev, int pin, int *value);
typedef int (*gpio_expander_write_t) (mdev_t *dev, int pin, int value);
typedef int (*gpio_expander_set_cb_t) (mdev_t *dev, int pin,
									   GPIO_Int_Type type,
									   void *data,
									   gpio_irq_cb gpio_cb);
typedef int (*gpio_expander_get_pins_t) (mdev_t *dev);
typedef int (*gpio_expander_close_t) (mdev_t *dev);

typedef struct _gpio_expander_ops {
	gpio_expander_open_t open;
	gpio_expander_setdir_t setdir;
	gpio_expander_read_t read;
	gpio_expander_write_t write;
	gpio_expander_set_cb_t set_cb;
	gpio_expander_get_pins_t get_pins;
	gpio_expander_close_t close;
} gpio_expander_ops_t;

typedef struct _gpio_expander {
	mdev_t *dev;
	int pins;
	int start;
	gpio_expander_ops_t ops;
} gpio_expander_t;

int add_io_expander(gpio_expander_t **expander, int start);

#endif /* _IO_EXPANDER_H_ */
