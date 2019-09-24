/*
*  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
*/

#ifndef _PCF8574_H_
#define _PCF8574_H_

#include <mdev.h>
#include <mdev_i2c.h>
#include <mdev_i2cbus.h>
#include <mdev_gpio.h>
#include <lowlevel_drivers.h>

#define MDEV_PCF8574 "MDEV_PCF8574"

int pcf8574_drv_init(I2C_ID_Type id, uint16_t addr, int gpio);
int pcf8574_drv_deinit(I2C_ID_Type id);
mdev_t *pcf8574_drv_open(const char *name);
int pcf8574_drv_close(mdev_t *dev);
int pcf8574_drv_set_cb(mdev_t *dev, int pin, GPIO_Int_Type type,
					   void *data, gpio_irq_cb gpio_cb);
int pcf8574_drv_set_input(mdev_t *dev, int pin);
int pcf8574_drv_set_output(mdev_t *dev, int pin, int value);
int pcf8574_drv_set(mdev_t *dev, int pin, int value);
int pcf8574_drv_get(mdev_t *dev, int pin);
int pcf8574_drv_get_pins(mdev_t *dev);

#endif /* _PCF8574_H_ */
