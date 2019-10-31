/*
*  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
*/

#ifndef _PCA9539_H_
#define _PCA9539_H_

#include <mdev.h>
#include <mdev_i2c.h>
#include <mdev_i2cbus.h>
#include <mdev_gpio.h>
#include <lowlevel_drivers.h>

#define MDEV_PCA9539 "MDEV_PCA9539"

int pca9539_drv_init(I2C_ID_Type id, uint16_t addr, int gpio);
int pca9539_drv_deinit(I2C_ID_Type id);
mdev_t *pca9539_drv_open(const char *name);
int pca9539_drv_close(mdev_t *dev);
int pca9539_drv_set_cb(mdev_t *dev, int pin, GPIO_Int_Type type,
					   void *data, gpio_irq_cb gpio_cb);
int pca9539_drv_set_input(mdev_t *dev, int pin);
int pca9539_drv_set_output(mdev_t *dev, int pin, int value);
int pca9539_drv_set(mdev_t *dev, int pin, int value);
int pca9539_drv_get(mdev_t *dev, int pin);
int pca9539_drv_get_pins(mdev_t *dev);

#endif /* _PCA9539_H_ */
