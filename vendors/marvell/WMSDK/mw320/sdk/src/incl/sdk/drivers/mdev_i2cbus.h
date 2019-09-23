/*
*  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
*/

#ifndef _MDEV_I2CBUS_H_
#define _MDEV_I2CBUS_H_

#include <mdev.h>
#include <mdev_i2c.h>
#include <lowlevel_drivers.h>
int i2cbus_drv_init(I2C_ID_Type id);
mdev_t* i2cbus_drv_open(I2C_ID_Type id, uint16_t addr, uint32_t flags);
int i2cbus_drv_close(mdev_t *dev);

int i2cbus_drv_read_bytes(mdev_t *dev, void *buf, uint32_t nbytes);
int i2cbus_drv_write_bytes(mdev_t *dev, void *buf, uint32_t nbytes);
int i2cbus_drv_read_byte_data(mdev_t *dev, void *buf, uint8_t cmd,
							  uint32_t nbytes);
int i2cbus_drv_write_byte_data(mdev_t *dev, void *buf, uint8_t cmd,
							   uint32_t nbytes);
int i2cbus_drv_read_word_data(mdev_t *dev, void *buf, uint8_t cmd,
							  uint32_t nbytes);
int i2cbus_drv_write_word_data(mdev_t *dev, void *buf, uint8_t cmd,
							   uint32_t nbytes);
int i2cbus_drv_read_word_cmd_data(mdev_t *dev, void *buf, uint16_t cmd,
								  uint32_t nbytes);
int i2cbus_drv_write_word_cmd_data(mdev_t *dev, void *buf, uint16_t cmd,
								   uint32_t nbytes);
int i2cbus_drv_read_word_cmd_byte_data(mdev_t *dev, void *buf,
									   uint16_t cmd, uint32_t nbytes);
int i2cbus_drv_write_word_cmd_byte_data(mdev_t *dev, void *buf,
										uint16_t cmd, uint32_t nbytes);
#endif /* _MDEV_I2CBUS_H_ */
