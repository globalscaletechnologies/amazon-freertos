/*
*  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
*/

#ifndef _BMP180_H_
#define _BMP180_H_

#include <stdbool.h>
#include <mdev.h>

#define MDEV_BMP180 "MDEV_BMP180"

#define BMP180_I2C_ADDR		0x77
#define BMP180_CHIP_ID		0x55

#define BMP180_OUT_XLSB		0xF8	/* ADC output - XLSB */
#define BMP180_OUT_LSB		0xF7	/* ADC output - LSB */
#define BMP180_OUT_MSB		0xF6	/* ADC output - MSB */
#define BMP180_CTRL_MEAS	0xF4	/* Measurement & Rate control */
#define BMP180_SOFT_RESET	0xE0	/* Software Reset */
#define BMP180_ID			0xD0	/* CHIP ID */
#define BMP180_CALIB_DATA	0xAA	/* Calibration Data: 0xAA - 0xBF */

#define BMP180_CTRL_OSS_OFF		6	/* oversampling ratio control */
#define BMP180_CTRL_SCO			(0x1 << 5)	/* start of conversion */

#define BMP180_TEMP_MEASUREMENT			0x2E
#define BMP180_PRESSURE_MEASUREMENT		0x34

#define BMP180_TEMP_CONVERT_TIME	5	/* wait > 4.5 ms for get temperature */

int bmp180_drv_init(I2C_ID_Type id);
int bmp180_drv_deinit(I2C_ID_Type id);
mdev_t *bmp180_drv_open(const char *name);
int bmp180_drv_close(mdev_t *dev);
int bmp180_setOversamplingSetting(mdev_t *dev, int val);
int bmp180_getOversamplingSetting(mdev_t *dev);
int bmp180_getTemperature(mdev_t *dev, int *temperature);
int bmp180_getPressure(mdev_t *dev, int *pressure);

#endif /* _BMP180_H_ */
