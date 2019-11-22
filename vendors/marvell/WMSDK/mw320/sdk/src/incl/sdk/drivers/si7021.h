/*
*  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
*/

#ifndef _SI7021_H_
#define _SI7021_H_

#include <stdbool.h>
#include <mdev.h>

#define MDEV_SI7021 "MDEV_SI7021"

#define SI7021_I2C_ADDR			0x40
#define SI7021_CACHE_TIMEOUT	500		/* 500ms */

#define SI7021_READ_RH			0xE5	/* Measure Relative Humidity */
#define SI7021_READ_TEMP		0xE3	/* Measure Temperature */
#define SI7021_READ_TEMP_PRE_RH	0xE0	/* Read Temperature Value from
										 * Previous RH Measurement */
#define SI7021_RESET			0xFE	/* Reset */
#define SI7021_WRITE_USER1		0xE6	/* Write RH/T User Register 1 */
#define SI7021_READ_USER1		0xE7	/* Read RH/T User Register 1 */
#define SI7021_ID1_H			0xFA	/* Read ID 1st Byte #0 */
#define SI7021_ID1_L			0x0F	/* Read ID 1st Byte #1 */
#define SI7021_ID2_H			0xFC	/* Read ID 2nd Byte #0 */
#define SI7021_ID2_L			0xC9	/* Read ID 2nd Byte #1 */
#define SI7021_FIRMWARE_REV_H	0x84	/* Read Firmware Revision #0 */
#define SI7021_FIRMWARE_REV_L	0xB8	/* Read Firmware Revision #1 */

#define SNB3_ID_SI7021			0x15

/* User 1 Register bits */
#define SI7021_USER1_RES1_BIT	(1 << 7) /* Measurement Resolution */
#define SI7021_USER1_RES0_BIT	(1 << 0)
#define SI7021_USER1_RES_MODE0	0x00
#define SI7021_USER1_RES_MODE1	0x01
#define SI7021_USER1_RES_MODE2	0x80
#define SI7021_USER1_RES_MODE3	0x81
#define SI7021_USER1_MASK		(SI7021_USER1_RES1_BIT | \
								 SI7021_USER1_RES0_BIT)
#define SI7021_USER1_VDDS_BIT	(1 << 6) /* VDD Status */
#define SI7021_USER1_HTRE_BIT	(1 << 2) /* on-chip heater on/off */

int si7021_getHumidityPercent(mdev_t *dev, int *humidity);
int si7021_getCelsiusHundredths(mdev_t *dev, int *temperature);
int si7021_getFahrenheitHundredths(mdev_t *dev);
int si7021_getHumidityAndTemperature(mdev_t *dev, int *humidity,
									 int *temperature);
int si7021_getDeviceID(mdev_t *dev, uint8_t *id);
int si7021_getFirmwareRevision(mdev_t *dev, uint8_t *rev);
int si7021_setHeater(mdev_t *dev, bool on);
int si7021_setMeasurementResolution(mdev_t *dev, int mode);
int si7021_reset(mdev_t *dev);
int si7021_drv_init(I2C_ID_Type id);
int si7021_drv_deinit(I2C_ID_Type id);
mdev_t *si7021_drv_open(const char *name);
int si7021_drv_close(mdev_t *dev);

#endif /* _SI7021_H_ */
