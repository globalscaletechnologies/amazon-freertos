/** @file si7021.c
 *
 *  @brief This file provides driver for I2C humidity and temperature
 *  sensor (Si7021)
 *
 *  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
 *
 */

#include <wmstdio.h>
#include <stdbool.h>
#include <mdev_i2cbus.h>
#include <si7021.h>
#include <wm_os.h>
#include <math.h>
#include <lowlevel_drivers.h>

typedef struct _si7021_priv {
	mdev_t *dev;
	mdev_t *i2cbus;

	uint8_t id;
	uint8_t rev;
	bool header_on;
	uint64_t last_rh_measure;
	os_mutex_t rw_mutex;
} si7021_priv_t;

static mdev_t mdev_si7021;
static si7021_priv_t si7021_priv_data;

static inline void *mdev_to_priv(mdev_t *dev)
{
	return (dev)? (void *)(dev->private_data) : NULL;
}

static uint8_t crc8(uint8_t *data, uint8_t len)
{
	uint8_t i, crc = 0;

	while (len--) {
		crc ^= *data++;
		for (i = 8; i > 0; --i) {
			if (crc & 0x80)
				crc = (crc << 1) ^ 0x31;
			else
				crc = (crc << 1);
		}
	}
	return crc;
}

int si7021_getHumidityPercent(mdev_t *dev, int *humidity)
{
	int ret, humraw = 0;
	uint8_t buf[3];
	si7021_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata || !humidity) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	if ((ret = i2cbus_drv_read_byte_data(privdata->i2cbus, buf,
										 SI7021_READ_RH, 3)) < 0) {
		goto f_exit;
	}
	privdata->last_rh_measure = os_total_ticks_get_new();

	if (crc8(buf, 2) != buf[2]) {
		ret = -WM_FAIL;
		goto f_exit;
	}
	humraw = buf[0] << 8 | buf[1];
	*humidity = ((125 * humraw) >> 16) - 6;

f_exit:
	os_mutex_put(&privdata->rw_mutex);
	return ret;
}

int si7021_getCelsiusHundredths(mdev_t *dev, int *temperature)
{
	int ret, tempraw = 0;
	bool use_cache = false;
	uint8_t buf[3];
	si7021_priv_t *privdata = mdev_to_priv(dev);
	uint64_t ticks;

	if (!privdata || !temperature) {
		return -WM_FAIL;
	}

	ticks = privdata->last_rh_measure +
			os_msec_to_ticks(SI7021_CACHE_TIMEOUT);
	if (os_total_ticks_get_new() < ticks) {
		use_cache = true;
	}

	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	if (use_cache) {
		if ((ret = i2cbus_drv_read_byte_data(privdata->i2cbus, buf,
									SI7021_READ_TEMP_PRE_RH, 2)) < 0) {
			goto f_exit;
		}
	} else {
		if ((ret = i2cbus_drv_read_byte_data(privdata->i2cbus, buf,
									SI7021_READ_TEMP, 3)) < 0) {
			goto f_exit;
		}
		if (crc8(buf, 2) != buf[2]) {
			ret = -WM_FAIL;
			goto f_exit;
		}
	}
	tempraw = buf[0] << 8 | buf[1];
	/* units = 0.01 degree celsius */
	*temperature = ((17572 * tempraw) >> 16) - 4685;

f_exit:
	os_mutex_put(&privdata->rw_mutex);
	return ret;
}

int si7021_getFahrenheitHundredths(mdev_t *dev)
{
	int celsius = 0, fahrenheit = 0;

	if (si7021_getCelsiusHundredths(dev, &celsius) == WM_SUCCESS) {
		fahrenheit = ((celsius * 9) / 5) + 3200;
		return fahrenheit;
	}
	return -WM_FAIL;
}

int si7021_getHumidityAndTemperature(mdev_t *dev, int *humidity,
									 int *temperature)
{
	int ret, humraw = 0, tempraw = 0;
	uint8_t buf[3];
	si7021_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata || !humidity) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	if ((ret = i2cbus_drv_read_byte_data(privdata->i2cbus, buf,
										 SI7021_READ_RH, 3)) < 0) {
		goto f_exit;
	}

	if (crc8(buf, 2) != buf[2]) {
		ret = -WM_FAIL;
		goto f_exit;
	}
	humraw = buf[0] << 8 | buf[1];
	*humidity = ((125 * humraw) >> 16) - 6;

	if ((ret = i2cbus_drv_read_byte_data(privdata->i2cbus, buf,
								SI7021_READ_TEMP_PRE_RH, 2)) < 0) {
		goto f_exit;
	}
	tempraw = buf[0] << 8 | buf[1];
	/* units = 0.01 degree celsius */
	*temperature = ((17572 * tempraw) >> 16) - 4685;

f_exit:
	os_mutex_put(&privdata->rw_mutex);
	return ret;
}

int si7021_getDeviceID(mdev_t *dev, uint8_t *id)
{
	uint8_t sna[8], snb[6], buf[8], i;
	uint16_t c1, c2;
	int ret;
	si7021_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata || !id) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	/* 1st read */
	c1 = SI7021_ID1_H << 8 | SI7021_ID1_L;
	ret = i2cbus_drv_read_word_cmd_byte_data(privdata->i2cbus, &sna,
											 c1, 8);
	if (ret < 0) {
		goto f_exit;
	}
	buf[0] = sna[0];
	buf[1] = sna[2];
	buf[2] = sna[4];
	buf[3] = sna[6];
	/* crc check */
	for (i = 0; i < 4; i++) {
		if (crc8(buf, i + 1) != sna[i * 2 + 1]) {
			ret = -WM_FAIL;
			goto f_exit;
		}
	}

	/* 2nd read */
	c2 = SI7021_ID2_H << 8 | SI7021_ID2_L;
	ret = i2cbus_drv_read_word_cmd_byte_data(privdata->i2cbus, &snb,
											 c2, 6);
	if (ret < 0) {
		goto f_exit;
	}
	buf[4] = snb[0];
	buf[5] = snb[1];
	buf[6] = snb[3];
	buf[7] = snb[4];
	/* crc check */
	for (i = 0; i < 2; i++) {
		if (crc8(&buf[4], (i * 2) + 2) != snb[i * 3 + 2]) {
			ret = -WM_FAIL;
			goto f_exit;
		}
	}
	*id = buf[4];

f_exit:
	os_mutex_put(&privdata->rw_mutex);
	return ret;
}

int si7021_getFirmwareRevision(mdev_t *dev, uint8_t *rev)
{
	uint16_t cmd;
	int ret;
	si7021_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata || !rev) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	cmd = SI7021_FIRMWARE_REV_H << 8 | SI7021_FIRMWARE_REV_L;
	ret = i2cbus_drv_read_word_cmd_byte_data(privdata->i2cbus, rev,
											 cmd, 1);
	os_mutex_put(&privdata->rw_mutex);
	return ret;
}

int si7021_setHeater(mdev_t *dev, bool on)
{
	int ret;
	uint8_t user1 = 0;
	si7021_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	ret = i2cbus_drv_read_byte_data(privdata->i2cbus, &user1,
									SI7021_READ_USER1, 1);
	if (ret < 0) {
		goto f_exit;
	}
	if (on)
		user1 |= SI7021_USER1_HTRE_BIT;
	else
		user1 &= SI7021_USER1_HTRE_BIT;

	ret = i2cbus_drv_write_byte_data(privdata->i2cbus, &user1,
									 SI7021_WRITE_USER1, 1);
f_exit:
	os_mutex_put(&privdata->rw_mutex);
	return ret;
}

int si7021_setMeasurementResolution(mdev_t *dev, int mode)
{
	int ret;
	uint8_t user1 = 0;
	si7021_priv_t *privdata = mdev_to_priv(dev);

	/* Measurement Resolution:
	 *        RH      Temp
	 * 00:    12 bit  14 bit  (default)
	 * 01:    8 bit   12 bit
	 * 10:    10 bit  13 bit
	 * 11:    11 bit  11 bit
	 */

	if (!privdata || mode < 0 || mode > 3) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	ret = i2cbus_drv_read_byte_data(privdata->i2cbus, &user1,
									SI7021_READ_USER1, 1);
	if (ret < 0) {
		goto f_exit;
	}
	user1 &= ~SI7021_USER1_MASK;
	switch (mode) {
		case 0:
			user1 |= SI7021_USER1_RES_MODE0;
		break;
		case 1:
			user1 |= SI7021_USER1_RES_MODE1;
		break;
		case 2:
			user1 |= SI7021_USER1_RES_MODE2;
		break;
		case 3:
			user1 |= SI7021_USER1_RES_MODE3;
		break;
	}

	ret = i2cbus_drv_write_byte_data(privdata->i2cbus, &user1,
									 SI7021_WRITE_USER1, 1);
f_exit:
	os_mutex_put(&privdata->rw_mutex);
	return ret;
}

int si7021_reset(mdev_t *dev)
{
	int ret;
	uint8_t cmd = SI7021_RESET;
	si7021_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	ret = i2cbus_drv_write_bytes(privdata->i2cbus, &cmd, 1);
	if (ret < 0) {
		goto f_exit;
	}
	_os_delay(20);
f_exit:
	os_mutex_put(&privdata->rw_mutex);
	return ret;
}

mdev_t *si7021_drv_open(const char *name)
{
	mdev_t *dev = mdev_get_handle(name);

	if (dev == NULL) {
		return NULL;
	}
	return dev;
}

int si7021_drv_close(mdev_t *dev)
{
	si7021_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	return WM_SUCCESS;
}

static int si7021_drv_setup(I2C_ID_Type id, uint16_t addr)
{
	int ret;
	mdev_t *dev = &mdev_si7021;
	si7021_priv_t *privdata = &si7021_priv_data;

	dev->port_id = 0;
	dev->name = MDEV_SI7021;
	dev->pNextMdev = NULL;
	dev->private_data = (uint32_t)privdata;

	privdata->dev = dev;
	privdata->header_on = false;
	privdata->last_rh_measure = 0;

	ret = os_mutex_create(&privdata->rw_mutex, "si7021_RegAccess",
							  OS_MUTEX_INHERIT);
	if (ret != WM_SUCCESS) {
		goto err_setup;
	}

	ret = i2cbus_drv_init(id);
	if (ret != WM_SUCCESS) {
		goto err_setup;
	}

	privdata->i2cbus = i2cbus_drv_open(id, addr, 0);
	if (!privdata->i2cbus) {
		goto err_setup;
	}

	/* soft reset */
	si7021_reset(dev);

	/* get device id */
	if (si7021_getDeviceID(dev, &privdata->id) < 0) {
		goto err_setup;
	}

	/* check device id */
	if (privdata->id != SNB3_ID_SI7021) {
		goto err_setup;
	}
	/* get firmware revision */
	if (si7021_getFirmwareRevision(dev, &privdata->rev) < 0) {
		goto err_setup;
	}
	return WM_SUCCESS;

err_setup:
	if (!privdata->i2cbus) {
		i2cbus_drv_close(privdata->i2cbus);
		privdata->i2cbus = NULL;
	}
	if (privdata->rw_mutex) {
		os_mutex_delete(&privdata->rw_mutex);
		privdata->rw_mutex = NULL;
	}
	return -WM_FAIL;
}

int si7021_drv_init(I2C_ID_Type id)
{
	if (mdev_get_handle(MDEV_SI7021) != NULL) {
		return WM_SUCCESS;
	}
	if (si7021_drv_setup(id, SI7021_I2C_ADDR) != WM_SUCCESS) {
		return -WM_FAIL;
	}
	return mdev_register(&mdev_si7021);
}

int si7021_drv_deinit(I2C_ID_Type id)
{
	si7021_priv_t *privdata = &si7021_priv_data;

	if (!mdev_get_handle(MDEV_SI7021)) {
		return WM_SUCCESS;
	}

	if (!privdata->i2cbus) {
		i2cbus_drv_close(privdata->i2cbus);
		privdata->i2cbus = NULL;
	}
	if (privdata->rw_mutex) {
		os_mutex_delete(&privdata->rw_mutex);
		privdata->rw_mutex = NULL;
	}
	mdev_deregister(MDEV_SI7021);
	return WM_SUCCESS;
}
