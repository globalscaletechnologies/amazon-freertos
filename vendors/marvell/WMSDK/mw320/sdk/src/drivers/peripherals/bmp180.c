/** @file bmp180.c
 *
 *  @brief This file provides driver for Digital Pressure Sensor(BMP180)
 *
 *  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
 *
 */

#include <wmstdio.h>
#include <stdbool.h>
#include <mdev_i2cbus.h>
#include <bmp180.h>
#include <wm_os.h>
#include <math.h>
#include <lowlevel_drivers.h>

typedef struct _bmp180_calib {
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
} bmp180_calib_t;

typedef struct _bmp180_priv {
	mdev_t *dev;
	mdev_t *i2cbus;
	uint8_t id;
	uint8_t oversampling;
	int b6; /* temporary variable, for pressure calc */
	bmp180_calib_t calib;
	os_mutex_t rw_mutex;
	os_semaphore_t done_sem;
} bmp180_priv_t;

static mdev_t mdev_bmp180;
static bmp180_priv_t bmp180_priv_data;

static inline void *mdev_to_priv(mdev_t *dev)
{
	return (dev)? (void *)(dev->private_data) : NULL;
}

static int bmp180_getCalibData(bmp180_priv_t *privdata)
{
	uint8_t buf[22];
	int ret;

	if (!privdata) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	ret = i2cbus_drv_read_byte_data(privdata->i2cbus, buf,
									BMP180_CALIB_DATA, 22);
	if (ret < 0) {
		ret = -WM_FAIL;
		goto f_exit;
	}

	privdata->calib.AC1 = buf[0] << 8 | buf[1];
	privdata->calib.AC2 = buf[2] << 8 | buf[3];
	privdata->calib.AC3 = buf[4] << 8 | buf[5];
	privdata->calib.AC4 = buf[6] << 8 | buf[7];
	privdata->calib.AC5 = buf[8] << 8 | buf[9];
	privdata->calib.AC6 = buf[10] << 8 | buf[11];
	privdata->calib.B1 = buf[12] << 8 | buf[13];
	privdata->calib.B2 = buf[14] << 8 | buf[15];
	privdata->calib.MB = buf[16] << 8 | buf[17];
	privdata->calib.MC = buf[18] << 8 | buf[19];
	privdata->calib.MD = buf[20] << 8 | buf[21];

f_exit:
	os_mutex_put(&privdata->rw_mutex);
	return ret;
}

static int bmp180_getChipID(bmp180_priv_t *privdata, uint8_t *id)
{
	int ret;

	if (!privdata || !id) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	ret = i2cbus_drv_read_byte_data(privdata->i2cbus, id, BMP180_ID, 1);
	os_mutex_put(&privdata->rw_mutex);

	return (ret < 0)? -WM_FAIL : WM_SUCCESS;
}

int bmp180_setOversamplingSetting(mdev_t *dev, int val)
{
	bmp180_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	if (val < 0 || val > 3) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	privdata->oversampling = val;
	os_mutex_put(&privdata->rw_mutex);
	return WM_SUCCESS;
}

int bmp180_getOversamplingSetting(mdev_t *dev)
{
	bmp180_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	return privdata->oversampling;
}

static int bmp180_getRawTemperature(bmp180_priv_t *privdata,
									uint32_t *temperature)
{
	int ret, timeout;
	uint8_t buf[2], ctrl = 0;

	if (!privdata || !temperature) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	/* start temperature measurement */
	ctrl = BMP180_TEMP_MEASUREMENT;
	if ((ret = i2cbus_drv_write_byte_data(privdata->i2cbus, &ctrl,
										 BMP180_CTRL_MEAS, 1)) < 0) {
		goto f_exit;
	}
	/* wait measurement completed */
	timeout = 1 + os_msec_to_ticks(BMP180_TEMP_CONVERT_TIME);
	os_semaphore_get(&privdata->done_sem, timeout);

	/* read adc value */
	if ((ret = i2cbus_drv_read_byte_data(privdata->i2cbus, buf,
										BMP180_OUT_MSB, 2)) < 0) {
		goto f_exit;
	}
	/* raw temperature value */
	*temperature = (buf[0] << 8) | buf[1];
f_exit:
	os_mutex_put(&privdata->rw_mutex);
	return (ret < 0)? -WM_FAIL : WM_SUCCESS;
}

static int bmp180_getRawPressure(bmp180_priv_t *privdata,
								 uint32_t *pressure)
{
	int ret, timeout;
	uint8_t buf[3], ctrl = 0;

	if (!privdata || !pressure) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->rw_mutex, OS_WAIT_FOREVER);
	/* start pressure measurement */
	ctrl = (privdata->oversampling << BMP180_CTRL_OSS_OFF) |
		   BMP180_PRESSURE_MEASUREMENT;
	if ((ret = i2cbus_drv_write_byte_data(privdata->i2cbus, &ctrl,
										 BMP180_CTRL_MEAS, 1)) < 0) {
		goto f_exit;
	}
	/* wait measurement completed */
	timeout = 1 + os_msec_to_ticks(2 + (3 << privdata->oversampling));
	os_semaphore_get(&privdata->done_sem, timeout);

	/* read adc value */
	if ((ret = i2cbus_drv_read_byte_data(privdata->i2cbus, buf,
										BMP180_OUT_MSB, 3)) < 0) {
		goto f_exit;
	}
	/* raw pressure value */
	*pressure = (buf[0] << 16) | (buf[1] << 8) | buf[2];
	*pressure = *pressure >> (8 - privdata->oversampling);
f_exit:
	os_mutex_put(&privdata->rw_mutex);
	return (ret < 0)? -WM_FAIL : WM_SUCCESS;
}

int bmp180_getTemperature(mdev_t *dev, int *temperature)
{
	bmp180_priv_t *privdata = mdev_to_priv(dev);
	bmp180_calib_t *calib;
	int t, ut = 0, x1, x2, b5;

	if (!privdata) {
		return -WM_FAIL;
	}
	calib = &privdata->calib;
	if (bmp180_getRawTemperature(privdata, (uint32_t*)&ut) < 0) {
		return -WM_FAIL;
	}
	x1 = ((ut - calib->AC6) * calib->AC5) >> 15;
	x2 = (calib->MC << 11) / (x1 + calib->MD);
	b5 = x1 + x2;
	privdata->b6 = b5 - 4000;
	if (temperature) {
		t = (b5 + 8) >> 4;
		*temperature = t; /* units = 0.1 degree celsius */
	}
	return WM_SUCCESS;
}

int bmp180_getPressure(mdev_t *dev, int *pressure)
{
	bmp180_priv_t *privdata = mdev_to_priv(dev);
	bmp180_calib_t *calib;
	int p, up = 0, x1, x2, x3, b3, b6, oss;
	uint32_t b4, b7;

	if (!privdata || !pressure) {
		return -WM_FAIL;
	}
	calib = &privdata->calib;
	if (bmp180_getTemperature(dev, NULL) < 0) {
		return -WM_FAIL;
	}
	b6 = privdata->b6;
	oss = privdata->oversampling;
	if (bmp180_getRawPressure(privdata, (uint32_t *)&up) < 0) {
		return -WM_FAIL;
	}
	/* calculate b3 */
	x1 = ((int)calib->B2 * ((b6 * b6) >> 12)) >> 11;
	x2 = ((int)calib->AC2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = ((((int)calib->AC1 * 4 + x3) << oss) + 2) >> 2;

	/* calculate b4 */
	x1 = ((int)calib->AC3 * b6) >> 13;
	x2 = ((int)calib->B1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (calib->AC4 * (uint32_t)(x3 + 32768)) >> 15;

	b7 = (uint32_t)(up - b3) * (50000 >> oss);
	if (b7 < 0x80000000) {
		p = (b7 << 1) / b4;
	} else {
		p = (b7 / b4) << 1;
	}

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p += (x1 + x2 + 3791) >> 4;

	/* value returned will be pressure in units of Pa. */
	*pressure = p;
	return WM_SUCCESS;
}

mdev_t *bmp180_drv_open(const char *name)
{
	mdev_t *dev = mdev_get_handle(name);

	if (dev == NULL) {
		return NULL;
	}
	return dev;
}

int bmp180_drv_close(mdev_t *dev)
{
	bmp180_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	return WM_SUCCESS;
}

static int bmp180_drv_setup(I2C_ID_Type id, uint16_t addr)
{
	int ret;
	mdev_t *dev = &mdev_bmp180;
	bmp180_priv_t *privdata = &bmp180_priv_data;

	dev->port_id = 0;
	dev->name = MDEV_BMP180;
	dev->pNextMdev = NULL;
	dev->private_data = (uint32_t)privdata;

	privdata->dev = dev;
	privdata->oversampling = 0;

	ret = os_mutex_create(&privdata->rw_mutex, "bmp180_access",
							  OS_MUTEX_INHERIT);
	if (ret != WM_SUCCESS) {
		goto f_c_sem;
	}

	ret = os_semaphore_create(&privdata->done_sem, "bmp180_convert");
	if (ret != WM_SUCCESS) {
		goto f_c_sem_1;
	}
	os_semaphore_get(&privdata->done_sem, OS_WAIT_FOREVER);

	ret = i2cbus_drv_init(id);
	if (ret != WM_SUCCESS) {
		goto f_i2cbus;
	}

	privdata->i2cbus = i2cbus_drv_open(id, addr, 0);
	if (!privdata->i2cbus) {
		ret = -WM_FAIL;
		goto f_i2cbus;
	}
	/* check device id */
	if ((ret = bmp180_getChipID(privdata, &privdata->id)) < 0) {
		goto f_i2cbus;
	}
	if (privdata->id != BMP180_CHIP_ID) {
		/* chip-id not match */
		ret = -WM_FAIL;
		goto f_i2cbus;
	}

	/* get calibration data from eeprom */
	if ((ret = bmp180_getCalibData(privdata)) < 0) {
		goto f_i2cbus;
	}
	return WM_SUCCESS;
f_i2cbus:
	os_semaphore_delete(&privdata->done_sem);
f_c_sem_1:
	os_mutex_delete(&privdata->rw_mutex);
f_c_sem:
	return ret;
}

int bmp180_drv_init(I2C_ID_Type id)
{
	if (mdev_get_handle(MDEV_BMP180) != NULL) {
		return WM_SUCCESS;
	}
	if (bmp180_drv_setup(id, BMP180_I2C_ADDR) != WM_SUCCESS) {
		return -WM_FAIL;
	}
	return mdev_register(&mdev_bmp180);
}

int bmp180_drv_deinit(I2C_ID_Type id)
{
	bmp180_priv_t *privdata = &bmp180_priv_data;

	if (!mdev_get_handle(MDEV_BMP180)) {
		return WM_SUCCESS;
	}

	i2cbus_drv_close(privdata->i2cbus);
	os_semaphore_delete(&privdata->done_sem);
	os_mutex_delete(&privdata->rw_mutex);
	mdev_deregister(MDEV_BMP180);
	return WM_SUCCESS;
};
