/** @file mdev_i2cbus.c
 *
 *  @brief This file provides mdev driver for IO expander
 *
 *  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
 *
 */


/*
 * mdev_i2cbus.c: mdev driver for i2c bus.
 */

#include <wmstdio.h>
#include <mdev_i2c.h>
#include <mdev_i2cbus.h>
#include <mdev_gpio.h>
#include <wm_os.h>
#include <lowlevel_drivers.h>

#define NUM_MDEV_I2C_PORTS		2

static const char *mdev_i2cbus_name[NUM_MDEV_I2C_PORTS] = {
	"MDEV_I2CBUS0",
	"MDEV_I2CBUS1",
};

static const char *rw_mutex_name[NUM_MDEV_I2C_PORTS] = {
	"i2cbus0-rw-mutex",
	"i2cbus1-rw-mutex",
};

typedef struct _mdev_i2cbus_priv_data {
	mdev_t *dev;
	int refcnt;
	os_mutex_t i2cbus_mutex;
	os_mutex_t i2cbus_rw_mutex;
} mdev_i2cbus_priv_data_t;

/* The device objects */
static mdev_t mdev_i2cbus[NUM_MDEV_I2C_PORTS];
static mdev_i2cbus_priv_data_t i2cbus_priv_dara[NUM_MDEV_I2C_PORTS];

static int i2cbus_drv_read_cmd_data_n(mdev_t *dev, void *buf, void *cmd,
							   uint8_t w_cmd, uint8_t w_data,
							   uint32_t length)
{
	mdev_t *i2c_dev;
	uint8_t cmdbuf[2], *p = cmd;
	mdev_i2cbus_priv_data_t *privdata;
	int ret, len;

	if (!dev || !buf || !length) {
		return -WM_FAIL;
	}
	privdata = (mdev_i2cbus_priv_data_t *)dev->private_data;
	if (!privdata) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->i2cbus_rw_mutex, OS_WAIT_FOREVER);

	i2c_dev = i2c_drv_open(dev->port_id, dev->dev_id);
	if (!i2c_dev) {
		os_mutex_put(&privdata->i2cbus_rw_mutex);
		return -WM_FAIL;
	}

	if (cmd) {
		if (w_cmd == 1) {
			cmdbuf[0] = p[0] & 0xFF;
			ret = i2c_drv_write(i2c_dev, cmdbuf, 1);
		} else if (w_cmd == 2) {
			cmdbuf[0] = p[1];
			cmdbuf[1] = p[0];
			ret = i2c_drv_write(i2c_dev, cmdbuf, 2);
		} else {
			ret = -WM_FAIL;
		}
		if (ret < 0) {
			goto end_r_d_n;
		}
	}

	len = length * w_data;
	if (w_data == 1 || w_data == 2) {
		ret = i2c_drv_read(i2c_dev, buf, len);
	} else {
		ret = -WM_FAIL;
	}
	if (ret != len) {
		goto end_r_d_n;
	}
end_r_d_n:
	if (i2c_drv_close(i2c_dev) != WM_SUCCESS) {
		ret = -WM_FAIL;
	}
	os_mutex_put(&privdata->i2cbus_rw_mutex);
	return ret;
}

static int i2cbus_drv_write_cmd_data_n(mdev_t *dev, void *buf, void *cmd,
								uint8_t w_cmd, uint8_t w_data,
								uint32_t length)
{
	mdev_t *i2c_dev;
	uint8_t *data, *p = cmd;
	int ret, idx = 0, len;
	mdev_i2cbus_priv_data_t *privdata;

	if (!dev || !buf || !length) {
		return -WM_FAIL;
	}
	privdata = (mdev_i2cbus_priv_data_t *)dev->private_data;
	if (!privdata) {
		return -WM_FAIL;
	}
	os_mutex_get(&privdata->i2cbus_rw_mutex, OS_WAIT_FOREVER);

	i2c_dev = i2c_drv_open(dev->port_id, dev->dev_id);
	if (!i2c_dev) {
		os_mutex_put(&privdata->i2cbus_rw_mutex);
		return -WM_FAIL;
	}

	len = (length * w_data) + ((cmd)? w_cmd : 0);
	data = os_mem_calloc(len);
	if (!data) {
		ret = -WM_FAIL;
		goto alloc_fail;
	}
	if (cmd) {
		if (w_cmd == 1) {
			data[idx++] = p[0];
		} else if (w_cmd == 2) {
			data[idx++] = p[1];
			data[idx++] = p[0];
		} else {
			ret = -WM_FAIL;
			goto end_w_d_n;
		}
	}
	memcpy(&data[idx], buf, length * w_data);
	ret = i2c_drv_write(i2c_dev, data, len);
	if (ret != len) {
		goto end_w_d_n;
	}
end_w_d_n:
	os_mem_free(data);
alloc_fail:
	if (i2c_drv_close(i2c_dev) != WM_SUCCESS) {
		ret = -WM_FAIL;
	}
	os_mutex_put(&privdata->i2cbus_rw_mutex);
	return ret;
}

int i2cbus_drv_read_bytes(mdev_t *dev, void *buf, uint32_t nbytes)
{
	return i2cbus_drv_read_cmd_data_n(dev, buf, NULL, 0, 1, nbytes);
}

int i2cbus_drv_write_bytes(mdev_t *dev, void *buf, uint32_t nbytes)
{
	return i2cbus_drv_write_cmd_data_n(dev, buf, NULL, 0, 1, nbytes);
}

int i2cbus_drv_read_byte_data(mdev_t *dev, void *buf, uint8_t cmd,
							  uint32_t nbytes)
{
	return i2cbus_drv_read_cmd_data_n(dev, buf, &cmd, 1, 1, nbytes);
}

int i2cbus_drv_write_byte_data(mdev_t *dev, void *buf, uint8_t cmd,
							   uint32_t nbytes)
{
	return i2cbus_drv_write_cmd_data_n(dev, buf, &cmd, 1, 1, nbytes);
}

int i2cbus_drv_read_word_data(mdev_t *dev, void *buf, uint8_t cmd,
							  uint32_t nbytes)
{
	return i2cbus_drv_read_cmd_data_n(dev, buf, &cmd, 1, 2, nbytes);
}

int i2cbus_drv_write_word_data(mdev_t *dev, void *buf, uint8_t cmd,
							   uint32_t nbytes)
{
	return i2cbus_drv_write_cmd_data_n(dev, buf, &cmd, 1, 2, nbytes);
}

int i2cbus_drv_read_word_cmd_data(mdev_t *dev, void *buf, uint16_t cmd,
								  uint32_t nbytes)
{
	return i2cbus_drv_read_cmd_data_n(dev, buf, &cmd, 2, 2, nbytes);
}

int i2cbus_drv_write_word_cmd_data(mdev_t *dev, void *buf, uint16_t cmd,
								   uint32_t nbytes)
{
	return i2cbus_drv_write_cmd_data_n(dev, buf, &cmd, 2, 2, nbytes);
}

int i2cbus_drv_read_word_cmd_byte_data(mdev_t *dev, void *buf,
									   uint16_t cmd, uint32_t nbytes)
{
	return i2cbus_drv_read_cmd_data_n(dev, buf, &cmd, 2, 1, nbytes);
}

int i2cbus_drv_write_word_cmd_byte_data(mdev_t *dev, void *buf,
										uint16_t cmd, uint32_t nbytes)
{
	return i2cbus_drv_write_cmd_data_n(dev, buf, &cmd, 2, 1, nbytes);
}

mdev_t* i2cbus_drv_open(I2C_ID_Type id, uint16_t addr, uint32_t flags)
{
	mdev_t *i2cbus_dev = NULL;
	mdev_t *i2ccli_dev = NULL;
	mdev_i2cbus_priv_data_t *privdata = NULL;
	char name[14];

	i2cbus_dev = mdev_get_handle(mdev_i2cbus_name[id]);
	if (i2cbus_dev == NULL) {
		return NULL;
	}
	privdata = (mdev_i2cbus_priv_data_t *)i2cbus_dev->private_data;
	if (privdata == NULL) {
		return NULL;
	}
	os_mutex_get(&privdata->i2cbus_mutex, OS_WAIT_FOREVER);
	snprintf(name, sizeof(name), "MDEV_I2C_%04X", addr);
	i2ccli_dev = mdev_get_handle((const char *)name);
	if (i2ccli_dev == NULL) {
		i2ccli_dev = (mdev_t *)os_mem_alloc(sizeof(mdev_t) + 14);
		if (!i2ccli_dev) {
			goto exit_open;
		}
		memset(i2ccli_dev, 0, sizeof(mdev_t) + 14);
	}
	memcpy((char*)&i2ccli_dev[1], name, 14);
	i2ccli_dev->dev_id = (addr & 0x3FF) | (flags & 0xFC00);
	i2ccli_dev->port_id = id;
	i2ccli_dev->name = (const char*)&i2ccli_dev[1];
	i2ccli_dev->private_data = (uint32_t)privdata;

	if (mdev_register(i2ccli_dev) != WM_SUCCESS) {
		os_mem_free(i2ccli_dev);
		i2ccli_dev = NULL;
	}
	privdata->refcnt++;
exit_open:
	os_mutex_put(&privdata->i2cbus_mutex);
	return i2ccli_dev;
}

int i2cbus_drv_close(mdev_t *dev)
{
	int ret = -WM_FAIL;
	mdev_i2cbus_priv_data_t *privdata;

	if (dev == NULL || !dev->name) {
		return ret;
	}
	privdata = (mdev_i2cbus_priv_data_t *)dev->private_data;
	os_mutex_get(&privdata->i2cbus_mutex, OS_WAIT_FOREVER);
	if (mdev_deregister(dev->name) == WM_SUCCESS) {
		privdata->refcnt--;
		os_mem_free(dev);
		ret = WM_SUCCESS;
	}
	os_mutex_put(&privdata->i2cbus_mutex);
	return ret;
}

static int i2cbus_drv_mdev_init(I2C_ID_Type id)
{
	int ret;
	mdev_t *i2cbus_dev = &mdev_i2cbus[id];
	mdev_i2cbus_priv_data_t *privdata = &i2cbus_priv_dara[id];

	i2cbus_dev->port_id = id;
	i2cbus_dev->name = mdev_i2cbus_name[id];
	i2cbus_dev->pNextMdev = NULL;
	i2cbus_dev->private_data = (uint32_t)privdata;

	privdata->dev = i2cbus_dev;
	privdata->refcnt = 0;

	ret = os_mutex_create(&privdata->i2cbus_mutex,
						  mdev_i2cbus_name[id], OS_MUTEX_INHERIT);

	if (ret != WM_SUCCESS) {
		return ret;
	}
	ret = os_mutex_create(&privdata->i2cbus_rw_mutex,
						  rw_mutex_name[id], OS_MUTEX_INHERIT);
	if (ret != WM_SUCCESS) {
		os_mutex_delete(&privdata->i2cbus_mutex);
	}
	return ret;
}

int i2cbus_drv_init(I2C_ID_Type id)
{
	int ret;

	if (mdev_get_handle(mdev_i2cbus_name[id]) != NULL) {
		return WM_SUCCESS;
	}

	ret = i2c_drv_init(id);
	if (ret != WM_SUCCESS) {
		return ret;
	}
	if (i2cbus_drv_mdev_init(id) != WM_SUCCESS) {
		return -WM_FAIL;
	}
	return mdev_register(&mdev_i2cbus[id]);
}
