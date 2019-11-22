/** @file Grove_PIRMotion.c
 *
 *  @brief This file provides driver for Grove PIR Motion Sensor
 *
 *  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
 *
 */

#include <wmstdio.h>
#include <stdbool.h>
#include <mdev_i2cbus.h>
#include <Grove_PIRMotion.h>
#include <wm_os.h>
#include <math.h>
#include <lowlevel_drivers.h>

typedef struct _pir_motion_priv {
	mdev_t *dev;
	mdev_t *gpiod;
	int irq;
	motion_event_cb callback;
} pir_motion_priv_t;

static mdev_t mdev_motion;
static pir_motion_priv_t motion_privdata;
static os_mutex_t motion_mutex;

static inline void *mdev_to_priv(mdev_t *dev)
{
	return (dev)? (void *)(dev->private_data) : NULL;
}

static void motion_event_handler(int pin, void *data)
{
	pir_motion_priv_t *privdata = data;
	int state = 0;

	if (!privdata) {
		return;
	}
	if (privdata->callback) {
		if (gpio_drv_read(privdata->gpiod, pin, &state) < 0) {
			return;
		}
		privdata->callback(!!state);
	}
	return;
}

int PIRMotion_drv_register_callback(mdev_t *dev,
									motion_event_cb callback)
{
	int ret;
	pir_motion_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata || !callback) {
		return -WM_FAIL;
	}

	/* register gpio interrupt */
	ret = gpio_drv_set_cb(privdata->gpiod, privdata->irq,
						  GPIO_INT_BOTH_EDGES, privdata,
						  motion_event_handler);
	if (ret == WM_SUCCESS) {
		privdata->callback = callback;
	}
	return ret;
}

int PIRMotion_drv_unregister_callback(mdev_t *dev)
{
	pir_motion_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}

	if (privdata->callback) {
		gpio_drv_set_cb(privdata->gpiod, privdata->irq,
						GPIO_INT_DISABLE, NULL, NULL);
		privdata->callback = NULL;
	}
	return WM_SUCCESS;
}

mdev_t *PIRMotion_drv_open(const char *name, motion_event_cb callback)
{
	mdev_t *dev = mdev_get_handle(name);
	pir_motion_priv_t *privdata;

	if (dev == NULL) {
		return NULL;
	}
	privdata = mdev_to_priv(dev);
	if (!privdata) {
		return NULL;
	}
	if (os_mutex_get(&motion_mutex, OS_WAIT_FOREVER) < 0) {
		return NULL;
	}

	if (callback) {
		PIRMotion_drv_register_callback(dev, callback);
	}
	return dev;
}

int PIRMotion_drv_close(mdev_t *dev)
{
	pir_motion_priv_t *privdata = mdev_to_priv(dev);

	if (!privdata) {
		return -WM_FAIL;
	}
	if (privdata->callback) {
		PIRMotion_drv_unregister_callback(dev);
	}
	return os_semaphore_put(&motion_mutex);
}

static int PIRMotion_drv_setup(int gpio)
{
	mdev_t *dev = &mdev_motion;
	pir_motion_priv_t *privdata = &motion_privdata;

	dev->port_id = 0;
	dev->name = MDEV_GROVE_PIRMOTION;
	dev->pNextMdev = NULL;
	dev->private_data = (uint32_t)privdata;

	privdata->dev = dev;
	privdata->irq = gpio;
	privdata->callback = NULL;

	gpio_drv_init();
	privdata->gpiod = gpio_drv_open("MDEV_GPIO");
	if (!privdata->gpiod) {
		return -WM_FAIL;
	}
	gpio_drv_setdir(privdata->gpiod, gpio, GPIO_INPUT);

	return WM_SUCCESS;
}

int PIRMotion_drv_init(int gpio)
{
	int ret;
	if (mdev_get_handle(MDEV_GROVE_PIRMOTION) != NULL) {
		return WM_SUCCESS;
	}
	ret = os_mutex_create(&motion_mutex, "PIRMotion", OS_MUTEX_INHERIT);
	if (ret < 0) {
		return -WM_FAIL;
	}
	if (PIRMotion_drv_setup(gpio) != WM_SUCCESS) {
		os_mutex_delete(&motion_mutex);
		return -WM_FAIL;
	}
	return mdev_register(&mdev_motion);
}

int PIRMotion_drv_deinit(void)
{
	pir_motion_priv_t *privdata;
	mdev_t *dev = mdev_get_handle(MDEV_GROVE_PIRMOTION);

	if (!dev) {
		return WM_SUCCESS;
	}
	privdata = mdev_to_priv(dev);
	if (!privdata) {
		return -WM_FAIL;
	}
	if (!privdata->gpiod) {
		gpio_drv_close(dev);
	}
	os_mutex_delete(&motion_mutex);
	mdev_deregister(MDEV_GROVE_PIRMOTION);
	return WM_SUCCESS;
}
