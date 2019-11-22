/*
*  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
*/

#ifndef _GROVE_PIR_MOTION_H_
#define _GROVE_PIR_MOTION_H_

#include <stdbool.h>
#include <mdev.h>

#define MDEV_GROVE_PIRMOTION "MDEV_GROVE_PIRMOTION"
typedef void (*motion_event_cb) (bool detect);

int PIRMotion_drv_init(int gpio);
int PIRMotion_drv_deinit(void);
mdev_t *PIRMotion_drv_open(const char *name, motion_event_cb callback);
int PIRMotion_drv_close(mdev_t *dev);
int PIRMotion_drv_register_callback(mdev_t *dev,
									motion_event_cb callback);
int PIRMotion_drv_unregister_callback(mdev_t *dev);

#endif /* _GROVE_PIR_MOTION_H_ */
