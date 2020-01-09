/*
*  (C) Copyright 2020 GlobalScale Technologie Inc. All Rights Reserved.
*/

#ifndef _HT1818G_H_
#define _HT1818G_H_

#include <sc16is740.h>

#define MDEV_HT1818G              "MDEV_HT1818G"

#define HT1818G_BAUD              9600
#define DEFAULT_FIXING_TIMEOUT    (2 * 60 * 1000)

int ht1818g_drv_get_location(mdev_t *dev,
                             float *latitude,
                             float *longitude,
                             int timeout);
int ht1818g_drv_get_location(mdev_t *dev,
                             float *latitude,
                             float *longitude,
                             int timeout);
mdev_t *ht1818g_drv_open();
int ht1818g_drv_close(mdev_t *dev);
int ht1818g_drv_init();
int ht1818g_drv_deinit();

extern int board_gps_power_pin();
#endif /* _HT1818G_H_ */
