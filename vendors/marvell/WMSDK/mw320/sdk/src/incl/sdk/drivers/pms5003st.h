/*
*  (C) Copyright 2020 GlobalScale Technologie Inc. All Rights Reserved.
*/

#ifndef _PMS5003ST_H_
#define _PMS5003ST_H_

#include <sc16is740.h>

#define MDEV_PMS5003ST              "MDEV_PMS5003ST"

#define PMS5003ST_BAUD              9600
#define PMS5003ST_FRAME_SIZE        40
#define PMS_PASSIVE_MODE            0
#define PMS_ACTIVE_MODE             1
#define PMS_PM_1_0                  0       // PM1.0
#define PMS_PM_2_5                  1       // PM2.5
#define PMS_PM_10_0                 2       // PM10

#define PMS_PCS_0_3UM               0       // particulates 0.3um
#define PMS_PCS_0_5UM               1       // particulates 0.5um
#define PMS_PCS_1_0UM               2       // particulates 1.0um
#define PMS_PCS_2_5UM               3       // particulates 2.5um
#define PMS_PCS_5_0UM               4       // particulates 5.0um
#define PMS_PCS_10_0UM              5       // particulates 10um

int pms5003st_sleep(mdev_t *dev);
int pms5003st_wakeup(mdev_t *dev);
int pms5003st_getHumidity(mdev_t *dev, int *humidity);
int pms5003st_getTemperature(mdev_t *dev, int *temperature);
int pms5003st_getHcho(mdev_t *dev, int *hcho);
int pms5003st_getPMAtmo(mdev_t *dev, int index, int *value);
int pms5003st_getPMCF1(mdev_t *dev, int index, int *value);
int pms5003st_getPcs(mdev_t *dev, int index, int *value);
int pms5003st_requestData(mdev_t *dev, int timeout);
mdev_t *pms5003st_drv_open(uint8_t mode);
int pms5003st_drv_close(mdev_t *dev);
int pms5003st_drv_init();
int pms5003st_drv_deinit();

#endif /* _PMS5003ST_H_ */
