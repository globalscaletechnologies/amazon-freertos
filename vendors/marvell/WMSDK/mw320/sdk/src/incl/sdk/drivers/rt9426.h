#ifndef _RT9426_H_
#define _RT9426_H_

#define MDEV_RT9426 "MDEV_RT9426"

#define RT9426_I2C_ADDR                 (0x55)
#define RT9426_SWVER                    (1)
#define RT9426_POLLING_INTERVAL         (30 * 1000) /* 30s */

typedef void (*rt9426_notify) (int level, bool charging);

typedef struct _battery_info {
    int level;
    bool charging;
    int current;
    int voltage;
    float temperature;
    int health;
    int remaing_capacity;
    int full_capacity;
    int design_capacity;
} battery_info_t;

int rt9426_getBatteryInfo(mdev_t *dev, battery_info_t *info);
mdev_t *rt9426_drv_open(rt9426_notify notify);
int rt9426_drv_close(mdev_t *dev);
int rt9426_drv_init();
int rt9426_drv_deinit();
#endif /* _RT9426_H_ */
