/** @file ht1818g.c
 *
 *  @brief This file provides driver for GPS (HT-1818-G) module
 *
 *  (C) Copyright 2020 GlobalScale Technologie Inc. All Rights Reserved.
 *
 */

#include <wm_os.h>
#include <wmstdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <mdev_gpio.h>
#include <lowlevel_drivers.h>
#include <sc16is740.h>
#include <ht1818g.h>

typedef enum _nmea0183_type {
    NMEA_GGA = 1,
    NMEA_RMC = 2,
    NMEA_OTHERS = 0 /* we only care GGA & RMC type */
} nmea0183_type_t;

typedef struct _gps_position {
    float latitude;
    float longitude;
} gps_position_t;

typedef struct _ht1818g_priv {
    mdev_t *dev;
    mdev_t *uart;
    mdev_t *gpiod;

    bool bValidPosition;
    gps_position_t position;
    os_semaphore_t semGPSFixing;

    bool bExitThread;
    os_thread_t threadUartRecv;
} ht1818g_priv_t;

static mdev_t mdev_ht1818g;
static ht1818g_priv_t ht1818g_priv_data;
static os_thread_stack_define(ht1818g_recvdata_stack, 1024);

static float _latitude = 0;
static float _longitude = 0;

static inline void *mdev_to_priv(mdev_t *dev)
{
    return (dev)? (void *)(dev->private_data) : NULL;
}

float to_degree(char *str)
{
    /* string format : ddmm.mmmm | dddmm.mmmm */
    int i, dot = -1, left = 0, right = 0, val = 0, n = 1;
    int degree = 0, minute = 0;
    float latitude = 0;

    for (i = 0; i < strlen(str); i++) {
        if (str[i] == '.') {
            dot = i;
            left = val;
            val = 0;
            continue;
        }
        val = val * 10 + (str[i] - 0x30);
        n = (dot > 0)? n * 10 : n;
    }
    if (dot != -1) {
        right = val;
    }
    degree = left / 100;
    minute = (left % 100) * n + right;
    latitude = (float) degree + ((float)minute / (n * 60));
    return latitude;
}

bool parse(uint8_t c)
{
    static uint8_t message[256] = {0};
    static int idx = 0;
    static int nums = 0;
    static int type = 0;
    static uint8_t checksum = 0;
    static int is_fixed = 0;
    static int status = 0;
    static bool is_checksum = false;
    uint8_t chksum = 0;

    if (c > 0x7F) {
        /* drop invalid data */
        return false;
    }
    switch(c) {
        case ',':
        checksum ^= c;
        case '*':
        case '\r':
        case '\n':
        message[idx++] = 0;
        nums++;
        if (c == '*') {
            is_checksum = true;
        }
        idx = 0;
        if (nums == 1) {
            if(strcmp((const char*)message, "GPGGA") == 0 ||
               strcmp((const char*)message, "GNGGA") == 0) {
                type = NMEA_GGA;
            } else  if(strcmp((const char*)message, "GPRMC") == 0 ||
                       strcmp((const char*)message, "GNRMC") == 0) {
                type = NMEA_RMC;
            } else {
                type = NMEA_OTHERS;
            }
        }
        if (type == NMEA_GGA && nums > 1) {
            switch (nums) {
                case 3: // Latitude
                    _latitude = to_degree((char*)message);
                break;
                case 4: // N/S indicator
                    if (strncmp((const char *)message, "S", 1) == 0) {
                        _latitude *= -1.0;
                    }
                break;
                case 5: // Longitude
                    _longitude = to_degree((char*)message);
                break;
                case 6:
                    if (strncmp((const char *)message, "W", 1) == 0) {
                        _longitude *= -1.0;
                    }
                break;
                case 7: // GPS quality indicator
                    is_fixed = atoi((char*)message);
                break;
                case 16: // checksum
                chksum = (uint8_t)strtol((const char *)message,
                                         NULL,
                                         16);
                if (chksum == checksum) {
                    if (is_fixed > 0) {
                        return true;
                    }
                }
                break;
            }
        }
        if (type == NMEA_RMC && nums > 1) {
            switch (nums) {
                case 3: // status
                    if (strncmp((const char *)message, "A", 1) == 0) {
                        status = 1;
                    }
                break;
                case 4: // Latitude
                    _latitude = to_degree((char*)message);
                break;
                case 5: // N/S indicator
                    if (strncmp((const char *)message, "S", 1) == 0) {
                        _latitude *= -1.0;
                    }
                break;
                case 6: // Longitude
                    _longitude = to_degree((char*)message);
                break;
                case 7:
                    if (strncmp((const char *)message, "W", 1) == 0) {
                        _longitude *= -1.0;
                    }
                break;
                case 13: // checksum
                chksum = (uint8_t)strtol((const char *)message,
                                         NULL,
                                         16);
                if (chksum == checksum) {
                    if (status) {
                        return true;
                    }
                }
                break;
            }
        }
        return false;

        case '$':
        /* sentence start */
        nums = 0;
        idx = 0;
        checksum = 0;
        is_checksum = false;
        status = 0;
        is_fixed = 0;
        type = NMEA_OTHERS;
        memset(message, 0 , sizeof(message));
        return false;
    }
    message[idx++] = c;
    if (!is_checksum) {
        checksum ^= c;
    }
    return false;
}

static void ht1818g_processingThread(os_thread_arg_t data)
{
    int reads = 0;
    ht1818g_priv_t *privdata = data;
    uint8_t buf[128];
    uint8_t *p;

    do {
        reads = sc16is740_drv_read(privdata->uart, buf, 128);
        if (privdata->bExitThread) {
            break;
        }
        p = buf;
        while (reads-- > 0) {
            if (parse(*p++)) {
                privdata->position.latitude = _latitude;
                privdata->position.longitude = _longitude;

                /* notify */
                if (!privdata->bValidPosition &&
                    privdata->semGPSFixing) {
                    privdata->bValidPosition = true;
                    os_semaphore_put(&privdata->semGPSFixing);
                }
            }
        }
    } while (1);

    os_thread_self_complete(NULL);
}

static int ht1818g_drv_set_power(ht1818g_priv_t *privdata, bool on)
{
    if (!privdata) {
        return -WM_FAIL;
    }
    gpio_drv_write(privdata->gpiod,
                   board_gps_power_pin(),
                   (on)? GPIO_IO_LOW : GPIO_IO_HIGH);
    return WM_SUCCESS;
}

int ht1818g_drv_get_location(mdev_t *dev,
                             float *latitude,
                             float *longitude,
                             int timeout)
{
    int ret = 0;
    ht1818g_priv_t *privdata = mdev_to_priv(dev);

    if (!privdata) {
        return 0;
    }
    privdata->bValidPosition = false;
    /* turn on gps */
    ht1818g_drv_set_power(privdata, true);
    /* start to get gps location */
    ret = os_semaphore_get(&privdata->semGPSFixing,
                           os_msec_to_ticks(timeout));

    if (ret == WM_SUCCESS && privdata->bValidPosition) {
        *latitude = privdata->position.latitude;
        *longitude = privdata->position.longitude;
    }
    /* turn off gps */
    ht1818g_drv_set_power(privdata, false);

    return (privdata->bValidPosition)? WM_SUCCESS : -WM_FAIL;
}

mdev_t *ht1818g_drv_open()
{
    int ret;
    ht1818g_priv_t *privdata;
    mdev_t *dev = mdev_get_handle(MDEV_HT1818G);

    privdata = mdev_to_priv(dev);
    if (privdata == NULL) {
        return NULL;
    }

    /* configure uart options */
    if (sc16is740_drv_set_opts(SC_DATABITS_8,
                               SC_PARITY_NONE,
                               SC_STOPBITS_1,
                               SC_FLOWCTRL_NONE) < 0)
    {
        goto err_open;
    }
    if (sc16is740_drv_timeout(1000, 1000) < 0) {
        goto err_open;
    }
    if (sc16is740_drv_blocking_read(true) < 0) {
        goto err_open;
    }
    privdata->uart = sc16is740_drv_open(HT1818G_BAUD);
    if (privdata->uart == NULL) {
        goto err_open;
    }

    ret = os_thread_create(&privdata->threadUartRecv,
                           "pms5gst_threadUartRecv",
                           ht1818g_processingThread,
                           privdata,
                           &ht1818g_recvdata_stack,
                           OS_PRIO_1);
    if (ret < 0) {
        goto err_open;
    }
    return dev;

err_open:

    if (privdata->threadUartRecv) {
        os_thread_delete(&privdata->threadUartRecv);
        privdata->threadUartRecv = NULL;
    }
    if (privdata->uart) {
        sc16is740_drv_close(privdata->uart);
        privdata->uart = NULL;
    }
    return NULL;
}

int ht1818g_drv_close(mdev_t *dev)
{
    ht1818g_priv_t *privdata = mdev_to_priv(dev);

    if (!privdata) {
        return -WM_FAIL;
    }

    privdata->bExitThread = true;

    if (privdata->threadUartRecv) {
        os_thread_delete(&privdata->threadUartRecv);
        privdata->threadUartRecv = NULL;
    }
    if (privdata->uart) {
        sc16is740_drv_close(privdata->uart);
        privdata->uart = NULL;
    }

    return WM_SUCCESS;
}

static int ht1818g_drv_setup()
{
    int ret;
    mdev_t *dev = &mdev_ht1818g;
    ht1818g_priv_t *privdata = &ht1818g_priv_data;
    bool bUartInitialized = false;

    /* init mdev node */
    dev->port_id = 0;
    dev->name = MDEV_HT1818G;
    dev->pNextMdev = NULL;
    dev->private_data = (uint32_t)privdata;

    privdata->dev = dev;
    privdata->bValidPosition = false;
    privdata->position.latitude = 0;
    privdata->position.longitude = 0;
    privdata->bExitThread = false;

    ret = os_semaphore_create(&privdata->semGPSFixing,
                              "ht1818g_gps_fixing");
    if (ret != WM_SUCCESS) {
        goto err_setup;
    }
    os_semaphore_get(&privdata->semGPSFixing, OS_WAIT_FOREVER);

    ret = sc16is740_drv_init();
    if (ret < 0) {
        goto err_setup;
    }
    bUartInitialized = true;

    gpio_drv_init();
    privdata->gpiod = gpio_drv_open("MDEV_GPIO");

    if (privdata->gpiod == NULL) {
        goto err_setup;
    }
    gpio_drv_setdir(privdata->gpiod,
                    board_gps_power_pin(),
                    GPIO_OUTPUT);
    gpio_drv_write(privdata->gpiod,
                   board_gps_power_pin(),
                   GPIO_IO_HIGH);

    return WM_SUCCESS;

err_setup:
    if (privdata->gpiod) {
        gpio_drv_close(privdata->gpiod);
        privdata->gpiod = NULL;
    }
    if (bUartInitialized) {
        sc16is740_drv_deinit();
        bUartInitialized = false;
    }

    if (privdata->semGPSFixing) {
        os_semaphore_delete(&privdata->semGPSFixing);
        privdata->semGPSFixing = NULL;
    }
    return -WM_FAIL;
}

int ht1818g_drv_init()
{
    if (mdev_get_handle(MDEV_HT1818G) != NULL) {
        return WM_SUCCESS;
    }
    if (ht1818g_drv_setup() != WM_SUCCESS) {
        return -WM_FAIL;
    }
    return mdev_register(&mdev_ht1818g);
}

int ht1818g_drv_deinit()
{
    ht1818g_priv_t *privdata = &ht1818g_priv_data;

    if (!mdev_get_handle(MDEV_HT1818G)) {
        return WM_SUCCESS;
    }

    if (privdata->uart) {
        sc16is740_drv_close(privdata->uart);
        privdata->uart = NULL;
    }
    sc16is740_drv_deinit();

    if (privdata->semGPSFixing) {
        os_semaphore_delete(&privdata->semGPSFixing);
        privdata->semGPSFixing = NULL;
    }
    mdev_deregister(MDEV_HT1818G);
    return WM_SUCCESS;
}
