/** @file pms5003st.c
 *
 *  @brief This file provides driver for pms5003st air quality sensor
 *
 *  (C) Copyright 2020 GlobalScale Technologie Inc. All Rights Reserved.
 *
 */

#include <wmstdio.h>
#include <stdbool.h>
#include <mdev_gpio.h>
#include <sc16is740.h>
#include <wm_os.h>
#include <lowlevel_drivers.h>
#include <pms5003st.h>
#include <ringbytebuffer.h>
#include <stream_buffer.h>

#define START_SIG1              0x42
#define START_SIG2              0x4D

#define PMS_CMD_READ            0xE2    /* require data */

#define PMS_CMD_MODE            0xE1    /* switch mode */
#define PMS_PASSIVE             0x0
#define PMS_ACTIVE              0x1

#define PMS_CMD_SUSPEND         0xE4    /* suspend mode */
#define PMS_DEVICE_SUSPEND      0x0
#define PMS_DEVICE_WAKEUP       0x1

#define PMS_BUFFER_SIZE         1000

#define swap_byte_16(x) \
            ((uint16_t)((((uint16_t)(x) & 0x00ff) << 8) | \
                        (((uint16_t)(x) & 0xff00) >> 8)))

typedef struct __attribute__((packed)) {
    uint8_t sig1;
    uint8_t sig2;
    uint8_t cmd;
    uint16_t data;
    uint16_t checksum;
} pms_comm_header_t;

typedef struct __attribute__((packed)) {
    uint16_t pm10_std;
    uint16_t pm25_std;
    uint16_t pm100_std;
    uint16_t pm10_env;
    uint16_t pm25_env;
    uint16_t pm100_env;
    uint16_t particles_03um;
    uint16_t particles_05um;
    uint16_t particles_10um;
    uint16_t particles_25um;
    uint16_t particles_50um;
    uint16_t particles_100um;
    uint16_t hcho;
    uint16_t temperature;
    uint16_t humidity;
    uint16_t reserved;
    uint8_t err_code;
    uint8_t version;
    uint16_t checksum;
} pms5003st_data_t;

typedef struct _pms5003st_priv {
    mdev_t *dev;
    mdev_t *uart;

    uint8_t mode;
    bool suspend;
    pms5003st_data_t data;
    os_mutex_t mutexUartAccess;
    os_mutex_t mutexDataAccess;

    StreamBufferHandle_t streambufRecvData;
    bool bExitThread;
    os_thread_t threadUartRecv;
    os_thread_t threadDataDecode;
    os_semaphore_t semDataReady;
} pms5003st_priv_t;

static mdev_t mdev_pms5003st;
static pms5003st_priv_t pms5003st_priv_data;
static os_thread_stack_define(pms5003st_recvdata_stack, 512);
static os_thread_stack_define(pms5003st_datadecode_stack, 512);

static inline void *mdev_to_priv(mdev_t *dev)
{
    return (dev)? (void *)(dev->private_data) : NULL;
}

static int pms5003st_sendCommand(pms5003st_priv_t *privdata,
                                 uint8_t cmd,
                                 uint16_t data)
{
    pms_comm_header_t header;
    uint8_t *p = (uint8_t *)&header;
    uint16_t checksum = 0, i;
    int ret;

    if (!privdata) {
        return -WM_FAIL;
    }

    header.sig1 = START_SIG1;
    header.sig2 = START_SIG2;
    header.cmd = cmd;
    header.data = swap_byte_16(data);
    for (i = 0; i < 5; i++) {
        checksum += p[i];
    }
    header.checksum = swap_byte_16(checksum);
    os_mutex_get(&privdata->mutexUartAccess, OS_WAIT_FOREVER);
    ret = sc16is740_drv_write(privdata->uart, &header, sizeof(header));
    os_mutex_put(&privdata->mutexUartAccess);
    if (ret < 0) {
        return -WM_FAIL;
    }
    return WM_SUCCESS;
}

static int pms5003st_recvResponse(pms5003st_priv_t *privdata,
                                  uint8_t cmd,
                                  uint8_t *resp,
                                  uint16_t *length)
{
    uint16_t checksum = 0, checksum1 = 0, len = 0;
    int i, size = 0, reads = 0;
    uint8_t buf[PMS5003ST_FRAME_SIZE];

    if (!privdata) {
        return -WM_FAIL;
    }

    sc16is740_drv_rx_buf_reset(privdata->uart);
    /* only 0xe2 command return 40 bytes */
    size = (cmd == PMS_CMD_READ)? PMS5003ST_FRAME_SIZE : 0x8;

    os_mutex_get(&privdata->mutexUartAccess, OS_WAIT_FOREVER);
    reads = sc16is740_drv_read(privdata->uart,
                               buf,
                               size);
    os_mutex_put(&privdata->mutexUartAccess);
    if (reads != size) {
        wm_printf("receive packet size is not match\r\n");
        return -WM_FAIL;
    }
    if (buf[0] == START_SIG1 && buf[1] == START_SIG2) {
        len = buf[2] << 8 | buf[3];
        checksum1 = buf[len + 4 - 2] << 8 | buf[len + 4 - 1];
        for (i = 0, checksum = 0; i < ((len + 4) - 2); i++) {
            checksum += buf[i];
        }
        if (checksum == checksum1) {
            memcpy(resp, &buf[4], len);
            *length = len;
        } else {
            wm_printf("invalid packet, checksum not match\r\n");
            return -WM_FAIL;
        }
    } else {
        wm_printf("invalid packet #1\r\n");
        return -WM_FAIL;
    }
    return WM_SUCCESS;
}

static int pms5003st_setMode(pms5003st_priv_t *privdata, uint8_t mode)
{
    int ret;
    uint8_t buf[4];
    uint16_t len = 0;

    if (!privdata) {
        return -WM_FAIL;
    }
    if (mode > 1) {
        wm_printf("[PMS5GST] unsupported mode.\r\n");
        return -WM_FAIL;
    }
    ret = pms5003st_sendCommand(privdata, PMS_CMD_MODE, mode);

    if (ret == WM_SUCCESS) {
        privdata->mode = mode;
        /* read reponse */
        pms5003st_recvResponse(privdata, PMS_CMD_MODE, buf, &len);
    }
    return ret;
}

int pms5003st_sleep(mdev_t *dev)
{
    pms5003st_priv_t *privdata = mdev_to_priv(dev);
    int ret;

    if (!privdata) {
        return -WM_FAIL;
    }

    ret = pms5003st_sendCommand(privdata,
                                PMS_CMD_SUSPEND,
                                PMS_DEVICE_SUSPEND);
    if (ret == WM_SUCCESS) {
        privdata->suspend = true;
    }
    return ret;
}

int pms5003st_wakeup(mdev_t *dev)
{
    pms5003st_priv_t *privdata = mdev_to_priv(dev);
    int ret;

    if (!privdata) {
        return -WM_FAIL;
    }

    ret = pms5003st_sendCommand(privdata,
                                PMS_CMD_SUSPEND,
                                PMS_DEVICE_WAKEUP);
    if (ret == WM_SUCCESS) {
        privdata->suspend = false;
    }
    return ret;
}

int pms5003st_getHumidity(mdev_t *dev, int *humidity)
{
    pms5003st_priv_t *privdata = mdev_to_priv(dev);

    if (!privdata) {
        return -WM_FAIL;
    }
    /* percentage * 10 */
    os_mutex_get(&privdata->mutexDataAccess, OS_WAIT_FOREVER);
    *humidity = swap_byte_16(privdata->data.humidity);
    os_mutex_put(&privdata->mutexDataAccess);
    return WM_SUCCESS;
}

int pms5003st_getTemperature(mdev_t *dev, int *temperature)
{
    pms5003st_priv_t *privdata = mdev_to_priv(dev);

    if (!privdata) {
        return -WM_FAIL;
    }
    /* 0.1 celsius */
    os_mutex_get(&privdata->mutexDataAccess, OS_WAIT_FOREVER);
    *temperature = swap_byte_16(privdata->data.temperature);
    os_mutex_put(&privdata->mutexDataAccess);
    return WM_SUCCESS;
}

int pms5003st_getHcho(mdev_t *dev, int *hcho)
{
    pms5003st_priv_t *privdata = mdev_to_priv(dev);

    if (!privdata) {
        return -WM_FAIL;
    }
    /* 0.001 mg/m3 */
    os_mutex_get(&privdata->mutexDataAccess, OS_WAIT_FOREVER);
    *hcho = swap_byte_16(privdata->data.hcho);
    os_mutex_put(&privdata->mutexDataAccess);
    return WM_SUCCESS;
}

int pms5003st_getPMAtmo(mdev_t *dev, int index, int *value)
{
    pms5003st_priv_t *privdata = mdev_to_priv(dev);
    int ret = WM_SUCCESS;

    if (!privdata) {
        return -WM_FAIL;
    }
    os_mutex_get(&privdata->mutexDataAccess, OS_WAIT_FOREVER);
    switch (index) {
        case PMS_PM_1_0:
            /* unit: ug/m3 */
            *value = swap_byte_16(privdata->data.pm10_env);
        break;
        case PMS_PM_2_5:
            *value = swap_byte_16(privdata->data.pm25_env);
        break;
        case PMS_PM_10_0:
            *value = swap_byte_16(privdata->data.pm100_env);
        break;
        default:
            ret = -WM_FAIL;
        break;
    }
    os_mutex_put(&privdata->mutexDataAccess);
    return ret;
}

int pms5003st_getPMCF1(mdev_t *dev, int index, int *value)
{
    pms5003st_priv_t *privdata = mdev_to_priv(dev);
    int ret = WM_SUCCESS;

    if (!privdata) {
        return -WM_FAIL;
    }
    os_mutex_get(&privdata->mutexDataAccess, OS_WAIT_FOREVER);
    switch (index) {
        case PMS_PM_1_0:
            *value = swap_byte_16(privdata->data.pm10_std);
        break;
        case PMS_PM_2_5:
            *value = swap_byte_16(privdata->data.pm25_std);
        break;
        case PMS_PM_10_0:
            *value = swap_byte_16(privdata->data.pm100_std);
        break;
        default:
            ret = -WM_FAIL;
        break;
    }
    os_mutex_put(&privdata->mutexDataAccess);
    return ret;
}

int pms5003st_getPcs(mdev_t *dev, int index, int *value)
{
    pms5003st_priv_t *privdata = mdev_to_priv(dev);
    int ret = WM_SUCCESS;

    if (!privdata) {
        return -WM_FAIL;
    }
    os_mutex_get(&privdata->mutexDataAccess, OS_WAIT_FOREVER);
    switch (index) {
        case PMS_PCS_0_3UM:
            /* units: pcs/0.1L */
            *value = swap_byte_16(privdata->data.particles_03um);
        break;
        case PMS_PCS_0_5UM:
            *value = swap_byte_16(privdata->data.particles_05um);
        break;
        case PMS_PCS_1_0UM:
            *value = swap_byte_16(privdata->data.particles_10um);
        break;
        case PMS_PCS_2_5UM:
            *value = swap_byte_16(privdata->data.particles_25um);
        break;
        case PMS_PCS_5_0UM:
            *value = swap_byte_16(privdata->data.particles_50um);
        break;
        case PMS_PCS_10_0UM:
            *value = swap_byte_16(privdata->data.particles_100um);
        break;
        default:
            ret = -WM_FAIL;
        break;
    }
    os_mutex_put(&privdata->mutexDataAccess);
    return ret;
}

int pms5003st_requestData(mdev_t *dev, int timeout)
{
    pms5003st_priv_t *privdata = mdev_to_priv(dev);
    uint8_t buf[PMS5003ST_FRAME_SIZE - 4];
    uint16_t len = 0;
    int ret = WM_SUCCESS;

    if (!privdata) {
        return -WM_FAIL;
    }

    if (privdata->mode == PMS_PASSIVE) {
        ret = pms5003st_sendCommand(privdata, PMS_CMD_READ, 0);
        if (ret == WM_SUCCESS) {
            ret = pms5003st_recvResponse(privdata,
                                         PMS_CMD_READ,
                                         buf,
                                         &len);
            if (ret == WM_SUCCESS) {
                os_mutex_get(&privdata->mutexDataAccess,
                             OS_WAIT_FOREVER);
                memcpy(&privdata->data, buf, len);
                os_mutex_put(&privdata->mutexDataAccess);
            }
        }
    } else {
        if (privdata->semDataReady) {
            ret = os_semaphore_get(&privdata->semDataReady,
                                   pdMS_TO_TICKS(timeout));
        }
    }
    return ret;
}

static void pms5003st_recvThread(os_thread_arg_t data)
{
    mdev_t *dev;
    int reads = 0, writes = 0;
    pms5003st_priv_t *privdata = data;
    uint8_t buf[PMS5003ST_FRAME_SIZE];

    dev = privdata->uart;

    do {
        os_mutex_get(&privdata->mutexUartAccess, OS_WAIT_FOREVER);
        reads = sc16is740_drv_read(dev, buf, PMS5003ST_FRAME_SIZE);
        os_mutex_put(&privdata->mutexUartAccess);
        if (privdata->bExitThread) {
            break;
        }
        if (reads > 0) {
            writes = xStreamBufferSend(privdata->streambufRecvData,
                                       (void *)buf,
                                       reads,
                                       pdMS_TO_TICKS(100));
            if (writes != reads) {
                wm_printf("no enough space to save uart data\r\n");
            }
        }
    } while (1);

    os_thread_self_complete(NULL);
}

static void pms5003st_decodeThread(os_thread_arg_t data)
{
    int reads = 0, i;
    uint16_t len = 0, checksum = 0, checksum1;
    pms5003st_priv_t *privdata = data;
    uint8_t buf[PMS5003ST_FRAME_SIZE];

    while (!privdata->bExitThread) {
        reads = xStreamBufferReceive(privdata->streambufRecvData,
                                     buf + reads,
                                     2 - reads,
                                     pdMS_TO_TICKS(200));
        if (reads < 2) {
            continue;
        }

        if (buf[0] == START_SIG1 && buf[1] == START_SIG2) {
            reads -= 2;
        } else {
            if (buf[1] == START_SIG1) {
                buf[0] = buf[1];
                reads--;
            } else {
                reads -= 2;
            }
            continue;
        }

        reads = xStreamBufferReceive(privdata->streambufRecvData,
                                     &buf[2],
                                     PMS5003ST_FRAME_SIZE - 2,
                                     pdMS_TO_TICKS(150));
        if (reads != (PMS5003ST_FRAME_SIZE - 2)) {
            /* wait to long, drop this packet */
            continue;
        }
        len = buf[2] << 8 | buf[3];
        checksum1 = buf[38] << 8 | buf[39];
        for (i = 0, checksum = 0; i < (PMS5003ST_FRAME_SIZE - 2); i++) {
            checksum += buf[i];
        }
        if (len == 0x24 && checksum == checksum1) {
            os_mutex_get(&privdata->mutexDataAccess, OS_WAIT_FOREVER);
            memcpy(&privdata->data, &buf[4], sizeof(pms5003st_data_t));
            os_mutex_put(&privdata->mutexDataAccess);
            if (privdata->semDataReady) {
                os_semaphore_put(&privdata->semDataReady);
            }
        } else {
            wm_printf("invalid packet, checksum not match\r\n");
        }
    }

    os_thread_self_complete(NULL);
}

mdev_t *pms5003st_drv_open(uint8_t mode)
{
    int ret;
    pms5003st_priv_t *privdata;
    mdev_t *dev = mdev_get_handle(MDEV_PMS5003ST);

    privdata = mdev_to_priv(dev);
    if (privdata == NULL) {
        return NULL;
    }

    if (mode > 1) {
        wm_printf("unsupported mode %u\r\n", mode);
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
    if (sc16is740_drv_timeout(1000, 1000) < 0)
    {
        goto err_open;
    }
    if (sc16is740_drv_blocking_read(true) < 0) {
        goto err_open;
    }
    privdata->uart = sc16is740_drv_open(PMS5003ST_BAUD);
    if (privdata->uart == NULL) {
        goto err_open;
    }
    /* reset uart buffer */
    sc16is740_drv_rx_buf_reset(privdata->uart);

    if (pms5003st_setMode(privdata, mode) < 0) {
        goto err_open;
    }

    if (mode == PMS_ACTIVE) {
        privdata->streambufRecvData = xStreamBufferCreate(
                                                PMS_BUFFER_SIZE,
                                                PMS5003ST_FRAME_SIZE);
        if (!privdata->streambufRecvData) {
            wm_printf("no enough memory to allocate streambuffer\r\n");
            goto err_open;
        }
        ret = os_thread_create(&privdata->threadUartRecv,
                               "pms5gst_threadUartRecv",
                               pms5003st_recvThread,
                               privdata,
                               &pms5003st_recvdata_stack,
                               OS_PRIO_2);
        if (ret < 0) {
            goto err_open;
        }
        ret = os_thread_create(&privdata->threadDataDecode,
                               "pms5gst_threadDataDecode",
                               pms5003st_decodeThread,
                               privdata,
                               &pms5003st_datadecode_stack,
                               OS_PRIO_2);
        if (ret < 0) {
            goto err_open;
        }
    } else {
        if (os_semaphore_create(&privdata->semDataReady,
                                "pms5gst_dataReady") < 0) {
            goto err_open;
        }
        os_semaphore_get(&privdata->semDataReady, OS_WAIT_FOREVER);
    }
    return dev;

err_open:

    if (privdata->semDataReady) {
        os_semaphore_delete(&privdata->semDataReady);
        privdata->semDataReady = NULL;
    }
    if (privdata->threadUartRecv) {
        os_thread_delete(&privdata->threadUartRecv);
        privdata->threadUartRecv = NULL;
    }
    if (privdata->streambufRecvData) {
        vStreamBufferDelete(privdata->streambufRecvData);
        privdata->streambufRecvData = NULL;
    }
    if (privdata->uart) {
        sc16is740_drv_close(privdata->uart);
        privdata->uart = NULL;
    }
    return NULL;
}

int pms5003st_drv_close(mdev_t *dev)
{
    pms5003st_priv_t *privdata = mdev_to_priv(dev);

    if (!privdata) {
        return -WM_FAIL;
    }

    privdata->bExitThread = true;
    if (privdata->semDataReady) {
        os_semaphore_put(&privdata->semDataReady);
        os_semaphore_delete(&privdata->semDataReady);
        privdata->semDataReady = NULL;
    }
    if (privdata->threadUartRecv) {
        os_thread_delete(&privdata->threadUartRecv);
        privdata->threadUartRecv = NULL;
    }
    if (privdata->threadDataDecode) {
        os_thread_delete(&privdata->threadDataDecode);
        privdata->threadDataDecode = NULL;
    }
    if (privdata->streambufRecvData) {
        vStreamBufferDelete(privdata->streambufRecvData);
        privdata->streambufRecvData = NULL;
    }
    if (privdata->uart) {
        sc16is740_drv_close(privdata->uart);
        privdata->uart = NULL;
    }

    return WM_SUCCESS;
}

static int pms5003st_drv_setup()
{
    int ret;
    mdev_t *dev = &mdev_pms5003st;
    pms5003st_priv_t *privdata = &pms5003st_priv_data;
    bool bUartInitialized = false;

    /* init mdev node */
    dev->port_id = 0;
    dev->name = MDEV_PMS5003ST;
    dev->pNextMdev = NULL;
    dev->private_data = (uint32_t)privdata;

    privdata->dev = dev;
    privdata->bExitThread = false;

    ret = os_mutex_create(&privdata->mutexUartAccess,
                          "pms5gst_uart_access",
                          OS_MUTEX_INHERIT);
    if (ret != WM_SUCCESS) {
        goto err_setup;
    }
    ret = os_mutex_create(&privdata->mutexDataAccess,
                          "pms5gst_data_access",
                          OS_MUTEX_INHERIT);
    if (ret != WM_SUCCESS) {
        goto err_setup;
    }

    ret = sc16is740_drv_init();
    if (ret < 0) {
        goto err_setup;
    }
    bUartInitialized = true;

    return WM_SUCCESS;

err_setup:
    if (bUartInitialized) {
        sc16is740_drv_deinit();
        bUartInitialized = false;
    }
    if (privdata->mutexDataAccess) {
        os_mutex_delete(&privdata->mutexDataAccess);
        privdata->mutexDataAccess = NULL;
    }
    if (privdata->mutexUartAccess) {
        os_mutex_delete(&privdata->mutexUartAccess);
        privdata->mutexUartAccess = NULL;
    }
    return -WM_FAIL;
}

int pms5003st_drv_init()
{
    if (mdev_get_handle(MDEV_PMS5003ST) != NULL) {
        return WM_SUCCESS;
    }
    if (pms5003st_drv_setup() != WM_SUCCESS) {
        return -WM_FAIL;
    }
    return mdev_register(&mdev_pms5003st);
}

int pms5003st_drv_deinit()
{
    pms5003st_priv_t *privdata = &pms5003st_priv_data;

    if (!mdev_get_handle(MDEV_PMS5003ST)) {
        return WM_SUCCESS;
    }
    if (privdata->uart) {
        sc16is740_drv_close(privdata->uart);
        privdata->uart = NULL;
    }
    sc16is740_drv_deinit();

    if (privdata->mutexDataAccess) {
        os_mutex_delete(&privdata->mutexDataAccess);
        privdata->mutexDataAccess = NULL;
    }

    if (privdata->mutexUartAccess) {
        os_mutex_delete(&privdata->mutexUartAccess);
        privdata->mutexUartAccess = NULL;
    }
    mdev_deregister(MDEV_PMS5003ST);
    return WM_SUCCESS;
}
