/*
 * rt9426.c: mdev driver for battery gauge (rt9426)
 *
 * based on Richtek RT9426 Driver Sample Code V1.1.4 20190726
 */
#include <wmstdio.h>
#include <mdev_gpio.h>
#include <mdev_i2cbus.h>
#include <wm_os.h>
#include <lowlevel_drivers.h>
#include <rt9426.h>

#define REG_CNTL                   (0x00)
#define REG_RSVD                   (0x02)
#define REG_CURR                   (0x04)
#define REG_TEMP                   (0x06)
#define REG_VBAT                   (0x08)
#define REG_FLAG1                  (0x0A)
#define REG_FLAG2                  (0x0C)
#define REG_RM                     (0x10)
#define REG_FCC                    (0x12)
#define REG_DUMMY                  (0x1E)
#define REG_INTT                   (0x28)
#define REG_CYC                    (0x2A)
#define REG_SOC                    (0x2C)
#define REG_SOH                    (0x2E)
#define REG_VER                    (0x20)
#define REG_FLAG3                  (0x30)
#define REG_IRQ                    (0x36)
#define REG_DC                     (0x3C)
#define REG_BDCNTL                 (0x3E)
#define REG_CMD3F                  (0x3F)
#define REG_SWINDOW1               (0x40)
#define REG_SWINDOW2               (0x42)
#define REG_SWINDOW3               (0x44)
#define REG_SWINDOW4               (0x46)
#define REG_SWINDOW5               (0x48)
#define REG_SWINDOW6               (0x4A)
#define REG_SWINDOW7               (0x4C)
#define REG_SWINDOW8               (0x4E)
#define REG_CMD60                  (0x60)
#define REG_CMD61                  (0x61)
#define REG_CMD78                  (0x78)

#define MASK_REG_FLAG3_BSC_ACS     (0x0001)
#define MASK_REG_FLAG2_SLP_STS 	   (0x8000)
#define MASK_REG_FLAG2_SHDN_STS    (0x4000)

/* RT9426 Initail Setting */
#define RT9426_CFG_VERSION         (0x01)
#define RT9426_BATTERY_TYPE        (4200)
#define RT9426_DC                  (3300)
#define RT9426_FCC                 (3300)
#define RT9426_FC_VTH              (0x50)
#define RT9426_FC_ITH              (0x10)
#define RT9426_FC_STH              (0x05)
#define RT9426_FD_VTH              (0x6E)
#define RT9426_OPCFG1              (0x3480)
#define RT9426_OPCFG2              (0x1001)
#define RT9426_OPCFG3              (0x30FB)
#define RT9426_OPCFG4              (0x2000)
#define RT9426_OPCFG5              (0x087F)
#define RT9426_OTC_TTH             (0x0064)
#define RT9426_OTC_CHG_ITH         (0x0B5F)
#define RT9426_OTD_TTH             (0x0064)
#define RT9426_OTD_DCHG_ITH        (0x0B5F)
#define RT9426_UVOV_TH             (0x00FF)
#define RT9426_US_TH               (0x00)
#define RT9426_CURR_DB             (0x05)
#define RT9426_QS_EN               (0x01)

#define RT9426_EXTREG_SIZE         (19)

const uint16_t RT9426_EXTREG_DATA[RT9426_EXTREG_SIZE * 3] =
{
    0x5, 0x46, 0x4105,
    0x6, 0x48, 0x332A,
    0x6, 0x4E, 0x0508,
    0x7, 0x40, 0x0617,
    0x7, 0x42, 0x0501,
    0x7, 0x44, 0x0505,
    0x7, 0x46, 0x0000,
    0x7, 0x48, 0xFD00,
    0x7, 0x4A, 0x0404,
    0x7, 0x4C, 0x0004,
    0x7, 0x4E, 0xFDFF,
    0x8, 0x40, 0xFDF3,
    0x8, 0x42, 0xFDFD,
    0x8, 0x44, 0xEDED,
    0x8, 0x46, 0xDBE9,
    0x8, 0x48, 0xEEEE,
    0x8, 0x4A, 0xE4EE,
    0x8, 0x4C, 0xDEE4,
    0x8, 0x4E, 0x00D8
};

const uint16_t RT9426_USR_OCV_TABLE[80] =
{
    0x0013, 0x49C0, 0x4B00, 0x4FF3, 0x528D, 0x52ED, 0x54ED, 0x57A0,
    0x5A60, 0x5B53, 0x5C9A, 0x5DD3, 0x5EA0, 0x60C6, 0x6280, 0x63D3,
    0x64FA, 0x6620, 0x6880, 0x6A80, 0x0500, 0x04F3, 0x0200, 0x0065,
    0x0083, 0x0090, 0x0057, 0x0039, 0x0079, 0x00AA, 0x0075, 0x006D,
    0x006C, 0x008A, 0x008C, 0x0083, 0x00B0, 0x0200, 0x0013, 0xFF80,
    0x0000, 0x0200, 0x049A, 0x0680, 0x0E4D, 0x17E6, 0x281A, 0x309A,
    0x3600, 0x39B3, 0x3D33, 0x474D, 0x4F80, 0x5466, 0x589A, 0x5D1A,
    0x6400, 0x6600, 0x0033, 0x0034, 0x0080, 0x0289, 0x01F3, 0x01C7,
    0x02F2, 0x0479, 0x021E, 0x0183, 0x0230, 0x0259, 0x0260, 0x01D9,
    0x01D3, 0x01F5, 0x0174, 0x0080, 0x5A21, 0x0000, 0x0000, 0x0000
};

#define DELAY_1MS                   (1)
#define DELAY_5MS                   (5)
#define DELAY_10MS                  (10)
#define DELAY_20MS                  (20)
#define DELAY_60MS                  (60)
#define DELAY_250MS                 (250)

#define Delay_msecond(x) _os_delay(x)

#ifdef RT9426_DEBUG
    #define RTDBG(x)    { configPRINTF(x); }
#else
    #define RTDBG(...)
#endif

uint32_t g_basic_key = 0x12345678;
uint32_t g_advance_key = 0xFFFFFFFF;
uint16_t g_rt9426_update_info[16]={0};

typedef struct _rt9426_priv {
    mdev_t *dev;
    mdev_t *i2cbus;

    bool invalid_data;
    bool stop_polling;
    rt9426_notify notify;
    os_mutex_t reg_rw_mutex;
    os_semaphore_t event_sem;
    os_thread_t battmon;
} rt9426_priv_t;

static mdev_t mdev_rt9426;
static rt9426_priv_t rt9426_priv_data;
static os_thread_stack_define(rt9426_battmon_stack, 1024);

static inline void *mdev_to_priv(mdev_t *dev)
{
	return (dev)? (void *)(dev->private_data) : NULL;
}

static int rt9426_getReg(rt9426_priv_t *privdata,
                         uint8_t reg,
                         uint16_t *val)
{
    int ret;

    if (!privdata || !val) {
        return -WM_FAIL;
    }
    os_mutex_get(&privdata->reg_rw_mutex, OS_WAIT_FOREVER);
    ret = i2cbus_drv_read_word_data(privdata->i2cbus, val, reg, 1);
    os_mutex_put(&privdata->reg_rw_mutex);
    return (ret < 0)? -WM_FAIL : WM_SUCCESS;
}

static int rt9426_setReg(rt9426_priv_t *privdata,
                         uint8_t reg,
                         uint16_t val)
{
    int ret;

    if (!privdata) {
        return -WM_FAIL;
    }
    os_mutex_get(&privdata->reg_rw_mutex, OS_WAIT_FOREVER);
    ret = i2cbus_drv_write_word_data(privdata->i2cbus, &val, reg, 1);
    os_mutex_put(&privdata->reg_rw_mutex);
    return (ret < 0)? -WM_FAIL : WM_SUCCESS;
}

static int rt9426_initial(rt9426_priv_t *privdata)
{
    uint16_t loop = 0, retry_times = 0;
    uint16_t i, j;
    uint16_t regval = 0;
    uint16_t sts = 0;
    uint16_t count = 0;

    if (!privdata) {
        return -WM_FAIL;
    }
    /* ++ Unseal RT9426 ++ */
    while (1) {
        count++;

        if (rt9426_setReg(privdata,
                          REG_CNTL,
                          g_basic_key & 0xFFFF) == WM_SUCCESS) {
            Delay_msecond(DELAY_1MS);
            if (rt9426_setReg(privdata,
                              REG_CNTL,
                              g_basic_key >> 16) == WM_SUCCESS) {
                rt9426_setReg(privdata, REG_DUMMY, 0x0000);
                Delay_msecond(DELAY_1MS);

                rt9426_getReg(privdata, REG_FLAG3, &sts);
                if (sts & MASK_REG_FLAG3_BSC_ACS) {
                    RTDBG(("RT9426_Unseal_Pass\r\n"));
                    break;
                }
            }
        }

        if (count > 5) {
            RTDBG(("RT9426_Unseal_Fail\r\n"));
            return -WM_FAIL;
        }
        Delay_msecond(DELAY_10MS);
    }
    /* -- Unseal RT9426 -- */

    /* ++ Read Reg IRQ to reinital Alert pin state ++ */
    rt9426_getReg(privdata, REG_IRQ, &regval);
    /* -- Read Reg IRQ to reinital Alert pin state -- */

    /* ++ Set User Define OCV Table ++ */
    if (RT9426_USR_OCV_TABLE[0] == 0x0013) {
        retry_times = 3;
        while (retry_times) {
            for (i = 0; i < 9; i++) {
                rt9426_setReg(privdata, REG_BDCNTL, 0xCB00 + i);
                rt9426_setReg(privdata, REG_BDCNTL, 0xCB00 + i);
                Delay_msecond(DELAY_1MS);
                for (j = 0; j < 8; j++) {
                    rt9426_setReg(privdata,
                                  0x40 + (j * 2),
                                  RT9426_USR_OCV_TABLE[(i * 8) + j]);
                    Delay_msecond(DELAY_1MS);
                }
            }

            rt9426_setReg(privdata, REG_BDCNTL, 0xCB09);
            rt9426_setReg(privdata, REG_BDCNTL, 0xCB09);
            Delay_msecond(DELAY_1MS);
            for (i = 0; i < 5; i++) {
                rt9426_setReg(privdata,
                              0x40 + (i * 2),
                              RT9426_USR_OCV_TABLE[72 + i]);
                Delay_msecond(DELAY_1MS);
            }
            rt9426_setReg(privdata, REG_DUMMY, 0x0000);
            Delay_msecond(DELAY_10MS);
            rt9426_getReg(privdata, REG_FLAG2, &regval);
            if (regval & 0x0800) {
                RTDBG(("RT9426_OCV_Setting_Pass\r\n"));
                break;
            }
            retry_times--;

            if (retry_times == 0) {
                RTDBG(("RT9426_OCV_Setting_Fail\r\n"));
                return -1;
            }
        }
    }
    /* -- Set User Define OCV Table -- */

    /* ++ Set Alert Threshold ++ */
    rt9426_setReg(privdata, REG_BDCNTL, 0x6553);
    rt9426_setReg(privdata, REG_BDCNTL, 0x6553);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW1, RT9426_OTC_TTH);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW2, RT9426_OTC_CHG_ITH);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW3, RT9426_OTD_TTH);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW4, RT9426_OTD_DCHG_ITH);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW5, RT9426_UVOV_TH);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW6, ((0x4600) | (RT9426_US_TH)));
    rt9426_setReg(privdata, REG_DUMMY, 0x0000);
    Delay_msecond(DELAY_1MS);
    /* -- Set Alert Threshold -- */

    /* ++ Set Default OCV Table ++ */
    rt9426_setReg(privdata, REG_BDCNTL, 0x6552);
    rt9426_setReg(privdata, REG_BDCNTL, 0x6552);
    Delay_msecond(DELAY_1MS);
    if (RT9426_BATTERY_TYPE == 4400) {
        rt9426_setReg(privdata, REG_SWINDOW1, 0x8200);
    } else if (RT9426_BATTERY_TYPE == 4352) {
        rt9426_setReg(privdata, REG_SWINDOW1, 0x8100);
    } else if (RT9426_BATTERY_TYPE == 4354) {
        rt9426_setReg(privdata, REG_SWINDOW1, 0x8300);
    } else if (RT9426_BATTERY_TYPE == 4200) {
        rt9426_setReg(privdata, REG_SWINDOW1, 0x8000);
    } else {
        rt9426_setReg(privdata, REG_SWINDOW1, 0x8100);
    }
    rt9426_setReg(privdata, REG_DUMMY, 0x0000);
    Delay_msecond(DELAY_1MS);
    /* -- Set Default OCV Table -- */

    /* ++ Set OP CONFIG ++ */
    rt9426_setReg(privdata, REG_BDCNTL, 0x6551);
    rt9426_setReg(privdata, REG_BDCNTL, 0x6551);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW1, RT9426_OPCFG1);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW2, RT9426_OPCFG2);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW3, RT9426_OPCFG3);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW4, RT9426_OPCFG4);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW5, RT9426_OPCFG5);
    rt9426_setReg(privdata, REG_DUMMY, 0x0000);
    Delay_msecond(DELAY_1MS);
    /* -- Set OP CONFIG -- */

    /* ++ Set Current Deadband ++ */
    rt9426_setReg(privdata, REG_BDCNTL, 0x6551);
    rt9426_setReg(privdata, REG_BDCNTL, 0x6551);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata,
                  REG_SWINDOW7,
                  ((0x0012) | (RT9426_CURR_DB << 8)));
    rt9426_setReg(privdata, REG_DUMMY, 0x0000);
    Delay_msecond(DELAY_1MS);
    /* -- Set Current Deadband -- */

    /* ++ Set FC FD Threshold ++ */
    regval = (RT9426_FC_VTH) | (RT9426_FC_ITH << 8);
    rt9426_setReg(privdata, REG_BDCNTL, 0x6555);
    rt9426_setReg(privdata, REG_BDCNTL, 0x6555);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW3, regval);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW4, (0x4100 | RT9426_FC_STH));
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW6, (0x1200 | RT9426_FD_VTH));
    rt9426_setReg(privdata, REG_DUMMY, 0x0000);
    Delay_msecond(DELAY_1MS);
    /* -- Set FC FD Threshold -- */

    /* ++ Set DC & FCC ++ */
    rt9426_setReg(privdata, REG_BDCNTL, 0x6552);
    rt9426_setReg(privdata, REG_BDCNTL, 0x6552);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW6, RT9426_DC);
    Delay_msecond(DELAY_1MS);
    rt9426_setReg(privdata, REG_SWINDOW7, RT9426_FCC);
    rt9426_setReg(privdata, REG_DUMMY, 0x0000);
    Delay_msecond(DELAY_1MS);

    /* -- Set DC & FCC -- */

    /* ++ Set Ext Register ++ */
    if (RT9426_EXTREG_SIZE != 0) {
        for (loop = 0; loop < RT9426_EXTREG_SIZE; loop++) {
            rt9426_setReg(privdata,
                          REG_BDCNTL,
                          0x6550 + (RT9426_EXTREG_DATA[loop * 3]));
            rt9426_setReg(privdata,
                          REG_BDCNTL,
                          0x6550 + (RT9426_EXTREG_DATA[loop * 3]));
            Delay_msecond(DELAY_1MS);
            rt9426_setReg(privdata,
                          RT9426_EXTREG_DATA[(loop * 3) + 1],
                          RT9426_EXTREG_DATA[(loop * 3) + 2]);
            Delay_msecond(DELAY_1MS);
        }
        rt9426_setReg(privdata, REG_DUMMY, 0x0000);
    }
    /* -- Set Ext Register -- */

    /* ++ Quick Start ++ */
    if (RT9426_QS_EN == 1) {
        rt9426_setReg(privdata, REG_CNTL, 0x4000);
        rt9426_setReg(privdata, REG_DUMMY, 0x0000);
        Delay_msecond(DELAY_5MS);
    }
    /* -- Quick Start -- */

    /* ++ Clear RI ++ */
    rt9426_getReg(privdata, REG_FLAG3, &regval);
    regval = (regval & ~0x0100);
    rt9426_setReg(privdata, REG_FLAG3, regval);
    rt9426_setReg(privdata, REG_DUMMY, 0x0000);
    Delay_msecond(DELAY_1MS);
    /* -- Clear RI -- */

    /* ++ Seal RT9426 ++ */
    rt9426_setReg(privdata, REG_CNTL, 0x0020);
    rt9426_setReg(privdata, REG_DUMMY, 0x0000);
    Delay_msecond(DELAY_5MS);
    /* -- Seal RT9426 -- */
    return WM_SUCCESS;
}

static int rt9426_exitShutdown(rt9426_priv_t *privdata)
{
    uint8_t SHDN_count = 0;
    uint16_t sts = 0;

    if (!privdata) {
        return -WM_FAIL;
    }
    rt9426_getReg(privdata, REG_FLAG2, &sts);
    if (sts & MASK_REG_FLAG2_SHDN_STS) {
        while (1) {
            SHDN_count++;
            rt9426_setReg(privdata, REG_CNTL, 0x6400);
            rt9426_setReg(privdata, REG_DUMMY, 0x0000);
            Delay_msecond(DELAY_5MS);
            rt9426_getReg(privdata, REG_FLAG2, &sts);
            if ((sts&MASK_REG_FLAG2_SHDN_STS) == 0) {
                Delay_msecond(DELAY_250MS);
                rt9426_setReg(privdata, REG_CNTL, 0x4000);
                rt9426_setReg(privdata, REG_DUMMY, 0x0000);
                Delay_msecond(DELAY_5MS);
                RTDBG(("RT9426_Exit_SHDN_Pass\r\n"));
                break;
            }
            if (SHDN_count > 3) {
                RTDBG(("RT9426_Exit_SHDN_error\r\n"));
                return -WM_FAIL;
            }
        }
    }
    return WM_SUCCESS;
}

static int rt9426_initFlow(rt9426_priv_t *privdata)
{
    uint16_t regval = 0;
    uint16_t count = 0;
    int sts = 0;

    if (!privdata) {
        return -WM_FAIL;
    }
    /* ++ Standard initial ++ */
    while (1) {
        count++;
         /* check RDY */
        rt9426_getReg(privdata, REG_FLAG2, &regval);
        if (regval & 0x0080) {
            Delay_msecond(DELAY_60MS);
            /* check RI */
            rt9426_getReg(privdata, REG_FLAG3, &regval);
            if (regval & 0x0100) {
                sts = rt9426_initial(privdata);
                if (sts == 0) {
                    RTDBG(("RT9426_init_successful\r\n"));
                } else {
                    RTDBG(("RT9426_init_fail\r\n"));
                    return -WM_FAIL;
                }
                /* Record SW VERSION to RSVD Register */
                rt9426_setReg(privdata, REG_RSVD, RT9426_SWVER);
                rt9426_setReg(privdata, REG_DUMMY, 0x0000);
                Delay_msecond(DELAY_5MS);
                break;
            } else {
                RTDBG(("No_Need_to_reinitil_RT9426\r\n"));
                break;
            }
        }
        if (count > 20) {
            RTDBG(("RT9426_init_error\r\n"));
            return -WM_FAIL;
        }
        Delay_msecond(DELAY_20MS);
    }
    /* -- Standard initial -- */

    count = 0;

    /* ++ Force initial ++ */
    /* Compare Present SWVER & Previous SWVER */
    rt9426_getReg(privdata, REG_RSVD, &regval);
    if (regval != (RT9426_SWVER)) {

        /* Depends on differnet SW update condition to
         * Reset RT9426 or not */
        while (1) {
            count++;
            /* check RDY */
            rt9426_getReg(privdata, REG_FLAG2, &regval);
            if (regval & 0x0080) {
                Delay_msecond(DELAY_60MS);
                sts = rt9426_initial(privdata);
                if (sts == 0) {
                    RTDBG(("RT9426_init_successful\r\n"));
                } else {
                    RTDBG(("RT9426_init_fail\r\n"));
                    return -WM_FAIL;
                }
                /* Record SW VERSION to RSVD Register */
                rt9426_setReg(privdata, REG_RSVD, RT9426_SWVER);
                rt9426_setReg(privdata, REG_DUMMY, 0x0000);
                Delay_msecond(DELAY_5MS);
                break;
            }
            if (count > 20) {
                RTDBG(("RT9426_init_error\r\n"));
                return -WM_FAIL;
            }
            Delay_msecond(DELAY_20MS);
        }
    }
    /* -- Force initial -- */
    return WM_SUCCESS;
}

static int rt9426_updateInfo(rt9426_priv_t *privdata)
{
    uint8_t loop = 0;
    uint16_t regval = 0;

    if (!privdata) {
        return -WM_FAIL;
    }
    /* check flag3, if RI=1, reinitial IC */
    rt9426_getReg(privdata, REG_FLAG3, &regval);
    RTDBG(("RT9426_REG_FLAG3 = %04x\r\n", regval));
    if (regval & 0x0100) {
        Delay_msecond(DELAY_5MS);
        rt9426_getReg(privdata, REG_FLAG3, &regval);
        /* check flag3, if RI=1, reinitial IC */
        RTDBG(("RT9426_REG_FLAG3 = %04x\r\n", regval));
        if (regval & 0x0100) {
            rt9426_initFlow(privdata);
        }
    }

    for (loop = 0; loop < 16; loop++) {
        g_rt9426_update_info[loop] = 0;
    }

    g_rt9426_update_info[0] = 0;
    rt9426_getReg(privdata, REG_VBAT, &g_rt9426_update_info[1]);
    rt9426_getReg(privdata, REG_CURR, &g_rt9426_update_info[2]);
    rt9426_getReg(privdata, REG_TEMP, &g_rt9426_update_info[3]);
    rt9426_getReg(privdata, REG_INTT, &g_rt9426_update_info[4]);
    rt9426_getReg(privdata, REG_SOC, &g_rt9426_update_info[5]);
    rt9426_getReg(privdata, REG_RM, &g_rt9426_update_info[6]);
    rt9426_getReg(privdata, REG_SOH, &g_rt9426_update_info[7]);
    rt9426_getReg(privdata, REG_DC, &g_rt9426_update_info[8]);
    rt9426_getReg(privdata, REG_FCC, &g_rt9426_update_info[9]);
    rt9426_getReg(privdata, REG_FLAG1, &g_rt9426_update_info[10]);
    rt9426_getReg(privdata, REG_FLAG2, &g_rt9426_update_info[11]);
    rt9426_getReg(privdata, REG_FLAG3, &g_rt9426_update_info[12]);
    rt9426_getReg(privdata, REG_IRQ, &g_rt9426_update_info[13]);

    RTDBG(("CURR: %d\r\n", (int16_t)g_rt9426_update_info[2]));
    RTDBG(("SOC: %u%%\r\n", g_rt9426_update_info[5]));

    privdata->invalid_data = false;
    return WM_SUCCESS;
}

int rt9426_getBatteryInfo(mdev_t *dev, battery_info_t *info)
{
    rt9426_priv_t *privdata = mdev_to_priv(dev);
    int ret;

    if (!privdata || !info) {
        return -WM_FAIL;
    }
    ret = rt9426_updateInfo(privdata);

    if (ret == WM_SUCCESS) {
        info->level = (int)g_rt9426_update_info[5];
        info->current = (int)((int16_t)g_rt9426_update_info[2]);
        info->charging = (info->current > 0)? true : false;
        info->voltage = (int)g_rt9426_update_info[1];
        info->temperature = ((float)g_rt9426_update_info[3] / 10.0)
                            - 273.15;
        info->health = (int)g_rt9426_update_info[7];
        info->remaing_capacity = (int)g_rt9426_update_info[6];
        info->full_capacity = (int)g_rt9426_update_info[9];
        info->design_capacity = (int)g_rt9426_update_info[8];
    }

    return ret;
}

static void rt9426_batteryMonitor(os_thread_arg_t data)
{
    rt9426_priv_t *privdata = data;
    bool prev_charging = false, charging = false;
    int prev_level = 0, level = 0, ret;
    int16_t current = 0;

    while (1) {
        if (!privdata->invalid_data) {
            os_semaphore_get(&privdata->event_sem,
                             os_msec_to_ticks(RT9426_POLLING_INTERVAL));
        }
        if (privdata->stop_polling) {
            break;
        }
        ret = rt9426_updateInfo(privdata);
        if (ret == WM_SUCCESS && privdata->notify) {
            current = (int16_t)g_rt9426_update_info[2];
            level = (int)g_rt9426_update_info[5];
            charging = (current > 0)? true : false;

            if ((charging != prev_charging) || (level != prev_level)) {
                privdata->notify(level, charging);
                prev_charging = charging;
                prev_level = level;
            }
        }
    }
    os_thread_self_complete(NULL);
}

static int rt9426_drv_setup(I2C_ID_Type id, uint16_t addr)
{
    mdev_t *dev = &mdev_rt9426;
    rt9426_priv_t *privdata = &rt9426_priv_data;

    /* init mdev node */
    dev->port_id = 0;
    dev->name = MDEV_RT9426;
    dev->pNextMdev = NULL;
    dev->private_data = (uint32_t)privdata;

    privdata->dev = dev;
    privdata->stop_polling = false;
    privdata->invalid_data = true;

    if (os_mutex_create(&privdata->reg_rw_mutex, "rt9426_register",
                        OS_MUTEX_INHERIT) < 0) {
        goto err_setup;
    }
    if (i2cbus_drv_init(id) < 0) {
        goto err_setup;
    }
    privdata->i2cbus = i2cbus_drv_open(id, addr, 0);
    if (!privdata->i2cbus) {
        goto err_setup;
    }
    return WM_SUCCESS;

err_setup:
    if (privdata->i2cbus) {
        i2cbus_drv_close(privdata->i2cbus);
        privdata->i2cbus = NULL;
    }

    if (privdata->reg_rw_mutex) {
        os_mutex_delete(&privdata->reg_rw_mutex);
        privdata->reg_rw_mutex = NULL;
    }
    return -WM_FAIL;
}

mdev_t *rt9426_drv_open(rt9426_notify notify)
{
    int ret;
    mdev_t *dev = mdev_get_handle(MDEV_RT9426);
    rt9426_priv_t *privdata = mdev_to_priv(dev);

    if (privdata == NULL) {
        return NULL;
    }

    if (notify) {
        privdata->notify = notify;
        ret = os_semaphore_create(&privdata->event_sem, "rt9426_event");
        if (ret != WM_SUCCESS) {
            goto err_open;
        }
        os_semaphore_get(&privdata->event_sem, OS_WAIT_FOREVER);

        ret = os_thread_create(&privdata->battmon,
                               "rt9426_monitor",
                               rt9426_batteryMonitor,
                               privdata,
                               &rt9426_battmon_stack,
                               OS_PRIO_2);
        if (ret < 0) {
            goto err_open;
        }
    }
    return dev;

err_open:
    privdata->notify = NULL;

    if (privdata->event_sem) {
        os_semaphore_delete(&privdata->event_sem);
        privdata->event_sem = NULL;
    }

    if (privdata->battmon) {
        os_thread_delete(&privdata->battmon);
        privdata->battmon = NULL;
    }
    return NULL;
}

int rt9426_drv_close(mdev_t *dev)
{
    rt9426_priv_t *privdata = mdev_to_priv(dev);

    if (!privdata) {
        return -WM_FAIL;
    }

    privdata->stop_polling = true;

    if (privdata->battmon) {
        os_semaphore_put(&privdata->event_sem);
        os_thread_delete(&privdata->battmon);
        privdata->battmon = NULL;
    }
    if (privdata->event_sem) {
        os_semaphore_delete(&privdata->event_sem);
        privdata->event_sem = NULL;
    }

    return WM_SUCCESS;
}

int rt9426_drv_init()
{
    if (mdev_get_handle(MDEV_RT9426) != NULL) {
        return WM_SUCCESS;
    }
    if (rt9426_drv_setup(board_peripheral_i2c_id(),
                         RT9426_I2C_ADDR) != WM_SUCCESS) {
        return -WM_FAIL;
    }
    return mdev_register(&mdev_rt9426);
}

int rt9426_drv_deinit()
{
    rt9426_priv_t *privdata = &rt9426_priv_data;

    if (!mdev_get_handle(MDEV_RT9426)) {
        return WM_SUCCESS;
    }

    if (privdata->i2cbus) {
        i2cbus_drv_close(privdata->i2cbus);
        privdata->i2cbus = NULL;
    }

    if (privdata->reg_rw_mutex) {
        os_mutex_delete(&privdata->reg_rw_mutex);
        privdata->reg_rw_mutex = NULL;
    }
    mdev_deregister(MDEV_RT9426);
    return WM_SUCCESS;
}
