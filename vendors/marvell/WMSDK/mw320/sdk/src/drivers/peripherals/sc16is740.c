/** @file sc16is740.c
 *
 *  @brief This file provides driver for SC16IS740i2c to uart converter
 *
 *  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
 *
 */

#include <wmstdio.h>
#include <stdbool.h>
#include <mdev_gpio.h>
#include <mdev_i2cbus.h>
#include <sc16is740.h>
#include <ringbytebuffer.h>
#include <wm_os.h>
#include <lowlevel_drivers.h>

typedef struct _sc16is740_priv {
    mdev_t *dev;
    mdev_t *i2cbus;
    mdev_t *gpiod;

    int irq;
    uint8_t exit;

    int oscillator;
    int rx_bufsize;
    bool read_blocking;
    uint32_t rx_timout;
    uint32_t tx_timout;
    sc_UartFormat_t format;
    sc_FifoCfg_t fifo;
    ringbytebuffer_t *rbuf;

    os_mutex_t reg_rw_mutex;
    os_semaphore_t intr_event_sem;
    os_semaphore_t read_blocking_sem;
    os_thread_t irq_thread;
} sc16is740_priv_t;

static os_thread_stack_define(sc16is740_thread_stack, 1024);

static mdev_t mdev_sc16is740;
static sc16is740_priv_t sc16is740_priv_data;
static os_mutex_t sc16is740_mutex;

static inline void *mdev_to_priv(mdev_t *dev)
{
    return (dev)? (void *)(dev->private_data) : NULL;
}

static int sc16is740_getReg(sc16is740_priv_t *privdata,
                            uint8_t reg,
                            uint8_t *val)
{
    int ret;
    uint8_t sc_reg;

    if (!privdata || !val) {
        return -WM_FAIL;
    }

    os_mutex_get(&privdata->reg_rw_mutex, OS_WAIT_FOREVER);
    sc_reg = reg << 3;
    ret = i2cbus_drv_read_byte_data(privdata->i2cbus, val, sc_reg, 1);
    os_mutex_put(&privdata->reg_rw_mutex);

    return (ret < 0)? -WM_FAIL : WM_SUCCESS;
}

static int sc16is740_setReg(sc16is740_priv_t *privdata,
                            uint8_t reg,
                            uint8_t val)
{
    int ret;
    uint8_t sc_reg;

    if (!privdata) {
        return -WM_FAIL;
    }

    os_mutex_get(&privdata->reg_rw_mutex, OS_WAIT_FOREVER);
    sc_reg = reg << 3;
    ret = i2cbus_drv_write_byte_data(privdata->i2cbus, &val, sc_reg, 1);
    os_mutex_put(&privdata->reg_rw_mutex);

    return (ret < 0)? -WM_FAIL : WM_SUCCESS;
}

static int sc16is740_updateReg(sc16is740_priv_t *privdata,
                               uint8_t reg,
                               uint8_t val,
                               uint8_t mask)
{
    int ret;
    uint8_t data = 0, sc_reg;

    if (!privdata) {
        return -WM_FAIL;
    }

    os_mutex_get(&privdata->reg_rw_mutex, OS_WAIT_FOREVER);
    sc_reg = reg << 3;
    ret = i2cbus_drv_read_byte_data(privdata->i2cbus, &data, sc_reg, 1);
    if (ret > 0) {
        data &= ~mask;
        data |= (val & mask);
        ret = i2cbus_drv_write_byte_data(privdata->i2cbus,
                                         &data,
                                         sc_reg,
                                         1);
    }
    os_mutex_put(&privdata->reg_rw_mutex);

    return (ret < 0)? -WM_FAIL : WM_SUCCESS;
}

static int sc16is740_setTxFifoLevel(sc16is740_priv_t *privdata,
                                    sc_TxFIFOLevel_t level)
{
    uint8_t reg;

    if (level > 3) {
        return -WM_FAIL;
    }
    reg = level << SC16IS740_FCR_TXLV_OFF;
    return sc16is740_updateReg(privdata,
                               SC16IS740_REG_FCR,
                               reg,
                               SC16IS740_FCR_TXLV_MSK);
}

static int sc16is740_setRxFifoLevel(sc16is740_priv_t *privdata,
                                    sc_TxFIFOLevel_t level)
{
    uint8_t reg;

    if (level > 3) {
        return -WM_FAIL;
    }
    reg = level << SC16IS740_FCR_RXLV_OFF;
    return sc16is740_updateReg(privdata,
                               SC16IS740_REG_FCR,
                               reg,
                               SC16IS740_FCR_RXLV_MSK);
}

static int sc16is740_setFlowctrl(sc16is740_priv_t *privdata,
                                 sc_FlowCtrl_t mode)
{
    int ret;
    uint8_t lcr = 0, flow = 0, mask;

    if (!privdata) {
        return -WM_FAIL;
    }

    ret = sc16is740_getReg(privdata, SC16IS740_REG_LCR, &lcr);
    if (ret < 0) {
        return -WM_FAIL;
    }

    sc16is740_setReg(privdata,
                     SC16IS740_REG_LCR,
                     SC16IS740_LCR_ENHANCED_MODE);
    sc16is740_setReg(privdata, SC16IS740_REG_XON1, XON_CHAR);
    sc16is740_setReg(privdata, SC16IS740_REG_XOFF1, XOFF_CHAR);

    if (mode & SC_FLOWCTRL_RTSCTS) {
        flow |= SC16IS740_FER_CTS_BIT | SC16IS740_FER_RTS_BIT;
    }
    if (mode & SC_FLOWCTRL_IXON) {
        flow |= SC16IS740_EFR_SWFLOW3_BIT;
    }
    if (mode & SC_FLOWCTRL_IXOFF) {
        flow |= SC16IS740_EFR_SWFLOW1_BIT;
    }

    mask = SC16IS740_FER_CTS_BIT | SC16IS740_FER_RTS_BIT |
           SC16IS740_FER_SCHD_BIT | SC16IS740_FER_FLOW_MASK;
    sc16is740_updateReg(privdata, SC16IS740_REG_EFR, flow, mask);
    sc16is740_setReg(privdata, SC16IS740_REG_LCR, lcr);
    privdata->format.flowctrl = mode;

    return WM_SUCCESS;
}

static int sc16is740_setBaud(sc16is740_priv_t *privdata, sc_Baud_t baud)
{
    int ret, div;
    uint8_t lcr = 0, prescaler = 0;

    if (!privdata) {
        return -WM_FAIL;
    }

    div = privdata->oscillator / (baud * 16);
    if (div > 0xFFFF) {
        div /= 4;
        prescaler = SC16IS740_MCR_CLKD_BIT;
    }
    if (div == 0) {
        return -WM_FAIL;
    }

    ret = sc16is740_getReg(privdata, SC16IS740_REG_LCR, &lcr);
    if (ret < 0) {
        return -WM_FAIL;
    }
    sc16is740_setReg(privdata,
                     SC16IS740_REG_LCR,
                     SC16IS740_LCR_ENHANCED_MODE);
    sc16is740_updateReg(privdata,
                        SC16IS740_REG_EFR,
                        SC16IS740_EFR_ENABLE_BIT,
                        SC16IS740_EFR_ENABLE_BIT);
    sc16is740_setReg(privdata, SC16IS740_REG_LCR, lcr);
    sc16is740_updateReg(privdata,
                        SC16IS740_REG_MCR,
                        prescaler,
                        SC16IS740_MCR_CLKD_BIT);
    sc16is740_setReg(privdata,
                     SC16IS740_REG_LCR,
                     SC16IS740_LCR_DIVLE_BIT);
    sc16is740_setReg(privdata, SC16IS740_REG_DLL, div % 256);
    sc16is740_setReg(privdata, SC16IS740_REG_DLH, div / 256);
    sc16is740_setReg(privdata, SC16IS740_REG_LCR, lcr);
    privdata->format.baud = baud;
    if (prescaler > 0) {
        ret = privdata->oscillator / (16 * 4 * div);
    } else {
        ret = privdata->oscillator / (16 * div);
    }
    return ret;
}

static int sc16is740_updateUartParam(sc16is740_priv_t *privdata,
                                     sc_UartFormat_t *format)
{
    uint8_t lcr = 0, mask;

    if (!privdata) {
        return -WM_FAIL;
    }

    /* Word size */
    privdata->format.databits = format->databits;
    switch (format->databits) {
        case SC_DATABITS_5:
            lcr = SC16IS740_LCR_WORD_LEN_5;
        break;
        case SC_DATABITS_6:
            lcr = SC16IS740_LCR_WORD_LEN_6;
        break;
        case SC_DATABITS_7:
            lcr = SC16IS740_LCR_WORD_LEN_7;
        break;
        case SC_DATABITS_8:
        default:
            lcr = SC16IS740_LCR_WORD_LEN_8;
            privdata->format.databits = SC_DATABITS_8;
        break;
    }

    /* Parity */
    privdata->format.parity = format->parity;
    if (format->parity == 1) {
        lcr |= SC16IS740_LCR_PARITY_BIT; /* odd parity */
    } else if (format->parity == 2) {
        lcr |= SC16IS740_LCR_PARITYEVEN_BIT; /* even parity */
    } else {
        privdata->format.parity = SC_PARITY_NONE;
    }

    /* Stop bits */
    privdata->format.stopbits = format->stopbits;
    if (format->stopbits == SC_STOPBITS_2) {
        lcr |= SC16IS740_LCR_STOP_BIT; /* 2 stops */
    } else {
        privdata->format.stopbits = SC_STOPBITS_1;
    }
    mask = SC16IS740_LCR_PARITYSET_BIT |
           SC16IS740_LCR_PARITYEVEN_BIT |
           SC16IS740_LCR_PARITY_BIT |
           SC16IS740_LCR_STOP_BIT |
           SC16IS740_LCR_WORD_LEN_MSK;
    sc16is740_updateReg(privdata, SC16IS740_REG_LCR, lcr, mask);

    sc16is740_setBaud(privdata, format->baud);
    sc16is740_setFlowctrl(privdata, format->flowctrl);

    return WM_SUCCESS;
}

int sc16is740_drv_set_opts(sc_DataBits_t databits,
                           sc_Parity_t parity,
                           sc_StopBits_t stopbits,
                           sc_FlowCtrl_t flow_control)
{
    sc16is740_priv_t *privdata;
    mdev_t *dev = mdev_get_handle(MDEV_SC16IS740);

    privdata = mdev_to_priv(dev);
    if (!privdata) {
        return -WM_FAIL;
    }

    if (os_mutex_get(&sc16is740_mutex, OS_NO_WAIT) < 0) {
        return -WM_FAIL;
    }
    privdata->format.databits = databits;
    privdata->format.parity = parity;
    privdata->format.stopbits = stopbits;
    privdata->format.flowctrl = flow_control;
    os_mutex_put(&sc16is740_mutex);
    return WM_SUCCESS;
}

static int sc16is740_resetFifo(sc16is740_priv_t *privdata)
{
    uint8_t reg;
    int ret;

    reg = SC16IS740_FCR_RXRESET_BIT | SC16IS740_FCR_TXRESET_BIT;
    ret = sc16is740_setReg(privdata, SC16IS740_REG_FCR, reg);
    if (ret < 0) {
        return -WM_FAIL;
    }
    _os_delay(1);
    return ret;
}

static int sc16is740_enableFifo(sc16is740_priv_t *privdata, bool enable)
{
    uint8_t reg;

    reg = (enable)? SC16IS740_FCR_FIFO_EN_BIT : 0;
    return sc16is740_updateReg(privdata,
                               SC16IS740_REG_FCR,
                               reg,
                               SC16IS740_FCR_FIFO_EN_BIT);
}

static int sc16is740_power(sc16is740_priv_t *privdata, int on)
{
    uint8_t reg;

    reg = (on)? 0 : SC16IS740_IER_SLEEP_BIT;
    return sc16is740_updateReg(privdata,
                               SC16IS740_REG_IER,
                               reg,
                               SC16IS740_IER_SLEEP_BIT);
}

static void sc16is740_irq_handler(int pin, void *data)
{
    sc16is740_priv_t *privdata = data;

    if (privdata && privdata->intr_event_sem) {
        os_semaphore_put(&privdata->intr_event_sem);
    }
    return;
}

static int sc16is740_write_fifo(sc16is740_priv_t *privdata,
                                uint8_t *buf,
                                int len)
{
    int ret;

    if (!privdata || !buf) {
        return -WM_FAIL;
    }
    os_mutex_get(&privdata->reg_rw_mutex, OS_WAIT_FOREVER);
    ret = i2cbus_drv_write_byte_data(privdata->i2cbus,
                                     buf,
                                     SC16IS740_REG_THR << 3,
                                     len);
    os_mutex_put(&privdata->reg_rw_mutex);
    return ret;
}

static int sc16is740_read_fifo(sc16is740_priv_t *privdata,
                               uint8_t *buf,
                               int len)
{
    int ret;

    if (!privdata) {
        return -WM_FAIL;
    }
    os_mutex_get(&privdata->reg_rw_mutex, OS_WAIT_FOREVER);
    ret = i2cbus_drv_read_byte_data(privdata->i2cbus,
                                    buf,
                                    SC16IS740_REG_RHR << 3,
                                    len);
    os_mutex_put(&privdata->reg_rw_mutex);
    return ret;
}

static int sc16is740_handle_rx(sc16is740_priv_t *privdata,
                               uint8_t intr,
                               uint8_t rxlen)
{
    uint8_t buf[SC_FIFO_DEPTH], lsr = 0;
    bool err = (intr == SC16IS740_IIR_RLSE)? true : false;
    int reads = 0, ret;

    while (rxlen > 0) {
        if (err) {
            sc16is740_getReg(privdata, SC16IS740_REG_LSR, &lsr);
            if (!(lsr & SC16IS740_LSR_FIFOE_BIT)) {
                err = 0;
            }
        } else {
            lsr = 0;
        }
        reads = sc16is740_read_fifo(privdata, buf, (err)? 1 : rxlen);
        if (reads < 0) {
            return -WM_FAIL;
        }
        lsr &= SC16IS740_LSR_BRK_ERROR_MASK;
        if (lsr) {
            if (lsr & SC16IS740_LSR_BRKI_BIT) { /* break interrupt */
                /* TODO */
                continue;
            }
            if (lsr & SC16IS740_LSR_FRME_BIT) { /* framing error */
                /* TODO */
                continue;
            }
            if (lsr & SC16IS740_LSR_PARE_BIT) { /* parity error */
                /* TODO */
                continue;
            }
            if (lsr & SC16IS740_LSR_OVRE_BIT) { /* overrun error */
                /* TODO */
                continue;
            }
        }
        ret = ringbytebuffer_puts(privdata->rbuf, buf, reads);
        if (ret != reads) {
            /* TODO: overrun */
        }
        rxlen -= reads;
    }
    os_semaphore_put(&privdata->read_blocking_sem);

    return 0;
}

static void sc16is740_irq_thread(os_thread_arg_t data)
{
    uint8_t iir = 0, intr = 0, rxlen = 0;
    sc16is740_priv_t *privdata = data;

    do {
        os_semaphore_get(&privdata->intr_event_sem, OS_WAIT_FOREVER);
        if (privdata->exit) {
            break;
        }
        sc16is740_getReg(privdata, SC16IS740_REG_IIR, &iir);
        if (iir & SC16IS740_IIR_STATUS_BIT) {
            /* no interrupt is pending */
            continue;
        }
        intr = iir & SC16IS740_IIR_INTS_MASK;
        switch (intr)
        {
            case SC16IS740_IIR_RLSE:
            case SC16IS740_IIR_RTOI:
            case SC16IS740_IIR_RHRI:
            case SC16IS740_IIR_XOFF:
                sc16is740_getReg(privdata, SC16IS740_REG_RXLVL, &rxlen);
                if (rxlen) {
                    sc16is740_handle_rx(privdata, intr, rxlen);
                }
            break;
            case SC16IS740_IIR_THRI:
            /* THR interrupt */
            break;
            case SC16IS740_IIR_CTSRTS:
            /* CTS, RTS change of state from active to inactive */
            break;
        }
    } while (1);
    os_thread_self_complete(NULL);
}

int sc16is740_drv_timeout(uint32_t tx_timeout, uint32_t rx_timeout)
{
    sc16is740_priv_t *privdata;
    mdev_t *dev = mdev_get_handle(MDEV_SC16IS740);

    privdata = mdev_to_priv(dev);
    if (!privdata) {
        return -WM_FAIL;
    }

    if (os_mutex_get(&sc16is740_mutex, OS_NO_WAIT) < 0) {
        return -WM_FAIL;
    }
    privdata->rx_timout = rx_timeout;
    privdata->tx_timout = tx_timeout;
    os_mutex_put(&sc16is740_mutex);
    return WM_SUCCESS;
}

int sc16is740_drv_blocking_read(bool is_blocking)
{
    sc16is740_priv_t *privdata;
    mdev_t *dev = mdev_get_handle(MDEV_SC16IS740);

    privdata = mdev_to_priv(dev);
    if (!privdata) {
        return -WM_FAIL;
    }

    if (os_mutex_get(&sc16is740_mutex, OS_NO_WAIT) < 0) {
        return -WM_FAIL;
    }
    privdata->read_blocking = is_blocking;
    os_mutex_put(&sc16is740_mutex);
    return WM_SUCCESS;
}

int sc16is740_drv_rx_buf_reset(mdev_t *dev)
{
    sc16is740_priv_t *privdata = mdev_to_priv(dev);

    if (!privdata) {
        return -WM_FAIL;
    }

    ringbytebuffer_reset(privdata->rbuf);
    return WM_SUCCESS;
}

int sc16is740_drv_rx_available(mdev_t *dev)
{
    sc16is740_priv_t *privdata = mdev_to_priv(dev);

    if (!privdata) {
        return 0;
    }

    return ringbytebuffer_available(privdata->rbuf);
}

int sc16is740_drv_rxbuf_size(uint32_t size)
{
    sc16is740_priv_t *privdata;
    mdev_t *dev = mdev_get_handle(MDEV_SC16IS740);

    privdata = mdev_to_priv(dev);
    if (!privdata || size < SC_BUF_SIZE) {
        return -WM_FAIL;
    }

    if (os_mutex_get(&sc16is740_mutex, OS_NO_WAIT) < 0) {
        return -WM_FAIL;
    }
    privdata->rx_bufsize = size;
    os_mutex_put(&sc16is740_mutex);
    return WM_SUCCESS;
}

int sc16is740_drv_read(mdev_t *dev, void* buf, int size)
{
    int ret = 0, n, reads = 0;
    sc16is740_priv_t *privdata = mdev_to_priv(dev);
    uint64_t t1 = 0, t2 = 0;
    uint32_t timeout;

    if (!privdata) {
        return -WM_FAIL;
    }
    timeout = os_msec_to_ticks(privdata->rx_timout);

    if (privdata->read_blocking) {
        reads = 0;
        t1 = os_total_ticks_get_new();
        while (reads < size) {
            n = ringbytebuffer_available(privdata->rbuf);
            if (n == 0) {
                os_semaphore_get(&privdata->read_blocking_sem,
                                 pdMS_TO_TICKS(privdata->rx_timout));
                n = ringbytebuffer_available(privdata->rbuf);
            }
            reads += ringbytebuffer_gets(privdata->rbuf,
                                         buf + reads,
                                         ((size - reads) > n)? n :
                                         (size - reads));
            t2 = os_total_ticks_get_new();
            if ((uint32_t)(t2 - t1) > timeout) {
                break;
            }
            ret = reads;
        }
    } else {
        n = ringbytebuffer_available(privdata->rbuf);
        if (n > 0) {
            ret = ringbytebuffer_gets(privdata->rbuf,
                                      buf,
                                      (size > n)? n : size);
        }
    }
    return ret;
}

int sc16is740_drv_write(mdev_t *dev, void* buf, int size)
{
    int len, writes = size;
    uint8_t *p, txlen = 0;
    sc16is740_priv_t *privdata = mdev_to_priv(dev);

    if (!privdata) {
        return -WM_FAIL;
    }

    p = buf;
    while (writes > 0) {
        if (sc16is740_getReg(privdata,
                             SC16IS740_REG_TXLVL,
                             &txlen) < 0)
        {
            return -WM_FAIL;
        }
        len = txlen;
        if (writes < len) {
            len = writes;
        }
        sc16is740_write_fifo(privdata, p, len);
        p += len;
        writes -= len;
    }
    return size;
}

static int sc16is740_getSupportBaud(uint32_t baud)
{
    int i;
    uint32_t supported[] =
    {
        SC_BAUD_110,
        SC_BAUD_300,
        SC_BAUD_600,
        SC_BAUD_2400,
        SC_BAUD_4800,
        SC_BAUD_9600,
        SC_BAUD_14400,
        SC_BAUD_19200,
        SC_BAUD_38400,
        SC_BAUD_57600
    };

    if (baud == 0) {
        /* use default baudrate */
        return (int)DEFAULT_BAUD;
    }

    for (i = 0; i < sizeof(supported); i++) {
        if (baud == supported[i]) {
            return (int)supported[i];
        }
    }
    return -WM_FAIL;
}

mdev_t *sc16is740_drv_open(uint32_t baud)
{
    int ret;
    uint8_t lcr = 0, ier = 0;
    sc16is740_priv_t *privdata;
    mdev_t *dev = mdev_get_handle(MDEV_SC16IS740);

    privdata = mdev_to_priv(dev);
    if (!privdata) {
        return NULL;
    }
    ret = os_mutex_get(&sc16is740_mutex, OS_WAIT_FOREVER);
    if (ret < 0) {
        return NULL;
    }

    ret = sc16is740_getSupportBaud(baud);
    if (ret < 0) {
        wm_printf("unsupported baudrate %d\r\n", baud);
    }
    baud = ret;

    privdata->rbuf = ringbytebuffer_create(privdata->rx_bufsize);
    if (!privdata->rbuf) {
        goto err_open;
    }

    /* interrupt event semaphore */
    if (os_semaphore_create(&privdata->intr_event_sem,
                            "scIntrEvent") < 0) {
        goto err_open;
    }
    os_semaphore_get(&privdata->intr_event_sem, OS_WAIT_FOREVER);

    /* uart read blocking semaphore */
    if (os_semaphore_create(&privdata->read_blocking_sem,
                            "scReadBlocking") < 0) {
        goto err_open;
    }
    os_semaphore_get(&privdata->read_blocking_sem, OS_WAIT_FOREVER);

    if (os_thread_create(&privdata->irq_thread,
                         "scIntrThread",
                         sc16is740_irq_thread,
                         privdata,
                         &sc16is740_thread_stack,
                         OS_PRIO_2) < 0)
    {
        goto err_open;
    }

    /* reset ringbuf */
    ringbytebuffer_reset(privdata->rbuf);

    sc16is740_power(privdata, 1);
    sc16is740_resetFifo(privdata);
    sc16is740_enableFifo(privdata, true);

    sc16is740_getReg(privdata, SC16IS740_REG_LCR, &lcr);
    /* enable enhanced features */
    sc16is740_setReg(privdata, SC16IS740_REG_LCR,
                     SC16IS740_LCR_ENHANCED_MODE);

    sc16is740_updateReg(privdata, SC16IS740_REG_EFR,
                        SC16IS740_EFR_ENABLE_BIT,
                        SC16IS740_EFR_ENABLE_BIT);
    /* Enable TCR/TLR */
    sc16is740_updateReg(privdata, SC16IS740_REG_MCR,
                        SC16IS740_MCR_TCLTLR_BIT,
                        SC16IS740_MCR_TCLTLR_BIT);

    /* Configure flow control levels */
    /* Flow control halt level 48, resume level 24 */
    sc16is740_setReg(privdata, SC16IS740_REG_TCR,
                     SC16IS740_TCR_RX_RESUME(24) |
                     SC16IS740_TCR_RX_HALT(48));

    sc16is740_setReg(privdata, SC16IS740_REG_LCR, lcr);

    privdata->format.baud = baud;

    sc16is740_updateUartParam(privdata, &privdata->format);

    sc16is740_setTxFifoLevel(privdata, privdata->fifo.TxFifo);
    sc16is740_setRxFifoLevel(privdata, privdata->fifo.RxFifo);
    /* enable the Rx and Tx FIFO */
    sc16is740_updateReg(privdata,
                        SC16IS740_REG_EFCR,
                        0,
                        SC16IS740_FECR_TXDIS_BIT |
                        SC16IS740_FECR_RXDIS_BIT);

    /* enable RX, TX, CTS change interrupts */
    ier = SC16IS740_IER_RHRI_BIT |
          SC16IS740_IER_THRI_BIT |
          SC16IS740_IER_CTS_BIT;
    sc16is740_setReg(privdata, SC16IS740_REG_IER, ier);
    return dev;

err_open:
    if (privdata->irq_thread) {
        os_thread_delete(&privdata->irq_thread);
        privdata->irq_thread = NULL;
    }
    if (privdata->read_blocking_sem) {
        os_semaphore_delete(&privdata->read_blocking_sem);
        privdata->read_blocking_sem = NULL;
    }
    if (privdata->intr_event_sem) {
        os_semaphore_delete(&privdata->intr_event_sem);
        privdata->intr_event_sem = NULL;
    }
    if (privdata->rbuf) {
        ringbytebuffer_destroy(privdata->rbuf);
        privdata->rbuf = NULL;
    }
    return NULL;
}

int sc16is740_drv_close(mdev_t *dev)
{
    sc16is740_priv_t *privdata = mdev_to_priv(dev);

    if (!privdata) {
        return -WM_FAIL;
    }

    if (privdata->read_blocking) {
        os_semaphore_put(&privdata->read_blocking_sem);
        ringbytebuffer_reset(privdata->rbuf);
    }

    /* disable all interrupts */
    sc16is740_setReg(privdata, SC16IS740_REG_IER, 0);
    /* disable TX/RX */
    sc16is740_updateReg(privdata,
                        SC16IS740_REG_EFCR,
                        SC16IS740_FECR_TXDIS_BIT |
                        SC16IS740_FECR_RXDIS_BIT,
                        SC16IS740_FECR_TXDIS_BIT |
                        SC16IS740_FECR_RXDIS_BIT);

    sc16is740_power(privdata, 0);

    /* stop thread */
    privdata->exit = 1;
    os_semaphore_put(&privdata->intr_event_sem);

    if (privdata->irq_thread) {
        os_thread_delete(&privdata->irq_thread);
        privdata->irq_thread = NULL;
    }
    if (privdata->read_blocking_sem) {
        os_semaphore_delete(&privdata->read_blocking_sem);
        privdata->read_blocking_sem = NULL;
    }
    if (privdata->intr_event_sem) {
        os_semaphore_delete(&privdata->intr_event_sem);
        privdata->intr_event_sem = NULL;
    }
    return os_semaphore_put(&sc16is740_mutex);
}

static int sc16is740_drv_release(sc16is740_priv_t *privdata)
{
    if (privdata->gpiod) {
        gpio_drv_close(privdata->gpiod);
        privdata->gpiod = NULL;
    }
    if (privdata->i2cbus) {
        i2cbus_drv_close(privdata->i2cbus);
        privdata->i2cbus = NULL;
    }
    if (privdata->irq_thread) {
        os_thread_delete(&privdata->irq_thread);
        privdata->irq_thread = NULL;
    }
    if (privdata->read_blocking_sem) {
        os_semaphore_delete(&privdata->read_blocking_sem);
        privdata->read_blocking_sem = NULL;
    }
    if (privdata->intr_event_sem) {
        os_semaphore_delete(&privdata->intr_event_sem);
        privdata->intr_event_sem = NULL;
    }
    if (privdata->reg_rw_mutex) {
        os_mutex_delete(&privdata->reg_rw_mutex);
        privdata->reg_rw_mutex = NULL;
    }
    if (privdata->rbuf) {
        ringbytebuffer_destroy(privdata->rbuf);
    }
    return WM_SUCCESS;
}

static int sc16is740_drv_setup(I2C_ID_Type id, uint16_t addr, int gpio)
{
    int ret;
    mdev_t *dev = &mdev_sc16is740;
    sc16is740_priv_t *privdata = &sc16is740_priv_data;

    /* init mdev node */
    dev->port_id = 0;
    dev->name = MDEV_SC16IS740;
    dev->pNextMdev = NULL;
    dev->private_data = (uint32_t)privdata;

    privdata->dev = dev;

    /* register access semaphore */
    if (os_mutex_create(&privdata->reg_rw_mutex, "scRegAccess",
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

    privdata->irq = gpio;
    privdata->oscillator = OSCILLATOR_12M;
    privdata->rx_bufsize = SC_BUF_SIZE;
    privdata->fifo.RxFifo = SC_RXFIFO_BYTE_8;
    privdata->fifo.TxFifo = SC_RXFIFO_BYTE_8;

    privdata->format.baud = DEFAULT_BAUD;
    privdata->format.databits = SC_DATABITS_8;
    privdata->format.parity = SC_PARITY_NONE;
    privdata->format.stopbits = SC_STOPBITS_1;
    privdata->format.flowctrl = SC_FLOWCTRL_NONE;

    privdata->read_blocking = false;
    privdata->rx_timout = OS_WAIT_FOREVER;
    privdata->tx_timout = OS_WAIT_FOREVER;

    /* disable all interrupts */
    sc16is740_setReg(privdata, SC16IS740_REG_IER, 0);
    /* disable tx/rx */
    sc16is740_setReg(privdata,
                     SC16IS740_REG_EFCR,
                     SC16IS740_FECR_TXDIS_BIT |
                     SC16IS740_FECR_RXDIS_BIT);

    /* enable EFR */
    sc16is740_setReg(privdata,
                     SC16IS740_REG_LCR,
                     SC16IS740_LCR_ENHANCED_MODE);
    sc16is740_setReg(privdata,
                     SC16IS740_REG_EFR,
                     SC16IS740_EFR_ENABLE_BIT);
    sc16is740_setReg(privdata, SC16IS740_REG_LCR, 0);

    /* suspend sc16is740 */
    sc16is740_power(privdata, 0);

    gpio_drv_init();
    privdata->gpiod = gpio_drv_open("MDEV_GPIO");
    gpio_drv_setdir(privdata->gpiod, gpio, GPIO_INPUT);
    /* setup interrupt */
    ret = gpio_drv_set_cb(privdata->gpiod,
                          gpio,
                          GPIO_INT_FALLING_EDGE,
                          privdata,
                          sc16is740_irq_handler);
    if (ret != WM_SUCCESS) {
        goto err_setup;
    }
    return WM_SUCCESS;

err_setup:

    if (privdata->gpiod) {
        /* disable gpio interrupt */
        gpio_drv_set_cb(privdata->gpiod,
                        privdata->irq,
                        GPIO_INT_DISABLE,
                        NULL,
                        NULL);
        gpio_drv_close(privdata->gpiod);
        privdata->gpiod = NULL;
    }
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

int sc16is740_drv_init()
{
    int ret;

    /* check driver has registered or not */
    if (mdev_get_handle(MDEV_SC16IS740) != NULL) {
        return WM_SUCCESS;
    }
    ret = os_mutex_create(&sc16is740_mutex, "sc16is740",
                          OS_MUTEX_INHERIT);
    if (ret < 0) {
        return -WM_FAIL;
    }
    /* initialize driver setting */
    if (sc16is740_drv_setup(board_sc16is7xx_i2c_port_id(),
                            board_sc16is7xx_i2c_address(),
                            board_sc16is7xx_intr_pin()) < 0)
    {
        os_mutex_delete(&sc16is740_mutex);
        return -WM_FAIL;
    }
    return mdev_register(&mdev_sc16is740);
}

int sc16is740_drv_deinit()
{
    sc16is740_priv_t *privdata = &sc16is740_priv_data;

    /* check driver has registered or not */
    if (!mdev_get_handle(MDEV_SC16IS740)) {
        return WM_SUCCESS;
    }

    /* disable gpio interrupt */
    gpio_drv_set_cb(privdata->gpiod,
                    privdata->irq,
                    GPIO_INT_DISABLE,
                    NULL,
                    NULL);
    sc16is740_drv_release(privdata);
    os_mutex_delete(&sc16is740_mutex);
    mdev_deregister(MDEV_SC16IS740);
    return WM_SUCCESS;
}
