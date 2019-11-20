/*
*  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
*/

#ifndef _SC16IS740_H_
#define _SC16IS740_H_

#include <stdbool.h>
#include <mdev.h>
#include <mdev_i2c.h>
#include <mdev_i2cbus.h>
#include <mdev_gpio.h>
#include <lowlevel_drivers.h>

#define MDEV_SC16IS740 "MDEV_SC16IS740"

#define OSCILLATOR_12M      12288000    /* 12.288MHz */
#define OSCILLATOR_1_8M     1843200     /* 1.8432MHz */
#define XON_CHAR            0x11
#define XOFF_CHAR           0x13
#define SC_FIFO_DEPTH       64
#define SC_BUF_SIZE         256

typedef enum _sc_Baud {
    SC_BAUD_110 = 110,
    SC_BAUD_300 = 300,
    SC_BAUD_600 = 600,
    SC_BAUD_2400 = 2400,
    SC_BAUD_4800 = 4800,
    SC_BAUD_9600 = 9600,
    SC_BAUD_14400 = 14400,
    SC_BAUD_19200 = 19200,
    SC_BAUD_38400 = 38400,
    SC_BAUD_57600 = 57600
} sc_Baud_t;

#define DEFAULT_BAUD        SC_BAUD_9600

typedef enum _sc_DataBits {
    SC_DATABITS_5,
    SC_DATABITS_6,
    SC_DATABITS_7,
    SC_DATABITS_8
} sc_DataBits_t;

typedef enum _sc_Parity {
    SC_PARITY_NONE,
    SC_PARITY_ODD,
    SC_PARITY_EVEN
} sc_Parity_t;

typedef enum _sc_StopBits {
    SC_STOPBITS_1,
    SC_STOPBITS_2
} sc_StopBits_t;

typedef enum _sc_FlowCtrl {
    SC_FLOWCTRL_NONE = 0x0,
    SC_FLOWCTRL_RTSCTS = 0x1,
    SC_FLOWCTRL_IXON = 0x2,
    SC_FLOWCTRL_IXOFF = 0x4
} sc_FlowCtrl_t;

typedef enum _sc_RxFIFOLevel {
    SC_RXFIFO_BYTE_8,
    SC_RXFIFO_BYTE_16,
    SC_RXFIFO_BYTE_32,
    SC_RXFIFO_BYTE_56
} sc_RxFIFOLevel_t;

typedef enum _sc_TxFIFOLevel {
    SC_TXFIFO_BYTE_8,
    SC_TXFIFO_BYTE_16,
    SC_TXFIFO_BYTE_32,
    SC_TXFIFO_BYTE_56
} sc_TxFIFOLevel_t;

typedef struct _sc_FifoCfg {
    sc_RxFIFOLevel_t RxFifo;
    sc_TxFIFOLevel_t TxFifo;
    bool FifoEnable;
} sc_FifoCfg_t;

typedef struct _sc_UartFormat {
    sc_Baud_t baud;
    sc_DataBits_t databits;
    sc_Parity_t parity;
    sc_StopBits_t stopbits;
    sc_FlowCtrl_t flowctrl;
} sc_UartFormat_t;

/* general register set, only accessible on LCR[7]=0 */
#define SC16IS740_REG_RHR           0x00
#define SC16IS740_REG_THR           0x00
#define SC16IS740_REG_IER           0x01
#define SC16IS740_REG_FCR           0x02
#define SC16IS740_REG_IIR           0x02
#define SC16IS740_REG_LCR           0x03
#define SC16IS740_REG_MCR           0x04
#define SC16IS740_REG_LSR           0x05
#define SC16IS740_REG_MSR           0x06
#define SC16IS740_REG_SPR           0x07
/* TCR/TLR only available on MCR[2] = 1 and EFR[4] = 1 */
#define SC16IS740_REG_TCR           0x06
#define SC16IS740_REG_TLR           0x07
#define SC16IS740_REG_TXLVL         0x08
#define SC16IS740_REG_RXLVL         0x09
#define SC16IS740_REG_IODIR         0x0A
#define SC16IS740_REG_IOSTATE       0x0B
#define SC16IS740_REG_IOINT         0x0C
#define SC16IS740_REG_IOCONTROL     0x0E
#define SC16IS740_REG_EFCR          0x0F
/* special register set, only accessible on LCR[7]=1 and not 0xBF */
#define SC16IS740_REG_DLL           0x00
#define SC16IS740_REG_DLH           0x01
/* enhanced register set, only accessible on LCR = 0xBF */
#define SC16IS740_REG_EFR           0x02
#define SC16IS740_REG_XON1          0x04
#define SC16IS740_REG_XON2          0x05
#define SC16IS740_REG_XOFF1         0x06
#define SC16IS740_REG_XOFF2         0x07

/* IER register bits */
#define SC16IS740_IER_CTS_BIT       (1 << 7) /* nCTS interrupt enable */
#define SC16IS740_IER_RTS_BIT       (1 << 6) /* nRTS interrupt enable */
#define SC16IS740_IER_XOFFI_BIT     (1 << 5) /* Xoff interrupt wnable */
#define SC16IS740_IER_SLEEP_BIT     (1 << 4) /* Sleep mode */
#define SC16IS740_IER_MSI_BIT       (1 << 3) /* Modem status interrupt */
#define SC16IS740_IER_RLSI_BIT      (1 << 2) /* Receive line status interrupt */
#define SC16IS740_IER_THRI_BIT      (1 << 1) /* Transmit holding register interrupt */
#define SC16IS740_IER_RHRI_BIT      (1 << 0) /* Receive holding register interrupt */

/* FCR register bits */
#define SC16IS740_FCR_RXLV_OFF      6 /* RX FIFO trigger level */
#define SC16IS740_FCR_RXLV_MSK      (3 << 6) /* RX FIFO trigger level */
#define SC16IS740_FCR_TXLV_OFF      4 /* TX FIFO trigger level */
#define SC16IS740_FCR_TXLV_MSK      (3 << 4) /* TX FIFO trigger level */
#define SC16IS740_FCR_TXRESET_BIT   (1 << 2) /* reset TX FIFO */
#define SC16IS740_FCR_RXRESET_BIT   (1 << 1) /* reset RX FIFO */
#define SC16IS740_FCR_FIFO_EN_BIT   (1 << 0) /* FIFO ENABLE */

/* LCR register bits */
#define SC16IS740_LCR_DIVLE_BIT     (1 << 7) /* divisor latch enable */
#define SC16IS740_LCR_BRK_BIT       (1 << 6) /* break control bit */
#define SC16IS740_LCR_PARITYSET_BIT (1 << 5) /* force set parity */
#define SC16IS740_LCR_PARITYEVEN_BIT    (1 << 4) /* parity type select */
#define SC16IS740_LCR_PARITY_BIT    (1 << 3) /* parity type select */
#define SC16IS740_LCR_STOP_BIT      (1 << 2) /* number of stop bits */
#define SC16IS740_LCR_WORD_LEN_OFF  0   /* word length offset */
#define SC16IS740_LCR_WORD_LEN_MSK  (3 << 0) /* word length mask */
#define SC16IS740_LCR_WORD_LEN_5        0 /* 5 bits word */
#define SC16IS740_LCR_WORD_LEN_6        1 /* 6 bits word */
#define SC16IS740_LCR_WORD_LEN_7        2 /* 7 bits word */
#define SC16IS740_LCR_WORD_LEN_8        3 /* 8 bits word */

#define SC16IS740_LCR_ENHANCED_MODE     0xBF /* enhanced register mode */

/* LSR register bits */
#define SC16IS740_LSR_FIFOE_BIT     (1 << 7) /* fifo data error */
#define SC16IS740_LSR_THSRE_BIT     (1 << 6) /* THR and TSR empty */
#define SC16IS740_LSR_THRE_BIT      (1 << 5) /* THR empty */
#define SC16IS740_LSR_BRKI_BIT      (1 << 4) /* break interrupt */
#define SC16IS740_LSR_FRME_BIT      (1 << 3) /* framing error */
#define SC16IS740_LSR_PARE_BIT      (1 << 2) /* parity error */
#define SC16IS740_LSR_OVRE_BIT      (1 << 1) /* overrun error */
#define SC16IS740_LSR_DATR_BIT      (1 << 0) /* data in receiver */
#define SC16IS740_LSR_BRK_ERROR_MASK    0x1E /* BI, FE, PE, OE bits */

/* MCR register bits */
#define SC16IS740_MCR_CLKD_BIT      (1 << 7) /* clock divisor */
#define SC16IS740_MCR_IREN_BIT      (1 << 6) /* IrDA mode enable */
#define SC16IS740_MCR_XONA_BIT      (1 << 5) /* Xon Any */
#define SC16IS740_MCR_LOOP_BIT      (1 << 4) /* enable loopback */
#define SC16IS740_MCR_TCLTLR_BIT    (1 << 2) /* TCR and TLR enable */
#define SC16IS740_MCR_RTS_BIT       (1 << 1) /* RTS */
#define SC16IS740_MCR_DTR_BIT       (1 << 0) /* DTR */

/* MSR register bits */
#define SC16IS740_MSR_CD_BIT        (1 << 7) /* CD */
#define SC16IS740_MSR_RI_BIT        (1 << 6) /* RI */
#define SC16IS740_MSR_DSR_BIT       (1 << 5) /* DSR */
#define SC16IS740_MSR_CTS_BIT       (1 << 4) /* CTS */
#define SC16IS740_MSR_DCD_BIT       (1 << 3) /* delta CD */
#define SC16IS740_MSR_DRI_BIT       (1 << 2) /* delta RI */
#define SC16IS740_MSR_DDSR_BIT      (1 << 1) /* delta DSR */
#define SC16IS740_MSR_DCTS_BIT      (1 << 0) /* delta CTS */

/* IIR register bits */
#define SC16IS740_IIR_MFCR_OFF      6 /* mirror the contents of FCR[0] offset */
#define SC16IS740_IIR_MFCR_MSK      (3 << 6) /* mirror the contents of FCR[0] mask */
#define SC16IS740_IIR_INTS_OFF      1 /* interrupt source offset */
#define SC16IS740_IIR_INTS_MASK     (0x1F << 1) /* interrupt source mask */
#define SC16IS740_IIR_STATUS_BIT    (1 << 0) /* interrupt status */
#define SC16IS740_IIR_RLSE          0x06 /* Receiver Line Status error */
#define SC16IS740_IIR_RTOI          0x0C /* Receiver time-out interrupt */
#define SC16IS740_IIR_RHRI          0x04 /* RHR interrupt */
#define SC16IS740_IIR_THRI          0x02 /* THR interrupt */
#define SC16IS740_IIR_MDMI          0x00 /* modem interrupt */
#define SC16IS740_IIR_IPCS          0x30 /* input pin change of state */
#define SC16IS740_IIR_XOFF          0x10 /* received Xoff signal/special character */
#define SC16IS740_IIR_CTSRTS        0x20 /* CTS, RTS change of state from active to inactive */

/* EFR register bits */
#define SC16IS740_FER_CTS_BIT       (1 << 7) /* CTS flow control enable */
#define SC16IS740_FER_RTS_BIT       (1 << 6) /* RTS flow control enable */
#define SC16IS740_FER_SCHD_BIT      (1 << 5) /* Special character detect */
#define SC16IS740_EFR_ENABLE_BIT    (1 << 4) /* Enhanced functions enable bit */
#define SC16IS740_EFR_SWFLOW3_BIT   (1 << 3) /* SWFLOW bit 3 */
#define SC16IS740_EFR_SWFLOW2_BIT   (1 << 2) /* SWFLOW bit 2 */
#define SC16IS740_EFR_SWFLOW1_BIT   (1 << 1) /* SWFLOW bit 1 */
#define SC16IS740_EFR_SWFLOW0_BIT   (1 << 0) /* SWFLOW bit 0 */
#define SC16IS740_FER_FLOW_OFF      0 /* software flow control offset */
#define SC16IS740_FER_FLOW_MASK     0x0F /* software flow control mask */

/* EFCR register bits */
#define SC16IS740_FECR_IRDA_BIT     (1 << 7) /* IrDA mode */
#define SC16IS740_FECR_RTSI_BIT     (1 << 5) /* invert RTS signal in RS-485 mode */
#define SC16IS740_FECR_RTSC_BIT     (1 << 4) /* enable the transmitter to control the RTS pin */
#define SC16IS740_FECR_TXDIS_BIT    (1 << 2) /* disable transmitter */
#define SC16IS740_FECR_RXDIS_BIT    (1 << 1) /* disable receiver */
#define SC16IS740_FECR_9BIT_BIT     (1 << 0) /* Enable 9-bit or Multidrop mode (RS-485) */

/* TCR register bits */
#define SC16IS740_TCR_RX_HALT(words)    ((((words) / 4) & 0x0f) << 0)
#define SC16IS740_TCR_RX_RESUME(words)  ((((words) / 4) & 0x0f) << 4)

int sc16is740_drv_rx_available(mdev_t *dev);
int sc16is740_drv_rxbuf_size(uint32_t size);
int sc16is740_drv_set_opts(sc_DataBits_t databits,
                           sc_Parity_t parity,
                           sc_StopBits_t stopbits,
                           sc_FlowCtrl_t flow_control);
int sc16is740_drv_timeout(uint32_t tx_timeout, uint32_t rx_timeout);
int sc16is740_drv_blocking_read(bool is_blocking);
int sc16is740_drv_rx_buf_reset(mdev_t *dev);
int sc16is740_drv_write(mdev_t *dev, void* buf, int size);
int sc16is740_drv_read(mdev_t *dev, void* buf, int size);
mdev_t *sc16is740_drv_open(uint32_t baud);
int sc16is740_drv_close(mdev_t *dev);
int sc16is740_drv_init();
int sc16is740_drv_deinit();

extern I2C_ID_Type board_sc16is7xx_i2c_port_id();
extern int board_sc16is7xx_i2c_address();
extern int board_sc16is7xx_intr_pin();

#endif /* _SC16IS740_H_ */
