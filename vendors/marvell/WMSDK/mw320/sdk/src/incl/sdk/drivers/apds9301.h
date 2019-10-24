/*
*  (C) Copyright 2019 GlobalScale Technologie Inc. All Rights Reserved.
*/

#ifndef _APDS9301_H_
#define _APDS9301_H_

#include <stdbool.h>
#include <mdev.h>

#define MDEV_APDS9301 "MDEV_APDS9301"

typedef enum  {
	rControl		= 0x0,
	rTiming			= 0x1,
	rThresholdLow	= 0x2,
	rThresholdHigh	= 0x4,
	rInterrupt		= 0x6,
	rID				= 0xa,
	rData0			= 0xc,
	rData1			= 0xe
} APDS9301_Register_t;

typedef void (*apds9301_irq_cb) (void *data);

/* Command register bits */
#define APDS9301_CMD	(1 << 7) /* Select command register. Must write as 1 */
#define APDS9301_CLEAR	(1 << 6) /* Interrupt clear. Clears pending interrupt */
#define APDS9301_WORD	(1 << 5) /* I2C write/read: if 1 word, if 0 byte */

/* Register set */
#define APDS9301_CONTROL	0x00 /* Control of basic functions */
#define APDS9301_TIMING		0x01 /* Integration time / gain control */
#define APDS9301_THRESHLOWLOW	0x02 /* Low byte of low interrupt threshold */
#define APDS9301_THRESHHIGHLOW	0x04 /* Low byte of high interrupt threshold */
#define APDS9301_INTERRUPT	0x06 /* Interrupt control */
#define APDS9301_CRC		0x08 /* Factory test */
#define APDS9301_ID			0x0a /* Part number / Rev ID */
#define APDS9301_DATA0LOW	0x0c /* Low byte of ADC channel 0 */
#define APDS9301_DATA1LOW	0x0e /* Low byte of ADC channel 1 */

#define APDS9301_GAIN		(0x01 << 4)
#define APDS9301_MANUAL		(0x01 << 3)
#define APDS9301_INTEG		(0x03 << 0)

#define APDS9301_INTEG_LOW		0x00	/* scale 0.034 , time 13.7ms */
#define APDS9301_INTEG_MIDIUM	0x01	/* scale 0.252 , time 101ms */
#define APDS9301_INTEG_HIGH		0x02	/* scale 1 , time 402ms */
#define APDS9301_INTEG_MANUAL	0x03	/* n/a */

/* Power on/off value for APDS9300_CONTROL register */
#define APDS9301_POWER_ON	0x03
#define APDS9301_POWER_OFF	0x00

/* Interrupts */
#define APDS9301_INTR_ENABLE		(0x01 << 4)
#define APDS9301_INTR_ENABLE_MASK	(0x03 << 4)
#define APDS9301_INTR_PERSIST_MASK	(0x0f << 0)

int apds9301_powerOn(mdev_t *dev);
int apds9301_powerOff(mdev_t *dev);
int apds9301_isPowerOn(mdev_t *dev);
int apds9301_setLowThreshold(mdev_t *dev, uint16_t threshold);
int apds9301_setHighThreshold(mdev_t *dev, uint16_t threshold);
int apds9301_setADCGain(mdev_t *dev, uint8_t gain);
int apds9301_getADCGain(mdev_t *dev);
int apds9301_setIntegrationTime(mdev_t *dev, uint8_t time);
int apds9301_getIntegrationTime(mdev_t *dev);
int apds9301_setInterruptOn(mdev_t *dev);
int apds9301_setInterruptOff(mdev_t *dev);
int apds9301_getInterruptState(mdev_t *dev);
int apds9301_setGenInterruptEveryADCCycle(mdev_t *dev);
int apds9301_setGenInterruptPeriodOfOutRange(mdev_t *dev, int period);
int apds9301_getGenerateInterruptMode(mdev_t *dev);
int apds9301_getADCChannel(mdev_t *dev, int channel);
float apds9301_getLux(mdev_t *dev);
int apds9301_getPartNum(mdev_t *dev);
int apds9301_getRevNum(mdev_t *dev);
int apds9301_setInterruptCallback(mdev_t *dev, apds9301_irq_cb cb);
mdev_t *apds9301_drv_open(const char *name);
int apds9301_drv_close(mdev_t *dev);
int apds9301_drv_init(I2C_ID_Type id, uint16_t addr, int gpio);
int apds9301_drv_deinit(I2C_ID_Type id);

#endif /* _APDS9301_H_ */
