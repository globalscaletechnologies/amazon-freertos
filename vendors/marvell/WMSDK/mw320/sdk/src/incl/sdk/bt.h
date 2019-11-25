/** @file bt.h
*
*  @brief BT Interaction Header
*
*  (C) Copyright 2008-2018 Marvell International Ltd. All Rights Reserved
*
*  MARVELL CONFIDENTIAL
*  The source code contained or described herein and all documents related to
*  the source code ("Material") are owned by Marvell International Ltd or its
*  suppliers or licensors. Title to the Material remains with Marvell
*  International Ltd or its suppliers and licensors. The Material contains
*  trade secrets and proprietary and confidential information of Marvell or its
*  suppliers and licensors. The Material is protected by worldwide copyright
*  and trade secret laws and treaty provisions. No part of the Material may be
*  used, copied, reproduced, modified, published, uploaded, posted,
*  transmitted, distributed, or disclosed in any way without Marvell's prior
*  express written permission.
*
*  No license under any patent, copyright, trade secret or other intellectual
*  property right is granted to or conferred upon you by disclosure or delivery
*  of the Materials, either expressly, by implication, inducement, estoppel or
*  otherwise. Any license under such intellectual property rights must be
*  express and approved by Marvell in writing.
*
*/


#ifndef _BT_H_
#define _BT_H_
#include <stdint.h>
#include <mdev_uart.h>

/* Macro's */
/* 1K buffer size for UART receive data */
#define UART_RX_BUF_SIZE		1024
/* Flash read buffer size */
#define FLASH_RX_BUF_SIZE		(1024 * 2)

/* Structure for function pointers related to BLE interface */
typedef struct {
	/* Function to select mrvlstack transport layer */
	void (*hcitrans_mdev_init)(void);
	/* Function to initialize the BLE interface. */
	int (*ble_init)(void);
} ble_interface_t;

/** Download firmware for BT and initialise the stack.
 * This function calls the init function which downloads the firmware
 * as per the selected board. The init functions are defined
 * in bt_sdio.c and bt_uart.c respectively.
 * It initialises the BLE stack for both the boards.
 * \return WM_SUCCESS on success, -WM_FAIL otherwise.
 */
int bt_init(void);
/**
 * This function handles the transport layer selection of
 * BLE interface (SDIO, UART) and BLE initialisation for different
 * boards.
 */
ble_interface_t board_ble_interface();
/* Initialize BT UART with the configuration */
/*
 * It is required to boot the Bluetooth chip through UART.
 */
int bt_drv_uart_init(uint32_t baud, uint32_t parity, uint32_t stopbits,
	flow_control_t flow_control);
/* This function is used by the BT UART driver. */
/* It gets the buffer to write on UART */
int bt_drv_uart_send(uint8_t *data, uint32_t size);
/* This function is used by the BT UART driver. */
/* It reads the buffer from uart */
int bt_drv_uart_recv(uint8_t *data, uint32_t size);
/* It reads the buffer from uart in non blocking way*/
int bt_drv_uart_recv_with_timeout(uint8_t *data, 
					uint32_t size, 
					uint32_t timeout);
/* This function is used to deinit BT UART driver */
int bt_drv_uart_deinit(void);
#endif /* !_BT_H_ */
