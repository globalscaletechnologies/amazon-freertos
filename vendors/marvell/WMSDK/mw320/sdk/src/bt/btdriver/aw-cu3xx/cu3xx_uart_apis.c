/*
 *  (C) Copyright 2018 Marvell International Ltd. All Rights Reserved.
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

/** @file cu3xx_uart_apis.c
 *
 *  @brief This file contains UART api implementaion.
 */

#include <wmtypes.h>
#include <mdev_uart.h>
#include <wm_os.h>
#include <wmstdio.h>
#include <wmlog.h>
#include <cu3xx_ble_requestor.h>
#include <cu3xx_host_apis.h>
#include <bt_uart_common.h>
#if defined(CONFIG_BOOT_BLE_ONCE)
#include <bt.h>
#include <bt_uart.h>
#endif

static mdev_t *dev;
#if defined(CONFIG_BOOT_BLE_ONCE)
#define BLE_STX_WAIT_TIME 2000000 //2sec
#define UART_BOOT_BAUD_RATE	57600
#define STX			0x02

int uart_check_if_bt_already_running(bool *first_boot)
{
	uint8_t read_cmd = 0;
	uint32_t cur = os_get_timestamp();

	if (bt_drv_uart_init(UART_BOOT_BAUD_RATE, UART_PARITY_NONE, ENABLE,
		FLOW_CONTROL_NONE) != WM_SUCCESS) {
		bt_e("uart_check_if_bt_already_running() failed");
		return -WM_FAIL;
	}

	/* Wait 2 sec for STX message. If not received consider it is not first Boot.
	 * This might add Boot delay. */
	while ((os_get_timestamp() - cur) < BLE_STX_WAIT_TIME){
		bt_drv_uart_recv_with_timeout(&read_cmd, sizeof(read_cmd), BLE_STX_WAIT_TIME);
		if (read_cmd == STX)
			break;
	}
	if (read_cmd != STX) {
		bt_e("BT already Running");
		*first_boot = false;
	} else {
		*first_boot = true;
	}
	bt_drv_uart_deinit();
	return WM_SUCCESS;
}
#endif
int uart_proto_init()
{
	if (uart_drv_init(UART1_ID, UART_8BIT) != WM_SUCCESS) {
	bt_e("Failed to initialize UART1");
		return -WM_FAIL;
	}
	uart_drv_set_opts(UART1_ID, UART_PARITY_NONE, UART_STOPBITS_1,
			  FLOW_CONTROL_HW);
	uart_drv_blocking_read(UART1_ID, true);
	dev = uart_drv_open(UART1_ID, 115200);
	if (dev == NULL) {
	bt_e("Failed to open UART1");
		return -WM_FAIL;
	}
	uart_flush_all_data();
	return WM_SUCCESS;
}

void uart_flush_all_data()
{
	int read_len;
	uint8_t byte;

	uart_drv_blocking_read(UART1_ID, false);
	while (1) {
		read_len = uart_drv_read(dev, &byte, 1);
		if (read_len <= 0) {
			break;
		}
	}
	uart_drv_blocking_read(UART1_ID, true);
}

uint16_t uart_read_data(uint8_t *buf, uint16_t len)
{
	uint16_t read_len, total_len = 0;

	while (total_len < len) {
		read_len = uart_drv_read(dev, buf + total_len,
					 len - total_len);
		if (read_len < 0) {
			bt_e("Invalid data");
			return read_len;
		}
		total_len += read_len;
	}
	return total_len;
}

uint16_t uart_write_data(uint8_t *buf, uint16_t len)
{
	return (uint16_t) uart_drv_write(dev, buf, len);

}
