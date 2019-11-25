/** @file bt_uart_common.c
*
*  @brief UART Interface with Chip
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

#include <mdev_uart.h>
#include <partition.h>
#include <wm_os.h>
#include <bt_uart.h>
#include <bt_uart_common.h>
#include <bt.h>
#include <mw300_uart.h>

static mdev_t *bt_uart_dev;


/* Initialize UART with the configuration for UART Boot */
int bt_drv_uart_init(uint32_t baud, uint32_t parity, uint32_t stopbits,
	flow_control_t flow_control)
{
	UART_ID_Type uart_id = board_ble_uart_id();
	/* Initialize UART1 with 8bit. This will register UART1 MDEV handler */
	int rv = uart_drv_init(uart_id, UART_8BIT);
	if (rv != WM_SUCCESS)
		return rv;
	/* Set hardware flow control */
	rv = uart_drv_set_opts(uart_id, parity, stopbits, flow_control);
	if (rv != WM_SUCCESS)
		return rv;
	/* Set the RX Buffer size to be 1k */
	rv = uart_drv_rxbuf_size(uart_id, UART_RX_BUF_SIZE);
	if (rv != WM_SUCCESS)
		return rv;

	/* Blocking read enabled */
	rv = uart_drv_blocking_read(uart_id, true);
	if (rv != WM_SUCCESS)
		return rv;

	bt_uart_dev = uart_drv_open(uart_id, baud);
	if (bt_uart_dev == NULL)
		return -WM_FAIL;
	return WM_SUCCESS;
}

/* This function is used by the mrvlstack HCI-UART driver. */
/* It gets the buffer to write on UART */
int bt_drv_uart_send(uint8_t *data, uint32_t size)
{
	int len = uart_write(bt_uart_dev, data, size);
	return len;
}

/* This function is used by the mrvlstack HCI-UART driver. */
/* It reads the buffer from uart */
int bt_drv_uart_recv(uint8_t *data, uint32_t size)
{
	int len = uart_read(bt_uart_dev, data, size);
	return len;
}

int bt_drv_uart_recv_with_timeout(uint8_t *data, uint32_t size, uint32_t timeout)
{
	int len = uart_read_non_blocking(bt_uart_dev, data, size, timeout);
	return len;
}

/* Read from UART */
int uart_read_non_blocking(mdev_t *uart_dev, 
					uint8_t *data_buf, 
					int len, 
					uint32_t timeout)
{
	int l_len = 0;
	uint32_t cur = os_get_timestamp();
	uart_drv_blocking_read(board_ble_uart_id(), false);
	while ((l_len < len) &&
			(timeout > (os_get_timestamp() - cur))){
		l_len += uart_drv_read(uart_dev,
			data_buf + l_len, len - l_len);
	}
	uart_drv_blocking_read(board_ble_uart_id(), true);
	return l_len;
}

int uart_read(mdev_t *uart_dev, uint8_t *data_buf, int len)
{
	int l_len = 0;
	while (l_len < len)
		l_len += uart_drv_read(uart_dev,
			data_buf + l_len, len - l_len);
	return l_len;
}

/* Write to UART */
int uart_write(mdev_t *uart_dev, uint8_t *data_buf, int len)
{
	int l_len = 0;
	while (l_len < len)
		l_len += uart_drv_write(uart_dev,
			data_buf + l_len, len - l_len);
	return l_len;
}

/* Deinitialize the UART */
int uart_deinit(mdev_t *uart_dev)
{
	UART_ID_Type uart_id = board_ble_uart_id();
	int rv = uart_drv_close(uart_dev);
	if (rv != WM_SUCCESS)
		return rv;
	uart_drv_deinit(uart_id);
	return WM_SUCCESS;
}

int bt_drv_uart_deinit()
{
	return uart_deinit(bt_uart_dev);
}
