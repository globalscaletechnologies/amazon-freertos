/** @file bt_uart.c
*
*  @brief UART Interface with BT
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

/* UART Boot Baud Rate */
#define UART_BOOT_BAUD_RATE	57600
/* Start Transfer */
#define STX			0x02
/* Start of Header */
#define SOH			0x01
#define ACK			0x06
#define NACK			0x15

/* On EVB-I4 Board, an inverter is used in a path
	to trigger RESET_BT signal by GPIO_0.
	In such case reset trigger logic need to be inverted.
	Default:  RESET_BT_WITH_INVERTER
*/
#ifdef RESET_BT_NO_INVERTER
#define RESET_BT_ACTIVE		1
#define RESET_BT_INACTIVE	0
#else
#define RESET_BT_ACTIVE		0
#define RESET_BT_INACTIVE	1
#endif

static int ble_init();

/* This structure is used by AW-CU302 and it selects UART
 * transport layer interface (external).
 */
ble_interface_t external_ble_interface = {
	.hcitrans_mdev_init = NULL,
	.ble_init = (int(*)(void)) ble_init
};

/* Structure for all the components related to the
 * BT firmware download protocol over UART */
typedef struct {
	/* Flash device handle */
	mdev_t *fl_dev;
	/* Data length */
	uint32_t data_len;
	/* Data buffer */
	uint8_t *data_buf;
	/* Current address of firmware */
	int fw_cur_address;
} bt_fw_priv_t;

/* Global structure */
static bt_fw_priv_t bt_fw_priv;

/* Initialization function for flash */
static int flash_open()
{
	/* Initialize flash memory to be used */
	flash_drv_init();
	/* Open the flash and get the internal flash device handle */
	bt_fw_priv.fl_dev = flash_drv_open(FL_INT);
	if (bt_fw_priv.fl_dev == NULL)
		return -WM_FAIL;
	/* Allocate the data buffer for storing data */
	bt_fw_priv.data_buf = os_mem_alloc(FLASH_RX_BUF_SIZE);
	if (bt_fw_priv.data_buf == NULL)
		return -WM_FAIL;
	/* Initialize the data length to be zero */
	bt_fw_priv.data_len = 0;
	return WM_SUCCESS;
}

/* Close the flash device */
static int flash_close()
{
	int rv = flash_drv_close(bt_fw_priv.fl_dev);
	if (rv != WM_SUCCESS)
		return rv;
	/* Deallocate the data buffer */
	os_mem_free(bt_fw_priv.data_buf);
	/* Make the data length to be zero */
	bt_fw_priv.data_len = 0;
	return rv;
}

/* Read data from the flash */
static int read_flash_data(uint8_t *data_buf, uint32_t data_len)
{
	int rv = flash_drv_read(bt_fw_priv.fl_dev,
			data_buf,
			data_len,
			bt_fw_priv.fw_cur_address);

	if (rv != WM_SUCCESS)
		return rv;
	/* Increment the current address to read from the flash */
	bt_fw_priv.fw_cur_address += data_len;
	return data_len;
}

/* Calculate XOR of input */
static uint8_t calculate_crc(uint8_t *buf, int size)
{
	int i = 0, crc = 0;
	for (i = 0; i < size; i++)
		crc = crc ^ buf[i];
	return crc;

}
static int ble_init()
{
	short history = 0;
	uint16_t len = 0;
	int rv, read_len;
	uint32_t data_len;
	uint8_t crc = 0, read_cmd, write_cmd[3] = {SOH, 0x00, 0x00};
	struct partition_entry *part_entry, *f1, *f2;;

	/* Partition Init */
	rv = part_init();
	if (rv != WM_SUCCESS)
		return rv;

	/* Find the active partition and use it for UART boot */
	f1 = part_get_layout_by_id(FC_COMP_BT_FW, &history);
	f2 = part_get_layout_by_id(FC_COMP_BT_FW, &history);

	if (f1 && f2)
		part_entry = part_get_active_partition(f1, f2);
	else if (!f1 && f2)
		part_entry = f2;
	else if (!f2 && f1)
		part_entry = f1;
	else {
		bt_e("BT Firmware not found");
		return -WM_FAIL;
	}

	len = part_entry->size;
	memcpy(&write_cmd[1], &len, sizeof(len));

	/* Open the flash device */
	rv = flash_open();
	if (rv != WM_SUCCESS) {
		bt_e("flash_open() failed");
		return rv;
	}

	/* Reset DA14580 */
	GPIO_PinMuxFun(GPIO_0, GPIO0_GPIO0);
	GPIO_SetPinDir(GPIO_0, GPIO_OUTPUT);
	GPIO_WritePinOutput(GPIO_0, RESET_BT_ACTIVE);
	os_thread_sleep(os_msec_to_ticks(100));
	GPIO_WritePinOutput(GPIO_0, RESET_BT_INACTIVE);

	rv = bt_drv_uart_init(UART_BOOT_BAUD_RATE, UART_PARITY_NONE, ENABLE,
		FLOW_CONTROL_NONE);
	if (rv != WM_SUCCESS) {
		bt_e("bt_drv_uart_init() failed");
		return rv;
	}

	while (1) {
		rv = bt_drv_uart_recv(&read_cmd, sizeof(read_cmd));
		if (read_cmd == STX)
			break;
	}

	rv = bt_drv_uart_send(write_cmd, sizeof(write_cmd));
	if (rv != sizeof(write_cmd)) {
		bt_e("Writing header to UART failed");
		goto end;
	}

	rv = bt_drv_uart_recv(&read_cmd, sizeof(read_cmd));
	if ((rv != sizeof(read_cmd)) || (read_cmd != ACK)) {
		bt_e("Expected ACK 0x%2x Received 0x%2x", ACK, read_cmd);
		goto end;
	}

	bt_d("ACK received as expected\r\n");
	/* Load the BLE firmware over UART */
	bt_d("Loading firmware from address 0x%2x of length %u\r\n",
		part_entry->start, part_entry->size);
	bt_fw_priv.data_len = len;
	bt_fw_priv.fw_cur_address = part_entry->start;

	while (bt_fw_priv.data_len) {
		if (bt_fw_priv.data_len > FLASH_RX_BUF_SIZE)
			read_len = FLASH_RX_BUF_SIZE;
		else
			read_len = bt_fw_priv.data_len;
		/* Read the requested length data from the flash */
		data_len = read_flash_data(bt_fw_priv.data_buf, read_len);
		/* Calculate CRC */
		crc = crc ^ calculate_crc(bt_fw_priv.data_buf, read_len);
		/* Write the data read from the flash to UART port */
		bt_drv_uart_send(bt_fw_priv.data_buf, data_len);

		bt_fw_priv.data_len -= data_len;
	}

	bt_d("Calculated CRC: 0x%2x", crc);
	rv = bt_drv_uart_recv(&read_cmd, sizeof(read_cmd));
	if ((rv != sizeof(read_cmd)) || (read_cmd != crc)) {
		bt_e("CRC validation failed. Expected 0x%2x and received 0x%2x",
			crc, read_cmd);
		goto end;
	}

	write_cmd[0] = ACK;
	rv = bt_drv_uart_send(write_cmd, 1);
	if (rv != 1) {
		bt_e("Writing ACK to UART failed");
		goto end;
	}

	bt_d("UART Boot successful");
end:
	bt_drv_uart_deinit();
	/* Close the flash device */
	flash_close();
	return rv;
}
