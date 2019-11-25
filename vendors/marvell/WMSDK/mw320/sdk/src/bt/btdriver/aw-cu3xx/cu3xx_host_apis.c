/** @file cu3xx_host_apis.c
*
*  @brief Host Commands API
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

#include <string.h>
#include <wm_os.h>
#include <wmtypes.h>
#include <wmstdio.h>
#include <mdev_uart.h>
#include <wmlog.h>
#include <wm_utils.h>
#include <cu3xx_uart_apis.h>
#include <cu3xx_host_apis.h>
#include <cu3xx_uart_protocol_common.h>
#include <bt_uart_common.h>
static ble_header_t header;
static os_thread_t uart_rx_thread;
static uint8_t request_buffer[MAX_REQ_BUFFER_SIZE];

static cmd_subtype_handler_t cmd_subtype_handler[MAX_SUBTYPE_LOOKUP_INDEX_VALUE];
static os_thread_stack_define(uart_rx_handler_stack, 8 * 1024);

/* Define CONFIG_DEBUG_BUILD to enable dump_hex,
 * CONFIG_BT_UART_HEX_DEBUG_ENABLE to enable hex_dbg_with_name
 * and CONFIG_BT_DEBUG for bt debug logs
 */

#ifdef CONFIG_BT_UART_HEX_DEBUG_ENABLE
static void hex_dbg_with_name(char *name, uint8_t *buf, int len)
{
	bt_d("%s\r\n", name);
	dump_hex(buf, len);
	bt_d("\r\n");
}
#else
static void hex_dbg_with_name(char *name, uint8_t *buf, int len)
{

}
#endif
static uint8_t calculate_crc(const uint8_t *data, uint16_t len)
{
	int i = 0;
	uint8_t calculated_crc = 0x00;

	while (i < len) {
		calculated_crc ^= data[i++];
	}
	return calculated_crc;
}
static int lookup_subtype_index(uint8_t subtype)
{
	uint8_t i = 0;
	for(i = 0; i <= MAX_SUBTYPE_LOOKUP_INDEX_VALUE; i++) {
		if(subtype == cmd_subtype_handler[i].cmd_subtype) {
			return i;
		}
	}
	return -WM_FAIL;
}
static int invoke_cmd_handler(ble_header_t *header, uint8_t *reqbuf)
{
	int index = 0;
	uint8_t send_data;
	if (header->subtype > MAX_CMD_SUBTYPE_VALUE ) {
			return -WM_FAIL;
	}
	index = lookup_subtype_index(header->subtype);
	if(index != (-WM_FAIL)) {
		cmd_subtype_handler[index].host_request_handlers(header, reqbuf);
		return WM_SUCCESS;
	} else {
		bt_e("Command handler not registered for subtype = %d",header->subtype);
		send_data = TYPE_SUBTYPE_MISMATCH;
		header->type = C_HOST_TO_BLE_RESP;
		send_response(header, &send_data, sizeof(send_data));
		return -WM_FAIL;
	}
	return WM_SUCCESS;
}

static int handle_response(ble_header_t *header)
{
	uint16_t ret;
	db_entry_t *entry;
	uint8_t crc, calculated_crc;

	uart_tx_lock_get();

	entry = requestor_database_get_entry(header->seq_num);
	if (entry == NULL) {
		uart_tx_lock_put();
		bt_e("Invalid sequence number");
		return -WM_FAIL;
	}

	if (header->data_len > entry->max_resp_len) {
		uart_tx_lock_put();
		bt_e("Not enough buffer space");
		return -WM_FAIL;
	}

	ret = uart_read_data(entry->resp_ptr, header->data_len);
	if (ret != header->data_len) {
		uart_tx_lock_put();
		bt_e("Invalid data");
		return -WM_FAIL;
	}

	hex_dbg_with_name("Dumping response:", (void *)entry->resp_ptr,
			  header->data_len);

	calculated_crc = calculate_crc(entry->resp_ptr, header->data_len);
	ret = uart_read_data(&crc, sizeof(crc));
	if (ret != sizeof(crc) || calculated_crc != crc) {
		uart_tx_lock_put();
		bt_e("Invalid crc");
		return -WM_FAIL;
	}

	uart_tx_lock_put();
	entry->cb(entry->resp_ptr, header->data_len);

	uart_tx_lock_get();
	requestor_database_delete_entry(entry->seq_num);
	uart_tx_lock_put();

	return WM_SUCCESS;
}

static int handle_request(ble_header_t *header)
{
	uint16_t ret;
	uint8_t crc, calculated_crc;

	if (header->data_len > MAX_REQ_BUFFER_SIZE) {
		bt_e("Not enough memory to handle request");
		return -WM_FAIL;
	}

	memset(request_buffer, 0, sizeof(request_buffer));
	ret = uart_read_data(request_buffer, header->data_len);
	if (ret != header->data_len) {
		bt_e("Invalid data");
		return -WM_FAIL;
	}

	hex_dbg_with_name("Received request:", (void *)request_buffer,
			  header->data_len);

	calculated_crc = calculate_crc(request_buffer, header->data_len);
	ret = uart_read_data(&crc, sizeof(crc));
	if (ret != sizeof(crc) || calculated_crc != crc) {
		bt_e("Invalid crc");
		return -WM_FAIL;
	}

	invoke_cmd_handler(header, request_buffer);

	return WM_SUCCESS;
}

static void uart_rx_handler(os_thread_arg_t data)
{
	int ret;
	uint8_t send_data;
	while (1) {
		ret = uart_read_data((uint8_t *)&header, sizeof(ble_header_t));
		hex_dbg_with_name("Received header:", (void *)&header,
				  sizeof(header));
		if (calculate_crc((uint8_t *)&header, (sizeof(header)-sizeof(header.crc))) != header.crc) {
			bt_e("Invalid Header CRC");
			uart_flush_all_data();
			continue;
		}
		switch (header.type) {
		case C_BLE_TO_HOST_REQ:
			ret = handle_request(&header);
			if (ret != WM_SUCCESS) {
				bt_e("Handle request failed");
				uart_flush_all_data();
			}
			break;
		case C_BLE_TO_HOST_RESP:
			ret = handle_response(&header);
			if (ret != WM_SUCCESS) {
				bt_e("Handle response failed");
				uart_flush_all_data();
			}
			break;
		default:
			uart_flush_all_data();
			header.type = C_HOST_TO_BLE_RESP;
			header.subtype = SC_FAULT;
			send_data = TYPE_SUBTYPE_MISMATCH;
			send_response(&header, &send_data, sizeof(send_data));
			break;
		}

	}
	host_proto_deinit();
	os_thread_self_complete(NULL);
	return;
}

static int host_rx_init()
{
	int ret;

	if (!uart_rx_thread) {
		ret = os_thread_create(
			&uart_rx_thread,
			"uart rx thread",
			uart_rx_handler,
			NULL,
			&uart_rx_handler_stack,
			OS_PRIO_3);
		if (ret != WM_SUCCESS) {
			bt_e("Failed to create uart rx handler "
				 "thread");
			return -WM_FAIL;
		}
	}

	return WM_SUCCESS;
}

int host_proto_init()
{
	bt_d("UART DRV init done\r\n");
	if (uart_proto_init() != WM_SUCCESS) {
		bt_e("Failed to initilize uart");
		return -WM_FAIL;
	}

	if (uart_tx_lock_init() != WM_SUCCESS) {
		bt_e("Failed to create uart lock");
		return -WM_FAIL;
	}

	if (requestor_database_init(MAX_DATABASE_SIZE) != WM_SUCCESS) {
		bt_e("Failed to create requestor information "
			 "database");
		return -WM_FAIL;
	}

	if (host_rx_init() != WM_SUCCESS) {
		bt_e("Failed to create requestor information "
			 "database");
		return -WM_FAIL;
	}

	return WM_SUCCESS;
}

int send_request(uint8_t *reqbuf, uint16_t reqbuf_len,
		 uint8_t *respbuf, uint16_t respbuf_len,
		 ble_cmd_subtype_t cmd, response_cb_t resp_cb)
{
	int ret = WM_SUCCESS;
	db_entry_t entry;
	ble_header_t header;
	uint8_t calculated_crc;
	uint16_t len;
	static uint8_t seq_counter;

	header.type = C_HOST_TO_BLE_REQ;
	header.subtype = cmd;
	header.data_len = reqbuf_len;
	header.seq_num = ++seq_counter;
	header.crc=calculate_crc((uint8_t *)&header,(sizeof(header)-sizeof(header.crc)));
	entry.resp_ptr = respbuf;
	entry.max_resp_len = respbuf_len;
	entry.cb = resp_cb;
	entry.seq_num = seq_counter;

	calculated_crc = calculate_crc(reqbuf, reqbuf_len);

	hex_dbg_with_name("Dumping header:", (void *)&header, sizeof(header));

	hex_dbg_with_name("Dumping buffer:", (void *)reqbuf, reqbuf_len);

	hex_dbg_with_name("Calculated CRC:", (void *)&calculated_crc,
			  sizeof(calculated_crc));

	if (seq_counter == 255)
		seq_counter = 0;

	uart_tx_lock_get();
	len = uart_write_data((uint8_t *)&header, sizeof(header));
	if (len != sizeof(header)) {
		bt_e("Failed to send request header");
		ret = -WM_FAIL;
		goto out;
	}
	len = uart_write_data(reqbuf, reqbuf_len);
	if (len != reqbuf_len) {
		bt_e("Failed to send request data");
		ret = -WM_FAIL;
		goto out;
	}
	len = uart_write_data(&calculated_crc, sizeof(calculated_crc));
	if (len != sizeof(calculated_crc)) {
		bt_e("Failed to send crc of request data");
		ret = -WM_FAIL;
		goto out;
	}
	requestor_database_set_entry(&entry);
out:
	uart_tx_lock_put();
	return ret;
}

int send_response(ble_header_t *req_header, uint8_t *respbuf, uint16_t len)
{
	int ret = WM_SUCCESS;
	uint8_t calculated_crc;
	uint16_t ret_len;
	ble_header_t resp_header;

	resp_header.type = C_HOST_TO_BLE_RESP;
	resp_header.subtype = req_header->subtype;
	resp_header.seq_num = req_header->seq_num;
	resp_header.data_len = len;
	resp_header.crc=calculate_crc((uint8_t *)&resp_header,(sizeof(resp_header)-sizeof(resp_header.crc)));
	calculated_crc = calculate_crc(respbuf, len);


	hex_dbg_with_name("Dumping response header", (void *)&resp_header,
			  sizeof(ble_header_t));

	hex_dbg_with_name("Dumping response data", (void *)respbuf, len);

	hex_dbg_with_name("Calculated CRC:", (void *)&calculated_crc,
			  sizeof(calculated_crc));

	uart_tx_lock_get();
	ret_len = uart_write_data((uint8_t *) &resp_header,
				  sizeof(ble_header_t));
	if (ret_len != sizeof(ble_header_t)) {
		bt_e("Failed to send response header");
		ret = -WM_FAIL;
		goto out;
	}
	ret_len = uart_write_data(respbuf, len);
	if (ret_len != len) {
		bt_e("Failed to send response data");
		ret = -WM_FAIL;
		goto out;
	}
	ret_len = uart_write_data(&calculated_crc, sizeof(uint8_t));
	if (ret_len != sizeof(uint8_t)) {
		bt_e("Failed to send crc of response data");
		ret = -WM_FAIL;
		goto out;
	}
	hex_dbg_with_name("Success in sending response", NULL, 0);
out:
	uart_tx_lock_put();
	return ret;
}

int register_request_handler(int ble_cmd_subtype,
			     ble_request_handler_t ble_request_handler)
{
	static uint8_t registered_subtype_counter = 0;
	if (ble_cmd_subtype >= MAX_CMD_SUBTYPE_VALUE) {
		bt_e("Invalid command subtype");
		return -WM_FAIL;
	}
	cmd_subtype_handler[registered_subtype_counter].cmd_subtype = ble_cmd_subtype;
	cmd_subtype_handler[registered_subtype_counter].host_request_handlers = ble_request_handler;
	registered_subtype_counter += 1;
	return WM_SUCCESS;
}
