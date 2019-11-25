/*
 *
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

/** @file cu3xx_ble_database.c
 *
 *  @brief This file contains the BLE database.
 */

#include <limits.h>
#include <wm_os.h>
//#include <psm.h>
//#include <mrvlstack_config.h>
//#include <main_gap_api.h>
//#include <main_gatt_api.h>
#include <cu3xx_ble_requestor.h>
#include <cu3xx_host_apis.h>
#include <cu3xx_ble_database.h>

#define MAX_SERVICE_DATA_LEN        64
#define MAX_CHARACTERISTIC_DATA_LEN 64
#define MAX_RESPONSE_BUF_LEN        16
#define SEND_REQUEST_TIMEOUT        10000
#define BLE_SET_ADV_OFFSET          3
#define MAX_REQUEST_BUF_LEN         64

#define UNUSED_ARG(x) (void)(x)
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

os_semaphore_t wait_resp;
uint8_t resp_buf[MAX_RESPONSE_BUF_LEN];
uint16_t handle;

static void rec_request_cb(uint8_t *respbuf, int len)
{
	if (respbuf[0] == 0x00)
		memcpy((uint8_t *)&handle, &respbuf[1],
		       sizeof(uint16_t));
	os_semaphore_put(&wait_resp);
}

void common_req_cb(uint8_t *respbuf, int len)
{
	os_semaphore_put(&wait_resp);
}

int ble_add_service(uint16_t sid, rec_uuid_t *serv_uuid, uint16_t total_size,
		    uint16_t n_16, uint16_t n_32, uint16_t n_128,
		    uint16_t *serv_handle)
{
	UNUSED_ARG(sid);
	int index = 0;
	uint8_t data_service[MAX_SERVICE_DATA_LEN];

	memset(data_service, 0, sizeof(data_service));

	data_service[index++] = TYPE_UUID;
	data_service[index++] = serv_uuid->uuid_len;
	switch (serv_uuid->uuid_len) {
	case SIZEOF_RECORD_UUID_16BIT:
		memcpy(&data_service[index], &serv_uuid->uuid.uuid16,
		       serv_uuid->uuid_len);
		break;
	case SIZEOF_RECORD_UUID_32BIT:
		memcpy(&data_service[index], &serv_uuid->uuid.uuid32,
		       serv_uuid->uuid_len);
		break;
	case SIZEOF_RECORD_UUID_128BIT:
		memcpy(&data_service[index], serv_uuid->uuid.uuid128,
		       serv_uuid->uuid_len);
		break;
	}
	index += serv_uuid->uuid_len;

	/* Populate TOTAL SIZE TLV */
	data_service[index++] = TYPE_TOTAL_SIZE;
	data_service[index++] = sizeof(uint16_t);
	memcpy(&data_service[index], &total_size, sizeof(uint16_t));
	index += sizeof(uint16_t);

	/* Populate Count of 16 bit record TLV */
	data_service[index++] = TYPE_NUM_16_BIT_UUID_ATTR;
	data_service[index++] = sizeof(uint8_t);
	memcpy(&data_service[index], &n_16, sizeof(uint8_t));
	index += sizeof(uint8_t);

	/* Populate Count of 32 bit record TLV */
	data_service[index++] = TYPE_NUM_32_BIT_UUID_ATTR;
	data_service[index++] = sizeof(uint8_t);
	memcpy(&data_service[index], &n_32, sizeof(uint8_t));
	index += sizeof(uint8_t);

	/* Populate Count of 128 bit record TLV */
	data_service[index++] = TYPE_NUM_128_BIT_UUID_ATTR;
	data_service[index++] = sizeof(uint8_t);
	memcpy(&data_service[index], &n_128, sizeof(uint8_t));
	index += sizeof(uint8_t);

	send_request(data_service, index, resp_buf, sizeof(resp_buf),
		     SC_ADD_GATT_SVC, rec_request_cb);
	os_semaphore_get(&wait_resp, os_msec_to_ticks(SEND_REQUEST_TIMEOUT));
	memcpy(serv_handle, &handle, sizeof(uint32_t));
	return WM_SUCCESS;
}

int ble_add_characteristic(rec_uuid_t *char_uuid, uint8_t permissions,
			   uint16_t serv_handle, uint16_t char_size,
			   uint16_t *char_handle)
{
	int index = 0;
	uint8_t data_service[MAX_SERVICE_DATA_LEN];

	memset(data_service, 0, sizeof(data_service));

	data_service[index++] = TYPE_UUID;
	data_service[index++] = char_uuid->uuid_len;
	switch (char_uuid->uuid_len) {
	case SIZEOF_RECORD_UUID_16BIT:
		memcpy(&data_service[index], &char_uuid->uuid.uuid16,
		       char_uuid->uuid_len);
		break;
	case SIZEOF_RECORD_UUID_32BIT:
		memcpy(&data_service[index], &char_uuid->uuid.uuid32,
		       char_uuid->uuid_len);
		break;
	case SIZEOF_RECORD_UUID_128BIT:
		memcpy(&data_service[index], char_uuid->uuid.uuid128,
		       char_uuid->uuid_len);
		break;
	}
	index += char_uuid->uuid_len;

	/* Populate PROP TLV */
	data_service[index++] = TYPE_PROP;
	data_service[index++] = sizeof(uint8_t);
	memcpy(&data_service[index], &permissions, sizeof(uint8_t));
	index += sizeof(uint8_t);

	/* Populate Service handle */
	data_service[index++] = TYPE_HANDLE;
	data_service[index++] = sizeof(uint16_t);
	memcpy(&data_service[index], &serv_handle, sizeof(uint16_t));
	index += sizeof(uint16_t);

	/* Populate characteristic size  */
	data_service[index++] = TYPE_DATA_LEN;
	data_service[index++] = sizeof(uint16_t);
	memcpy(&data_service[index], &char_size, sizeof(uint16_t));
	index += sizeof(uint16_t);

	send_request(data_service, index, resp_buf, sizeof(resp_buf),
		     SC_ADD_GATT_CHAR, rec_request_cb);
	os_semaphore_get(&wait_resp, os_msec_to_ticks(SEND_REQUEST_TIMEOUT));
	memcpy(char_handle, &handle, sizeof(uint32_t));

	return WM_SUCCESS;
}

int ble_add_characteristic_descriptor(rec_uuid_t *char_uuid,
				      uint8_t permissions,
				      uint16_t serv_handle,
				      uint16_t char_size,
				      void *data, int data_len,
				      uint16_t *char_handle)
{
	int index = 0;
	uint8_t data_service[MAX_CHARACTERISTIC_DATA_LEN];

	UNUSED_ARG(data);
	UNUSED_ARG(data_len);

	memset(data_service, 0, sizeof(data_service));

	data_service[index++] = TYPE_UUID;
	data_service[index++] = char_uuid->uuid_len;
	switch (char_uuid->uuid_len) {
	case SIZEOF_RECORD_UUID_16BIT:
		memcpy(&data_service[index], &char_uuid->uuid.uuid16,
		       char_uuid->uuid_len);
		break;
	case SIZEOF_RECORD_UUID_128BIT:
		memcpy(&data_service[index], char_uuid->uuid.uuid128,
		       char_uuid->uuid_len);
		break;
	}
	index += char_uuid->uuid_len;

	/* Populate PROP TLV */
	data_service[index++] = TYPE_PROP;
	data_service[index++] = sizeof(uint8_t);
	memcpy(&data_service[index], &permissions, sizeof(uint8_t));
	index += sizeof(uint8_t);

	/* Populate Service handle */
	data_service[index++] = TYPE_HANDLE;
	data_service[index++] = sizeof(uint16_t);
	memcpy(&data_service[index], &serv_handle, sizeof(uint16_t));
	index += sizeof(uint16_t);

	/* Populate characteristic size  */
	data_service[index++] = TYPE_DATA_LEN;
	data_service[index++] = sizeof(uint16_t);
	memcpy(&data_service[index], &char_size, sizeof(uint16_t));
	index += sizeof(uint16_t);

	send_request(data_service, index, resp_buf, sizeof(resp_buf),
		     SC_ADD_GATT_CHAR_DESC, rec_request_cb);
	os_semaphore_get(&wait_resp, os_msec_to_ticks(SEND_REQUEST_TIMEOUT));
	memcpy(char_handle, &handle, sizeof(uint32_t));

	return WM_SUCCESS;
}

int ble_set_advertisement_data(uint8_t *adv_data, int len)
{
	if (len < 0)
		return -WM_FAIL;
	send_request(&adv_data[BLE_SET_ADV_OFFSET], len - BLE_SET_ADV_OFFSET,
		     resp_buf, sizeof(resp_buf), SC_SET_ADV_DATA,
		     common_req_cb);
	os_semaphore_get(&wait_resp, OS_WAIT_FOREVER);
	return WM_SUCCESS;
}

int ble_set_scan_data(uint8_t *scan_data, int len)
{
	if (len < 0)
		return -WM_FAIL;
	send_request(scan_data, len, resp_buf, sizeof(resp_buf),
		     SC_SET_SCAN_RSP_DATA, common_req_cb);
	os_semaphore_get(&wait_resp, OS_WAIT_FOREVER);
	return WM_SUCCESS;
}

int ble_start_advertisement()
{
	send_request(NULL, 0, resp_buf, sizeof(resp_buf), SC_ADV_START,
		     common_req_cb);
	os_semaphore_get(&wait_resp, os_msec_to_ticks(SEND_REQUEST_TIMEOUT));
	return WM_SUCCESS;
}

int ble_stop_advertisement()
{
	send_request(NULL, 0, resp_buf, sizeof(resp_buf), SC_ADV_STOP,
		     common_req_cb);
	os_semaphore_get(&wait_resp, os_msec_to_ticks(SEND_REQUEST_TIMEOUT));
	return WM_SUCCESS;
}

void ble_disconnect()
{
	send_request(NULL, 0, resp_buf, sizeof(resp_buf), SC_DISCONNECT_REQ,
		     common_req_cb);
	os_semaphore_get(&wait_resp, os_msec_to_ticks(SEND_REQUEST_TIMEOUT));
}

int ble_set_adv_int(uint16_t min_intv, uint16_t max_intv)
{
	int index = 0;
	uint8_t data_service[MAX_CHARACTERISTIC_DATA_LEN];

	memset(data_service, 0, sizeof(data_service));

	memcpy(&data_service[index], &min_intv,
		   sizeof(min_intv));
	index += sizeof(min_intv);

	memcpy(&data_service[index], &min_intv,
		   sizeof(min_intv));
	index += sizeof(min_intv);

	send_request(data_service, index, resp_buf, sizeof(resp_buf),
				 SC_SET_ADV_INTV, common_req_cb);
	os_semaphore_get(&wait_resp, os_msec_to_ticks(SEND_REQUEST_TIMEOUT));
	return WM_SUCCESS;
}

