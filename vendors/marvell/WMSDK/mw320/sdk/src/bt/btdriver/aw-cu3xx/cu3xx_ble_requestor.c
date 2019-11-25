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

/** @file cu3xx_ble_requestor.c
 *
 *  @brief This file contains Requestor DB helper functions.
 */

#include <string.h>
#include <wm_os.h>
#include <wmtypes.h>
#include <wmstdio.h>
#include <mdev_uart.h>
#include <wmstdio.h>
#include <wmlog.h>
#include <cu3xx_ble_requestor.h>
#include <bt_uart_common.h>
static os_mutex_t uart_tx_lock;
static db_entry_t *requestor_database;

int uart_tx_lock_init()
{
	return os_mutex_create(&uart_tx_lock, "UART lock", OS_MUTEX_INHERIT);
}

int uart_tx_lock_deinit()
{
	return os_mutex_delete(&uart_tx_lock);
}

int uart_tx_lock_get()
{
	return os_mutex_get(&uart_tx_lock, OS_WAIT_FOREVER);
}

int uart_tx_lock_put()
{
	return os_mutex_put(&uart_tx_lock);
}

int requestor_database_init(int num_requestor)
{
	if (num_requestor > MAX_DATABASE_SIZE) {
		bt_e("Number of requestor exceeding the database "
			 "size");
		return -WM_FAIL;
	}

	requestor_database = os_mem_calloc(num_requestor * sizeof(db_entry_t));
	if (requestor_database == NULL) {
		bt_e("Requestor database memory allocation "
			 "failure");
		return -WM_FAIL;
	}
	return WM_SUCCESS;
}

int requestor_database_deinit()
{
	if (requestor_database != NULL) {
		os_mem_free(requestor_database);
		requestor_database = NULL;
	}
	return WM_SUCCESS;
}

db_entry_t *requestor_database_get_entry(uint16_t seq_num)
{
	int i;

	if (requestor_database == NULL) {
		bt_e("Requestor database not initialized");
		return NULL;
	}

	for (i = 0; i < MAX_DATABASE_SIZE; i++) {
		if (requestor_database[i].seq_num == seq_num) {
			return &requestor_database[i];
		}
	}

	return NULL;
}

int requestor_database_set_entry(db_entry_t *db_entry)
{
	int i;
	db_entry_t null_entry;
	memset(&null_entry, 0, sizeof(db_entry_t));

	if (requestor_database == NULL) {
		bt_e("Requestor database not initialized");
		return -WM_FAIL;
	}

	if (requestor_database_get_entry(db_entry->seq_num) != NULL) {
		bt_e("Entry already exists in the database for "
			 "sequence number %d", db_entry->seq_num);
		return -WM_FAIL;
	}

	for (i = 0; i < MAX_DATABASE_SIZE; i++) {
		if (!memcmp(&null_entry, &requestor_database[i],
			    sizeof(db_entry_t))) {
			memcpy(&requestor_database[i], db_entry,
			       sizeof(db_entry_t));
			return WM_SUCCESS;
		}
	}

	bt_e("Database limit has reached. Returning!");
	return -WM_FAIL;
}

int requestor_database_delete_entry(uint16_t seq_num)
{
	int i;

	if (requestor_database == NULL) {
		bt_e("Requestor database not initialized");
		return -WM_FAIL;
	}

	for (i = 0; i < MAX_DATABASE_SIZE; i++) {
		if (requestor_database[i].seq_num == seq_num) {
			memset(&requestor_database[i], 0, sizeof(db_entry_t));
			return WM_SUCCESS;
		}
	}

	return -WM_FAIL;
}
