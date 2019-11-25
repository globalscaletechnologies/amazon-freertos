/*
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

/** @file cu3xx_ble_requestor.h
*
*  @brief This file contains BLE database,uart requester data
*/

#ifndef __CU3xx_BLE_REQUESTOR_H__
#define __CU3xx_BLE_REQUESTOR_H__
/*
*  includes
*/
#include "cu3xx_uart_protocol_common.h"

#define MAX_DATABASE_SIZE       10
#define MAX_RECV_BUFFER_SIZE    512
#define MAX_SEM_WAIT            500
#define BLE_MOD_NAME            "ble"
#define VAR_BLE_ANTENNA_MODE    "antenna_mode"

/** Maximum BLE device name length */
#define MAX_DEVICE_NAME_LEN           32

/*
* TYPE DEFINITIONS
****************************************************************************************
*/

typedef void (*response_cb_t)(uint8_t *, int);
typedef struct {
	uint8_t seq_num;
	uint8_t *resp_ptr;
	uint16_t max_resp_len;
	uint16_t *resp_len_ptr;
	response_cb_t cb;
} db_entry_t;

int uart_tx_lock_init();

int uart_tx_lock_deinit();

int uart_tx_lock_get();

int uart_tx_lock_put();

/** BLE requester database initialisation
 *
 * This function initialises requester database.
 *
 * \param[in] num_requester number of expected requester entries.
 *
 * \return WM_SUCCESS on success
 */
int requestor_database_init(int num_requestor);

/** BLE requester database deinitialisation
 *
 * This function deinitialises requester database.
 *
 * \return WM_SUCCESS on success
 */
int requestor_database_deinit();

/** BLE requester database get entry
 *
 * This function gives requester database entry for sequence number.
 *
 * \param[in] seq_num sequnce number .
 *
 * \return WM_SUCCESS on success
 */
db_entry_t *requestor_database_get_entry(uint16_t seq_num);

/** BLE requester database set entry
 *
 * This function sets requester database entry.
 *
 * \param[in] db_entry pointer for expected requester entry.
 *
 * \return WM_SUCCESS on success
 */
int requestor_database_set_entry(db_entry_t *db_entry);

/** BLE requester database delete entry
 *
 * This function deletes requester database entry.
 *
 * \param[in] seq_num sequence number for expected entry to be deleted.
 *
 * \return WM_SUCCESS on success
 */
int requestor_database_delete_entry(uint16_t seq_num);

#endif  /* ! __CU3xx_BLE_REQUESTOR_H__ */
