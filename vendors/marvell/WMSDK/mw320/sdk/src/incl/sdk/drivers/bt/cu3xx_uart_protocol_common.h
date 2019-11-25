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

/** @file cu3xx_uart_protocol_common.h
*
*  @brief This file contains data required for uart communication protocol.
*/

#ifndef _CU3xx_UART_PROTOCOL_COMMON_H_
#define _CU3xx_UART_PROTOCOL_COMMON_H_

#define MAX_SUBTYPE_LOOKUP_INDEX_VALUE 26

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef enum {
	C_HOST_TO_BLE_REQ = 0x01,
	C_HOST_TO_BLE_RESP,
	C_BLE_TO_HOST_REQ,
	C_BLE_TO_HOST_RESP,
} ble_cmd_type_t;

/* Make sure updating the value of macro 'MAX_SUBTYPE_LOOKUP_INDEX_VALUE' incase
 * of addition of new command else it wont be handled
 */
typedef enum {
	SC_READY = 0x01,
	SC_MTU_SUPP,
	SC_ADD_GATT_SVC,
	SC_ADD_GATT_CHAR,
	SC_ADD_GATT_CHAR_DESC,
	SC_SET_ADV_DATA,
	SC_SET_SCAN_RSP_DATA,
	SC_SET_ADV_INTV,
	SC_ADV_START,
	SC_ADV_STOP,
	SC_CONNECTED,
	SC_DISCONNECTED,
	SC_MTU_INFO,
	SC_READ,
	SC_WRITE,
	SC_INDICATE,
	SC_DISCONNECT_REQ,
	SC_SET_ANT_MODE,
	SC_FAULT,		/*in future it can also be used to handle different error conditions*/
	SC_SET_BACKUP_DATA,
	SC_GET_BACKUP_DATA,
	SC_GET_DASDK_COMP_VERSION,
	SC_SET_BD_ADDR,
	SC_SET_BD_NAME,
	SC_INDICATE_CONFIRMATION,
	/* This should be the last command for sdk.Place all sdk specific command subtypes before this*/
	SC_SDK_LAST_CMD = 0xBF,
	/* Add application specific command subtypes here starting from 0xC0*/

} ble_cmd_subtype_t;

typedef enum {
	TYPE_SUBTYPE_MISMATCH = 0x01,

} cmd_error_t;

typedef struct __attribute__((__packed__)) {
	ble_cmd_type_t type;
	ble_cmd_subtype_t subtype;
	uint8_t seq_num;
	uint16_t data_len;  /* Data len: Little endian format */
	uint8_t crc;
} ble_header_t;

typedef struct __attribute__((__packed__)) {
	uint8_t crc;
} ble_trailer_t;

typedef enum {
	TYPE_UUID = 0x01,
	TYPE_TOTAL_SIZE,
	TYPE_NUM_16_BIT_UUID_ATTR,
	TYPE_NUM_32_BIT_UUID_ATTR,
	TYPE_NUM_128_BIT_UUID_ATTR,
	TYPE_PROP,
	TYPE_HANDLE,
	TYPE_DATA_LEN,
	TYPE_DATA,
} ble_gatt_db_tlv_t;

#endif   //_CU3xx_UART_PROTOCOL_COMMON_H_
