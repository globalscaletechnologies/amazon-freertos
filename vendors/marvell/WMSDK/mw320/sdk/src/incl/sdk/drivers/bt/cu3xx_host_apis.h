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

/** @file cu3xx_host_apis.h
*
*  @brief This file contains apis for creating BLE application
*/

#ifndef __CU3xx_HOST_APIS_H__
#define __CU3xx_HOST_APIS_H__

#include <string.h>
#include <wm_os.h>
#include <wmtypes.h>
#include <wmstdio.h>
#include <mdev_uart.h>
#include "cu3xx_uart_apis.h"
#include "cu3xx_ble_requestor.h"

#define MAX_REQ_BUFFER_SIZE 1024
#define MAX_CMD_SUBTYPE_VALUE 	256

typedef	int (*ble_request_handler_t)(ble_header_t *header, uint8_t *reqbuf);
/** Structure for registering cmd subtype functions*/
typedef struct{
	/** The cmd id for subtype */
	uint8_t cmd_subtype;
	/** The function that should be invoked for this subtype. */
	ble_request_handler_t host_request_handlers;
}cmd_subtype_handler_t;

/** Send uart request
 *
 * This function sends request packet over uart
 *
 * \param[in] reqbuf uint8_t pointer to data to be sent.
 * \param[in] reqbuf_len length of request data buffer
 * \param[in] respbuf uint8_t pointer to response buffer.
 * \param[in] respbuf_len length of resp buffer.
 * \param[in] cmd subtype of command to be sent.
 * \param[in] resp_cb call back to the function after reception of response.
 *
 * \return WM_SUCCESS on success
 */
int send_request(uint8_t *reqbuf, uint16_t reqbuf_len,
		 uint8_t *respbuf, uint16_t respbuf_len,
		 ble_cmd_subtype_t cmd, response_cb_t resp_cb);

/** Send uart response
 *
 * This function sends response packet over uart.
 *
 * \param[in] req_header structure pointer to the header to be sent.
 * \param[in] respbuf uint8_t pointer to response data to be sent.
 * \param[in] len length of response data buffer
 *
 * \return WM_SUCCESS on success
 */
int send_response(ble_header_t *req_header, uint8_t *respbuf, uint16_t len);

int host_proto_init();

int host_proto_deinit();

/** Request handler registration
 *
 * This function registers handlers for requests from BLE.
 *
 * \param[in] subtype for which handler to be called.
 * \param[in] ble_request_handler function pointer for the function to be used when
 * received registered subtype.
 *
 * \return WM_SUCCESS on success
 */
int register_request_handler(int ble_cmd_subtype,
			     ble_request_handler_t ble_request_handler);

#endif  /* ! __CU3xx_HOST_APIS_H__ */
