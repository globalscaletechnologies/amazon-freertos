/** @file bt_uart.h
*
*  @brief Interface BT with UART
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

#ifndef _BT_UART_H_
#define _BT_UART_H_

/* Firmware type */
typedef enum {
	/* Helper UART firmware */
	FW_HELPER_UART,
	/* BT firmware */
	FW_BT,
} fw_type_t;

/** Initialize the UART for HCI UART interface.
 * It initializes the UART port with hardware
 * flow control enabled.
 *
 * \return WM_SUCCESS on success, -WM_FAIL otherwise.
 */
int bt_drv_uart_init(uint32_t baud, uint32_t parity, uint32_t stopbits,
	flow_control_t flow_control);

/** Send a HCI packet over UART
 *
 * This function is used to send a HCI packet over UART.
 *
 * \param[in] data Pointer to the packet contents.
 * \param[in] size Length of the packet in bytes.
 *
 * \return WM_SUCCESS on success, -WM_FAIL otherwise.
 */
int bt_drv_uart_send(uint8_t *data, uint32_t size);

/** Receive a HCI packet over UART
 *
 * This function is used to receive a HCI packet over UART.
 *
 * \param[in] data Pointer to the packet contents.
 * \param[in] size Length of the packet in bytes.
 *
 * \return WM_SUCCESS on success, -WM_FAIL otherwise.
 */
int bt_drv_uart_recv(uint8_t *data, uint32_t size);
#endif /* !_BT_UART_H_ */
