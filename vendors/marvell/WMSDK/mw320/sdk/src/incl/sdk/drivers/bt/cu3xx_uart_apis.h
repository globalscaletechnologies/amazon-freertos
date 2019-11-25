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

/** @file cu3xx_uart_apis.h
*
*  @brief This file contains uart apis used for communication
*/

#ifndef __CU3xx_UART_APIS_H__
#define __CU3xx_UART_APIS_H__

/** UART Initialisation
 *
 * This function initialises UART with 115200 baudrate.
 *
 *
 * \return WM_SUCCEESS on successful initialisation.
 */
int uart_proto_init();

int uart_check_if_bt_already_running();

void uart_flush_all_data();
/** Read UART data
 *
 * This function reads data over uart with number of bytes provided as len argument.
 *
 * \param[in] buf uint8_t pointer to data buffer.
 * \param[in] len length of data buffer.
 *
 * \return number of bytes read.
 */
uint16_t uart_read_data(uint8_t *buf, uint16_t len);

/** Write data over UART.
 *
 * This function writes data over uart with number of bytes provided as len argument.
 *
 * \param[in] buf uint8_t pointer to data buffer.
 * \param[in] len length of data buffer.
 *
 * \return number of bytes written.
 */
uint16_t uart_write_data(uint8_t *buf, uint16_t len);

#endif  /* ! __CU3xx_UART_APIS_H__ */
