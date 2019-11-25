/** @file bt_uart_common.h
*
*  @brief UART Interface with BT Chip
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

#ifndef _BT_UART_COMMON_H_
#define _BT_UART_COMMON_H_

#include <wmlog.h>

#ifdef CONFIG_BT_DEBUG
#define bt_d(...)                            \
        wmlog("bt_drv", ##__VA_ARGS__)
#else
#define bt_d(...)
#endif /* ! CONFIG_BT_DEBUG */

#define bt_e(...)				\
	wmlog_e("bt_drv", ##__VA_ARGS__)
#define bt_w(...)				\
	wmlog_w("bt_drv", ##__VA_ARGS__)

/* Read from UART */
int uart_read(mdev_t *uart_dev, uint8_t *data_buf, int len);
/* Read from UART Without blocking*/
int uart_read_non_blocking(mdev_t *uart_dev, 
					uint8_t *data_buf, 
					int len, 
					uint32_t timeout);
/* Write to UART */
int uart_write(mdev_t *uart_dev, uint8_t *data_buf, int len);
/* Deinitialize the UART */
int uart_deinit(mdev_t *uart_dev);
#endif /* !_BT_UART_COMMON_H_ */
