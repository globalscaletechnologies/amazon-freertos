/** @file bt_sdio.h
*
*  @brief Interface BT with SDIO
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


#ifndef _BT_SDIO_H_
#define _BT_SDIO_H_
/** Initialize BT driver
 *
 * This function registers BT driver with the mdev interface.
 * It initializes the BT firmware.
 * Note that firmware needs to be already downloaded
 * through the wifi driver.
 *
 * \return WM_SUCCESS on success, -WM_FAIL otherwise.
 */
int32_t bt_drv_init(void);

/** De-initialize BT driver
 *
 * This function de-registers BT driver.
 *
 */
void bt_drv_deinit();

/** Register the receive callback
 *
 * This function is used to register a callback with the
 * BT driver which will be called when a valid HCI packet
 * is received from the card.
 *
 * \param[in] cb_func Pointer to the callback function.
 *
 * \return Always returns WM_SUCCESS.
 */
int32_t bt_drv_set_cb(void (*cb_func)(uint8_t pkt_type, uint8_t *data,
							uint32_t size));

/** Send a HCI packet
 *
 * This function is used to send a HCI packet to the card.
 *
 * \param[in] pkt_type HCI Packet Type.
 * \param[in] data Pointer to the packet contents.
 * \param[in] size Length of the packet in bytes.
 *
 * \return WM_SUCCESS on success, -WM_FAIL otherwise.
 */
int32_t bt_drv_send(uint8_t pkt_type, uint8_t *data, uint32_t size);
#endif /* !_BT_SDIO_H_ */
