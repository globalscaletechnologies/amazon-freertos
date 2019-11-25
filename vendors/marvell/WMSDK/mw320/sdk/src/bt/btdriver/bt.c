/** @file bt.c
*
*  @brief BLE Init
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

#include <board.h>
#include <bt.h>
#include <mdev_uart.h>

/*
 * This function is used by the application to download BT firmware
 * and initialise the BT stack.
 * It calls the init function which downloads the firmware as per the
 * selected board. The init functions are defined
 * in bt_sdio.c and bt_uart.c respectively.
 * Stack is initialised for both the boards.
 */
int bt_init()
{
	/* Firmware download */
	ble_interface_t ble_interface = board_ble_interface();
	if (ble_interface.ble_init != NULL)
		ble_interface.ble_init();
	else
		return -WM_FAIL;

	return WM_SUCCESS;
}
