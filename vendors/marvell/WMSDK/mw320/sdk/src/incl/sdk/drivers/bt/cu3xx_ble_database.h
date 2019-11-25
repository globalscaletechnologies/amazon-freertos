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

/** @file cu3xx_ble_database.h
*
*  @brief This file contains BLE application supporting data
*/

#ifndef __BLE_DATABASE_H__
#define __BLE_DATABASE_H__

#define SIZEOF_RECORD_UUID_16BIT     2
#define SIZEOF_RECORD_UUID_32BIT     4
#define SIZEOF_RECORD_UUID_128BIT   16

typedef struct {
	int uuid_len;
	union {
		uint16_t uuid16;
		uint32_t uuid32;
		uint8_t uuid128[16];
	} uuid;
} rec_uuid_t;

/** Common Request callbacks
 *
 * This function can be used as common callback for requests.
 *
 * \param[in] respbuf response buffer pointer.
 * \param[in] len response buffer length.
 *
 */
void common_req_cb(uint8_t *respbuf, int len);

/** Add BLE service
 *
 * This function adds BLE service
 *
 * \param[in] sid id for service
 * \param[in] serv_uuid uuid for service
 * \param[in] total_size total size of the service
 * \param[in] n_16 count of 16 bit record TLVs
 * \param[in] n_32 count of 32 bit record TLVs
 * \param[in] n_128 count of 128 bit record TLVs
 * \param[in] handle handle for service.
 *
 * \return WM_SUCCESS on success
 */
int ble_add_service(uint16_t sid, rec_uuid_t *serv_uuid, uint16_t total_size,
		    uint16_t n_16, uint16_t n_32, uint16_t n_128,
		    uint16_t *handle);

/** Add BLE characteristic
 *
 * This function adds BLE characteristics
 *
 * \param[in] char_uuid pointer for structure of characteristic uuid. It includes length and value of uuid.
 * \param[in] permissions permission for characterisic.
 * \param[in] serv_handle service handle in which characteristic is present.
 * \param[in] char_size size of characteristics.
 * \param[in] handle handle for characteristic.
 *
 * \return WM_SUCCESS on success
 */
int ble_add_characteristic(rec_uuid_t *char_uuid, uint8_t permissions,
			   uint16_t serv_handle, uint16_t char_size,
			   uint16_t *char_handle);

int ble_add_characteristic_descriptor(rec_uuid_t *char_uuid,
				      uint8_t permissions,
				      uint16_t serv_handle,
				      uint16_t char_size,
				      void *data, int data_len,
				      uint16_t *char_handle);

/** Set BLE advertisement interval
 *
 * This function sets BLE advertisement interval
 *
 * \param[in] min_intv minimum value of interval to be set
 * \param[in] max_intv maximum value of interval to be set
 *
 * \return WM_SUCCESS on success
 */
int ble_set_adv_int(uint16_t min_intv, uint16_t max_intv);

/** BLE disconnect
 *
 * This function sends request to disconnect BLE.
 */
void ble_disconnect();

/** BLE stop advertisement
 *
 * This function stops BLE advertisement
 *
 * \return WM_SUCCESS on success
 */
int ble_stop_advertisement();

/** Set BLE advertisement data
 *
 * This function sets BLE advertisement data
 *
 * \param[in] adv_data uint8_t pointer to data to be advertised.
 * \param[in] len length of data to be aadvertsed.
 *
 * \return WM_SUCCESS on success
 */
int ble_set_advertisement_data(uint8_t *adv_data, int len);

int ble_set_scan_data(uint8_t *scan_data, int len);

/** BLE start advertisement
 *
 * This function startsBLE advertisement
 *
 * \return WM_SUCCESS on success
 */
int ble_start_advertisement();
#endif
