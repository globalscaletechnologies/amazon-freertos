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

/** @file aws_utils.h
*
*  @brief This file contains AWS configuration APIs
*/

#ifndef _AWS_UTILS_H_
#define _AWS_UTILS_H_

#define AWS_PUB_CERT_SIZE 2046
#define AWS_PRIV_KEY_SIZE 2046
#define AWS_MAX_REGION_SIZE 126
#define AWS_MAX_THING_SIZE 126

/** Read the configured AWS Certificate
 *
 * This API reads the AWS certificate that is configured during provisioning.
 *
 * \param[out] cert Pointer to a buffer that should hold the certificate.
 * \param[in] cert_len The length of the buffer pointed to by cert above
 *
 * \return WM_SUCCESS on success
 * \return error code otherwise
 */
int read_aws_certificate(char *cert, unsigned cert_len);

/** Read the configured AWS Key
 *
 * This API reads the AWS key that is configured during provisioning.
 *
 * \param[out] key Pointer to a buffer that should hold the key.
 * \param[in] key_len The length of the buffer pointed to by key above
 *
 * \return WM_SUCCESS on success
 * \return error code otherwise
 */
int read_aws_key(char *key, unsigned key_len);

/** Read the configured AWS region
 *
 * This API reads the AWS region that is configured during provisioning.
 *
 * \param[out] region Pointer to a buffer that should hold the region.
 * \param[in] region_len The length of the buffer pointed to by region above
 *
 * \return WM_SUCCESS on success
 * \return error code otherwise
 */
int read_aws_region(char *region, unsigned region_len);

/** Read the configured AWS thing name
 *
 * This API reads the AWS thing name that is configured during provisioning.
 *
 * \param[out] thing Pointer to a buffer that should hold the thing name.
 * \param[in] thing_len The length of the buffer pointed to by thing above
 *
 * \return WM_SUCCESS on success
 * \return error code otherwise
 */
int read_aws_thing(char *thing, unsigned thing_len);

/** Read device mac address
 *
 * This API copies the AWS device MAC address to the 6-byte array pointed by
 * \a device_mac. In the event of an error, nothing is copied to \a device_mac.
 *
 * \param[out] device_mac A pointer to a 6-byte array where the MAC address
 *             will be copied.
 *
 * \return WM_SUCCESS on success
 * \return error code otherwise
 */
int read_aws_device_mac(uint8_t *device_mac);

#endif /* ! _AWS_UTILS_H_ */
