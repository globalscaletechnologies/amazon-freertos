/** @file aws_utils.c
*
*  @brief AWS Utilities
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

/*
 * Cloud Utils
 *
 * Summary:
 *
 * Description:
 *
 */

#include <psm-v2.h>
#include <psm-utils.h>
#include <aws_utils.h>
#include <partition.h>
#include <wlan.h>


#define AWS_PUB_CERT  "aws.certificate"
#define AWS_PRIV_KEY  "aws.private_key"
#define AWS_REGION    "aws.region"
#define AWS_THING     "aws.thing"

int read_aws_certificate(char *cert, unsigned cert_len)
{
	int i, j, n;
	char *buf = os_mem_calloc(AWS_PUB_CERT_SIZE);
	if (!buf) {
		wmprintf("Failed to allocate memory");
		return -WM_FAIL;
	}

	n = psm_get_variable(sys_psm_get_handle(), AWS_PUB_CERT, buf,
			     AWS_PUB_CERT_SIZE);
	if (n <= 0 || n > cert_len) {
		os_mem_free(buf);
		return -WM_FAIL;
	}
	for (i = 0, j = 0; i < n; i++) {
		if (buf[i] == 0x5c && buf[i + 1] == 0x6e) {
			cert[j] = 0x0a;
			i++;
		} else {
			cert[j] = buf[i];
		}
		j++;
	}

	os_mem_free(buf);
	return WM_SUCCESS;
}

int read_aws_key(char *key, unsigned key_len)
{
	int i, j, n;
	char *buf = os_mem_calloc(AWS_PRIV_KEY_SIZE);
	if (!buf) {
		wmprintf("Failed to allocate memory");
		return -WM_FAIL;
	}
	n = psm_get_variable(sys_psm_get_handle(), AWS_PRIV_KEY, buf,
			     AWS_PRIV_KEY_SIZE);
	if (n <= 0 || n > key_len) {
		os_mem_free(buf);
		return -WM_FAIL;
	}
	for (i = 0, j = 0; i < n; i++) {
		if (buf[i] == 0x5c && buf[i + 1] == 0x6e) {
			key[j] = 0x0a;
			i++;
		} else {
			key[j] = buf[i];
		}
		j++;
	}
	os_mem_free(buf);
	return WM_SUCCESS;
}

int read_aws_region(char *region, unsigned region_len)
{
	int n;
	n = psm_get_variable(sys_psm_get_handle(), AWS_REGION, region,
			     AWS_MAX_REGION_SIZE);
	if (n <= 0 || n > region_len)
		return -WM_FAIL;
	return WM_SUCCESS;
}

int read_aws_thing(char *thing, unsigned thing_len)
{
	int n;
	n = psm_get_variable(sys_psm_get_handle(), AWS_THING, thing,
			     AWS_MAX_THING_SIZE);
	if (n <= 0 || n > thing_len)
		return -WM_FAIL;
	return WM_SUCCESS;
}

int read_aws_device_mac(uint8_t *device_mac)
{
	return wlan_get_mac_address(device_mac);
}
