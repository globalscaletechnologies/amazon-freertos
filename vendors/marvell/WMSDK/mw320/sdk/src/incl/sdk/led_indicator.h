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


/** @file led_indicator.h
*
*  @brief This file contains LED indicator helper functions
*/

#ifndef __LED_INDICATOR_H__
#define __LED_INDICATOR_H__

#include <generic_io.h>
#define LED_COUNT 8

/** State of the LED */
enum led_state {
	/** LED is Off */
	LED_OFF = 0,
	/** LED is On */
	LED_ON,
};

/** Switch ON LED
 *
 * \param[in] led The output LED GPIO configuration of type
 * \ref output_gpio_cfg_t
 */
void led_on(output_gpio_cfg_t led);

/** Switch OFF LED
 *
 * \param[in] led The output LED GPIO configuration of type
 * \ref output_gpio_cfg_t
 */
void led_off(output_gpio_cfg_t led);

/** Blink the LED with given duty cycle
 *
 * \param[in] led The output LED GPIO configuration of type
 * \ref output_gpio_cfg_t
 * \param[in] on_duty_cycle Time in millisec for which LED will be
 * ON in blinking cycle
 * \param[in] off_duty_cycle Time in millisec for which LED will be
 * OFF in blinking cycle
 */
void led_blink(output_gpio_cfg_t led, int on_duty_cycle, int off_duty_cycle);

/** Blink the LED with given duty cycle and for given counts
 *
 * \param[in] led The output LED GPIO configuration of type
 * \ref output_gpio_cfg_t
 * \param[in] on_duty_cycle Time in millisec for which LED will be
 * ON in blinking cycle
 * \param[in] off_duty_cycle Time in millisec for which LED will be
 * OFF in blinking cycle
 * \param[in] count Number of times for which the LED must blink.
 */
void led_blink_with_count(output_gpio_cfg_t led, int on_duty_cycle,
		int off_duty_cycle, int count);

/** Get the current LED state
 *
 * \param[in] led The output LED GPIO configuration of type
 * \ref output_gpio_cfg_t
 *
 * \return \ref LED_ON if the LED is On
 * \return \ref LED_OFF if the LED is Off
 * \return -WM_FAIL if invalid parameter specified
 */
int led_get_state(output_gpio_cfg_t led);

/** Fast Blink the LED
 *
 *  Blink LED with on_duty_cycle = 200ms and off_duty_cycle = 200ms
 *
 *  \param[in] led The output LED GPIO configuration of type
 *  \ref output_gpio_cfg_t
 */
static inline void led_fast_blink(output_gpio_cfg_t led)
{
	led_blink(led, 200, 200);
}

/** Slow Blink the LED
 *
 *  Blink LED with on_duty_cycle = 1000ms and off_duty_cycle = 1000ms
 *
 *  \param[in] led The output LED GPIO configuration of type
 *  \ref output_gpio_cfg_t
 */
static inline void led_slow_blink(output_gpio_cfg_t led)
{
	led_blink(led, 1000, 1000);
}
#endif /* ! __LED_INDICATOR_H__ */
