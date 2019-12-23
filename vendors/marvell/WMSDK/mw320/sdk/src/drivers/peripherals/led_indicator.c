/** @file led_indicator.c
 *
 *  @brief This file provides  LED indicator and callback functions
 *
 *  (C) Copyright 2008-2018 Marvell International Ltd. All Rights Reserved.
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


#include <wmstdio.h>
#include <wm_os.h>
#include <lowlevel_drivers.h>
#include <mdev_pinmux.h>
#include <board.h>
#include <led_indicator.h>
#include <wmlog.h>

/*-----------------------Global declarations----------------------*/
struct led_private_data {
	output_gpio_cfg_t led_cfg;
	uint8_t curr_state;
	uint8_t cnt;
	os_timer_t timer;
	int on_duty_cycle;
	int off_duty_cycle;
} led_data[LED_COUNT];

static int decide_led_array_index(output_gpio_cfg_t led_cfg)
{
	int i;
	for (i = 0; i < LED_COUNT; i++) {
		if (led_data[i].led_cfg.gpio == led_cfg.gpio) {
			return i;
		} else if (led_data[i].led_cfg.gpio == 0) {
			led_data[i].led_cfg = led_cfg;
			return i;
		}
	}
	return -WM_FAIL;
}

static void __led_on(output_gpio_cfg_t led)
{
	GPIO_PinMuxFun(led.gpio, pinmux_drv_get_gpio_func(led.gpio));
	GPIO_SetPinDir(led.gpio, GPIO_OUTPUT);
	if (led.type == GPIO_ACTIVE_LOW)
		GPIO_WritePinOutput(led.gpio, GPIO_IO_LOW);
	else
		GPIO_WritePinOutput(led.gpio, GPIO_IO_HIGH);
}
void led_on(output_gpio_cfg_t led)
{
	if (led.gpio < 0)
		return;

	int idx = decide_led_array_index(led);
	int ret;

	if (idx == -WM_FAIL)
		return;

	if (led_data[idx].timer) {
		if (os_timer_is_running(&led_data[idx].timer))
			os_timer_deactivate(&led_data[idx].timer);
		ret = os_timer_delete(&led_data[idx].timer);
		if (ret != WM_SUCCESS) {
			wmlog_e("led", "Unable to delete LED timer");
			return;
		}
	}
	led_data[idx].curr_state = LED_ON;
	led_data[idx].cnt = 0;
	__led_on(led);
}

static void __led_off(output_gpio_cfg_t led)
{
	GPIO_PinMuxFun(led.gpio, pinmux_drv_get_gpio_func(led.gpio));
	GPIO_SetPinDir(led.gpio, GPIO_OUTPUT);
	if (led.type == GPIO_ACTIVE_LOW)
		GPIO_WritePinOutput(led.gpio, GPIO_IO_HIGH);
	else
		GPIO_WritePinOutput(led.gpio, GPIO_IO_LOW);
}
void led_off(output_gpio_cfg_t led)
{
	if (led.gpio < 0)
		return;

	int idx = decide_led_array_index(led);
	int ret;

	if (idx == -WM_FAIL)
		return;
	if (led_data[idx].timer) {
		if (os_timer_is_running(&led_data[idx].timer))
			os_timer_deactivate(&led_data[idx].timer);

		ret = os_timer_delete(&led_data[idx].timer);
		if (ret != WM_SUCCESS) {
			wmlog_e("led", "Unable to delete LED timer");
			return;
		}
	}
	led_data[idx].cnt = 0;
	led_data[idx].curr_state = LED_OFF;
	__led_off(led);
}

static void led_cb(os_timer_arg_t handle)
{
	int tid = (int) os_timer_get_context(&handle);
	if (tid >= LED_COUNT)
		return;
	if (led_data[tid].curr_state == LED_ON) {
		os_timer_change(&led_data[tid].timer,
				led_data[tid].off_duty_cycle, -1);
		os_timer_activate(&led_data[tid].timer);
		__led_off(led_data[tid].led_cfg);
		led_data[tid].curr_state = LED_OFF;
	} else {
		os_timer_change(&led_data[tid].timer,
				led_data[tid].on_duty_cycle, -1);
		os_timer_activate(&led_data[tid].timer);
		__led_on(led_data[tid].led_cfg);
		led_data[tid].curr_state = LED_ON;
	}
	/* If cnt is 0, it means that the LED should keep blinking.
	 * So, these checks are skipped. Else, we check the count value
	 * to decide if we need to stop the blinks.
	 */
	if (led_data[tid].cnt > 0) {
		led_data[tid].cnt--;
		if (led_data[tid].cnt == 0) {
			/* Calling explicit led_on/off will disable
			 * the timer and set the LED to appropriate
			 * state.
			 */
			if (led_data[tid].curr_state == LED_ON)
				led_on(led_data[tid].led_cfg);
			else
				led_off(led_data[tid].led_cfg);
			return;
		}
	}
}

void led_blink_with_count(output_gpio_cfg_t led, int on_duty_cycle,
		int off_duty_cycle, int count)
{
	if (led.gpio < 0)
		return;

	int err, idx;

	idx = decide_led_array_index(led);

	if (idx == -WM_FAIL)
		return;

	if (os_timer_is_running(&led_data[idx].timer)) {
		err = os_timer_delete(&led_data[idx].timer);
		if (err != WM_SUCCESS)
			return;
	}
	led_data[idx].curr_state = led_get_state(led);
	led_data[idx].on_duty_cycle = on_duty_cycle;
	led_data[idx].off_duty_cycle = off_duty_cycle;
	/* The delta is for cases wherein a blink routine is already in
	 * progress when a new request is received.
	 * Adding delta (0/1) makes sure that the state of the LED after
	 * the blink routine is same as the one before the routine by making
	 * total state change counts even.
	 */
	uint8_t delta = 0;
	if (led_data[idx].cnt)
		delta = led_data[idx].cnt % 2;
	/* Multiply cnt by 2 since a count of 1 consists of
	 * 1 On and 1 Off event (i.e. a total of 2) and then add
	 * delta.
	 */
	led_data[idx].cnt = (2 * count) + delta;

	/* First we create a timer which will timeout immediately
	 * (after 10msec). The further state transitions are then
	 * managed in the timer callback. This has been done so as
	 * to avoid state changes from 2 different thread contexts,
	 * thereby maintaining correct state
	 */
	err = os_timer_create(&led_data[idx].timer,
			      "led-timer",
			      os_msec_to_ticks(10),
			      led_cb,
			      (void *)idx,
			      OS_TIMER_ONE_SHOT,
			      OS_TIMER_AUTO_ACTIVATE);
	if (err != WM_SUCCESS)
		return;
}
void led_blink(output_gpio_cfg_t led, int on_duty_cycle, int off_duty_cycle)
{
	led_blink_with_count(led, on_duty_cycle, off_duty_cycle, 0);
}

int led_get_state(output_gpio_cfg_t led)
{
	if (led.gpio < 0)
		return -WM_FAIL;

	GPIO_IO_Type led_status = GPIO_ReadPinLevel(led.gpio);
	if (led.type == GPIO_ACTIVE_LOW) {
		if (led_status == GPIO_IO_LOW)
			return LED_ON;
		else
			return LED_OFF;
	} else if (led.type == GPIO_ACTIVE_HIGH) {
		if (led_status == GPIO_IO_LOW)
			return LED_OFF;
		else
			return LED_ON;

	}
	return -WM_FAIL;
}
