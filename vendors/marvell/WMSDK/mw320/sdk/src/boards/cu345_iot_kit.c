/** @file cu345_iot_kit.c
*
*  @brief Board File
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
 * This is a board specific configuration file for
 * AzureWave AW-CU302 evaluation board based on schematics
 * as of Aug 3, 2016.
 */

#include <wmtypes.h>
#include <wmerrno.h>
#include <wm_os.h>
#include <board.h>
#include <lowlevel_drivers.h>
#include <mdev_gpio.h>
#include <io_expander.h>

/* Source module specific board functions for AW-CU302 module */
#include <modules/aw-cu345.c>

int board_cpu_freq()
{
	return 200000000;
}

int board_32k_xtal()
{
	return true;
}

int board_32k_osc()
{
	return false;
}

int board_rc32k_calib()
{
	return false;
}

void board_gpio_power_on()
{
	/* AON and D0 domains are at 1.8 V */
	PMU_ConfigVDDIOLevel(PMU_VDDIO_AON, PMU_VDDIO_LEVEL_1P8V);
	PMU_ConfigVDDIOLevel(PMU_VDDIO_0, PMU_VDDIO_LEVEL_1P8V);
	/* Wakeup push buttons are active low */
	PMU_ConfigWakeupPin(PMU_GPIO22_INT, PMU_WAKEUP_LEVEL_LOW);
	PMU_ConfigWakeupPin(PMU_GPIO23_INT, PMU_WAKEUP_LEVEL_LOW);
	/*COnfigure COEX pins */
	GPIO_PinMuxFun(GPIO_41, GPIO41_GPIO41);
	GPIO_SetPinDir(GPIO_41, GPIO_OUTPUT);
	GPIO_WritePinOutput(GPIO_41, GPIO_IO_LOW);
	GPIO_PinMuxFun(GPIO_39, GPIO39_GPIO39);
	GPIO_SetPinDir(GPIO_39, GPIO_INPUT);
	GPIO_PinMuxFun(GPIO_40, GPIO40_GPIO40);
	GPIO_SetPinDir(GPIO_40, GPIO_INPUT);

	/* PCA9539_INT */
	GPIO_PinMuxFun(GPIO_23, GPIO23_GPIO23);
	GPIO_SetPinDir(GPIO_23, GPIO_INPUT);

	/* NB-IoT BC66 RI (Ring indication) */
	GPIO_PinMuxFun(GPIO_23, GPIO23_GPIO23);
	GPIO_SetPinDir(GPIO_23, GPIO_INPUT);

	/* SC16IS740_INT */
	GPIO_PinMuxFun(GPIO_47, GPIO47_GPIO47);
	GPIO_SetPinDir(GPIO_47, GPIO_INPUT);
}

void board_uart_pin_config(int id)
{
	switch (id) {
	case UART0_ID:
		GPIO_PinMuxFun(GPIO_2, GPIO2_UART0_TXD);
		GPIO_PinMuxFun(GPIO_3, GPIO3_UART0_RXD);
		break;
	case UART1_ID:
		GPIO_PinMuxFun(GPIO_42, GPIO42_UART1_CTSn);
		GPIO_PinMuxFun(GPIO_43, GPIO43_UART1_RTSn);
		GPIO_PinMuxFun(GPIO_44, GPIO44_UART1_TXD);
		GPIO_PinMuxFun(GPIO_45, GPIO45_UART1_RXD);
		break;
	case UART2_ID:
		GPIO_PinMuxFun(GPIO_48, GPIO48_UART2_TXD);
		GPIO_PinMuxFun(GPIO_49, GPIO49_UART2_RXD);
		break;
	}
}

void board_coex_pin_config()
{
	GPIO_PinMuxFun(GPIO_39, PINMUX_FUNCTION_7);
	GPIO_PinModeConfig(GPIO_39, PINMODE_PULLDOWN);
	GPIO_PinMuxFun(GPIO_40, PINMUX_FUNCTION_7);
	GPIO_PinModeConfig(GPIO_40, PINMODE_PULLDOWN);
	GPIO_PinMuxFun(GPIO_41, PINMUX_FUNCTION_7);
	GPIO_PinModeConfig(GPIO_41, PINMODE_PULLDOWN);
}

void board_i2c_pin_config(int id)
{
	switch (id) {
	case I2C0_PORT:
		GPIO_PinMuxFun(GPIO_4, GPIO4_I2C0_SDA);
		GPIO_PinMuxFun(GPIO_5, GPIO5_I2C0_SCL);
		break;
	case I2C1_PORT:
		GPIO_PinMuxFun(GPIO_17, GPIO17_I2C1_SCL);
		GPIO_PinMuxFun(GPIO_18, GPIO18_I2C1_SDA);
		break;
	}
}

void board_usb_pin_config()
{
}

void board_ssp_pin_config(int id, int cs)
{
	/* To do */
	switch (id) {
	case SSP0_ID:
		GPIO_PinMuxFun(GPIO_0, GPIO0_SSP0_CLK);
		if (cs)
			GPIO_PinMuxFun(GPIO_1, GPIO1_SSP0_FRM);
		GPIO_PinMuxFun(GPIO_2, GPIO2_SSP0_TXD);
		GPIO_PinMuxFun(GPIO_3, GPIO3_SSP0_RXD);
		break;
	case SSP1_ID:
		GPIO_PinMuxFun(GPIO_11, GPIO11_SSP1_CLK);
		if (cs)
			GPIO_PinMuxFun(GPIO_12, GPIO12_SSP1_FRM);
		else {
			GPIO_PinMuxFun(GPIO_12, GPIO12_GPIO12);
			GPIO_SetPinDir(GPIO_12, GPIO_INPUT);
		}
		GPIO_PinMuxFun(GPIO_13, GPIO13_SSP1_TXD);
		GPIO_PinMuxFun(GPIO_14, GPIO14_SSP1_RXD);
		break;
	case SSP2_ID:
		break;
	}
}

int board_adc_pin_config(int adc_id, int channel)
{
	/* GPIO 42/43 are used for BLE uart */
	if (channel == ADC_CH0 || channel == ADC_CH1) {
		return -WM_FAIL;
	}
	/* Channel 2 and channel 3 need GPIO 44
	 * and GPIO 45 which are used for
	 * RF control and not available for ADC
	 */
	if (channel == ADC_CH2 || channel == ADC_CH3) {
		return -WM_FAIL;
	}
	/* GPIO47 is used for SC16IS740 */
	if (channel == ADC_CH5) {
		return -WM_FAIL;
	}
	/* GPIO48/49 are used for UART 2 */
	if (channel == ADC_CH6 || channel == ADC_CH7) {
		return -WM_FAIL;
	}
	GPIO_PinMuxFun((GPIO_42 + channel),
			 PINMUX_FUNCTION_1);
	return WM_SUCCESS;
}

void board_dac_pin_config(int channel)
{
	switch (channel) {
	case DAC_CH_A:
		/* For this channel GPIO 44 is needed
		 * GPIO 44 is reserved for  RF control
		 * on this module so channel DAC_CH_A
		 * should not be used.
		 */
		break;
	case DAC_CH_B:
		GPIO_PinMuxFun(GPIO_43, GPIO43_DACB);
		break;
	}
}

output_gpio_cfg_t board_led_1()
{
	output_gpio_cfg_t gcfg = {
		.gpio = GPIO_16,
		.type = GPIO_ACTIVE_HIGH,
	};

	return gcfg;
}

output_gpio_cfg_t board_led_2()
{
	output_gpio_cfg_t gcfg = {
		.gpio = GPIO_27,
		.type = GPIO_ACTIVE_HIGH,
	};

	return gcfg;
}

output_gpio_cfg_t board_led_3()
{
	output_gpio_cfg_t gcfg = {
		.gpio = -1,
	};

	return gcfg;
}

output_gpio_cfg_t board_led_4()
{
	output_gpio_cfg_t gcfg = {
		.gpio = -1,
	};

	return gcfg;
}

int board_button_1()
{
	return GPIO_IOP_N(6);
}

int board_button_2()
{
	return GPIO_IOP_N(7);
}

int board_button_3()
{
	return -WM_FAIL;
}

int board_button_pressed(int pin)
{
	if (pin < 0)
		return false;

	GPIO_SetPinDir(pin, GPIO_INPUT);
	if (GPIO_ReadPinLevel(pin) == GPIO_IO_LOW)
		return true;

	return false;
}

int board_wakeup0_functional()
{
	return true;
}

int board_wakeup1_functional()
{
	return true;
}

unsigned int board_antenna_select()
{
	return 1;
}

ADC_ChannelSource_Type board_adc_interface()
{
	return ADC_CH4;
}

I2C_ID_Type board_peripheral_i2c_id(void)
{
	return I2C0_PORT;
}

int board_io_expander_supports()
{
	return true;
}

int board_io_expander_id()
{
	/* PCA9539 */
	return 0x9539;
}

int board_io_expander_irq()
{
	return GPIO_23;
}

int board_io_expander_i2c_address()
{
	/* PCA9539 i2c address, A0:1, A1:0 */
	return 0x76;
}

uint32_t board_io_expander_pinctrl()
{
	int i;
	uint32_t pinctrl = 0;
	/* for AWS IoT Development Kit (PCA9539R) */
	uint8_t pinctrl_map[16][2] =
	{
		{GPIO_OUTPUT, GPIO_IO_LOW},   /* PCA9539:IO0_0 : NB_PWRKEY */
		{GPIO_INPUT, 0},              /* PCA9539:IO0_1 : NB_PSM_EINT */
		{GPIO_OUTPUT, GPIO_IO_HIGH},  /* PCA9539:IO0_2 : NB_RESETn */
		{GPIO_INPUT, 0},              /* PCA9539:IO0_3 : NB_RI */
		{GPIO_OUTPUT, GPIO_IO_HIGH},  /* PCA9539:IO0_4 : GPS_PWR */
		{GPIO_INPUT, 0},              /* PCA9539:IO0_5 : SEN_DET */
		{GPIO_INPUT, 0},              /* PCA9539:IO0_6 : BUTTON-1 */
		{GPIO_INPUT, 0},              /* PCA9539:IO0_7 : BUTTON-2 */

		{GPIO_INPUT, 0},              /* PCA9539:IO1_0 : PWR_CTL-1 (CHGn) */
		{GPIO_INPUT, 0},              /* PCA9539:IO1_1 : PWR_CTL-2 (ALERTn)*/
		{GPIO_OUTPUT, GPIO_IO_HIGH},  /* PCA9539:IO1_2 : PWR_CTL-3 (PSM)*/
		{GPIO_INPUT, 0},              /* PCA9539:IO1_3 : IO1_3 */
		{GPIO_INPUT, 0},              /* PCA9539:IO1_4 : IO1_4 */
		{GPIO_INPUT, 0},              /* PCA9539:IO1_5 : IO1_5 */
		{GPIO_INPUT, 0},              /* PCA9539:IO1_6 : IO1_6 */
		{GPIO_INPUT, 0},              /* PCA9539:IO1_7 : IO1_7 */
	};

	for (i = 0; i < 16; i++) {
		pinctrl |= (pinctrl_map[i][0] << (16 + i)) |
				   (pinctrl_map[i][1] << i);
	}
	return pinctrl;
}

#ifdef CONFIG_ADK_SUPPORT
I2C_ID_Type board_mfi_i2c_port_id()
{
        return I2C0_PORT;
}

GPIO_IO_Type board_mfi_RST_pin_state()
{
        /*
         * Return the state of the RST pin of the MFI chip. This will help
         * our application software decide the I2C slave address of the
         * MFI chip. Set this to appropriate value even if the RST line is
         * not connected to any GPIO of the board.
         */
        return GPIO_IO_LOW;
}

int board_mfi_RST_pin_gpio_no()
{
        /*
         * This is the GPIO pin on which the RST line is attached. If on
         * your board the RST line is not connected to any GPIO return -1
         * here.
         *
         * Note that appropriate pull-down or pull-up must be present on
         * the RST line for the warm reset to work.
         */
        return -1;
}

bool board_supports_5GHz(void)
{
        return false;
}

int board_console_uart_id(void)
{
        return UART0_ID;
}

#endif

I2C_ID_Type board_sc16is7xx_i2c_port_id()
{
    return I2C0_PORT;
}

int board_sc16is7xx_i2c_address()
{
    return 0x4d;
}

int board_sc16is7xx_intr_pin()
{
    return GPIO_47;
}

int board_gps_power_pin()
{
    return GPIO_IOP_N(4);
}
