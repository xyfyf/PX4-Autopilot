/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * Board internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

// #define FLASH_BASED_PARAMS


/* RGB LED, active high */
#  define GPIO_LED_RED          /* PE6 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN6)
#  define GPIO_LED_GREEN        /* PE5 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN5)
#  define GPIO_LED_BLUE         /* PC5 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN5)

#  define BOARD_HAS_CONTROL_STATUS_LEDS      1
#  define BOARD_OVERLOAD_LED     LED_RED
#  define BOARD_ARMED_STATE_LED  LED_BLUE

/* I2C busses */
/* Devices on the onboard buses.
 *
 * Note that these are unshifted addresses.
 */
// #define PX4_I2C_OBDEV_SE050         0x48

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

#define ADC12_CH(n)		(n)

#define PX4_ADC_GPIO  \
	/* PC0  */  GPIO_ADC123_INP10

/* Battery voltage only (PC0 = ADC1 INP10); there is no current sense */
#define ADC_BATTERY_VOLTAGE_CHANNEL             ADC12_CH(10)
#define ADC_BATTERY_CURRENT_CHANNEL             ((uint8_t)(-1))

#define ADC_CHANNELS \
	(1 << ADC_BATTERY_VOLTAGE_CHANNEL)

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* Voltage divider from the EFT_CAAC hwdef (HAL_BATT_VOLT_SCALE 31.0) */
#define BOARD_BATTERY1_V_DIV         (31.0f)

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   12


/* No buzzer on this board */

/* USB OTG FS: no VBUS sensing pin on this board.
 * TODO: decide how to detect USB connection (see HANDOFF_eft_caac_port.md).
 */

/* High-resolution timer */
#define HRT_TIMER               5  /* TIM8_CH1 (PC6) drives the WS2812 strip, so HRT uses TIM5 */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

/* RC Serial port */
#define RC_SERIAL_PORT                     "/dev/ttyS4"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_LIB_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */
#define SDIO_SLOTNO             0  /* Only one slot */
#define SDIO_MINOR              0
#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_LIB_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO, \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 4


__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
