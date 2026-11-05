/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

/*
 * PX4 bootloader settings for the EFT ag-h743 (WK_FMU-2.0).
 *
 * The board has no USB (PA11/PA12 carry Motor_CAN), so the bootloader only
 * listens on UART7, the REC pair on FMU1 pins 13/15 that the application uses
 * as its console. Enter it with `reboot -b` from the console, then run
 * Tools/px4_uploader.py on the same USB-TTL adapter at the bootloader baud.
 * With no valid application it stays in the bootloader.
 *
 * See the template in boards/auterion/fmu-v6s/src/hw_config.h for what each
 * constant means.
 */

/* Boot device selection list */
#define SERIAL0_DEV    0x01

#define APP_LOAD_ADDRESS               0x08020000
#define BOOTLOADER_DELAY               5000
#define INTERFACE_USB                  0
#define INTERFACE_USART                1
#define INTERFACE_USART_CONFIG         "/dev/ttyS0,921600"    /* UART7, the only UART in the bootloader build */
#define BOARD_TYPE                     9903                   /* must match firmware.prototype */
#define _FLASH_KBYTES                  (*(uint32_t *)0x1FF1E880)
#define BOARD_FLASH_SECTORS            (15)
#define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

#define OSC_FREQ                       16

/* RGB LED cathodes are open drain, low = on */
#define BOARD_PIN_LED_ACTIVITY         GPIO_LED_BLUE
#define BOARD_PIN_LED_BOOTLOADER       GPIO_LED_GREEN
#define BOARD_LED_ON                   0
#define BOARD_LED_OFF                  1

/* A floating or noisy REC line at power-up must not hold the board in the bootloader */
#define SERIAL_BREAK_DETECT_DISABLED   1

#if !defined(ARCH_SN_MAX_LENGTH)
# define ARCH_SN_MAX_LENGTH 12
#endif

#if !defined(APP_RESERVATION_SIZE)
#  define APP_RESERVATION_SIZE 0
#endif

#if !defined(BOARD_FIRST_FLASH_SECTOR_TO_ERASE)
#  define BOARD_FIRST_FLASH_SECTOR_TO_ERASE 1
#endif

#if !defined(USB_DATA_ALIGN)
# define USB_DATA_ALIGN
#endif

#ifndef BOOT_DEVICES_SELECTION
#  define BOOT_DEVICES_SELECTION SERIAL0_DEV
#endif

#ifndef BOOT_DEVICES_FILTER_ONUSB
#  define BOOT_DEVICES_FILTER_ONUSB SERIAL0_DEV
#endif
