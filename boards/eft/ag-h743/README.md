# EFT ag-h743 board (PX4)

Provisional name: the hardware team has not named this board yet.

STM32H743VIT6, 16 MHz HSE, board_id 9903. Build: `make eft_ag-h743_default`.

Status: builds; not flashed or bench-tested. Scope is first flight only, without the PMU.

Every pin comes from the schematic screenshots, which may be incomplete. micoair/h743-v2 was
only a reference for the hardware team; the schematic wins where they differ.

## Flashing

There is no USB and no bootloader: the application is linked at `0x08000000`. Flash the `.bin`
over SWD (header pins: 3.3V, GND, PA14 CLK, PA13 DATA). NRST and BOOT0 are not on the header,
so do not reuse PA13/PA14 in firmware.

## Serial ports

| NuttX | MCU | Pins (TX/RX) | Schematic | PX4 role |
|---|---|---|---|---|
| ttyS0 | USART1 | PB14/PB15 | UM482 | GPS1 |
| ttyS1 | USART2 | PD5/PD6 | PMU | none yet |
| ttyS2 | USART3 | PB10/PB11 | FMU | TEL1 |
| ttyS3 | UART7 | PE8/PE7 | REC | console, 57600 |
| ttyS4 | UART8 | -/PE0 | SBUS | RC |

The nets `UM482_*` and `PMU_*` are named from the far end (`UM482_TX` goes to the MCU RX).
USART2 stays enabled so the numbering does not move when the PMU is added.
Using REC as the console is a bring-up choice: there is no dedicated debug port.

## PWM outputs

| Output | Pin | Timer | Output | Pin | Timer |
|---|---|---|---|---|---|
| 1 | PD15 | TIM4_CH4 | 5 | PA0 | TIM5_CH1 |
| 2 | PD14 | TIM4_CH3 | 6 | PA1 | TIM5_CH2 |
| 3 | PD13 | TIM4_CH2 | 7 | PA2 | TIM5_CH3 |
| 4 | PD12 | TIM4_CH1 | 8 | PA3 | TIM5_CH4 |

All outputs go through a SN74LVC8T245 whose OE# is PE10 (active low, 4.7k pull-up). The lines
are high impedance from reset until the end of `board_app_initialize()`, and again on every
`board_on_reset()`. Nothing pulls the ESC side low while they are disconnected, so check how
the ESCs behave on a floating line.

## Sensors

| Device | Bus | CS | Start |
|---|---|---|---|
| BMI088 accel | SPI2 (PA9, PC1, PC2) | PC0 | `bmi088 -A -s -b 2 start` |
| BMI088 gyro | SPI2 | PA8 | `bmi088 -G -s -b 2 start` |
| SPL06 | SPI2 | PB4 | `spl06 -s -b 2 start` |

The BMI088 INT pins are not connected, so the driver reads on a timer. Rotation (`-R`) is not
set yet. CSB1 as accel and CSB2 as gyro follows the BMI088 datasheet; confirm against the board.

## Hardware-specific behaviour

- SBUS is inverted by a 74LVC2G240, so `RC_INVERT_INPUT` is defined as a no-op. Otherwise
  `rc_input` would invert the UART again while scanning for SBUS.
- No magnetometer on the schematic. `rc.board_defaults` takes heading from the UM482 dual
  antenna (`GPS_1_PROTOCOL` 6, `EKF2_GPS_CTRL` 15, `EKF2_MAG_TYPE` 5, `SYS_HAS_MAG` 0). Whether
  the UM482 emits the heading messages the driver expects has not been tested.
- No battery measurement and no I2C on the schematic, so `board_adc`, `battery_status` and I2C
  are not built. Arming needs `CBRK_SUPPLY_CHK` = 894281, which disables all power checks: set it
  in the test parameter file for bench and close-range tests only. It is deliberately not a default.
- There is no FRAM, so parameters live on the SD NAND (`/fs/microsd/params`).
- LED anodes go to +5V through 430R, so the colours are active low. Confirm the LEDs turn fully
  off with the pin at 3.3V.

## Not enabled yet

`GPIO_CAN1_*` and `GPIO_CAN2_*` in `board.h` follow the pins, not the net names: net CAN2 is
FDCAN1 (Motor_CAN) and net CAN1 is FDCAN2 (GNSS_CAN). The S pins (PD8, PA10) are pulled high,
which is silent mode; drive them low before enabling `uavcan`.

Also not enabled: PMU protocol, RTCM input on PB5 (UART5 RX), W25Q16 on SPI4.

## Open items

Board name and version, full schematic, magnetometer, battery measurement, value of the crystal
load capacitors C3/C4 (the screenshot reads 2.2 uF), whether PC2_C needs setup to work as a digital
pin, airframe file and motor count.
