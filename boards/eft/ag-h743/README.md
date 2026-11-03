# EFT ag-h743 board (PX4)

Provisional name. The schematic is `WK_FMU-2.0`, and every pin here comes from its netlist.

STM32H743VIT6, 16 MHz HSE, board_id 9903. Build: `make eft_ag-h743_default`.

Status: builds; not flashed or bench-tested. Scope is first flight only, without the PMU.

## Schematic labels to confirm

The hardware has been in service for over two years without problems, so these look like
schematic labelling issues. They do not block power-on or flight; hardware has been asked what
the boards actually carry.

- **C3 and C4 read 2.2 uF on the 16 MHz crystal**, which a distributor lists with a 12 pF load.
  If HSE ever fails to start, `stm32_stdclockconfig()` leaves the MCU on HSI without the PLL, so
  baud rates are wrong and the console looks garbled.
- **U4 (74LVC2G240) has 2A, 2OE# and 2Y grounded**, which as drawn drives the unused gate's
  output into ground. SBUS goes through its other gate.

## Flashing

There is no USB and no bootloader: the application is linked at `0x08000000`. Flash the `.bin`
over SWD (P1: 3.3V, GND, PA14 CLK, PA13 DATA). NRST and BOOT0 are not on the header, so do not
reuse PA13/PA14 in firmware.

## Serial ports

| NuttX | MCU | TX/RX | FMU1 pins (TX/RX) | Schematic | PX4 role |
|---|---|---|---|---|---|
| ttyS0 | USART1 | PB14/PB15 | 10/8 | UM482 (a UM982) | GPS1 |
| ttyS1 | USART2 | PD5/PD6 | 22/20 | PMU | none yet |
| ttyS2 | USART3 | PB10/PB11 | 16/14 | FMU | TEL1 |
| ttyS3 | UART7 | PE8/PE7 | 13/15 | REC | console, 57600 |
| ttyS4 | UART8 | -/PE0 | -/21 | SBUS | RC |

The nets `UM482_*` and `PMU_*` are named from the far end (`UM482_TX` goes to the MCU RX).
Hardware says the receiver on that net is a UM982 (RTK). USART2 stays enabled so the numbering
does not move when the PMU is added.
Using REC as the console is a bring-up choice: there is no dedicated debug port, and FMU1 is the
only way out, so the console needs whatever board FMU1 plugs into. What REC connects to is not
known yet; move the console if it is in use.

## PWM outputs

| Output | Pin | Timer | FMU1 | Output | Pin | Timer | FMU1 |
|---|---|---|---|---|---|---|---|
| 1 | PD15 | TIM4_CH4 | 25 | 5 | PA0 | TIM5_CH1 | 33 |
| 2 | PD14 | TIM4_CH3 | 27 | 6 | PA1 | TIM5_CH2 | 35 |
| 3 | PD13 | TIM4_CH2 | 29 | 7 | PA2 | TIM5_CH3 | 37 |
| 4 | PD12 | TIM4_CH1 | 31 | 8 | PA3 | TIM5_CH4 | 39 |

All outputs go through an 8-bit level shifter whose OE# is PE10 (active low, 4.7k pull-up). The
lines are high impedance from reset until the end of `board_app_initialize()`, and again on every
`board_on_reset()`. Nothing pulls the ESC side low while they are disconnected, so check how the
ESCs behave on a floating line. DIR is tied high, so bidirectional DShot cannot work.

If `stm32_sdio_initialize()` fails, `board_app_initialize()` returns before it drives PE10 low, so
the outputs stay disabled: no PWM points at the SD NAND first. PWM1 to PWM4 share TIM4 and PWM5 to
PWM8 share TIM5, so each group runs at one rate (`PWM_MAIN_TIM0`, `PWM_MAIN_TIM1`). Keep 50 Hz
pumps or servos out of a motor group.

## Sensors

| Device | Bus | CS | Start |
|---|---|---|---|
| BMI088 accel | SPI2 (PA9, PC1, PC2_C) | PC0 | `bmi088 -A -s -b 2 start` |
| BMI088 gyro | SPI2 | PA8 | `bmi088 -G -s -b 2 start` |
| SPL06 | SPI2 | PB4 | `spl06 -s -b 2 start` |

The BMI088 INT pins are not connected, so the driver reads on a timer. Rotation (`-R`) is not
set. Hardware says the board is mounted vertically, perpendicular to the ground; which edge faces
the nose and which faces up is still open, and can be measured on the board if no layout arrives.

## Hardware-specific behaviour

- PLL1 uses the 8 to 16 MHz input range and flash uses 4 wait states, as RM0433 asks for a 16 MHz
  reference and a 240 MHz ACLK. Other PX4 H7 boards use the 4 to 8 MHz range and 2 wait states, so
  compare CPU load if this board looks slower.
- `stm32_boardinitialize()` closes the PC2 analog switch; without that SPI2 may read nothing on
  MISO. Sources disagree on its reset state for this package, and this has not been tried on a board.
- SBUS is inverted by U4, so `RC_INVERT_INPUT` is defined as a no-op. Otherwise `rc_input` would
  invert the UART again while scanning for SBUS.
- No magnetometer and no I2C on this board. The airframe's compass is in the M9N module on
  GNSS_CAN, which needs DroneCAN (see below). Until then `rc.board_defaults` takes heading from
  dual antenna GNSS (`GPS_1_PROTOCOL` 6, `EKF2_GPS_CTRL` 15, `EKF2_MAG_TYPE` 5, `SYS_HAS_MAG` 0).
  The driver recognises the UM982 by `UNIAGRICA` and requests `UNIHEADINGA` from there, which
  matches the receiver hardware confirmed. Test it before relying on it. The heading is also
  dropped until `SENS_GNSS0_HDG` is 2 and both antenna positions are set (`SENS_GNSS0_OFFX/Y/Z`,
  `SENS_GNSS0_AUXX/Y/Z`), and while the receiver's baseline differs from them by more than 20%.
  Those depend on the airframe, so they are not board defaults.
- No battery measurement, so `board_adc` and `battery_status` are not built. Arming needs
  `CBRK_SUPPLY_CHK` = 894281, which disables all power checks: set it in the test parameter file
  for bench tests only. It is deliberately not a default. The data can only come in over a serial
  port or CAN.
- There is no FRAM, so parameters live on the soldered SD NAND (`/fs/microsd/params`).
- Logs share that SD NAND (about 512 MB, no card slot). `rc.board_defaults` sets
  `SDLOG_MAX_SIZE` 40 and `SDLOG_ROTATE` 95: with the 1 GiB default the cleanup target exceeds
  the device, and each new log would delete every older one. Without USB, logs come out over
  MAVLink FTP on a serial port, which is slow at 57600 baud.
- LED cathodes are on PE4 to PE6. Their anode goes to 5V through 430R, so the pins are open drain:
  a 3.3V push-pull high could leave the red LED glowing. The schematic ties that anode to the net
  `+5V_BUS`, which connects to nothing else, so the LEDs may not light at all.

## Not enabled yet

`GPIO_CAN1_*` and `GPIO_CAN2_*` in `board.h` follow the pins, not the net names: net CAN2 is
FDCAN1 (Motor_CAN) and net CAN1 is FDCAN2 (GNSS_CAN). The S pins (PD8, PA10) are pulled high,
which is silent mode; drive them low before enabling `uavcan`. The M9N and compass module sits on
GNSS_CAN, which is FDCAN2, so `CONFIG_BOARD_UAVCAN_INTERFACES` has to be 2 to reach it.

Also not enabled: PMU protocol, RTCM input on PB5 (UART5 RX, receive only; not needed, since the
correction radio connects to the UM982 directly), W25Q16 on SPI4
(PE11 to PE14), hardware watchdog.

## Open items

The vehicle is a quad, so the four motors fit in the TIM4 group (PWM1 to PWM4) and PWM5 to PWM8
stay free for 50 Hz loads. Still open: board name; FMU1 power source; which edge faces the nose
and up for `-R` (`SENS_BOARD_ROT` until then); which motor is on which output; PWM rate and range
across the five or more ESC models (all PWM, no DShot or CAN); battery data source; whether the M9N
and compass module on GNSS_CAN speaks DroneCAN, and at what bitrate; UM982 antenna baseline.
