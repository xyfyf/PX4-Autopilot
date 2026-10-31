# EFT ag-h743 board (PX4)

Provisional name: the hardware team has not named this board yet.

STM32H743VIT6, 16 MHz HSE, board_id 9903.
Build: `make eft_ag-h743_default` and `make eft_ag-h743_bootloader`.

Status: builds; not flashed or bench-tested.

**This board follows the proposed hardware interface spec V0.1, not the schematic screenshots
received so far.** Those differ on the IMU bus, CAN, USB, console, ADC, I2C and more. For the
screenshots as drawn, see commit 612e0f541d. The spec and the reasons are in the UAV repository:
`proposals/PX4/hardware/硬件接口规格_V0.1草案.md`.

## Flashing

First time: flash `eft_ag-h743_bootloader.bin` at `0x08000000` over SWD. After that, upload the
`.px4` over USB with the PX4 bootloader; the application is linked at `0x08020000`.
The USB VID/PID 0x1209/0x0001 is the pid.codes test pair: replace it before shipping.

## Serial ports

| NuttX | MCU | Pins (TX/RX) | Connected to | PX4 role |
|---|---|---|---|---|
| ttyS0 | USART1 | PB14/PB15 | UM982 | GPS1 |
| ttyS1 | USART2 | PD5/PD6 | PMU | none yet |
| ttyS2 | USART3 | PB10/PB11 | telemetry | TEL1 |
| ttyS3 | USART6 | PC6/PC7 | debug header | console, 57600 |
| ttyS4 | UART7 | PE8/PE7 | spare | TEL2 |
| ttyS5 | UART8 | -/PE0 | SBUS | RC |

## PWM outputs

| Output | Pin | Timer | Output | Pin | Timer |
|---|---|---|---|---|---|
| 1 | PD15 | TIM4_CH4 | 5 | PA0 | TIM5_CH1 |
| 2 | PD14 | TIM4_CH3 | 6 | PA1 | TIM5_CH2 |
| 3 | PD13 | TIM4_CH2 | 7 | PA2 | TIM5_CH3 |
| 4 | PD12 | TIM4_CH1 | 8 | PA3 | TIM5_CH4 |

All outputs go through a SN74LVC8T245 whose OE# is PE10 (active low, 4.7k pull-up). The lines
are high impedance from reset until the end of `board_app_initialize()`, and again on every
`board_on_reset()`. The spec adds a pull-down on each output.

## Sensors

| Device | Bus | CS | DRDY | Start |
|---|---|---|---|---|
| BMI088 accel | SPI1 (PA5, PA6, PA7) | PC0 | PE2 | `bmi088 -A -s -b 1 start` |
| BMI088 gyro | SPI1 | PA8 | PE3 | `bmi088 -G -s -b 1 start` |
| BMI270 | SPI2 (PD3, PC2, PC1) | PD4 | PD7 | `bmi270 -s -b 2 start` |
| SPL06 | SPI2 | PB4 | - | `spl06 -s -b 2 start` |
| QMC5883L | I2C1 external (PB8, PB7) | - | - | `qmc5883l -X -a 0x0D start` |

Rotation (`-R`) is not set yet; it depends on the PCB and the mounting.

## Hardware-specific behaviour

- SBUS is inverted by a 74LVC2G240, so `RC_INVERT_INPUT` is defined as a no-op. Otherwise
  `rc_input` would invert the UART again while scanning for SBUS.
- There is no FRAM yet, so parameters live on `/fs/microsd/params`.
- CAN: FDCAN1 (PD0, PD1) is Motor_CAN and FDCAN2 (PB12, PB13) is GNSS_CAN. The transceiver S pins
  (PA10, PD8) are pulled high in hardware and driven low at boot.
- LED anodes go to +5V through 430R, so the colours are active low.
- Battery: voltage on PC5, current on PC4. `BAT1_V_DIV` and `BAT1_A_PER_V` still need values.

## Bring-up without flying

Motors can be tested without GNSS, compass or RC: with props removed, use `actuator_test` while
disarmed. Use `SYS_AUTOSTART` 6001 (generic hexa X) until an airframe file exists.

## Not enabled yet

PMU protocol, RTCM input on PB5 (UART5 RX), FRAM or W25Q16 on SPI4, PPS capture on PE9, heater on
PB9, hardware version sense on PA4.
