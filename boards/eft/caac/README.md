# EFT_CAAC board (PX4)

Ported from the ArduPilot `EFT_CAAC` hwdef. STM32H743, 16 MHz HSE, board_id 9902
(uses the existing bootloader). Build: `make eft_caac_default`.

Status: builds; not yet flashed or bench-tested.

## Serial ports

| NuttX | MCU | Function | PX4 role |
|---|---|---|---|
| ttyS0 | USART1 | COM1 (console, 57600) | TEL3 |
| ttyS1 | USART2 | GPS1 | GPS1 |
| ttyS2 | USART3 | Remote ID | - |
| ttyS3 | UART4 | COM2 | TEL2 |
| ttyS4 | UART5 | ELRS | RC |
| ttyS5 | UART7 | RTK (UM982) | GPS2 |
| ttyS6 | UART8 | 3DR telemetry | TEL1 |

## PWM outputs

PX4 needs the channels of one timer to be consecutive, so the order differs from
the ArduPilot `PWM(n)` numbering. See the table in `src/timer_config.cpp`.

| PX4 | Pin | AP PWM | PX4 | Pin | AP PWM |
|---|---|---|---|---|---|
| 1 | PA8 | 1 | 7 | PB4 | 9 |
| 2 | PE11 | 6 | 8 | PB0 | 8 |
| 3 | PE13 | 5 | 9 | PB1 | 7 |
| 4 | PE14 | 4 | 10 | PD14 | 12 |
| 5 | PB3 | 10 | 11 | PD15 | 11 |
| 6 | PB10 | 3 | 12 | PB11 | 2 |

PWM pins are pulled down at boot (as in the ArduPilot hwdef). The ArduPilot hwdef
additionally drives PWM1-6 at 1000 us/400 Hz right after timer start; PX4 has no
equivalent yet, so check ESC behaviour at power-up on the bench.

## Bench bring-up checklist

1. Flash; console on ttyS0 (USART1, 57600) shows the boot log.
2. `ver all`, `dmesg`: no driver errors.
3. `listener sensor_accel`, `sensor_gyro`, `sensor_mag`, `sensor_baro`: values sane,
   axes correct (the QMC5883P is mounted ROLL_180, `-R 8`).
4. GPS1, RTK (GPS2), ELRS (RC), telemetry (TEL1): data present.
5. PWM: `actuator_test`, check each output against the table above, with props off.
6. CAN1/CAN2: `uavcan status` with a node attached (only compiled so far).
7. Battery voltage reading against a meter (divider 31.0, no current sense).

## Open items

- No VBUS sense pin; `board_read_VBUS_state()` always reports connected. Ask the
  hardware team whether detection is possible.
- Flash is 90% used; trim unused modules before adding agricultural features.
- UART DMA is inherited from the template; UART8 has no DMA mapping.
- WS2812 strip (PC6/TIM8) is not driven.
- Vendor-specific features (MAVLink start byte 0xEF, encrypted logs, link
  encryption, Remote ID chain) are not ported.
