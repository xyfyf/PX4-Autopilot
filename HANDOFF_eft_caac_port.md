# 交接：PX4 植保机项目（2026-09-28，北京时间）

> **当前任务（用户 2026-09-28 校正）：先不动手写代码。** 要出一份项目的大致方案，写清人员投入、周期和风险点，用户拿它和老板讨论。
> 下面的板级映射只作为方案的技术依据：它说明飞控板移植工作量可控，也点出厂商定制等风险。板级移植要等方案定了再开工。

目标：用 PX4 二次开发实现翼飞特植保机的基本功能。硬件借鉴现有植保机方案（先用 EFT_CAAC 飞控板），我们在 ArduPilot 上完善过的算法作为借鉴参考，按原理在 PX4 上重新实现，不整体移植。
范围与架构见 `/home/yuli/UAV/docs/PX4植保软件开发基线_20260928.md` 及同日另两篇（这三篇尚未提交）。

## 环境

- worktree：`/home/yuli/UAV-work/PX4-agricultural-drone`，分支 `feat/PX4-agricultural-drone`，从 PX4 `dev-algo`（af2e7b4311，v1.18.0-beta1-409）分出
- 子模块用本地引用初始化；仿真、modalai、rl_tools、tflite、cyclonedds、rosidl、mc_raptor 这几类没初始化
- 编译前必须先用主 checkout 的 venv，否则缺 kconfiglib：
  `PATH=/home/yuli/UAV/PX4-Autopilot/.venv/bin:$PATH make <board>_default -j16`
- 已验证：`hkust_nxt-v1_default` 在这个 worktree 里能编过

## 模板

选 `boards/hkust/nxt-v1`：H743、16 MHz HSE、SDMMC1、BMI088 + ICM42688P，与 EFT_CAAC 最接近。
新板建议放 `boards/eft/caac`；模板的 bootloader 相关文件与 mtd.cpp 不要，继续用现有的 ArduPilot bootloader。

## EFT_CAAC → PX4 映射

来源：`ardupilot/libraries/AP_HAL_ChibiOS/hwdef/EFT_CAAC/hwdef.dat`

| 项目 | EFT_CAAC | PX4 写法 / 注意 |
|---|---|---|
| 板型 ID | AP_HW_EFT_CAAC = 9902 | `firmware.prototype` 的 board_id 填 9902，才能用现有 bootloader 刷写 |
| Flash | 起始预留 128 KB 给 bootloader；AP 参数存在第 14、15 扇区 | 链接地址 0x08020000（与模板相同）；参数放 SD 卡 |
| SPI1 | PA5/PA6/PA7；BMI088 加速度计 CS=PC4，陀螺 CS=PA4；无 DRDY | 引脚复用 SPI1_*_1；`bmi088 -A/-G -s -b 1` |
| SPI2 | SCK PD3，MISO PB14，MOSI PC1；ICM42688 CS=PE12 | SPI2_SCK_5 / SPI2_MISO_1 / SPI2_MOSI_2；不用 SPI3 |
| I2C1（内部） | PB8/PB9；QMC5883P 0x2C（ROLL_180）；SPL06 0x77 | I2C1_SCL_2 / I2C1_SDA_2；`qmc5883p -I -R 8`，`spl06 -I -a 0x77` |
| CAN | CAN1 PD0/PD1，CAN2 PB5/PB6 | CAN1_*_3 / CAN2_*_2；模板把 UART4 放在 PD0/PD1，必须改 |
| USART1 | PA9/PA10，COM1；AP 用作 stdout，波特率 57600 | 串口控制台 → ttyS0。COM1 平时接什么设备待核实 |
| USART2 | PA2/PA3，GPS1 | ttyS1 → GPS1 |
| USART3 | PD8/PD9，Remote ID | ttyS2 |
| UART4 | PA0/PA1，COM2 | UART4_*_2 → ttyS3 |
| UART5 | PB13/PB12，ELRS | UART5_*_1 → ttyS4 → RC |
| UART7 | PE8/PE7，RTK（UM982） | ttyS5 → GPS2 |
| UART8 | PE1/PE0，3DR 数传 | UART8_*_1 → ttyS6 → TEL1 |
| PWM（12 路） | TIM1 CH1/4/3/2 = PA8/PE14/PE13/PE11；TIM2 CH4/3/2 = PB11/PB10/PB3；TIM3 CH4/3/1 = PB1/PB0/PB4；TIM4 CH4/3 = PD15/PD14 | 输出顺序照 PWM(1..12)；上电前 6 路输出 1000us/400Hz |
| HRT | AP 系统定时器用 TIM5 | 模板的 HRT 用 TIM8，但本板 TIM8_CH1（PC6）是 WS2812 灯带 → HRT 改用 TIM5 |
| LED | 红 PE6，绿 PE5，蓝 PC5，高电平亮 | 模板 led.c 是低电平亮（`!state`），必须改极性 |
| 蜂鸣器 | 无 | 关掉 TONE_ALARM |
| ADC | PC0 电池电压（ADC1 INP10），分压系数 31.0；无电流采样 | BOARD_BATTERY1_V_DIV 31.0；电流通道不配 |
| USB | PA11/PA12，无 VBUS 检测脚 | BOARD_ADC_USB_CONNECTED 需要另想办法（其它板有 `board_read_VBUS_state()` 的写法） |
| SD 卡 | SDMMC1 标准引脚 | 与模板相同 |

## 厂商定制（App 与合规依赖，要单独排期）

MAVLink 起始字节 0xEF（仅 USB 和 3DR 口）、`.EFT` 加密日志、链路加密、Remote ID/UOM 合规链、COMPASS_UBLOX_GPS。
在 PX4 上要逐项复刻，或确认可以舍弃。

## 还没做

板级文件还一个都没写（上一轮会话写了一半的副本已删除），也还没编译或上台架验证。
