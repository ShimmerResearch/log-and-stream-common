# Shimmer3R Peripheral Allocation

Which STM32U5 peripheral instance serves which part, so that adding a sensor or
a signal does not collide with something already there.

The Shimmer3R counterpart to a bus-allocation map. Read this before assigning a
pin, a timer channel or an EXTI line.

> **Verified against** — the revisions these claims were read from.
>
> - `shimmer3r-firmware` @ `a8f105e5` — `Core/Src/` peripheral initialisers,
>   `Core/Src/spi.c` and `Core/Src/i2c.c` channel configurators,
>   `Shimmer_Driver/hal_FactoryTest.h` (which enumerates the bus-to-part
>   mapping as test IDs).
> - `log-and-stream-common` @ `f3cf73e`.

> **Shimmer3R only.** The Shimmer3 equivalent is an MSP430 allocation and is
> not covered here.

**Source references:**

| Layer | File |
|---|---|
| Peripheral initialisers | `Core/Src/{spi,i2c,adc,tim,sdmmc,usart,gpdma,gpio,crc,rng,iwdg,mdf}.c` |
| Bus-to-part mapping | `Core/Src/spi.c`, `Core/Src/i2c.c`; `Shimmer_Driver/hal_FactoryTest.h` |
| Channel ordering | [SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §4.2 |
| Test IDs | [SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md) §4 |

---

## 1. Sensor bus allocation

This is the authoritative mapping, cross-checked between the channel
configurators and the factory-test registry, which names the bus for each part:

| Bus | Parts | Factory test |
|---|---|---|
| **SPI1** | ADS7028 external ADC, LSM6DSV (low-noise accel + gyro), BMP390 / BMP581 (pressure), ADXL371 (high-g accel) | `S3R_TEST_0015` – `0018` |
| **SPI2** | LIS3MDL (alt. mag), LIS2DW12 (wide-range accel) | `S3R_TEST_0019`, `0020` |
| **SPI3** | ADS1292R (ExG) | `S3R_TEST_0021` |
| **I2C1** | LIS2MDL (mag), CAT24C16 (EEPROM) | `S3R_TEST_0022`, `0023` |
| **I2C4** | CAT24C16 on a test rig, or the GSR fixture | `S3R_TEST_0024`, `0025` |

> **SPI1 is heavily loaded and I2C1 is nearly empty.** Four sampled parts share
> SPI1, including the external ADC that carries every analog channel; I2C1
> carries one sensor and the EEPROM. A new high-rate part should go on SPI2 or
> SPI3 rather than SPI1.

> **Almost everything moved from I2C to SPI relative to Shimmer3.** On Shimmer3
> the gyro, wide-range accel, mag and pressure sensor were all on I2C. On
> Shimmer3R only the LIS2MDL magnetometer remains on I2C1. That is why the
> per-generation channel order differs so much
> ([SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §4).

> **I2C4 is a production-test bus, not a product bus** on most models. Both its
> tests report "not applicable for this model" on hardware without GSR. Do not
> assume it is available for a new part without checking the model matrix.

## 2. Peripheral instances in use

From the `Core/Src` initialisers:

| Peripheral | Instances | Purpose |
|---|---|---|
| SPI | `hspi1`, `hspi2`, `hspi3` | Sensor buses, §1 |
| I2C | `hi2c1`, `hi2c4` | Sensor bus and test bus, §1 |
| ADC | `hadc1`, `hadc2`, `hadc4` | MCU-internal measurements |
| SDMMC | `hsd1` | SD card |
| UART | `huart1`, `huart3`, `huart6` | Bluetooth, dock, debug |
| TIM | `htim1`, `htim2`, `htim3`, `htim6`, `htim7` | Sampling, timeouts, PWM |
| GPDMA | multiple | SPI and SDMMC transfers |
| CRC | `hcrc` | Hardware CRC |
| RNG | `hrng` | |
| IWDG | `hiwdg` | Independent watchdog |
| MDF | `hmdf1` | Microphone (PDM) |

> **The MCU ADCs are not the sensor ADC.** `hadc1`, `hadc2` and `hadc4` measure
> MCU-internal quantities — reference, core voltage, battery pin, die
> temperature, which are `S3R_TEST_0007` through `0010`. Every *sensor* analog
> channel goes through the external ADS7028 on SPI1. This is why
> `ShimBrd_areMcuAdcsUsedForSensing()` gates whether `ADC_configureChannels`
> contributes any channels at all
> ([SHIMMER3_ARCHITECTURE_OVERVIEW.md](SHIMMER3_ARCHITECTURE_OVERVIEW.md) §6).

## 3. Timers

`htim1`, `htim2`, `htim3`, `htim6` and `htim7` are all initialised. `htim2`,
`htim3`, `htim6` and `htim7` are referenced heavily; `htim1` appears once.

> **TIM1 is the one to check before claiming a channel.** It is initialised but
> barely referenced, which makes it look free — and it is the timer whose
> channels are the natural home for an input-capture feature. An event-capture
> investigation identified TIM1_CH1 as the candidate for a hardware capture
> input precisely because the timer is otherwise idle. Confirm the current
> state before assuming either that it is free or that it is not.

## 4. EXTI lines

`Core/Src/gpio.c` enables `EXTI1_IRQn` and `EXTI6_IRQn`.

> **An EXTI line is shared by pin number across every port.** `EXTI9` is
> occupied by VBUS detection, which means **PE9 cannot be used as an interrupt
> source** even though the pin itself is otherwise free. This is the specific
> constraint that stalled an event-capture design: PE9 is available as
> TIM1_CH1, so it can be used for hardware *capture*, but not for a GPIO
> interrupt.

> **Check the EXTI line, not just the pin.** The STM32 external-interrupt
> controller multiplexes by pin number: `PA9`, `PB9`, `PC9`, `PE9` all contend
> for `EXTI9`. A free pin on a taken line is not a free interrupt.

## 5. Adding a peripheral — checklist

1. **Pick a bus with headroom.** SPI2 or SPI3 for a new sampled part; SPI1 is
   the busiest (§1).
2. **Check the pin's alternate functions** for a timer channel or bus signal
   that already claims it.
3. **Check the EXTI line, by pin number, across all ports** (§4).
4. **Check DMA channel availability** — SPI and SDMMC already consume GPDMA
   channels.
5. **Add the channel to the right configurator**, in the position you want it
   in the packet — ordering is by append order, not by channel ID
   ([SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §4).
6. **Add a factory test** and take the next unused ID — do not reuse a gap
   ([SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md) §4).
7. **Keep the Shimmer3 build valid** if you touched anything in `common`
   ([SHIMMER3_BUILD_AND_PROGRAMMING.md](SHIMMER3_BUILD_AND_PROGRAMMING.md) §3).

## Appendix A. Pin map (STM32U5A5QJIxQ, UBGA132)

Extracted from `LogAndStream_Shimmer3R_UBGA132_SMPS.ioc` at the pinned revision.
"Signal" is the CubeMX alternate function; a bare `GPIO_*` signal is a plain
input or output. Pins not listed are unassigned.

| Pin | Signal | Label | Notes |
|---|---|---|---|
| PA0 | TIM2_CH1 | LED_UPR_RD | upper LED red PWM |
| PA1 | GPIO | DOCK_DETECT | EXTI |
| PA2 | USART2_TX | BSL_TX | system-bootloader UART |
| PA3 | USART2_RX | BSL_RX | system-bootloader UART |
| PA6 | EXTI6 | USER_BTN | |
| PA8 | GPIO out | SW_MIC | microphone rail switch |
| PA9 | GPIO in / EXTI9 | USB_VBUS | `INT_LINE_USB_VBUS` |
| PA10 | GPIO out | CS_HIGH_G | high-g accelerometer chip select |
| PA11 | USB_OTG_HS_DM | | internal HS PHY |
| PA12 | USB_OTG_HS_DP | | internal HS PHY |
| PB1 | GPIO in | GPIO_INTERNAL3 | expansion |
| PB2 | GPIO in | GPIO_INTERNAL4 | expansion |
| PB4 | SPI3_MISO | | EXG bus |
| PB5 | SPI3_MOSI | | EXG bus |
| PB6 | USART1_TX | DOCK_TXD | |
| PB7 | USART1_RX | DOCK_RXD | |
| PB8 | I2C1_SCL | | main sensor I2C bus |
| PB9 | I2C1_SDA | | main sensor I2C bus |
| PB10 | TIM2_CH3 | LED_UPR_GR | |
| PB11 | TIM2_CH4 | LED_UPR_BLU | |
| PB12 | EXTI12 | LIS2DW12_INT1 | |
| PB13 | SPI2_SCK | | sensing bus 2 |
| PB14 | SPI2_MISO | | sensing bus 2 |
| PB15 | SPI2_MOSI | | sensing bus 2 |
| PC0 | GPIO out | CS_ADS7028 | external ADC chip select |
| PC2 | GPIO out | CS_LSM6DSV | |
| PC3 | ADC1_IN4 / ADC2_IN4 | VBAT_SENSE | |
| PC4 | GPIO out | SW_SD_MCU_DOCK | SD card mux |
| PC5 | GPIO out | DETECT_N | |
| PC6 | GPIO | BT_CP_ROLE | |
| PC7 | GPIO out | SW_BT | Bluetooth rail switch |
| PC8-PC11 | SDMMC1_D0-D3 | | 4-bit SD bus |
| PC12 | SDMMC1_CK | | |
| PC13 | GPIO | GPIO_INTERNAL0 | EXTI |
| PD0 | GPIO in | SD_DETECT_N | EXTI |
| PD1 | GPIO out | SW_FLASH | |
| PD2 | SDMMC1_CMD | | |
| PD3 | GPIO out | SW_SENSE | sensing rail switch |
| PD4 | GPIO | GPIO_INTERNAL1 | EXTI |
| PD5 | GPIO | GPIO_INTERNAL2 | |
| PD6 | GPIO out | CS_BMP390 | |
| PD7 | GPIO out | CS_LIS2DW12 | |
| PD8 | USART3_TX | BT_TXD | |
| PD9 | USART3_RX | BT_RXD | |
| PD10 | GPIO | BT_LP_MODE | |
| PD11 | USART3_CTS | BT_CTS | |
| PD12 | USART3_RTS | BT_RTS | EXTI when used as GPIO |
| PD13 | GPIO | BT_RST | |
| PD14 | GPIO | BT_CONNECTION | EXTI |
| PD15 | GPIO | BT_CYSPP | EXTI |
| PE0 | GPIO in | GPIO_INTERNAL5 | |
| PE2 | EXTI2 | LIS3MDL_DRDY | |
| PE3 | TIM3_CH1 | LED_LWR_RD | |
| PE4 | TIM3_CH2 | LED_LWR_GR | |
| PE5 | TIM3_CH3 | LED_LWR_BLU | |
| PE6 | GPIO out | CS_LIS3MDL | |
| PE7 | GPIO | LSM6DSV_INT1 | EXTI |
| PE8 | GPIO | BT_HOST_WAKE | |
| PE9 | GPIO | GPIO_EXTERNAL | header pin; shares EXTI line 9 with `PA9` |
| PE10 | EXTI10 | LIS2MDL_DRDY | |
| PE11 | EXTI11 | BMP390_INT | |
| PE12 | GPIO out | SW_SENSE_IO | |
| PE13 | SPI1_SCK | | sensing bus 1 |
| PE14 | SPI1_MISO | | sensing bus 1 |
| PE15 | SPI1_MOSI | | sensing bus 1 |
| PF0 | GPIO in | CHG_STAT1 | charger status |
| PF1 | GPIO in | CHG_STAT2 | charger status |
| PF3 | ADF1_CCK0 | MIC_CK | PDM microphone clock |
| PF4 | ADF1_SDI0 | MIC_SD | PDM microphone data |
| PF14 | I2C4_SCL | | factory-test rig bus |
| PF15 | I2C4_SDA | | factory-test rig bus |
| PG9 | SPI3_SCK | | EXG bus |
| PH3 | | | BOOT0 |

Bus ownership as wired in `spi.c` / `i2c.c`: `hspi1` → `hspiSensing1`, `hspi2`
→ `hspiSensing2`, `hspi3` → `hspiExg`, `hi2c1` → `hi2cMainBus`; `hi2c4` is used
only by the factory test (GSR rig and alternate EEPROM). Which sensor sits on
which SPI bus follows from the chip-select pins above but is decided in the
per-sensor drivers, not in the pin map.

## Still unverified / not found in code

- ~~The full pin map~~ — resolved: the appendix below reproduces every pin the
  CubeMX project assigns (75 pins, `LogAndStream_Shimmer3R_UBGA132_SMPS.ioc`,
  STM32U5A5QJIxQ).
- ~~Which GPDMA channel serves which peripheral~~ — resolved (`Core/Src`):
  `GPDMA1` channel 0 `USART3_RX`, 1 `USART3_TX`, 4 `SPI1_RX`, 5 `SPI1_TX`,
  6 `SPI2_RX`, 7 `SPI2_TX`, 8 `SPI3_RX`, 9 `SPI3_TX`, 10 `I2C1_RX`.
  Channels 2, 3 and 11-15 are free. SDMMC1 uses its own internal DMA.
- ~~What `htim1` currently drives~~ — resolved: nothing. There is no
  `MX_TIM1_Init`; the single reference is a commented-out `MspDeInit` in
  `spi.c`. `TIM1` is free. The LED PWM timers are `TIM2` (upper RGB,
  `PA0`/`PB10`/`PB11`) and `TIM3` (lower RGB, `PE3`/`PE4`/`PE5`), both
  prescaler 48, period 255.
- ~~`hadc4`'s role~~ — resolved: `ADC4` is configured for `ADC_CHANNEL_VCORE`
  only. `ADC1` and `ADC2` both bring `PC3` (`VBAT_SENSE`) in as `IN4`;
  `hal_adc.c` uses `ADC1` as the sensing ADC (`hadcSensPtr`, which also
  sequences the `VREFINT`, `VBAT/4` and `TEMPSENSOR` debug channels) and
  `ADC2` as the battery ADC (`hadcBattPtr`).
- ~~Whether `EXTI9` is definitely VBUS~~ — resolved: `INT_LINE_USB_VBUS` is
  `EXTI9_IRQn` (`hal_Board.h`) and the pin is `PA9` `USB_VBUS` (`gpio.c`). It
  is not revision-dependent in the source; the other pin on line 9, `PE9`, is
  the `GPIO_EXTERNAL` header pin.
- ~~UART instance roles~~ — resolved (`usart.c` `setUartPeripheralPointers`):
  `huart1` = dock (`PB6` `DOCK_TXD`, `PB7` `DOCK_RXD`, 115200); `huart3` =
  Bluetooth (`PD8`/`PD9`, `CTS`/`RTS` on `PD11`/`PD12`, 1 Mbaud initial,
  retuned through `huartBt->Init.BaudRate`); `huart6` is the dock only under
  `SHIMMER4_SDK`. `USART2` on `PA2`/`PA3` is labelled `BSL_TX`/`BSL_RX` — the
  system bootloader's UART, not initialised by the application. There is no
  debug UART; the `swo.c` `huart1` route is commented out.
- ~~`hrng` usage~~ — resolved: `MX_RNG_Init()` is called from `main()` and
  nothing consumes it — no `HAL_RNG_GenerateRandomNumber` call outside the HAL
  driver. The peripheral is clocked for nothing.
