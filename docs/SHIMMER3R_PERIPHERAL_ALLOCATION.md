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

## Still unverified / not found in code

- **The full pin map.** This document maps peripheral *instances* to parts. The
  per-pin assignment lives in the CubeMX `.ioc` file and in `gpio.c`, and is not
  reproduced here — a partial pin table would be worse than none, because the
  value of a pin map is completeness.
- **Which GPDMA channel serves which peripheral.** `hdmarx` / `hdmatx` handles
  are referenced from the SPI and SDMMC code, but the channel assignments were
  not extracted.
- **What `htim1` currently drives.** One reference was found in `Core/Src`. The
  claim in §3 that it is "barely referenced" is exactly that — a reference
  count, not a confirmation that its channels are free.
- **`hadc4`'s role.** Three ADC instances are initialised and the four MCU
  measurements are attributed to them collectively; which instance performs
  which was not traced.
- **Whether `EXTI9` is definitely VBUS on all board revisions.** Stated here
  because it is the constraint recorded against the event-capture work, but the
  pin assignment was not re-confirmed from `gpio.c` for this document, and it
  may vary by revision.
- **UART instance roles.** Three are initialised; the mapping of `huart1`,
  `huart3` and `huart6` to Bluetooth, dock and debug is inferred from usage
  counts rather than read from a definition.
- **`hrng` usage.** Initialised; no consumer was identified.
