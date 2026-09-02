# Shimmer3 / Shimmer3R Streaming Data Format

This document defines the on-the-wire format of the sensor data packets a
**Shimmer3** or **Shimmer3R** running **LogAndStream** emits once streaming has
been started, and the rules a host must follow to turn those bytes back into
per-channel physical values. The same channel encodings are used for SD-card
logging, so a parser written from this document reads both.

> **Verified against:**
> - **Firmware (authority for bytes):** `log-and-stream-common` @ `c13cbde` —
>   `Sensing/shimmer_sensing.h` (packet layout `#define`s, channel-ID `#define`s
>   per platform, `MAX_NUM_CHANNELS`), `Sensing/shimmer_sensing.c`
>   (`ShimSens_configureChannels` — channel order and `dataLen` accumulation),
>   `Configuration/shimmer_config.h` (the enable bits that select channels).
> - **Platform packers (channel order and byte widths):** `shimmer3-firmware` @
>   `2765ff4` — `LogAndStream_Shimmer3/adc.c`, `i2c.c`, `spi.c` (plus
>   `Shimmer_Driver/BMPX80/bmpX80.h` for the pressure/temperature widths);
>   `shimmer3r-firmware` @ `a8f105e5` — `LogAndStream_Shimmer3R/Core/Src/i2c.c`,
>   `Core/Src/spi.c`, `Shimmer_Driver/hal_adc.c`.
> - **Host reference implementations:** `Shimmer-Java-Android-API` @ `edc3f7d9`
>   (v0.11.8_beta) — `driver/Configuration.java` (`Shimmer3.Channel`,
>   `SensorBitmap`, `SENSOR_ID`), `driver/ShimmerObject.java`
>   (`interpretDataPacketFormat` — the per-ID type strings and the raw-to-physical
>   maths), `driverUtilities/UtilParseData.java` (the signed/unsigned integer
>   decoders); `shimmer-web-sdk` @ `8f78313` —
>   `devices/shimmer3r/channelFormats.ts`, `devices/shimmer3r/streamFraming.ts`.

> **How to read this document.** **S3** = Shimmer3 (MSP430, LogAndStream);
> **S3R** = Shimmer3R (STM32U5). Channel IDs are **shared numbers with
> platform-specific meanings**: IDs `0x0D`–`0x13` are ADC channels whose physical
> pin differs between the two platforms, so the registry gives an "S3 meaning"
> and an "S3R meaning" column and a host must know which platform it is talking
> to (from `GET_DEVICE_VERSION_COMMAND`) before naming a channel.
>
> A packet carries **no per-channel identification**. The channel list from the
> most recent `INQUIRY_RESPONSE` is the only thing that says what the bytes mean,
> and the firmware recomputes that list on every configuration change — so the
> inquiry must be re-read after any `SET_*` that touches sensor enables.
>
> This document describes **LogAndStream** streaming. The SD-card file format
> shares the channel encodings but adds its own header; BtStream-era differences
> are confined to the appendix of the
> [protocol document](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md).

**Source references:**

| Layer | File |
|---|---|
| Packet layout | `Sensing/shimmer_sensing.h` — `PACKET_HEADER_IDX`, `PACKET_TIMESTAMP_LEN`, `FIRST_CH_BYTE_IDX` |
| Channel IDs | `Sensing/shimmer_sensing.h` |
| Channel order and lengths | `Sensing/shimmer_sensing.c` — `ShimSens_configureChannels` |
| ADC packing (S3) | `shimmer3-firmware` `LogAndStream_Shimmer3/adc.c` |
| I²C sensor packing (S3) | `shimmer3-firmware` `LogAndStream_Shimmer3/i2c.c` |
| SPI sensor packing (S3) | `shimmer3-firmware` `LogAndStream_Shimmer3/spi.c` |
| I²C sensor packing (S3R) | `shimmer3r-firmware` `LogAndStream_Shimmer3R/Core/Src/i2c.c` |
| SPI sensor packing (S3R) | `shimmer3r-firmware` `LogAndStream_Shimmer3R/Core/Src/spi.c` |
| ADC packing (S3R) | `shimmer3r-firmware` `LogAndStream_Shimmer3R/Shimmer_Driver/hal_adc.c` |
| Enable bits | `Configuration/shimmer_config.h` — `gConfigBytes` idx 3-5, 128-129 |
| Calibration parameters | `Calibration/shimmer_calibration.{h,c}` — see [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) |
| Host reference (Java) | `Shimmer-Java-Android-API` — `driver/Configuration.java`, `driver/ShimmerObject.java` |
| Host reference (TypeScript) | `shimmer-web-sdk` — `devices/shimmer3r/channelFormats.ts` |

---

## 1. Overview

_TODO: one data packet per sample period, pushed unsolicited between command responses; the host learns the layout from `INQUIRY_RESPONSE`; identical channel encodings on SD. Source: `Sensing/shimmer_sensing.c`._

## 2. Packet layout and timestamp

_TODO: `[0x00][timestamp 3 bytes LE][channel data...][CRC 0-2 bytes]`, the 32768 Hz tick base, 24-bit wraparound every ~512 s and how a host unwraps it, and that the CRC length follows the session's `SET_CRC_COMMAND` mode. Source: `Sensing/shimmer_sensing.h` packet `#define`s, `CRC/shimmer_crc.h`._

## 3. Channel ID registry

_The table in this section is generated mechanically from the firmware channel-ID
`#define`s, the per-platform packers' byte-width accumulation, the Java channel
enum and type strings, and the TypeScript channel formats. Do not hand-edit it._

Channel IDs are shared numbers whose *meaning* can differ between the two
generations (the ADC block `0x0D`–`0x13` in particular). Byte widths are derived
from the `sensing.dataLen +=` accumulation that follows each run of channel-ID
writes in the platform packers, so they are the widths the firmware actually
emits. `Encoding` and `SDK name` come from the web SDK's channel-format table.

| ID | FW name | S3 meaning | S3R meaning | Bytes | Encoding | Java type string | Java channel name | SDK name | Flags |
|---|---|---|---|---|---|---|---|---|---|
| `0x00` | `X_LN_ACCEL` | `X_LN_ACCEL` | `X_LN_ACCEL` | 2 | i16 le | i16, u12 | `XAAccel` | `LN_ACCEL_X` |  |
| `0x01` | `Y_LN_ACCEL` | `Y_LN_ACCEL` | `Y_LN_ACCEL` | 2 | i16 le | i16, u12 | `YAAccel` | `LN_ACCEL_Y` |  |
| `0x02` | `Z_LN_ACCEL` | `Z_LN_ACCEL` | `Z_LN_ACCEL` | 2 | i16 le | i16, u12 | `ZAAccel` | `LN_ACCEL_Z` |  |
| `0x03` | `VBATT` | `VBATT` | `VBATT` | 2 |  | i16, u12 | `VBatt` |  | SDK_MISSING |
| `0x04` | `X_WR_ACCEL` | `X_WR_ACCEL` | `X_WR_ACCEL` | 2 | i16 le | i16, u12 | `XDAccel` | `WR_ACCEL_X` |  |
| `0x05` | `Y_WR_ACCEL` | `Y_WR_ACCEL` | `Y_WR_ACCEL` | 2 | i16 le | i16, u12 | `YDAccel` | `WR_ACCEL_Y` |  |
| `0x06` | `Z_WR_ACCEL` | `Z_WR_ACCEL` | `Z_WR_ACCEL` | 2 | i16 le | i16 | `ZDAccel` | `WR_ACCEL_Z` |  |
| `0x07` | `X_MAG` | `X_MAG` | `X_MAG` | 2 | i16 le | i16, i16r | `XMag` | `MAG_X` |  |
| `0x08` | `Y_MAG` | `Y_MAG` | `Y_MAG` | 2 | i16 le | i16, i16r | `YMag` | `MAG_Y` |  |
| `0x09` | `Z_MAG` | `Z_MAG` | `Z_MAG` | 2 | i16 le | i16, i16r, u12 | `ZMag` | `MAG_Z` |  |
| `0x0A` | `X_GYRO` | `X_GYRO` | `X_GYRO` | 2 | i16 le | i16, i16r, u12 | `XGyro` | `GYRO_X` |  |
| `0x0B` | `Y_GYRO` | `Y_GYRO` | `Y_GYRO` | 2 | i16 le | i16, i16r, u16 | `YGyro` | `GYRO_Y` |  |
| `0x0C` | `Z_GYRO` | `Z_GYRO` | `Z_GYRO` | 2 | i16 le | i16, i16r, u16 | `ZGyro` | `GYRO_Z` |  |
| `0x0D` | `EXTERNAL_ADC_7 / EXTERNAL_ADC_0` | `EXTERNAL_ADC_7` | `EXTERNAL_ADC_0` | 2 |  | u14, u12 | `ExtAdc7`, `ExtAdc9` |  | SDK_MISSING, MEANING_DIFFERS |
| `0x0E` | `EXTERNAL_ADC_6 / EXTERNAL_ADC_1` | `EXTERNAL_ADC_6` | `EXTERNAL_ADC_1` | 2 |  | u14, u12 | `ExtAdc6`, `ExtAdc11` |  | SDK_MISSING, MEANING_DIFFERS |
| `0x0F` | `EXTERNAL_ADC_15 / EXTERNAL_ADC_2` | `EXTERNAL_ADC_15` | `EXTERNAL_ADC_2` | 2 |  | u14, u12 | `ExtAdc15`, `ExtAdc12` |  | SDK_MISSING, MEANING_DIFFERS |
| `0x10` | `INTERNAL_ADC_1 / INTERNAL_ADC_3` | `INTERNAL_ADC_1` | `INTERNAL_ADC_3` | 2 |  | u14, u12 | `IntAdc1`, `IntAdc17` |  | SDK_MISSING, MEANING_DIFFERS |
| `0x11` | `INTERNAL_ADC_12 / INTERNAL_ADC_0` | `INTERNAL_ADC_12` | `INTERNAL_ADC_0` | 2 |  | u14, u12 | `IntAdc12`, `IntAdc10` |  | SDK_MISSING, MEANING_DIFFERS |
| `0x12` | `INTERNAL_ADC_13 / INTERNAL_ADC_1` | `INTERNAL_ADC_13` | `INTERNAL_ADC_1` | 2 | i16 le | u14, u12, u8, u16 | `IntAdc13`, `IntAdc15` | `PPG` | MEANING_DIFFERS |
| `0x13` | `INTERNAL_ADC_14 / INTERNAL_ADC_2` | `INTERNAL_ADC_14` | `INTERNAL_ADC_2` | 2 |  | u14, u12 | `IntAdc14`, `IntAdc16` |  | SDK_MISSING, MEANING_DIFFERS |
| `0x14` | `X_ALT_ACCEL` | `X_ALT_ACCEL` | `X_ALT_ACCEL` | 2 | i12* le | i12*> | `XAlterAccel` | `HG_ACCEL_X` |  |
| `0x15` | `Y_ALT_ACCEL` | `Y_ALT_ACCEL` | `Y_ALT_ACCEL` | 2 | i12* le | i12*> | `YAlterAccel` | `HG_ACCEL_Y` |  |
| `0x16` | `Z_ALT_ACCEL` | `Z_ALT_ACCEL` | `Z_ALT_ACCEL` | 2 | i12* le | i12*> | `ZAlterAccel` | `HG_ACCEL_Z` |  |
| `0x17` | `X_ALT_MAG` | `X_ALT_MAG` | `X_ALT_MAG` | 2 |  | i16 | `XAlterMag` |  | SDK_MISSING |
| `0x18` | `Y_ALT_MAG` | `Y_ALT_MAG` | `Y_ALT_MAG` | 2 |  | i16 | `YAlterMag` |  | SDK_MISSING |
| `0x19` | `Z_ALT_MAG` | `Z_ALT_MAG` | `Z_ALT_MAG` | 2 |  | i16 | `ZAlterMag` |  | SDK_MISSING |
| `0x1A` | `BMP_TEMPERATURE` | `BMP_TEMPERATURE` | `BMP_TEMPERATURE` | S3: 2<br>S3R: 3 |  | u16r, u24 | `Temperature` |  | SDK_MISSING |
| `0x1B` | `BMP_PRESSURE` | `BMP_PRESSURE` | `BMP_PRESSURE` | 3 |  | u24r, u24 | `Pressure` |  | SDK_MISSING |
| `0x1C` | `GSR_RAW` | `GSR_RAW` | `GSR_RAW` | 2 | u16 le | u16 | `GsrRaw` | `GSR` |  |
| `0x1D` | `EXG_ADS1292R_1_STATUS` | `EXG_ADS1292R_1_STATUS` | `EXG_ADS1292R_1_STATUS` | 1 | u8 le | u8 | `EXG_ADS1292R_1_STATUS` | `Exg1_Status` |  |
| `0x1E` | `EXG_ADS1292R_1_CH1_24BIT` | `EXG_ADS1292R_1_CH1_24BIT` | `EXG_ADS1292R_1_CH1_24BIT` | 3 | i24 be | i24r | `EXG_ADS1292R_1_CH1_24BIT` | `Exg1_CH1_24Bit` |  |
| `0x1F` | `EXG_ADS1292R_1_CH2_24BIT` | `EXG_ADS1292R_1_CH2_24BIT` | `EXG_ADS1292R_1_CH2_24BIT` | 3 | i24 be | i24r | `EXG_ADS1292R_1_CH2_24BIT` | `Exg1_CH2_24Bit` |  |
| `0x20` | `EXG_ADS1292R_2_STATUS` | `EXG_ADS1292R_2_STATUS` | `EXG_ADS1292R_2_STATUS` | 1 | u8 le | u8 | `EXG_ADS1292R_2_STATUS` | `Exg2_Status` |  |
| `0x21` | `EXG_ADS1292R_2_CH1_24BIT` | `EXG_ADS1292R_2_CH1_24BIT` | `EXG_ADS1292R_2_CH1_24BIT` | 3 | i24 be | i24r | `EXG_ADS1292R_2_CH1_24BIT` | `Exg2_CH1_24Bit` |  |
| `0x22` | `EXG_ADS1292R_2_CH2_24BIT` | `EXG_ADS1292R_2_CH2_24BIT` | `EXG_ADS1292R_2_CH2_24BIT` | 3 | i24 be | i24r | `EXG_ADS1292R_2_CH2_24BIT` | `Exg2_CH2_24Bit` |  |
| `0x23` | `EXG_ADS1292R_1_CH1_16BIT` | `EXG_ADS1292R_1_CH1_16BIT` | `EXG_ADS1292R_1_CH1_16BIT` | 2 | i16 be | i16r | `EXG_ADS1292R_1_CH1_16BIT` | `Exg1_CH1_16Bit` |  |
| `0x24` | `EXG_ADS1292R_1_CH2_16BIT` | `EXG_ADS1292R_1_CH2_16BIT` | `EXG_ADS1292R_1_CH2_16BIT` | 2 | i16 be | i16r | `EXG_ADS1292R_1_CH2_16BIT` | `Exg1_CH2_16Bit` |  |
| `0x25` | `EXG_ADS1292R_2_CH1_16BIT` | `EXG_ADS1292R_2_CH1_16BIT` | `EXG_ADS1292R_2_CH1_16BIT` | 2 | i16 be | i16r | `EXG_ADS1292R_2_CH1_16BIT` | `Exg2_CH1_16Bit` |  |
| `0x26` | `EXG_ADS1292R_2_CH2_16BIT` | `EXG_ADS1292R_2_CH2_16BIT` | `EXG_ADS1292R_2_CH2_16BIT` | 2 | i16 be | i16r | `EXG_ADS1292R_2_CH2_16BIT` | `Exg2_CH2_16Bit` |  |
| `0x27` | `STRAIN_HIGH` | `STRAIN_HIGH` | `STRAIN_HIGH` | 2 |  | u12 | `BridgeAmpHigh` |  | SDK_MISSING |
| `0x28` | `STRAIN_LOW` | `STRAIN_LOW` | `STRAIN_LOW` | 2 |  | u12 | `BridgeAmpLow` |  | SDK_MISSING |

## 4. Channel order rules

_TODO: that channel order is not the numeric ID order but the order `ShimSens_configureChannels` appends them, the fixed sensor-block sequence it walks, and that `INQUIRY_RESPONSE` reports exactly this order. Source: `Sensing/shimmer_sensing.c` — `ShimSens_configureChannels`._

## 5. Per-channel encodings

_TODO: the encoding families — 12-bit ADC right-aligned in 2 bytes, 16-bit signed/unsigned little- and big-endian IMU words, 24-bit signed ExG samples, the 1-byte ExG status byte, the packed GSR range-plus-value word, and the pressure/temperature widths per sensor. Source: the five packer files plus `UtilParseData.java`._

## 6. Configuration snapshot needed for conversion

_TODO: the minimum set a host must hold to convert a packet — sampling rate, sensor enables, each sensor's range/rate, GSR range, pressure oversampling, ExG gain and reference settings, and the calibration blocks — and where each comes from. Source: `Configuration/shimmer_config.h`, `driver/ShimmerObject.java`._

## 7. Raw-to-physical conversion

### 7.1 Inertial

_TODO: the bias / sensitivity / alignment triple applied as `(R^-1)(K^-1)(raw - b)`, the per-range default sensitivities for each accel, gyro and mag part on both platforms, and the units. Cross-reference [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) §7. Source: `Calibration/shimmer_calibration.c` defaults, `driver/ShimmerObject.java`._

### 7.2 ADC and battery

_TODO: the 12-bit ADC reference and the two-point calibration the Java driver applies, and the battery-voltage divider. Source: `sensors/SensorBattVoltage.java`, `adc.c`._

### 7.3 GSR

_TODO: the four resistor ranges plus autorange, extracting the range bits from the sample word, the per-range resistance polynomial, and conversion to conductance. Source: `GSR/shimmer_gsr.{h,c}`, `driver/ShimmerObject.java`._

### 7.4 Pressure and temperature

_TODO: the four supported pressure parts (BMP180, BMP280, BMP390, BMP581), which raw widths each emits, and that BMP180/BMP280/BMP390 need the coefficient block while BMP581 outputs pre-compensated data. Cross-reference [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) §5. Source: `sensors/bmpX80/`, `ShimBt_sendRsp` `case GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND`._

### 7.5 ExG

_TODO: the 24-bit and 16-bit modes, the gain table from the CH1SET/CH2SET register fields, the reference voltage, conversion to millivolts, and decoding the status byte's lead-off bits. Source: `Configuration/shimmer_config.h` `gExgADS1292rRegs`, `driver/ShimmerObject.java`._

### 7.6 Bridge amplifier

_TODO: the `STRAIN_HIGH` / `STRAIN_LOW` pair, the offset and gain applied, and the SR49 board dependency. Source: `driver/Configuration.java`, `driver/ShimmerObject.java`._

### 7.7 Timestamps

_TODO: converting ticks to seconds, unwrapping the 24-bit counter, and reconciling stream timestamps with the real-world clock set by `SET_RWC_COMMAND`. Source: `RTC/shimmer_rtc.{h,c}`, `driver/ShimmerObject.java`._

## 8. Normative interpretation rules

_TODO: the short list a conforming parser must obey — trust the inquiry channel list over any assumed order, re-inquire after configuration writes, treat unknown channel IDs as fatal rather than skippable because widths are not self-describing, and honour the session CRC mode. Source: as above._

## Still unverified / not found in code

- _TODO: populate as the doc pass proceeds._
