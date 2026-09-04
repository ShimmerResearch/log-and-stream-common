# Shimmer3 / Shimmer3R Streaming Data Format

This document defines the on-the-wire format of the sensor data packets a
**Shimmer3** or **Shimmer3R** running **LogAndStream** emits once streaming has
been started, and the rules a host must follow to turn those bytes back into
per-channel physical values. The same channel encodings are used for SD-card
logging, so a parser written from this document reads both.

> **Verified against** — the revisions these byte-level claims were read from.
> A pinned commit is a citation, not a claim of currency: when the firmware
> moves on, this document needs re-checking against it rather than the stamp
> being wrong, and the `file:line` references throughout only resolve because
> the revision is pinned here.
>
> - **Firmware (authority for bytes):** `log-and-stream-common` @ `f3cf73e` —
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

While streaming, the device emits **one data packet per sample period**,
unsolicited, interleaved with command responses on the same link. There is no
request-response cycle for data: the host enables streaming and packets arrive
until it stops them.

A packet is **not self-describing**. It carries a header byte, a timestamp and
then a bare concatenation of channel samples with no channel identifiers, no
lengths and no delimiters. The host learns the layout exactly once, from the
`INQUIRY_RESPONSE`, and must apply that layout to every subsequent packet. Get
the layout wrong by a single byte and every channel after the error decodes as
garbage — silently, because there is nothing in the packet to detect it
against.

The same channel encodings are used for data logged to the SD card, so a parser
written for one works for the other; only the framing around them differs. See
[SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md).

Sampling is driven by `ShimSens_configureChannels`, which walks the enabled
sensors in a fixed order and builds both the channel list reported by the
inquiry and the byte offsets the packers write into. Because one function
produces both, the inquiry list and the packet layout cannot disagree — which
is precisely why the inquiry is authoritative and any host-side assumption
about ordering is not.

## 2. Packet layout and timestamp

```
+--------+---------------+------------------------+-----------+
| 0x00   | timestamp     | channel data           | CRC       |
| 1 byte | 3 bytes, LE   | sum of channel widths  | 0-2 bytes |
+--------+---------------+------------------------+-----------+
```

| Field | Offset | Constant | Size |
|---|---:|---|---:|
| Header / data-packet marker | 0 | `PACKET_HEADER_IDX`, `PACKET_HEADER_LEN` | 1 |
| Timestamp | 1 | `PACKET_TIMESTAMP_IDX`, `PACKET_TIMESTAMP_LEN` | 3 |
| First channel byte | 4 | `FIRST_CH_BYTE_IDX` | — |
| CRC | last | session CRC mode | 0, 1 or 2 |

The header byte is `0x00` (`DATA_PACKET`), which is what distinguishes a data
packet from a command response on the same stream.

### 2.1 The timestamp

Three bytes, **little-endian**, counting ticks of the 32768 Hz clock. It is a
free-running counter, not a time of day.

- **Resolution** is 1/32768 s, about 30.5 µs.
- **Range** is 2^24 ticks = 16,777,216 ticks = **512 seconds exactly**, after
  which it wraps to zero.

A host must unwrap it. The standard approach is to track the previous raw value
and increment a rollover count whenever the new value is lower:

```
if (rawNow < rawPrev) rolloverCount++;
totalTicks = rawNow + rolloverCount * 16777216
seconds    = totalTicks / 32768.0
```

> **Unwrapping fails if the host misses more than 512 seconds of packets.** The
> counter gives no absolute reference, so a gap longer than one wrap period is
> indistinguishable from a short one. A host that reconnects mid-trial cannot
> recover absolute time from the stream alone; it must re-anchor against the
> real-world clock. See [SHIMMER3_TIMEKEEPING.md](SHIMMER3_TIMEKEEPING.md).

> **The timestamp is the *sampling* instant, not the transmission instant.**
> Bluetooth buffering and retransmission mean packets can arrive late, in
> bursts, and with jitter that the timestamp does not show. Always plot against
> the timestamp, never against arrival time.

### 2.2 CRC

The trailing CRC is 0, 1 or 2 bytes according to the mode selected for the
session with `SET_CRC_COMMAND`:

| `COMMS_CRC_MODE` | Value | Trailing bytes |
|---|---:|---:|
| `CRC_OFF` | 0 | 0 |
| `CRC_1BYTE_ENABLED` | 1 | 1 |
| `CRC_2BYTES_ENABLED` | 2 | 2 |

(`CRC_MAX_SUPPORTED_BYTES` is a bounds-check sentinel, not a selectable mode.)

The algorithm is CRC-16 with initial value `CRC_INIT = 0xB0CA`; in 1-byte mode
only the low byte is transmitted. `calculateCrcAndInsert` appends it and
`checkCrc` verifies.

> **The CRC length is session state, not packet state.** Nothing in the packet
> says whether a CRC is present or how long it is. A host that loses track of
> the mode it negotiated will mis-frame every packet. Default is off; set the
> mode explicitly at session start rather than assuming.

## 3. Channel ID registry

_The table in this section is generated mechanically from the firmware channel-ID
`#define`s, the per-platform packers' byte-width accumulation, the Java channel
enum and type strings, and the TypeScript channel formats. Do not hand-edit it._

_One cell has been corrected by hand since: the alternate-accelerometer rows
carried a Java type string of `i12*>`, where the trailing character is an
artefact of the extractor rather than anything in the driver. Fix it in the
generator before the next run, or it will come back._

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
| `0x14` | `X_ALT_ACCEL` | `X_ALT_ACCEL` | `X_ALT_ACCEL` | 2 | i12* le | i12* | `XAlterAccel` | `HG_ACCEL_X` |  |
| `0x15` | `Y_ALT_ACCEL` | `Y_ALT_ACCEL` | `Y_ALT_ACCEL` | 2 | i12* le | i12* | `YAlterAccel` | `HG_ACCEL_Y` |  |
| `0x16` | `Z_ALT_ACCEL` | `Z_ALT_ACCEL` | `Z_ALT_ACCEL` | 2 | i12* le | i12* | `ZAlterAccel` | `HG_ACCEL_Z` |  |
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

**Channel order is not numeric channel-ID order.** It is the order in which
`ShimSens_configureChannels` appends entries to `sensing.cc`, which is a fixed
walk over the sensor blocks — and the two orders are genuinely different: on
Shimmer3R the magnetometer (IDs `0x07`-`0x09`) is emitted *before* the gyroscope
(`0x0A`-`0x0C`), and the low-noise accelerometer (`0x00`-`0x02`) after both.

`ShimSens_configureChannels` calls three per-bus configurators in this order:

1. `ADC_configureChannels()` — only when `ShimBrd_areMcuAdcsUsedForSensing()`
2. `I2C_configureChannels()`
3. `SPI_configureChannels()`

Each appends the channels for the sensors it owns, in its own fixed sequence,
and accumulates `sensing.dataLen`. The `INQUIRY_RESPONSE` reports exactly the
resulting list.

### 4.1 Shimmer3 order

MCU ADC channels come first on Shimmer3, because the MSP430's own ADC carries
most of the analog sensors:

| Order | Bus | Channels | Bytes |
|---:|---|---|---:|
| 1 | ADC | `X_LN_ACCEL`, `Y_LN_ACCEL`, `Z_LN_ACCEL` | 6 |
| 2 | ADC | `VBATT` | 2 |
| 3 | ADC | `EXTERNAL_ADC_7` | 2 |
| 4 | ADC | `EXTERNAL_ADC_6` | 2 |
| 5 | ADC | `EXTERNAL_ADC_15` | 2 |
| 6 | ADC | `INTERNAL_ADC_12` | 2 |
| 7 | ADC | `STRAIN_HIGH`, `STRAIN_LOW` | 4 |
| 8 | ADC | `INTERNAL_ADC_13` | 2 |
| 9 | ADC | `INTERNAL_ADC_14` | 2 |
| 10 | ADC | `GSR_RAW` **or** `INTERNAL_ADC_1` | 2 |
| 11 | I2C | `X_GYRO`, `Y_GYRO`, `Z_GYRO` | 6 |
| 12 | I2C | `X_WR_ACCEL`, `Y_WR_ACCEL`, `Z_WR_ACCEL` | 6 |
| 13 | I2C | `X_MAG`, then **`Z_MAG`, `Y_MAG`** on LSM303DLHC, else `Y_MAG`, `Z_MAG` | 6 |
| 14 | I2C | `X_ALT_ACCEL`, `Y_ALT_ACCEL`, `Z_ALT_ACCEL` | 6 |
| 15 | I2C | `X_ALT_MAG`, `Y_ALT_MAG`, `Z_ALT_MAG` | 6 |
| 16 | I2C | `BMP_TEMPERATURE`, `BMP_PRESSURE` | `BMPX80_PACKET_SIZE` |
| 17 | SPI | ExG chip 1: status, CH1, CH2 | 7 (24-bit) or 5 (16-bit) |
| 18 | SPI | ExG chip 2: status, CH1, CH2 | 7 (24-bit) or 5 (16-bit) |

> **The Shimmer3 magnetometer axis order is board-dependent.** With an
> LSM303DLHC the order is **X, Z, Y**; with the LSM303AH it is X, Y, Z. The
> firmware branches on `ShimBrd_isWrAccelInUseLsm303dlhc()`. A host that assumes
> XYZ will silently swap the Y and Z magnetometer axes on older boards. This is
> visible in the inquiry channel list — another reason to read it rather than
> assume.

> **GSR must be the last analog channel.** The source comment says so
> explicitly. `GSR_RAW` and `INTERNAL_ADC_1` are alternatives on the same input,
> which is why §10 of the InfoMem document has the firmware silently disable one
> when both are requested.

### 4.2 Shimmer3R order

Almost everything moved to SPI on Shimmer3R; only the LIS2MDL magnetometer is
on I2C, and the external ADS7028 ADC provides the analog channels at the end of
the SPI block.

| Order | Bus | Channels | Bytes |
|---:|---|---|---:|
| 1 | I2C | `X_MAG`, `Y_MAG`, `Z_MAG` (LIS2MDL) | 6 |
| 2 | SPI | `X_GYRO`, `Y_GYRO`, `Z_GYRO` | 6 |
| 3 | SPI | `X_LN_ACCEL`, `Y_LN_ACCEL`, `Z_LN_ACCEL` | 6 |
| 4 | SPI | **`BMP_PRESSURE`, `BMP_TEMPERATURE`** | 3 + 3 |
| 5 | SPI | `X_ALT_ACCEL`, `Y_ALT_ACCEL`, `Z_ALT_ACCEL` | 6 |
| 6 | SPI | `X_WR_ACCEL`, `Y_WR_ACCEL`, `Z_WR_ACCEL` | 6 |
| 7 | SPI | `X_ALT_MAG`, `Y_ALT_MAG`, `Z_ALT_MAG` | 6 |
| 8 | SPI | ExG chip 1: status, CH1, CH2 | 7 (24-bit) or 5 (16-bit) |
| 9 | SPI | ExG chip 2: status, CH1, CH2 | 7 (24-bit) or 5 (16-bit) |
| 10 | SPI (ADS7028) | `INTERNAL_ADC_0` | 2 |
| 11 | SPI (ADS7028) | `STRAIN_HIGH`, `STRAIN_LOW` | 4 |
| 12 | SPI (ADS7028) | `INTERNAL_ADC_1` | 2 |
| 13 | SPI (ADS7028) | `INTERNAL_ADC_2` | 2 |
| 14 | SPI (ADS7028) | `GSR_RAW` **or** `INTERNAL_ADC_3` | 2 |
| 15 | SPI (ADS7028) | `EXTERNAL_ADC_0` | 2 |
| 16 | SPI (ADS7028) | `EXTERNAL_ADC_1` | 2 |
| 17 | SPI (ADS7028) | `EXTERNAL_ADC_2` | 2 |
| 18 | SPI (ADS7028) | `VBATT` | 2 |

### 4.3 The differences that bite

| | Shimmer3 | Shimmer3R |
|---|---|---|
| Pressure / temperature order | **temperature, then pressure** | **pressure, then temperature** |
| Temperature width | 2 bytes | 3 bytes |
| Pressure width | 3 bytes | 3 bytes |
| First channel block | Low-noise accel (ADC) | Magnetometer (I2C) |
| `VBATT` position | Second | Last |
| Magnetometer axis order | X, Z, Y on LSM303DLHC | X, Y, Z |

> **The pressure/temperature reversal is the single most damaging difference.**
> The two channels are enabled and disabled together by one configuration bit,
> so a host that hard-codes the Shimmer3 order and widths against a Shimmer3R
> mis-reads six bytes and shifts every channel after them. There is no error
> indication.

`MAX_NUM_CHANNELS` is 45 on Shimmer3 and 50 on Shimmer3R — the upper bound on
the inquiry channel list.

## 5. Per-channel encodings

Widths and encodings are in the §3 registry. The families are:

### 5.1 ADC channels — 2 bytes, unsigned, right-aligned

`VBATT`, all `EXTERNAL_ADC_*` and `INTERNAL_ADC_*`, and the bridge-amplifier
pair. Two bytes little-endian holding an unsigned value whose significant width
is the converter's, not 16 bits: the Java driver's type strings are `u12` and
`u14` for these channels. Mask before use — the unused high bits are not
guaranteed.

### 5.2 IMU channels — 2 bytes, signed

Accelerometer, gyroscope and magnetometer axes: `int16`, little-endian on both
platforms.

The alternate accelerometer (`0x14`-`0x16`) is 12-bit data in a 16-bit field on
Shimmer3 (`i12`), and needs sign extension from bit 11 rather than bit 15.

### 5.3 ExG — status byte plus 24-bit or 16-bit big-endian samples

| Channel | Width | Encoding |
|---|---:|---|
| `EXG_ADS1292R_n_STATUS` | 1 | `uint8` bit flags |
| `EXG_ADS1292R_n_CHm_24BIT` | 3 | `int24`, **big-endian** |
| `EXG_ADS1292R_n_CHm_16BIT` | 2 | `int16`, **big-endian** |

ExG is the only sensor whose samples are big-endian, because the bytes come
straight off the ADS1292R in its own order. Sign-extend from bit 23 or bit 15.

A chip contributes 7 bytes in 24-bit mode (1 + 3 + 3) or 5 bytes in 16-bit mode
(1 + 2 + 2). The two chips are independent — one may be 24-bit while the other
is 16-bit.

### 5.4 GSR — packed range and value

`GSR_RAW` is one 16-bit little-endian word carrying **two fields**:

| Bits | Field |
|---|---|
| 15-14 | Active feedback resistor (0-3) |
| 13-0 | ADC value |

`GSR_range()` packs it as `ADC_val | (current_active_resistor << 14)`. A host
must mask with `0x3FFF` before treating the low field as a measurement, and
must read the range bits **per sample**: under auto-range the firmware changes
resistor mid-stream, so the range is not a constant for the trial.

### 5.5 Pressure and temperature

| Platform | Temperature | Pressure |
|---|---:|---:|
| Shimmer3 | 2 bytes | 3 bytes |
| Shimmer3R | 3 bytes | 3 bytes |

Both are unsigned and raw — the compensation formula and the part's coefficient
block turn them into physical units (§7.4).

### 5.6 The width problem

**Widths are not derivable from the channel ID alone.** ID `0x1A`
(`BMP_TEMPERATURE`) is 2 bytes on Shimmer3 and 3 on Shimmer3R; the ADC block
`0x0D`-`0x13` means different physical inputs on each platform. A parser must
resolve widths against the hardware generation, which it learns from the
inquiry response, not from the channel list in isolation.

> A decoder that defaults unknown channel IDs to 2 bytes will appear to work and
> produce wrong data. Pressure at 3 bytes is the common case that exposes it:
> every channel after pressure shifts by one byte. Treat an unrecognised ID as
> fatal (§8).

## 6. Configuration snapshot needed for conversion

Raw counts alone are not interpretable. To convert a packet a host must hold:

| Item | Source | Needed for |
|---|---|---|
| Hardware generation | `GET_DEVICE_VERSION` / inquiry | Channel meanings and widths |
| Channel list, in order | `INQUIRY_RESPONSE` | Slicing the packet at all |
| Sampling rate | InfoMem bytes 0-1 | Time base, filter design |
| Accel / gyro / mag ranges | InfoMem bytes 6, 8, 9, 130 (with MSB bits) | Selecting the right calibration set |
| Calibration blocks | InfoMem, or `GET_CALIB_DUMP` | Bias, sensitivity, alignment |
| GSR range | InfoMem byte 9 bits 3-1, **and the per-sample range bits** | Feedback resistor |
| Pressure oversampling | InfoMem bytes 9 and 130 | Sample timing |
| Pressure coefficients | `GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND` | Compensation |
| ExG registers | InfoMem bytes 10-29 | Gain, reference, lead-off |
| CRC mode | Session state | Packet framing |

> **Re-read after every configuration write.** Changing a range changes which
> calibration set applies; changing an enable bit changes the packet layout
> entirely. The firmware also silently corrects illegal combinations
> ([SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §10),
> so what the host asked for and what is in force may differ.

## 7. Raw-to-physical conversion

### 7.1 Inertial

Apply the three-parameter kinematic model:

```
C = inv(R) * inv(K) * (U - B)
```

with `B` bias, `K` the diagonal sensitivity matrix and `R` the alignment matrix.
The parameter block layout, the big-endian trap in bias and sensitivity, the
gyro sensitivity scale factor of 100, the degenerate-matrix fallback, and the
complete per-range default tables for every part on both platforms are in
[SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) §3, §6 and §7.

Units: accelerometers m/s², gyroscopes deg/s.

The set that applies is the one matching the sensor's **currently configured
range**. Reading calibration without also reading the range gives the wrong
scale.

### 7.2 ADC and battery

ADC channels are converted with the converter's reference and resolution.
`VBATT` additionally passes through a resistive divider on the board, so the
battery voltage is the converted ADC voltage multiplied by the divider ratio.

The Java driver applies a two-point calibration to ADC channels where one has
been stored, falling back to the nominal reference otherwise.

> The exact reference voltage, resolution and divider ratio are per-board and
> live in the platform firmware and the Java sensor classes rather than in
> `log-and-stream-common`. They are not restated here — see *Still unverified*.

### 7.3 GSR

1. Split the 16-bit word: `range = w >> 14`, `adc = w & 0x3FFF` (§5.4).
2. Convert `adc` to millivolts at the converter's reference.
3. Apply the op-amp equation the firmware itself uses in `GSR_calcResistance`:

```
resistance_ohms = R_feedback[range] / (((mV / 1000) / 0.5) - 1.0)
```

4. Conductance in microsiemens is `1e6 / resistance_ohms`.

Feedback resistors, from `GSR_FEEDBACK_RESISTORS_OHMS`:

| Range code | Constant | Resistance |
|---:|---|---:|
| 0 | `HW_RES_40K` | 40,200 Ω |
| 1 | `HW_RES_287K` | 287,000 Ω |
| 2 | `HW_RES_1M` | 1,000,000 Ω |
| 3 | `HW_RES_3M3` | 3,300,000 Ω |
| 4 | `GSR_AUTORANGE` | Not a resistor — configuration only |

> **Range code 4 never appears in a sample.** `GSR_AUTORANGE` is a configuration
> value meaning "switch automatically"; the two bits in the sample always carry
> the *actual* resistor, 0-3. A host that sees 4 in the packed field has
> mis-parsed something.

Auto-range switching thresholds, applied by `GSR_controlRange`, are in raw ADC
counts with hysteresis:

| Current resistor | Switch down (to a larger resistor) below | Switch up (to a smaller resistor) above |
|---|---:|---:|
| `HW_RES_40K` | 1490 | — |
| `HW_RES_287K` | 1490 | `HW_RES_287K_MAX_ADC_VAL` |
| `HW_RES_1M` | 1630 | 3700 |
| `HW_RES_3M3` | 1125 | 3930 |

> **Auto-range transitions produce a discontinuity.** The firmware runs
> `GSR_smoothTransition` and repeats the last ADC value during the settling
> window, so consecutive identical samples around a range change are expected
> and are not a dropped-sample artefact.

### 7.4 Pressure and temperature

Four parts are supported, and the host needs the coefficient block for three of
them:

| Part | ID | Coefficient bytes | Notes |
|---|---:|---:|---|
| BMP180 | 0 | 22 | Shimmer3 |
| BMP280 | 1 | 24 | Shimmer3. Header splits them 22 + 2 |
| BMP390 | 2 | 21 | Both |
| BMP581 | 3 | **0** | Shimmer3R. Outputs pre-compensated data |

Fetch with `GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND`, whose reply carries
the sensor ID so the host knows which compensation formula to apply — see
[SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) §5. The formulae themselves
are Bosch's and are in each part's datasheet; the firmware does not implement
them.

For the BMP581 the raw values are already compensated: scale and use directly.

### 7.5 ExG

**24-bit mode.** Sign-extend the big-endian `int24`, then:

```
millivolts = sample * (V_REF * 1000) / (gain * (2^23 - 1))
```

**16-bit mode.** As above, sign-extending from bit 15 and with `2^15 - 1`.

Gain comes from the `CH1SET` / `CH2SET` registers in the configuration bytes
(InfoMem 10-29, see
[SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §5) — the
PGA gain field of each. The reference voltage is set by the `CONFIG2` register's
internal-reference selection. Both are ADS1292R fields; the datasheet is the
authority for the bit encodings.

**Status byte.** One byte per chip, preceding its channel samples, carrying the
lead-off detection bits. The ADS1292R datasheet defines the bit positions; the
firmware passes the register through untouched.

### 7.6 Bridge amplifier

`STRAIN_HIGH` and `STRAIN_LOW` are a pair of ADC channels, always enabled and
disabled together, 4 bytes total. They are the two outputs of the bridge
amplifier on the SR49 expansion board; conversion is an offset and gain applied
per board revision, held in the Java driver's board configuration rather than
in the firmware.

Note that enabling the bridge amplifier forces two internal ADC channels off
(InfoMem §10.1), because they share inputs.

### 7.7 Timestamps

1. Unwrap the 24-bit counter (§2.1).
2. Divide by 32768 for seconds since streaming started.
3. To place samples on an absolute timeline, anchor against the real-world
   clock. The device's RWC is set by `SET_RWC_COMMAND` and read by
   `GET_RWC_COMMAND`; the offset between the free-running tick counter and the
   RWC is what converts one to the other.

> On Shimmer3 the SD header carries that offset in `SDH_RTC_DIFF_*`. On
> Shimmer3R the same eight bytes instead carry the top three bytes of the
> file's 64-bit initial timestamp, so a logged file on either generation can be
> placed on an absolute timeline — but by different arithmetic. See
> [SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md) §3.3 and
> [SHIMMER3_TIMEKEEPING.md](SHIMMER3_TIMEKEEPING.md).

## 8. Normative interpretation rules

A conforming parser must:

1. **Take the channel list and its order from `INQUIRY_RESPONSE`.** Never from
   the sensor-enable bitmap, never from numeric channel-ID order, never from a
   hard-coded table.
2. **Re-inquire after any configuration write.** The firmware silently corrects
   illegal combinations, so the layout after a write may not be the one
   requested.
3. **Resolve channel widths against the hardware generation.** IDs `0x0D`-`0x13`
   and `0x1A` mean different things and have different widths on the two
   platforms.
4. **Treat an unknown channel ID as fatal.** Widths are not self-describing, so
   an unrecognised ID makes every subsequent byte offset in the packet unsound.
   Do not skip it and continue; stop, or mark the whole packet untrusted. A
   decoder that guesses two bytes will produce plausible, wrong numbers.
5. **Validate the total.** The sum of the resolved channel widths plus 4 plus
   the CRC length must equal the received packet length. This is the only
   integrity check available on the layout, and it catches most width errors
   immediately.
6. **Honour the session CRC mode** — the trailing byte count is session state,
   not something the packet declares.
7. **Read the GSR range bits per sample**, not once per trial.
8. **Unwrap the timestamp**, and do not assume the host's arrival time bears any
   relation to it.

## Still unverified / not found in code

- **ADC reference voltages, resolutions and the battery divider ratio.** These
  are per-board and live in the platform firmware and the Java sensor classes,
  not in `log-and-stream-common`. The Java type strings imply 12-bit and 14-bit
  converters depending on channel, but the numeric reference values were not
  read for this document, so no formula with concrete constants is given in
  §7.2.
- **`HW_RES_40K_MIN_ADC_VAL` and `HW_RES_287K_MAX_ADC_VAL`.** Both are defined
  as multi-line expressions rather than literals, so §7.3's table gives the
  literal thresholds only where the source states one. The two derived values
  were not evaluated.
- **`BMPX80_PACKET_SIZE` on Shimmer3.** The I2C configurator accumulates this
  symbol rather than a literal, and it resolves in the platform repo. The
  registry's 2-byte temperature and 3-byte pressure for Shimmer3 come from the
  generated table, not from expanding this constant.
- **Channels declared but never emitted.** `X_ALT_MAG` through `Z_ALT_MAG` and
  several ADC channels appear in the ID registry with a Java channel name but no
  SDK entry (`SDK_MISSING` in §3). Whether every one is reachable on shipping
  hardware was not established.
- **The `i12*` encoding marker** in §3's alternate-accelerometer rows. The
  registry note records that the trailing character is an extractor artefact;
  the underlying encoding is 12-bit signed data in a 16-bit field, but the exact
  alignment within the field was not confirmed against a captured packet.
- **Bridge-amplifier offset and gain constants.** Held in the Java driver's
  per-board configuration; not present in the firmware and not restated here.
