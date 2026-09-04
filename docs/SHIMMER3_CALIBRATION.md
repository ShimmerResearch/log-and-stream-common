# Shimmer3 / Shimmer3R Sensor Calibration

Per-device kinematic calibration — bias, sensitivity and axis alignment for each
accelerometer, gyroscope and magnetometer — is stored on the device, exposed to
host software over the Bluetooth protocol in two different shapes, mirrored into
the SD-card file header, and kept as a file on the SD card so a logged trial can
be converted after the fact.

This document is the byte-level specification of those blocks, shared by the
firmware and the host SDKs.

> **Verified against** — the revisions these byte-level claims were read from.
> A pinned commit is a citation, not a claim of currency: when the firmware
> moves on, this document needs re-checking against it rather than the stamp
> being wrong, and the `file:line` references throughout only resolve because
> the revision is pinned here.
>
> - **Firmware (authority for bytes):** `log-and-stream-common` @ `f3cf73e` —
>   `Calibration/shimmer_calibration.h` (`sc_t` / `sc_data_u` / `sc_default_t`,
>   the `SC_OFFSET_*` blob offsets, `SC_SENSOR_*` sensor IDs, `SC_SENSOR_RANGE_*`
>   range codes, `SC_DATA_LEN_STD_IMU_CALIB`),
>   `Calibration/shimmer_calibration.c` (blob assembly, per-sensor default seeds,
>   RAM/file persistence, config-byte synchronisation),
>   `Configuration/shimmer_config.h` (`gImuConfig` and the `NV_*_CALIBRATION`
>   offsets), `Comms/shimmer_bt_uart.c`
>   (`ShimBt_replySingleSensorCalibCmd`, `ShimBt_calibrationChangeCommon`,
>   the `GET`/`SET_CALIB_DUMP` handlers).
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4`,
>   `shimmer3r-firmware` @ `a8f105e5`.
> - **Host reference implementations:** `Shimmer-Java-Android-API` @ `edc3f7d9`
>   (v0.11.8_beta) — the `sensors/` calibration classes
>   (`adxl371/SensorADXL371.java`, `bmpX80/SensorBMP390.java`,
>   `bmpX80/SensorBMP180.java`, `SensorBattVoltage.java`) and
>   `driver/ShimmerObject.java`; `shimmer-web-sdk` @ `8f78313`.

> **How to read this document.** **S3** = Shimmer3 (MSP430, LogAndStream);
> **S3R** = Shimmer3R (STM32U5). The **Gen** column says which generation a
> sensor ID or block applies to; the `SC_SENSOR_*` IDs are **disjoint between the
> two platforms** (S3 uses 2 and 30-36, S3R uses 37-43), so a blob identifies its
> own hardware generation by the IDs it contains.
>
> There are **two representations of the same calibration data** and it matters
> which one a host is holding: the *per-sensor 21-byte block* as it sits in the
> configuration bytes and as the six per-sensor Bluetooth commands exchange it,
> and the *calibration dump blob*, a self-describing TLV-ish container with a
> header, per-sensor headers and timestamps, which is what `GET_CALIB_DUMP` /
> `SET_CALIB_DUMP` page across and what the SD-card calibration file holds.
> The firmware keeps both in sync; §8 gives the precedence rules.
>
> This document covers **LogAndStream** only.

**Source references:**

| Layer | File |
|---|---|
| Blob format, offsets, sensor IDs | `Calibration/shimmer_calibration.h` |
| Blob assembly, defaults, persistence | `Calibration/shimmer_calibration.c` |
| 21-byte block in the config bytes | `Configuration/shimmer_config.h` — `gImuConfig`, `NV_*_CALIBRATION` |
| Bluetooth access | `Comms/shimmer_bt_uart.c` — see [SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md) §7.6 |
| SD header mirror | `SDCard/shimmer_sd_header.h` — `SDH_*_CALIBRATION` |
| Calibration file on the SD card | `Calibration/shimmer_calibration.c` — `ShimCalib_ram2File` / `ShimCalib_file2Ram` |
| Pressure coefficient blocks | `Comms/shimmer_bt_uart.c` — `case GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND` |
| Applying calibration | `Shimmer-Java-Android-API` — `driver/ShimmerObject.java`, `sensors/` |
| Host reference (TypeScript) | `shimmer-web-sdk` |

---

## 1. Overview

Every Shimmer3 and Shimmer3R carries per-device kinematic calibration for its
inertial sensors. "Kinematic" here means the three-parameter model shared by
every accelerometer, gyroscope and magnetometer on both platforms: a **bias**
(offset) vector, a **sensitivity** vector, and a 3x3 **alignment** matrix. One
such parameter set exists *per sensor, per range* — changing the wide-range
accelerometer from ±2 g to ±4 g selects a different stored set, it does not
rescale the current one.

Pressure sensors are calibrated by a completely different mechanism: the part's
own factory trim coefficients, read straight out of the sensor and passed to
the host uninterpreted (§5).

The same kinematic parameters exist in four places at once, and the firmware
keeps them synchronised:

| Where | Shape | Purpose |
|---|---|---|
| Configuration bytes (InfoMem) | Bare 21-byte blocks at fixed offsets | The authoritative persistent copy; survives power loss |
| Calibration dump in RAM | Self-describing blob, header + per-sensor records | What `GET`/`SET_CALIB_DUMP` page across; carries per-sensor timestamps and covers ranges that are not currently selected |
| SD-card file `/Calibration/calib_<mac4>` | Byte-identical copy of the RAM blob | Lets a logged trial be converted after the fact, and survives a configuration rewrite |
| SD-card data-file header | Bare 21-byte blocks at `SDH_*` offsets, plus timestamps | Stamped into each logged file so the file is self-describing |

A host can therefore read calibration in **two different shapes**, and it
matters which one it is holding:

- **Per-sensor 21-byte blocks** — what the per-sensor Bluetooth calibration
  commands exchange, and what sits in the configuration bytes and the SD
  header. These cover only the *currently configured range* of each sensor.
- **The calibration dump blob** — what `GET_CALIB_DUMP` / `SET_CALIB_DUMP`
  transfer. Self-describing, carries every range the firmware has a set for,
  and carries a timestamp per sensor.

§8 gives the precedence rules between them.

> **Not the same as the streaming data.** Calibration parameters describe how to
> convert raw ADC counts to physical units; they are never applied on-device to
> the streamed or logged samples, which are always raw. Conversion is entirely
> the host's job — see §7.

## 2. Calibration dump blob

The blob is a flat byte array: a 10-byte global header followed by a sequence of
variable-length per-sensor records. It is held in RAM as `shimmerCalib_ram[]`,
written verbatim to the SD card, and paged over Bluetooth in 128-byte chunks.

### 2.1 Global header (10 bytes)

| Offset | Name | Size | Meaning |
|---:|---|---:|---|
| 0 | `SC_OFFSET_LENGTH_L` | 2 (LE) | Blob length, **excluding these two length bytes** |
| 2 | `SC_OFFSET_VER_HW_ID_L` | 2 (LE) | `DEVICE_VER` — hardware identifier |
| 4 | `SC_OFFSET_VER_FW_ID_L` | 2 (LE) | `FW_IDENTIFIER` |
| 6 | `SC_OFFSET_VER_FW_MAJOR_L` | 2 (LE) | Firmware major version |
| 8 | `SC_OFFSET_VER_FW_MINOR_L` | 1 | Firmware minor version |
| 9 | `SC_OFFSET_VER_FW_INTER_L` | 1 | Firmware patch ("internal") version |
| 10 | `SC_OFFSET_FIRST_SENSOR` | — | First per-sensor record starts here |

> **The length field excludes itself.** A freshly initialised blob with no
> sensor records reports length **8**, not 10 — `ShimCalib_init` sets
> `shimmerCalib_ramLen = 8` while the header occupies bytes 0-9. Total bytes on
> the wire or on disk are always `length + 2`. Hosts that treat the field as the
> total will truncate the last two bytes of the final record.

> **The version fields are stamped by the device, not by the writer.** After a
> `SET_CALIB_DUMP` completes, `ShimCalib_ramWrite` calls `ShimCalib_initVer`,
> overwriting bytes 2-9 of whatever the host just uploaded with the running
> firmware's own identifiers. A host cannot forge these, and a blob read back
> will not byte-match the one written if the host supplied different version
> bytes.

### 2.2 Per-sensor record

Each record is a 12-byte header followed by `data_len` bytes of payload, laid
out to match the packed `sc_t` struct:

| Offset in record | Name | Size | Meaning |
|---:|---|---:|---|
| 0 | `SC_OFFSET_SENSOR_ID_L` | 2 (LE) | Sensor ID — see §2.3 |
| 2 | `SC_OFFSET_SENSOR_RANGE` | 1 | Range code this set applies to |
| 3 | `SC_OFFSET_SENSOR_LENGTH` | 1 | `data_len` — payload byte count |
| 4 | `SC_OFFSET_SENSOR_TIMESTAMP` | 8 | Calibration timestamp, RWC ticks, little-endian |
| 12 | `SC_OFFSET_SENSOR_DATA` | `data_len` | Payload — 21 bytes for every kinematic sensor |

Records are found by linear scan from offset 10, stepping `12 + data_len` each
time; there is no index. A record is identified by the **pair** (`id`, `range`),
so the same sensor appears once per range for which a set exists.

An all-zero timestamp means "never calibrated, this is a firmware default seed"
— `ShimCalib_singleSensorToCalibDump` is called with `setCalibTsZero = 1` on the
boot path and `0` when a genuine calibration is being recorded, in which case it
stamps `RTC_getRwcTime()`.

### 2.3 Sensor IDs

The IDs are **disjoint between the two platforms**, so a blob identifies its own
hardware generation by the IDs it contains.

| ID | Constant | Gen | Payload |
|---:|---|:--:|---:|
| 2 | `SC_SENSOR_ANALOG_ACCEL` | S3 | 21 |
| 30 | `SC_SENSOR_MPU9X50_ICM20948_GYRO` | S3 | 21 |
| 31 | `SC_SENSOR_LSM303_ACCEL` | S3 | 21 |
| 32 | `SC_SENSOR_LSM303_MAG` | S3 | 21 |
| 33 | `SC_SENSOR_MPU9X50_ICM20948_ACCEL` | S3 | — |
| 34 | `SC_SENSOR_MPU9X50_ICM20948_MAG` | S3 | — |
| 36 | `SC_SENSOR_BMP180_PRESSURE` | S3 | 22 |
| 37 | `SC_SENSOR_LSM6DSV_ACCEL` | S3R | 21 |
| 38 | `SC_SENSOR_LSM6DSV_GYRO` | S3R | 21 |
| 39 | `SC_SENSOR_LIS2DW12_ACCEL` | S3R | 21 |
| 40 | `SC_SENSOR_ADXL371_ACCEL` | S3R | 21 |
| 41 | `SC_SENSOR_LIS3MDL_MAG` | S3R | 21 |
| 42 | `SC_SENSOR_LIS2MDL_MAG` | S3R | 21 |
| 43 | `SC_SENSOR_BMP390_PRESSURE` | S3R | — |
| 100 | `SC_SENSOR_HOST_ECG` | both | — |
| 0xFF | `SC_SENSOR_ALL` | both | Wildcard, not a record ID |

> **IDs 33, 34, 43 and 100 are declared but never written.**
> `ShimCalib_defaultAll` seeds only 2/30/31/32 on Shimmer3 and
> 37/38/39/40/41/42 on Shimmer3R, and `ShimCalib_findLength` returns **0** for
> 33, 34, 43 and 100. A record with one of those IDs cannot round-trip: it would
> be accepted into the blob by a `SET_CALIB_DUMP` but read back with a
> zero-length payload. Pressure calibration reaches the host over a separate
> command (§5), not through the blob, despite IDs 36 and 43 existing.

### 2.4 Size limits and paging

| Constant | Value | Meaning |
|---|---:|---|
| `SHIMMER_CALIB_RAM_MAX` | 1024 | Maximum blob size, both platforms (S3R via `INFOMEM_CALIB_SIZE = 0x400`) |
| `SHIMMER_CALIB_COPY_SIZE` | 128 | Paging unit for the SD file and the Bluetooth transfer |
| `SHIMMER_CALIB_DATA_MAX` | 22 | Largest per-record payload the `sc_data_u` union can hold |

`ShimCalib_ramWrite` and `ShimCalib_ramRead` both reject any request with
`length > 128`, `offset > 1023`, or `length + offset > 1024`.

A `SET_CALIB_DUMP` transfer is **stateful and must start at offset 0**. The
first chunk (offset 0 or 1) is what tells the firmware the total size — it reads
the length field out of the chunk itself and sets the expected byte count to
`length + 2`. Chunks at offset 2 or beyond only accumulate a running total.
Starting a transfer part-way through therefore accumulates against a stale
expected size, and the firmware commits the blob as soon as the running total
reaches it. On commit, `ShimCalib_ramWrite` re-stamps the version bytes, clears
the staging buffer, and pushes the new values out to the configuration bytes and
the SD header via `ShimCalib_calibDumpToConfigBytesAndSdHeaderAll(1)` — which
writes InfoMem.

## 3. The 21-byte kinematic block

Every kinematic sensor uses the same 21-byte payload, identical in the
configuration bytes, in the SD header, on the wire, and inside a calibration
dump record:

| Bytes | Field | Encoding |
|---:|---|---|
| 0-1 | Bias X | `int16`, **big-endian** |
| 2-3 | Bias Y | `int16`, big-endian |
| 4-5 | Bias Z | `int16`, big-endian |
| 6-7 | Sensitivity X | `int16`, big-endian |
| 8-9 | Sensitivity Y | `int16`, big-endian |
| 10-11 | Sensitivity Z | `int16`, big-endian |
| 12 | Alignment XX | `int8`, ÷ 100 |
| 13 | Alignment XY | `int8`, ÷ 100 |
| 14 | Alignment XZ | `int8`, ÷ 100 |
| 15 | Alignment YX | `int8`, ÷ 100 |
| 16 | Alignment YY | `int8`, ÷ 100 |
| 17 | Alignment YZ | `int8`, ÷ 100 |
| 18 | Alignment ZX | `int8`, ÷ 100 |
| 19 | Alignment ZY | `int8`, ÷ 100 |
| 20 | Alignment ZZ | `int8`, ÷ 100 |

### 3.1 The byte-order trap

Bias and sensitivity are **big-endian on the wire and in storage**, but the
`sc_data_u` struct the firmware manipulates them through is native-endian
(little-endian on both MSP430 and STM32). The firmware bridges this with
`ShimCalib_reverseBiasAndSensitivityByteOrder`, which byte-swaps all six 16-bit
fields in place.

Every `ShimCalib_setDefault*Calib` function calls it immediately before
`ShimCalib_singleSensorWrite`, so the values assigned in readable decimal in the
C source are stored byte-swapped. The alignment bytes are single bytes and are
never swapped.

The practical consequence for a host: **read bias and sensitivity as big-endian
signed 16-bit**, regardless of your own platform. Getting this wrong yields
sensitivities in the tens of thousands and calibrated output off by orders of
magnitude, rather than an obvious failure.

### 3.2 Scaling

- **Bias** is in raw ADC counts. Never scaled.
- **Alignment** is always divided by 100, giving a matrix whose entries are
  normally 0 or ±1.
- **Sensitivity** is in counts per physical unit and is divided by a
  per-sensor-type scale factor: **100 for gyroscopes**, **1 for accelerometers
  and magnetometers**. This mirrors `CALIBRATION_SCALE_FACTOR` in the Java
  driver, and matches the firmware seeding gyro sensitivity as
  `sensitivity * 100` (`ShimCalib_setDefaultLsm6dsvGyroCalib`).

### 3.3 Sentinel values

An all-`0xFF` or all-`0x00` 21-byte block means **no calibration stored**. Hosts
must treat either as "keep the default" rather than as literal parameters — an
all-zero block has zero sensitivity and would divide by zero.

The firmware applies the same rule in the other direction:
`ShimCalib_singleSensorToCalibDump` scans the configuration-byte block and only
promotes it into the dump if at least one byte differs from `0xFF`.

> **Known defect in that scan.** The loop is
> `for (byte_cnt = data_len; byte_cnt > 0; byte_cnt--)`, indexing
> `configBytePtr[21]` down to `configBytePtr[1]` for a 21-byte block. It reads
> one byte **past** the block and never examines byte **0**. A block whose only
> non-`0xFF` byte is its first (bias X high byte) is therefore misclassified as
> absent, and the validity decision depends on one byte of whatever follows the
> block in the configuration-byte array.

## 4. Where calibration blocks live

### 4.1 Configuration bytes (InfoMem)

| Sensor role | Constant | Offset | Gen |
|---|---|---:|:--:|
| Low-noise accel | `NV_LN_ACCEL_CALIBRATION` | 34 | both |
| Gyroscope | `NV_GYRO_CALIBRATION` | 55 | both |
| Magnetometer | `NV_MAG_CALIBRATION` | 76 | both |
| Wide-range accel | `NV_WR_ACCEL_CALIBRATION` | 97 | both |
| Alt. accel (ADXL371) | `NV_ALT_ACCEL_CALIBRATION` | 133 | S3R |
| Alt. mag (LIS3MDL) | `NV_ALT_MAG_CALIBRATION` | 154 | S3R |

Each occupies 21 bytes. The first four are contiguous from 34 to 117; the two
Shimmer3R additions live in the second 128-byte configuration page, which is why
the firmware splits the sync into
`ShimCalib_configBytes0To127ToCalibDumpBytes` and
`ShimCalib_configBytes128To255ToCalibDumpBytes`. See
[SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §7.

### 4.2 SD-card data-file header

| Sensor role | Constant | Offset | Timestamp constant | Offset |
|---|---|---:|---|---:|
| Wide-range accel | `SDH_WR_ACCEL_CALIBRATION` | 76 | `SDH_WR_ACCEL_CALIB_TS` | 182 |
| Gyroscope | `SDH_GYRO_CALIBRATION` | 97 | `SDH_GYRO_CALIB_TS` | 190 |
| Magnetometer | `SDH_MAG_CALIBRATION` | 118 | `SDH_MAG_CALIB_TS` | 198 |
| Low-noise accel | `SDH_LN_ACCEL_CALIBRATION` | 139 | `SDH_LN_ACCEL_CALIB_TS` | 206 |
| Temp/pressure | `SDH_TEMP_PRES_CALIBRATION` | 160 | — | — |
| Alt. accel | `SDH_ALT_ACCEL_CALIBRATION` | 256 | `SDH_ALT_ACCEL_CALIB_TS` | 277 |
| Alt. mag | `SDH_ALT_MAG_CALIBRATION` | 285 | `SDH_ALT_MAG_CALIB_TS` | 306 |

> **The SD-header order is not the InfoMem order.** InfoMem runs
> LN-accel, gyro, mag, WR-accel; the SD header runs WR-accel, gyro, mag,
> LN-accel. A host that assumes the two layouts are parallel will swap the
> low-noise and wide-range accelerometer calibrations. The full mapping is
> enumerated in [SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md).

Unlike the configuration bytes, the SD header also carries an 8-byte
calibration timestamp per sensor, copied from the dump record.

### 4.3 The SD-card calibration file

`ShimCalib_ram2File` writes the whole RAM blob to:

```
/Calibration/calib_<last 4 characters of the Bluetooth MAC address>
```

The directory is tried as `/Calibration` first and `/calibration` second, and is
created as `/Calibration` if neither exists. The MAC suffix comes from
`ShimBt_macIdStrPtrGet() + 8` — the last four characters of the MAC string, not
the whole address — so one card can hold calibration for several devices, and
two devices whose MACs share their last four hex digits will collide.

The file is written in 128-byte chunks and is `length + 2` bytes long: a
byte-for-byte image of the blob, with no additional framing.

`ShimCalib_file2Ram` reads it back and returns 1 (failure) if neither directory
exists or the file is absent, leaving the RAM blob as the firmware defaults.

## 5. Pressure sensor coefficient blocks

Pressure sensors are not part of the kinematic model. Their factory trim
coefficients are read out of the part and passed to the host verbatim, for the
host to feed into that part's published compensation formula.

The current command is `GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND`, whose
reply is:

```
PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE  [len]  [sensorId]  [coefficients...]
```

where `len` is `1 + coefficientByteCount` — it counts the sensor-ID byte.

| `sensorId` | Part | Coefficient bytes |
|---:|---|---:|
| 0 | `PRESSURE_SENSOR_BMP180` | 22 |
| 1 | `PRESSURE_SENSOR_BMP280` | 24 |
| 2 | `PRESSURE_SENSOR_BMP390` | 21 |
| 3 | `PRESSURE_SENSOR_BMP581` | **0** |

> **The BMP581 deliberately returns zero coefficients.** It outputs
> pre-compensated data, so there is nothing for the host to apply. The firmware
> still answers with the sensor-ID byte rather than a NACK, precisely so a host
> can positively identify the fitted part — a NACK would be ambiguous with older
> firmware that does not implement the command at all.

Two legacy part-specific commands remain for backward compatibility:

| Command | Behaviour |
|---|---|
| `GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND` | Shimmer3 only. Returns the 22 coefficient bytes if a BMP180 is fitted, otherwise 22 bytes of `0x01` filler. **NACKed on Shimmer3R.** |
| `GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND` | Shimmer3 only. Returns the 24 coefficient bytes if a BMP280 is fitted, otherwise 24 bytes of `0x01` filler. **NACKed on Shimmer3R.** |

Both legacy commands answer with filler rather than an error when the wrong part
is fitted, which is indistinguishable from real data. New host code should use
the generic command and read the sensor ID.

## 6. Default seed values

When no calibration is stored, the firmware seeds a default set for every
sensor and every range it knows about. `ShimCalib_defaultAll` runs from
`ShimCalib_init`.

Values below are shown **as written in the C source** — that is, before
`ShimCalib_reverseBiasAndSensitivityByteOrder` byte-swaps bias and sensitivity
for storage. Alignment is shown after the ÷100 scaling, as the matrix a host
actually applies.

### 6.1 Shimmer3

| Sensor | ID | Range code | Bias | Sensitivity |
|---|---:|---|---:|---:|
| Analog (Kionix) accel | 2 | 0 | 2047 | 83 |
| MPU9x50 / ICM20948 gyro | 30 | 0 (±250 dps) | 0 | 13100 |
| | | 1 (±500 dps) | 0 | 6550 |
| | | 2 (±1000 dps) | 0 | 3280 |
| | | 3 (±2000 dps) | 0 | 1640 |
| LSM303 wide-range accel | 31 | 0 (±2 g) | 0 | 1631 |
| | | 1 (±4 g) | 0 | 815 |
| | | 2 (±8 g) | 0 | 408 |
| | | 3 (±16 g) | 0 | 135 |
| LSM303 mag | 32 | 1 (±1.3 Ga) | 0 | 1100 / 1100 / 980 |
| | | 2 (±1.9 Ga) | 0 | 855 / 855 / 760 |
| | | 3 (±2.5 Ga) | 0 | 670 / 670 / 600 |
| | | 4 (±4.0 Ga) | 0 | 450 / 450 / 400 |
| | | 5 (±4.7 Ga) | 0 | 400 / 400 / 355 |
| | | 6 (±5.6 Ga) | 0 | 330 / 330 / 295 |
| | | 7 (±8.1 Ga) | 0 | 230 / 230 / 205 |

Where three sensitivity values are given they are X / Y / Z; the LSM303
magnetometer is the only sensor with an anisotropic default. Note that the
magnetometer range codes start at **1**, not 0, and that the ±16 g
accelerometer default of 135 does not follow the halving pattern of the lower
ranges.

Alignment matrices, Shimmer3:

| Sensor | Matrix (row-major) |
|---|---|
| Analog accel, gyro | `[0, -1, 0, -1, 0, 0, 0, 0, -1]` |
| LSM303 accel, LSM303 mag | `[-1, 0, 0, 0, 1, 0, 0, 0, -1]` |

### 6.2 Shimmer3R

| Sensor | ID | Range code | Bias | Sensitivity (stored) |
|---|---:|---|---:|---:|
| LSM6DSV low-noise accel | 37 | 0 (±2 g) | 0 | 1672 |
| | | 1 (±4 g) | 0 | 836 |
| | | 2 (±8 g) | 0 | 418 |
| | | 3 (±16 g) | 0 | 209 |
| LSM6DSV gyro | 38 | 0 (±125 dps) | 0 | 22900 |
| | | 1 (±250 dps) | 0 | 11400 |
| | | 2 (±500 dps) | 0 | 5700 |
| | | 3 (±1000 dps) | 0 | 2900 |
| | | 4 (±2000 dps) | 0 | 1400 |
| | | 5 (±4000 dps) | 0 | 700 |
| LIS2DW12 wide-range accel | 39 | 0 (±2 g) | 0 | 1671 |
| | | 1 (±4 g) | 0 | 836 |
| | | 2 (±8 g) | 0 | 418 |
| | | 3 (±16 g) | 0 | 209 |
| ADXL371 alt. accel | 40 | 0 | 10 | 1 |
| LIS3MDL alt. mag | 41 | 0 (±4 Ga) | 0 | 6842 |
| | | 1 (±8 Ga) | 0 | 3421 |
| | | 2 (±12 Ga) | 0 | 2281 |
| | | 3 (±16 Ga) | 0 | 1711 |
| LIS2MDL mag | 42 | 0 | 0 | 667 |

> **The gyro sensitivities above are the stored values, already multiplied by
> 100.** The source assigns 229 / 114 / 57 / 29 / 14 / 7 and then applies
> `sensitivity = sensitivity * 100`. A host must divide by 100 again (§3.2) to
> recover counts per dps. The base values are integers, so ±125 dps resolves to
> 229 rather than the 228.6 the part's nominal 4.375 mdps/LSB would give — the
> default carries roughly 0.2 % of scale error by construction and is only a
> placeholder until a real calibration is written.

> **The ADXL371 default is a placeholder.** Bias 10 and sensitivity 1 are
> commented in the source as "+1 g" and "100 mg/LSB which equates to
> 1.0197 LSB/m/s^2". A sensitivity of 1 count per unit will not produce sane
> physical values; this part needs a real calibration before its data is usable.

Alignment matrices, Shimmer3R:

| Sensor | Matrix (row-major) |
|---|---|
| LSM6DSV accel, LSM6DSV gyro | `[-1, 0, 0, 0, 1, 0, 0, 0, -1]` |
| LIS2DW12 accel | `[0, -1, 0, -1, 0, 0, 0, 0, -1]` |
| ADXL371 accel | `[0, 1, 0, 1, 0, 0, 0, 0, -1]` |
| LIS2MDL mag | `[-1, 0, 0, 0, -1, 0, 0, 0, -1]` |
| LIS3MDL mag | `[1, 0, 0, 0, -1, 0, 0, 0, -1]` |

All five differ. The alignment matrix is what reconciles each part's own axis
convention with the Shimmer body frame, so these are not interchangeable.

## 7. Calibration math

Raw counts are converted to physical units with the standard three-parameter
model (Ferraris, Grimaldi and Parvis, 1995):

```
C = inv(R) * inv(K) * (U - B)
```

| Term | Meaning |
|---|---|
| `U` | Raw sample, 3-vector of ADC counts |
| `B` | Bias vector, raw counts (§3, bytes 0-5) |
| `K` | Diagonal sensitivity matrix, `diag(sens_x, sens_y, sens_z)` after the §3.2 scale division |
| `R` | Alignment matrix, the nine `int8` values ÷ 100, row-major |
| `C` | Calibrated 3-vector in physical units |

Because `R` and `K` change only when calibration or range changes, both host
SDKs precompute `M = inv(R) * inv(K)` once and evaluate `C = M * (U - B)` per
sample.

Units by sensor type:

| Sensor | Calibrated unit |
|---|---|
| Accelerometer | m/s^2 |
| Gyroscope | deg/s |
| Magnetometer | counts ÷ sensitivity (see the note in *Still unverified*) |

### 7.1 Degenerate matrices

A singular alignment matrix, or a sensitivity axis of zero, makes the inverse
undefined. Both host SDKs substitute the identity for the offending inverse
rather than throwing, so calibration degrades to a pass-through instead of
producing `NaN` or crashing the stream. Hosts implementing this from scratch
should do the same: an all-zero 21-byte block is a realistic input (§3.3).

## 8. Persistence and precedence

Four copies exist and only some directions are automatic. The rules:

### 8.1 Configuration bytes to calibration dump

`ShimCalib_configBytesToCalibDump(id, setCalibTsZero)` reads the 21-byte block
out of the configuration bytes and writes it into the dump as a record tagged
with the **currently configured range** for that sensor. Two wrappers drive it:

| Function | When | Timestamp |
|---|---|---|
| `ShimCalib_initFromConfigBytesAll` | Boot | Zeroed |
| `ShimCalib_updateFromConfigBytesAll` | After a configuration change | `RTC_getRwcTime()` |

Only blocks that pass the "not all `0xFF`" test are promoted (§3.3).

Because the record is tagged with the *current* range, this direction cannot
populate sets for ranges that are not selected. Only a `SET_CALIB_DUMP` or the
firmware defaults can do that.

### 8.2 Calibration dump to configuration bytes and SD header

`ShimCalib_calibDumpToConfigBytesAndSdHeaderAll(writeToFlash)` walks every
sensor, looks up the record matching (sensor, current range), and copies its
payload into both the configuration bytes and the SD header, plus the 8-byte
timestamp into the SD header only. With `writeToFlash = 1` it also calls
`InfoMem_write` per sensor, making the change persistent.

> **A missing record silently zeroes the target.** `ShimCalib_singleSensorRead`
> memsets the caller's payload buffer before scanning, and returns 1 without
> restoring anything if no record matches. The caller here ignores that return
> value and copies the zeroed buffer onward. Selecting a range for which the
> dump holds no record therefore writes an all-zero calibration block into
> InfoMem and the SD header — which hosts correctly read as "no calibration"
> (§3.3), but which has also overwritten whatever was previously stored there.

### 8.3 Precedence

1. **A completed `SET_CALIB_DUMP` wins over everything.** It replaces the RAM
   blob wholesale and immediately pushes to configuration bytes, InfoMem and the
   SD header.
2. **The configuration bytes are the persistent authority across a power
   cycle.** The RAM blob is rebuilt from them at boot by
   `ShimCalib_initFromConfigBytesAll`, so anything present only in the RAM blob
   — most importantly, sets for non-selected ranges loaded from the SD file — is
   lost on reset unless it was written back.
3. **The SD file is a snapshot, not a live copy.** It is read only by an
   explicit `ShimCalib_file2Ram` and written only by an explicit
   `ShimCalib_ram2File`; it does not track changes made over Bluetooth on its
   own.
4. **The SD header is write-only from the firmware's point of view** — a record
   of what was in force when the file was created. Nothing ever reads
   calibration back out of it.

### 8.4 Docked deferral

Writing the calibration file requires the SD card, which is not available to the
firmware while the device is docked and the card is presented to the host. The
Bluetooth layer defers the write in that case — see `ShimBt_updateCalibDumpFile`
and [SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md)
§7.6. A `SET_CALIB_DUMP` accepted while docked still updates RAM, configuration
bytes and InfoMem immediately; only the SD file lags.

## 9. Host SDK mapping

| Concept | Java (`Shimmer-Java-Android-API`) | TypeScript (`shimmer-web-sdk`) |
|---|---|---|
| 21-byte block codec | `CalibDetailsKinematic.parseCalParamByteArray` / `generateCalParamByteArray` | `parseKinematicCalibBlock` / `generateKinematicCalibBlock` |
| Calibration math | `UtilCalibration.calibrateInertialSensorData` | `calibrateVector3` |
| Matrix inverse | `UtilCalibration.matrixInverse3x3` | `matrixInverse3x3` |
| Precomputed `M` | `CalibArraysKinematic` cached product | `KinematicCalibration.m` |
| Gyro scale factor | `mSensitivityScaleFactor = ONE_HUNDRED` | `ParseKinematicOptions.sensitivityScale` |
| Dump blob codec | dump parsing in `ShimmerObject` | `devices/calibration/dump.ts` |
| Default seeds | per-sensor classes under `sensors/` | `devices/calibration/defaults.ts` |

Two divergences are worth knowing about:

- **Java truncates, it does not round, when serialising the bias.**
  `generateCalParamByteArray` casts with `(int)`, discarding the fractional part
  toward zero, while sensitivity and alignment are rounded before their cast.
  The TypeScript SDK reproduces this deliberately (`Math.trunc` for offset,
  `Math.round` for the others) so the two agree byte-for-byte.
- **Java carries two different range-code tables for what looks like the same
  part.** `SensorLSM303AH` and `SensorLSM303DLHC` both declare
  `ListofLSM303AccelRangeConfigValues`, with *different* values — `{0, 2, 3, 1}`
  and `{0, 1, 2, 3}` respectively. Porting the wrong one silently selects the
  wrong full-scale range and produces calibrated output wrong by a factor of
  four. Take range codes from the firmware, not from a Java table, when the two
  disagree.

## Still unverified / not found in code

- **Magnetometer calibrated units.** The firmware and both SDKs divide counts by
  the stored sensitivity, but no source consulted states whether the intended
  output unit is Gauss, milligauss or µT. The Shimmer3 LSM303 defaults
  (1100 counts at ±1.3 Ga) and the Shimmer3R LIS3MDL defaults (6842 at ±4 Ga)
  are both consistent with counts-per-Gauss, but that is inference, not a
  statement found in the source.
- **`SC_SENSOR_HOST_ECG` (100).** Declared on both platforms, never written,
  never read, and `ShimCalib_findLength` gives it length 0. Its intended payload
  is not described anywhere in the firmware.
- **`SC_SENSOR_MPU9X50_ICM20948_ACCEL` (33) and `..._MAG` (34).** Declared but
  never seeded and never given a length. Whether these were ever populated by an
  older firmware, and therefore whether a legacy blob might contain them, is not
  determinable from the current source.
- **Shimmer3 `SC_SENSOR_BMP180_PRESSURE` (36) with `SC_DATA_LEN_BMP180 = 22`.**
  A length exists and `ShimCalib_findLength` returns it, but nothing in
  `ShimCalib_defaultAll` or the config-byte sync ever creates such a record, and
  the pressure coefficients reach the host by the separate command in §5.
  Whether a record with this ID can legitimately appear in a blob is unclear.
- **Shimmer3 magnetometer range code 0.** `SC_SENSOR_RANGE_MAX_LSM303_MAG` is 7
  and the seed loop runs `range < 7`, i.e. codes 0-6, but the value assignments
  are keyed to codes 1-7. Code 0 therefore receives the final `else` branch
  (the ±8.1 Ga sensitivities) and code 7 is never seeded at all. Whether the
  off-by-one is in the loop bound or in the constants was not resolved.
