# Shimmer3 / Shimmer3R Sensor Calibration

Per-device kinematic calibration — bias, sensitivity and axis alignment for each
accelerometer, gyroscope and magnetometer — is stored on the device, exposed to
host software over the Bluetooth protocol in two different shapes, mirrored into
the SD-card file header, and kept as a file on the SD card so a logged trial can
be converted after the fact.

This document is the byte-level specification of those blocks, shared by the
firmware and the host SDKs.

> **Verified against:**
> - **Firmware (authority for bytes):** `log-and-stream-common` @ `c13cbde` —
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
| Pressure coefficient blocks | `Comms/shimmer_bt_uart.c` — `case GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND` |
| Applying calibration | `Shimmer-Java-Android-API` — `driver/ShimmerObject.java`, `sensors/` |
| Host reference (TypeScript) | `shimmer-web-sdk` |

---

## 1. Overview

_TODO: what is calibrated on each platform, where it lives (config bytes, calibration RAM dump, SD file, SD header), how a host reads and writes it, and that pressure-sensor coefficients are a separate mechanism. Source: `Calibration/shimmer_calibration.{h,c}`._

## 2. Calibration dump blob

_TODO: the 10-byte global header (`SC_OFFSET_LENGTH_L` .. `SC_OFFSET_VER_FW_INTER_L`), then a sequence of per-sensor records each with a 12-byte header (`SC_OFFSET_SENSOR_ID_L`, range, length, 8-byte timestamp) followed by its data; total size limits `SHIMMER_CALIB_RAM_MAX` and the 128-byte paging unit `SHIMMER_CALIB_COPY_SIZE`. Source: `Calibration/shimmer_calibration.h` `SC_OFFSET_*`, `shimmer_calibration.c`._

## 3. The 21-byte kinematic block

_TODO: `gImuConfig` / `sc_default_t` — three `int16` biases, three `int16` sensitivities, nine `int8` alignment values; the byte order caveat (`ShimCalib_reverseBiasAndSensitivityByteOrder` — big-endian in the config bytes and on the wire, native in the struct); and the alignment scaling factor of 100. Source: `Configuration/shimmer_config.h` `gImuConfig`, `ShimCalib_reverseBiasAndSensitivityByteOrder`._

## 4. Where calibration blocks live

_TODO: the four legacy config-byte offsets (34, 55, 76, 97) plus the two Shimmer3R offsets (133, 154), their `SDH_*` counterparts, the calibration RAM dump, and `calib.ini` on the SD card. Cross-reference [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §7. Source: `Configuration/shimmer_config.h`, `ShimCalib_ram2File` / `ShimCalib_file2Ram`._

## 5. Pressure sensor coefficient blocks

_TODO: `GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND` and its `[len][sensorId][coefficients...]` reply, the per-part coefficient sizes (BMP180 22 bytes, BMP280, BMP390) and that BMP581 returns the sensor ID with zero coefficients because it outputs pre-compensated data; plus the two legacy part-specific commands. Source: `ShimBt_sendRsp`, `sensors/bmpX80/`._

## 6. Default seed values

_TODO: the per-part default bias/sensitivity/alignment the firmware seeds when calibration is absent or invalid, for every accel, gyro and mag on both platforms, and the per-range sensitivity tables. Source: `Calibration/shimmer_calibration.c` — `ShimCalib_setDefault*Calib`._

## 7. Calibration math

_TODO: the conversion `c = (R^-1)(K^-1)(u - b)` with R the alignment matrix scaled by 1/100, K the diagonal sensitivity matrix, b the bias vector, and u the raw sample; units per sensor type; and the degenerate-matrix fallback hosts apply. Source: `driver/ShimmerObject.java`._

## 8. Persistence and precedence

_TODO: the sync paths between the config bytes, the calibration RAM dump and the SD file (`ShimCalib_configBytesToCalibDump`, `ShimCalib_calibDumpToConfigBytesAndSdHeaderAll`, `ShimCalib_ram2File`, `ShimCalib_file2Ram`), which wins on a conflict, when the dump file is rewritten, and the docked-device deferral. Source: `Calibration/shimmer_calibration.c`, `ShimBt_updateCalibDumpFile`._

## 9. Host SDK mapping

_TODO: how the Java driver and the TypeScript SDK model these blocks, the range-code to sensitivity lookups each carries, and where each diverges from the firmware defaults. Source: `Shimmer-Java-Android-API` `sensors/`, `shimmer-web-sdk`._

## Still unverified / not found in code

- _TODO: populate as the doc pass proceeds._
