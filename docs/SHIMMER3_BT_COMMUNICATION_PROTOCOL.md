# Shimmer3 / Shimmer3R Bluetooth Communication Protocol

This document describes the byte-level command/response protocol spoken between a
**host** (PC, phone, browser, or another Shimmer acting as a sync centre) and a
**Shimmer3** or **Shimmer3R** running **LogAndStream** firmware. The protocol is
transport-agnostic: the same byte sequences travel over classic Bluetooth SPP, over
BLE, and over the dock's serial link.

> **Verified against:**
> - **Firmware (authority for bytes):** `log-and-stream-common` @ `c13cbde` —
>   `Comms/shimmer_bt_uart.h` (opcode `#define`s), `Comms/shimmer_bt_uart.c`
>   (`ShimBt_dmaConversionDone` argument counts, `ShimBt_processCmd` handlers,
>   `ShimBt_sendRsp` response assembly, `ShimBt_assembleStatusBytes`,
>   `ShimBt_isCmdBlockedWhileSensing`, `ShimBt_isCmdAllowedWhileSdSyncing`),
>   `Comms/shimmer_sd_file_transfer.{h,c}`, `Configuration/shimmer_config.h`,
>   `Calibration/shimmer_calibration.h`, `log_and_stream_definitions.h`.
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4`
>   (`LogAndStream_Shimmer3/`), `shimmer3r-firmware` @ `a8f105e5`
>   (`LogAndStream_Shimmer3R/`).
> - **Host reference implementations:** `Shimmer-Java-Android-API` @ `edc3f7d9`
>   (v0.11.8_beta) — `driver/ShimmerObject.java`,
>   `bluetooth/ShimmerBluetooth.java`, `bluetooth/BtCommandDetails.java`;
>   `shimmer-web-sdk` @ `8f78313` — `devices/shimmer3r/constants.ts`,
>   `devices/shimmer3/protocol.ts`, `devices/shimmer3r/sdTransfer/protocol.ts`;
>   `Extras/python_scripts/` in this repository.

> **How to read this document.** **S3** = Shimmer3 (MSP430, LogAndStream);
> **S3R** = Shimmer3R (STM32U5). The **Gen** column in the opcode tables says
> which generation *serves* an opcode — that is, whether firmware compiled for
> that platform actually acts on it: `both` = served on both, `S3` / `S3R` =
> served only there (the code sits behind a `#if defined(SHIMMER3)` /
> `#if defined(SHIMMER3R)` guard), and `— (reserved)` = the opcode number is
> declared in the shared header but no LogAndStream code on either platform
> references it. Note that nearly every opcode `#define` itself is
> unconditional, so header visibility is not the same thing as being served.
>
> **Response payload length** counts the bytes that follow the response opcode,
> excluding both the leading `ACK` byte and any CRC bytes — the same convention
> the Java driver's `mExpectedResponseByteLength` uses. Lengths that depend on
> runtime state are given as a formula in the firmware's own terms.
>
> This document describes **LogAndStream** only. It is the single firmware lineage
> that both streams over Bluetooth and logs to SD. The historical **BtStream**
> (stream-only) and **SDLog** (log-only) firmware images share most of these
> opcodes but differ in which ones they serve and in some argument semantics;
> where that matters it is confined to the appendices, and any statement in the
> body of this document is a statement about LogAndStream.
>
> An opcode's number is protocol-wide even when only one generation serves it:
> hosts must not reuse a reserved number. Older firmware silently ignores an
> unknown command byte (`default:` in `ShimBt_dmaConversionDone`) rather than
> NACKing it, so hosts gate optional features on `GET_FW_VERSION_COMMAND`
> rather than probing.

**Source references:**

| Layer | File |
|---|---|
| Opcode definitions | `Comms/shimmer_bt_uart.h` |
| Command parsing / argument counts | `Comms/shimmer_bt_uart.c` — `ShimBt_dmaConversionDone` |
| Command handlers | `Comms/shimmer_bt_uart.c` — `ShimBt_processCmd` |
| Response assembly | `Comms/shimmer_bt_uart.c` — `ShimBt_sendRsp` |
| Status byte assembly | `Comms/shimmer_bt_uart.c` — `ShimBt_assembleStatusBytes` |
| Command gating while sensing / syncing | `Comms/shimmer_bt_uart.c` — `ShimBt_isCmdBlockedWhileSensing`, `ShimBt_isCmdAllowedWhileSdSyncing` |
| CRC modes | `CRC/shimmer_crc.{h,c}` |
| SD file transfer (Shimmer3R) | `Comms/shimmer_sd_file_transfer.{h,c}` |
| SD sync | `SDSync/shimmer_sd_sync.{h,c}` |
| Configuration byte layout | `Configuration/shimmer_config.h` — see [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) |
| Streaming packet format | `Sensing/shimmer_sensing.h` — see [SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) |
| Calibration payloads | `Calibration/shimmer_calibration.{h,c}` — see [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) |
| Host reference (Python) | `Extras/python_scripts/` |
| Host reference (Java) | `Shimmer-Java-Android-API` — `driver/ShimmerObject.java`, `bluetooth/ShimmerBluetooth.java` |
| Host reference (TypeScript) | `shimmer-web-sdk` — `devices/shimmer3r/constants.ts`, `devices/shimmer3/protocol.ts` |

---

## 1. Overview

_TODO: What the protocol is, that it is host-initiated request/response with an ACK-then-response shape, that data packets flow unsolicited once streaming starts, and the LogAndStream lineage. Source: `Comms/shimmer_bt_uart.c` (`ShimBt_processCmd` / `ShimBt_sendRsp` pairing)._

## 2. Transport

### 2.1 Classic Bluetooth SPP

_TODO: RN42 / RN4678 (S3) and Vela IF820 / CYW20820 (S3R) modules, SPP as a raw byte stream with no framing, module status strings and the `RN4678_STATUS_STRING_SEPARATOR` in-band escape. Source: `shimmer3-firmware` `RN4X/`, `shimmer3r-firmware` `hal_CYW20820.c`, `Comms/shimmer_bt_uart.c`._

### 2.2 BLE (Shimmer3R)

_TODO: Shimmer3R-only BLE path, GATT service/characteristics, MTU and the resulting fragmentation, and the EZ-Serial SOF byte collision that forced `SD_LIST_DIR_COMMAND` to 0xCC. Source: `Comms/shimmer_bt_uart.h` (comment above the SD opcodes), `shimmer3r-firmware` `hal_CYW20820.c`._

### 2.3 Framing guarantees per transport

_TODO: There is no length field and no delimiter — the receiver is a state machine keyed on the opcode's fixed argument count; consequences for resynchronisation, and `DUMMY_COMMAND` as the no-op probe. Source: `Comms/shimmer_bt_uart.c` (`ShimBt_dmaConversionDone`), `BT_RX_COMMS_TIMEOUT_TICKS`._

### 2.4 Baud rates

_TODO: `enum BT_BAUD_RATE` values and which module supports which, the default, and why `SET_BT_COMMS_BAUD_RATE` no longer changes anything. Source: `Comms/shimmer_bt_uart.h` (`enum BT_BAUD_RATE`), `ShimBt_setBtBaudRateToUse`._

## 3. Message structure

### 3.1 Command frame

_TODO: `[opcode][fixed args...]` and the two variable-length shapes (in-band length byte at args[0], or at args[2] for ExG), `MAX_COMMAND_ARG_SIZE` = 131, and the oversize-payload clamp that NACKs rather than truncating. Source: `Comms/shimmer_bt_uart.c` (`ShimBt_dmaConversionDone` `waitingForArgsLength` block)._

### 3.2 Response frame and ACK

_TODO: ACK byte first, then the response opcode and payload in one transmission; NACK replaces the whole frame; `INSTREAM_CMD_RESPONSE` wrapping for responses that can arrive mid-stream; `SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE`. Source: `Comms/shimmer_bt_uart.c` (`ShimBt_sendRsp` prologue and the switch-set-NACK override)._

### 3.3 CRC modes

_TODO: `SET_CRC_COMMAND` and `COMMS_CRC_MODE` (off / 1 byte / 2 bytes), that the mode is a session setting appended to every response including in-stream ones and data packets, and the CRC algorithm. Source: `CRC/shimmer_crc.{h,c}`, `ShimBt_setCrcMode`, `calculateCrcAndInsert` call sites._

### 3.4 Size limits and paging

_TODO: `RESPONSE_PACKET_SIZE`, the 128-byte page limit on InfoMem / calibration / daughter-card reads and writes, and how hosts page a 512-byte InfoMem through it. Source: `Comms/shimmer_bt_uart.c` (`GET_INFOMEM_COMMAND` / `SET_INFOMEM_COMMAND` bounds checks), `log_and_stream_definitions.h`._

## 4. Opcode table

_The tables in this section are generated mechanically from the firmware header,
the firmware command parser and response assembler, and the two host
implementations. Do not hand-edit them._

The **Notes** column carries comparison flags, all of which are statements about
*named constants* rather than about behaviour:

| Flag | Meaning |
|---|---|
| `FW_ONLY` | No constant with this byte value exists in the Java driver. The driver may still handle the byte inline — this flags a missing *name*, not necessarily missing support. |
| `JAVA_ONLY` | The Java driver defines a constant for this byte value but the firmware header does not. Almost all are Shimmer2r-era or BtStream/SDLog-era opcodes; see [Appendix B](#appendix-b-java-only-and-legacy-opcodes). |
| `SDK_MISSING` | The TypeScript web SDK has no entry for this opcode. |
| `LEN_MISMATCH` | The Java registry's expected response payload length disagrees with the length the firmware assembles. |
| `no LogAndStream handler` | No code compiled for either platform references the opcode; the number is reserved only. |

### 4.1 Core

| Opcode | FW name | Kind | Args | Response opcode | Response payload length | Gen | Blocked while sensing | Java name (if different) | SDK name (if different) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| `0x00` | `DATA_PACKET` | DATA | 0 |  |  | both |  |  |  |  |
| `0x01` | `INQUIRY_COMMAND` | GET | 0 | `INQUIRY_RESPONSE` | S3: 8 + numberOfChannels<br>S3R: 11 + numberOfChannels | both |  |  |  |  |
| `0x02` | `INQUIRY_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x03` | `GET_SAMPLING_RATE_COMMAND` | GET | 0 | `SAMPLING_RATE_RESPONSE` | 2 | both |  |  |  |  |
| `0x04` | `SAMPLING_RATE_RESPONSE` | RSP |  |  |  | both |  |  |  | SDK payload len 2 |
| `0x05` | `SET_SAMPLING_RATE_COMMAND` | SET | 2 |  |  | both | yes |  |  |  |
| `0x06` | `TOGGLE_LED_COMMAND` | CTRL | 0 |  |  | both |  |  |  |  |
| `0x07` | `START_STREAMING_COMMAND` | CTRL | 0 |  |  | both |  |  |  |  |
| `0x08` | `SET_SENSORS_COMMAND` | SET | 3 |  |  | both | yes |  |  |  |
| `0x09` | `SET_WR_ACCEL_RANGE_COMMAND` | SET | 1 |  |  | both | yes | `SET_ACCEL_SENSITIVITY_COMMAND` |  | Java also:  |
| `0x0A` | `WR_ACCEL_RANGE_RESPONSE` | RSP |  |  |  | both |  | `ACCEL_SENSITIVITY_RESPONSE` |  | Java also:  |
| `0x0B` | `GET_WR_ACCEL_RANGE_COMMAND` | GET | 0 | `WR_ACCEL_RANGE_RESPONSE` | 1 | both |  | `GET_ACCEL_SENSITIVITY_COMMAND` |  | Java also:  |
| `0x0E` | `SET_CONFIG_SETUP_BYTES_COMMAND` | SET | 4 |  |  | both | yes | `SET_CONFIG_BYTE0_COMMAND` |  |  |
| `0x0F` | `CONFIG_SETUP_BYTES_RESPONSE` | RSP |  |  |  | both |  | `CONFIG_BYTE0_RESPONSE` |  |  |
| `0x10` | `GET_CONFIG_SETUP_BYTES_COMMAND` | GET | 0 | `CONFIG_SETUP_BYTES_RESPONSE` | 4 | both |  | `GET_CONFIG_BYTE0_COMMAND` |  |  |
| `0x20` | `STOP_STREAMING_COMMAND` | CTRL | 0 |  |  | both |  |  |  |  |
| `0x21` | `SET_GSR_RANGE_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x22` | `GSR_RANGE_RESPONSE` | RSP |  |  |  | both |  |  |  | SDK payload len 1 |
| `0x23` | `GET_GSR_RANGE_COMMAND` | GET | 0 | `GSR_RANGE_RESPONSE` | 1 | both |  |  |  |  |
| `0x25` | `DEVICE_VERSION_RESPONSE` | RSP |  |  |  | both |  | `GET_SHIMMER_VERSION_RESPONSE` |  | SDK payload len 1 |
| `0x2E` | `GET_FW_VERSION_COMMAND` | GET | 0 | `FW_VERSION_RESPONSE` | 6 | both |  |  |  |  |
| `0x2F` | `FW_VERSION_RESPONSE` | RSP |  |  |  | both |  |  |  | SDK payload len 6 |
| `0x30` | `SET_CHARGE_STATUS_LED_COMMAND` | SET | 1 |  |  | both |  | `SET_BLINK_LED` |  |  |
| `0x31` | `CHARGE_STATUS_LED_RESPONSE` | RSP |  |  |  | both |  | `BLINK_LED_RESPONSE` |  |  |
| `0x32` | `GET_CHARGE_STATUS_LED_COMMAND` | GET | 0 | `CHARGE_STATUS_LED_RESPONSE` | 1 | both |  | `GET_BLINK_LED` |  |  |
| `0x35` | `BUFFER_SIZE_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x36` | `GET_BUFFER_SIZE_COMMAND` | GET | 0 | `BUFFER_SIZE_RESPONSE` | 1 | both |  |  |  |  |
| `0x37` | `SET_MAG_GAIN_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x38` | `MAG_GAIN_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x39` | `GET_MAG_GAIN_COMMAND` | GET | 0 | `MAG_GAIN_RESPONSE` | 1 | both |  |  |  |  |
| `0x3A` | `SET_MAG_SAMPLING_RATE_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x3B` | `MAG_SAMPLING_RATE_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x3C` | `GET_MAG_SAMPLING_RATE_COMMAND` | GET | 0 | `MAG_SAMPLING_RATE_RESPONSE` | 1 | both |  |  |  |  |
| `0x3D` | `UNIQUE_SERIAL_RESPONSE` | RSP |  |  |  | both |  |  |  | FW_ONLY |
| `0x3E` | `GET_UNIQUE_SERIAL_COMMAND` | GET | 0 | `UNIQUE_SERIAL_RESPONSE` | S3: 8<br>S3R: 12 | both |  |  |  | FW_ONLY |
| `0x3F` | `GET_DEVICE_VERSION_COMMAND` | GET | 0 | `DEVICE_VERSION_RESPONSE` | 1 | both |  | `GET_SHIMMER_VERSION_COMMAND_NEW` |  |  |
| `0x40` | `SET_WR_ACCEL_SAMPLING_RATE_COMMAND` | SET | 1 |  |  | both | yes | `SET_ACCEL_SAMPLING_RATE_COMMAND` |  | Java also:  |
| `0x41` | `WR_ACCEL_SAMPLING_RATE_RESPONSE` | RSP |  |  |  | both |  | `ACCEL_SAMPLING_RATE_RESPONSE` |  | Java also:  |
| `0x42` | `GET_WR_ACCEL_SAMPLING_RATE_COMMAND` | GET | 0 | `WR_ACCEL_SAMPLING_RATE_RESPONSE` | 1 | both |  | `GET_ACCEL_SAMPLING_RATE_COMMAND` |  | Java also:  |
| `0x43` | `SET_WR_ACCEL_LPMODE_COMMAND` | SET | 1 |  |  | both | yes | `SET_LSM303DLHC_ACCEL_LPMODE_COMMAND` |  | Java also:  |
| `0x44` | `WR_ACCEL_LPMODE_RESPONSE` | RSP |  |  |  | both |  | `LSM303DLHC_ACCEL_LPMODE_RESPONSE` |  | Java also:  |
| `0x45` | `GET_WR_ACCEL_LPMODE_COMMAND` | GET | 0 | `WR_ACCEL_LPMODE_RESPONSE` | 1 | both |  | `GET_LSM303DLHC_ACCEL_LPMODE_COMMAND` |  | Java also:  |
| `0x46` | `SET_WR_ACCEL_HRMODE_COMMAND` | SET | 1 |  |  | both | yes | `SET_LSM303DLHC_ACCEL_HRMODE_COMMAND` |  | Java also:  |
| `0x47` | `WR_ACCEL_HRMODE_RESPONSE` | RSP |  |  |  | both |  | `LSM303DLHC_ACCEL_HRMODE_RESPONSE` |  | Java also:  |
| `0x48` | `GET_WR_ACCEL_HRMODE_COMMAND` | GET | 0 | `WR_ACCEL_HRMODE_RESPONSE` | 1 | both |  | `GET_LSM303DLHC_ACCEL_HRMODE_COMMAND` |  | Java also:  |
| `0x49` | `SET_GYRO_RANGE_COMMAND` | SET | 1 |  |  | both | yes | `SET_LSM6DSV_GYRO_RANGE_COMMAND` |  | Java also: SET_MPU9150_GYRO_RANGE_COMMAND |
| `0x4A` | `GYRO_RANGE_RESPONSE` | RSP |  |  |  | both |  | `LSM6DSV_GYRO_RANGE_RESPONSE` |  | Java also: MPU9150_GYRO_RANGE_RESPONSE |
| `0x4B` | `GET_GYRO_RANGE_COMMAND` | GET | 0 | `GYRO_RANGE_RESPONSE` | 1 | both |  | `GET_LSM6DSV_GYRO_RANGE_COMMAND` |  | Java also: GET_MPU9150_GYRO_RANGE_COMMAND |
| `0x4C` | `SET_GYRO_SAMPLING_RATE_COMMAND` | SET | 1 |  |  | both | yes | `SET_LSM6DSV_SAMPLING_RATE_COMMAND` |  | Java also: SET_MPU9150_SAMPLING_RATE_COMMAND |
| `0x4D` | `GYRO_SAMPLING_RATE_RESPONSE` | RSP |  |  |  | both |  | `LSM6DSV_SAMPLING_RATE_RESPONSE` |  | Java also: MPU9150_SAMPLING_RATE_RESPONSE |
| `0x4E` | `GET_GYRO_SAMPLING_RATE_COMMAND` | GET | 0 | `GYRO_SAMPLING_RATE_RESPONSE` | 1 | both |  | `GET_LSM6DSV_SAMPLING_RATE_COMMAND` |  | Java also: GET_MPU9150_SAMPLING_RATE_COMMAND |
| `0x4F` | `SET_ALT_ACCEL_RANGE_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x50` | `ALT_ACCEL_RANGE_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x51` | `GET_ALT_ACCEL_RANGE_COMMAND` | GET | 0 | `ALT_ACCEL_RANGE_RESPONSE` | 1 | both |  |  |  |  |
| `0x52` | `SET_PRESSURE_OVERSAMPLING_RATIO_COMMAND` | SET | 1 |  |  | both | yes | `SET_BMP180_PRES_RESOLUTION_COMMAND` |  | Java also:  |
| `0x53` | `PRESSURE_OVERSAMPLING_RATIO_RESPONSE` | RSP |  |  |  | both |  | `BMP180_PRES_RESOLUTION_RESPONSE` |  | Java also:  |
| `0x54` | `GET_PRESSURE_OVERSAMPLING_RATIO_COMMAND` | GET | 0 | `PRESSURE_OVERSAMPLING_RATIO_RESPONSE` | 1 | both |  | `GET_BMP180_PRES_RESOLUTION_COMMAND` |  | Java also:  |
| `0x5A` | `RESET_TO_DEFAULT_CONFIGURATION_COMMAND` | CTRL | 0 |  |  | both | yes |  |  |  |
| `0x5E` | `SET_INTERNAL_EXP_POWER_ENABLE_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x5F` | `INTERNAL_EXP_POWER_ENABLE_RESPONSE` | RSP |  |  |  | both |  |  |  | SDK payload len 1 |
| `0x60` | `GET_INTERNAL_EXP_POWER_ENABLE_COMMAND` | GET | 0 | `INTERNAL_EXP_POWER_ENABLE_RESPONSE` | 1 | both |  |  |  |  |
| `0x61` | `SET_EXG_REGS_COMMAND` | SET | 3 + [args[2]] bytes |  |  | both | yes |  |  |  |
| `0x62` | `EXG_REGS_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x63` | `GET_EXG_REGS_COMMAND` | GET | 3 | `EXG_REGS_RESPONSE` | 1 + exgLength | both |  |  |  |  |
| `0x64` | `SET_DAUGHTER_CARD_ID_COMMAND` | SET | 2 + [args[0]] bytes |  |  | both | yes |  |  | FW_ONLY |
| `0x65` | `DAUGHTER_CARD_ID_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x66` | `GET_DAUGHTER_CARD_ID_COMMAND` | GET | 2 | `DAUGHTER_CARD_ID_RESPONSE` | 1 + dcMemLength | both |  |  |  |  |
| `0x67` | `SET_DAUGHTER_CARD_MEM_COMMAND` | SET | 3 + [args[2]] bytes |  |  | both | yes |  |  | FW_ONLY |
| `0x68` | `DAUGHTER_CARD_MEM_RESPONSE` | RSP |  |  |  | both |  |  |  | FW_ONLY |
| `0x69` | `GET_DAUGHTER_CARD_MEM_COMMAND` | GET | 3 | `DAUGHTER_CARD_MEM_RESPONSE` | 1 + dcMemLength | both |  |  |  | FW_ONLY |
| `0x6D` | `SET_DERIVED_CHANNEL_BYTES` | SET | 8 |  |  | both | yes |  |  |  |
| `0x6E` | `DERIVED_CHANNEL_BYTES_RESPONSE` | RSP |  |  |  | both |  |  |  | Java registry expects 3 payload bytes, FW emits 8 (S3); LEN_MISMATCH |
| `0x6F` | `GET_DERIVED_CHANNEL_BYTES` | GET | 0 | `DERIVED_CHANNEL_BYTES_RESPONSE` | 8 | both |  |  |  |  |
| `0x8A` | `INSTREAM_CMD_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x8B` | `SET_CRC_COMMAND` | SET | 1 |  |  | both |  |  |  |  |
| `0x8C` | `SET_INFOMEM_COMMAND` | SET | 3 + [args[2]] bytes |  |  | both | yes |  |  |  |
| `0x8D` | `INFOMEM_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x8E` | `GET_INFOMEM_COMMAND` | GET | 3 | `INFOMEM_RESPONSE` | 1 + infomemLength | both |  |  |  |  |
| `0x94` | `VBATT_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x95` | `GET_VBATT_COMMAND` | GET | 0 | `INSTREAM_CMD_RESPONSE` | 2 | both |  |  |  |  |
| `0x96` | `TEST_CONNECTION_COMMAND` | CTRL | 0 |  |  | both |  |  |  |  |
| `0xA1` | `GET_BT_VERSION_STR_COMMAND` | GET | 0 | `BT_VERSION_STR_RESPONSE` | 1 + btVerStrLen | both |  | `GET_BT_FW_VERSION_STR_COMMAND` |  |  |
| `0xA2` | `BT_VERSION_STR_RESPONSE` | RSP |  |  |  | both |  | `BT_FW_VERSION_STR_RESPONSE` |  |  |
| `0xA3` | `SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE` | SET | 1 |  |  | both |  |  |  | FW_ONLY |
| `0xA4` | `SET_DATA_RATE_TEST` | SET | 1 |  | 0 | both | yes |  |  | FW_ONLY |
| `0xA5` | `DATA_RATE_TEST_RESPONSE` | RSP |  |  |  | both |  |  |  | FW_ONLY |
| `0xA8` | `SET_FACTORY_TEST` | SET | 1 |  |  | both | yes | `SET_TEST` |  |  |
| `0xAC` | `SET_ALT_ACCEL_SAMPLING_RATE_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0xAD` | `ALT_ACCEL_SAMPLING_RATE_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0xAE` | `GET_ALT_ACCEL_SAMPLING_RATE_COMMAND` | GET | 0 | `ALT_ACCEL_SAMPLING_RATE_RESPONSE` | 1 | both |  |  |  |  |
| `0xB2` | `SET_ALT_MAG_SAMPLING_RATE_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0xB3` | `ALT_MAG_SAMPLING_RATE_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0xB4` | `GET_ALT_MAG_SAMPLING_RATE_COMMAND` | GET | 0 | `ALT_MAG_SAMPLING_RATE_RESPONSE` | 1 | both |  |  |  |  |
| `0xB5` | `DUMMY_COMMAND` | CTRL | 0 |  |  | both |  |  |  | FW_ONLY |
| `0xB6` | `RESET_BT_ERROR_COUNTS` | CTRL | 0 |  |  | both |  |  |  | FW_ONLY |
| `0xB7` | `SET_FEATURE` | SET | 2 |  |  | both |  |  |  |  |
| `0xFE` | `NACK_COMMAND_PROCESSED` | ACK | 0 |  |  | both |  |  |  | FW_ONLY |
| `0xFF` | `ACK_COMMAND_PROCESSED` | ACK | 1 |  |  | both |  |  |  | permitted while SD syncing |

### 4.2 SD / trial configuration

| Opcode | FW name | Kind | Args | Response opcode | Response payload length | Gen | Blocked while sensing | Java name (if different) | SDK name (if different) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| `0x70` | `START_SDBT_COMMAND` | CTRL | 0 |  |  | both |  |  |  |  |
| `0x71` | `STATUS_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x72` | `GET_STATUS_COMMAND` | GET | 0 | `INSTREAM_CMD_RESPONSE` | 1 + ShimBt_assembleStatusBytes() | both |  |  |  |  |
| `0x73` | `SET_TRIAL_CONFIG_COMMAND` | SET | 3 |  |  | both | yes |  |  |  |
| `0x74` | `TRIAL_CONFIG_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x75` | `GET_TRIAL_CONFIG_COMMAND` | GET | 0 | `TRIAL_CONFIG_RESPONSE` | 3 | both |  |  |  |  |
| `0x76` | `SET_CENTER_COMMAND` | SET | 1 + [args[0]] bytes |  |  | both | yes |  |  |  |
| `0x77` | `CENTER_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x78` | `GET_CENTER_COMMAND` | GET | 0 | `CENTER_RESPONSE` | 1 | both |  |  |  |  |
| `0x79` | `SET_SHIMMERNAME_COMMAND` | SET | 1 + [args[0]] bytes |  |  | both | yes |  |  |  |
| `0x7A` | `SHIMMERNAME_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x7B` | `GET_SHIMMERNAME_COMMAND` | GET | 0 | `SHIMMERNAME_RESPONSE` | 1 + shimmer_name_len | both |  |  |  |  |
| `0x7C` | `SET_EXPID_COMMAND` | SET | 1 + [args[0]] bytes |  |  | both | yes |  |  |  |
| `0x7D` | `EXPID_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x7E` | `GET_EXPID_COMMAND` | GET | 0 | `EXPID_RESPONSE` | 1 + exp_id_name_len | both |  |  |  |  |
| `0x7F` | `SET_MYID_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x80` | `MYID_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x81` | `GET_MYID_COMMAND` | GET | 0 | `MYID_RESPONSE` | 1 | both |  |  |  |  |
| `0x82` | `SET_NSHIMMER_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x83` | `NSHIMMER_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x84` | `GET_NSHIMMER_COMMAND` | GET | 0 | `NSHIMMER_RESPONSE` | 1 | both |  |  |  |  |
| `0x85` | `SET_CONFIGTIME_COMMAND` | SET | 1 + [args[0]] bytes |  |  | both | yes |  |  |  |
| `0x86` | `CONFIGTIME_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x87` | `GET_CONFIGTIME_COMMAND` | GET | 0 | `CONFIGTIME_RESPONSE` | 1 + cfgtime_name_len | both |  |  |  |  |
| `0x88` | `DIR_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x89` | `GET_DIR_COMMAND` | GET | 0 | `INSTREAM_CMD_RESPONSE` | 2 + dir_len | both |  |  |  |  |
| `0x8F` | `SET_RWC_COMMAND` | SET | 8 |  |  | both |  |  |  |  |
| `0x90` | `RWC_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x91` | `GET_RWC_COMMAND` | GET | 0 | `RWC_RESPONSE` | 8 | both |  |  |  |  |
| `0x92` | `START_LOGGING_COMMAND` | CTRL | 0 |  |  | both |  | `START_LOGGING_ONLY_COMMAND` |  |  |
| `0x93` | `STOP_LOGGING_COMMAND` | CTRL | 0 |  |  | both |  | `STOP_LOGGING_ONLY_COMMAND` |  |  |
| `0x97` | `STOP_SDBT_COMMAND` | CTRL | 0 |  |  | both |  |  |  |  |
| `0x9C` | `UPD_SDLOG_CFG_COMMAND` | CTRL | 0 |  |  | both | yes |  |  | FW aliases: SET_I2C_BATT_STATUS_FREQ_COMMAND, UPD_SDLOG_CFG_COMMAND |
| `0xE0` | `SET_SD_SYNC_COMMAND` | SET | SYNC_PACKET_PAYLOAD_SIZE + BT_SD_SYNC_CRC_MODE |  |  | both |  | `ROUTINE_COMMUNICATION` |  | permitted while SD syncing |
| `0xE1` | `SD_SYNC_RESPONSE` | RSP |  |  |  | both |  |  |  | FW_ONLY |

### 4.3 Calibration

| Opcode | FW name | Kind | Args | Response opcode | Response payload length | Gen | Blocked while sensing | Java name (if different) | SDK name (if different) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| `0x11` | `SET_LN_ACCEL_CALIBRATION_COMMAND` | SET | SC_DATA_LEN_STD_IMU_CALIB |  |  | both | yes | `SET_ACCEL_CALIBRATION_COMMAND` |  | Java also:  |
| `0x12` | `LN_ACCEL_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  | `ACCEL_CALIBRATION_RESPONSE` |  | Java also:  |
| `0x13` | `GET_LN_ACCEL_CALIBRATION_COMMAND` | GET | 0 | `(dynamic: ShimBt_getExpectedRspForGetCmd())` | ShimBt_replySingleSensorCalibCmd() | both |  | `GET_ACCEL_CALIBRATION_COMMAND` |  | Java also:  |
| `0x14` | `SET_GYRO_CALIBRATION_COMMAND` | SET | SC_DATA_LEN_STD_IMU_CALIB |  |  | both | yes |  |  |  |
| `0x15` | `GYRO_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x16` | `GET_GYRO_CALIBRATION_COMMAND` | GET | 0 | `(dynamic: ShimBt_getExpectedRspForGetCmd())` | ShimBt_replySingleSensorCalibCmd() | both |  |  |  |  |
| `0x17` | `SET_MAG_CALIBRATION_COMMAND` | SET | SC_DATA_LEN_STD_IMU_CALIB |  |  | both | yes |  |  |  |
| `0x18` | `MAG_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x19` | `GET_MAG_CALIBRATION_COMMAND` | GET | 0 | `(dynamic: ShimBt_getExpectedRspForGetCmd())` | ShimBt_replySingleSensorCalibCmd() | both |  |  |  |  |
| `0x1A` | `SET_WR_ACCEL_CALIBRATION_COMMAND` | SET | SC_DATA_LEN_STD_IMU_CALIB |  |  | both | yes | `SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND` |  | Java also:  |
| `0x1B` | `WR_ACCEL_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  | `LSM303DLHC_ACCEL_CALIBRATION_RESPONSE` |  | Java also:  |
| `0x1C` | `GET_WR_ACCEL_CALIBRATION_COMMAND` | GET | 0 | `(dynamic: ShimBt_getExpectedRspForGetCmd())` | ShimBt_replySingleSensorCalibCmd() | both |  | `GET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND` |  | Java also:  |
| `0x2C` | `GET_ALL_CALIBRATION_COMMAND` | GET | 0 | `ALL_CALIBRATION_RESPONSE` | S3: ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd()<br>S3R: ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() | both |  |  |  |  |
| `0x2D` | `ALL_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x58` | `BMP180_CALIBRATION_COEFFICIENTS_RESPONSE` | RSP |  |  |  | S3 |  |  |  |  |
| `0x59` | `GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND` | GET | 0 | `S3 only: BMP180_CALIBRATION_COEFFICIENTS_RESPONSE` | S3: BMP180_CALIB_DATA_SIZE<br>S3R: 0 (NACK on unsupported hardware) | both |  |  |  |  |
| `0x5B` | `RESET_CALIBRATION_VALUE_COMMAND` | CTRL | 0 |  |  | both | yes |  |  |  |
| `0x5C` | `MPU9150_MAG_SENS_ADJ_VALS_RESPONSE` | RSP |  |  |  | S3 |  |  |  |  |
| `0x5D` | `GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND` | GET | 0 | `S3: MPU9150_MAG_SENS_ADJ_VALS_RESPONSE<br>S3R: ACK_COMMAND_PROCESSED` | S3: 4<br>S3R: 0 | both |  |  |  |  |
| `0x98` | `SET_CALIB_DUMP_COMMAND` | SET | 3 + [args[2]] bytes |  |  | both | yes |  |  |  |
| `0x99` | `RSP_CALIB_DUMP_COMMAND` | RSP |  |  |  | both |  |  |  |  |
| `0x9A` | `GET_CALIB_DUMP_COMMAND` | GET | 3 | `RSP_CALIB_DUMP_COMMAND` | 3 + calibRamLength | both |  |  |  |  |
| `0x9B` | `UPD_CALIB_DUMP_COMMAND` | CTRL | 0 |  |  | both | yes |  |  |  |
| `0x9F` | `BMP280_CALIBRATION_COEFFICIENTS_RESPONSE` | RSP |  |  |  | S3 |  |  |  |  |
| `0xA0` | `GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND` | GET | 0 | `S3 only: BMP280_CALIBRATION_COEFFICIENTS_RESPONSE` | S3: BMP280_CALIB_DATA_SIZE<br>S3R: 0 (NACK on unsupported hardware) | both |  |  |  |  |
| `0xA6` | `PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0xA7` | `GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND` | GET | 0 | `PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE` | S3: 4 + bmpCalibByteLen<br>S3R: 2 + bmpCalibByteLen | both |  |  |  |  |
| `0xA9` | `SET_ALT_ACCEL_CALIBRATION_COMMAND` | SET | SC_DATA_LEN_STD_IMU_CALIB |  |  | both | yes |  |  |  |
| `0xAA` | `ALT_ACCEL_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0xAB` | `GET_ALT_ACCEL_CALIBRATION_COMMAND` | GET | 0 | `(dynamic: ShimBt_getExpectedRspForGetCmd())` | ShimBt_replySingleSensorCalibCmd() | both |  |  |  |  |
| `0xAF` | `SET_ALT_MAG_CALIBRATION_COMMAND` | SET | SC_DATA_LEN_STD_IMU_CALIB |  |  | both | yes |  |  |  |
| `0xB0` | `ALT_MAG_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0xB1` | `GET_ALT_MAG_CALIBRATION_COMMAND` | GET | 0 | `(dynamic: ShimBt_getExpectedRspForGetCmd())` | ShimBt_replySingleSensorCalibCmd() | both |  |  |  |  |

### 4.4 Shimmer3R-served extensions

| Opcode | FW name | Kind | Args | Response opcode | Response payload length | Gen | Blocked while sensing | Java name (if different) | SDK name (if different) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| `0xC1` | `SD_LIST_DIR_RESPONSE` | RSP |  |  |  | S3R |  |  |  | FW_ONLY |
| `0xC2` | `SD_FILE_STAT_COMMAND` | CTRL | 1 + path |  | ShimSdFileTransfer_buildStatRsp() | S3R |  |  |  | FW_ONLY |
| `0xC3` | `SD_FILE_STAT_RESPONSE` | RSP |  |  |  | S3R |  |  |  | FW_ONLY |
| `0xC4` | `SD_FILE_READ_COMMAND` | CTRL | 11 + path |  |  | S3R |  |  |  | FW_ONLY |
| `0xC5` | `SD_FILE_DATA_RESPONSE` | RSP |  |  |  | S3R |  |  |  | FW_ONLY |
| `0xC6` | `SD_FILE_STATUS_RESPONSE` | RSP |  |  |  | S3R |  |  |  | FW_ONLY |
| `0xC7` | `SD_TRANSFER_ABORT_COMMAND` | CTRL | 0 |  |  | S3R |  |  |  | FW_ONLY |
| `0xC8` | `SD_FREE_SPACE_COMMAND` | CTRL | 0 |  | ShimSdFileTransfer_buildFreeSpaceRsp() | S3R |  |  |  | FW_ONLY |
| `0xC9` | `SD_FREE_SPACE_RESPONSE` | RSP |  |  |  | S3R |  |  |  | FW_ONLY |
| `0xCA` | `SD_DELETE_COMMAND` | CTRL | 1 + path |  | ShimSdFileTransfer_buildDeleteRsp() | S3R |  |  |  | FW_ONLY |
| `0xCB` | `SD_DELETE_RESPONSE` | RSP |  |  |  | S3R |  |  |  | FW_ONLY |
| `0xCC` | `SD_LIST_DIR_COMMAND` | CTRL | 4 + path |  | ShimSdFileTransfer_buildListDirRsp() | S3R |  |  |  | FW_ONLY |

### 4.5 Deprecated / no-op

| Opcode | FW name | Kind | Args | Response opcode | Response payload length | Gen | Blocked while sensing | Java name (if different) | SDK name (if different) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| `0x24` | `DEPRECATED_GET_DEVICE_VERSION_COMMAND` | CTRL | 0 | `DEVICE_VERSION_RESPONSE` | 1 | both |  | `GET_SHIMMER_VERSION_COMMAND` |  | SDK_MISSING |
| `0x6A` | `SET_BT_COMMS_BAUD_RATE` | SET | 1 |  |  | both | yes | `SET_BAUD_RATE_COMMAND` |  | SDK_MISSING |
| `0x6B` | `BT_COMMS_BAUD_RATE_RESPONSE` | RSP |  |  |  | both |  | `BAUD_RATE_RESPONSE` |  | SDK_MISSING |
| `0x6C` | `GET_BT_COMMS_BAUD_RATE` | GET | 0 | `BT_COMMS_BAUD_RATE_RESPONSE` | 1 | both |  | `GET_BAUD_RATE_COMMAND` |  | SDK_MISSING |

### 4.6 Not served by LogAndStream

| Opcode | FW name | Kind | Args | Response opcode | Response payload length | Gen | Blocked while sensing | Java name (if different) | SDK name (if different) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| `0x0C` | — |  |  |  |  | — |  | `SET_5V_REGULATOR_COMMAND` |  | JAVA_ONLY |
| `0x0D` | — |  |  |  |  | — |  | `SET_PMUX_COMMAND` |  | JAVA_ONLY |
| `0x26` | — |  |  |  |  | — |  | `SET_EMG_CALIBRATION_COMMAND` |  | JAVA_ONLY |
| `0x27` | — |  |  |  |  | — |  | `EMG_CALIBRATION_RESPONSE` |  | JAVA_ONLY |
| `0x28` | — |  |  |  |  | — |  | `GET_EMG_CALIBRATION_COMMAND` |  | JAVA_ONLY |
| `0x29` | — |  |  |  |  | — |  | `SET_ECG_CALIBRATION_COMMAND` |  | JAVA_ONLY |
| `0x2A` | — |  |  |  |  | — |  | `ECG_CALIBRATION_RESPONSE` |  | JAVA_ONLY |
| `0x2B` | — |  |  |  |  | — |  | `GET_ECG_CALIBRATION_COMMAND` |  | JAVA_ONLY |
| `0x33` | — |  |  |  |  | — |  | `SET_GYRO_TEMP_VREF_COMMAND` |  | JAVA_ONLY |
| `0x34` | — |  |  |  |  | — |  | `SET_BUFFER_SIZE_COMMAND` |  | JAVA_ONLY |
| `0x55` | — |  |  |  |  | — |  | `SET_BMP180_PRES_CALIBRATION_COMMAND` |  | JAVA_ONLY |
| `0x56` | — |  |  |  |  | — |  | `BMP180_PRES_CALIBRATION_RESPONSE` |  | JAVA_ONLY |
| `0x57` | — |  |  |  |  | — |  | `GET_BMP180_PRES_CALIBRATION_COMMAND` |  | JAVA_ONLY |
| `0x9D` | `RSP_I2C_BATT_STATUS_COMMAND` | RSP |  |  |  | S4 only |  |  |  | no LogAndStream handler; FW_ONLY, SDK_MISSING |
| `0x9E` | `GET_I2C_BATT_STATUS_COMMAND` | GET |  |  |  | S4 only |  |  |  | no LogAndStream handler; FW_ONLY, SDK_MISSING |

## 5. Status, ACK/NACK and in-stream responses

_TODO: `ACK_COMMAND_PROCESSED` 0xFF / `NACK_COMMAND_PROCESSED` 0xFE, the three NACK preconditions (sync-mode XOR, blocked-while-sensing, truncated payload), the status bit field from `ShimBt_assembleStatusBytes` (1 byte on S3, 2 on S3R), and unsolicited status pushes on dock/undock/button/low battery. Source: `ShimBt_processCmd` prologue, `ShimBt_assembleStatusBytes`, `ShimBt_instreamStatusRespSendIfNotBtCmd`._

## 6. Streaming data flow

_TODO: `START_STREAMING_COMMAND` / `STOP_STREAMING_COMMAND` / `START_SDBT_COMMAND` / `STOP_SDBT_COMMAND` / `START_LOGGING_COMMAND` / `STOP_LOGGING_COMMAND`, that `INQUIRY_COMMAND` must be re-issued after a configuration change because it is what fixes the channel order, and the interleaving of command responses with 0x00 data packets. Cross-reference [SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md). Source: `ShimBt_processCmd`, `ShimSens_configureChannels` call in `ShimBt_sendRsp`._

## 7. Command reference

### 7.1 Inquiry

`INQUIRY_COMMAND` is the only command that tells a host how to parse a data
packet. Everything else about the streaming format — channel widths, channel
order — follows from the channel-ID list this response carries.

- **Request:** `[0x01]` (no arguments)
- **Response:** `[ACK][0x02][payload]`

Before assembling the response the handler calls `ShimSens_configureChannels()`,
which recomputes the enabled-channel list and the packet length from the current
configuration.

> `Comms/shimmer_bt_uart.c:1785-1810` — `case INQUIRY_COMMAND`, with the
> `ShimSens_configureChannels()` call at `:1788`.

**Payload — Shimmer3** (8 fixed bytes, then the channel list):

| Offset | Size | Field | Encoding / Notes |
|---|---|---|---|
| 0..1 | 2 | Sampling-rate divider | `uint16_le`, InfoMem 0-1. Sample rate in Hz = `32768 / value` (`Configuration/shimmer_config.c:306`) |
| 2 | 1 | Config setup byte 0 | InfoMem 6 |
| 3 | 1 | Config setup byte 1 | InfoMem 7 |
| 4 | 1 | Config setup byte 2 | InfoMem 8 |
| 5 | 1 | Config setup byte 3 | InfoMem 9 |
| 6 | 1 | `numberOfChannels` | Number of channel-ID bytes that follow |
| 7 | 1 | `bufferSize` | InfoMem 2. Always `1` in current firmware (`ShimConfig_setDefaultConfig`, `shimmer_config.c:155`) |
| 8..8+n-1 | n | Channel IDs | One byte per channel, **in packet order** |

**Payload — Shimmer3R** (11 fixed bytes, then the channel list): identical, with
three extra configuration bytes inserted after config setup byte 3, so the
counts shift by three.

| Offset | Size | Field | Encoding / Notes |
|---|---|---|---|
| 0..1 | 2 | Sampling-rate divider | `uint16_le` |
| 2..5 | 4 | Config setup bytes 0-3 | InfoMem 6-9 |
| 6 | 1 | Config setup byte 4 | InfoMem 130 |
| 7 | 1 | Config setup byte 5 | InfoMem 131 |
| 8 | 1 | Config setup byte 6 | InfoMem 132 |
| 9 | 1 | `numberOfChannels` | |
| 10 | 1 | `bufferSize` | |
| 11..11+n-1 | n | Channel IDs | |

A host must therefore know the generation *before* it can find
`numberOfChannels`, and the only way to know the generation is
`GET_DEVICE_VERSION_COMMAND` ([§7.2](#72-version-information)). Read the version
first, then inquire.

**Normative rules for hosts.**

1. Re-issue `INQUIRY_COMMAND` after **every** configuration write that could
   change the enabled-sensor set or a sensor's range. The channel list is a
   snapshot, not a subscription.
2. Take channel widths and order from this list, never from the sensor
   enable bitmap. Channel IDs `0x0D`-`0x13` mean different physical inputs on
   the two generations, and the Shimmer3 magnetometer axis order depends on
   the fitted part — see [SHIMMER3_STREAMING_DATA_FORMAT.md §3](SHIMMER3_STREAMING_DATA_FORMAT.md#3-channel-id-registry)
   and [§4 Channel order rules](SHIMMER3_STREAMING_DATA_FORMAT.md#4-channel-order-rules).
3. The configuration bytes echoed here are the same bytes readable through
   `GET_CONFIG_SETUP_BYTES_COMMAND` and `GET_INFOMEM_COMMAND`; they are
   *post-correction* values (see [§7.5](#75-infomem)), so what the inquiry
   reports may differ from what the host last wrote.

> **Caution.** `GET_CONFIG_SETUP_BYTES_COMMAND` (0x10) returns only 4 bytes
> (InfoMem 6-9) on **both** generations, while the Shimmer3R inquiry response
> carries 7. There is no dedicated command for setup bytes 4-6; read them
> through the inquiry response or `GET_INFOMEM_COMMAND`, and write them through
> `SET_INFOMEM_COMMAND` or the individual rate/range commands.
> `Comms/shimmer_bt_uart.c:2077-2084` versus `:1798-1802`.

### 7.2 Version information

Four independent identity commands. A host should issue at least the first two
before anything else: the device version selects the channel and InfoMem
vocabulary, and the firmware version gates every optional feature
([Appendix A](#appendix-a-firmware-version-gates)).

#### `GET_DEVICE_VERSION_COMMAND` (0x3F)

- **Request:** `[0x3F]`
- **Response:** `[ACK][0x25][deviceVer]` — 1 payload byte

| `deviceVer` | Hardware |
|---|---|
| `3` | Shimmer3 |
| `10` | Shimmer3R |
| `58` | Shimmer4 SDK (not covered by this document) |

> The byte is the platform's `DEVICE_VER` macro, emitted at
> `Comms/shimmer_bt_uart.c:2100-2105`. Values from
> `shimmer3-firmware` `Shimmer_Driver/shimmer_definitions.h:54` and
> `shimmer3r-firmware` `Shimmer_Driver/shimmer_definitions.h:12`. They match the
> Java driver's `ShimmerVerDetails.HW_ID` constants
> (`driverUtilities/ShimmerVerDetails.java:28,35`).

`DEPRECATED_GET_DEVICE_VERSION_COMMAND` (0x24) is served by the identical
handler and returns the identical response — the two cases fall through
together. It is deprecated only because the byte `0x24` is ASCII `$`, which the
RN42 module interprets as the start of its remote-configuration escape when
that feature is enabled; the command byte could therefore be swallowed by the
module and never reach the firmware.

> `Comms/shimmer_bt_uart.h:79-81` carries the reason in a comment;
> `Comms/shimmer_bt_uart.c:2100-2101` shows the shared handler.

**Hosts should send 0x3F.** 0x24 exists so that pre-0x3F firmware can still be
identified.

#### `GET_FW_VERSION_COMMAND` (0x2E)

- **Request:** `[0x2E]`
- **Response:** `[ACK][0x2F][payload]` — 6 payload bytes

| Offset | Size | Field | Encoding |
|---|---|---|---|
| 0..1 | 2 | Firmware identifier | `uint16_le` |
| 2..3 | 2 | Version major | `uint16_le` |
| 4 | 1 | Version minor | `uint8` |
| 5 | 1 | Version patch | `uint8` |

| Identifier | Firmware lineage |
|---|---|
| `3` | LogAndStream (both generations) |
| `12` | Shimmer4 SDK LogAndStream |

> `Comms/shimmer_bt_uart.c:2107-2117`. `FW_IDENTIFIER` is `3` on both Shimmer3
> and Shimmer3R; the Java driver's `FW_ID.LOGANDSTREAM` is likewise `3`
> (`driverUtilities/ShimmerVerDetails.java:186`). Major/minor/patch come from
> the generated `version.h`, e.g. `FW_VERSION_STRING "v1.01.006"`.

The identifier does **not** distinguish the generations — both report `3`.
Generation comes from `GET_DEVICE_VERSION_COMMAND`; the version triple is only
meaningful within a lineage. See
[Appendix A](#appendix-a-firmware-version-gates) for why the four tag lineages
are not comparable with one another.

#### `GET_UNIQUE_SERIAL_COMMAND` (0x3E)

- **Request:** `[0x3E]`
- **Response:** `[ACK][0x3D][uid]` — **8** payload bytes on Shimmer3, **12** on
  Shimmer3R

The payload is the MCU's factory-programmed unique device ID, copied verbatim:
`HAL_GetUID()` on the MSP430, the three `HAL_GetUIDw0/1/2()` words on the
STM32U5. It is not a Shimmer serial number, it has no defined text form, and
nothing in the firmware interprets it — treat it as an opaque per-unit
identifier of generation-dependent length.

> `Comms/shimmer_bt_uart.c:2130-2145`.

⚠️ **Java driver gap.** The Java driver defines no constant for either `0x3D` or
`0x3E` (flagged `FW_ONLY` in [§4.1](#41-core)), so a Java-based host cannot ask
for the unique serial without sending the raw byte. The web SDK covers it.

#### `GET_BT_VERSION_STR_COMMAND` (0xA1)

- **Request:** `[0xA1]`
- **Response:** `[ACK][0xA2][len][ascii × len]` — `1 + len` payload bytes

The string is the Bluetooth module's own firmware banner, captured during module
bring-up and cached. On Shimmer3 it is the RN41/RN42/RN4678 version response with
any trailing `CMD>` prompt stripped; on Shimmer3R it is the CYW20820 EZ-Serial
application version. It is **not** null-terminated on the wire — take the length
from the length byte. The buffer is 100 bytes
(`Comms/shimmer_bt_uart.c:53`), which is sized for the longest banner either
module produces.

> `Comms/shimmer_bt_uart.c:2085-2093`; the capture and `CMD>` strip are at
> `:341-463`.

This is a diagnostic string with no stable format. Hosts should display it and
log it, and must not parse it to make protocol decisions — use
`GET_FW_VERSION_COMMAND` for that.

### 7.3 Device status

#### `GET_STATUS_COMMAND` (0x72)

- **Request:** `[0x72]`
- **Response:** `[ACK][0x8A][0x71][status × STATUS_BYTE_COUNT]`

The reply is wrapped in `INSTREAM_CMD_RESPONSE` (0x8A) because the firmware also
pushes this exact frame unsolicited while streaming — see
[§5](#5-status-acknack-and-in-stream-responses) for the byte layout, the bit
field, and the unsolicited-push rules. `STATUS_BYTE_COUNT` is **1** on Shimmer3
and **2** on Shimmer3R.

> `Comms/shimmer_bt_uart.c:1841-1847`; `ShimBt_assembleStatusBytes` at
> `:2920-2932`; `STATUS_BYTE_COUNT` at `Comms/shimmer_bt_uart.h:260-264`.

#### `GET_DIR_COMMAND` (0x89)

- **Request:** `[0x89]`
- **Response:** `[ACK][0x8A][0x88][len][ascii × len]` — `2 + len` payload bytes

Returns the *directory* portion of the current SD data-file path — the firmware
takes the cached file-name pointer and subtracts 3 from its length, dropping the
per-file numeric suffix. The string is not null-terminated on the wire.

> `Comms/shimmer_bt_uart.c:1905-1915`. Note `dir_len = strlen(fileNamePtr) - 3`
> at `:1908`.

The value is only meaningful once a logging session has established a path; the
firmware does not validate that a file exists. Like `GET_STATUS_COMMAND` this
reply is `INSTREAM_CMD_RESPONSE`-wrapped, so it is safe to issue while
streaming.

#### `TEST_CONNECTION_COMMAND` (0x96)

- **Request:** `[0x96]`
- **Response:** `[ACK]` — nothing else

The handler body is empty; the command exists purely so that a host can prove
the link is alive and the firmware is answering. Use it as a keep-alive or a
post-connect handshake probe.

> `Comms/shimmer_bt_uart.c:906-909`.

`DUMMY_COMMAND` (0xB5) is the *silent* equivalent and does not even ACK — see
[§7.12](#712-control-and-test).

#### `TOGGLE_LED_COMMAND` (0x06)

- **Request:** `[0x06]`
- **Response:** `[ACK]`

Flips `shimmerStatus.toggleLedRedCmd`. This is a **toggle**, not a set: there is
no command to drive the state to a known value, so a host that needs a known
state must read it back from status bit 7 and toggle again if it guessed wrong.
The bit is reported in the status byte on firmware new enough to carry it
(`isSupportedRedLedStateInStatus`, [Appendix A](#appendix-a-firmware-version-gates)).

> `Comms/shimmer_bt_uart.c:910-914`; the status bit at `:2922`.

### 7.4 Battery

#### `GET_VBATT_COMMAND` (0x95)

- **Request:** `[0x95]`
- **Response:** `[ACK][0x8A][0x94][adcLo][adcHi][chargerStatus]`

The handler forces a fresh measurement (`manageReadBatt(1)`) before assembling
the reply, so each call costs an ADC conversion and returns current data rather
than the cached periodic sample.

| Offset | Size | Field | Encoding / Notes |
|---|---|---|---|
| 0..1 | 2 | Battery ADC value | `uint16_le`, raw ADC counts — **not** millivolts |
| 2 | 1 | Charger status | bit 6 = `STAT1`, bit 7 = `STAT2`, bits 0-5 unused |

The two charger-chip pins are **active low** — a `0` bit means the pin is
asserted:

| bit7 `STAT2` | bit6 `STAT1` | Byte value (mask `0xC0`) | Meaning |
|---|---|---|---|
| 1 | 1 | `0xC0` | Charging suspended / no charger |
| 1 | 0 | `0x80` | Pre-conditioning |
| 0 | 1 | `0x40` | Fully charged |
| 0 | 0 | `0x00` | Bad battery |

> `Comms/shimmer_bt_uart.c:1848-1860`; the `BattStatusRaw` union, the
> `STAT1`/`STAT2` bit positions and the `CHRG_CHIP_STATUS_*` values are at
> `Battery/shimmer_battery.h:40-72`.

Converting the ADC value to millivolts needs the platform's ADC reference and
divider ratio, which are outside this document — see
[SHIMMER3_STREAMING_DATA_FORMAT.md §7.2](SHIMMER3_STREAMING_DATA_FORMAT.md#72-adc-and-battery),
which describes the same conversion for the `VBATT` streaming channel. The
firmware's own thresholds are in `Battery/shimmer_battery.h:17-26`.

⚠️ **The generated length column under-reports this response.** [§4.1](#41-core)
gives `GET_VBATT_COMMAND` a response payload length of 2; the firmware emits
**4** bytes after the `0x8A` response opcode (`0x94`, then three status bytes).
The three bytes are written in a `for` loop that the table's extractor could not
count statically. Firmware is authoritative: expect
`[ACK][0x8A][0x94][b0][b1][b2]`, i.e. 6 bytes with CRC off.
`Comms/shimmer_bt_uart.c:1853-1858`.

#### `GET_CHARGE_STATUS_LED_COMMAND` (0x32)

- **Request:** `[0x32]`
- **Response:** `[ACK][0x31][battStat]` — 1 payload byte

`battStat` is the firmware's own coarse battery band, computed from the ADC value
with hysteresis, not a raw reading:

| Value | Constant | Meaning |
|---|---|---|
| `0x01` | `BATT_LOW` | Low |
| `0x02` | `BATT_MID` | Medium |
| `0x04` | `BATT_HIGH` | High |

> `Comms/shimmer_bt_uart.c:2118-2123`; constants at
> `Battery/shimmer_battery.h:13-20`.

#### `SET_CHARGE_STATUS_LED_COMMAND` (0x30)

- **Request:** `[0x30][value]`
- **Response:** `[ACK]`

⚠️ **This command has no handler.** 0x30 is armed for one argument byte
(`Comms/shimmer_bt_uart.c:675`) and reaches `ShimBt_processCmd`, but there is no
`case SET_CHARGE_STATUS_LED_COMMAND` in the switch, so it falls to `default:`
(`:1607-1610`) and is ACKed without effect. The following
`GET_CHARGE_STATUS_LED_COMMAND` returns the firmware's own computed band,
unchanged. The Java driver names the pair `SET_BLINK_LED` / `GET_BLINK_LED`,
which reflects its historical Shimmer2 meaning; on LogAndStream the *get* is a
battery-level read and the *set* is inert. Do not use it.

### 7.5 InfoMem

InfoMem is the non-volatile configuration image. This section covers only how it
is read and written over Bluetooth; every byte's meaning is in
[SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md).

**Sizes.** The modelled configuration image is **384** bytes
(`NV_TOTAL_NUM_CONFIG_BYTES`) inside a **512**-byte addressable window
(`NV_NUM_RWMEM_BYTES` = `STOREDCONFIG_SIZE`). Both commands validate against the
512-byte window, so offsets 384-511 are addressable even though the firmware
models nothing there.

> `Configuration/shimmer_config.h:56,64-66`.

#### `GET_INFOMEM_COMMAND` (0x8E)

- **Request:** `[0x8E][len][offsetLo][offsetHi]`
- **Response:** `[ACK][0x8D][len][data × len]` — `1 + len` payload bytes

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | `len` — bytes to read, **max 128** |
| 1..2 | 2 | `offset` — `uint16_le` byte offset into the 512-byte window |

Validation is `len <= 128 && offset <= 511 && len + offset <= 512`.

> `Comms/shimmer_bt_uart.c:1383-1393` (validation),
> `:2279-2286` (response assembly).

> **Caution — a rejected read still ACKs.** When the bounds check fails the
> handler simply does not set `getCmdWaitingResponse`, so the command falls
> through to the common tail which sends a **bare ACK with no
> `INFOMEM_RESPONSE`** (`Comms/shimmer_bt_uart.c:1618-1626`). A host that waits
> for `0x8D` after seeing the ACK will hang. Validate `len` and `offset`
> host-side rather than relying on the firmware to reject them.

#### `SET_INFOMEM_COMMAND` (0x8C)

- **Request:** `[0x8C][len][offsetLo][offsetHi][data × len]`
- **Response:** `[ACK]`

Same field layout and the same 128-byte / 512-byte validation as the read.

> `Comms/shimmer_bt_uart.c:1394-1438`.

Writing a chunk does considerably more than copy bytes. In order:

1. The payload is copied into the RAM configuration image at `offset`.
2. **If `offset == 128`**, the six MAC-address bytes at InfoMem 224-229 are
   overwritten with the module's real MAC, unconditionally — the firmware does
   not check whether the host's chunk even reached byte 224. Whatever the host
   supplied for those bytes is discarded.
3. `ShimConfig_checkAndCorrectConfig()` runs, over the **whole** image, after
   **every** chunk. It clamps out-of-range fields and fills in defaults, so it
   can change bytes the host did not write, and it can change bytes the host
   *did* write in this very chunk.
4. The corrected image is flushed to InfoMem in full
   (`LogAndStream_infomemUpdate()`).
5. **If `offset == 0`**, the calibration dump's first half is regenerated from
   configuration bytes 0-127 (`ShimCalib_configBytes0To127ToCalibDumpBytes`) and
   the on-card calibration file is queued for rewrite.
6. **On Shimmer3R only, if `offset == 128`**, the dump's second half is
   likewise regenerated from bytes 128-255.
7. The SD header mirror is rebuilt and the SD configuration file is queued for
   rewrite.

> The `offset == 0` / `offset == 128` tests are written as
> `INFOMEM_SEG_D_ADDR_MSP430 - INFOMEM_OFFSET_MSP430` (= 0) and
> `INFOMEM_SEG_C_ADDR_MSP430 - INFOMEM_OFFSET_MSP430` (= 128) — the MSP430
> InfoMem segment addresses that the layout is still named after.
> `shimmer3-firmware` `Shimmer_Driver/5xx_HAL/hal_InfoMem.h:57,62-63`.

> **Caution — a rejected write is answered with silence.** On a bounds-check
> failure the handler executes a bare `return`
> (`Comms/shimmer_bt_uart.c:1433-1436`), which leaves `ShimBt_processCmd`
> *before* the ACK/NACK tail. The host gets **no reply at all** — not an ACK,
> not a NACK. This is the only command in the protocol that answers a malformed
> request with silence, and it is why a host needs a response timeout even on a
> reliable transport.

**Normative rules for hosts.**

1. Page the image in 128-byte chunks: offsets 0, 128, 256 (and 384 if you use
   the unmodelled tail). This is what both reference hosts do — the Python
   helper reads exactly `384` bytes in three 128-byte requests
   (`Extras/python_scripts/Shimmer_common/shimmer_comms_bluetooth.py:318-339`)
   and writes in the same stride (`:297-316`).
2. **Always read back after writing.** Step 3 above means the stored image is
   not necessarily what you sent, and there is no error report when it differs.
3. Write **ascending** offsets. Because the correction pass runs per chunk and
   sees a partially-updated image, a descending or interleaved write order can
   have the pass "correct" a field against bytes that have not been updated yet.
4. Do not attempt to preserve the MAC bytes across a write to offset 128;
   the firmware owns them.
5. After the last chunk, issue `UPD_SDLOG_CFG_COMMAND` (0x9C) if you want the
   SD configuration file rewritten promptly rather than at the next natural
   trigger — see [§7.7](#77-sd-logging-and-trial-configuration).

### 7.6 Calibration dump and per-sensor calibration

Two independent mechanisms reach the same calibration state: a whole-blob
transfer (the *calibration dump*), and six per-sensor 21-byte accessors. The
blob is authoritative; the per-sensor commands are a convenience path that
writes into the configuration image and then syncs the dump. The blob's internal
structure is in [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md).

Firmware calibration RAM is **1024** bytes (`SHIMMER_CALIB_RAM_MAX`,
`Calibration/shimmer_calibration.h:18`).

#### `GET_CALIB_DUMP_COMMAND` (0x9A)

- **Request:** `[0x9A][len][offsetLo][offsetHi]`
- **Response:** `[ACK][0x99][len][offsetLo][offsetHi][data × len]` —
  `3 + len` payload bytes

The response echoes the requested length and offset before the data, so a host
can match a reply to a request without keeping its own ordering state.

Validation happens inside `ShimCalib_ramRead`, not in the command handler:
`len <= 128 && offset <= 1023 && len + offset <= 1024`. On a violation the
destination is **zero-filled** rather than left unwritten, so an out-of-range
read yields a well-formed response full of zeros.

> `Comms/shimmer_bt_uart.c:2241-2250`; `Calibration/shimmer_calibration.c:372-386`.

The blob's first two bytes are its own total length (`uint16_le`, excluding those
two bytes), which is how a host discovers how much to read. The Python reference
requests 128 bytes, reads the length out of the first two payload bytes, and
continues from there
(`Extras/python_scripts/Shimmer_common/shimmer_comms_bluetooth.py:260-295`).

#### `SET_CALIB_DUMP_COMMAND` (0x98)

- **Request:** `[0x98][len][offsetLo][offsetHi][data × len]`
- **Response:** `[ACK]`

⚠️ **The firmware's own usage comments are wrong on both counts.**
`Comms/shimmer_bt_uart.c:1142-1158` documents the argument order as
`offset, offset, length` and labels the *get* case with opcode `0x98`. The code
reads `args[0]` as the **length** and `args[1..2]` as the little-endian
**offset**, and `GET_CALIB_DUMP_COMMAND` is `0x9A`. The argument-count machinery
agrees with the code — it arms three fixed bytes and then takes the payload
length from `args[0]` (`:470-487`) — as do both reference hosts. The wire format
is `[opcode][len][offsetLo][offsetHi]`, matching `GET`/`SET_INFOMEM_COMMAND` and
`GET`/`SET_DAUGHTER_CARD_MEM_COMMAND`. **Never take the argument order from
those comments.**

Writes are **accumulated**, not applied per chunk:

1. A chunk at `offset < 2` is treated as the start of a new transfer. The
   firmware reads the blob's declared total length out of the first two bytes,
   adds 2, and stores that as the expected total.
2. Every later chunk adds its own length to a running counter.
3. When the running counter reaches the declared total, the staging buffer is
   promoted to live calibration RAM, the version header is stamped, and the
   configuration bytes plus SD header are regenerated from the new dump.

> `Calibration/shimmer_calibration.c:176-215`. The staging-buffer semantics are
> in the function's own comment at `:189-191`: "to start a new calib_dump
> transmission, the sw must use offset = 0 to setup the correct length …
> starting with offset > 2 is not accepted."

**Normative rules for hosts.**

1. **Start at offset 0 and write strictly forward.** A transfer that begins at
   any offset ≥ 2 never establishes a total length, so it never completes.
2. **Do not expect a NACK for a malformed transfer.** The handler acts only on
   the `1` return (transfer complete) and ignores both the "still accumulating"
   `0` and the out-of-range `0xFF`
   (`Comms/shimmer_bt_uart.c:1158-1162`). An out-of-order or oversized write is
   discarded silently, after a normal ACK.
3. Chunk at 128 bytes, as the Python reference does
   (`shimmer_comms_bluetooth.py:240-258`).
4. Read the blob back with `GET_CALIB_DUMP_COMMAND` to confirm.

⚠️ **`SET_CALIB_DUMP_COMMAND` lacks the bounds check its InfoMem sibling has.**
`SET_INFOMEM_COMMAND` validates length and offset in the handler before
touching anything (`Comms/shimmer_bt_uart.c:1398-1400`); `SET_CALIB_DUMP_COMMAND`
calls `ShimCalib_ramWrite` directly (`:1158`). Containment comes from
`ShimCalib_ramWrite`'s own check plus the 131-byte `args[]` truncation clamp
(`:559-577`) — not from the handler.

#### `UPD_CALIB_DUMP_COMMAND` (0x9B)

- **Request:** `[0x9B]`
- **Response:** `[ACK]`

Re-applies the current calibration RAM to the configuration bytes and SD header,
and queues the on-card calibration file for rewrite. This is *not* required after
a normal `SET_CALIB_DUMP_COMMAND` sequence — step 3 above already does it once
the declared length is reached. Use it to force a re-apply, for example after
writing calibration bytes indirectly through `SET_INFOMEM_COMMAND`.

> `Comms/shimmer_bt_uart.c:1165-1170`.

#### Per-sensor calibration (six GET/SET pairs)

Each pair moves one 21-byte kinematic calibration block
(`SC_DATA_LEN_STD_IMU_CALIB`, `Calibration/shimmer_calibration.h:119`).

| Sensor | SET | RSP | GET | Shimmer3 part | Shimmer3R part |
|---|---|---|---|---|---|
| Low-noise accel | `0x11` | `0x12` | `0x13` | analog accel | LSM6DSV accel |
| Gyroscope | `0x14` | `0x15` | `0x16` | MPU9x50 / ICM-20948 | LSM6DSV gyro |
| Magnetometer | `0x17` | `0x18` | `0x19` | LSM303 mag | LIS2MDL mag |
| Wide-range accel | `0x1A` | `0x1B` | `0x1C` | LSM303 accel | LIS2DW12 accel |
| Alternative accel | `0xA9` | `0xAA` | `0xAB` | MPU9x50 / ICM-20948 accel | ADXL371 accel |
| Alternative mag | `0xAF` | `0xB0` | `0xB1` | MPU9x50 / ICM-20948 mag | LIS3MDL mag |

- **GET request:** `[opcode]` (no arguments)
- **GET response:** `[ACK][rspOpcode][21 bytes]`
- **SET request:** `[opcode][21 bytes]`
- **SET response:** `[ACK]`

The same opcode addresses a **different physical sensor on each generation** —
the table above is the whole story, and a host that has not read
`GET_DEVICE_VERSION_COMMAND` cannot label the block it just fetched. The read
path also stamps the block's range field from live configuration, so the reply
describes calibration *for the range currently configured*.

> Sensor-ID selection for the write path is at
> `Comms/shimmer_bt_uart.c:1183-1232` and `:1454-1479`; the read path, including
> the per-sensor range stamping, is `ShimBt_replySingleSensorCalibCmd` at
> `:1678-1751`. The response opcode is looked up by
> `ShimBt_getExpectedRspForGetCmd` (`:2351-2370`), which is why the generated
> table shows these rows as `(dynamic: ShimBt_getExpectedRspForGetCmd())`.

A SET writes through to the configuration image, flushes those 21 InfoMem bytes,
updates the SD header, syncs the block into calibration RAM, and queues the
on-card calibration file for rewrite.

> `ShimBt_calibrationChangeCommon`, `Comms/shimmer_bt_uart.c:1649-1661`.

#### `GET_ALL_CALIBRATION_COMMAND` (0x2C)

- **Request:** `[0x2C]`
- **Response:** `[ACK][0x2D][blocks]`

Concatenates the per-sensor blocks with no separators and no count:

| Generation | Blocks | Payload bytes |
|---|---|---|
| Shimmer3 | low-noise accel, gyro, mag, wide-range accel | 4 × 21 = **84** |
| Shimmer3R | the same four, then alternative accel, alternative mag | 6 × 21 = **126** |

> `Comms/shimmer_bt_uart.c:2171-2195`. The two extra calls are inside
> `#if defined(SHIMMER3R)` at `:2187-2193`.

The payload length is the only thing that distinguishes the two shapes, so a host
must know the generation before parsing. Prefer the calibration dump for a full
read: it is self-describing.

#### `RESET_CALIBRATION_VALUE_COMMAND` (0x5B)

- **Request:** `[0x5B]`
- **Response:** `[ACK]`

Re-seeds calibration RAM with the firmware's built-in defaults
(`ShimCalib_init()`), re-applies it to the configuration bytes and SD header, and
queues the on-card file for rewrite. **This discards per-unit factory
calibration** — there is no undo and no confirmation step. Blocked while
sensing.

> `Comms/shimmer_bt_uart.c:1301-1307`.

### 7.7 SD logging and trial configuration

These commands configure an autonomous SD-logging trial: what the unit is called,
which trial it belongs to, its role in a synchronised group, and the wall-clock
time it stamps data with. All of them except the two clock commands are blocked
while sensing.

#### `GET_TRIAL_CONFIG_COMMAND` (0x75) / `SET_TRIAL_CONFIG_COMMAND` (0x73)

- **GET request:** `[0x75]`
- **GET response:** `[ACK][0x74][cfg0][cfg1][interval]` — 3 payload bytes
- **SET request:** `[0x73][cfg0][cfg1][interval]`
- **SET response:** `[ACK]`

The three bytes are InfoMem 217, 218 and 219 (`NV_SD_TRIAL_CONFIG0`,
`NV_SD_TRIAL_CONFIG1`, `NV_SD_BT_INTERVAL`) — the trial flag bytes and the
sync Bluetooth interval in seconds. Individual bit meanings are in
[SHIMMER3_CONFIGURATION_INFOMEM.md §8](SHIMMER3_CONFIGURATION_INFOMEM.md#8-sd-logging-and-trial-configuration).

The write is a straight raw-byte copy of all three, then a flush of InfoMem
217-219 and the matching SD-header field, via `ShimBt_settingChangeCommon` —
which also runs the whole-image correction pass, so read back.

> `Comms/shimmer_bt_uart.c:980-989` (set), `:1861-1868` (get),
> `:1638-1647` (`ShimBt_settingChangeCommon`).

#### `GET_CENTER_COMMAND` (0x78) / `SET_CENTER_COMMAND` (0x76)

- **GET request:** `[0x78]`
- **GET response:** `[ACK][0x77][masterEnable]` — 1 payload byte
- **SET request:** `[0x76][len][data × len]`
- **SET response:** `[ACK]`

The *set* form is variable-length — the parser arms one byte, takes it as a
length, and then collects that many more
(`Comms/shimmer_bt_uart.c:500-511`) — but the handler reads only
`args[0] & 0x01`, the master-enable bit. Send `[0x76][0x01][0x00 or 0x01]`.

⚠️ **`SET_CENTER_COMMAND` does not survive a reboot.** The handler writes
`masterEnable` into the RAM image correctly, but then asks for the wrong InfoMem
byte to be flushed:

```c
case SET_CENTER_COMMAND:
{
  storedConfigPtr->masterEnable = args[0] & 0x01;
  ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE4, SDH_CONFIG_SETUP_BYTE4, 1);
  break;
}
```

> `Comms/shimmer_bt_uart.c:990-995`. `masterEnable` lives in the
> `SDTrialConfig0` bitfield group (`Configuration/shimmer_config.h:384`), which
> the struct layout places at InfoMem **217** (`NV_SD_TRIAL_CONFIG0`, i.e.
> `128 + 89`). The flush is directed at InfoMem **130**
> (`NV_CONFIG_SETUP_BYTE4`).

Observed consequences:

1. `GET_CENTER_COMMAND` reads back the new value for the rest of the session,
   because it reads RAM (`Comms/shimmer_bt_uart.c:1869-1873`). The command
   *appears* to work.
2. InfoMem 217 is never written, so **the setting is lost on the next boot**.
3. InfoMem 130 is flushed instead, rewriting the alternative-accel rate and the
   pressure-oversampling / low-power-mode / gyro-range MSB bits with their
   current RAM values.
4. The SD-header mirror is updated at the wrong offset for the same reason.

**Host workaround:** set the master-enable bit through
`SET_TRIAL_CONFIG_COMMAND` instead, which writes InfoMem 217 correctly
(`:980-989`) and is read back by `GET_TRIAL_CONFIG_COMMAND`. A host that must
know whether the setting persisted should verify with `GET_TRIAL_CONFIG_COMMAND`
or an InfoMem read, never with `GET_CENTER_COMMAND`.

#### `GET_SHIMMERNAME_COMMAND` (0x7B) / `SET_SHIMMERNAME_COMMAND` (0x79)

- **GET request:** `[0x7B]`
- **GET response:** `[ACK][0x7A][len][ascii × len]` — `1 + len` payload bytes
- **SET request:** `[0x79][len][ascii × len]`
- **SET response:** `[ACK]`

Stored in 12 InfoMem bytes at 187 (`NV_SD_SHIMMER_NAME`), not
null-terminated. Longer input is silently truncated to 12 bytes
(`ShimConfig_shimmerNameSet`, `Configuration/shimmer_config.c`). The read
returns the leading run of printable characters, at most 12; if the first byte is
not printable the firmware **regenerates the default name** `Shimmer_XXXX`
(where `XXXX` is the last four MAC hex digits) and returns that instead of an
empty string.

> `Comms/shimmer_bt_uart.c:996-1003` (set), `:1875-1884` (get);
> `ShimConfig_parseShimmerNameFromConfigBytes` and
> `ShimConfig_setDefaultShimmerName` in `Configuration/shimmer_config.c:244-245,782-792`.

A zero-length write (`[0x79][0x00]`) is dispatched and blanks the field, which
means the very next read returns the regenerated default — a blank name is not
representable.

Unlike most setters this one writes InfoMem directly and does **not** call
`ShimBt_settingChangeCommon`, so the whole-image correction pass does not run.

#### `GET_EXPID_COMMAND` (0x7E) / `SET_EXPID_COMMAND` (0x7C)

Identical shape to the Shimmer-name pair: 12 InfoMem bytes at 199
(`NV_SD_EXP_ID_NAME`), same truncation, same printable-prefix read, default
`DefaultTrial`.

- **GET response:** `[ACK][0x7D][len][ascii × len]`
- **SET request:** `[0x7C][len][ascii × len]`

> `Comms/shimmer_bt_uart.c:1004-1011`, `:1885-1894`;
> `Configuration/shimmer_config.c:250`.

#### `GET_CONFIGTIME_COMMAND` (0x87) / `SET_CONFIGTIME_COMMAND` (0x85)

- **GET request:** `[0x87]`
- **GET response:** `[ACK][0x86][len][ascii × len]` — `1 + len` payload bytes
- **SET request:** `[0x85][len][ascii × len]`
- **SET response:** `[ACK]`

The configuration timestamp is stored as a **4-byte big-endian** integer at
InfoMem 211 (`NV_SD_CONFIG_TIME`), but crosses the wire as a **decimal ASCII
string**. The write parses it with `atol` after copying at most 10 characters;
the read formats the stored integer back to decimal ASCII. There is no
null terminator on the wire in either direction.

> `Comms/shimmer_bt_uart.c:1012-1019` (set), `:1895-1904` (get);
> `ShimConfig_configTimeSetFromStr` in `Configuration/shimmer_config.c`;
> byte order at `:261,270`.

The value is a host-chosen epoch-seconds stamp identifying the configuration
generation. Nothing in the firmware validates it or compares it to the real-world
clock.

#### `GET_MYID_COMMAND` (0x81) / `SET_MYID_COMMAND` (0x7F)

- **GET response:** `[ACK][0x80][myTrialID]` — 1 payload byte
- **SET request:** `[0x7F][myTrialID]`

The unit's index within a synchronised trial. InfoMem 215
(`NV_SD_MYTRIAL_ID`). Stored verbatim — no range check.

> `Comms/shimmer_bt_uart.c:1026-1031`, `:1922-1927`.

#### `GET_NSHIMMER_COMMAND` (0x84) / `SET_NSHIMMER_COMMAND` (0x82)

- **GET response:** `[ACK][0x83][numberOfShimmers]` — 1 payload byte
- **SET request:** `[0x82][numberOfShimmers]`

The number of units in the trial. InfoMem 216 (`NV_SD_NSHIMMER`). Stored
verbatim.

> `Comms/shimmer_bt_uart.c:1020-1025`, `:1916-1921`.

#### `GET_RWC_COMMAND` (0x91) / `SET_RWC_COMMAND` (0x8F)

- **GET request:** `[0x91]`
- **GET response:** `[ACK][0x90][ticks × 8]` — 8 payload bytes
- **SET request:** `[0x8F][ticks × 8]`
- **SET response:** `[ACK]`

The real-world clock is a **64-bit little-endian count of 32768 Hz ticks**.
Milliseconds = ticks / 32.768.

> `Comms/shimmer_bt_uart.c:1439-1453` (set), `:2161-2170` (get). The Java driver
> encodes with `UtilShimmer.convertMilliSecondsToShimmerRtcDataBytesLSB`
> (`bluetooth/ShimmerBluetooth.java:691-696`) and decodes by reversing the eight
> bytes and dividing by 32.768 (`:1721-1728`).

Setting the clock has three side effects beyond the clock itself: it sets the
`rtcSetByBt` bit in InfoMem 217 and flushes that byte, re-runs the real-world
clock error check, and on Shimmer3R re-arms the periodic battery-read RTC alarm.
Once set, status bit 2 reports "clock has been set" on firmware new enough to
carry it (`isSupportedRtcStateInStatus`,
[Appendix A](#appendix-a-firmware-version-gates)).

> **Caution — the epoch is a host convention, not a firmware one.** The firmware
> stores and returns whatever 64-bit tick value it was given; it has no notion of
> time zone or of UTC. Host software has settled on **local civil time**, because
> that is what makes logged data split at local midnight. A host that writes UTC
> will produce files whose day boundaries do not match the operator's calendar.
> Nothing in the firmware detects or corrects this.

`SET_RWC_COMMAND` is **not** blocked while sensing — it is one of the few writes
permitted mid-recording ([§10](#10-important-boundaries)) — and its handler is
deliberately reached as early as possible after the argument bytes arrive, to
minimise the delay between the host reading its own clock and the device
adopting the value.

#### `UPD_SDLOG_CFG_COMMAND` (0x9C)

- **Request:** `[0x9C]`
- **Response:** `[ACK]`

Queues a rewrite of the SD card's `sdlog.cfg` from the current configuration
image (`TASK_SDLOG_CFG_UPDATE`). Blocked while sensing.

> `Comms/shimmer_bt_uart.c:1171-1175`.

Most configuration writes already queue this rewrite themselves, so an explicit
call is only needed after writes that do not — principally a sequence of
`SET_INFOMEM_COMMAND` chunks where the host wants the card updated before it
disconnects. The command number is shared with the Shimmer4 SDK's
`SET_I2C_BATT_STATUS_FREQ_COMMAND` (`Comms/shimmer_bt_uart.h:194-200`), which is
why [§4.2](#42-sd--trial-configuration) shows both names; on LogAndStream only
`UPD_SDLOG_CFG_COMMAND` exists.

### 7.8 Sensor settings

_TODO: the `SET_SENSORS_COMMAND` 3-byte enable bitmap and the per-sensor rate/range GET/SET triplets, each of which writes one config-setup byte and re-runs the correction pass. Source: `ShimBt_processCmd`._

### 7.9 ExG registers

_TODO: `GET_EXG_REGS_COMMAND` 0x63 `[chip][startAddr][len]` and `SET_EXG_REGS_COMMAND` 0x61 `[chip][startAddr][len][data...]`, the validation limits (chip < 2, addr < 10, len < 11), and the SR47-4+ CONFIG2 bit-3 forcing. Source: `ShimBt_processCmd`._

### 7.10 Daughter card

_TODO: `GET/SET_DAUGHTER_CARD_ID_COMMAND` (16-byte ID page) and `GET/SET_DAUGHTER_CARD_MEM_COMMAND` (the 2032-byte user area at EEPROM offset 16), with their bounds checks. Source: `ShimBt_processCmd`._

### 7.11 Derived channels

_TODO: `SET_DERIVED_CHANNEL_BYTES` 0x6D / `GET_DERIVED_CHANNEL_BYTES` 0x6F, the 8 bytes split 3 + 5 across two non-contiguous InfoMem regions, and that these are host-side algorithm flags the firmware only stores. Source: `ShimBt_processCmd`, `ShimBt_sendRsp`._

### 7.12 Control and test

_TODO: `SET_CRC_COMMAND`, `SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE`, `SET_DATA_RATE_TEST` / `DATA_RATE_TEST_RESPONSE`, `SET_FACTORY_TEST`, `SET_FEATURE`, `RESET_BT_ERROR_COUNTS`, `RESET_TO_DEFAULT_CONFIGURATION_COMMAND`, `DUMMY_COMMAND`. Source: `ShimBt_processCmd`._

### 7.13 SD file transfer (Shimmer3R)

_TODO: the 0xC1–0xCC block — list dir, stat, read with a window, data/status frames, abort, free space, delete — payload layouts and the asynchronous data-frame flow. Source: `Comms/shimmer_sd_file_transfer.{h,c}`, `shimmer-web-sdk` `devices/shimmer3r/sdTransfer/protocol.ts`._

### 7.14 SD sync

_TODO: `SET_SD_SYNC_COMMAND` 0xE0 / `SD_SYNC_RESPONSE` 0xE1, the centre/node roles, the ACK-carrying-a-command-byte shape, and that sync mode blocks all other commands. Source: `SDSync/shimmer_sd_sync.{h,c}`, `ShimBt_isCmdAllowedWhileSdSyncing`._

## 8. Connection session workflow

_TODO: the recommended host sequence — connect, `GET_FW_VERSION_COMMAND` to establish capability, optional `SET_CRC_COMMAND`, read InfoMem and calibration, `INQUIRY_COMMAND`, start streaming — and what must be re-read after each configuration write. Source: `Extras/python_scripts/`, `bluetooth/ShimmerBluetooth.java` connect state machine._

## 9. Host-side implementations

_TODO: table of the four host implementations (Python in this repo, Java driver, TypeScript web SDK, C#) with what each covers and where its opcode registry lives. Source: as listed in Source references._

## 10. Important boundaries

_TODO: what this protocol does not do — no length-prefixed framing, no flow control from the host, no negative acknowledgement of unknown opcodes, no atomic multi-byte configuration write, and the commands that are rejected while sensing. Source: `ShimBt_isCmdBlockedWhileSensing`, `ShimBt_dmaConversionDone` `default:` case._

## Appendix A. Firmware version gates

Host software gates optional protocol and layout features on the firmware
version reported by `GET_FW_VERSION_COMMAND`. The **host gate** column is the
normative answer for host authors: it names the Java driver predicate and the
LogAndStream version threshold it applies.

The final column is corroborating evidence only, obtained by locating the commit
that changed the corresponding firmware symbol earliest (`git log -S`) and taking
the earliest release tag per lineage that contains it. Read it with three
caveats:

- **`≤` means inconclusive.** The introducing commit predates that lineage's
  first tag, so the tag shown is merely the earliest tag that exists — the
  feature is at or before it. Old releases were tagged sparsely.
- **A symbol's earliest commit can be a rename, not the feature's birth.** Where
  the date is much later than the host threshold suggests, the symbol was
  probably renamed or moved into the shared module at that commit.
- **Tag lineages are separate product lines.** `LogAndStream_Shimmer3`,
  `LogAndStream_Shimmer3_BLE`, `LogAndStream_Shimmer3R` and the legacy
  unprefixed `LogAndStream` version numbers are not comparable with each other.

Release provenance is taken from the two platform repositories. The shared
module's own history begins when it was extracted, so its tags date the
extraction rather than the feature; it is consulted only for features that
exist nowhere else, which are exactly the ones added after the extraction.

| Area | Feature | Host gate / threshold | Firmware symbol | Earliest release tag containing the introducing commit |
|---|---|---|---|---|
| Status response | status bit 7 carries the red-LED toggle state | `isSupportedRedLedStateInStatus`<br>LogAndStream 0.7.10 | `toggleLedRedCmd` | Shimmer3: `LogAndStream_Shimmer3_BLE_v0.16.011`, `LogAndStream_Shimmer3_v1.00.003` (introducing commit c42e0f6, 2024-10-07)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 65aa6176, 2023-11-09) |
| Status response | status bits 6 and 5 carry SD bad-file / SD-inserted | `isSupportedSdInfoInStatus`<br>LogAndStream 0.7.12 | `sdBadFile` | Shimmer3: `LogAndStream_Shimmer3_BLE_v0.16.012`, `LogAndStream_Shimmer3_v1.00.003` (introducing commit ed662e8, 2024-11-05)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 42a86589, 2024-11-05) |
| Status response | status bit 2 carries "real-world clock has been set" | `isSupportedRtcStateInStatus`<br>LogAndStream 0.7.14 | `RTC_isRwcTimeSet` | Shimmer3: `LogAndStream_Shimmer3_BLE_v1.00.000`, `LogAndStream_Shimmer3_v1.00.003` (introducing commit e501346, 2025-06-30)<br>Shimmer3R: `LogAndStream_Shimmer3R_v1.00.027` (introducing commit b4964c95, 2025-06-30) |
| Status response | a SECOND status byte carrying USB-plugged-in (STATUS_BYTE_COUNT 2) | `isSupportedUSBPluggedInStatus`<br>LogAndStream 1.0.24 (Shimmer3R) | `usbPluggedIn` | Shimmer3R: `LogAndStream_Shimmer3R_v1.00.019` (introducing commit d1f311cf, 2025-02-28) |
| Commands | GET_BT_VERSION_STR_COMMAND (Bluetooth module firmware string) | `isSupportedBtFwVerRequest`<br>_no version threshold (hardware-keyed or unconditional)_ | `GET_BT_VERSION_STR_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000` (introducing commit 372a4de, 2023-02-08)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Commands | GET_STATUS_COMMAND | `isSupportedBtStatusRequest`<br>LogAndStream 0.5.2 (Shimmer3) | `GET_STATUS_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 8b7e616, 2014-07-22)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Commands | GET_VBATT_COMMAND | `isSupportedBtBatteryRequest`<br>LogAndStream 0.5.9 (Shimmer3) | `GET_VBATT_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 15dd04c, 2016-02-11)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Commands | UPD_SDLOG_CFG_COMMAND (rewrite the SD config file after an InfoMem write) | `isBtMemoryUpdateCommandSupported`<br>_no version threshold (hardware-keyed or unconditional)_ | `UPD_SDLOG_CFG_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 5eae006, 2016-09-16)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Calibration | GET/SET/UPD_CALIB_DUMP_COMMAND and the calibration-dump blob | `isSupportedCalibDump`<br>_no version threshold (hardware-keyed or unconditional)_ | `SET_CALIB_DUMP_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 5eae006, 2016-09-16)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Commands | SD logging commands | `isSupportedSdCardAccess`<br>_no version threshold (hardware-keyed or unconditional)_ | `START_LOGGING_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 15dd04c, 2016-02-11)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Dock | configuration over the dock UART | `isSupportedConfigViaUart`<br>_no version threshold (hardware-keyed or unconditional)_ | — | _no firmware symbol identified_ |
| Dock | real-world clock set over the dock UART | `isSupportedRtcConfigViaUart`<br>_no version threshold (hardware-keyed or unconditional)_ | — | _no firmware symbol identified_ |
| InfoMem | the MPL/9-DoF fusion calibration block | `isSupportedMpl`<br>_no version threshold (hardware-keyed or unconditional)_ | `NV_MPL_GYRO_CALIBRATION` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 00425d6, 2015-03-30)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| InfoMem | derived-channel bytes 3..7 at InfoMem 118-122 (8 bytes total, not 3) | `isSupportedEightByteDerivedSensors`<br>LogAndStream 0.7.1 | `NV_DERIVED_CHANNELS_3` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit d97bbd6, 2017-05-24)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| SD header | the expansion-board id in the SD file header | `isSupportedExpansionBrdIdInSdHeader`<br>LogAndStream 0.6.13 | — | _no firmware symbol identified_ |
| Commands | SET_FEATURE (runtime feature toggles) | — | `SET_FEATURE` | Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 52bb8f0e, 2024-03-27) |
| Commands | SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE | — | `SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE` | Shimmer3: `LogAndStream_Shimmer3_BLE_v0.16.001`, `LogAndStream_Shimmer3_v0.15.004` (introducing commit 25ae95d, 2023-02-17)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Commands | SET_CRC_COMMAND (response CRC modes) | — | `SET_CRC_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 392f693, 2015-03-30)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Calibration | GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND (sensor-agnostic pressure coefficients) | — | `GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND` | Shimmer3: `LogAndStream_Shimmer3_BLE_v0.16.006`, `LogAndStream_Shimmer3_v1.00.003` (introducing commit a5151db, 2024-07-08)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 707d1387, 2024-07-01) |
| Calibration | alternative accel / alternative mag calibration commands | — | `SET_ALT_ACCEL_CALIBRATION_COMMAND` | Shimmer3: `LogAndStream_Shimmer3_BLE_v0.16.013`, `LogAndStream_Shimmer3_v1.00.003` (introducing commit 6459f66, 2024-12-12)<br>Shimmer3R: `LogAndStream_Shimmer3R_v1.00.005` (introducing commit 15947690, 2024-11-28) |
| SD transfer | SD file transfer over Bluetooth (Shimmer3R) | — | `SD_FILE_READ_COMMAND` | shared module: `LogAndStream_Shimmer3R_v1.01.009` (introducing commit 71d283f, 2026-08-18) |
| InfoMem | configuration setup bytes 4-6 at InfoMem 130-132 | — | `NV_CONFIG_SETUP_BYTE4` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 00425d6, 2015-03-30)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| InfoMem | the Bluetooth PIN byte at InfoMem 231 | — | `NV_BT_SET_PIN` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, `LogAndStream_v0.9.0` (introducing commit 87ff27c, 2017-08-18)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Commands | RESET_BT_ERROR_COUNTS | — | `RESET_BT_ERROR_COUNTS` | shared module: `LogAndStream_Shimmer3R_v1.00.049`, `LogAndStream_Shimmer3_v1.00.015` (introducing commit 039dbbd, 2025-11-27) |
| Commands | SET_DATA_RATE_TEST / DATA_RATE_TEST_RESPONSE | — | `SET_DATA_RATE_TEST` | Shimmer3: `LogAndStream_Shimmer3_BLE_v0.16.002`, `LogAndStream_Shimmer3_v1.00.003` (introducing commit 7e8558f, 2023-12-15)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit da26df24, 2023-12-15) |

Repository history starts: common 2025-01-28; s3 2013-10-16; s3r 2023-10-17.

## Appendix B. Java-only and legacy opcodes

_TODO: opcodes present in the Java driver's registry but not served by LogAndStream — BtStream/SDLog-era commands and Shimmer2r carry-overs — so host maintainers can tell "not implemented here" from "wrong opcode". Source: `driver/ShimmerObject.java`, `bluetooth/ShimmerBluetooth.java`._

## Still unverified / not found in code

- _TODO: populate as the doc pass proceeds._
