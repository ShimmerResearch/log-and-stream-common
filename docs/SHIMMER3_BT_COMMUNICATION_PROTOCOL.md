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

_TODO: `INQUIRY_COMMAND` 0x01 request and the `INQUIRY_RESPONSE` 0x02 payload, which is 8 bytes of fixed fields on S3 and 11 on S3R before the variable channel-ID list. Source: `ShimBt_sendRsp` `case INQUIRY_COMMAND`._

### 7.2 Version information

_TODO: `GET_DEVICE_VERSION_COMMAND` 0x3F (and why 0x24 was retired), `GET_FW_VERSION_COMMAND` 0x2E's 6-byte identifier/major/minor/patch payload, `GET_BT_VERSION_STR_COMMAND`, `GET_UNIQUE_SERIAL_COMMAND` (8 bytes S3 / 12 bytes S3R). Source: `ShimBt_sendRsp`._

### 7.3 Device status

_TODO: `GET_STATUS_COMMAND` 0x72, `GET_DIR_COMMAND` 0x89, `TEST_CONNECTION_COMMAND` 0x96, `TOGGLE_LED_COMMAND` 0x06. Source: `ShimBt_sendRsp`, `ShimBt_processCmd`._

### 7.4 Battery

_TODO: `GET_VBATT_COMMAND` 0x95 and its 3-byte raw ADC + charger-status payload, `GET_CHARGE_STATUS_LED_COMMAND` / `SET_CHARGE_STATUS_LED_COMMAND`. Source: `ShimBt_sendRsp` `case GET_VBATT_COMMAND`, `Battery/shimmer_battery.{h,c}`._

### 7.5 InfoMem

_TODO: `GET_INFOMEM_COMMAND` 0x8E `[len][offLo][offHi]` and `SET_INFOMEM_COMMAND` 0x8C `[len][offLo][offHi][data...]`, the 128-byte cap, the MAC-address overwrite on the segment-C page, and the `ShimConfig_checkAndCorrectConfig` pass that can change what a host just wrote. Cross-reference [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md). Source: `ShimBt_processCmd`._

### 7.6 Calibration dump and per-sensor calibration

_TODO: `GET_CALIB_DUMP_COMMAND` 0x9A / `SET_CALIB_DUMP_COMMAND` 0x98 / `UPD_CALIB_DUMP_COMMAND` 0x9B and the argument order, the six per-sensor 21-byte GET/SET pairs, `GET_ALL_CALIBRATION_COMMAND` 0x2C, `RESET_CALIBRATION_VALUE_COMMAND` 0x5B. Cross-reference [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md). Source: `ShimBt_processCmd`, `ShimBt_replySingleSensorCalibCmd`._

### 7.7 SD logging and trial configuration

_TODO: `SET_TRIAL_CONFIG_COMMAND` / `GET_TRIAL_CONFIG_COMMAND`, `SET_CENTER_COMMAND` / `GET_CENTER_COMMAND`, `SET_SHIMMERNAME_COMMAND`, `SET_EXPID_COMMAND`, `SET_MYID_COMMAND`, `SET_NSHIMMER_COMMAND`, `SET_CONFIGTIME_COMMAND`, `SET_RWC_COMMAND` / `GET_RWC_COMMAND`, `UPD_SDLOG_CFG_COMMAND`. Source: `ShimBt_processCmd`._

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
