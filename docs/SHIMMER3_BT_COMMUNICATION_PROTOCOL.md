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
> which generation *serves* an opcode: `both` = the handler compiles on both
> platforms, `S3` / `S3R` = the handler sits behind a `#if defined(SHIMMER3)` /
> `#if defined(SHIMMER3R)` guard, and `—` = the opcode number is reserved in the
> shared header but no LogAndStream handler exists.
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

### 4.1 Core

_TODO: paste generated core opcode rows._

### 4.2 SD / trial configuration

_TODO: paste generated SD/trial opcode rows._

### 4.3 Calibration

_TODO: paste generated calibration opcode rows._

### 4.4 Shimmer3R-served extensions

_TODO: paste generated Shimmer3R-only opcode rows (SD file transfer, alt accel / alt mag, pressure coefficients)._

### 4.5 Deprecated / no-op

_TODO: paste generated rows for `DEPRECATED_GET_DEVICE_VERSION_COMMAND`, `SET_BT_COMMS_BAUD_RATE`, `RESET_BT_ERROR_COUNTS` on S3R, and the commented-out `UPD_FLASH_COMMAND`._

### 4.6 Not served by LogAndStream

_TODO: paste generated rows for opcodes defined in the shared header but unhandled here (`SHIMMER4_SDK` block, `SET_I2C_BATT_STATUS_*`), plus Java-only constants with no firmware handler._

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

_TODO: paste the generated version-gate table (host-side capability gates and the first firmware tag that satisfies each, or "predates firmware git history")._

## Appendix B. Java-only and legacy opcodes

_TODO: opcodes present in the Java driver's registry but not served by LogAndStream — BtStream/SDLog-era commands and Shimmer2r carry-overs — so host maintainers can tell "not implemented here" from "wrong opcode". Source: `driver/ShimmerObject.java`, `bluetooth/ShimmerBluetooth.java`._

## Still unverified / not found in code

- _TODO: populate as the doc pass proceeds._
