# Shimmer3 / Shimmer3R Configuration (InfoMem)

This document defines the byte-level layout of the Shimmer3 and Shimmer3R
**configuration block**, universally called *InfoMem* after the MSP430 information
memory it originally lived in. The same 512-byte image is exchanged over Bluetooth
(`GET_INFOMEM_COMMAND` / `SET_INFOMEM_COMMAND`), persisted in non-volatile memory,
and mirrored into the SD-card file header.

> **Verified against:**
> - **Firmware (authority for bytes):** `log-and-stream-common` @ `c13cbde` —
>   `Configuration/shimmer_config.h` (`NV_*` offset `#define`s, the `gConfigBytes`
>   packed union with its `#if defined(SHIMMER3)` / `#if defined(SHIMMER3R)`
>   variants), `Configuration/shimmer_config.c`
>   (`ShimConfig_setDefaultConfig` defaults, `ShimConfig_checkAndCorrectConfig`
>   validation), `SDCard/shimmer_sd_header.h` (the `SDH_*` mirror offsets),
>   `Comms/shimmer_bt_uart.c` (paging and the per-setting write paths).
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4` —
>   `LogAndStream_Shimmer3/Shimmer_Driver/5xx_HAL/hal_InfoMem.h`;
>   `shimmer3r-firmware` @ `a8f105e5` —
>   `LogAndStream_Shimmer3R/Shimmer_Driver/hal_Infomem.h`.
> - **Host reference implementations:** `Shimmer-Java-Android-API` @ `edc3f7d9`
>   (v0.11.8_beta) — `driver/shimmer2r3/ConfigByteLayoutShimmer3.java`,
>   `driver/ConfigByteLayout.java`; `shimmer-web-sdk` @ `8f78313` —
>   `devices/infomem/layout.ts`.

> **How to read this document.** **S3** = Shimmer3 (MSP430, LogAndStream);
> **S3R** = Shimmer3R (STM32U5). The **Gen** column says which generation a byte
> or bit applies to: `both` = identical on the two platforms, `S3` / `S3R` = the
> field exists only inside that platform's `#if` branch of `gConfigBytes`, and
> where the two platforms give the *same byte* different meanings both rows are
> shown.
>
> Bit numbering follows the firmware struct: `gConfigBytes` is a packed
> little-endian bitfield union, so within a byte the **first-declared bitfield
> occupies the least-significant bits**. A field documented as "bits 3-2" is
> `(byte >> 2) & 0x03`.
>
> This document describes the **LogAndStream** configuration block. BtStream and
> SDLog used overlapping but not identical layouts; those differences are confined
> to the appendix.

**Source references:**

| Layer | File |
|---|---|
| Offset definitions | `Configuration/shimmer_config.h` — `NV_*` |
| Field and bit layout | `Configuration/shimmer_config.h` — `gConfigBytes` |
| Defaults | `Configuration/shimmer_config.c` — `ShimConfig_setDefaultConfig` |
| Validation / correction | `Configuration/shimmer_config.c` — `ShimConfig_checkAndCorrectConfig` |
| SD header mirror | `SDCard/shimmer_sd_header.{h,c}` — `SDH_*`, `ShimSdHead_config2SdHead` |
| Non-volatile backing (S3) | `shimmer3-firmware` `Shimmer_Driver/5xx_HAL/hal_InfoMem.{h,c}` |
| Non-volatile backing (S3R) | `shimmer3r-firmware` `Shimmer_Driver/hal_Infomem.{h,c}` |
| Bluetooth access | `Comms/shimmer_bt_uart.c` — see [SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md) §7.5 |
| Host reference (Java) | `Shimmer-Java-Android-API` — `driver/shimmer2r3/ConfigByteLayoutShimmer3.java` |
| Host reference (TypeScript) | `shimmer-web-sdk` — `devices/infomem/layout.ts` |

---

## 1. Transport and size

_TODO: `STOREDCONFIG_SIZE` 512 / `NV_NUM_RWMEM_BYTES`, the historical 384-byte `NV_TOTAL_NUM_CONFIG_BYTES` split into settings + calibration + SD bytes, the MSP430 segment structure that produced the 128-byte page boundaries, and the 128-byte cap per Bluetooth transfer. Source: `Configuration/shimmer_config.h`, `hal_InfoMem.h`._

## 2. Byte map

_The table in this section is generated mechanically from the firmware `NV_*`
offsets and a walk of the `gConfigBytes` struct, cross-checked against the Java
`idx*` constants and the TypeScript layout resolver. Do not hand-edit it._

_TODO: paste generated InfoMem byte map._

## 3. Sampling rate, buffer size and sensor enables (bytes 0-5)

_TODO: `samplingRateTicks` as a 32768 Hz clock divider rather than a frequency, `bufferSize`, and the three (S3) / five (S3R) sensor-enable bytes with their per-platform bit meanings. Source: `gConfigBytes` idx 0-5, `ShimConfig_getShimmerSamplingFreq`._

## 4. Configuration setup bytes (6-9 and 130-132)

_TODO: the four legacy setup bytes and the three Shimmer3R additions, including the fields that are split across a legacy LSB and a new MSB bit (`gyroRange`, `wrAccelLpMode`, `pressureOversamplingRatio`) and why. Source: `gConfigBytes` idx 6-9 and 130-132, `ShimConfig_gyroRangeGet` / `Set` and siblings._

## 5. ExG register banks (10-29)

_TODO: the two 10-byte ADS1292R register images, their register names, and that they are written verbatim to the chip. Source: `gExgADS1292rRegs`, `NV_EXG_ADS1292R_1_CONFIG1`.._

## 6. Bluetooth baud and derived channels

_TODO: byte 30 `btCommsBaudRate` (stored but no longer applied), and the 8 derived-channel bytes split 31-33 + 118-122 with the algorithm flags they carry. Source: `gConfigBytes` idx 30-33 and 118-122._

## 7. Calibration blocks

_TODO: the four 21-byte kinematic blocks at 34/55/76/97 and the two Shimmer3R blocks at 133/154, and their relationship to the calibration dump. Cross-reference [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md). Source: `gImuConfig`, `NV_LN_ACCEL_CALIBRATION`.._

## 8. SD logging and trial configuration

_TODO: shimmer name, experiment ID, config time, trial ID, shimmer count, the two trial-config bit bytes, BT interval, estimated and maximum experiment length, MAC address, and the SD-config delay flag. Source: `gConfigBytes` idx 187-231._

## 9. Sync node table

_TODO: the centre address plus 21 node addresses of 6 bytes each from byte 256, `NV_NUM_BYTES_SYNC_CENTER_NODE_ADDRS`, and how the node count relates to `numberOfShimmers`. Source: `gConfigBytes` `syncNodeAddr*`, `NV_CENTER` / `NV_NODE0`._

## 10. Firmware validation and correction rules

_TODO: every clamp and coercion `ShimConfig_checkAndCorrectConfig` applies, so a host knows which written values will silently come back different. Source: `Configuration/shimmer_config.c` — `ShimConfig_checkAndCorrectConfig`._

## 11. Defaults

_TODO: the values `ShimConfig_setDefaultConfig` writes, including the default shimmer name / trial ID / config time and what an erased (all-`0xFF`) InfoMem is corrected to. Source: `Configuration/shimmer_config.c` — `ShimConfig_setDefaultConfig`, `ShimConfig_areConfigBytesValid`._

## 12. Layout version gates

_TODO: the host-side version gates that select a layout variant, and the note that for current LogAndStream and for all Shimmer3R every gate in the Java `ConfigByteLayoutShimmer3` constructor evaluates true, so only the newest layout is live. Source: `driver/shimmer2r3/ConfigByteLayoutShimmer3.java` constructor, `driverUtilities/ShimmerVerObject.java`._

## Appendix A. InfoMem ↔ SD-header pairs

_TODO: the mapping from each `NV_*` offset to its `SDH_*` counterpart in the SD-card file header, since the two are written together by `ShimBt_settingChangeCommon` but are not at the same offsets. Source: `SDCard/shimmer_sd_header.h`, `ShimSdHead_config2SdHead`._

## Still unverified / not found in code

- _TODO: populate as the doc pass proceeds._
