# Shimmer3 / Shimmer3R Configuration (InfoMem)

This document defines the byte-level layout of the Shimmer3 and Shimmer3R
**configuration block**, universally called *InfoMem* after the MSP430 information
memory it originally lived in. The same 512-byte image is exchanged over Bluetooth
(`GET_INFOMEM_COMMAND` / `SET_INFOMEM_COMMAND`), persisted in non-volatile memory,
and mirrored into the SD-card file header.

> **Verified against** — the revisions these byte-level claims were read from.
> A pinned commit is a citation, not a claim of currency: when the firmware
> moves on, this document needs re-checking against it rather than the stamp
> being wrong, and the `file:line` references throughout only resolve because
> the revision is pinned here.
>
> - **Firmware (authority for bytes):** `log-and-stream-common` @ `f3cf73e` —
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

### 1.1 Sizes

| Constant | Value | Meaning |
|---|---:|---|
| `STOREDCONFIG_SIZE` / `NV_NUM_RWMEM_BYTES` | 512 | Size of the `gConfigBytes` union — the in-RAM image |
| `NV_TOTAL_NUM_CONFIG_BYTES` | 384 | Bytes actually carrying configuration |
| `NV_NUM_SETTINGS_BYTES` | 34 | Historical split: sampling, sensors, setup bytes |
| `NV_NUM_CALIBRATION_BYTES` | 84 | Historical split: four 21-byte kinematic blocks |
| `NV_NUM_SD_BYTES` | 37 | Historical split: names, trial config, MAC |
| `NV_NUM_BYTES_SYNC_CENTER_NODE_ADDRS` | 126 | Sync centre + node address table (§9) |

The three historical split constants sum to 155, not 384. They date from a
smaller layout and are no longer a partition of anything — treat 384 as the
figure that matters and the other three as vestigial. Bytes **384-511** are the
`padding` member of the struct: present in RAM so the union is a round 512
bytes, never transferred and never persisted.

### 1.2 The 128-byte page structure

Every boundary in this layout is a multiple of 128 because the original MSP430
information memory is four 128-byte segments:

| Segment | MSP430 address | Config bytes |
|---|---|---|
| D | `0x1800` | 0-127 |
| C | `0x1880` | 128-255 |
| B | `0x1900` | 256-383 |
| A | `0x1980` | 384-511 (padding) |

The constants survive in the shared code as `INFOMEM_SEG_A_ADDR_MSP430` through
`INFOMEM_SEG_D_ADDR_MSP430` with `INFOMEM_OFFSET_MSP430 = 0x1800`, and they are
described in the header as "definitions used within the Shimmer3 comms
protocol" — that is, they are part of the *wire* contract, not just an MSP430
implementation detail.

This is why the Shimmer3R additions could not simply extend byte 132 onwards:
the second page (128-255) was the first free segment, which is why
`NV_ALT_ACCEL_CALIBRATION` is at 133 rather than adjacent to the other
calibration blocks at 34-117, and why the calibration sync code splits into
`ShimCalib_configBytes0To127ToCalibDumpBytes` and
`ShimCalib_configBytes128To255ToCalibDumpBytes`.

> **Two addressing conventions exist for the same bytes.** Some host code
> addresses InfoMem by the absolute MSP430 address (`0x1800 + offset`), and some
> by a flat offset from 0. The Bluetooth commands documented here use **flat
> offsets from 0**. A host that sends `0x1800`-based offsets will be rejected by
> the bounds check rather than silently misplacing data, but the failure mode is
> a NACK with no explanation.

### 1.3 Non-volatile backing

| Platform | Backing | Notes |
|---|---|---|
| Shimmer3 | MSP430 information memory, `0x1800` | Four 128-byte segments, erase granularity one segment |
| Shimmer3R | STM32U5 flash bank 2, `0x083F8000` | `INFOMEM_CONFIG_SIZE = 0x200` (512 B) at the base, followed by `INFOMEM_CALIB_SIZE = 0x400` (1024 B) for the calibration dump and `INFOMEM_TEST_SIZE = 0x200` (512 B) for factory-test data. Flash page size is 8 KB and `INFOMEM_NUM_OF_PAGES` is 4, so a write costs an 8 KB page erase. |

On Shimmer3R the configuration block, the calibration dump and the factory-test
block are three distinct regions of one 32 KB reservation, addressed through the
`INFOMEM_MASK_RAM` / `INFOMEM_MASK_CALIB` masks. Only the first 512 bytes are
the subject of this document.

### 1.4 Bluetooth transfer

`GET_INFOMEM_COMMAND` and `SET_INFOMEM_COMMAND` are **paged, with a hard cap of
128 bytes per transfer** — the same figure as a page, though the cap is a
protocol limit rather than a segment requirement, and a transfer may start at
any offset. A full 384-byte read is therefore three commands minimum.

Writes take effect in RAM immediately and are flushed to non-volatile memory by
the firmware; see
[SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md)
§7.5 for the exact framing, the bounds-check behaviour, and the rule that `SET`
commands are NACKed while the device is sensing.

## 2. Byte map

_The table in this section is generated mechanically from the firmware `NV_*`
offsets and a walk of the `gConfigBytes` struct, cross-checked against the Java
`idx*` constants and the TypeScript layout resolver. Do not hand-edit it._

Bit numbers are the real bitfield positions in the packed little-endian
`gConfigBytes` struct: **bit 0 is the least-significant bit** of the byte, and
bits are listed most-significant first for readability. Where the two
generations give a byte different meanings both are shown.

| Byte | FW field / bits | `NV_` constant | Java `idx` | SDK field | Gen | Notes |
|---|---|---|---|---|---|---|
| 0–1 | `samplingRateTicks` (uint16_t) | `NV_SAMPLING_RATE` | `idxShimmerSamplingRate` | `idxSamplingRate` | both |  |
| 2 | `bufferSize` (uint8_t) | `NV_BUFFER_SIZE` | `idxBufferSize` | `idxBufferSize` | both |  |
| 3 | **S3:** bit 7 `chEnLnAccel`; bit 6 `chEnGyro`; bit 5 `chEnMag`; bit 4 `chEnExg1_24Bit`; bit 3 `chEnExg2_24Bit`; bit 2 `chEnGsr`; bit 1 `chEnExtADC7`; bit 0 `chEnExtADC6`<br>**S3R:** bit 7 `chEnLnAccel`; bit 6 `chEnGyro`; bit 5 `chEnMag`; bit 4 `chEnExg1_24Bit`; bit 3 `chEnExg2_24Bit`; bit 2 `chEnGsr`; bit 1 `chEnExtADC0`; bit 0 `chEnExtADC1` | `NV_SENSORS0` | `idxSensors0` | `idxSensors0` | differs |  |
| 4 | **S3:** bit 7 `chEnBridgeAmp`; bit 6 `unusedIdx4Bit6`; bit 5 `chEnVBattery`; bit 4 `chEnWrAccel`; bit 3 `chEnExtADC15`; bit 2 `chEnIntADC1`; bit 1 `chEnIntADC12`; bit 0 `chEnIntADC13`<br>**S3R:** bit 7 `chEnBridgeAmp`; bit 6 `chEnMicrophone`; bit 5 `chEnVBattery`; bit 4 `chEnWrAccel`; bit 3 `chEnExtADC2`; bit 2 `chEnIntADC3`; bit 1 `chEnIntADC0`; bit 0 `chEnIntADC1` | `NV_SENSORS1` | `idxSensors1` | `idxSensors1` | differs |  |
| 5 | **S3:** bit 7 `chEnIntADC14`; bit 6 `chEnAltAccel`; bit 5 `chEnAltMag`; bit 4 `chEnExg1_16Bit`; bit 3 `chEnExg2_16Bit`; bit 2 `chEnPressureAndTemperature`; bit 1 `chEnMPU9x50temp`; bit 0 `unusedIdx5Bit0`<br>**S3R:** bit 7 `chEnIntADC2`; bit 6 `chEnAltAccel`; bit 5 `chEnAltMag`; bit 4 `chEnExg1_16Bit`; bit 3 `chEnExg2_16Bit`; bit 2 `chEnPressureAndTemperature`; bit 1 `unusedIdx5Bit1`; bit 0 `unusedIdx5Bit0` | `NV_SENSORS2` | `idxSensors2` | `idxSensors2` | differs |  |
| 6 | bits 7-4 `wrAccelRate`; bits 3-2 `wrAccelRange`; bit 1 `wrAccelLpModeLsb`; bit 0 `wrAccelHrMode` | `NV_CONFIG_SETUP_BYTE0` | `idxConfigSetupByte0` | `idxConfigSetupByte0` | both |  |
| 7 | `gyroRate` (uint8_t) | `NV_CONFIG_SETUP_BYTE1` | `idxConfigSetupByte1` |  | both |  |
| 8 | **S3:** bits 7-5 `magRange`; bits 4-2 `magRate`; bits 1-0 `gyroRangeLsb`<br>**S3R:** bits 7-5 `altMagRange`; bits 4-2 `magRate`; bits 1-0 `gyroRangeLsb` | `NV_CONFIG_SETUP_BYTE2` | `idxConfigSetupByte2` |  | differs |  |
| 9 | **S3:** bits 7-6 `altAccelRange`; bits 5-4 `pressureOversamplingRatioLsb`; bits 3-1 `gsrRange`; bit 0 `expansionBoardPower`<br>**S3R:** bits 7-6 `lnAccelRange`; bits 5-4 `pressureOversamplingRatioLsb`; bits 3-1 `gsrRange`; bit 0 `expansionBoardPower` | `NV_CONFIG_SETUP_BYTE3` | `idxConfigSetupByte3` | `idxConfigSetupByte3` | differs |  |
| 10–19 | `exgADS1292rRegsCh1` (gExgADS1292rRegs) | `NV_EXG_ADS1292R_1_CONFIG1` | `idxEXGADS1292RChip1Config1` | `idxExg1` | both |  |
| 20–29 | `exgADS1292rRegsCh2` (gExgADS1292rRegs) | `NV_EXG_ADS1292R_2_CONFIG1` | `idxEXGADS1292RChip2Config1` | `idxExg2` | both |  |
| 30 | `btCommsBaudRate` (uint8_t) | `NV_BT_COMMS_BAUD_RATE` | `idxBtCommBaudRate` | `idxBtCommBaudRate` | both |  |
| 31 | bit 7 `chEnPpgToHr2`; bit 6 `chEnPpgToHr1`; bit 5 `chEnPpgtoHr`; bit 4 `chEnPpg2`; bit 3 `chEnPpg1`; bit 2 `chEnPpg`; bit 1 `chEnSkinTemp`; bit 0 `chEnResAmp` | `NV_DERIVED_CHANNELS_0` | `idxDerivedSensors0` | `idxDerivedSensors0` | both |  |
| 32 | bit 7 `chEnEcg2HrChp1Ch1`; bit 6 `chEnEcg2HrChp1Ch2`; bit 5 `chEnEcg2HrChp2Ch1`; bit 4 `chEnEcg2HrChp2Ch2`; bit 3 `chEnHrVTime`; bit 2 `chEnHrVFreq`; bit 1 `chEnActivity`; bit 0 `chEnGsrMetricsGeneral` | `NV_DERIVED_CHANNELS_1` | `idxDerivedSensors1` | `idxDerivedSensors1` | both |  |
| 33 | bit 7 `chEnSixDofLnEuler`; bit 6 `chEnSixDofLnQuat`; bit 5 `chEnNineDofLnEuler`; bit 4 `chEnNineDofLnQuat`; bit 3 `chEnSixDofWrEuler`; bit 2 `chEnSixDofWrQuat`; bit 1 `chEnNineDofWrEuler`; bit 0 `chEnNineDofWrQuat` | `NV_DERIVED_CHANNELS_2` | `idxDerivedSensors2` | `idxDerivedSensors2` | both |  |
| 34–54 | `lnAccelCalib` (gImuConfig) | `NV_LN_ACCEL_CALIBRATION` | `idxAnalogAccelCalibration` |  | both |  |
| 55–75 | `gyroCalib` (gImuConfig) | `NV_GYRO_CALIBRATION` | `idxMPU9150GyroCalibration` |  | both |  |
| 76–96 | `magCalib` (gImuConfig) | `NV_MAG_CALIBRATION` | `idxLSM303DLHCMagCalibration` |  | both |  |
| 97–117 | `wrAccelCalib` (gImuConfig) | `NV_WR_ACCEL_CALIBRATION` | `idxLSM303DLHCAccelCalibration` |  | both |  |
| 118 | bit 7 `unusedByte118Bit7`; bit 6 `unusedByte118Bit6`; bit 5 `chEnGyroOnTheFlyCalib`; bit 4 `chEnGaitModule`; bit 3 `chEnGsrMetricsTrendPeak`; bit 2 `chEnGsrBaseline`; bit 1 `chEnEmgProcessingCh1`; bit 0 `chEnEmgProcessingCh2` | `NV_DERIVED_CHANNELS_3` | `idxDerivedSensors3` | `idxDerivedSensors3` | both |  |
| 119 | `derivedChannels4` (uint8_t) | `NV_DERIVED_CHANNELS_4` | `idxDerivedSensors4` | `idxDerivedSensors4` | both |  |
| 120 | `derivedChannels5` (uint8_t) | `NV_DERIVED_CHANNELS_5` | `idxDerivedSensors5` | `idxDerivedSensors5` | both |  |
| 121 | `derivedChannels6` (uint8_t) | `NV_DERIVED_CHANNELS_6` | `idxDerivedSensors6` | `idxDerivedSensors6` | both |  |
| 122 | `derivedChannels7` (uint8_t) | `NV_DERIVED_CHANNELS_7` | `idxDerivedSensors7` | `idxDerivedSensors7` | both |  |
| 123 | `unusedIdx123` (uint8_t) |  |  |  | both | reserved |
| 124 | `unusedIdx124` (uint8_t) |  |  |  | both | reserved |
| 125 | `unusedIdx125` (uint8_t) |  |  |  | both | reserved |
| 126 | `unusedIdx126` (uint8_t) |  |  |  | both | reserved |
| 127 | `unusedIdx127` (uint8_t) |  |  |  | both | reserved |
| 128 | `unusedIdx128` (uint8_t) | `NV_SENSORS3` | `idxSensors3` | `idxSensors3` | both |  |
| 129 | `unusedIdx129` (uint8_t) | `NV_SENSORS4` | `idxSensors4` | `idxSensors4` | both |  |
| 130 | bits 7-6 `altAccelRate`; bit 5 `unusedByte130Bit6`; bit 4 `unusedByte130Bit5`; bit 3 `unusedByte130Bit4`; bit 2 `gyroRangeMsb`; bit 1 `wrAccelLpModeMsb`; bit 0 `pressureOversamplingRatioMsb` | `NV_CONFIG_SETUP_BYTE4` | `idxConfigSetupByte4` |  | both |  |
| 131 | bit 7 `unusedByte131Bit7`; bit 6 `unusedByte131Bit6`; bits 5-0 `altMagRate` | `NV_CONFIG_SETUP_BYTE5` | `idxConfigSetupByte5` |  | both |  |
| 132 | `unusedIdx132` (uint8_t) | `NV_CONFIG_SETUP_BYTE6` | `idxConfigSetupByte6` |  | both |  |
| 133–153 | `altAccelCalib` (gImuConfig) | `NV_ALT_ACCEL_CALIBRATION` | `idxADXL371AltAccelCalibration`, `idxMPLAccelCalibration` |  | both |  |
| 154–174 | `altMagCalib` (gImuConfig) | `NV_ALT_MAG_CALIBRATION` | `idxLIS3MDLAltMagCalibration`, `idxMPLMagCalibration` |  | both |  |
| 175–186 | `unusedIdx175To186` (uint8_t[12]) | `NV_MPL_GYRO_CALIBRATION` | `idxMPLGyroCalibration` |  | both |  |
| 187–198 | `shimmerName` (char[12]) | `NV_SD_SHIMMER_NAME` | `idxSDShimmerName` | `idxSDShimmerName` | both |  |
| 199–210 | `expIdName` (char[12]) | `NV_SD_EXP_ID_NAME` | `idxSDEXPIDName` | `idxSDEXPIDName` | both |  |
| 211 | `configTime0` (uint8_t) | `NV_SD_CONFIG_TIME` | `idxSDConfigTime0` | `idxSDConfigTime0` | both |  |
| 212 | `configTime1` (uint8_t) |  | `idxSDConfigTime1` |  | both |  |
| 213 | `configTime2` (uint8_t) |  | `idxSDConfigTime2` |  | both |  |
| 214 | `configTime3` (uint8_t) |  | `idxSDConfigTime3` |  | both |  |
| 215 | `myTrialID` (uint8_t) | `NV_SD_MYTRIAL_ID` | `idxSDMyTrialID` | `idxSDMyTrialID` | both |  |
| 216 | `numberOfShimmers` (uint8_t) | `NV_SD_NSHIMMER` | `idxSDNumOfShimmers` | `idxSDNumOfShimmers` | both |  |
| 217 | bit 7 `rtcSetByBt`; bit 6 `btPinSetup`; bit 5 `userButtonEnable`; bit 4 `rtcErrorEnable`; bit 3 `bluetoothDisable`; bit 2 `syncEnable`; bit 1 `masterEnable`; bit 0 `sdErrorEnable` | `NV_SD_TRIAL_CONFIG0` | `idxSDExperimentConfig0` | `idxSDExperimentConfig0` | both |  |
| 218 | bit 7 `singleTouchStart`; bit 6 `unusedIdx218Bit6`; bit 5 `unusedIdx218Bit5`; bit 4 `tcxo`; bit 3 `unusedIdx218Bit3`; bit 2 `unusedIdx218Bit2`; bit 1 `unusedIdx218Bit1`; bit 0 `lowBatteryAutoStop` | `NV_SD_TRIAL_CONFIG1` | `idxSDExperimentConfig1` | `idxSDExperimentConfig1` | both |  |
| 219 | `btIntervalSecs` (uint8_t) | `NV_SD_BT_INTERVAL` | `idxSDBTInterval` | `idxSDBTInterval` | both |  |
| 220 | `experimentLengthEstimatedInSecMsb` (uint8_t) | `NV_EST_EXP_LEN_MSB` | `idxEstimatedExpLengthMsb` | `idxEstimatedExpLengthMsb` | both |  |
| 221 | `experimentLengthEstimatedInSecLsb` (uint8_t) | `NV_EST_EXP_LEN_LSB` | `idxEstimatedExpLengthLsb` | `idxEstimatedExpLengthLsb` | both |  |
| 222 | `experimentLengthMaxInMinutesMsb` (uint8_t) | `NV_MAX_EXP_LEN_MSB` | `idxMaxExpLengthMsb` | `idxMaxExpLengthMsb` | both |  |
| 223 | `experimentLengthMaxInMinutesLsb` (uint8_t) | `NV_MAX_EXP_LEN_LSB` | `idxMaxExpLengthLsb` | `idxMaxExpLengthLsb` | both |  |
| 224–229 | `macAddr` (uint8_t[6]) | `NV_MAC_ADDRESS` | `idxMacAddress` | `idxMacAddress` | both |  |
| 230 | bit 7 `unusedIdx230Bit7`; bit 6 `unusedIdx230Bit6`; bit 5 `unusedIdx230Bit5`; bit 4 `unusedIdx230Bit4`; bit 3 `unusedIdx230Bit3`; bit 2 `unusedIdx230Bit2`; bit 1 `unusedIdx230Bit1`; bit 0 `flagWriteCfgToSd` | `NV_SD_CONFIG_DELAY_FLAG` | `idxSDConfigDelayFlag` | `idxSDConfigDelayFlag` | both |  |
| 231 | `btSetPin` (uint8_t) | `NV_BT_SET_PIN` | `idxBtFactoryReset` | `idxBtFactoryReset` | both |  |
| 232 | `unusedIdx232` (uint8_t) |  |  |  | both | reserved |
| 233 | `unusedIdx233` (uint8_t) |  |  |  | both | reserved |
| 234 | `unusedIdx234` (uint8_t) |  |  |  | both | reserved |
| 235 | `unusedIdx235` (uint8_t) |  |  |  | both | reserved |
| 236 | `unusedIdx236` (uint8_t) |  |  |  | both | reserved |
| 237 | `unusedIdx237` (uint8_t) |  |  |  | both | reserved |
| 238 | `unusedIdx238` (uint8_t) |  |  |  | both | reserved |
| 239 | `unusedIdx239` (uint8_t) |  |  |  | both | reserved |
| 240 | `unusedIdx240` (uint8_t) |  |  |  | both | reserved |
| 241 | `unusedIdx241` (uint8_t) |  |  |  | both | reserved |
| 242 | `unusedIdx242` (uint8_t) |  |  |  | both | reserved |
| 243 | `unusedIdx243` (uint8_t) |  |  |  | both | reserved |
| 244 | `unusedIdx244` (uint8_t) |  |  |  | both | reserved |
| 245 | `unusedIdx245` (uint8_t) |  |  |  | both | reserved |
| 246 | `unusedIdx246` (uint8_t) |  |  |  | both | reserved |
| 247 | `unusedIdx247` (uint8_t) |  |  |  | both | reserved |
| 248 | `unusedIdx248` (uint8_t) |  |  |  | both | reserved |
| 249 | `unusedIdx249` (uint8_t) |  |  |  | both | reserved |
| 250 | `unusedIdx250` (uint8_t) |  |  |  | both | reserved |
| 251 | `unusedIdx251` (uint8_t) |  |  |  | both | reserved |
| 252 | `unusedIdx252` (uint8_t) |  |  |  | both | reserved |
| 253 | `unusedIdx253` (uint8_t) |  |  |  | both | reserved |
| 254 | `unusedIdx254` (uint8_t) |  |  |  | both | reserved |
| 255 | `unusedIdx255` (uint8_t) |  |  |  | both | reserved |
| 256–261 | `syncNodeAddr1` (uint8_t[6]) | `NV_CENTER` | `idxNode0` | `idxNode0` | both | sync node table |
| 262–267 | `syncNodeAddr2` (uint8_t[6]) | `NV_NODE0` |  |  | both | sync node table |
| 268–273 | `syncNodeAddr3` (uint8_t[6]) |  |  |  | both | sync node table |
| 274–279 | `syncNodeAddr4` (uint8_t[6]) |  |  |  | both | sync node table |
| 280–285 | `syncNodeAddr5` (uint8_t[6]) |  |  |  | both | sync node table |
| 286–291 | `syncNodeAddr6` (uint8_t[6]) |  |  |  | both | sync node table |
| 292–297 | `syncNodeAddr7` (uint8_t[6]) |  |  |  | both | sync node table |
| 298–303 | `syncNodeAddr8` (uint8_t[6]) |  |  |  | both | sync node table |
| 304–309 | `syncNodeAddr9` (uint8_t[6]) |  |  |  | both | sync node table |
| 310–315 | `syncNodeAddr` (uint8_t[6]) |  |  |  | both | sync node table |
| 316–321 | `syncNodeAddr11` (uint8_t[6]) |  |  |  | both | sync node table |
| 322–327 | `syncNodeAddr12` (uint8_t[6]) |  |  |  | both | sync node table |
| 328–333 | `syncNodeAddr13` (uint8_t[6]) |  |  |  | both | sync node table |
| 334–339 | `syncNodeAddr14` (uint8_t[6]) |  |  |  | both | sync node table |
| 340–345 | `syncNodeAddr15` (uint8_t[6]) |  |  |  | both | sync node table |
| 346–351 | `syncNodeAddr16` (uint8_t[6]) |  |  |  | both | sync node table |
| 352–357 | `syncNodeAddr17` (uint8_t[6]) |  |  |  | both | sync node table |
| 358–363 | `syncNodeAddr18` (uint8_t[6]) |  |  |  | both | sync node table |
| 364–369 | `syncNodeAddr19` (uint8_t[6]) |  |  |  | both | sync node table |
| 370–375 | `syncNodeAddr20` (uint8_t[6]) |  |  |  | both | sync node table |
| 376–381 | `syncNodeAddr21` (uint8_t[6]) |  |  |  | both | sync node table |
| 382 | `unusedIdx382` (uint8_t) |  |  |  | both | reserved |
| 383 | `unusedIdx383` (uint8_t) |  |  |  | both | reserved |
| 384–511 | `padding` (uint8_t[128]) |  |  |  | both | struct padding, not transferred |

## 3. Sampling rate, buffer size and sensor enables (bytes 0-5)

### 3.1 Sampling rate (bytes 0-1)

`samplingRateTicks` is a **clock divider, not a frequency**. The stored `uint16`
is the number of 32768 Hz ticks between samples:

```
frequency_Hz = 32768.0 / samplingRateTicks
samplingRateTicks = round(32768.0 / frequency_Hz)
```

`ShimConfig_getShimmerSamplingFreq` performs the first; `ShimConfig_freqDiv`
performs the second, via `samplingClockFreqGet()`.

Two consequences for hosts:

- **Not every frequency is representable.** The default of 51.2 Hz stores 640
  ticks, which is exactly 51.2 Hz. A request for 100 Hz stores 328 ticks and
  yields 99.902 Hz. Round-tripping a requested rate through the device will
  usually return a slightly different number.
- **The field is little-endian**, unlike most multi-byte fields in this layout
  (see §8 for the big-endian exceptions).

### 3.2 Buffer size (byte 2)

`bufferSize` is the number of samples the firmware accumulates before emitting
a Bluetooth packet. The default is **1** — one sample per packet. Values above 1
reduce radio overhead at the cost of latency.

### 3.3 Sensor enable bitmaps (bytes 3-5, plus 128-129)

Three bytes on Shimmer3, and the same three plus two nominally reserved bytes on
Shimmer3R. Every bit is one channel; the full per-bit meaning is in the §2 byte
map, which shows both platforms where they differ.

> **The bitmap is LSB-first, and it is the same order in `SET_SENSORS` and in
> InfoMem.** `idxSensors0` holds bits 0-7, `idxSensors1` bits 8-15, and so on.
> Neither is big-endian, and neither is reversed relative to the other. Getting
> this backwards makes an InfoMem parse and an inquiry response disagree about
> which sensors are enabled — a failure that looks like a firmware bug and is
> not.

The bits are **not independent**. Several combinations are illegal because the
channels share an ADC input, and the firmware silently disables one side rather
than rejecting the write. §10 lists every such rule.

> **Do not derive channel order from this bitmap.** The order in which channels
> appear in a data packet is set by the inquiry response, not by bit position,
> and the two differ. Shimmer3 emits temperature before pressure while Shimmer3R
> emits pressure before temperature, to take the clearest example. See
> [SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §4.

Bytes 128 (`NV_SENSORS3`) and 129 (`NV_SENSORS4`) have `NV_` constants and
Java/SDK index names but are declared `unusedIdx128` / `unusedIdx129` in the
current firmware struct on both platforms. They are copied to the SD header as
`SDH_SENSORS3` / `SDH_SENSORS4`. Treat them as reserved-but-transferred.

## 4. Configuration setup bytes (6-9 and 130-132)

Four setup bytes date from Shimmer3; three more were added for Shimmer3R at
130-132. The complete bit assignment is in the §2 byte map. This section covers
the part that catches people out.

### 4.1 Fields split across a legacy LSB and a new MSB bit

Shimmer3R needed more range codes than the original bit allocations could hold,
and the legacy bit positions could not move without breaking every existing
host. Three fields were therefore extended by borrowing a bit in byte 130,
leaving the low bits where they were:

| Field | Low bits | High bit | Combined getter |
|---|---|---|---|
| `gyroRange` | byte 8, bits 1-0 | byte 130, bit 2 | `ShimConfig_gyroRangeGet` |
| `wrAccelLpMode` | byte 6, bit 1 | byte 130, bit 1 | `ShimConfig_wrAccelLpModeGet` |
| `pressureOversamplingRatio` | byte 9, bits 5-4 | byte 130, bit 0 | `ShimConfig_configBytePressureOversamplingRatioGet` |

Each getter reassembles the value as `(msb << n) | lsb`. On Shimmer3 the MSB
bits do not exist and the getters return the LSB field alone.

> **A host that reads only the legacy bits will silently mis-report every
> Shimmer3R value above the legacy maximum.** A gyro range of ±4000 dps
> (code 5) reads back as code 1 (±250 dps) if byte 130 bit 2 is ignored. There
> is no error; the number is just wrong by a factor of sixteen.

> **`wrAccelLpModeMsb` is firmware-only.** The Java driver has no field for it.
> The Shimmer3R LIS2DW12 mode is actually three bits —
> `ShimConfig_wrAccelModeSet` packs `wrAccelHrMode` (byte 6 bit 0) above the
> two-bit low-power mode — so the complete mode is
> `(wrAccelHrMode << 2) | (wrAccelLpModeMsb << 1) | wrAccelLpModeLsb`.

### 4.2 Fields that changed meaning between generations

Two bytes carry different fields on the two platforms at the same bit
positions:

| Byte | Bits | Shimmer3 | Shimmer3R |
|---:|---|---|---|
| 8 | 7-5 | `magRange` (LSM303) | `altMagRange` (LIS3MDL) |
| 9 | 7-6 | `altAccelRange` (MPU9x50) | `lnAccelRange` (LSM6DSV) |

Byte 131 bits 5-0 hold `altMagRate` and exist only on Shimmer3R; the setter
forces the value to 0 on Shimmer3. Byte 132 (`NV_CONFIG_SETUP_BYTE6`) is
declared, transferred and mirrored to the SD header, but no firmware field uses
it on either platform.

> **The alt-mag rate is in byte 131, not byte 130.** The Java driver declares
> the shift under a comment naming a different byte. In byte 130 a 6-bit mask
> would collide with `altAccelRate` (bits 7-6) and `gyroRangeMsb` (bit 2).

> **`magRate` is not a composite field.** Only the three fields in §4.1 are
> split. A `bitShiftLIS2MDLMagRateMSB` exists in the Java source but is
> commented out in both parse and generate, and there is no corresponding
> firmware field.

### 4.3 Clamping in the setters

Every setter clamps out-of-range input **to a default, not to the nearest legal
value**:

| Setter | Out-of-range behaviour |
|---|---|
| `ShimConfig_gyroRangeSet` | S3: falls back to ±500 dps. S3R: falls back to ±500 dps |
| `ShimConfig_gyroRateSet` | S3R: falls back to the 60 Hz ODR |
| `ShimConfig_wrAccelLpModeSet` | S3: coerced to 0 or 1. S3R: falls back to 0 |
| `ShimConfig_configBytePressureOversamplingRatioSet` | Falls back to no oversampling / OSS 1 |
| `ShimConfig_configByteMagRateSet` | Falls back to 75 Hz (S3) or 100 Hz (S3R) |
| `ShimConfig_configByteAltMagRateSet` | Falls back to 80 Hz (S3R); forced to 0 on S3 |

So writing an invalid range does not produce an error and does not produce a
neighbouring value — it produces the factory default. A host must read back
after writing if it needs to know what actually took effect.

The pressure oversampling maximum is **part-dependent on Shimmer3R**: a BMP581
accepts up to 128x while a BMP390 stops at 32x, and the setter consults
`isBmp581InUse()` at write time.

## 5. ExG register banks (10-29)

Two 10-byte images of the TI ADS1292R register file, one per chip:

| Bytes | Chip | `NV_` base |
|---|---|---|
| 10-19 | ExG chip 1 | `NV_EXG_ADS1292R_1_CONFIG1` |
| 20-29 | ExG chip 2 | `NV_EXG_ADS1292R_2_CONFIG1` |

Each bank is the `gExgADS1292rRegs` struct, in register order:

| Offset in bank | Register |
|---:|---|
| 0 | `config1` |
| 1 | `config2` |
| 2 | `loff` |
| 3 | `ch1set` |
| 4 | `ch2set` |
| 5 | `rldSens` |
| 6 | `loffSens` |
| 7 | `loffStat` |
| 8 | `resp1` |
| 9 | `resp2` |

These bytes are **written verbatim to the chip** — the firmware does not
interpret them beyond the one correction below. Their meaning is the ADS1292R
datasheet's, not Shimmer's, and a host configuring ExG needs that datasheet.

> **One byte is silently rewritten.** On boards where the ADS1292R clock lines
> are tied (`ShimBrd_areADS1292RClockLinesTied()`),
> `ShimConfig_checkAndCorrectConfig` forces bit 3 of chip 1's `config2` to 1 and
> reports a correction. A host that writes that bit as 0 on such a board will
> read back a 1.

The firmware ships two canned register sets, applied by
`ShimConfig_setExgConfigForEcg` (the default) and
`ShimConfig_setExgConfigForTestSignal`. The ECG default is
`config1 = 0x02`, `config2 = 0x80`, `loff = 0x10`, `resp2 = 0x02`, all other
registers `0x00`, identically on both chips.

## 6. Bluetooth baud and derived channels

### 6.1 Byte 30 — `btCommsBaudRate`

Stored, transferred, mirrored to the SD header at `SDH_BT_COMMS_BAUD_RATE`, and
**never applied**. `SET_BT_COMMS_BAUD_RATE` is NACKed and its implementation is
commented out; the field is only defaulted, by `getDefaultBaudForBtVersion()`
when the stored value is `0xFF`.

The byte remains writable through `SET_INFOMEM`, so a host can change what is
stored — it will simply have no effect on the link.

> Until recently the command ACKed without doing anything, telling hosts a baud
> change had taken effect when nothing had. It now NACKs. A host that treats a
> NACK here as a fatal error will regress against current firmware; treat it as
> "not supported".

### 6.2 Derived channels — bytes 31-33 and 118-122

Eight bytes of flags for host-side derived signals, split across two
non-contiguous runs because the layout ran out of room in the first block:

| Bytes | `NV_` constants |
|---|---|
| 31-33 | `NV_DERIVED_CHANNELS_0` .. `_2` |
| 118-122 | `NV_DERIVED_CHANNELS_3` .. `_7` |

The per-bit meanings are in the §2 byte map. They cover PPG-to-heart-rate,
ECG-to-heart-rate per chip and channel, heart-rate variability in time and
frequency domains, activity, GSR metrics, EMG processing, gait, gyro
on-the-fly calibration, and the 6-DoF/9-DoF orientation quaternion and Euler
outputs for both the low-noise and wide-range accelerometers.

**The firmware does not compute any of these.** They are a configuration
record: the device stores which derived signals the host application should
produce, so that the setting survives in the SD header and travels with a
trial. Bytes 119-122 have no bitfield names at all in the firmware struct —
they are plain `uint8_t` passthrough.

Two flags in byte 31 do have a firmware consequence, indirectly:
`chEnSkinTemp` and `chEnResAmp` force an internal ADC channel on (§10).

## 7. Calibration blocks

Six 21-byte kinematic calibration blocks:

| Bytes | Field | `NV_` constant | Gen |
|---|---|---|:--:|
| 34-54 | `lnAccelCalib` | `NV_LN_ACCEL_CALIBRATION` | both |
| 55-75 | `gyroCalib` | `NV_GYRO_CALIBRATION` | both |
| 76-96 | `magCalib` | `NV_MAG_CALIBRATION` | both |
| 97-117 | `wrAccelCalib` | `NV_WR_ACCEL_CALIBRATION` | both |
| 133-153 | `altAccelCalib` | `NV_ALT_ACCEL_CALIBRATION` | S3R |
| 154-174 | `altMagCalib` | `NV_ALT_MAG_CALIBRATION` | S3R |

Bytes 175-186 are `NV_MPL_GYRO_CALIBRATION`, a 12-byte legacy MPL block, now
`unusedIdx175To186`.

The internal layout of a 21-byte block — three big-endian `int16` biases, three
big-endian `int16` sensitivities, nine `int8` alignment values scaled by 100 —
and the relationship between these blocks and the self-describing calibration
dump are specified in [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) §3
and §8.

Three things matter when writing configuration bytes directly:

1. **A block of all `0xFF` means "no calibration".**
   `ShimConfig_createBlankConfigBytes` deliberately fills all six with `0xFF`
   rather than zero, because zero is a legal-looking but useless calibration.
2. **These blocks are overwritten from the calibration dump.** Any path that
   calls `ShimCalib_calibDumpToConfigBytesAndSdHeaderAll` — including the
   completion of a `SET_CALIB_DUMP` — replaces them. Writing a calibration block
   through `SET_INFOMEM` and then completing a calibration-dump upload loses the
   former.
3. **The block written is the one for the currently selected range.** Changing
   a sensor's range swaps in a different set from the dump.

## 8. SD logging and trial configuration

| Bytes | Field | Notes |
|---|---|---|
| 187-198 | `shimmerName` | 12 chars, not NUL-terminated. Default `Shimmer_XXXX` with the last four characters replaced by the MAC suffix |
| 199-210 | `expIdName` | 12 chars, not NUL-terminated. Default `DefaultTrial` |
| 211-214 | `configTime0..3` | **Big-endian** `uint32`, seconds. Default 0 |
| 215 | `myTrialID` | Default 0 |
| 216 | `numberOfShimmers` | Default 0 |
| 217 | `NV_SD_TRIAL_CONFIG0` | Bit flags, see below |
| 218 | `NV_SD_TRIAL_CONFIG1` | Bit flags, see below |
| 219 | `btIntervalSecs` | Sync broadcast interval. Default 54 |
| 220-221 | `experimentLengthEstimatedInSec` | **Big-endian** (MSB at 220) |
| 222-223 | `experimentLengthMaxInMinutes` | **Big-endian** (MSB at 222). 0 = auto-stop disabled |
| 224-229 | `macAddr` | 6 bytes |
| 230 | `flagWriteCfgToSd` | Bit 0 only |
| 231 | `btSetPin` | Legacy |

> **Three multi-byte fields here are big-endian, while the sampling rate at
> bytes 0-1 is little-endian.** `configTime`, `experimentLengthEstimatedInSec`
> and `experimentLengthMaxInMinutes` all store most-significant byte first
> (`ShimConfig_configTimeSet` shifts down from 24). There is no rule to infer;
> the endianness is per-field.

### 8.1 Trial config byte 217

| Bit | Field | Meaning |
|---:|---|---|
| 7 | `rtcSetByBt` | The real-world clock was set over Bluetooth this session |
| 6 | `btPinSetup` | Legacy. Always forced to 0 (§10) |
| 5 | `userButtonEnable` | Button starts/stops logging. Default 1 |
| 4 | `rtcErrorEnable` | Signal an unset clock through the LEDs |
| 3 | `bluetoothDisable` | Default 0 |
| 2 | `syncEnable` | Multi-device SD sync |
| 1 | `masterEnable` | This device is the sync centre |
| 0 | `sdErrorEnable` | Signal SD errors through the LEDs. Default 1 |

> **`masterEnable` is byte 217, not byte 130.** `SET_CENTER` used to flush byte
> 130 after changing it, so the new value lived in RAM but was never persisted:
> a read-back succeeded and a reboot lost the setting. Fixed, but a host that
> works around the old behaviour should stop.

### 8.2 Trial config byte 218

| Bit | Field | Meaning |
|---:|---|---|
| 7 | `singleTouchStart` | One button press starts a synchronised trial |
| 4 | `tcxo` | Use the temperature-compensated oscillator |
| 0 | `lowBatteryAutoStop` | Stop logging on low battery |

Bits 6, 5, 3, 2 and 1 are unused. Both `tcxo` and `singleTouchStart` are forced
off at validation on hardware that does not support them (§10).

### 8.3 Defaults for the string fields

`ShimConfig_parseShimmerNameFromConfigBytes` scans for printable characters and,
finding none at index 0, resets the name to the default. A name is therefore
never empty; an all-`0xFF` field yields `Shimmer_<mac4>`.

## 9. Sync node table

Bytes **256-381**, `NV_NUM_BYTES_SYNC_CENTER_NODE_ADDRS = 126` bytes: twenty-one
six-byte Bluetooth addresses.

| Bytes | Field | `NV_` constant |
|---|---|---|
| 256-261 | `syncNodeAddr1` — the **centre** address | `NV_CENTER` |
| 262-267 | `syncNodeAddr2` — first node | `NV_NODE0` |
| 268-381 | `syncNodeAddr3` .. `syncNodeAddr21` | — |

`NV_CENTER` is `(128 + 128 + 0)` and `NV_NODE0` is `(128 + 128 + 6)`, making the
page arithmetic explicit: the table starts at the third 128-byte page.

The naming is off by one against its own semantics — the first entry is the
centre, so `syncNodeAddr2` is node 0 — and the tenth entry is named
`syncNodeAddr` with no numeric suffix, an apparent typo in the struct. Address
by offset, not by field name.

`numberOfShimmers` (byte 216) is the count the trial expects; it is not
enforced against the number of non-`0xFF` entries in this table. An unused slot
is all-`0xFF`, which is what `ShimConfig_createBlankConfigBytes` writes across
the whole 128-byte page.

Bytes 382-383 are reserved and complete the page.

## 10. Firmware validation and correction rules

`ShimConfig_checkAndCorrectConfig` runs on load, after
`ShimConfig_setDefaultConfig`, and whenever configuration changes. It returns a
flag saying whether it changed anything; when it did, the firmware writes the
corrected image back to non-volatile memory.

**Every rule below is silent.** A host that writes a rejected combination gets
an ACK and different bytes on read-back.

### 10.1 Mutually exclusive channels

| Rule | Effect | Reason |
|---|---|---|
| GSR + internal ADC | S3: clears `chEnIntADC1`. S3R: clears `chEnIntADC3` | Shared ADC input |
| Bridge amp + internal ADCs | S3: clears `chEnIntADC13` and `chEnIntADC14`. S3R: clears `chEnIntADC1` and `chEnIntADC2` | Shared inputs |
| ExG 24-bit + 16-bit, same chip | Clears the 16-bit flag | One chip, one width |
| Any ExG + internal ADCs | S3: clears `chEnIntADC1` and `chEnIntADC14`. S3R: clears `chEnIntADC3` and `chEnIntADC2` | Shared inputs |

In every case **the ADC channel loses**, except the ExG width rule where 24-bit
wins over 16-bit.

### 10.2 Forced-on channels

`chEnSkinTemp` or `chEnResAmp` set forces `chEnIntADC1` (S3) /
`chEnIntADC3` (S3R) **on**. This is the one rule that enables rather than
disables, and it does not set the corrected flag, so it is applied without the
image being written back on that account alone.

### 10.3 Value clamps

| Field | Rule |
|---|---|
| `gsrRange` | Greater than 4 becomes `GSR_AUTORANGE` |
| `btIntervalSecs` | If `syncEnable`, anything below 54 (`SYNC_INT_C`) becomes 54 |
| `magRange` (S3) | Forced to 0 unless the wide-range accel is an LSM303DLHC |

### 10.4 Capability gates

| Field | Rule |
|---|---|
| `tcxo` | Forced to 0 where `IS_SUPPORTED_TCXO` is false |
| `singleTouchStart` | Forced to 0 where `IS_SUPPORTED_SINGLE_TOUCH` is false. Where supported, setting it also forces `userButtonEnable` and `syncEnable` on — single-touch needs both |
| `exgADS1292rRegsCh1.config2` bit 3 | Forced to 1 where the ADS1292R clock lines are tied |
| `btPinSetup` | Always forced to 0 — the Bluetooth driver no longer needs it |

### 10.5 MAC address

The stored `macAddr` is compared against the address read from the Bluetooth
module and **overwritten from the module** on any mismatch. A host cannot set
this field; writing it is a no-op that will be silently reverted.

`ShimSdSync_checkSyncCenterName` also runs here and may adjust the sync
configuration.

## 11. Defaults

`ShimConfig_setDefaultConfig` starts from `ShimConfig_createBlankConfigBytes`
— all zeros, then all six calibration blocks to `0xFF`, the MAC copied from the
Bluetooth module, bytes 232-255 to `0xFF`, and the whole 128-byte sync page to
`0xFF` — and then applies:

| Setting | Default |
|---|---|
| Sampling rate | 51.2 Hz (640 ticks) |
| Buffer size | 1 |
| Enabled channels | Low-noise accel, gyro, mag, battery |
| Wide-range accel | 100 Hz, ±2 g, high-performance / HR and LP modes off |
| Gyro rate | S3: `0x9B` (8 kHz / 156 = 51.282 Hz). S3R: the 60 Hz ODR |
| Gyro range | ±500 dps |
| Mag | S3: ±1.3 Ga at 75 Hz. S3R: LIS2MDL at 100 Hz; LIS3MDL ±4 Ga at 80 Hz |
| Low-noise / alt accel range | ±2 g |
| Pressure oversampling | S3: OSS 1. S3R: none |
| GSR range | Auto-range |
| Expansion board power | Off |
| ExG registers | The ECG set (§5) |
| Bluetooth baud | `getDefaultBaudForBtVersion()`, only if currently `0xFF` |
| Shimmer name | `Shimmer_<mac4>` |
| Experiment ID | `DefaultTrial` |
| Config time | 0 |
| Trial ID, shimmer count | 0 |
| Button, RTC error, SD error | Enabled |
| Bluetooth | Enabled |
| Sync broadcast interval | 54 s |
| Max experiment length | 0 (auto-stop disabled) |
| Estimated experiment length | 1 s |

It finishes by running `ShimConfig_checkAndCorrectConfig`, setting
`flagWriteCfgToSd`, refreshing the calibration blocks from the dump, and
flushing to non-volatile memory.

### 11.1 What counts as an erased InfoMem

`ShimConfig_areConfigBytesValid` returns false only when **all six MAC address
bytes at 224-229 are `0xFF`**. Nothing else is examined. A block whose entire
first page is erased but whose MAC bytes are intact is treated as valid and used
as-is.

> The function's own commented-out predecessor compared the first six bytes
> instead. The live check is the MAC, which is why a partially erased
> configuration can survive validation.

## 12. Layout version gates

The firmware has no layout versioning: the running build's `gConfigBytes` is the
layout, and there is exactly one. Version gating is entirely a **host-side**
concern, needed because a host must parse configuration from devices running
older firmware.

Both host SDKs express this the same way, as predicates over a
`(hardwareVersion, firmwareIdentifier, major, minor, internal)` tuple compared
with the Java `UtilShimmer.compareVersions` semantics, where a wildcard
(`ANY_VERSION`, -1) in any field makes that field's test pass.

The gates that select layout variants cover: the MPL (MPU9150 DMP) block, which
is Shimmer3 with SDLog firmware in a bounded version window; the eight-byte
derived-sensor extension, which requires SDLog 0.13.1 or later; and the
Shimmer3R-only fields, gated on the hardware identifier alone.

> **For current LogAndStream on Shimmer3, and for all Shimmer3R, every gate
> evaluates true.** Only the newest layout is live. The gates matter when
> reading a device that has not been updated, or a stored configuration
> captured from one — not when talking to current firmware.

Because the MPL gate is SDLog-only, the MPL calibration bytes at 133-174 are
free for reuse on Shimmer3R, which is exactly what
`NV_ALT_ACCEL_CALIBRATION` and `NV_ALT_MAG_CALIBRATION` do. A host must
therefore resolve those bytes by hardware generation, not by offset alone: the
same addresses mean MPL accelerometer/magnetometer calibration on an
SDLog Shimmer3 and ADXL371/LIS3MDL calibration on a Shimmer3R. The Java driver
reflects this by declaring two index names for each — `idxMPLAccelCalibration`
alongside `idxADXL371AltAccelCalibration`, and `idxMPLMagCalibration` alongside
`idxLIS3MDLAltMagCalibration`.

## Appendix A. InfoMem to SD-header pairs

`ShimSdHead_config2SdHead` copies configuration into the SD-card data-file
header. The two layouts are **not parallel** — it is a field-by-field copy to
different offsets, which is the whole reason this table exists.

The header is filled with `0xFF` first, so any offset not listed below is `0xFF`
in a written file.

### A.1 Direct byte copies

| Config byte(s) | `NV_` constant | SD header | `SDH_` constant |
|---:|---|---:|---|
| 0-1 | `NV_SAMPLING_RATE` | 0-1 | `SDH_SAMPLE_RATE_0/1` |
| 2 | `NV_BUFFER_SIZE` | 2 | `SDH_BUFFER_SIZE` |
| 3 | `NV_SENSORS0` | 3 | `SDH_SENSORS0` |
| 4 | `NV_SENSORS1` | 4 | `SDH_SENSORS1` |
| 5 | `NV_SENSORS2` | 5 | `SDH_SENSORS2` |
| 128 | `NV_SENSORS3` | 6 | `SDH_SENSORS3` |
| 129 | `NV_SENSORS4` | 7 | `SDH_SENSORS4` |
| 6 | `NV_CONFIG_SETUP_BYTE0` | 8 | `SDH_CONFIG_SETUP_BYTE0` |
| 7 | `NV_CONFIG_SETUP_BYTE1` | 9 | `SDH_CONFIG_SETUP_BYTE1` |
| 8 | `NV_CONFIG_SETUP_BYTE2` | 10 | `SDH_CONFIG_SETUP_BYTE2` |
| 9 | `NV_CONFIG_SETUP_BYTE3` | 11 | `SDH_CONFIG_SETUP_BYTE3` |
| 130 | `NV_CONFIG_SETUP_BYTE4` | 12 | `SDH_CONFIG_SETUP_BYTE4` |
| 131 | `NV_CONFIG_SETUP_BYTE5` | 13 | `SDH_CONFIG_SETUP_BYTE5` |
| 132 | `NV_CONFIG_SETUP_BYTE6` | 14 | `SDH_CONFIG_SETUP_BYTE6` |
| 217 | `NV_SD_TRIAL_CONFIG0` | 16 | `SDH_TRIAL_CONFIG0` |
| 218 | `NV_SD_TRIAL_CONFIG1` | 17 | `SDH_TRIAL_CONFIG1` |
| 219 | `NV_SD_BT_INTERVAL` | 18 | `SDH_BROADCAST_INTERVAL` |
| 30 | `NV_BT_COMMS_BAUD_RATE` | 19 | `SDH_BT_COMMS_BAUD_RATE` |
| 220 | `NV_EST_EXP_LEN_MSB` | 20 | `SDH_EST_EXP_LEN_MSB` |
| 221 | `NV_EST_EXP_LEN_LSB` | 21 | `SDH_EST_EXP_LEN_LSB` |
| 222 | `NV_MAX_EXP_LEN_MSB` | 22 | `SDH_MAX_EXP_LEN_MSB` |
| 223 | `NV_MAX_EXP_LEN_LSB` | 23 | `SDH_MAX_EXP_LEN_LSB` |
| 215 | `NV_SD_MYTRIAL_ID` | 32 | `SDH_MYTRIAL_ID` |
| 216 | `NV_SD_NSHIMMER` | 33 | `SDH_NSHIMMER` |
| 31 | `NV_DERIVED_CHANNELS_0` | 40 | `SDH_DERIVED_CHANNELS_0` |
| 32 | `NV_DERIVED_CHANNELS_1` | 41 | `SDH_DERIVED_CHANNELS_1` |
| 33 | `NV_DERIVED_CHANNELS_2` | 42 | `SDH_DERIVED_CHANNELS_2` |
| 211-214 | `NV_SD_CONFIG_TIME` | 52-55 | `SDH_CONFIG_TIME_0..3` |
| 10-19 | `NV_EXG_ADS1292R_1_*` | 56-65 | `SDH_EXG_ADS1292R_1_*` |
| 20-29 | `NV_EXG_ADS1292R_2_*` | 66-75 | `SDH_EXG_ADS1292R_2_*` |
| 118 | `NV_DERIVED_CHANNELS_3` | 217 | `SDH_DERIVED_CHANNELS_3` |
| 119 | `NV_DERIVED_CHANNELS_4` | 218 | `SDH_DERIVED_CHANNELS_4` |
| 120 | `NV_DERIVED_CHANNELS_5` | 219 | `SDH_DERIVED_CHANNELS_5` |
| 121 | `NV_DERIVED_CHANNELS_6` | 220 | `SDH_DERIVED_CHANNELS_6` |
| 122 | `NV_DERIVED_CHANNELS_7` | 221 | `SDH_DERIVED_CHANNELS_7` |

> Note the reordering: the two ExG banks are contiguous in both layouts but the
> derived-channel bytes are split 40-42 / 217-221 in the header against
> 31-33 / 118-122 in the configuration, and the four setup bytes 6-9 land at
> 8-11 with the Shimmer3R additions 130-132 following at 12-14 rather than
> staying in their own page.

### A.2 Calibration blocks

Covered in [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) §4.2. The
important point is that **the order differs**: InfoMem runs
LN-accel (34), gyro (55), mag (76), WR-accel (97); the header runs
WR-accel (76), gyro (97), mag (118), LN-accel (139).

The header additionally carries an 8-byte calibration timestamp per sensor,
which has no InfoMem counterpart, and the copy is performed by
`ShimCalib_calibDumpToConfigBytesAndSdHeaderAll` rather than by
`ShimSdHead_config2SdHead` directly.

### A.3 Header-only fields

These have no configuration-byte source; they are stamped from the device at
file-creation time:

| SD header | `SDH_` constant | Source |
|---:|---|---|
| 24-29 | `SDH_MAC_ADDR` | Read from the Bluetooth module, **not** from `NV_MAC_ADDRESS` |
| 30-31 | `SDH_SHIMMERVERSION_BYTE_0/1` | `DEVICE_VER`, big-endian |
| 34-35 | `SDH_FW_VERSION_TYPE_0/1` | `FW_IDENTIFIER`, big-endian |
| 36-37 | `SDH_FW_VERSION_MAJOR_0/1` | Big-endian |
| 38 | `SDH_FW_VERSION_MINOR` | |
| 39 | `SDH_FW_VERSION_INTERNAL` | Patch version |
| 44-51 | `SDH_RTC_DIFF_0..7` | RWC offset, MSB order |
| 160-181 | `SDH_TEMP_PRES_CALIBRATION` | Pressure coefficients read from the part |
| 222-223 | `SDH_TEMP_PRES_EXTRA_CALIB_BYTES` | BMP280 only — its 24 bytes do not fit the 22-byte field |
| 214-216 | `SDH_DAUGHTER_CARD_ID_BYTE0` +3 | Expansion board ID |
| 251-255 | `SDH_INITIAL_TIMESTAMP_*` | Written when logging starts |
| 314 | `SDH_NUM_ENABLED_CHANNELS` | S3R only |
| 315+ | `SDH_CHANNEL_ID_BYTE_0` +50 | S3R only — the resolved channel order |

> **The MAC in the header is not the MAC in the configuration.**
> `ShimSdHead_config2SdHead` takes it "direct from BT and not configuration".
> In practice §10.5 keeps the two in step, but the header is the more
> trustworthy of the two.

> **`SDH_RTC_DIFF_*` means different things on the two platforms.** The
> Shimmer3 branch of `ShimSdHead_config2SdHead` copies
> `RTC_getRwcTimeDiffPtr()`; the Shimmer3R branch writes eight zero bytes under
> a `TODO check is this is working` comment. On Shimmer3R those bytes are then
> **repurposed at file-creation time**: `ShimSdDataFile_writeSdHeaderToFile`
> overwrites the top three with bits 40-63 of the 64-bit initial timestamp,
> because the five-byte `SDH_INITIAL_TIMESTAMP_*` field cannot hold it. See
> [SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md) §3.3 and
> [SHIMMER3_TIMEKEEPING.md](SHIMMER3_TIMEKEEPING.md).

> **`SDH_NUM_ENABLED_CHANNELS` and the channel-ID list are Shimmer3R-only.**
> On Shimmer3 those offsets stay `0xFF`, and channel order must be obtained from
> the inquiry response instead.

## Still unverified / not found in code

- **Bytes 232-255.** `ShimConfig_createBlankConfigBytes` fills them with `0xFF`
  via `memset(&rawBytes[NV_BT_SET_PIN + 1], 0xFF, 24)`, and the struct names
  them `unusedIdx232` through `unusedIdx255`. Whether any historical firmware
  used them is not determinable from the current source.
- ~~Byte 132 / `NV_CONFIG_SETUP_BYTE6`~~ — confirmed unused: allocated,
  transferred and mirrored to the SD header, but no field on either platform
  reads or writes it. Treat it as reserved.
- **The three historical size constants.** `NV_NUM_SETTINGS_BYTES` (34),
  `NV_NUM_CALIBRATION_BYTES` (84) and `NV_NUM_SD_BYTES` (37) sum to 155 while
  `NV_TOTAL_NUM_CONFIG_BYTES` is 384, and the comment beside the latter claims
  it is their sum. Which layout they described was not established.
- ~~`syncNodeAddr` (the tenth entry, bytes 310-315)~~ — resolved: a typo with
  no consequence. The only reference to the name is its declaration in
  `shimmer_config.h`; no code accesses any node slot by field name, so the
  missing suffix changes nothing.
- ~~Whether `numberOfShimmers` is ever validated against the node table~~ —
  resolved: it is **never consumed**. It is written by `SET_TRIAL_CONFIG`
  (`args[0]`) and by the `sdlog.cfg` `NSHIMMER` key, echoed in the
  corresponding responses and file, and never read by `SDSync/`; the sync code
  counts populated node entries into its own `syncNodeNum`. See
  [SHIMMER3_SD_SYNC.md](SHIMMER3_SD_SYNC.md).
