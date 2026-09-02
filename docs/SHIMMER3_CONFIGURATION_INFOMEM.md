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
