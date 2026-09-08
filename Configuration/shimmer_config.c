/*
 * Copyright (c) 2016, Shimmer Research, Ltd.
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *    * Neither the name of Shimmer Research, Ltd. nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *    * You may not use or distribute this Software or any derivative works
 *      in any form for commercial purposes with the exception of commercial
 *      purposes when used in conjunction with Shimmer products purchased
 *      from Shimmer or their designated agent or with permission from
 *      Shimmer.
 *      Examples of commercial purposes would be running business
 *      operations, licensing, leasing, or selling the Software, or
 *      distributing the Software for use with commercial products.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Weibo Pan
 * @date May, 2016
 */

#include <Configuration/shimmer_config.h>

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "log_and_stream_externs.h"
#include "log_and_stream_includes.h"
#include "shimmer_definitions.h"

#if defined(SHIMMER3R)
/* For the ADXL371 ODR enum. The LIS2DW12, LIS2MDL, LIS3MDL and LSM6DSV enums
 * arrive through shimmer_config.h; this one has no other route in. */
#include "ADXL371/adxl371.h"
#endif //SHIMMER3R

static gConfigBytes storedConfig;
uint8_t calibRamFlag = 0;

char expIdName[MAX_CHARS], shimmerName[MAX_CHARS], configTimeText[UINT32_LEN];

void ShimConfig_reset(void)
{
  memset(storedConfig.rawBytes, 0xFF, NV_NUM_RWMEM_BYTES);
  storedConfig.rawBytes[NV_SD_SHIMMER_NAME] = '\0';
  storedConfig.rawBytes[NV_SD_EXP_ID_NAME] = '\0';

  calibRamFlag = 0;

  memset(&expIdName[0], 0x00, sizeof(expIdName));
  memset(&shimmerName[0], 0x00, sizeof(shimmerName));
  memset(&configTimeText[0], 0x00, sizeof(configTimeText));
}

void ShimConfig_readRam(void)
{
  //init RAM (storedConfig)
#if USE_DEFAULT_SENSOR
  ShimConfig_setDefaultConfig();
#else
  gConfigBytes temp_storedConfig;
  InfoMem_read(0, temp_storedConfig.rawBytes, STOREDCONFIG_SIZE);
  if ((temp_storedConfig.rawBytes[NV_SENSORS1] == 0xFF)
      || temp_storedConfig.samplingRateTicks == 0)
  {
    //if config was never written to Infomem, write default
    //assuming some other app didn't make use of InfoMem, or else InfoMem was erased
    ShimConfig_setDefaultConfig();
  }
  else
  {
    //memcpy(temp_storedConfig+NV_MAC_ADDRESS, btMacHex, 6);
    ShimConfig_storedConfigSet(temp_storedConfig.rawBytes, 0, STOREDCONFIG_SIZE);
  }
#endif //USE_DEFAULT_SENSOR

  //TODO ShimConfig_checkBtModeFromConfig should be called at a higher level and from a single place
  /* Check BT module configuration after sensor configuration read from
   * infomem to see if it is in the correct state (i.e., BT on vs. BT off vs.
   * SD Sync) */
  ShimConfig_checkBtModeFromConfig();
}

gConfigBytes *ShimConfig_getStoredConfig(void)
{
  return &storedConfig;
}

/*
 * storedConfig: Set(), Get() and GetByte(), S4Ram_sdHeadTextSetByte()
 */

uint8_t ShimConfig_storedConfigSet(const uint8_t *buf, uint16_t offset, uint16_t length)
{
  if ((offset > STOREDCONFIG_SIZE - 1) || (offset + length > STOREDCONFIG_SIZE)
      || (length == 0))
  {
    return 1;
  }
  memcpy(&storedConfig.rawBytes[offset], buf, length);
  return 0;
}

uint8_t ShimConfig_storedConfigGet(uint8_t *buf, uint16_t offset, uint16_t length)
{
  if ((offset > STOREDCONFIG_SIZE - 1) || (offset + length > STOREDCONFIG_SIZE)
      || (length == 0))
  {
    return 1;
  }
  memcpy(buf, &storedConfig.rawBytes[offset], length);
  return 0;
}

uint8_t ShimConfig_storedConfigGetByte(uint16_t offset)
{
  if (offset > STOREDCONFIG_SIZE - 1)
  {
    return 0; //or not?
  }
  return storedConfig.rawBytes[offset];
}

uint8_t ShimConfig_storedConfigSetByte(uint16_t offset, uint8_t val)
{
  if (offset > STOREDCONFIG_SIZE - 1)
  {
    return 1;
  }
  storedConfig.rawBytes[offset] = val;
  return 0;
}

void ShimConfig_setDefaultConfig(void)
{
  ShimConfig_createBlankConfigBytes();
  storedConfig.samplingRateTicks = ShimConfig_freqDiv(51.2); //51.2Hz
  storedConfig.bufferSize = 1;
  /* core sensors enabled */
  storedConfig.chEnLnAccel = 1;
  storedConfig.chEnMag = 1;
  storedConfig.chEnGyro = 1;
  storedConfig.chEnVBattery = 1;

#if defined(SHIMMER3)
  /* LSM303 Accel 100Hz, +/-2G, Low Power and High Resolution modes off */
  storedConfig.wrAccelRate = LSM303DLHC_ACCEL_100HZ;
  storedConfig.wrAccelRange = ACCEL_2G;
  storedConfig.wrAccelHrMode = 0;
  ShimConfig_wrAccelLpModeSet(0);
  /* MPU9X50/ICM20948 sampling rate of 8kHz/(155+1), i.e. 51.282Hz */
  ShimConfig_gyroRateSet(0x9B);
  /* LSM303 Mag 75Hz, +/-1.3 Gauss, MPU9150 Gyro +/-500 degrees per second */
  storedConfig.magRange = LSM303DLHC_MAG_1_3G;
  ShimConfig_configByteMagRateSet(LSM303DLHC_MAG_75HZ);
  ShimConfig_gyroRangeSet(MPU9X50_GYRO_500DPS);
  /* MPU9X50/ICM20948 Accel +/-2G */
  storedConfig.altAccelRange = ACCEL_2G;
  /* BMP pressure oversampling ratio 1 */
  ShimConfig_configBytePressureOversamplingRatioSet(BMPX80_OSS_1);
#elif defined(SHIMMER3R)
  /* LIS2DW12 Accel 100Hz, +/-2G, Low Power and High Resolution modes off */
  storedConfig.wrAccelRate = LIS2DW12_XL_ODR_100Hz;
  storedConfig.wrAccelRange = LIS2DW12_2g;
  ShimConfig_wrAccelModeSet(LIS2DW12_HIGH_PERFORMANCE);
  /* LSM6DSV Gyro sampling rate, next highest to 51.2Hz */
  ShimConfig_gyroRateSet(LSM6DSV_ODR_AT_60Hz);
  /* LIS2MDL Mag 100Hz */
  ShimConfig_configByteMagRateSet(LIS2MDL_ODR_100Hz);
  /* LIS3MDL Mag 80Hz, +/-4 Gauss */
  storedConfig.altMagRange = LIS3MDL_4_GAUSS;
  ShimConfig_configByteAltMagRateSet(LIS3MDL_UHP_80Hz);
  /* LSM6DSV Gyro +/-500 degrees per second */
  ShimConfig_gyroRangeSet(LSM6DSV_500dps);
  storedConfig.lnAccelRange = LSM6DSV_2g;
  ShimConfig_configBytePressureOversamplingRatioSet(BMP3_NO_OVERSAMPLING);
#endif
  /* GSR auto range */
  storedConfig.gsrRange = GSR_AUTORANGE;

  /* EXP_RESET_N pin set low */
  storedConfig.expansionBoardPower = 0;

  //set all ExG registers to their reset values
  //setExgConfigForTestSignal();
  ShimConfig_setExgConfigForEcg();

  /*BT Baud Rate*/
  if (storedConfig.btCommsBaudRate == 0xFF)
  {
    storedConfig.btCommsBaudRate = getDefaultBaudForBtVersion();
  }

  //sd config
  //shimmername
  ShimConfig_setDefaultShimmerName();
  //exp_id
  ShimConfig_setDefaultTrialId();
  ShimConfig_setDefaultConfigTime();

  storedConfig.myTrialID = 0;
  storedConfig.numberOfShimmers = 0;
  storedConfig.userButtonEnable = 1;
  storedConfig.rtcErrorEnable = 1;
  storedConfig.sdErrorEnable = 1;
  storedConfig.btIntervalSecs = 54;
  storedConfig.bluetoothDisable = 0;

  /* Auto-stop disabled */
  ShimConfig_experimentLengthMaxInMinutesSet(0);

  /* SD sync */
  ShimConfig_experimentLengthEstimatedInSecSet(1);
  ShimSdSync_setSyncEstExpLen((uint32_t) ShimConfig_experimentLengthEstimatedInSecGet());

  ShimConfig_checkAndCorrectConfig();
  ShimConfig_setFlagWriteCfgToSd(1, 0);

  ShimCalib_calibDumpToConfigBytesAndSdHeaderAll(0);

  /* Write RAM contents to Infomem */
  LogAndStream_infomemUpdate();
}

void ShimConfig_setDefaultShimmerName(void)
{
  strcpy(&storedConfig.shimmerName[0], "Shimmer_XXXX");
  memcpy(&storedConfig.shimmerName[8], &ShimBt_macIdStrPtrGet()[8], 4);
}

void ShimConfig_setDefaultTrialId(void)
{
  memcpy(&storedConfig.expIdName[0], "DefaultTrial", 12);
}

void ShimConfig_setDefaultConfigTime(void)
{
  ShimConfig_configTimeSet(0);
}

void ShimConfig_configTimeSet(uint32_t time)
{
  //Config time is stored in MSB order in the config bytes
  storedConfig.configTime0 = (time >> 24) & 0xFF;
  storedConfig.configTime1 = (time >> 16) & 0xFF;
  storedConfig.configTime2 = (time >> 8) & 0xFF;
  storedConfig.configTime3 = (time >> 0) & 0xFF;
}

uint32_t ShimConfig_configTimeGet(void)
{
  uint32_t time = 0;
  time |= ((uint32_t) storedConfig.configTime0) << 24;
  time |= ((uint32_t) storedConfig.configTime1) << 16;
  time |= ((uint32_t) storedConfig.configTime2) << 8;
  time |= ((uint32_t) storedConfig.configTime3);
  return time;
}

uint8_t ShimConfig_getFlagWriteCfgToSd(void)
{
  return (storedConfig.flagWriteCfgToSd);
}

void ShimConfig_setFlagWriteCfgToSd(uint8_t flag, uint8_t writeToFlash)
{
  storedConfig.flagWriteCfgToSd = flag;
  if (writeToFlash)
  {
    InfoMem_write(NV_SD_CONFIG_DELAY_FLAG,
        &storedConfig.rawBytes[NV_SD_CONFIG_DELAY_FLAG], 1);
  }
}

uint8_t ShimConfig_getRamCalibFlag(void)
{
  return calibRamFlag;
}

void ShimConfig_setRamCalibFlag(uint8_t flag)
{
  //flag == 1: Ram>File, ShimmerCalib_ram2File()
  //        0: File>Ram, ShimmerCalib_file2Ram()
  calibRamFlag = flag;
}

float ShimConfig_getShimmerSamplingFreq(void)
{
  return 32768.0 / (float) storedConfig.samplingRateTicks;
}

void ShimConfig_gyroRangeSet(uint8_t value)
{
#if defined(SHIMMER3)
  value = (value <= MPU9X50_GYRO_2000DPS) ? value : MPU9X50_GYRO_500DPS;
#elif defined(SHIMMER3R)
  value = (value <= (LSM6DSV_2000dps + 1)) ? value : LSM6DSV_500dps;
#endif
  storedConfig.gyroRangeLsb = value & 0x03;
#if defined(SHIMMER3R)
  storedConfig.gyroRangeMsb = (value >> 2) & 0x01;
#endif
}

uint8_t ShimConfig_gyroRangeGet(void)
{
#if defined(SHIMMER3)
  return storedConfig.gyroRangeLsb;
#elif defined(SHIMMER3R)
  return (storedConfig.gyroRangeMsb << 2) | storedConfig.gyroRangeLsb;
#endif
}

void ShimConfig_gyroRateSet(uint8_t value)
{
#if defined(SHIMMER3R)
  value = (value < LSM6DSV_ODR_AT_7680Hz) ? value : LSM6DSV_ODR_AT_60Hz;
#endif
  storedConfig.gyroRate = value;
}

void ShimConfig_wrAccelLpModeSet(uint8_t value)
{
#if defined(SHIMMER3)
  value = (value == 1) ? 1 : 0;
#elif defined(SHIMMER3R)
  value = (value <= 3) ? value : 0;
#endif
  storedConfig.wrAccelLpModeLsb = value & 0x01;
#if defined(SHIMMER3R)
  storedConfig.wrAccelLpModeMsb = (value >> 1) & 0x01;
#endif
}

uint8_t ShimConfig_wrAccelLpModeGet(void)
{
#if defined(SHIMMER3)
  return storedConfig.wrAccelLpModeLsb;
#elif defined(SHIMMER3R)
  return (storedConfig.wrAccelLpModeMsb << 1) | storedConfig.wrAccelLpModeLsb;
#endif
}

#if defined(SHIMMER3R)
void ShimConfig_wrAccelModeSet(lis2dw12_mode_t value)
{
  storedConfig.wrAccelHrMode = (value >> 2) & 0x01;
  ShimConfig_wrAccelLpModeSet(value & 0x03);
}

lis2dw12_mode_t ShimConfig_wrAccelModeGet(void)
{
  lis2dw12_mode_t wrAccelMode = (lis2dw12_mode_t) ((storedConfig.wrAccelHrMode << 2)
      | ShimConfig_wrAccelLpModeGet());
  return wrAccelMode;
}
#endif

void ShimConfig_configBytePressureOversamplingRatioSet(uint8_t value)
{
#if defined(SHIMMER3)
  value = (value <= BMPX80_OSS_8) ? (value & 0x03) : BMPX80_OSS_1;
#elif defined(SHIMMER3R)
  /* The BMP581 supports two additional oversampling settings (64x and 128x)
   * over the BMP390's maximum of 32x */
  uint8_t maxOversamplingRatio = isBmp581InUse() ? BMP5_OVERSAMPLING_128X : BMP3_OVERSAMPLING_32X;
  value = (value <= maxOversamplingRatio) ? value : BMP3_NO_OVERSAMPLING;
#endif
  storedConfig.pressureOversamplingRatioLsb = value & 0x03;
#if defined(SHIMMER3R)
  storedConfig.pressureOversamplingRatioMsb = (value >> 2) & 0x01;
#endif
}

uint8_t ShimConfig_configBytePressureOversamplingRatioGet(void)
{
#if defined(SHIMMER3)
  return storedConfig.pressureOversamplingRatioLsb;
#elif defined(SHIMMER3R)
  return (storedConfig.pressureOversamplingRatioMsb << 2)
      | storedConfig.pressureOversamplingRatioLsb;
#endif
}

void ShimConfig_configByteMagRateSet(uint8_t value)
{
#if defined(SHIMMER3)
  value = (value < LSM303DLHC_MAG_220HZ) ? value : LSM303DLHC_MAG_75HZ;
#else
  value = (value <= LIS2MDL_ODR_100Hz) ? value : LIS2MDL_ODR_100Hz;
#endif
  storedConfig.magRate = value;
}

uint8_t ShimConfig_configByteMagRateGet(void)
{
  return storedConfig.magRate;
}

void ShimConfig_configByteAltMagRateSet(uint8_t value)
{
#if defined(SHIMMER3)
  value = 0; //not used
#elif defined(SHIMMER3R)
  value = value <= (LIS3MDL_UHP_80Hz) ? value : LIS3MDL_UHP_80Hz;
#endif
  storedConfig.altMagRate = value;
}

uint8_t ShimConfig_configByteAltMagRateGet(void)
{
  return storedConfig.altMagRate;
}

#if defined(SHIMMER3R)
/**
 * Output rate of an LSM6DSV ODR setting, in Hz.
 *
 * Decodes every one of the part's twelve plain ODR codes, 7680 Hz (code 12)
 * included. That is deliberate and worth spelling out, because this firmware's
 * own setter will not produce a 12: ShimConfig_gyroRateSet substitutes 60 Hz
 * for anything from LSM6DSV_ODR_AT_7680Hz upwards, which is why the correction
 * ladder in ShimConfig_lsm6dsvOdrForFreq stops at 3840. But the setter is not
 * the only way into the field. An InfoMem write is a raw memcpy
 * (ShimConfig_storedConfigSet), so a host can store a 12 unclamped, and spi.c
 * then hands it to lsm6dsv_configure verbatim - the chip really runs at
 * 7680 Hz. Reading that as 0 here would send it through a "correction" down to
 * 3840, actively downgrading a configuration that works.
 *
 * @param odr a value from the LSM6DSV ODR enum (lsm6dsv_reg.h)
 * @return the rate in Hz, or 0 for a setting that produces no new samples:
 *         power-down, and the high-accuracy encodings (LSM6DSV_ODR_HA01_* and
 *         HA02_*, 0x13 and up) that nothing in this firmware writes or
 *         configures. A 0 for one of those does trigger a correction, and
 *         should - it is an encoding this firmware does not support.
 */
static float ShimConfig_lsm6dsvOdrToHz(uint8_t odr)
{
  switch (odr)
  {
    case LSM6DSV_ODR_AT_1Hz875:
      return 1.875f;
    case LSM6DSV_ODR_AT_7Hz5:
      return 7.5f;
    case LSM6DSV_ODR_AT_15Hz:
      return 15.0f;
    case LSM6DSV_ODR_AT_30Hz:
      return 30.0f;
    case LSM6DSV_ODR_AT_60Hz:
      return 60.0f;
    case LSM6DSV_ODR_AT_120Hz:
      return 120.0f;
    case LSM6DSV_ODR_AT_240Hz:
      return 240.0f;
    case LSM6DSV_ODR_AT_480Hz:
      return 480.0f;
    case LSM6DSV_ODR_AT_960Hz:
      return 960.0f;
    case LSM6DSV_ODR_AT_1920Hz:
      return 1920.0f;
    case LSM6DSV_ODR_AT_3840Hz:
      return 3840.0f;
    case LSM6DSV_ODR_AT_7680Hz:
      return 7680.0f;
    default:
      return 0.0f;
  }
}

/**
 * Lowest LSM6DSV ODR that keeps up with a packet rate.
 *
 * The ladder is the Java driver's, branch for branch
 * (SensorLSM6DSV.getGyroRateFromFreq), and matching it is deliberate rather
 * than incidental: the correction below has to land on the value Consensys
 * would have chosen, or every write from Consensys would be "corrected" to a
 * different one, flagged as changed and persisted - churn on every connect.
 *
 * That is also why LSM6DSV_ODR_AT_15Hz is skipped even though the part
 * supports it: the Java ladder steps 7.5 Hz straight to 30 Hz.
 */
static uint8_t ShimConfig_lsm6dsvOdrForFreq(float freq)
{
  if (freq <= 7.5f)
  {
    return LSM6DSV_ODR_AT_7Hz5;
  }
  else if (freq <= 30.0f)
  {
    return LSM6DSV_ODR_AT_30Hz;
  }
  else if (freq <= 60.0f)
  {
    return LSM6DSV_ODR_AT_60Hz;
  }
  else if (freq <= 120.0f)
  {
    return LSM6DSV_ODR_AT_120Hz;
  }
  else if (freq <= 240.0f)
  {
    return LSM6DSV_ODR_AT_240Hz;
  }
  else if (freq <= 480.0f)
  {
    return LSM6DSV_ODR_AT_480Hz;
  }
  else if (freq <= 960.0f)
  {
    return LSM6DSV_ODR_AT_960Hz;
  }
  else if (freq <= 1920.0f)
  {
    return LSM6DSV_ODR_AT_1920Hz;
  }
  return LSM6DSV_ODR_AT_3840Hz;
}

/**
 * Effective output rate of a LIS2DW12 ODR setting, in Hz.
 *
 * Mode-dependent, unlike the LSM6DSV's: LIS2DW12_XL_ODR_1Hz6_LP_ONLY is 1.6 Hz
 * in a low-power mode and 12.5 Hz in high performance, and a low-power mode
 * tops out at 200 Hz however fast the ODR field asks for. Both are reported
 * here as what the part will actually deliver, so the check below cannot be
 * satisfied by a rate the part is not going to produce.
 *
 * @param odr LIS2DW12 ODR field value (lis2dw12_reg.h)
 * @param isHighPerformance the wrAccelHrMode bit - LIS2DW12_HIGH_PERFORMANCE is
 *        0x04, i.e. that bit, so it alone separates high performance from the
 *        continuous low-power modes
 * @return the delivered rate in Hz, or 0 for power-down
 */
static float ShimConfig_lis2dw12OdrToHz(uint8_t odr, uint8_t isHighPerformance)
{
  switch (odr)
  {
    case LIS2DW12_XL_ODR_1Hz6_LP_ONLY:
      return isHighPerformance ? 12.5f : 1.6f;
    case LIS2DW12_XL_ODR_12Hz5:
      return 12.5f;
    case LIS2DW12_XL_ODR_25Hz:
      return 25.0f;
    case LIS2DW12_XL_ODR_50Hz:
      return 50.0f;
    case LIS2DW12_XL_ODR_100Hz:
      return 100.0f;
    case LIS2DW12_XL_ODR_200Hz:
      return 200.0f;
    case LIS2DW12_XL_ODR_400Hz:
      return isHighPerformance ? 400.0f : 200.0f;
    case LIS2DW12_XL_ODR_800Hz:
      return isHighPerformance ? 800.0f : 200.0f;
    case LIS2DW12_XL_ODR_1k6Hz:
      return isHighPerformance ? 1600.0f : 200.0f;
    default:
      return 0.0f;
  }
}

/**
 * Lowest LIS2DW12 ODR that keeps up with a packet rate, in the current mode.
 *
 * Taken from the register map rather than from the Java driver, which is a
 * deliberate departure from how the LSM6DSV ladder above was derived.
 * SensorLIS2DW12.getAccelRateFromFreq returns ODR field value 1 for
 * "freq <= 12.5" and comments it 12.5 Hz, but value 1 is
 * LIS2DW12_XL_ODR_1Hz6_LP_ONLY. Copying that would make a correction in a
 * low-power mode re-select the same 1.6 Hz and never settle. In high
 * performance the two agree anyway, because value 1 really is 12.5 Hz there.
 *
 * Capped at the 200 Hz value in a low-power mode: the part cannot go faster in
 * that mode, so asking for a higher ODR would promise a rate it will not
 * deliver and leave the check firing on every pass.
 */
static uint8_t ShimConfig_lis2dw12OdrForFreq(float freq, uint8_t isHighPerformance)
{
  if (freq <= 12.5f)
  {
    return LIS2DW12_XL_ODR_12Hz5;
  }
  else if (freq <= 25.0f)
  {
    return LIS2DW12_XL_ODR_25Hz;
  }
  else if (freq <= 50.0f)
  {
    return LIS2DW12_XL_ODR_50Hz;
  }
  else if (freq <= 100.0f)
  {
    return LIS2DW12_XL_ODR_100Hz;
  }
  else if ((freq <= 200.0f) || !isHighPerformance)
  {
    return LIS2DW12_XL_ODR_200Hz;
  }
  else if (freq <= 400.0f)
  {
    return LIS2DW12_XL_ODR_400Hz;
  }
  else if (freq <= 800.0f)
  {
    return LIS2DW12_XL_ODR_800Hz;
  }
  return LIS2DW12_XL_ODR_1k6Hz;
}

/**
 * Output rate of a LIS2MDL ODR setting, in Hz.
 *
 * The part has no power-down code - 0 is 10 Hz - so a disabled magnetometer is
 * simply left slow rather than switched off. 100 Hz is its ceiling.
 */
static float ShimConfig_lis2mdlOdrToHz(uint8_t odr)
{
  switch (odr)
  {
    case LIS2MDL_ODR_10Hz:
      return 10.0f;
    case LIS2MDL_ODR_20Hz:
      return 20.0f;
    case LIS2MDL_ODR_50Hz:
      return 50.0f;
    case LIS2MDL_ODR_100Hz:
      return 100.0f;
    default:
      return 0.0f;
  }
}

/** Lowest LIS2MDL ODR that keeps up, capped at the part's 100 Hz ceiling. */
static uint8_t ShimConfig_lis2mdlOdrForFreq(float freq)
{
  if (freq <= 10.0f)
  {
    return LIS2MDL_ODR_10Hz;
  }
  else if (freq <= 20.0f)
  {
    return LIS2MDL_ODR_20Hz;
  }
  else if (freq <= 50.0f)
  {
    return LIS2MDL_ODR_50Hz;
  }
  return LIS2MDL_ODR_100Hz;
}

/**
 * Output rate of a LIS3MDL rate setting, in Hz.
 *
 * The field is composite - (operating mode << 4) | rate - so it is decoded
 * rather than tabulated. A low nibble of 1 selects the mode's own fast rate,
 * which differs per mode: 1000 Hz in low power, 560 Hz medium, 300 Hz high,
 * 155 Hz ultra-high. Every other low nibble is a shared rate ladder.
 */
static float ShimConfig_lis3mdlRateToHz(uint8_t rate)
{
  if ((rate & 0x0FU) == 0x01U)
  {
    switch (rate >> 4)
    {
      case 0:
        return 1000.0f;
      case 1:
        return 560.0f;
      case 2:
        return 300.0f;
      default:
        return 155.0f;
    }
  }

  switch (rate & 0x0FU)
  {
    case 0x00U:
      return 0.625f;
    case 0x02U:
      return 1.25f;
    case 0x04U:
      return 2.5f;
    case 0x06U:
      return 5.0f;
    case 0x08U:
      return 10.0f;
    case 0x0AU:
      return 20.0f;
    case 0x0CU:
      return 40.0f;
    case 0x0EU:
      return 80.0f;
    default:
      return 0.0f;
  }
}

/**
 * Lowest LIS3MDL setting that keeps up with a packet rate.
 *
 * The Java driver's ladder (SensorLIS3MDL.getMagRateFromFreq), which picks an
 * operating mode as well as a rate and is followed here so a correction lands
 * on the value Consensys would have chosen. Its two branches that both yielded
 * ultra-high 155 Hz are collapsed into one.
 *
 * Tops out at low-power 1000 Hz, the fastest the part offers.
 */
static uint8_t ShimConfig_lis3mdlRateForFreq(float freq)
{
  if (freq > 560.0f)
  {
    return LIS3MDL_LP_1kHz;
  }
  else if (freq > 300.0f)
  {
    return LIS3MDL_MP_560Hz;
  }
  else if (freq > 155.0f)
  {
    return LIS3MDL_HP_300Hz;
  }
  else if (freq > 50.0f)
  {
    return LIS3MDL_UHP_155Hz;
  }
  else if (freq > 20.0f)
  {
    return LIS3MDL_UHP_80Hz;
  }
  else if (freq > 10.0f)
  {
    return LIS3MDL_UHP_20Hz;
  }
  return LIS3MDL_LP_10Hz;
}

/**
 * Output rate of an ADXL371 ODR setting, in Hz.
 *
 * The high-g accelerometer's slowest setting is 320 Hz and it has no
 * power-down code, so it is the one part here that cannot be left below a
 * typical packet rate - the check exists for rates above 320 Hz. The stored
 * field is two bits wide, so 5120 Hz is unreachable.
 */
static float ShimConfig_adxl371OdrToHz(uint8_t odr)
{
  switch (odr)
  {
    case ADXL371_ODR_320HZ:
      return 320.0f;
    case ADXL371_ODR_640HZ:
      return 640.0f;
    case ADXL371_ODR_1280HZ:
      return 1280.0f;
    default:
      return 2560.0f;
  }
}

/** Lowest ADXL371 ODR that keeps up, within the two bits the field holds. */
static uint8_t ShimConfig_adxl371OdrForFreq(float freq)
{
  if (freq <= 320.0f)
  {
    return ADXL371_ODR_320HZ;
  }
  else if (freq <= 640.0f)
  {
    return ADXL371_ODR_640HZ;
  }
  else if (freq <= 1280.0f)
  {
    return ADXL371_ODR_1280HZ;
  }
  return ADXL371_ODR_2560HZ;
}
#endif //SHIMMER3R

uint8_t ShimConfig_checkAndCorrectConfig(void)
{
  uint8_t settingCorrected = 0;
  uint8_t i = 0;

  if (storedConfig.chEnGsr
#if defined(SHIMMER3)
      && storedConfig.chEnIntADC1)
#elif defined(SHIMMER3R)
      && storedConfig.chEnIntADC3)
#endif
  {
#if defined(SHIMMER3)
    //they are sharing Shimmer3 adc1, so ban intch1 when gsr is on
    storedConfig.chEnIntADC1 = 0;
#elif defined(SHIMMER3R)
    storedConfig.chEnIntADC3 = 0;
#endif
    settingCorrected = 1;
  }
  if (storedConfig.chEnBridgeAmp
#if defined(SHIMMER3)
      && (storedConfig.chEnIntADC13 || storedConfig.chEnIntADC14))
#elif defined(SHIMMER3R)
      && (storedConfig.chEnIntADC1 || storedConfig.chEnIntADC2))
#endif
  {
#if defined(SHIMMER3)
    //they are sharing adc13 and adc14
    storedConfig.chEnIntADC13 = 0;
    storedConfig.chEnIntADC14 = 0;
#elif defined(SHIMMER3R)
    storedConfig.chEnIntADC1 = 0;
    storedConfig.chEnIntADC2 = 0;
#endif
    settingCorrected = 1;
  }
  if (storedConfig.chEnExg1_24Bit && storedConfig.chEnExg1_16Bit)
  {
    storedConfig.chEnExg1_16Bit = 0;
    settingCorrected = 1;
  }
  if (storedConfig.chEnExg2_24Bit && storedConfig.chEnExg2_16Bit)
  {
    storedConfig.chEnExg2_16Bit = 0;
    settingCorrected = 1;
  }
  if ((storedConfig.chEnExg1_24Bit || storedConfig.chEnExg2_24Bit
          || storedConfig.chEnExg1_16Bit || storedConfig.chEnExg2_16Bit)
#if defined(SHIMMER3)
      && (storedConfig.chEnIntADC1 || storedConfig.chEnIntADC14))
#elif defined(SHIMMER3R)
      && (storedConfig.chEnIntADC3 || storedConfig.chEnIntADC2))
#endif
  {
#if defined(SHIMMER3)
    storedConfig.chEnIntADC1 = 0;
    storedConfig.chEnIntADC14 = 0;
#elif defined(SHIMMER3R)
    storedConfig.chEnIntADC3 = 0;
    storedConfig.chEnIntADC2 = 0;
#endif
    settingCorrected = 1;
  }

  if (storedConfig.chEnSkinTemp || storedConfig.chEnResAmp)
  {
#if defined(SHIMMER3)
    storedConfig.chEnIntADC1 = 1;
#elif defined(SHIMMER3R)
    storedConfig.chEnIntADC3 = 1;
#endif
  }

  if (storedConfig.gsrRange > 4)
  { //never larger than 4
    storedConfig.gsrRange = GSR_AUTORANGE;
    settingCorrected = 1;
  }

  //minimum sync broadcast interval is 54 seconds
  if (storedConfig.syncEnable && storedConfig.btIntervalSecs < SYNC_INT_C)
  {
    storedConfig.btIntervalSecs = SYNC_INT_C;
    settingCorrected = 1;
  }

#if !IS_SUPPORTED_TCXO
  if (storedConfig.tcxo)
  {
    storedConfig.tcxo = 0; /* Disable TCXO */
    settingCorrected = 1;
  }
#endif

#if IS_SUPPORTED_SINGLE_TOUCH
  //the button always works for singletouch mode
  //sync always works for singletouch mode
  if (storedConfig.singleTouchStart
      && (!storedConfig.userButtonEnable || !storedConfig.syncEnable))
  {
    storedConfig.userButtonEnable = 1;
    storedConfig.syncEnable = 1;
    settingCorrected = 1;
  }
#else
  storedConfig.singleTouchStart = 0;
#endif //IS_SUPPORTED_SINGLE_TOUCH

  if (ShimBrd_areADS1292RClockLinesTied() && !(storedConfig.exgADS1292rRegsCh1.config2 & 0x08))
  {
    /* Amend configuration byte 2 of ADS chip 1 to have bit 3 set to 1.
     * This ensures clock lines on ADS chip are correct */
    storedConfig.exgADS1292rRegsCh1.config2 |= 8;
    settingCorrected = 1;
  }

  /* This used to be used to trigger reset of the Bluetooth advertising name
   * and pin code but is no longer needed due to BT driver updates. */
  if (storedConfig.btPinSetup)
  {
    storedConfig.btPinSetup = 0;
    settingCorrected = 1;
  }

#if defined(SHIMMER3)
  if (!ShimBrd_isWrAccelInUseLsm303dlhc() && storedConfig.magRange != 0)
  {
    storedConfig.magRange = 0;
    settingCorrected = 1;
  }
#endif

  ShimSdSync_checkSyncCenterName();

#if defined(SHIMMER3R)
  /* The LSM6DSV's output rate has to be at least the packet rate, or the
   * device packages the same reading several times over with a fresh
   * timestamp on each - so timestamps stay regular, packet loss stays at 0%
   * and any CRC passes, while the signal is a staircase of repeats. Nothing on
   * a host contradicts it, because none of that is wrong.
   *
   * The two are independent InfoMem fields (samplingRateTicks at bytes 0-1,
   * gyroRate at ConfigSetupByte1) and until now nothing here related them.
   * The intent was only ever expressed in the defaults above, in a comment:
   * "LSM6DSV Gyro sampling rate, next highest to 51.2Hz".
   *
   * Corrected here rather than in any one host because this function is where
   * they all converge - a Bluetooth setting write and SET_INFOMEM
   * (Comms/shimmer_bt_uart.c:1716, :1474), the dock UART
   * (Comms/shimmer_dock_usart.c:383), these defaults, and the SD
   * configuration file (SDCard/shimmer_sd_cfg_file.c:905). A host that edits
   * the two fields independently, or one whose own model has the gyro parked
   * at its low-power rate, therefore cannot leave the device in a state that
   * streams repeats.
   *
   * Note this catches power-down with the channels enabled too:
   * ShimConfig_lsm6dsvOdrToHz reports 0 Hz for it, which is below any packet
   * rate.
   *
   * The target is compared against the stored value rather than written
   * unconditionally, so that a packet rate the part cannot reach - anything
   * above its 3840 Hz ceiling - settles instead of re-flagging a correction on
   * every pass, which for the SD path would mean a config rewrite on every
   * read. */
  if ((storedConfig.chEnGyro || storedConfig.chEnLnAccel) && storedConfig.samplingRateTicks > 0)
  {
    float packetRateHz = ShimConfig_getShimmerSamplingFreq();
    if (ShimConfig_lsm6dsvOdrToHz(storedConfig.gyroRate) < packetRateHz)
    {
      uint8_t gyroRateNew = ShimConfig_lsm6dsvOdrForFreq(packetRateHz);
      if (gyroRateNew != storedConfig.gyroRate)
      {
        ShimConfig_gyroRateSet(gyroRateNew);
        settingCorrected = 1;
      }
    }
  }

  /* The same for the wide-range accelerometer, which the driver parks at its
   * lowest rate when the sensor is disabled (setLowPowerAccelWR(true)) exactly
   * as it does the gyroscope. spi.c passes wrAccelRate to lis2dw12_configure
   * verbatim whenever chEnWrAccel is set, so a host that enables the channel
   * without re-deriving the rate leaves the part sampling far below the packet
   * rate - the same staircase, on a second chip.
   *
   * The target is re-read rather than compared against the ladder directly so
   * that a rate the part cannot exceed in its current mode settles instead of
   * being corrected on every pass. */
  if (storedConfig.chEnWrAccel && storedConfig.samplingRateTicks > 0)
  {
    float packetRateHz = ShimConfig_getShimmerSamplingFreq();
    uint8_t isHighPerformance = storedConfig.wrAccelHrMode;
    if (ShimConfig_lis2dw12OdrToHz(storedConfig.wrAccelRate, isHighPerformance) < packetRateHz)
    {
      uint8_t wrAccelRateNew
          = ShimConfig_lis2dw12OdrForFreq(packetRateHz, isHighPerformance);
      if (wrAccelRateNew != storedConfig.wrAccelRate)
      {
        storedConfig.wrAccelRate = wrAccelRateNew;
        settingCorrected = 1;
      }
    }
  }

  /* The magnetometer. i2c.c passes ShimConfig_configByteMagRateGet() to
   * lis2mdl_configure, and the driver drops this part to 10 Hz whenever the
   * sensor is disabled or its low-power flag is set - the same trap again. Its
   * ceiling is 100 Hz, so a packet rate above that settles there rather than
   * being corrected on every pass. */
  if (storedConfig.chEnMag && storedConfig.samplingRateTicks > 0)
  {
    float packetRateHz = ShimConfig_getShimmerSamplingFreq();
    if (ShimConfig_lis2mdlOdrToHz(storedConfig.magRate) < packetRateHz)
    {
      uint8_t magRateNew = ShimConfig_lis2mdlOdrForFreq(packetRateHz);
      if (magRateNew != storedConfig.magRate)
      {
        ShimConfig_configByteMagRateSet(magRateNew);
        settingCorrected = 1;
      }
    }
  }

  /* The alternative magnetometer, via lis3mdl_configure at spi.c. */
  if (storedConfig.chEnAltMag && storedConfig.samplingRateTicks > 0)
  {
    float packetRateHz = ShimConfig_getShimmerSamplingFreq();
    if (ShimConfig_lis3mdlRateToHz(storedConfig.altMagRate) < packetRateHz)
    {
      uint8_t altMagRateNew = ShimConfig_lis3mdlRateForFreq(packetRateHz);
      if (altMagRateNew != storedConfig.altMagRate)
      {
        ShimConfig_configByteAltMagRateSet(altMagRateNew);
        settingCorrected = 1;
      }
    }
  }

  /* The high-g accelerometer, via adxl371_configure at spi.c. Included for
   * completeness rather than because it has been implicated: its slowest
   * setting is 320 Hz and it has no power-down code, so unlike the others it
   * cannot be parked below a typical packet rate. */
  if (storedConfig.chEnAltAccel && storedConfig.samplingRateTicks > 0)
  {
    float packetRateHz = ShimConfig_getShimmerSamplingFreq();
    if (ShimConfig_adxl371OdrToHz(storedConfig.altAccelRate) < packetRateHz)
    {
      uint8_t altAccelRateNew = ShimConfig_adxl371OdrForFreq(packetRateHz);
      if (altAccelRateNew != storedConfig.altAccelRate)
      {
        storedConfig.altAccelRate = altAccelRateNew;
        settingCorrected = 1;
      }
    }
  }
#endif //SHIMMER3R

  uint8_t *macIdBytesPtr = ShimBt_macIdBytesPtrGet();
  for (i = 0; i < 6; i++)
  {
    if (*(macIdBytesPtr + i) != storedConfig.macAddr[i])
    {
      memcpy(&storedConfig.macAddr[0], macIdBytesPtr, 6);
      settingCorrected = 1;
      break;
    }
  }

  return settingCorrected;
}

void ShimConfig_setExgConfigForTestSignal(void)
{
  //square wave test
  storedConfig.exgADS1292rRegsCh1.config1 = 0x04;
  storedConfig.exgADS1292rRegsCh1.config2 = 0xab; //was 0xa3 for rev1
  storedConfig.exgADS1292rRegsCh1.loff = 0x10;
  storedConfig.exgADS1292rRegsCh1.ch1set = 0x05;
  storedConfig.exgADS1292rRegsCh1.ch2set = 0x05;
  storedConfig.exgADS1292rRegsCh1.rldSens = 0x00;
  storedConfig.exgADS1292rRegsCh1.loffSens = 0x00;
  storedConfig.exgADS1292rRegsCh1.loffStat = 0x00;
  storedConfig.exgADS1292rRegsCh1.resp1 = 0x02;
  storedConfig.exgADS1292rRegsCh1.resp2 = 0x01;
  storedConfig.exgADS1292rRegsCh2.config1 = 0x04;
  storedConfig.exgADS1292rRegsCh2.config2 = 0xa3;
  storedConfig.exgADS1292rRegsCh2.loff = 0x10;
  storedConfig.exgADS1292rRegsCh2.ch1set = 0x05;
  storedConfig.exgADS1292rRegsCh2.ch2set = 0x05;
  storedConfig.exgADS1292rRegsCh2.rldSens = 0x00;
  storedConfig.exgADS1292rRegsCh2.loffSens = 0x00;
  storedConfig.exgADS1292rRegsCh2.loffStat = 0x00;
  storedConfig.exgADS1292rRegsCh2.resp1 = 0x02;
  storedConfig.exgADS1292rRegsCh2.resp2 = 0x01;
}

void ShimConfig_setExgConfigForEcg(void)
{
  //ecg
  storedConfig.exgADS1292rRegsCh1.config1 = 0x02;
  storedConfig.exgADS1292rRegsCh1.config2 = 0x80;
  storedConfig.exgADS1292rRegsCh1.loff = 0x10;
  storedConfig.exgADS1292rRegsCh1.ch1set = 0x00;
  storedConfig.exgADS1292rRegsCh1.ch2set = 0x00;
  storedConfig.exgADS1292rRegsCh1.rldSens = 0x00;
  storedConfig.exgADS1292rRegsCh1.loffSens = 0x00;
  storedConfig.exgADS1292rRegsCh1.loffStat = 0x00;
  storedConfig.exgADS1292rRegsCh1.resp1 = 0x00;
  storedConfig.exgADS1292rRegsCh1.resp2 = 0x02;
  storedConfig.exgADS1292rRegsCh2.config1 = 0x02;
  storedConfig.exgADS1292rRegsCh2.config2 = 0x80;
  storedConfig.exgADS1292rRegsCh2.loff = 0x10;
  storedConfig.exgADS1292rRegsCh2.ch1set = 0x00;
  storedConfig.exgADS1292rRegsCh2.ch2set = 0x00;
  storedConfig.exgADS1292rRegsCh2.rldSens = 0x00;
  storedConfig.exgADS1292rRegsCh2.loffSens = 0x00;
  storedConfig.exgADS1292rRegsCh2.loffStat = 0x00;
  storedConfig.exgADS1292rRegsCh2.resp1 = 0x00;
  storedConfig.exgADS1292rRegsCh2.resp2 = 0x02;
}

/* Note samplingRate can be either a freq or a ticks value */
float ShimConfig_freqDiv(float samplingRate)
{
  return (samplingClockFreqGet() / samplingRate);
}

void ShimConfig_checkBtModeFromConfig(void)
{
  if (!shimmerStatus.btConnected)
  {
    gConfigBytes *configBytesPtr = ShimConfig_getStoredConfig();
    shimmerStatus.btSupportEnabled = configBytesPtr->bluetoothDisable ? 0 : 1;

    //Don't allow sync to be enabled if BT is disabled.
    shimmerStatus.sdSyncEnabled
        = (shimmerStatus.btSupportEnabled && configBytesPtr->syncEnable);

    /* Turn off BT if it has been disabled but it's still powered on. Also
     * turn off if BT module is not in the right configuration for SD sync.
     * Leave the SD sync code to turn on/off BT later when required. */
    if ((!shimmerStatus.btSupportEnabled && shimmerStatus.btPowerOn)
        || (shimmerStatus.sdSyncEnabled != shimmerStatus.btInSyncMode)
        || (ShimEeprom_isPresent()
            && ((ShimBrd_doesDeviceSupportBle() && ShimEeprom_isBleEnabled() != ShimBt_isBleCurrentlyEnabled())
                || (ShimBrd_doesDeviceSupportBtClassic()
                    && ShimEeprom_isBtClassicEnabled() != ShimBt_isBtClassicCurrentlyEnabled()))))
    {
      BtStop(0);
    }

    /* Turn on BT if normal LogAndStream mode is turned on */
    if (shimmerStatus.btSupportEnabled && !shimmerStatus.sdSyncEnabled
        && !shimmerStatus.btPowerOn)
    {
      InitialiseBtAfterBoot();
    }
  }
}

#if defined(SHIMMER3R)
uint8_t ShimConfig_isMicrophoneEnabled(void)
{
  return storedConfig.chEnMicrophone;
}
#endif

uint8_t ShimConfig_isGSREnabled(void)
{
  return storedConfig.chEnGsr;
}

uint8_t ShimConfig_isExpansionBoardPwrEnabled(void)
{
  return storedConfig.expansionBoardPower;
}

void ShimConfig_loadSensorConfigAndCalib(void)
{
  //Read storedConfig from flash or generate default config if not available
  ShimConfig_readRam();

  //Check the config from RAM for faults and write back if change needed
  if (ShimConfig_checkAndCorrectConfig())
  {
    LogAndStream_infomemUpdate();
  }

  ShimCalib_init();
  ShimCalib_initFromConfigBytesAll();

  //Read storedConfig from SD cfg file or update it from RAM if RAM is newer
  if (!shimmerStatus.docked && LogAndStream_checkSdInSlot())
  { //sd card ready to access
    if (!shimmerStatus.sdPowerOn)
    {
      //Hits here when undocked
      Board_setSdPower(1);
    }
    if (ShimConfig_getFlagWriteCfgToSd())
    { //info > sdcard
      ShimSdCfgFile_generate();
      ShimConfig_setFlagWriteCfgToSd(0, 1);
      if (!ShimSdDataFile_isFileStatusOk())
      {
        shimmerStatus.sdlogReady = 0;
        shimmerStatus.sdBadFile = 1;
      }
    }
    else
    {
      //Hits here when undocked
      ShimSdCfgFile_readSdConfiguration();
    }

    //If the calib dump file is available, read it into RAM. Else, generate it.
    if (ShimCalib_file2Ram())
    {
      //fail, i.e. no such file. use current DumpRam to generate a file
      ShimCalib_ram2File();
    }
  }

  //TODO should this only be called if calib dump file is available?
  ShimCalib_calibDumpToConfigBytesAndSdHeaderAll(1);
}

void ShimConfig_createBlankConfigBytes(void)
{
  memset(&storedConfig.rawBytes[0], 0, STOREDCONFIG_SIZE);

  /* Make all calibration bytes invalid (i.e., 0xFF) */
  memset(storedConfig.lnAccelCalib.rawBytes, 0xFF,
      sizeof(storedConfig.lnAccelCalib.rawBytes));
  memset(storedConfig.gyroCalib.rawBytes, 0xFF, sizeof(storedConfig.gyroCalib.rawBytes));
  memset(storedConfig.magCalib.rawBytes, 0xFF, sizeof(storedConfig.magCalib.rawBytes));
  memset(storedConfig.wrAccelCalib.rawBytes, 0xFF,
      sizeof(storedConfig.wrAccelCalib.rawBytes));
  memset(storedConfig.altAccelCalib.rawBytes, 0xFF,
      sizeof(storedConfig.altAccelCalib.rawBytes));
  memset(storedConfig.altMagCalib.rawBytes, 0xFF,
      sizeof(storedConfig.altMagCalib.rawBytes));

  /* Copy MAC ID directly from BT module */
  memcpy(&storedConfig.macAddr[0], ShimBt_macIdBytesPtrGet(), 6);

  /* Reset unused bytes */
  memset(&storedConfig.rawBytes[NV_BT_SET_PIN + 1], 0xFF, 24);

  /* Reset node addresses */
  memset(&storedConfig.rawBytes[NV_CENTER], 0xFF, 128);

  return;
}

uint8_t ShimConfig_areConfigBytesValid(void)
{
  //return memcmp(all0xff, &storedConfig.rawBytes[0], 6)
  uint8_t i;
  for (i = 0; i < 6; i++)
  {
    if (storedConfig.macAddr[i] != 0xFF)
    {
      return 1;
    }
  }
  return 0;
}

void ShimConfig_parseShimmerNameFromConfigBytes(void)
{
  uint8_t i;
  memset(&shimmerName[0], 0x00, sizeof(shimmerName));

  for (i = 0; (i < MAX_CHARS - 1) && isprint((uint8_t) storedConfig.shimmerName[i]); i++)
    ;
  if (i == 0)
  {
    ShimConfig_setDefaultShimmerName();
    i = 12;
  }
  memcpy((char *) shimmerName, &(storedConfig.shimmerName[0]), i);
}

void ShimConfig_parseExpIdNameFromConfigBytes(void)
{
  uint8_t i;
  memset(&expIdName[0], 0x00, sizeof(expIdName));

  for (i = 0; (i < MAX_CHARS - 1) && (isprint((uint8_t) storedConfig.expIdName[i])); i++)
    ;
  if (i == 0)
  {
    ShimConfig_setDefaultTrialId();
    i = 12;
  }
  memcpy((char *) expIdName, &(storedConfig.expIdName[0]), i);
}

void ShimConfig_parseCfgTimeFromConfigBytes(void)
{
  memset(&configTimeText[0], 0x00, sizeof(configTimeText));

  uint32_t configTime = ShimConfig_configTimeGet();
  /* Convert configTime to string */
  if (configTime > 0)
  {
    ShimUtil_ItoaNo0((uint64_t) configTime, configTimeText, sizeof(configTimeText));
  }
  else
  {
    strcpy((char *) configTimeText, "0");
  }
}

char *ShimConfig_shimmerNameParseToTxtAndPtrGet(void)
{
  ShimConfig_parseShimmerNameFromConfigBytes();
  return &shimmerName[0];
}

char *ShimConfig_expIdParseToTxtAndPtrGet(void)
{
  ShimConfig_parseExpIdNameFromConfigBytes();
  return &expIdName[0];
}

char *ShimConfig_configTimeParseToTxtAndPtrGet(void)
{
  ShimConfig_parseCfgTimeFromConfigBytes();
  return &configTimeText[0];
}

void ShimConfig_shimmerNameSet(uint8_t *strPtr, uint8_t strLen)
{
  uint8_t lenToCpy = (strLen < sizeof(storedConfig.shimmerName)) ?
      strLen :
      sizeof(storedConfig.shimmerName);
  memset(&storedConfig.shimmerName[0], 0, sizeof(storedConfig.shimmerName));
  memcpy(&storedConfig.shimmerName[0], strPtr, lenToCpy);
}

void ShimConfig_expIdSet(uint8_t *strPtr, uint8_t strLen)
{
  uint8_t lenToCpy = (strLen < sizeof(storedConfig.expIdName)) ?
      strLen :
      sizeof(storedConfig.expIdName);
  memset(&storedConfig.expIdName[0], 0, sizeof(storedConfig.expIdName));
  memcpy(&storedConfig.expIdName[0], strPtr, lenToCpy);
}

void ShimConfig_configTimeSetFromStr(uint8_t *strPtr, uint8_t strLen)
{
  uint32_t config_time;
  char configTimeTextTemp[UINT32_LEN] = { 0 };
  uint8_t lenToCpy = strLen < (UINT32_LEN - 1) ? strLen : (UINT32_LEN - 1);
  memcpy(&configTimeTextTemp[0], strPtr, lenToCpy);

  config_time = atol((char *) &configTimeTextTemp[0]);

  ShimConfig_configTimeSet(config_time);
}

void ShimConfig_experimentLengthEstimatedInSecSet(uint16_t value)
{
  storedConfig.experimentLengthEstimatedInSecMsb = (value >> 8) & 0xFF;
  storedConfig.experimentLengthEstimatedInSecLsb = value & 0xFF;
}

uint16_t ShimConfig_experimentLengthEstimatedInSecGet(void)
{
  return storedConfig.experimentLengthEstimatedInSecMsb << 8
      | storedConfig.experimentLengthEstimatedInSecLsb;
}

void ShimConfig_experimentLengthMaxInMinutesSet(uint16_t value)
{
  storedConfig.experimentLengthMaxInMinutesMsb = (value >> 8) & 0xFF;
  storedConfig.experimentLengthMaxInMinutesLsb = value & 0xFF;
}

uint16_t ShimConfig_experimentLengthMaxInMinutesGet(void)
{
  return storedConfig.experimentLengthMaxInMinutesMsb << 8
      | storedConfig.experimentLengthMaxInMinutesLsb;
}
