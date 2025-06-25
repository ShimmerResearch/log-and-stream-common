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
#include <string.h>

#include <log_and_stream_externs.h>
#include <log_and_stream_includes.h>
#if defined(SHIMMER3R)
#include "shimmer_definitions.h"
#endif

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

#if defined(SHIMMER3)
  ShimSdHead_config2SdHead();
  ShimConfig_configBytesToNames();
#endif

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
  storedConfig = ShimConfig_createBlankConfigBytes();

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
  ShimConfig_wrAccelLpModeSet(&storedConfig, 0);
  /* MPU9X50/ICM20948 sampling rate of 8kHz/(155+1), i.e. 51.282Hz */
  ShimConfig_gyroRateSet(&storedConfig, 0x9B);
  /* LSM303 Mag 75Hz, +/-1.3 Gauss, MPU9150 Gyro +/-500 degrees per second */
  storedConfig.magRange = LSM303DLHC_MAG_1_3G;
  ShimConfig_configByteMagRateSet(&storedConfig, LSM303DLHC_MAG_75HZ);
  ShimConfig_gyroRangeSet(&storedConfig, MPU9X50_GYRO_500DPS);
  /* MPU9X50/ICM20948 Accel +/-2G */
  storedConfig.altAccelRange = ACCEL_2G;
  /* BMP pressure oversampling ratio 1 */
  ShimConfig_configBytePressureOversamplingRatioSet(&storedConfig, BMPX80_OSS_1);
#elif defined(SHIMMER3R)
  /* LIS2DW12 Accel 100Hz, +/-2G, Low Power and High Resolution modes off */
  storedConfig.wrAccelRate = LIS2DW12_XL_ODR_100Hz;
  storedConfig.wrAccelRange = LIS2DW12_2g;
  ShimConfig_wrAccelModeSet(&storedConfig, LIS2DW12_HIGH_PERFORMANCE);
  /* LSM6DSV Gyro sampling rate, next highest to 51.2Hz */
  ShimConfig_gyroRateSet(&storedConfig, LSM6DSV_ODR_AT_60Hz);
  /* LIS2MDL Mag 100Hz */
  ShimConfig_configByteMagRateSet(&storedConfig, LIS2MDL_ODR_100Hz);
  /* LIS3MDL Mag 80Hz, +/-4 Gauss */
  storedConfig.altMagRange = LIS3MDL_4_GAUSS;
  ShimConfig_configByteAltMagRateSet(&storedConfig, LIS3MDL_UHP_80Hz);
  /* LSM6DSV Gyro +/-500 degrees per second */
  ShimConfig_gyroRangeSet(&storedConfig, LSM6DSV_500dps);
  storedConfig.lnAccelRange = LSM6DSV_2g;
  ShimConfig_configBytePressureOversamplingRatioSet(&storedConfig, BMP3_NO_OVERSAMPLING);
#endif
  /* GSR auto range */
  storedConfig.gsrRange = GSR_AUTORANGE;

  /* EXP_RESET_N pin set low */
  storedConfig.expansionBoardPower = 0;

  //set all ExG registers to their reset values
  //setExgConfigForTestSignal(&storedConfig);
  ShimConfig_setExgConfigForEcg(&storedConfig);

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
  ShimConfig_configTimeSet(0);

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

  ShimConfig_checkAndCorrectConfig(&storedConfig);

  /* Write RAM contents to Infomem */
#if defined(SHIMMER3)
  InfoMem_write(0, &storedConfig.rawBytes[0], NV_NUM_SETTINGS_BYTES);
  InfoMem_write(NV_SENSORS3, &storedConfig.rawBytes[NV_SENSORS3], 5);
  InfoMem_write(NV_DERIVED_CHANNELS_3,
      &ShimConfig_getStoredConfig()->rawBytes[NV_DERIVED_CHANNELS_3], 5);
  InfoMem_write(NV_SD_SHIMMER_NAME, &storedConfig.rawBytes[NV_SD_SHIMMER_NAME], NV_NUM_SD_BYTES);
  InfoMem_write(NV_MAC_ADDRESS + 7, &storedConfig.rawBytes[NV_MAC_ADDRESS + 7], 153); //25+128
#elif defined(SHIMMER3R)
  InfoMem_update();
#endif
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

void ShimConfig_configTimeSet(uint32_t time)
{
  // Config time is stored in MSB order in the config bytes
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

uint8_t ShimConfig_getSdCfgFlag(void)
{
  return (!storedConfig.sdCfgFlag && storedConfig.infoSdcfg);
}

void ShimConfig_setSdCfgFlag(uint8_t flag)
{
  if (flag)
  {
    storedConfig.sdCfgFlag = 0;
  }
  storedConfig.infoSdcfg = flag;
  InfoMem_write(NV_SD_CONFIG_DELAY_FLAG,
      &storedConfig.rawBytes[NV_SD_CONFIG_DELAY_FLAG], 1);
}

uint8_t ShimConfig_getCalibFlag()
{
  return (!storedConfig.sdCfgFlag && storedConfig.infoCalib);
}

void ShimConfig_setCalibFlag(uint8_t flag)
{
  if (flag)
  {
    storedConfig.sdCfgFlag = 0;
  }
  storedConfig.infoCalib = flag;
  InfoMem_write(NV_SD_CONFIG_DELAY_FLAG,
      &storedConfig.rawBytes[NV_SD_CONFIG_DELAY_FLAG], 1);
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

void ShimConfig_gyroRangeSet(gConfigBytes *storedConfigPtr, uint8_t value)
{
#if defined(SHIMMER3)
  value = (value <= MPU9X50_GYRO_2000DPS) ? value : MPU9X50_GYRO_500DPS;
#elif defined(SHIMMER3R)
  value = (value <= (LSM6DSV_2000dps + 1)) ? value : LSM6DSV_500dps;
#endif
  storedConfigPtr->gyroRangeLsb = value & 0x03;
#if defined(SHIMMER3R)
  storedConfigPtr->gyroRangeMsb = (value >> 2) & 0x01;
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

void ShimConfig_gyroRateSet(gConfigBytes *storedConfigPtr, uint8_t value)
{
#if defined(SHIMMER3R)
  value = (value < LSM6DSV_ODR_AT_7680Hz) ? value : LSM6DSV_ODR_AT_60Hz;
#endif
  storedConfigPtr->gyroRate = value;
}

void ShimConfig_wrAccelLpModeSet(gConfigBytes *storedConfigPtr, uint8_t value)
{
#if defined(SHIMMER3)
  value = (value == 1) ? 1 : 0;
#elif defined(SHIMMER3R)
  value = (value <= 3) ? value : 0;
#endif
  storedConfigPtr->wrAccelLpModeLsb = value & 0x01;
#if defined(SHIMMER3R)
  storedConfigPtr->wrAccelLpModeMsb = (value >> 1) & 0x01;
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
void ShimConfig_wrAccelModeSet(gConfigBytes *storedConfigPtr, lis2dw12_mode_t value)
{
  storedConfigPtr->wrAccelHrMode = (value >> 2) & 0x01;
  ShimConfig_wrAccelLpModeSet(storedConfigPtr, value & 0x03);
}

lis2dw12_mode_t ShimConfig_wrAccelModeGet(void)
{
  lis2dw12_mode_t wrAccelMode = (lis2dw12_mode_t) ((storedConfig.wrAccelHrMode << 2)
      | ShimConfig_wrAccelLpModeGet());
  return wrAccelMode;
}
#endif

void ShimConfig_configBytePressureOversamplingRatioSet(gConfigBytes *storedConfigPtr, uint8_t value)
{
#if defined(SHIMMER3)
  value = (value <= BMPX80_OSS_8) ? (value & 0x03) : BMPX80_OSS_1;
#elif defined(SHIMMER3R)
  value = (value <= BMP3_OVERSAMPLING_32X) ? value : BMP3_NO_OVERSAMPLING;
#endif
  storedConfigPtr->pressureOversamplingRatioLsb = value & 0x03;
#if defined(SHIMMER3R)
  storedConfigPtr->pressureOversamplingRatioMsb = (value >> 2) & 0x01;
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

void ShimConfig_configByteMagRateSet(gConfigBytes *storedConfigPtr, uint8_t value)
{
#if defined(SHIMMER3)
  value = (value < LSM303DLHC_MAG_220HZ) ? value : LSM303DLHC_MAG_75HZ;
#else
  value = (value <= LIS2MDL_ODR_100Hz) ? value : LIS2MDL_ODR_100Hz;
#endif
  storedConfigPtr->magRate = value;
}

uint8_t ShimConfig_configByteMagRateGet(void)
{
  return storedConfig.magRate;
}

void ShimConfig_configByteAltMagRateSet(gConfigBytes *storedConfigPtr, uint8_t value)
{
#if defined(SHIMMER3)
  value = 0; //not used
#elif defined(SHIMMER3R)
  value = value <= (LIS3MDL_UHP_80Hz) ? value : LIS3MDL_UHP_80Hz;
#endif
  storedConfigPtr->altMagRate = value;
}

uint8_t ShimConfig_configByteAltMagRateGet(void)
{
  return storedConfig.altMagRate;
}

uint8_t ShimConfig_checkAndCorrectConfig(gConfigBytes *storedConfigPtr)
{
  uint8_t settingCorrected = 0;
  uint8_t i = 0;

  if (storedConfigPtr->chEnGsr
#if defined(SHIMMER3)
      && storedConfigPtr->chEnIntADC1)
#elif defined(SHIMMER3R)
      && storedConfigPtr->chEnIntADC3)
#endif
  {
#if defined(SHIMMER3)
    //they are sharing Shimmer3 adc1, so ban intch1 when gsr is on
    storedConfigPtr->chEnIntADC1 = 0;
#elif defined(SHIMMER3R)
    storedConfigPtr->chEnIntADC3 = 0;
#endif
    settingCorrected = 1;
  }
  if (storedConfigPtr->chEnBridgeAmp
#if defined(SHIMMER3)
      && (storedConfigPtr->chEnIntADC13 || storedConfigPtr->chEnIntADC14))
#elif defined(SHIMMER3R)
      && (storedConfigPtr->chEnIntADC1 || storedConfigPtr->chEnIntADC2))
#endif
  {
#if defined(SHIMMER3)
    //they are sharing adc13 and adc14
    storedConfigPtr->chEnIntADC13 = 0;
    storedConfigPtr->chEnIntADC14 = 0;
#elif defined(SHIMMER3R)
    storedConfigPtr->chEnIntADC1 = 0;
    storedConfigPtr->chEnIntADC2 = 0;
#endif
    settingCorrected = 1;
  }
  if (storedConfigPtr->chEnExg1_24Bit && storedConfigPtr->chEnExg1_16Bit)
  {
    storedConfigPtr->chEnExg1_16Bit = 0;
    settingCorrected = 1;
  }
  if (storedConfigPtr->chEnExg2_24Bit && storedConfigPtr->chEnExg2_16Bit)
  {
    storedConfigPtr->chEnExg2_16Bit = 0;
    settingCorrected = 1;
  }
  if ((storedConfigPtr->chEnExg1_24Bit || storedConfigPtr->chEnExg2_24Bit
          || storedConfigPtr->chEnExg1_16Bit || storedConfigPtr->chEnExg2_16Bit)
#if defined(SHIMMER3)
      && (storedConfigPtr->chEnIntADC1 || storedConfigPtr->chEnIntADC14))
#elif defined(SHIMMER3R)
      && (storedConfigPtr->chEnIntADC3 || storedConfigPtr->chEnIntADC2))
#endif
  {
#if defined(SHIMMER3)
    storedConfigPtr->chEnIntADC1 = 0;
    storedConfigPtr->chEnIntADC14 = 0;
#elif defined(SHIMMER3R)
    storedConfigPtr->chEnIntADC3 = 0;
    storedConfigPtr->chEnIntADC2 = 0;
#endif
    settingCorrected = 1;
  }

  if (storedConfigPtr->gsrRange > 4)
  { //never larger than 4
    storedConfigPtr->gsrRange = GSR_AUTORANGE;
    settingCorrected = 1;
  }

  //minimum sync broadcast interval is 54 seconds
  if (storedConfigPtr->syncEnable && storedConfigPtr->btIntervalSecs < SYNC_INT_C)
  {
    storedConfigPtr->btIntervalSecs = SYNC_INT_C;
    settingCorrected = 1;
  }

#if !IS_SUPPORTED_TCXO
  if (storedConfigPtr->tcxo)
  {
    storedConfigPtr->tcxo = 0; /* Disable TCXO */
    settingCorrected = 1;
  }
#endif

  //the button always works for singletouch mode
  //sync always works for singletouch mode
  if (storedConfigPtr->singleTouchStart
      && (!storedConfigPtr->userButtonEnable || !storedConfigPtr->syncEnable))
  {
    storedConfigPtr->userButtonEnable = 1;
    storedConfigPtr->syncEnable = 1;
    settingCorrected = 1;
  }

  if (ShimBrd_areADS1292RClockLinesTied()
      && !(storedConfigPtr->exgADS1292rRegsCh1.config2 & 0x08))
  {
    /* Amend configuration byte 2 of ADS chip 1 to have bit 3 set to 1.
     * This ensures clock lines on ADS chip are correct */
    storedConfigPtr->exgADS1292rRegsCh1.config2 |= 8;
    settingCorrected = 1;
  }

  /* This used to be used to trigger reset of the Bluetooth advertising name
   * and pin code but is no longer needed due to BT driver updates. */
  if (storedConfigPtr->btPinSetup)
  {
    storedConfigPtr->btPinSetup = 0;
    settingCorrected = 1;
  }

#if defined(SHIMMER3)
  if (!ShimBrd_isWrAccelInUseLsm303dlhc() && storedConfigPtr->magRange != 0)
  {
    storedConfigPtr->magRange = 0;
    settingCorrected = 1;
  }
#endif

  ShimSdSync_checkSyncCenterName();

  uint8_t *macIdBytesPtr = ShimBt_macIdBytesPtrGet();
  for (i = 0; i < 6; i++)
  {
    if (*(macIdBytesPtr + i) != storedConfigPtr->macAddr[i])
    {
      memcpy(&storedConfigPtr->macAddr[0], macIdBytesPtr, 6);
      settingCorrected = 1;
      break;
    }
  }

  return settingCorrected;
}

void ShimConfig_setExgConfigForTestSignal(gConfigBytes *storedConfigPtr)
{
  //square wave test
  storedConfigPtr->exgADS1292rRegsCh1.config1 = 0x04;
  storedConfigPtr->exgADS1292rRegsCh1.config2 = 0xab; //was 0xa3 for rev1
  storedConfigPtr->exgADS1292rRegsCh1.loff = 0x10;
  storedConfigPtr->exgADS1292rRegsCh1.ch1set = 0x05;
  storedConfigPtr->exgADS1292rRegsCh1.ch2set = 0x05;
  storedConfigPtr->exgADS1292rRegsCh1.rldSens = 0x00;
  storedConfigPtr->exgADS1292rRegsCh1.loffSens = 0x00;
  storedConfigPtr->exgADS1292rRegsCh1.loffStat = 0x00;
  storedConfigPtr->exgADS1292rRegsCh1.resp1 = 0x02;
  storedConfigPtr->exgADS1292rRegsCh1.resp2 = 0x01;
  storedConfigPtr->exgADS1292rRegsCh2.config1 = 0x04;
  storedConfigPtr->exgADS1292rRegsCh2.config2 = 0xa3;
  storedConfigPtr->exgADS1292rRegsCh2.loff = 0x10;
  storedConfigPtr->exgADS1292rRegsCh2.ch1set = 0x05;
  storedConfigPtr->exgADS1292rRegsCh2.ch2set = 0x05;
  storedConfigPtr->exgADS1292rRegsCh2.rldSens = 0x00;
  storedConfigPtr->exgADS1292rRegsCh2.loffSens = 0x00;
  storedConfigPtr->exgADS1292rRegsCh2.loffStat = 0x00;
  storedConfigPtr->exgADS1292rRegsCh2.resp1 = 0x02;
  storedConfigPtr->exgADS1292rRegsCh2.resp2 = 0x01;
}

void ShimConfig_setExgConfigForEcg(gConfigBytes *storedConfigPtr)
{
  //ecg
  storedConfigPtr->exgADS1292rRegsCh1.config1 = 0x02;
  storedConfigPtr->exgADS1292rRegsCh1.config2 = 0x80;
  storedConfigPtr->exgADS1292rRegsCh1.loff = 0x10;
  storedConfigPtr->exgADS1292rRegsCh1.ch1set = 0x00;
  storedConfigPtr->exgADS1292rRegsCh1.ch2set = 0x00;
  storedConfigPtr->exgADS1292rRegsCh1.rldSens = 0x00;
  storedConfigPtr->exgADS1292rRegsCh1.loffSens = 0x00;
  storedConfigPtr->exgADS1292rRegsCh1.loffStat = 0x00;
  storedConfigPtr->exgADS1292rRegsCh1.resp1 = 0x00;
  storedConfigPtr->exgADS1292rRegsCh1.resp2 = 0x02;
  storedConfigPtr->exgADS1292rRegsCh2.config1 = 0x02;
  storedConfigPtr->exgADS1292rRegsCh2.config2 = 0x80;
  storedConfigPtr->exgADS1292rRegsCh2.loff = 0x10;
  storedConfigPtr->exgADS1292rRegsCh2.ch1set = 0x00;
  storedConfigPtr->exgADS1292rRegsCh2.ch2set = 0x00;
  storedConfigPtr->exgADS1292rRegsCh2.rldSens = 0x00;
  storedConfigPtr->exgADS1292rRegsCh2.loffSens = 0x00;
  storedConfigPtr->exgADS1292rRegsCh2.loffStat = 0x00;
  storedConfigPtr->exgADS1292rRegsCh2.resp1 = 0x00;
  storedConfigPtr->exgADS1292rRegsCh2.resp2 = 0x02;
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
        || (shimmerStatus.sdSyncEnabled != shimmerStatus.btInSyncMode))
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
  ShimCalib_init();
  ShimCalib_initFromConfigBytesAll();

  if (!shimmerStatus.docked && CheckSdInslot())
  { //sd card ready to access
    if (!shimmerStatus.sdPowerOn)
    {
      //Hits here when undocked
      Board_setSdPower(1);
    }
    if (ShimConfig_getSdCfgFlag())
    { //info > sdcard
      ShimConfig_readRam();
      ShimSdCfgFile_generate();
      ShimConfig_setSdCfgFlag(0);
      if (!ShimSd_isFileStatusOk())
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

    if (ShimCalib_file2Ram())
    {
      //fail, i.e. no such file. use current DumpRam to generate a file
      ShimCalib_ram2File();
      ShimConfig_readRam();
    }
  }
  else
  { //sd card not available
    ShimConfig_readRam();
    //CalibFromInfoAll();
  }

  ShimCalib_calibDumpToConfigBytesAndSdHeaderAll();
}

gConfigBytes ShimConfig_createBlankConfigBytes(void)
{
  gConfigBytes storedConfigNew;

  memset(&storedConfigNew.rawBytes[NV_SAMPLING_RATE], 0, STOREDCONFIG_SIZE);

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
  memcpy(&storedConfigNew.macAddr[0], ShimBt_macIdBytesPtrGet(), 6);

  /* Reset unused bytes */
  memset(&storedConfigNew.rawBytes[NV_BT_SET_PIN + 1], 0xFF, 24);

  /* Reset node addresses */
  memset(&storedConfigNew.rawBytes[NV_CENTER], 0xFF, 128);

  memset(storedConfig.rawBytes, 0x00, sizeof(storedConfig.rawBytes));

  return storedConfigNew;
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
  char configTimeTextTemp[UINT32_LEN] = {0};
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
