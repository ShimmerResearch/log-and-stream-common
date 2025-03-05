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

#include <math.h>
#include <string.h>

#include <Boards/shimmer_boards.h>
#include <SDCard/shimmer_sd.h>
#include <SDCard/shimmer_sd_header.h>
#include <SDSync/shimmer_sd_sync.h>
#include <log_and_stream_externs.h>
#if defined(SHIMMER3R)
#include "shimmer_definitions.h"
#endif

uint8_t btMacAscii[14], btMacHex[6];
static gConfigBytes storedConfig;
uint8_t calibRamFlag = 0;

uint32_t maxLen, maxLenCnt;

void ShimConfig_reset(void)
{
  calibRamFlag = 0;

  memset(storedConfig.rawBytes, 0xff, NV_NUM_RWMEM_BYTES);
  storedConfig.rawBytes[NV_SD_SHIMMER_NAME] = '\0';
  storedConfig.rawBytes[NV_SD_EXP_ID_NAME] = '\0';

  memset(btMacAscii, 0x00, sizeof(btMacAscii));
  memset(btMacHex, 0x00, sizeof(btMacHex));
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
  ShimConfig_storedConfigSet(btMacHex, NV_MAC_ADDRESS, 6);

#if defined(SHIMMER3)
  ShimSdHead_config2SdHead();
  SD_infomem2Names();
#endif

  /* Check BT module configuration after sensor configuration read from
   * infomem to see if it is in the correct state (i.e., BT on vs. BT off vs.
   * SD Sync) */
  checkBtModeConfig();
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

/*
 * btMacAscii: Set(), Get()
 */

void ShimConfig_btMacAsciiSet(char *buf)
{
  memcpy(btMacAscii, buf, 12);
  btMacAscii[12] = 0;
}

void ShimConfig_btMacAsciiGet(uint8_t *buf)
{
  memcpy(buf, btMacAscii, 12);
}

uint8_t *ShimConfig_getMacIdStrPtr(void)
{
  return &btMacAscii[0];
}

/*
 * btMacHex: Set(), Get()
 */

void ShimConfig_btMacHexSet(uint8_t *buf)
{
  memcpy(btMacHex, buf, 6);
}

void ShimConfig_btMacHexGet(uint8_t *buf)
{
  memcpy(buf, btMacHex, 6);
}

void ShimConfig_setDefaultConfig(void)
{
  memset(storedConfig.rawBytes, 0x00, sizeof(storedConfig.rawBytes));

  memset(storedConfig.lnAccelCalib.rawBytes, 0xFF,
      sizeof(storedConfig.lnAccelCalib.rawBytes));
  memset(storedConfig.gyroCalib.rawBytes, 0xFF, sizeof(storedConfig.gyroCalib.rawBytes));
  memset(storedConfig.magCalib.rawBytes, 0xFF, sizeof(storedConfig.magCalib.rawBytes));
  memset(storedConfig.wrAccelCalib.rawBytes, 0xFF,
      sizeof(storedConfig.wrAccelCalib.rawBytes));
#if defined(SHIMMER3R)
  memset(storedConfig.altAccelCalib.rawBytes, 0xFF,
      sizeof(storedConfig.altAccelCalib.rawBytes));
  memset(storedConfig.altMagCalib.rawBytes, 0xFF,
      sizeof(storedConfig.altMagCalib.rawBytes));
#endif
  memset(&storedConfig.rawBytes[NV_BT_SET_PIN], 0xFF, 25);
  memset(&storedConfig.rawBytes[NV_NODE0], 0xFF, 128);

  ShimConfig_btMacHexGet(storedConfig.macAddr);

  storedConfig.samplingRateTicks = FreqDiv(51.2); //51.2Hz
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
  storedConfig.magRateLsb = LSM303DLHC_MAG_75HZ;
  ShimConfig_gyroRangeSet(&storedConfig, MPU9X50_GYRO_500DPS);
  /* MPU9X50/ICM20948 Accel +/-2G */
  storedConfig.altAccelRange = ACCEL_2G;
  /* BMP pressure oversampling ratio 1 */
  ShimConfig_configBytePressureOversamplingRatioSet(&storedConfig, BMPX80_OSS_1);
#elif defined(SHIMMER3R)
  /* LIS2DW12 Accel 100Hz, +/-2G, Low Power and High Resolution modes off */
  storedConfig.wrAccelRate = LIS2DW12_XL_ODR_100Hz;
  storedConfig.wrAccelRange = LIS2DW12_2g;
  /* LSM6DSV Gyro sampling rate, next highest to 51.2Hz */
  ShimConfig_gyroRateSet(&storedConfig, LSM6DSV_ODR_AT_60Hz);
  /* LIS3MDL Mag 75Hz, +/-4 Gauss, MPU9150 Gyro +/-500 degrees per second */
  storedConfig.magRange = LIS3MDL_4_GAUSS;
  ShimConfig_configByteMagRateSet(&storedConfig, LIS3MDL_UHP_80Hz);
  /* LSM6DSV Gyro +/-500 degrees per second */
  ShimConfig_gyroRangeSet(&storedConfig, LSM6DSV_500dps);
  storedConfig.lnAccelRange = LSM6DSV_2g;
  ShimConfig_configBytePressureOversamplingRatioSet(&storedConfig, BMP3_NO_OVERSAMPLING);
  set_config_byte_wr_accel_mode(&storedConfig, LIS2DW12_HIGH_PERFORMANCE);
  storedConfig.altMagRate = LIS2MDL_ODR_100Hz;
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
  setDefaultShimmerName();
  //exp_id
  setDefaultTrialId();
  storedConfig.configTime = 0;

  storedConfig.myTrialID = 0;
  storedConfig.numberOfShimmers = 0;
  storedConfig.userButtonEnable = 1;
  storedConfig.rtcErrorEnable = 1;
  storedConfig.sdErrorEnable = 1;
  storedConfig.btIntervalSecs = 54;

  storedConfig.experimentLengthMaxInMinutes = 0;

  storedConfig.experimentLengthEstimatedInSec = 1;
  setSyncEstExpLen((uint32_t) storedConfig.experimentLengthEstimatedInSec);

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

void setDefaultShimmerName(void)
{
  strcpy(&storedConfig.shimmerName[0], "Shimmer_XXXX");
  memcpy(&storedConfig.shimmerName[8], btMacAscii + 8, 4);
}

void setDefaultTrialId(void)
{
  memcpy(&storedConfig.expIdName[0], "DefaultTrial", 12);
}

uint8_t GetSdCfgFlag(void)
{
  return (!storedConfig.sdCfgFlag && storedConfig.infoSdcfg);
}

void SetSdCfgFlag(uint8_t flag)
{
  if (flag)
  {
    storedConfig.sdCfgFlag = 0;
  }
  storedConfig.infoSdcfg = flag;
  InfoMem_write(NV_SD_CONFIG_DELAY_FLAG,
      &storedConfig.rawBytes[NV_SD_CONFIG_DELAY_FLAG], 1);
}

uint8_t GetCalibFlag()
{
  return (!storedConfig.sdCfgFlag && storedConfig.infoCalib);
}

void SetCalibFlag(uint8_t flag)
{
  if (flag)
  {
    storedConfig.sdCfgFlag = 0;
  }
  storedConfig.infoCalib = flag;
  InfoMem_write(NV_SD_CONFIG_DELAY_FLAG,
      &storedConfig.rawBytes[NV_SD_CONFIG_DELAY_FLAG], 1);
}

uint8_t GetRamCalibFlag(void)
{
  return calibRamFlag;
}

void SetRamCalibFlag(uint8_t flag)
{
  //flag == 1: Ram>File, ShimmerCalib_ram2File()
  //        0: File>Ram, ShimmerCalib_file2Ram()
  calibRamFlag = flag;
}

float get_shimmer_sampling_freq(void)
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
#if defined(SHIMMER3)
#elif defined(SHIMMER3R)
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
void set_config_byte_wr_accel_mode(gConfigBytes *storedConfigPtr, lis2dw12_mode_t value)
{
  storedConfigPtr->wrAccelHrMode = (value >> 2) & 0x01;
  ShimConfig_wrAccelLpModeSet(storedConfigPtr, value & 0x03);
}

lis2dw12_mode_t get_config_byte_wr_accel_mode(void)
{
  lis2dw12_mode_t wrAccelMode = (lis2dw12_mode_t) ((storedConfig.wrAccelHrMode << 2)
      | ShimConfig_wrAccelLpModeGet());
  return wrAccelMode;
}
#endif

void ShimConfig_configBytePressureOversamplingRatioSet(gConfigBytes *storedConfigPtr, uint8_t value)
{
#if defined(SHIMMER3)
  value = (value < 4) ? (value & 0x03) : BMPX80_OSS_1;
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
  value = (value < 4) ? (value & 0x03) : BMPX80_OSS_1;
#elif defined(SHIMMER3R)
  value = value <= (LIS3MDL_UHP_80Hz) ? value : LIS3MDL_UHP_80Hz;
#endif
  storedConfigPtr->magRateLsb = value & 0x07;
#if defined(SHIMMER3R)
  storedConfigPtr->magRateMsb = (value >> 3) & 0x07;
#endif
}

uint8_t ShimConfig_configByteMagRateGet(void)
{
#if defined(SHIMMER3)
  return storedConfig.magRateLsb;
#elif defined(SHIMMER3R)
  return (storedConfig.magRateMsb << 3) | storedConfig.magRateLsb;
#endif
}

uint8_t ShimConfig_checkAndCorrectConfig(gConfigBytes *storedConfigPtr)
{
  uint8_t settingCorrected = 0;

  if (storedConfigPtr->chEnGsr
#if defined(SHIMMER3)
      && storedConfigPtr->chEnIntADC1)
#elif defined(SHIMMER3)
      && (storedConfigPtr->chEnIntADC3)
#endif
  {
#if defined(SHIMMER3)
    //they are sharing Shimmer3 adc1, so ban intch1 when gsr is on
    storedConfigPtr->chEnIntADC1 = 0;
#elif defined(SHIMMER3)
    storedConfigPtr->chEnIntADC3 = 0;
#endif
    settingCorrected = 1;
  }
  if (storedConfigPtr->chEnBridgeAmp
#if defined(SHIMMER3)
      && (storedConfigPtr->chEnIntADC13 || storedConfigPtr->chEnIntADC14))
#elif defined(SHIMMER3)
      && (storedConfigPtr->chEnIntADC1 || storedConfigPtr->chEnIntADC2))
#endif
  {
#if defined(SHIMMER3)
    //they are sharing adc13 and adc14
    storedConfigPtr->chEnIntADC13 = 0;
    storedConfigPtr->chEnIntADC14 = 0;
#elif defined(SHIMMER3)
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
#elif defined(SHIMMER3)
      && (storedConfigPtr->chEnIntADC3 || storedConfigPtr->chEnIntADC2))
#endif
  {
#if defined(SHIMMER3)
    storedConfigPtr->chEnIntADC1 = 0;
    storedConfigPtr->chEnIntADC14 = 0;
#elif defined(SHIMMER3)
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
  if (storedConfigPtr->btIntervalSecs < SYNC_INT_C)
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

  if (areADS1292RClockLinesTied() && !(storedConfigPtr->exgADS1292rRegsCh1.config2 & 0x08))
  {
    /* Amend configuration byte 2 of ADS chip 1 to have bit 3 set to 1.
     * This ensures clock lines on ADS chip are correct */
    storedConfigPtr->exgADS1292rRegsCh1.config2 |= 8;
    settingCorrected = 1;
  }

  /* This used to be used to trigger reset of the Bluetooth advertising name
   * and pin code but is no longer needed due to BT driver updates. */
  if (storedConfigPtr->btPinSetup == 0xAA)
  {
    storedConfigPtr->btPinSetup = 0xAB;
    settingCorrected = 1;
  }

  checkSyncCenterName();

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

void ShimConfig_experimentLengthSecsMaxSet(uint16_t expLengthMins)
{
  maxLen = expLengthMins * 60;
}

uint16_t ShimConfig_experimentLengthSecsMaxGet(void)
{
  return maxLen;
}

void ShimConfig_experimentLengthCntReset(void)
{
  maxLenCnt = 0;
}

uint8_t ShimConfig_checkAutostopCondition(void)
{
  if (shimmerStatus.sensing && maxLen)
  {
    if (maxLenCnt < maxLen)
    {
      maxLenCnt++;
      return 0; //Continue sensing
    }
    else
    {
      ShimConfig_experimentLengthCntReset();
      return 1; //Trigger auto-stop
    }
  }
}

/* Note samplingRate can be either a freq or a ticks value */
uint16_t FreqDiv(float samplingRate)
{
  return (uint16_t) round(samplingClockFreqGet() / samplingRate);
}

void checkBtModeConfig(void)
{
  if (!shimmerStatus.btConnected)
  {
    shimmerStatus.btSupportEnabled
        = ShimConfig_getStoredConfig()->bluetoothDisable ? 0 : 1;

    //Don't allow sync to be enabled if BT is disabled.
    shimmerStatus.sdSyncEnabled
        = (shimmerStatus.btSupportEnabled && ShimConfig_getStoredConfig()->syncEnable);

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
