/*
 * shimmer_sd_cfg_file.c
 *
 *  Created on: Mar 26, 2025
 *      Author: MarkNolan
 */

#include "shimmer_sd_cfg_file.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <log_and_stream_includes.h>

FRESULT cfg_file_status;

static uint8_t all0xff[7U] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

void ShimSdCfgFile_init(void)
{
}

void ShimSdCfgFile_generate(void)
{
  FIL cfgFile;

  if (!shimmerStatus.docked && CheckSdInslot() && !shimmerStatus.sdBadFile)
  {
    uint8_t sd_power_state = shimmerStatus.sdPowerOn;
    if (!shimmerStatus.sdPowerOn)
    {
      Board_setSdPower(1);
    }

    char buffer[66], val_char[21];
    float val_num;
    uint16_t val_int, val_f;
    uint64_t temp64;

    uint8_t i;
    ShimSdSync_resetSyncVariablesBeforeParseConfig();

    UINT bw;

    char cfgname[] = "sdlog.cfg";

    gConfigBytes *storedConfig = ShimConfig_getStoredConfig();

    if (ShimConfig_areConfigBytesValid())
    {
      cfg_file_status = f_open(&cfgFile, cfgname, FA_WRITE | FA_CREATE_ALWAYS);

      //sensor0
      sprintf(buffer, "accel=%d\r\n", storedConfig->chEnLnAccel);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "gyro=%d\r\n", storedConfig->chEnGyro);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "mag=%d\r\n", storedConfig->chEnMag);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "exg1_24bit=%d\r\n", storedConfig->chEnExg1_24Bit);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "exg2_24bit=%d\r\n", storedConfig->chEnExg2_24Bit);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "gsr=%d\r\n", storedConfig->chEnGsr);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#if defined(SHIMMER3)
      sprintf(buffer, "extch7=%d\r\n", storedConfig->chEnExtADC7);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "extch6=%d\r\n", storedConfig->chEnExtADC6);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#elif defined(SHIMMER3)
      sprintf(buffer, "extch0=%d\r\n", storedConfig->chEnExtADC0);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "extch1=%d\r\n", storedConfig->chEnExtADC1);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#endif
      //sensor1
      sprintf(buffer, "br_amp=%d\r\n", storedConfig->chEnBridgeAmp);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "vbat=%d\r\n", storedConfig->chEnVBattery);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "accel_d=%d\r\n", storedConfig->chEnWrAccel);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#if defined(SHIMMER3)
      sprintf(buffer, "extch15=%d\r\n", storedConfig->chEnExtADC15);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "intch1=%d\r\n", storedConfig->chEnIntADC1);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "intch12=%d\r\n", storedConfig->chEnIntADC12);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "intch13=%d\r\n", storedConfig->chEnIntADC13);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      //sensor2
      sprintf(buffer, "intch14=%d\r\n", storedConfig->chEnIntADC14);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#elif defined(SHIMMER3)
      sprintf(buffer, "extch2=%d\r\n", storedConfig->chEnExtADC2);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "intch3=%d\r\n", storedConfig->chEnIntADC3);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "intch0=%d\r\n", storedConfig->chEnIntADC0);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "intch1=%d\r\n", storedConfig->chEnIntADC1);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      //sensor2
      sprintf(buffer, "intch2=%d\r\n", storedConfig->chEnIntADC2);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#endif
      sprintf(buffer, "accel_alt=%d\r\n", storedConfig->chEnAltAccel);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "mag_alt=%d\r\n", storedConfig->chEnAltMag);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "exg1_16bit=%d\r\n", storedConfig->chEnExg1_16Bit);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "exg2_16bit=%d\r\n", storedConfig->chEnExg2_16Bit);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "pres=%d\r\n", storedConfig->chEnPressureAndTemperature);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      //sample_rate
      val_num = ShimConfig_freqDiv(storedConfig->samplingRateTicks);
      val_int = (uint16_t) floor(val_num); //the compiler can't handle sprintf %f here
      val_f = (uint16_t) round((val_num - floor(val_num)) * 100);
      if (val_f == 100)
      {
        val_f = 0;
        val_int++;
      }
      if (val_f)
      {
        sprintf(val_char, "%d.%d", val_int, val_f);
      }
      else
      {
        sprintf(val_char, "%d", val_int);
      }
      sprintf(buffer, "sample_rate=%s\r\n", val_char);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      //setup config
      sprintf(buffer, "mg_internal_rate=%d\r\n", ShimConfig_configByteMagRateGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#if defined(SHIMMER3)
      sprintf(buffer, "mg_range=%d\r\n", storedConfig->magRange);
#else
      sprintf(buffer, "mag_alt_range=%d\r\n", storedConfig->altMagRange);
#endif
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "acc_internal_rate=%d\r\n", storedConfig->wrAccelRate);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#if defined(SHIMMER3)
      sprintf(buffer, "accel_alt_range=%d\r\n", storedConfig->altAccelRange);
#elif defined(SHIMMER3R)
      sprintf(buffer, "accel_ln_range=%d\r\n", storedConfig->lnAccelRange);
#endif
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#if defined(SHIMMER3)
      sprintf(buffer,
          (isBmp180InUse() ? ("pres_bmp180_prec=%d\r\n") : ("pres_bmp280_prec=%d\r\n")),
#elif defined(SHIMMER3R)
      sprintf(buffer, "pres_bmp390_prec=%d\r\n",
#endif
          ShimConfig_configBytePressureOversamplingRatioGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "gsr_range=%d\r\n", storedConfig->gsrRange);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "exp_power=%d\r\n", storedConfig->expansionBoardPower);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "gyro_range=%d\r\n", ShimConfig_gyroRangeGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "gyro_samplingrate=%d\r\n", storedConfig->gyroRate);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "acc_range=%d\r\n", storedConfig->wrAccelRange);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "acc_lpm=%d\r\n", ShimConfig_wrAccelLpModeGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "acc_hrm=%d\r\n", storedConfig->wrAccelHrMode);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#if defined(SHIMMER3R)
      sprintf(buffer, "mag_alt_rate=%d\r\n", ShimConfig_configByteAltMagRateGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "accel_alt_rate=%d\r\n", storedConfig->altAccelRate);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#endif

      //trial config
      sprintf(buffer, "user_button_enable=%d\r\n", storedConfig->userButtonEnable);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "rtc_error_enable=%d\r\n", storedConfig->rtcErrorEnable);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "sd_error_enable=%d\r\n", storedConfig->sdErrorEnable);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "iammaster=%d\r\n", storedConfig->masterEnable);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "sync=%d\r\n", storedConfig->syncEnable);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "low_battery_autostop=%d\r\n", storedConfig->lowBatteryAutoStop);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "interval=%d\r\n", storedConfig->btIntervalSecs);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "bluetoothDisabled=%d\r\n", storedConfig->bluetoothDisable);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      sprintf(buffer, "max_exp_len=%d\r\n", ShimConfig_experimentLengthMaxInMinutesGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      sprintf(buffer, "est_exp_len=%d\r\n", ShimConfig_experimentLengthEstimatedInSecGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      ShimSdSync_parseSyncNodeNamesFromConfig(&storedConfig->rawBytes[0]);
      for (i = 0; i < ShimSdSync_syncNodeNumGet(); i++)
      {
        sprintf(buffer, "node=%s\r\n", (char *) ShimSdSync_syncNodeNamePtrForIndexGet(i));
        f_write(&cfgFile, buffer, strlen(buffer), &bw);
      }

      if (memcmp(all0xff, storedConfig + NV_CENTER, 6))
      {
        ShimSdSync_parseSyncCenterNameFromConfig(&storedConfig->rawBytes[0]);
        sprintf(buffer, "center=%s\r\n", (char *) ShimSdSync_syncCenterNamePtrGet());
        f_write(&cfgFile, buffer, strlen(buffer), &bw);
      }

#if IS_SUPPORTED_SINGLE_TOUCH
      sprintf(buffer, "singletouch=%d\r\n", storedConfig->singleTouchStart);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#endif //IS_SUPPORTED_SINGLE_TOUCH

      sprintf(buffer, "myid=%d\r\n", storedConfig->myTrialID);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "Nshimmer=%d\r\n", storedConfig->numberOfShimmers);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      sprintf(buffer, "shimmername=%s\r\n", ShimConfig_shimmerNameParseToTxtAndPtrGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "experimentid=%s\r\n", ShimConfig_expIdParseToTxtAndPtrGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "configtime=%s\r\n", ShimConfig_configTimeParseToTxtAndPtrGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      temp64 = storedConfig->rawBytes[NV_DERIVED_CHANNELS_0]
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_1]) << 8)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_2]) << 16)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_3]) << 24)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_4]) << 32)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_5]) << 40)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_6]) << 48)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_7]) << 56);
      ShimUtil_ItoaNo0(temp64, val_char, 21);
      sprintf(buffer, "derived_channels=%s\r\n", val_char); //todo: got value 0?
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      sprintf(buffer, "EXG_ADS1292R_1_CONFIG1=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_CONFIG1]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_CONFIG2=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_CONFIG2]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_LOFF=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_LOFF]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_CH1SET=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_CH1SET]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_CH2SET=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_CH2SET]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_RLD_SENS=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_RLD_SENS]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_LOFF_SENS=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_LOFF_SENS]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_LOFF_STAT=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_LOFF_STAT]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_RESP1=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_RESP1]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_RESP2=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_RESP2]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_CONFIG1=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_CONFIG1]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_CONFIG2=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_CONFIG2]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_LOFF=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_LOFF]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_CH1SET=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_CH1SET]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_CH2SET=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_CH2SET]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_RLD_SENS=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_RLD_SENS]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_LOFF_SENS=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_LOFF_SENS]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_LOFF_STAT=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_LOFF_STAT]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_RESP1=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_RESP1]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_RESP2=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_RESP2]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      cfg_file_status = f_close(&cfgFile);
      ShimSd_setFileTimestamp(cfgname);

#if defined(SHIMMER3)
      _delay_cycles(2400000); //100ms @ 24MHz
#elif defined(SHIMMER3R)
      HAL_Delay(100); //100ms
#endif
    }
    else
    {
      cfg_file_status = FR_DISK_ERR;
    }
    if (!sd_power_state)
    {
      Board_setSdPower(0);
    }
  }
}

void ShimSdCfgFile_parse(void)
{
  FIL cfgFile;

  char buffer[66], *equals;
  uint8_t string_length = 0;
  float sample_rate = 51.2;
  float sample_period = 0;
  uint64_t derived_channels_val = 0;
  uint8_t gsr_range = 0;
  uint32_t config_time = 0;

  uint32_t est_exp_len = 0;
  uint32_t max_exp_len = 0;

  uint8_t triggerSdCardUpdate = 0;

  CheckSdInslot();
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();
  char cfgname[] = "sdlog.cfg";
  cfg_file_status = f_open(&cfgFile, cfgname, FA_READ | FA_OPEN_EXISTING);
  if (cfg_file_status == FR_NO_FILE)
  {
    ShimConfig_readRam();
    ShimSdCfgFile_generate();
    //fileBad = 0;
  }
  else if (cfg_file_status != FR_OK)
  {
    shimmerStatus.sdBadFile = 1;
    //fileBad = (initializing) ? 0 : 1;
    return;
  }
  else
  {
    /* Backup configuration bytes before parsing from cfg file */
    gConfigBytes storedConfigTemp;
    memcpy(&(storedConfigTemp.rawBytes[0]), &(storedConfigPtr->rawBytes[0]), STOREDCONFIG_SIZE);

    //Reset global configuration bytes to a blank state before parsing the config file.
    ShimConfig_createBlankConfigBytes();
    ShimSdSync_resetSyncVariablesBeforeParseConfig();
    ShimSdSync_resetSyncNodeArray();

    /* Copy back in the NV_SD_CONFIG_DELAY_FLAG byte */
    storedConfigPtr->rawBytes[NV_SD_CONFIG_DELAY_FLAG] = storedConfigTemp.rawBytes[NV_SD_CONFIG_DELAY_FLAG];

#if defined(SHIMMER3)
    storedConfigPtr->rawBytes[NV_SD_TRIAL_CONFIG0] &= ~SDH_SET_PMUX; //PMUX reserved as 0
//stored_config_temp.rawBytes[NV_SD_TRIAL_CONFIG0] |= SDH_TIME_STAMP; //TIME_STAMP always = 1
#endif
    storedConfigPtr->gsrRange = GSR_AUTORANGE;
    storedConfigPtr->bufferSize = 1;
    storedConfigPtr->btCommsBaudRate = getDefaultBaudForBtVersion();
    storedConfigPtr->bluetoothDisable = 0;
    storedConfigPtr->btIntervalSecs = SYNC_INT_C;

    while (f_gets(buffer, 64, &cfgFile))
    {
      if (!(equals = strchr(buffer, '=')))
      {
        continue;
      }
      equals++; //this is the value
      if (strstr(buffer, "accel="))
      {
        storedConfigPtr->chEnLnAccel = atoi(equals);
      }
      else if (strstr(buffer, "gyro="))
      {
        storedConfigPtr->chEnGyro = atoi(equals);
      }
      else if (strstr(buffer, "mag="))
      {
        storedConfigPtr->chEnMag = atoi(equals);
      }
      else if (strstr(buffer, "exg1_24bit="))
      {
        storedConfigPtr->chEnExg1_24Bit = atoi(equals);
      }
      else if (strstr(buffer, "exg2_24bit="))
      {
        storedConfigPtr->chEnExg2_24Bit = atoi(equals);
      }
      else if (strstr(buffer, "gsr="))
      {
        storedConfigPtr->chEnGsr = atoi(equals);
      }
#if defined(SHIMMER3)
      else if (strstr(buffer, "extch7="))
      {
        storedConfigPtr->chEnExtADC7 = atoi(equals);
      }
      else if (strstr(buffer, "extch6="))
      {
        storedConfigPtr->chEnExtADC6 = atoi(equals);
      }
#elif defined(SHIMMER3R)
      else if (strstr(buffer, "extch0="))
      {
        storedConfigPtr->chEnExtADC0 = atoi(equals);
      }
      else if (strstr(buffer, "extch1="))
      {
        storedConfigPtr->chEnExtADC1 = atoi(equals);
      }
#endif
      else if (strstr(buffer, "str=") || strstr(buffer, "br_amp="))
      {
        storedConfigPtr->chEnBridgeAmp = atoi(equals);
      }
      else if (strstr(buffer, "vbat="))
      {
        storedConfigPtr->chEnVBattery = atoi(equals);
      }
      else if (strstr(buffer, "accel_d="))
      {
        storedConfigPtr->chEnWrAccel = atoi(equals);
      }
#if defined(SHIMMER3)
      else if (strstr(buffer, "extch15="))
      {
        storedConfigPtr->chEnExtADC15 = atoi(equals);
      }
      else if (strstr(buffer, "intch1="))
      {
        storedConfigPtr->chEnIntADC1 = atoi(equals);
      }
      else if (strstr(buffer, "intch12="))
      {
        storedConfigPtr->chEnIntADC12 = atoi(equals);
      }
      else if (strstr(buffer, "intch13="))
      {
        storedConfigPtr->chEnIntADC13 = atoi(equals);
      }
      else if (strstr(buffer, "intch14="))
      {
        storedConfigPtr->chEnIntADC14 = atoi(equals);
      }
      else if (strstr(buffer, "accel_mpu="))
      {
        storedConfigPtr->chEnAltAccel = atoi(equals);
      }
      else if (strstr(buffer, "mag_mpu="))
      {
        storedConfigPtr->chEnAltMag = atoi(equals);
      }
#elif defined(SHIMMER3R)
      else if (strstr(buffer, "extch2="))
      {
        storedConfigPtr->chEnExtADC2 = atoi(equals);
      }
      else if (strstr(buffer, "intch3="))
      {
        storedConfigPtr->chEnIntADC3 = atoi(equals);
      }
      else if (strstr(buffer, "intch0="))
      {
        storedConfigPtr->chEnIntADC0 = atoi(equals);
      }
      else if (strstr(buffer, "intch1="))
      {
        storedConfigPtr->chEnIntADC1 = atoi(equals);
      }
      else if (strstr(buffer, "intch2="))
      {
        storedConfigPtr->chEnIntADC2 = atoi(equals);
      }
      else if (strstr(buffer, "accel_alt="))
      {
        storedConfigPtr->chEnAltAccel = atoi(equals);
      }
      else if (strstr(buffer, "mag_alt="))
      {
        storedConfigPtr->chEnAltMag = atoi(equals);
      }
#endif
      else if (strstr(buffer, "exg1_16bit="))
      {
        storedConfigPtr->chEnExg1_16Bit = atoi(equals);
      }
      else if (strstr(buffer, "exg2_16bit="))
      {
        storedConfigPtr->chEnExg2_16Bit = atoi(equals);
      }
      else if (strstr(buffer, "pres="))
      {
        storedConfigPtr->chEnPressureAndTemperature = atoi(equals);
      }
      else if (strstr(buffer, "sample_rate="))
      {
        sample_rate = atof(equals);
      }
      else if (strstr(buffer, "mg_internal_rate="))
      {
        ShimConfig_configByteMagRateSet(atoi(equals));
      }
#if defined(SHIMMER3)
      else if (strstr(buffer, "mg_range="))
      {
        storedConfigPtr->magRange = atoi(equals);
      }
#else
      else if (strstr(buffer, "mag_alt_range="))
      {
        storedConfigPtr->altMagRange = atoi(equals);
      }
#endif
      else if (strstr(buffer, "acc_internal_rate="))
      {
        storedConfigPtr->wrAccelRate = atoi(equals);
      }
#if defined(SHIMMER3)
      else if (strstr(buffer, "accel_alt_range="))
      {
        storedConfigPtr->altAccelRange = atoi(equals);
      }
#elif defined(SHIMMER3R)
      else if (strstr(buffer, "accel_ln_range="))
      {
        storedConfigPtr->lnAccelRange = atoi(equals);
      }
#endif
      else if (strstr(buffer, "acc_range="))
      {
        storedConfigPtr->wrAccelRange = atoi(equals);
      }
      else if (strstr(buffer, "acc_lpm="))
      {
        ShimConfig_wrAccelLpModeSet(atoi(equals));
      }
      else if (strstr(buffer, "acc_hrm="))
      {
        storedConfigPtr->wrAccelHrMode = atoi(equals);
      }
      else if (strstr(buffer, "gsr_range="))
      { //or "gsr_range="?
        gsr_range = atoi(equals);
        if (gsr_range > 4)
        {
          gsr_range = 4;
        }

        storedConfigPtr->gsrRange = gsr_range;
      }
      else if (strstr(buffer, "gyro_samplingrate="))
      {
        ShimConfig_gyroRateSet(atoi(equals));
      }
      else if (strstr(buffer, "gyro_range="))
      {
        ShimConfig_gyroRangeSet(atoi(equals));
      }
#if defined(SHIMMER3)
      else if (strstr(buffer, "pres_bmp180_prec=") || strstr(buffer, "pres_bmp280_prec="))
      {
        ShimConfig_configBytePressureOversamplingRatioSet(atoi(equals));
      }
#elif defined(SHIMMER3R)
      else if (strstr(buffer, "pres_bmp390_prec="))
      {
        ShimConfig_configBytePressureOversamplingRatioSet(atoi(equals));
      }
#endif

#if defined(SHIMMER3R)
      else if (strstr(buffer, "mag_alt_rate="))
      {
        ShimConfig_configByteAltMagRateSet(atoi(equals));
      }
      else if (strstr(buffer, "accel_alt_rate="))
      {
        storedConfigPtr->altAccelRate = atoi(equals);
      }
#endif
      else if (strstr(buffer, "rtc_error_enable="))
      {
        storedConfigPtr->rtcErrorEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "sd_error_enable="))
      {
        storedConfigPtr->sdErrorEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "user_button_enable="))
      {
        storedConfigPtr->userButtonEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "iammaster="))
      { //0=slave=node
        storedConfigPtr->masterEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "sync="))
      {
        storedConfigPtr->syncEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "bluetoothDisabled="))
      {
        storedConfigPtr->bluetoothDisable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "low_battery_autostop="))
      {
        storedConfigPtr->lowBatteryAutoStop = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "interval="))
      {
        storedConfigPtr->btIntervalSecs = atoi(equals) > 255 ? 255 : atoi(equals);
      }
      else if (strstr(buffer, "exp_power="))
      {
        storedConfigPtr->expansionBoardPower = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "center="))
      {
        ShimSdSync_parseSyncCenterNameFromCfgFile(&storedConfigPtr->rawBytes[0], equals);
      }
      else if (strstr(buffer, "node"))
      {
        ShimSdSync_parseSyncNodeNameFromCfgFile(&storedConfigPtr->rawBytes[0], equals);
      }
      else if (strstr(buffer, "est_exp_len="))
      {
        est_exp_len = atoi(equals);
        storedConfigPtr->experimentLengthEstimatedInSecMsb
            = (est_exp_len & 0xff00) >> 8;
        storedConfigPtr->experimentLengthEstimatedInSecLsb = est_exp_len & 0xff;
      }
      else if (strstr(buffer, "max_exp_len="))
      {
        max_exp_len = atoi(equals);
        storedConfigPtr->experimentLengthMaxInMinutesMsb = (max_exp_len & 0xff00) >> 8;
        storedConfigPtr->experimentLengthMaxInMinutesLsb = max_exp_len & 0xff;
      }
#if IS_SUPPORTED_SINGLE_TOUCH
      else if (strstr(buffer, "singletouch="))
      {
        storedConfigPtr->singleTouchStart = (atoi(equals) == 0) ? 0 : 1;
      }
#endif //IS_SUPPORTED_SINGLE_TOUCH
      else if (strstr(buffer, "myid="))
      {
        storedConfigPtr->myTrialID = atoi(equals);
      }
      else if (strstr(buffer, "Nshimmer="))
      {
        storedConfigPtr->numberOfShimmers = atoi(equals);
      }
      else if (strstr(buffer, "shimmername="))
      {
        string_length = strlen(equals);
        if (string_length > MAX_CHARS)
        {
          string_length = MAX_CHARS - 1;
        }
        else if (string_length >= 2)
        {
          string_length -= 2;
        }
        else
        {
          string_length = 0;
        }
        memcpy(&storedConfigPtr->shimmerName[0], equals, string_length);
      }
      else if (strstr(buffer, "experimentid="))
      {
        string_length = strlen(equals);
        if (string_length > MAX_CHARS)
        {
          string_length = MAX_CHARS - 1;
        }
        else if (string_length >= 2)
        {
          string_length -= 2;
        }
        else
        {
          string_length = 0;
        }
        memcpy(&storedConfigPtr->expIdName[0], equals, string_length);
      }
      else if (strstr(buffer, "configtime="))
      {
        config_time = atol(equals);
        ShimConfig_configTimeSet(config_time);
      }

      else if (strstr(buffer, "EXG_ADS1292R_1_CONFIG1="))
      {
        storedConfigPtr->exgADS1292rRegsCh1.config1 = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_CONFIG2="))
      {
        storedConfigPtr->exgADS1292rRegsCh1.config2 = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_LOFF="))
      {
        storedConfigPtr->exgADS1292rRegsCh1.loff = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_CH1SET="))
      {
        storedConfigPtr->exgADS1292rRegsCh1.ch1set = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_CH2SET="))
      {
        storedConfigPtr->exgADS1292rRegsCh1.ch2set = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_RLD_SENS="))
      {
        storedConfigPtr->exgADS1292rRegsCh1.rldSens = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_LOFF_SENS="))
      {
        storedConfigPtr->exgADS1292rRegsCh1.loffSens = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_LOFF_STAT="))
      {
        storedConfigPtr->exgADS1292rRegsCh1.loffStat = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_RESP1="))
      {
        storedConfigPtr->exgADS1292rRegsCh1.resp1 = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_RESP2="))
      {
        storedConfigPtr->exgADS1292rRegsCh1.resp2 = atoi(equals);
      }

      else if (strstr(buffer, "EXG_ADS1292R_2_CONFIG1="))
      {
        storedConfigPtr->exgADS1292rRegsCh2.config1 = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_CONFIG2="))
      {
        storedConfigPtr->exgADS1292rRegsCh2.config2 = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_LOFF="))
      {
        storedConfigPtr->exgADS1292rRegsCh2.loff = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_CH1SET="))
      {
        storedConfigPtr->exgADS1292rRegsCh2.ch1set = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_CH2SET="))
      {
        storedConfigPtr->exgADS1292rRegsCh2.ch2set = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_RLD_SENS="))
      {
        storedConfigPtr->exgADS1292rRegsCh2.rldSens = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_LOFF_SENS="))
      {
        storedConfigPtr->exgADS1292rRegsCh2.loffSens = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_LOFF_STAT="))
      {
        storedConfigPtr->exgADS1292rRegsCh2.loffStat = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_RESP1="))
      {
        storedConfigPtr->exgADS1292rRegsCh2.resp1 = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_RESP2="))
      {
        storedConfigPtr->exgADS1292rRegsCh2.resp2 = atoi(equals);
      }

      else if (strstr(buffer, "derived_channels="))
      {
        derived_channels_val = atoll(equals);
        storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_0] = derived_channels_val & 0xFF;
        storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_1]
            = (derived_channels_val >> 8) & 0xFF;
        storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_2]
            = (derived_channels_val >> 16) & 0xFF;
        storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_3]
            = (derived_channels_val >> 24) & 0xFF;
        storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_4]
            = (derived_channels_val >> 32) & 0xFF;
        storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_5]
            = (derived_channels_val >> 40) & 0xFF;
        storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_6]
            = (derived_channels_val >> 48) & 0xFF;
        storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_7]
            = (derived_channels_val >> 56) & 0xFF;
      }
    }
    cfg_file_status = f_close(&cfgFile);

#if defined(SHIMMER3)
    _delay_cycles(1200000); //50ms
#elif defined(SHIMMER3R)
    HAL_Delay(50); //50ms
#endif
    sample_period = (round) (ShimConfig_freqDiv(sample_rate));
    storedConfigPtr->samplingRateTicks = (uint16_t) sample_period;

    /* restoring original calibration bytes which is not updated from calib file*/
    memcpy((storedConfigPtr->lnAccelCalib.rawBytes),
        &(storedConfigTemp.lnAccelCalib.rawBytes),
        sizeof(storedConfigTemp.lnAccelCalib.rawBytes));
    memcpy((storedConfigPtr->gyroCalib.rawBytes),
        &(storedConfigTemp.gyroCalib.rawBytes),
        sizeof(storedConfigTemp.gyroCalib.rawBytes));
    memcpy((storedConfigPtr->magCalib.rawBytes),
        &(storedConfigTemp.magCalib.rawBytes),
        sizeof(storedConfigTemp.magCalib.rawBytes));
    memcpy((storedConfigPtr->wrAccelCalib.rawBytes),
        &(storedConfigTemp.wrAccelCalib.rawBytes),
        sizeof(storedConfigTemp.wrAccelCalib.rawBytes));
    memcpy((storedConfigPtr->altAccelCalib.rawBytes),
        &(storedConfigTemp.altAccelCalib.rawBytes),
        sizeof(storedConfigTemp.altAccelCalib.rawBytes));
    memcpy((storedConfigPtr->altMagCalib.rawBytes),
        &(storedConfigTemp.altMagCalib.rawBytes),
        sizeof(storedConfigTemp.altMagCalib.rawBytes));

    triggerSdCardUpdate |= ShimConfig_checkAndCorrectConfig();

    LogAndStream_infomemUpdate();
    /* If the configuration needed to be corrected, update the config file */
    if (triggerSdCardUpdate)
    {
      ShimSdCfgFile_generate();
    }
  }
}

void ShimSdCfgFile_readSdConfiguration(void)
{
  ShimTask_clear(TASK_STREAMDATA); //this will skip one sample
  sensing.isFileCreated = 0;
  Board_setSdPower(1);
  ShimSdCfgFile_parse();

  /* Check BT module configuration after sensor configuration read from SD
   * card to see if it is in the correct state (i.e., BT on vs. BT off vs. SD
   * Sync) */
  ShimConfig_checkBtModeFromConfig();
}
