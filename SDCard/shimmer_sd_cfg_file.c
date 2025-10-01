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
#include <string.h>

#include "log_and_stream_includes.h"

FRESULT cfg_file_status;

static uint8_t all0xff[7U] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

//Centralized config file constants to avoid duplicated string literals
#define CFG_FILENAME "sdlog.cfg"

//Common formatting strings
#define CFG_FMT_INT  "%s=%d\r\n"
#define CFG_FMT_STR  "%s=%s\r\n"

//Helper for key matching at the start of a line (e.g., "key=")
static int cfg_key_is(const char *line, const char *key)
{
  size_t klen = strlen(key);
  return (strncmp(line, key, klen) == 0) && (line[klen] == '=');
}

//Keys (without the trailing '=')
#define CFG_KEY_ACCEL              "accel"
#define CFG_KEY_GYRO               "gyro"
#define CFG_KEY_MAG                "mag"
#define CFG_KEY_EXG1_24BIT         "exg1_24bit"
#define CFG_KEY_EXG2_24BIT         "exg2_24bit"
#define CFG_KEY_GSR                "gsr"
#define CFG_KEY_EXTCH7             "extch7"
#define CFG_KEY_EXTCH6             "extch6"
#define CFG_KEY_EXTCH0             "extch0"
#define CFG_KEY_EXTCH1             "extch1"
#define CFG_KEY_BR_AMP             "br_amp"
#define CFG_KEY_STR                "str"
#define CFG_KEY_VBAT               "vbat"
#define CFG_KEY_ACCEL_D            "accel_d"
#define CFG_KEY_EXTCH15            "extch15"
#define CFG_KEY_INTCH1             "intch1"
#define CFG_KEY_INTCH12            "intch12"
#define CFG_KEY_INTCH13            "intch13"
#define CFG_KEY_INTCH14            "intch14"
#define CFG_KEY_EXTCH2             "extch2"
#define CFG_KEY_INTCH3             "intch3"
#define CFG_KEY_INTCH0             "intch0"
#define CFG_KEY_INTCH2             "intch2"
#define CFG_KEY_ACCEL_ALT          "accel_alt"
#define CFG_KEY_MAG_ALT            "mag_alt"
//Legacy names used under SHIMMER3 parsing
#define CFG_KEY_ACCEL_MPU          "accel_mpu"
#define CFG_KEY_MAG_MPU            "mag_mpu"

#define CFG_KEY_EXG1_16BIT         "exg1_16bit"
#define CFG_KEY_EXG2_16BIT         "exg2_16bit"
#define CFG_KEY_PRES               "pres"
#define CFG_KEY_SAMPLE_RATE        "sample_rate"
#define CFG_KEY_MG_INTERNAL_RATE   "mg_internal_rate"
#define CFG_KEY_MG_RANGE           "mg_range"
#define CFG_KEY_MAG_ALT_RANGE      "mag_alt_range"
#define CFG_KEY_ACC_INTERNAL_RATE  "acc_internal_rate"
#define CFG_KEY_ACCEL_ALT_RANGE    "accel_alt_range"
#define CFG_KEY_ACCEL_LN_RANGE     "accel_ln_range"
#define CFG_KEY_PRES_BMP180_PREC   "pres_bmp180_prec"
#define CFG_KEY_PRES_BMP280_PREC   "pres_bmp280_prec"
#define CFG_KEY_PRES_BMP390_PREC   "pres_bmp390_prec"
#define CFG_KEY_GSR_RANGE          "gsr_range"
#define CFG_KEY_EXP_POWER          "exp_power"
#define CFG_KEY_GYRO_RANGE         "gyro_range"
#define CFG_KEY_GYRO_SAMPLINGRATE  "gyro_samplingrate"
#define CFG_KEY_ACC_RANGE          "acc_range"
#define CFG_KEY_ACC_LPM            "acc_lpm"
#define CFG_KEY_ACC_HRM            "acc_hrm"
#define CFG_KEY_MAG_ALT_RATE       "mag_alt_rate"
#define CFG_KEY_ACCEL_ALT_RATE     "accel_alt_rate"

#define CFG_KEY_USER_BUTTON_ENABLE "user_button_enable"
#define CFG_KEY_RTC_ERROR_ENABLE   "rtc_error_enable"
#define CFG_KEY_SD_ERROR_ENABLE    "sd_error_enable"
#define CFG_KEY_IAMMASTER          "iammaster"
#define CFG_KEY_SYNC               "sync"
#define CFG_KEY_LOW_BATT_AUTOSTOP  "low_battery_autostop"
#define CFG_KEY_INTERVAL           "interval"
#define CFG_KEY_BT_DISABLED        "bluetoothDisabled"

#define CFG_KEY_MAX_EXP_LEN        "max_exp_len"
#define CFG_KEY_EST_EXP_LEN        "est_exp_len"
#define CFG_KEY_NODE               "node"
#define CFG_KEY_CENTER             "center"
#define CFG_KEY_SINGLETOUCH        "singletouch"
#define CFG_KEY_MYID               "myid"
#define CFG_KEY_NSHIMMER           "Nshimmer"
#define CFG_KEY_SHIMMERNAME        "shimmername"
#define CFG_KEY_EXPERIMENTID       "experimentid"
#define CFG_KEY_CONFIGTIME         "configtime"

#define CFG_KEY_DERIVED_CHANNELS   "derived_channels"

//EXG register keys
#define CFG_KEY_EXG_1_CONFIG1      "EXG_ADS1292R_1_CONFIG1"
#define CFG_KEY_EXG_1_CONFIG2      "EXG_ADS1292R_1_CONFIG2"
#define CFG_KEY_EXG_1_LOFF         "EXG_ADS1292R_1_LOFF"
#define CFG_KEY_EXG_1_CH1SET       "EXG_ADS1292R_1_CH1SET"
#define CFG_KEY_EXG_1_CH2SET       "EXG_ADS1292R_1_CH2SET"
#define CFG_KEY_EXG_1_RLD_SENS     "EXG_ADS1292R_1_RLD_SENS"
#define CFG_KEY_EXG_1_LOFF_SENS    "EXG_ADS1292R_1_LOFF_SENS"
#define CFG_KEY_EXG_1_LOFF_STAT    "EXG_ADS1292R_1_LOFF_STAT"
#define CFG_KEY_EXG_1_RESP1        "EXG_ADS1292R_1_RESP1"
#define CFG_KEY_EXG_1_RESP2        "EXG_ADS1292R_1_RESP2"
#define CFG_KEY_EXG_2_CONFIG1      "EXG_ADS1292R_2_CONFIG1"
#define CFG_KEY_EXG_2_CONFIG2      "EXG_ADS1292R_2_CONFIG2"
#define CFG_KEY_EXG_2_LOFF         "EXG_ADS1292R_2_LOFF"
#define CFG_KEY_EXG_2_CH1SET       "EXG_ADS1292R_2_CH1SET"
#define CFG_KEY_EXG_2_CH2SET       "EXG_ADS1292R_2_CH2SET"
#define CFG_KEY_EXG_2_RLD_SENS     "EXG_ADS1292R_2_RLD_SENS"
#define CFG_KEY_EXG_2_LOFF_SENS    "EXG_ADS1292R_2_LOFF_SENS"
#define CFG_KEY_EXG_2_LOFF_STAT    "EXG_ADS1292R_2_LOFF_STAT"
#define CFG_KEY_EXG_2_RESP1        "EXG_ADS1292R_2_RESP1"
#define CFG_KEY_EXG_2_RESP2        "EXG_ADS1292R_2_RESP2"

//Write helpers (depend on local buffer and bw symbols in scope)
#define CFG_WRITE_INT(file, key, value)               \
  do                                                  \
  {                                                   \
    sprintf(buffer, CFG_FMT_INT, key, (int) (value)); \
    f_write((file), buffer, strlen(buffer), &bw);     \
  } while (0)

#define CFG_WRITE_STR(file, key, strval)          \
  do                                              \
  {                                               \
    sprintf(buffer, CFG_FMT_STR, key, (strval));  \
    f_write((file), buffer, strlen(buffer), &bw); \
  } while (0)

void ShimSdCfgFile_init(void)
{
}

void ShimSdCfgFile_generate(void)
{
  FIL cfgFile;

  if (!shimmerStatus.docked && LogAndStream_checkSdInSlot() && !shimmerStatus.sdBadFile)
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

    char cfgname[] = CFG_FILENAME;

    gConfigBytes *storedConfig = ShimConfig_getStoredConfig();

    if (ShimConfig_areConfigBytesValid())
    {
      cfg_file_status = f_open(&cfgFile, cfgname, FA_WRITE | FA_CREATE_ALWAYS);

      //sensor0
      CFG_WRITE_INT(&cfgFile, CFG_KEY_ACCEL, storedConfig->chEnLnAccel);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_GYRO, storedConfig->chEnGyro);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_MAG, storedConfig->chEnMag);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG1_24BIT, storedConfig->chEnExg1_24Bit);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG2_24BIT, storedConfig->chEnExg2_24Bit);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_GSR, storedConfig->chEnGsr);
#if defined(SHIMMER3)
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXTCH7, storedConfig->chEnExtADC7);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXTCH6, storedConfig->chEnExtADC6);
#elif defined(SHIMMER3R)
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXTCH0, storedConfig->chEnExtADC0);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXTCH1, storedConfig->chEnExtADC1);
#endif
      //sensor1
      CFG_WRITE_INT(&cfgFile, CFG_KEY_BR_AMP, storedConfig->chEnBridgeAmp);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_VBAT, storedConfig->chEnVBattery);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_ACCEL_D, storedConfig->chEnWrAccel);
#if defined(SHIMMER3)
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXTCH15, storedConfig->chEnExtADC15);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_INTCH1, storedConfig->chEnIntADC1);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_INTCH12, storedConfig->chEnIntADC12);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_INTCH13, storedConfig->chEnIntADC13);
      //sensor2
      CFG_WRITE_INT(&cfgFile, CFG_KEY_INTCH14, storedConfig->chEnIntADC14);
#elif defined(SHIMMER3R)
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXTCH2, storedConfig->chEnExtADC2);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_INTCH3, storedConfig->chEnIntADC3);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_INTCH0, storedConfig->chEnIntADC0);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_INTCH1, storedConfig->chEnIntADC1);
      //sensor2
      CFG_WRITE_INT(&cfgFile, CFG_KEY_INTCH2, storedConfig->chEnIntADC2);
#endif
      CFG_WRITE_INT(&cfgFile, CFG_KEY_ACCEL_ALT, storedConfig->chEnAltAccel);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_MAG_ALT, storedConfig->chEnAltMag);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG1_16BIT, storedConfig->chEnExg1_16Bit);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG2_16BIT, storedConfig->chEnExg2_16Bit);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_PRES, storedConfig->chEnPressureAndTemperature);
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
      CFG_WRITE_STR(&cfgFile, CFG_KEY_SAMPLE_RATE, val_char);
      //setup config
      CFG_WRITE_INT(&cfgFile, CFG_KEY_MG_INTERNAL_RATE, ShimConfig_configByteMagRateGet());
#if defined(SHIMMER3)
      CFG_WRITE_INT(&cfgFile, CFG_KEY_MG_RANGE, storedConfig->magRange);
#else
      CFG_WRITE_INT(&cfgFile, CFG_KEY_MAG_ALT_RANGE, storedConfig->altMagRange);
#endif
      CFG_WRITE_INT(&cfgFile, CFG_KEY_ACC_INTERNAL_RATE, storedConfig->wrAccelRate);
#if defined(SHIMMER3)
      CFG_WRITE_INT(&cfgFile, CFG_KEY_ACCEL_ALT_RANGE, storedConfig->altAccelRange);
#elif defined(SHIMMER3R)
      CFG_WRITE_INT(&cfgFile, CFG_KEY_ACCEL_LN_RANGE, storedConfig->lnAccelRange);
#endif
#if defined(SHIMMER3)
      CFG_WRITE_INT(&cfgFile,
          (isBmp180InUse() ? CFG_KEY_PRES_BMP180_PREC : CFG_KEY_PRES_BMP280_PREC),
          ShimConfig_configBytePressureOversamplingRatioGet());
#elif defined(SHIMMER3R)
      CFG_WRITE_INT(&cfgFile, CFG_KEY_PRES_BMP390_PREC,
          ShimConfig_configBytePressureOversamplingRatioGet());
#endif
      CFG_WRITE_INT(&cfgFile, CFG_KEY_GSR_RANGE, storedConfig->gsrRange);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXP_POWER, storedConfig->expansionBoardPower);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_GYRO_RANGE, ShimConfig_gyroRangeGet());
      CFG_WRITE_INT(&cfgFile, CFG_KEY_GYRO_SAMPLINGRATE, storedConfig->gyroRate);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_ACC_RANGE, storedConfig->wrAccelRange);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_ACC_LPM, ShimConfig_wrAccelLpModeGet());
      CFG_WRITE_INT(&cfgFile, CFG_KEY_ACC_HRM, storedConfig->wrAccelHrMode);
#if defined(SHIMMER3R)
      CFG_WRITE_INT(&cfgFile, CFG_KEY_MAG_ALT_RATE, ShimConfig_configByteAltMagRateGet());
      CFG_WRITE_INT(&cfgFile, CFG_KEY_ACCEL_ALT_RATE, storedConfig->altAccelRate);
#endif

      //trial config
      CFG_WRITE_INT(&cfgFile, CFG_KEY_USER_BUTTON_ENABLE, storedConfig->userButtonEnable);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_RTC_ERROR_ENABLE, storedConfig->rtcErrorEnable);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_SD_ERROR_ENABLE, storedConfig->sdErrorEnable);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_IAMMASTER, storedConfig->masterEnable);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_SYNC, storedConfig->syncEnable);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_LOW_BATT_AUTOSTOP, storedConfig->lowBatteryAutoStop);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_INTERVAL, storedConfig->btIntervalSecs);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_BT_DISABLED, storedConfig->bluetoothDisable);

      CFG_WRITE_INT(&cfgFile, CFG_KEY_MAX_EXP_LEN,
          ShimConfig_experimentLengthMaxInMinutesGet());
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EST_EXP_LEN,
          ShimConfig_experimentLengthEstimatedInSecGet());

      ShimSdSync_parseSyncNodeNamesFromConfig(&storedConfig->rawBytes[0]);
      for (i = 0; i < ShimSdSync_syncNodeNumGet(); i++)
      {
        CFG_WRITE_STR(&cfgFile, CFG_KEY_NODE,
            (char *) ShimSdSync_syncNodeNamePtrForIndexGet(i));
      }

      if (memcmp(all0xff, storedConfig + NV_CENTER, 6))
      {
        ShimSdSync_parseSyncCenterNameFromConfig(&storedConfig->rawBytes[0]);
        CFG_WRITE_STR(&cfgFile, CFG_KEY_CENTER, (char *) ShimSdSync_syncCenterNamePtrGet());
      }

#if IS_SUPPORTED_SINGLE_TOUCH
      CFG_WRITE_INT(&cfgFile, CFG_KEY_SINGLETOUCH, storedConfig->singleTouchStart);
#endif //IS_SUPPORTED_SINGLE_TOUCH

      CFG_WRITE_INT(&cfgFile, CFG_KEY_MYID, storedConfig->myTrialID);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_NSHIMMER, storedConfig->numberOfShimmers);

      CFG_WRITE_STR(&cfgFile, CFG_KEY_SHIMMERNAME,
          ShimConfig_shimmerNameParseToTxtAndPtrGet());
      CFG_WRITE_STR(&cfgFile, CFG_KEY_EXPERIMENTID, ShimConfig_expIdParseToTxtAndPtrGet());
      CFG_WRITE_STR(&cfgFile, CFG_KEY_CONFIGTIME,
          ShimConfig_configTimeParseToTxtAndPtrGet());

      temp64 = storedConfig->rawBytes[NV_DERIVED_CHANNELS_0]
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_1]) << 8)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_2]) << 16)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_3]) << 24)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_4]) << 32)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_5]) << 40)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_6]) << 48)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_7]) << 56);
      ShimUtil_ItoaNo0(temp64, val_char, 21);
      CFG_WRITE_STR(&cfgFile, CFG_KEY_DERIVED_CHANNELS, val_char); //todo: got value 0?

      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_1_CONFIG1,
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_CONFIG1]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_1_CONFIG2,
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_CONFIG2]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_1_LOFF,
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_LOFF]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_1_CH1SET,
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_CH1SET]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_1_CH2SET,
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_CH2SET]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_1_RLD_SENS,
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_RLD_SENS]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_1_LOFF_SENS,
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_LOFF_SENS]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_1_LOFF_STAT,
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_LOFF_STAT]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_1_RESP1,
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_RESP1]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_1_RESP2,
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_RESP2]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_2_CONFIG1,
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_CONFIG1]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_2_CONFIG2,
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_CONFIG2]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_2_LOFF,
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_LOFF]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_2_CH1SET,
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_CH1SET]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_2_CH2SET,
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_CH2SET]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_2_RLD_SENS,
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_RLD_SENS]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_2_LOFF_SENS,
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_LOFF_SENS]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_2_LOFF_STAT,
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_LOFF_STAT]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_2_RESP1,
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_RESP1]);
      CFG_WRITE_INT(&cfgFile, CFG_KEY_EXG_2_RESP2,
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_RESP2]);

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

  LogAndStream_checkSdInSlot();
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();
  char cfgname[] = CFG_FILENAME;
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
      if (cfg_key_is(buffer, CFG_KEY_ACCEL))
      {
        storedConfigPtr->chEnLnAccel = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_GYRO))
      {
        storedConfigPtr->chEnGyro = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_MAG))
      {
        storedConfigPtr->chEnMag = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG1_24BIT))
      {
        storedConfigPtr->chEnExg1_24Bit = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG2_24BIT))
      {
        storedConfigPtr->chEnExg2_24Bit = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_GSR))
      {
        storedConfigPtr->chEnGsr = atoi(equals);
      }
#if defined(SHIMMER3)
      else if (cfg_key_is(buffer, CFG_KEY_EXTCH7))
      {
        storedConfigPtr->chEnExtADC7 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXTCH6))
      {
        storedConfigPtr->chEnExtADC6 = atoi(equals);
      }
#elif defined(SHIMMER3R)
      else if (cfg_key_is(buffer, CFG_KEY_EXTCH0))
      {
        storedConfigPtr->chEnExtADC0 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXTCH1))
      {
        storedConfigPtr->chEnExtADC1 = atoi(equals);
      }
#endif
      else if (cfg_key_is(buffer, CFG_KEY_STR) || cfg_key_is(buffer, CFG_KEY_BR_AMP))
      {
        storedConfigPtr->chEnBridgeAmp = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_VBAT))
      {
        storedConfigPtr->chEnVBattery = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_ACCEL_D))
      {
        storedConfigPtr->chEnWrAccel = atoi(equals);
      }
#if defined(SHIMMER3)
      else if (cfg_key_is(buffer, CFG_KEY_EXTCH15))
      {
        storedConfigPtr->chEnExtADC15 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_INTCH1))
      {
        storedConfigPtr->chEnIntADC1 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_INTCH12))
      {
        storedConfigPtr->chEnIntADC12 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_INTCH13))
      {
        storedConfigPtr->chEnIntADC13 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_INTCH14))
      {
        storedConfigPtr->chEnIntADC14 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_ACCEL_MPU))
      {
        storedConfigPtr->chEnAltAccel = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_MAG_MPU))
      {
        storedConfigPtr->chEnAltMag = atoi(equals);
      }
#elif defined(SHIMMER3R)
      else if (cfg_key_is(buffer, CFG_KEY_EXTCH2))
      {
        storedConfigPtr->chEnExtADC2 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_INTCH3))
      {
        storedConfigPtr->chEnIntADC3 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_INTCH0))
      {
        storedConfigPtr->chEnIntADC0 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_INTCH1))
      {
        storedConfigPtr->chEnIntADC1 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_INTCH2))
      {
        storedConfigPtr->chEnIntADC2 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_ACCEL_ALT))
      {
        storedConfigPtr->chEnAltAccel = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_MAG_ALT))
      {
        storedConfigPtr->chEnAltMag = atoi(equals);
      }
#endif
      else if (cfg_key_is(buffer, CFG_KEY_EXG1_16BIT))
      {
        storedConfigPtr->chEnExg1_16Bit = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG2_16BIT))
      {
        storedConfigPtr->chEnExg2_16Bit = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_PRES))
      {
        storedConfigPtr->chEnPressureAndTemperature = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_SAMPLE_RATE))
      {
        sample_rate = atof(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_MG_INTERNAL_RATE))
      {
        ShimConfig_configByteMagRateSet(atoi(equals));
      }
#if defined(SHIMMER3)
      else if (cfg_key_is(buffer, CFG_KEY_MG_RANGE))
      {
        storedConfigPtr->magRange = atoi(equals);
      }
#else
      else if (cfg_key_is(buffer, CFG_KEY_MAG_ALT_RANGE))
      {
        storedConfigPtr->altMagRange = atoi(equals);
      }
#endif
      else if (cfg_key_is(buffer, CFG_KEY_ACC_INTERNAL_RATE))
      {
        storedConfigPtr->wrAccelRate = atoi(equals);
      }
#if defined(SHIMMER3)
      else if (cfg_key_is(buffer, CFG_KEY_ACCEL_ALT_RANGE))
      {
        storedConfigPtr->altAccelRange = atoi(equals);
      }
#elif defined(SHIMMER3R)
      else if (cfg_key_is(buffer, CFG_KEY_ACCEL_LN_RANGE))
      {
        storedConfigPtr->lnAccelRange = atoi(equals);
      }
#endif
      else if (cfg_key_is(buffer, CFG_KEY_ACC_RANGE))
      {
        storedConfigPtr->wrAccelRange = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_ACC_LPM))
      {
        ShimConfig_wrAccelLpModeSet(atoi(equals));
      }
      else if (cfg_key_is(buffer, CFG_KEY_ACC_HRM))
      {
        storedConfigPtr->wrAccelHrMode = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_GSR_RANGE))
      { //or "gsr_range="?
        gsr_range = atoi(equals);
        if (gsr_range > 4)
        {
          gsr_range = 4;
        }

        storedConfigPtr->gsrRange = gsr_range;
      }
      else if (cfg_key_is(buffer, CFG_KEY_GYRO_SAMPLINGRATE))
      {
        ShimConfig_gyroRateSet(atoi(equals));
      }
      else if (cfg_key_is(buffer, CFG_KEY_GYRO_RANGE))
      {
        ShimConfig_gyroRangeSet(atoi(equals));
      }
#if defined(SHIMMER3)
      else if (cfg_key_is(buffer, CFG_KEY_PRES_BMP180_PREC)
          || cfg_key_is(buffer, CFG_KEY_PRES_BMP280_PREC))
      {
        ShimConfig_configBytePressureOversamplingRatioSet(atoi(equals));
      }
#elif defined(SHIMMER3R)
      else if (cfg_key_is(buffer, CFG_KEY_PRES_BMP390_PREC))
      {
        ShimConfig_configBytePressureOversamplingRatioSet(atoi(equals));
      }
#endif

#if defined(SHIMMER3R)
      else if (cfg_key_is(buffer, CFG_KEY_MAG_ALT_RATE))
      {
        ShimConfig_configByteAltMagRateSet(atoi(equals));
      }
      else if (cfg_key_is(buffer, CFG_KEY_ACCEL_ALT_RATE))
      {
        storedConfigPtr->altAccelRate = atoi(equals);
      }
#endif
      else if (cfg_key_is(buffer, CFG_KEY_RTC_ERROR_ENABLE))
      {
        storedConfigPtr->rtcErrorEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (cfg_key_is(buffer, CFG_KEY_SD_ERROR_ENABLE))
      {
        storedConfigPtr->sdErrorEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (cfg_key_is(buffer, CFG_KEY_USER_BUTTON_ENABLE))
      {
        storedConfigPtr->userButtonEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (cfg_key_is(buffer, CFG_KEY_IAMMASTER))
      { //0=slave=node
        storedConfigPtr->masterEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (cfg_key_is(buffer, CFG_KEY_SYNC))
      {
        storedConfigPtr->syncEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (cfg_key_is(buffer, CFG_KEY_BT_DISABLED))
      {
        storedConfigPtr->bluetoothDisable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (cfg_key_is(buffer, CFG_KEY_LOW_BATT_AUTOSTOP))
      {
        storedConfigPtr->lowBatteryAutoStop = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (cfg_key_is(buffer, CFG_KEY_INTERVAL))
      {
        storedConfigPtr->btIntervalSecs = atoi(equals) > 255 ? 255 : atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXP_POWER))
      {
        storedConfigPtr->expansionBoardPower = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (cfg_key_is(buffer, CFG_KEY_CENTER))
      {
        ShimSdSync_parseSyncCenterNameFromCfgFile(&storedConfigPtr->rawBytes[0], equals);
      }
      else if (strstr(buffer, CFG_KEY_NODE))
      {
        ShimSdSync_parseSyncNodeNameFromCfgFile(&storedConfigPtr->rawBytes[0], equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EST_EXP_LEN))
      {
        est_exp_len = atoi(equals);
        storedConfigPtr->experimentLengthEstimatedInSecMsb = (est_exp_len & 0xff00) >> 8;
        storedConfigPtr->experimentLengthEstimatedInSecLsb = est_exp_len & 0xff;
      }
      else if (cfg_key_is(buffer, CFG_KEY_MAX_EXP_LEN))
      {
        max_exp_len = atoi(equals);
        storedConfigPtr->experimentLengthMaxInMinutesMsb = (max_exp_len & 0xff00) >> 8;
        storedConfigPtr->experimentLengthMaxInMinutesLsb = max_exp_len & 0xff;
      }
#if IS_SUPPORTED_SINGLE_TOUCH
      else if (cfg_key_is(buffer, CFG_KEY_SINGLETOUCH))
      {
        storedConfigPtr->singleTouchStart = (atoi(equals) == 0) ? 0 : 1;
      }
#endif //IS_SUPPORTED_SINGLE_TOUCH
      else if (cfg_key_is(buffer, CFG_KEY_MYID))
      {
        storedConfigPtr->myTrialID = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_NSHIMMER))
      {
        storedConfigPtr->numberOfShimmers = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_SHIMMERNAME))
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
      else if (cfg_key_is(buffer, CFG_KEY_EXPERIMENTID))
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
      else if (cfg_key_is(buffer, CFG_KEY_CONFIGTIME))
      {
        config_time = atol(equals);
        ShimConfig_configTimeSet(config_time);
      }

      else if (cfg_key_is(buffer, CFG_KEY_EXG_1_CONFIG1))
      {
        storedConfigPtr->exgADS1292rRegsCh1.config1 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_1_CONFIG2))
      {
        storedConfigPtr->exgADS1292rRegsCh1.config2 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_1_LOFF))
      {
        storedConfigPtr->exgADS1292rRegsCh1.loff = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_1_CH1SET))
      {
        storedConfigPtr->exgADS1292rRegsCh1.ch1set = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_1_CH2SET))
      {
        storedConfigPtr->exgADS1292rRegsCh1.ch2set = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_1_RLD_SENS))
      {
        storedConfigPtr->exgADS1292rRegsCh1.rldSens = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_1_LOFF_SENS))
      {
        storedConfigPtr->exgADS1292rRegsCh1.loffSens = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_1_LOFF_STAT))
      {
        storedConfigPtr->exgADS1292rRegsCh1.loffStat = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_1_RESP1))
      {
        storedConfigPtr->exgADS1292rRegsCh1.resp1 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_1_RESP2))
      {
        storedConfigPtr->exgADS1292rRegsCh1.resp2 = atoi(equals);
      }

      else if (cfg_key_is(buffer, CFG_KEY_EXG_2_CONFIG1))
      {
        storedConfigPtr->exgADS1292rRegsCh2.config1 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_2_CONFIG2))
      {
        storedConfigPtr->exgADS1292rRegsCh2.config2 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_2_LOFF))
      {
        storedConfigPtr->exgADS1292rRegsCh2.loff = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_2_CH1SET))
      {
        storedConfigPtr->exgADS1292rRegsCh2.ch1set = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_2_CH2SET))
      {
        storedConfigPtr->exgADS1292rRegsCh2.ch2set = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_2_RLD_SENS))
      {
        storedConfigPtr->exgADS1292rRegsCh2.rldSens = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_2_LOFF_SENS))
      {
        storedConfigPtr->exgADS1292rRegsCh2.loffSens = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_2_LOFF_STAT))
      {
        storedConfigPtr->exgADS1292rRegsCh2.loffStat = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_2_RESP1))
      {
        storedConfigPtr->exgADS1292rRegsCh2.resp1 = atoi(equals);
      }
      else if (cfg_key_is(buffer, CFG_KEY_EXG_2_RESP2))
      {
        storedConfigPtr->exgADS1292rRegsCh2.resp2 = atoi(equals);
      }

      else if (cfg_key_is(buffer, CFG_KEY_DERIVED_CHANNELS))
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
    memcpy((storedConfigPtr->gyroCalib.rawBytes), &(storedConfigTemp.gyroCalib.rawBytes),
        sizeof(storedConfigTemp.gyroCalib.rawBytes));
    memcpy((storedConfigPtr->magCalib.rawBytes), &(storedConfigTemp.magCalib.rawBytes),
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

    /* Copy back in the NV_SD_CONFIG_DELAY_FLAG byte. This carries forward whether the CFG and/or config files need to be updated. */
    //TODO Why would the CFG file need to be updated at this point when we are reading it?
    storedConfigPtr->rawBytes[NV_SD_CONFIG_DELAY_FLAG]
        = storedConfigTemp.rawBytes[NV_SD_CONFIG_DELAY_FLAG];

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
