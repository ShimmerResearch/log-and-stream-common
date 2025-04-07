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

#include <stdint.h>

#ifndef SHIMMER_CONFIG_H
#define SHIMMER_CONFIG_H

#include <log_and_stream_definitions.h>

#if defined(SHIMMER3R)
#include "lis2dw12_reg.h"
#include "shimmer_definitions.h"
#include <shimmer_include.h>
#endif

#define STOREDCONFIG_SIZE        512

//Infomem contents
//#define NV_NUM_CONFIG_BYTES             100
//Infomem contents
#define NV_NUM_SETTINGS_BYTES    34
#define NV_NUM_CALIBRATION_BYTES 84
#define NV_NUM_SD_BYTES          37
#define NV_TOTAL_NUM_CONFIG_BYTES \
  384 //NV_NUM_SETTINGS_BYTES + NV_NUM_CALIBRATION_BYTES + NV_NUM_SD_BYTES
#define NV_NUM_RWMEM_BYTES                  STOREDCONFIG_SIZE
#define NV_NUM_BYTES_SYNC_CENTER_NODE_ADDRS 126

#define NV_SAMPLING_RATE                    0
#define NV_BUFFER_SIZE                      2
#define NV_SENSORS0                         3
#define NV_SENSORS1                         4
#define NV_SENSORS2                         5
#define NV_CONFIG_SETUP_BYTE0               6 //sensors setting bytes
#define NV_CONFIG_SETUP_BYTE1               7
#define NV_CONFIG_SETUP_BYTE2               8
#define NV_CONFIG_SETUP_BYTE3               9
#define NV_EXG_ADS1292R_1_CONFIG1           10 //exg bytes, not implemented yet
#define NV_EXG_ADS1292R_1_CONFIG2           11
#define NV_EXG_ADS1292R_1_LOFF              12
#define NV_EXG_ADS1292R_1_CH1SET            13
#define NV_EXG_ADS1292R_1_CH2SET            14
#define NV_EXG_ADS1292R_1_RLD_SENS          15
#define NV_EXG_ADS1292R_1_LOFF_SENS         16
#define NV_EXG_ADS1292R_1_LOFF_STAT         17
#define NV_EXG_ADS1292R_1_RESP1             18
#define NV_EXG_ADS1292R_1_RESP2             19
#define NV_EXG_ADS1292R_2_CONFIG1           20
#define NV_EXG_ADS1292R_2_CONFIG2           21
#define NV_EXG_ADS1292R_2_LOFF              22
#define NV_EXG_ADS1292R_2_CH1SET            23
#define NV_EXG_ADS1292R_2_CH2SET            24
#define NV_EXG_ADS1292R_2_RLD_SENS          25
#define NV_EXG_ADS1292R_2_LOFF_SENS         26
#define NV_EXG_ADS1292R_2_LOFF_STAT         27
#define NV_EXG_ADS1292R_2_RESP1             28
#define NV_EXG_ADS1292R_2_RESP2             29
#define NV_BT_COMMS_BAUD_RATE               30
#define NV_DERIVED_CHANNELS_0               31
#define NV_DERIVED_CHANNELS_1               32
#define NV_DERIVED_CHANNELS_2               33
#define NV_LN_ACCEL_CALIBRATION             34
#define NV_GYRO_CALIBRATION                 55
#define NV_MAG_CALIBRATION                  76
#define NV_WR_ACCEL_CALIBRATION             97 //97->117
#define NV_CALIBRATION_END                  117
#define NV_DERIVED_CHANNELS_3               118
#define NV_DERIVED_CHANNELS_4               119
#define NV_DERIVED_CHANNELS_5               120
#define NV_DERIVED_CHANNELS_6               121
#define NV_DERIVED_CHANNELS_7               122

#define NV_SENSORS3                         (128 + 0)
#define NV_SENSORS4                         (128 + 1)
#define NV_CONFIG_SETUP_BYTE4               (128 + 2)
#define NV_CONFIG_SETUP_BYTE5               (128 + 3)
#define NV_CONFIG_SETUP_BYTE6               (128 + 4)
#define NV_ALT_ACCEL_CALIBRATION            (128 + 5)  //+21
#define NV_ALT_MAG_CALIBRATION              (128 + 26) //+21
#define NV_MPL_GYRO_CALIBRATION             (128 + 47) //+12
#define NV_SD_SHIMMER_NAME                  (128 + 59) //+12 bytes
#define NV_SD_EXP_ID_NAME                   (128 + 71) //+12 bytes
#define NV_SD_CONFIG_TIME                   (128 + 83) //+4 bytes
#define NV_SD_MYTRIAL_ID                    (128 + 87) //1 byte
#define NV_SD_NSHIMMER                      (128 + 88) //1 byte
#define NV_SD_TRIAL_CONFIG0                 (128 + 89)
#define NV_SD_TRIAL_CONFIG1                 (128 + 90)
#define NV_SD_BT_INTERVAL                   (128 + 91)
#define NV_EST_EXP_LEN_MSB                  (128 + 92) //2bytes
#define NV_EST_EXP_LEN_LSB                  (128 + 93)
#define NV_MAX_EXP_LEN_MSB                  (128 + 94) //2bytes
#define NV_MAX_EXP_LEN_LSB                  (128 + 95)
#define NV_MAC_ADDRESS                      (128 + 96) //6bytes
#define NV_SD_CONFIG_DELAY_FLAG             (128 + 102)
#define NV_BT_SET_PIN                       (128 + 103)
//#define NV_TEMP_PRES_CALIBRATION            (128 + 104) //+22 bytes, till 128+125

#define NV_CENTER                           (128 + 128 + 0)
#define NV_NODE0                            (128 + 128 + 6)

typedef union
{
  uint8_t rawBytes[21];

  struct __attribute__((packed))
  {
    int16_t calibOffset_B0;
    int16_t calibOffset_B1;
    int16_t calibOffset_B2;
    int16_t calibSensitivity_K0;
    int16_t calibSensitivity_K1;
    int16_t calibSensitivity_K2;
    int8_t calibAlignment_R00;
    int8_t calibAlignment_R01;
    int8_t calibAlignment_R02;
    int8_t calibAlignment_R10;
    int8_t calibAlignment_R11;
    int8_t calibAlignment_R12;
    int8_t calibAlignment_R20;
    int8_t calibAlignment_R21;
    int8_t calibAlignment_R22;
  };
} gImuConfig;

typedef union
{
  uint8_t rawBytes[10];

  struct __attribute__((packed))
  {
    uint8_t config1;
    uint8_t config2;
    uint8_t loff;
    uint8_t ch1set;
    uint8_t ch2set;
    uint8_t rldSens;
    uint8_t loffSens;
    uint8_t loffStat;
    uint8_t resp1;
    uint8_t resp2;
  };
} gExgADS1292rRegs;

typedef union
{
  uint8_t rawBytes[STOREDCONFIG_SIZE];

  struct __attribute__((packed))
  {
    //cfg in common
    uint16_t samplingRateTicks;
    uint8_t bufferSize;

    //Idx 3: Sensors 0
#if defined(SHIMMER3)
    uint8_t chEnExtADC6 : 1;
    uint8_t chEnExtADC7 : 1;
#else
    uint8_t chEnExtADC1 : 1; //S3 = ADC6, S3R = ADC11, S4_SDK = ADC8
    uint8_t chEnExtADC0 : 1; //S3 = ADC7, S3R = ADC9, S4_SDK = ADC9
#endif
    uint8_t chEnGsr        : 1; //S3 = IntADC1, S3R = IntADC3
    uint8_t chEnExg2_24Bit : 1;
    uint8_t chEnExg1_24Bit : 1;
    uint8_t chEnMag  : 1; //S3/S4_SDK = LSM303DLHC/LSM303AH, S3R = LIS3MDL Mag
    uint8_t chEnGyro : 1; //S3/S4_SDK = MPU9x50/ICM20948, S3R = LSM6DSV Gyro
    uint8_t chEnLnAccel : 1; //S3/S4_SDK = KXRB5-2042/KXTC9-2050, S3R = LSM6DSV Accel

    //Idx 4: Sensors 1
#if defined(SHIMMER3)
    uint8_t chEnIntADC13 : 1;
    uint8_t chEnIntADC12 : 1;
    uint8_t chEnIntADC1  : 1;
    uint8_t chEnExtADC15 : 1;
#else
    uint8_t chEnIntADC1 : 1; //S3 = ADC13, S3R = ADC15, S4_SDK = ADC11
    uint8_t chEnIntADC0 : 1; //S3 = ADC12, S3R = ADC10, S4_SDK = ADC10
    uint8_t chEnIntADC3 : 1; //S3 = ADC1, S3R = ADC17, S4_SDK = ADC2
    uint8_t chEnExtADC2 : 1; //S3 = ADC15, S3R = ADC12, S4_SDK = ADC1
#endif
    uint8_t chEnWrAccel : 1; //S3/S4_SDK = LSM303DLHC/LSM303AH, S3R = LIS2DW12 Accel
    uint8_t chEnVBattery : 1;
#if defined(SHIMMER4_SDK)
    uint8_t chEnPpgApp : 1;
#elif defined(SHIMMER3R)
    uint8_t chEnMicrophone : 1;
#else
    uint8_t unusedIdx4Bit6 : 1;
#endif
    uint8_t chEnBridgeAmp : 1; //S3 = IntADC13 & IntADC14, S3R = IntADC1 & IntADC2

    //Idx 5: Sensors 2
#if defined(SHIMMER3)
    uint8_t unusedIdx5Bit0  : 1;
    uint8_t chEnMPU9x50temp : 1; //Shimmer3 MPU9x50/ICM20948 temperature
#elif defined(SHIMMER3R)
    uint8_t unusedIdx5Bit0 : 1;
    uint8_t unusedIdx5Bit1 : 1;
#elif defined(SHIMMER4_SDK)
    uint8_t chEnStc3100 : 1;
    uint8_t chEnIntADC4 : 1; //S4_SDK = ADC12
#endif
    uint8_t chEnPressureAndTemperature : 1;
    uint8_t chEnExg2_16Bit             : 1;
    uint8_t chEnExg1_16Bit             : 1;
    uint8_t chEnAltMag   : 1; //S3/S4_SDK MPU9x50/ICM20948 Mag, S3R = LIS2MDL
    uint8_t chEnAltAccel : 1; //S3/S4_SDK MPU9x50/ICM20948 Accel, S3R = ADXL371
#if defined(SHIMMER3)
    uint8_t chEnIntADC14 : 1;
#else
    uint8_t chEnIntADC2 : 1; //S3 = ADC14, S3R = ADC16, S4_SDK = ADC0
#endif

    //Idx 6: Config setup byte 0
    uint8_t wrAccelHrMode    : 1;
    uint8_t wrAccelLpModeLsb : 1;
    uint8_t wrAccelRange     : 2;
    uint8_t wrAccelRate      : 4;

    //Idx 7: Config setup byte 1
    uint8_t gyroRate;

    //Idx 8: Config setup byte 2
    uint8_t gyroRangeLsb : 2;
    uint8_t magRateLsb   : 3;
    uint8_t magRange     : 3;

    //Idx 9: Config setup byte 3
    uint8_t expansionBoardPower          : 1;
    uint8_t gsrRange                     : 3;
    uint8_t pressureOversamplingRatioLsb : 2;
#if defined(SHIMMER3)
    uint8_t altAccelRange : 2; //S3/S4_SDK MPU9x50/ICM20948 Accel
#else
    uint8_t lnAccelRange : 2; //S3R = LSM6DSV Accel
#endif
    gExgADS1292rRegs exgADS1292rRegsCh1;
    gExgADS1292rRegs exgADS1292rRegsCh2;

    uint8_t btCommsBaudRate;

    //Idx 31: Derived Channels 0
    uint8_t chEnResAmp   : 1;
    uint8_t chEnSkinTemp : 1;
    uint8_t chEnPpg      : 1; //ppg_12.13
    uint8_t chEnPpg1     : 1; //ppg1_12.13
    uint8_t chEnPpg2     : 1; //ppg2_1.14
    uint8_t chEnPpgtoHr  : 1; //ppgtoHr_12.13
    uint8_t chEnPpgToHr1 : 1; //ppgtoHr1_12.13
    uint8_t chEnPpgToHr2 : 1; //ppgtoHr2_1.14

    //Idx 32: Derived Channels 1
    uint8_t chEnGsrMetricsGeneral : 1;
    uint8_t chEnActivity          : 1;
    uint8_t chEnHrVFreq           : 1;
    uint8_t chEnHrVTime           : 1;
    uint8_t chEnEcg2HrChp2Ch2     : 1;
    uint8_t chEnEcg2HrChp2Ch1     : 1;
    uint8_t chEnEcg2HrChp1Ch2     : 1;
    uint8_t chEnEcg2HrChp1Ch1     : 1;

    //Idx 33: Derived Channels 2
    uint8_t chEnNineDofWrQuat  : 1;
    uint8_t chEnNineDofWrEuler : 1;
    uint8_t chEnSixDofWrQuat   : 1;
    uint8_t chEnSixDofWrEuler  : 1;
    uint8_t chEnNineDofLnQuat  : 1;
    uint8_t chEnNineDofLnEuler : 1;
    uint8_t chEnSixDofLnQuat   : 1;
    uint8_t chEnSixDofLnEuler  : 1;

    gImuConfig lnAccelCalib;
    gImuConfig gyroCalib;
    gImuConfig magCalib;
    gImuConfig wrAccelCalib;

    //Idx 118: Derived Channels 3
    uint8_t chEnEmgProcessingCh2    : 1;
    uint8_t chEnEmgProcessingCh1    : 1;
    uint8_t chEnGsrBaseline         : 1;
    uint8_t chEnGsrMetricsTrendPeak : 1;
    uint8_t chEnGaitModule          : 1;
    uint8_t chEnGyroOnTheFlyCalib   : 1;
    uint8_t unusedByte118Bit6       : 1;
    uint8_t unusedByte118Bit7       : 1;

    uint8_t derivedChannels4;
    uint8_t derivedChannels5;
    uint8_t derivedChannels6;
    uint8_t derivedChannels7;

    uint8_t unusedIdx123;
    uint8_t unusedIdx124;
    uint8_t unusedIdx125;
    uint8_t unusedIdx126;
    uint8_t unusedIdx127;

    //Idx 128: Sensors3
    uint8_t unusedIdx128;

    //Idx 129: Sensors4
    uint8_t unusedIdx129;

    //Idx 130: Config setup byte 4
    uint8_t pressureOversamplingRatioMsb : 1;
    uint8_t wrAccelLpModeMsb             : 1;
    uint8_t gyroRangeMsb                 : 1;
    uint8_t unusedByte130Bit4            : 1;
    uint8_t altMagRate                   : 2;
    uint8_t altAccelRate                 : 2;

    //Idx 131: Config setup byte 5
    uint8_t unusedByte131Bit0 : 1;
    uint8_t unusedByte131Bit1 : 1;
    uint8_t unusedByte131Bit2 : 1;
    uint8_t magRateMsb        : 3;
    uint8_t unusedByte131Bit6 : 1;
    uint8_t unusedByte131Bit7 : 1;

    //Idx 132: Config setup byte 6
    uint8_t unusedIdx132;

    gImuConfig altAccelCalib;
    gImuConfig altMagCalib;

    uint8_t unusedIdx175To186[12];

    char shimmerName[12];
    char expIdName[12];
    uint32_t configTime;
    uint8_t myTrialID;
    uint8_t numberOfShimmers;

    //SDTrialConfig0
    uint8_t sdErrorEnable    : 1;
    uint8_t masterEnable     : 1;
    uint8_t syncEnable       : 1;
    uint8_t bluetoothDisable : 1;
    uint8_t rtcErrorEnable   : 1;
    uint8_t userButtonEnable : 1;
    uint8_t btPinSetup       : 1;
    uint8_t rtcSetByBt       : 1;

    //SDTrialConfig1
    uint8_t lowBatteryAutoStop : 1;
    uint8_t unusedIdx218Bit1   : 1;
    uint8_t unusedIdx218Bit2   : 1;
    uint8_t unusedIdx218Bit3   : 1;
    uint8_t tcxo               : 1;
    uint8_t unusedIdx218Bit5   : 1;
    uint8_t unusedIdx218Bit6   : 1;
    uint8_t singleTouchStart   : 1;

    uint8_t btIntervalSecs;
    uint16_t experimentLengthEstimatedInSec; //Used for SD Sync (min = 1)
    uint16_t experimentLengthMaxInMinutes;   //Used for auto-stop (0ff = 0)
    uint8_t macAddr[6];

    //SDConfigDelayFlag;
    uint8_t infoSdcfg : 1;
    /* Used to be used by older, individual calibration files. Replaced by calib
     * dump file*/
    uint8_t infoCalib        : 1;
    uint8_t unusedIdx230Bit2 : 1;
    uint8_t unusedIdx230Bit3 : 1;
    uint8_t unusedIdx230Bit4 : 1;
    uint8_t unusedIdx230Bit5 : 1;
    uint8_t unusedIdx230Bit6 : 1;
    uint8_t sdCfgFlag        : 1;

    uint8_t btSetPin;
    uint8_t unusedIdx232;
    uint8_t unusedIdx233;
    uint8_t unusedIdx234;
    uint8_t unusedIdx235;
    uint8_t unusedIdx236;
    uint8_t unusedIdx237;
    uint8_t unusedIdx238;
    uint8_t unusedIdx239;
    uint8_t unusedIdx240;
    uint8_t unusedIdx241;
    uint8_t unusedIdx242;
    uint8_t unusedIdx243;
    uint8_t unusedIdx244;
    uint8_t unusedIdx245;
    uint8_t unusedIdx246;
    uint8_t unusedIdx247;
    uint8_t unusedIdx248;
    uint8_t unusedIdx249;
    uint8_t unusedIdx250;
    uint8_t unusedIdx251;
    uint8_t unusedIdx252;
    uint8_t unusedIdx253;
    uint8_t unusedIdx254;
    uint8_t unusedIdx255;

    //cfg for sync //put it as individual bytes
    uint8_t syncNodeAddr1[6];
    uint8_t syncNodeAddr2[6];
    uint8_t syncNodeAddr3[6];
    uint8_t syncNodeAddr4[6];
    uint8_t syncNodeAddr5[6];
    uint8_t syncNodeAddr6[6];
    uint8_t syncNodeAddr7[6];
    uint8_t syncNodeAddr8[6];
    uint8_t syncNodeAddr9[6];
    uint8_t syncNodeAddr[6];
    uint8_t syncNodeAddr11[6];
    uint8_t syncNodeAddr12[6];
    uint8_t syncNodeAddr13[6];
    uint8_t syncNodeAddr14[6];
    uint8_t syncNodeAddr15[6];
    uint8_t syncNodeAddr16[6];
    uint8_t syncNodeAddr17[6];
    uint8_t syncNodeAddr18[6];
    uint8_t syncNodeAddr19[6];
    uint8_t syncNodeAddr20[6];
    uint8_t syncNodeAddr21[6];

    uint8_t unusedIdx382;
    uint8_t unusedIdx383;

    uint8_t padding[128];
  };
} gConfigBytes;

void ShimConfig_reset(void);
void ShimConfig_readRam(void);
gConfigBytes *ShimConfig_getStoredConfig(void);

uint8_t ShimConfig_storedConfigSet(const uint8_t *buf, uint16_t offset, uint16_t length);
uint8_t ShimConfig_storedConfigGet(uint8_t *buf, uint16_t offset, uint16_t length);
uint8_t ShimConfig_storedConfigGetByte(uint16_t offset);
uint8_t ShimConfig_storedConfigSetByte(uint16_t offset, uint8_t val);

void ShimConfig_setDefaultConfig(void);

void ShimConfig_setDefaultShimmerName(void);
void ShimConfig_setDefaultTrialId(void);
uint8_t ShimConfig_getSdCfgFlag(void);
void ShimConfig_setSdCfgFlag(uint8_t flag);
uint8_t ShimConfig_getCalibFlag();
void ShimConfig_setCalibFlag(uint8_t flag);
uint8_t ShimConfig_getRamCalibFlag(void);
void ShimConfig_setRamCalibFlag(uint8_t flag);
float ShimConfig_getShimmerSamplingFreq(void);

void ShimConfig_gyroRangeSet(gConfigBytes *storedConfigPtr, uint8_t value);
uint8_t ShimConfig_gyroRangeGet(void);
void ShimConfig_gyroRateSet(gConfigBytes *storedConfigPtr, uint8_t value);
void ShimConfig_wrAccelLpModeSet(gConfigBytes *storedConfigPtr, uint8_t value);
uint8_t ShimConfig_wrAccelLpModeGet(void);
#if defined(SHIMMER3R)
void ShimConfig_wrAccelModeSet(gConfigBytes *storedConfigPtr, lis2dw12_mode_t value);
lis2dw12_mode_t ShimConfig_wrAccelModeGet(void);
#endif
void ShimConfig_configBytePressureOversamplingRatioSet(gConfigBytes *storedConfigPtr,
    uint8_t value);
uint8_t ShimConfig_configBytePressureOversamplingRatioGet(void);
void ShimConfig_configByteMagRateSet(gConfigBytes *storedConfigPtr, uint8_t value);
uint8_t ShimConfig_configByteMagRateGet(void);
uint8_t ShimConfig_checkAndCorrectConfig(gConfigBytes *storedConfig);
void ShimConfig_setExgConfigForTestSignal(gConfigBytes *storedConfigPtr);
void ShimConfig_setExgConfigForEcg(gConfigBytes *storedConfigPtr);
void ShimConfig_experimentLengthSecsMaxSet(uint16_t expLengthMins);
uint16_t ShimConfig_experimentLengthSecsMaxGet(void);
void ShimConfig_experimentLengthCntReset(void);
uint8_t ShimConfig_checkAutostopCondition(void);
uint16_t ShimConfig_freqDiv(float samplingRate);
void ShimConfig_checkBtModeFromConfig(void);
#if defined(SHIMMER3R)
uint8_t ShimConfig_isMicrophoneEnabled(void);
#endif
uint8_t ShimConfig_isGSREnabled(void);
uint8_t ShimConfig_isExpansionBoardPwrEnabled(void);
void ShimConfig_loadSensorConfigAndCalib(void);
gConfigBytes ShimConfig_createBlankConfigBytes(void);
uint8_t ShimConfig_areConfigBytesValid(void);

void ShimConfig_setShimmerName(void);
void ShimConfig_setExpIdName(void);
void ShimConfig_setCfgTime(void);
void ShimConfig_setConfigTimeTextIfEmpty(void);
void ShimConfig_infomem2Names(void);
char *ShimConfig_shimmerNamePtrGet(void);
char *ShimConfig_expIdPtrGet(void);
char *ShimConfig_configTimeTextPtrGet(void);

#endif //SHIMMER_CONFIG_H
