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

#ifndef SHIMMER_SENSING_H
#define SHIMMER_SENSING_H

#include <stdint.h>

#include "log_and_stream_externs.h"

#if defined(SHIMMER3R)
#include "shimmer_definitions.h"
#include "shimmer_include.h"
#endif

#define SAVE_DATA_FROM_RTC_INT 0x0

#define FIRST_CH_BYTE_IDX      (1 + 3) //0x00 + timestamp

#if defined(SHIMMER3)
/* 3xanalogAccel + 3xdigiGyro + 3xdigiMag +
 * 3xLSM303DLHCAccel + 3xMPU9150Accel + 3xMPU9150MAG +
 * BMPX80TEMP + BMPX80PRESS + batteryVoltage +
 * 3xexternalADC + 4xinternalADC + (ExG) */
#define MAX_NUM_CHANNELS 45
#elif defined(SHIMMER3R)
/* Approximately:
 * 3xAccel1 + 3xAccel2 + 3xAccel3 + 3xdigiGyro + 3xMag1 + 3xMag2
 * + BMPTEMP + BMPPRESS + 3xExtADC + 5xIntADC + 10xExG = 38
 * Note: ExG can't be enabled at the same time as internal ADC channels so this
 * number incorporates some buffer. Similarly, not all models have all IMUs
 * fitted. */
#define MAX_NUM_CHANNELS 50
#elif defined(SHIMMER4_SDK)
/* 3xanalogAccel + 3xdigiGyro + 3xdigiMag +
 * 3xLSM303DLHCAccel + 3xMPU9250Accel + 3xMPU9250MAG +
 * BMP180TEMP + BMP180PRESS + batteryVoltage + 5stc3100 +
 * 3xexternalADC + 5xinternalADC */
#define MAX_NUM_CHANNELS 34
#endif

//Streaming Channel contents
#define X_LN_ACCEL 0x00
#define Y_LN_ACCEL 0x01
#define Z_LN_ACCEL 0x02
#define VBATT      0x03
#define X_WR_ACCEL 0x04
#define Y_WR_ACCEL 0x05
#define Z_WR_ACCEL 0x06
#define X_MAG      0x07
#define Y_MAG      0x08
#define Z_MAG      0x09
#define X_GYRO     0x0A
#define Y_GYRO     0x0B
#define Z_GYRO     0x0C
#if defined(SHIMMER3R)
#define EXTERNAL_ADC_0 0x0D
#define EXTERNAL_ADC_1 0x0E
#define EXTERNAL_ADC_2 0x0F
#define INTERNAL_ADC_3 0x10
#define INTERNAL_ADC_0 0x11
#define INTERNAL_ADC_1 0x12
#define INTERNAL_ADC_2 0x13
#elif defined(SHIMMER3)
#define EXTERNAL_ADC_7  0x0D
#define EXTERNAL_ADC_6  0x0E
#define EXTERNAL_ADC_15 0x0F
#define INTERNAL_ADC_1  0x10
#define INTERNAL_ADC_12 0x11
#define INTERNAL_ADC_13 0x12
#define INTERNAL_ADC_14 0x13
#endif
#define X_ALT_ACCEL              0x14
#define Y_ALT_ACCEL              0x15
#define Z_ALT_ACCEL              0x16
#define X_ALT_MAG                0x17
#define Y_ALT_MAG                0x18
#define Z_ALT_MAG                0x19
#define BMP_TEMPERATURE          0x1A
#define BMP_PRESSURE             0x1B
#define GSR_RAW                  0x1C
#define EXG_ADS1292R_1_STATUS    0x1D
#define EXG_ADS1292R_1_CH1_24BIT 0x1E
#define EXG_ADS1292R_1_CH2_24BIT 0x1F
#define EXG_ADS1292R_2_STATUS    0x20
#define EXG_ADS1292R_2_CH1_24BIT 0x21
#define EXG_ADS1292R_2_CH2_24BIT 0x22
#define EXG_ADS1292R_1_CH1_16BIT 0x23
#define EXG_ADS1292R_1_CH2_16BIT 0x24
#define EXG_ADS1292R_2_CH1_16BIT 0x25
#define EXG_ADS1292R_2_CH2_16BIT 0x26
#define STRAIN_HIGH              0x27
#define STRAIN_LOW               0x28
#if defined(SHIMMER4_SDK)
#define STC3100_CH_1    0x29
#define STC3100_CH_2    0x2A
#define STC3100_CH_3    0x2B
#define STC3100_CH_4    0x2C
#define STC3100_CH_5    0x2D
#define EXTERNAL_ADC_0  0x2E
#define EXTERNAL_ADC_1  0x2F
#define EXTERNAL_ADC_15 0x30
#define INTERNAL_ADC_1  0x31
#define INTERNAL_ADC_12 0x32
#define INTERNAL_ADC_13 0x33
#define INTERNAL_ADC_14 0x34
#define INT_ADC_3       0x35
#define PPG_1           0x36
#define PPG_2           0x37
#endif
#define DATA_BUF_SIZE 256U /* serial buffer in bytes (power 2)  */

typedef struct
{ //data ptr (offset)
  uint8_t ts;
  uint8_t temperature;
  uint8_t pressure;
  uint8_t accel2; //"WR Accel"
  uint8_t mag1;   //"Mag"
  uint8_t gyro;
  uint8_t accel3; //"Alt Accel"
  uint8_t mag2;   //"Alt Mag"
  uint8_t accel1; //"LN Accel"
#if defined(SHIMMER4_SDK)
  uint8_t stc3100Batt;
#endif
  uint8_t exg1;
  uint8_t exg2;
  uint8_t gsr;
  uint8_t strainGauge;
  uint8_t ppg;
  uint8_t extADC0;
  uint8_t extADC1;
  uint8_t extADC2;
  uint8_t intADC0;
#if defined(SHIMMER4_SDK)
  uint8_t intADC4;
#endif
  uint8_t intADC1;
  uint8_t intADC2;
  uint8_t intADC3;
  uint8_t batteryAnalog;
} DATAPTRTypeDef;

typedef enum
{
  SAMPLING_COMPLETE = 0x00,
  SAMPLING_IN_PROGRESS = 0x01,
  SAMPLE_NOT_READY = 0x02
} SAMPLING_STATUS;

typedef struct
{ //sensor data
  volatile uint8_t isSampling;
  uint64_t latestTs;
  uint8_t dataBuf[DATA_BUF_SIZE];
  uint8_t rdIdx;
  uint8_t wrIdx;
} PACKETBufferTypeDef;

typedef struct
{ //sensor data
  //uint8_t     en;
  //uint8_t     configuring;
  //uint8_t     i2cSensor;
  volatile uint8_t isSampling;
  uint8_t isSdOperating;
  uint8_t isFileCreated;
  uint8_t inSdWr;
  uint16_t inSdWrCnt;
  float freq;
  uint16_t clkInterval4096;
  uint16_t clkInterval16k;
  uint8_t status;
  uint8_t cc[MAX_NUM_CHANNELS]; //channelContents
  uint8_t ccLen;                //channelContentsLength
  uint8_t nbrMcuAdcChans;
  uint8_t nbrI2cChans;
  uint8_t nbrSpiChans;
  uint8_t dataLen;
  //uint8_t     sdlogEnabled;
  //uint8_t     btStreamEnabled;
  uint64_t startTs;
  uint64_t latestTs;
  volatile uint8_t packetBufferIdx;
  PACKETBufferTypeDef packetBuffers[2];
  //STATTypeDef stat;
  DATAPTRTypeDef ptr;
} SENSINGTypeDef;

extern SENSINGTypeDef sensing;

void ShimSens_init(void);
SENSINGTypeDef *ShimSens_getSensing(void);
void ShimSens_configureChannels(void);
uint8_t ShimSens_checkStartLoggingConditions(void);
uint8_t ShimSens_checkStartStreamingConditions(void);
uint8_t ShimSens_shouldStopLogging(void);
uint8_t ShimSens_shouldStopStreaming(void);
void ShimSens_startSensing(void);
void ShimSens_stopSensing(uint8_t enableDockUartIfDocked);
void ShimSens_stopPeripherals(void);
void ShimSens_stopSensingWrapup(void);
void ShimSens_streamData(void);
void ShimSens_bufPoll(void);
void ShimSens_saveTimestampToPacket(void);
void ShimSens_gatherData(void);

void ShimSens_stepInit(void);
void ShimSens_adcCompleteCb(void);
void ShimSens_i2cCompleteCb(void);
void ShimSens_spiCompleteCb(void);
void ShimSens_stageCompleteCb(uint8_t stage);
#if defined(SHIMMER4_SDK)
void ShimSens_step1Start(void);
void ShimSens_step2Start(void);
void ShimSens_step3Start(void);
void ShimSens_step4Start(void);
void ShimSens_step5Start(void);
#endif
void ShimSens_stepDone(void);

void ShimSens_saveData(void);

uint8_t ShimSens_getNumEnabledChannels(void);
void ShimSens_startLoggingIfUndockStartEnabled(void);

uint8_t ShimSens_checkAutostopLoggingCondition(void);
void ShimSens_currentExperimentLengthReset(void);
void ShimSens_maxExperimentLengthSecsSet(uint16_t expLengthMins);

#endif //SHIMMER_SENSING_H
