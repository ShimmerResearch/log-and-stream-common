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

#include <Sensing/shimmer_sensing.h>

#include <log_and_stream_externs.h>
#include <log_and_stream_includes.h>

#if defined(SHIMMER3)
#include "../../Shimmer_Driver/RN4X/RN4X.h"
#elif defined(SHIMMER3R)
#include "shimmer_definitions.h"
#endif

SENSINGTypeDef sensing;
//static STATTypeDef * pStat;

#if defined(SHIMMER3R)
ADCTypeDef *sensing_adc;
I2C_TypeDef *sensing_i2c;
I2C_TypeDef *sensing_i2c_batt;
#endif

volatile uint8_t expectedCbFlags = 0;
volatile uint8_t currentCbFlags = 0;
#if defined(SHIMMER4_SDK)
uint32_t temp_cnt1, temp_cnt2, temp_cnt3, temp_cnt4;
#endif
//I2CBatteryTypeDef *sensing_i2c_batt;

uint32_t maxExpLenSecs, currentExpLenSecs;

void ShimSens_init(void)
{
  memset((uint8_t *) &sensing, 0, sizeof(sensing));

  ShimSens_currentExperimentLengthReset();
  ShimSens_maxExperimentLengthSecsSet(0);
}

SENSINGTypeDef *ShimSens_getSensing(void)
{
  return &sensing;
}

void ShimSens_configureChannels(void)
{
  sensing.nbrMcuAdcChans = sensing.nbrI2cChans = sensing.nbrSpiChans = 0;
  sensing.ccLen = 0;
  sensing.ptr.ts = 1;
  sensing.dataLen = FIRST_CH_BYTE_IDX;

  if (ShimBrd_areMcuAdcsUsedForSensing())
  {
    ADC_configureChannels();
  }
  I2C_configureChannels();
  SPI_configureChannels();

#if defined(SHIMMER3)
  calculateClassicBtTxSampleSetBufferSize(
      sensing.dataLen, ShimConfig_getStoredConfig()->samplingRateTicks);
#endif

  expectedCbFlags = 0;
  if (sensing.nbrMcuAdcChans)
  {
    expectedCbFlags |= STAT_PERI_ADC;
  }
  if (sensing.nbrI2cChans)
  {
    expectedCbFlags |= STAT_PERI_I2C_SENS;
  }
  if (sensing.nbrSpiChans)
  {
    expectedCbFlags |= STAT_PERI_SPI_SENS;
  }
}

/* Check if the conditions are met to start logging to SD card.
 * Return 1 if conditions are met, 0 otherwise.
 */
uint8_t ShimSens_checkStartLoggingConditions(void)
{
  return shimmerStatus.sdInserted && !shimmerStatus.sdBadFile
      && shimmerStatus.sdlogReady && !shimmerStatus.sdLogging;
}

/* Check if the conditions are met to start streaming to BT.
 * Return 1 if conditions are met, 0 otherwise.
 */
uint8_t ShimSens_checkStartStreamingConditions(void)
{
  return shimmerStatus.btConnected && shimmerStatus.btstreamReady
      && !shimmerStatus.btStreaming;
}

/* Check if the conditions are met to stop logging to SD card.
 * Return 1 if conditions are met, 0 otherwise.
 */
uint8_t ShimSens_checkStopLoggingConditions(void)
{
  return !shimmerStatus.sdlogReady || (shimmerStatus.sdlogCmd == SD_LOG_CMD_STATE_STOP);
}

/* Check if the conditions are met to stop streaming to BT.
 * Return 1 if conditions are met, 0 otherwise.
 */
uint8_t ShimSens_checkStopStreamingConditions(void)
{
  return !shimmerStatus.btstreamReady
      || (shimmerStatus.btstreamCmd == BT_STREAM_CMD_STATE_STOP);
}

void ShimSens_startSensing(void)
{
  shimmerStatus.configuring = 1;

  uint8_t sdLogPendingStart = (shimmerStatus.sdlogCmd == SD_LOG_CMD_STATE_START)
      && ShimSens_checkStartLoggingConditions();
  uint8_t streamPendingStart = (shimmerStatus.btstreamCmd == BT_STREAM_CMD_STATE_START)
      && ShimSens_checkStartStreamingConditions();

  if (sdLogPendingStart || streamPendingStart)
  {
    shimmerStatus.sensing = 1;
    sensing.isFileCreated = 0;
    sensing.isSampling = SAMPLE_NOT_READY;
    ShimSens_configureChannels();
    if (ShimSens_getNumEnabledChannels() == 0)
    {
      shimmerStatus.configuring = 0;
      shimmerStatus.sensing = 0;
      return;
    }

    memset(sensing.dataBuf, 0, sizeof(sensing.dataBuf));
    /* Not needed due to memset above but explicitly setting DATA_PACKET for
     * clarity. */
    sensing.dataBuf[0] = DATA_PACKET;

#if defined(SHIMMER3R)
    Board_enableSensingPower(SENSE_PWR_SENSING, 1);
#endif

    if (ShimConfig_isExpansionBoardPwrEnabled())
    {
      Board_setExpansionBrdPower(1);
    }

    uint16_t samplingRateTicks = ShimConfig_getStoredConfig()->samplingRateTicks;
    sensing.freq = ShimConfig_getShimmerSamplingFreq();
    if (sensing.freq > 4096.0)
    { //Please don't go too fast, Thx, Best Regards.
      shimmerStatus.configuring = 0;
      shimmerStatus.sensing = 0;
      return;
    }
    sensing.clkInterval4096 = (uint16_t) 4096
        / sensing.freq; //216000000 = 8192*26367 or 108000000 = 4096*26367
    sensing.clkInterval16k = samplingRateTicks / 2;

    if (shimmerStatus.docked)
    {
      DockUart_disable();
    }
    ShimSens_stepInit();

    if (sensing.nbrMcuAdcChans)
    {
      ADC_startSensing();
    }
    I2C_startSensing();
    SPI_startSensing();

#if defined(SHIMMER3R)
    if (ShimConfig_isMicrophoneEnabled())
    {
      //TODO remove IF when fully switched from eval board to BGA variant
#ifdef S3R_NUCLEO
      MX_MDF1_Init();
#else
      MX_ADF1_Init();
      micStartSensing();
#endif
    }
#endif

#if defined(SHIMMER3)
    SampleTimerStart();
#else
    RTC_wakeUpSet(samplingRateTicks);
#endif

    sensing.startTs = RTC_get64();
  }

  //If the conditions are met, start logging to SD card.
  if (sdLogPendingStart)
  {
    shimmerStatus.sdLogging = 1;
    shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_IDLE;

    gConfigBytes *configBytesPtr = ShimConfig_getStoredConfig();

    //Setup auto-stop if enabled
    uint16_t maxExpLenMins = ShimConfig_experimentLengthMaxInMinutesGet();
    ShimSens_maxExperimentLengthSecsSet(maxExpLenMins);
    ShimSens_currentExperimentLengthReset();

    ShimSdDataFile_fileInit();

    if (shimmerStatus.sdSyncEnabled)
    {
      ShimSdSync_start(configBytesPtr->masterEnable,
          ShimConfig_experimentLengthEstimatedInSecGet());
    }
  }
  //If the conditions are met, start streaming to BT.
  if (streamPendingStart)
  {
    shimmerStatus.btStreaming = 1;
    shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_IDLE;
  }

  shimmerStatus.configuring = 0;
}

void ShimSens_stopSensing(uint8_t enableDockUartIfDocked)
{
  /* If not sensing, do nothing */
  if (!shimmerStatus.sensing)
  {
    return;
  }

  /* If stop logging has been activated, stop that activity first. */
  if (ShimSens_checkStopLoggingConditions())
  {
    shimmerStatus.sdLogging = 0;
    shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_IDLE;

    ShimTask_clear(TASK_SDWRITE);

    ShimSdDataFile_close();
#if defined(SHIMMER3)
    _delay_cycles(240000); //10ms @ 24MHz
#elif defined(SHIMMER3R)
    HAL_Delay(10); //10ms
#endif

    if (shimmerStatus.sdSyncEnabled)
    {
      ShimSdSync_stop();
    }

    ShimSens_currentExperimentLengthReset();

    if (LogAndStream_isSdInfoSyncDelayed())
    {
      LogAndStream_syncConfigAndCalibOnSd();
    }
  }

  if (ShimSens_checkStopStreamingConditions())
  {
    shimmerStatus.btStreaming = 0;
    shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_IDLE;
    ShimTask_clear(TASK_STREAMDATA);

    if (enableDockUartIfDocked && shimmerStatus.docked && !shimmerStatus.sdLogging)
    {
      DockUart_enable();
    }
  }

  /* Both logging and streaming have been stopped, so we can stop sensing. */
  if (!shimmerStatus.sdLogging && !shimmerStatus.btStreaming)
  {
    shimmerStatus.configuring = 1;
    shimmerStatus.sensing = 0;
    sensing.startTs = 0;
    sensing.isSampling = SAMPLE_NOT_READY;
    ShimSens_stopPeripherals();

    ShimSens_stopSensingWrapup();

    shimmerStatus.configuring = 0;
  }
}

void ShimSens_stopPeripherals(void)
{
#if defined(SHIMMER3)
  SampleTimerStop();
#elif defined(SHIMMER4_SDK)
  S4_RTC_WakeUpSetSlow();
#elif defined(SHIMMER3R)
  RTC_wakeUpOff();
#endif

  if (sensing.nbrMcuAdcChans)
  {
    ADC_stopSensing();
  }
  if (sensing.nbrI2cChans)
  {
    I2C_stopSensing();
  }
  if (sensing.nbrSpiChans)
  {
    SPI_stopSensing();
  }

#if defined(SHIMMER3R)
  Board_enableSensingPower(SENSE_PWR_SENSING, 0);

  if (ShimConfig_isMicrophoneEnabled())
  {
    micStopSensing();
  }
#endif

  if (ShimConfig_getStoredConfig()->expansionBoardPower)
  { //EXT_RESET_N
    Board_setExpansionBrdPower(0);
  }
}

//Can be overwritten by main application
__attribute__((weak)) void ShimSens_stopSensingWrapup(void)
{
  __NOP();
}

void ShimSens_streamData(void)
{
#if SKIP_50MS
  if (sensing.startTs == 0xffffffffffffffff)
  {
  }
  else if ((sensing.startTs == 0) || (sensing.latestTs - sensing.startTs < 1638))
  {
    sensing.startTs = 0xffffffffffffffff;
    sensing.isSampling = SAMPLING_COMPLETE;
    return;
  }
  else
  {
    sensing.startTs = 0xffffffffffffffff;
  }
#endif

  ShimSens_bufPoll();

  //ExpUart_TxIT(sensing.dataBuf, sensing.dataLen);

#if defined(SHIMMER4_SDK)
  //HAL_Delay(500);
  saveData();
#endif
}

void ShimSens_bufPoll()
{
  currentCbFlags = 0;

  if (sensing.nbrMcuAdcChans)
  {
    ADC_gatherDataStart();
  }
  if (sensing.nbrI2cChans)
  {
    I2C_pollSensors();
  }
  if (sensing.nbrSpiChans)
  {
    SPI_pollSensors();
  }

#if defined(SHIMMER3R)
  //TODO come back to when support is added
  //if (isMicrophoneEnabled())
  //{
  //  micStartSensing();
  //}
#endif
}

void ShimSens_saveTimestampToPacket(void)
{
  if (shimmerStatus.sensing)
  {
    /* TODO Shimmer3 only saved 64-bit for first sample and 32-bit after
     * that. Need to considering efficiency impact here versus affect on
     * other things that rely on sensing.latestTs */
    //if (firstTsFlag == 1)
    //{
    //    firstTs = RTC_get64();
    //    firstTsFlag = 2;
    //    *(uint32_t*) currentSampleTsTicks = (uint64_t) firstTs;
    //}
    //else
    //{
    //    *(uint32_t*) currentSampleTsTicks = RTC_get32();
    //}

    sensing.latestTs = RTC_get64();
    sensing.dataBuf[sensing.ptr.ts] = sensing.latestTs & 0xff;
    sensing.dataBuf[sensing.ptr.ts + 1] = (sensing.latestTs >> 8) & 0xff;
    sensing.dataBuf[sensing.ptr.ts + 2] = (sensing.latestTs >> 16) & 0xff;
  }
}

//this is to be called in the ISR
void ShimSens_gatherData(void)
{
#if SAVE_DATA_FROM_RTC_INT
  if (shimmerStatus.sensing && (sensing.isSampling != SAMPLING_IN_PROGRESS))
#else  /* SAVE_DATA_FROM_RTC_INT */
  if (shimmerStatus.sensing)
#endif /* SAVE_DATA_FROM_RTC_INT */
  {
    sensing.isSampling = SAMPLING_IN_PROGRESS;
    ShimSens_saveTimestampToPacket();
    ShimSens_streamData();
  }
}

void ShimSens_stepInit(void)
{
#if defined(SHIMMER3R)
  if (ShimBrd_areMcuAdcsUsedForSensing())
  {
    ADC_gatherDataCb(ShimSens_adcCompleteCb);
  }
  I2cSens_gatherDataCb(ShimSens_i2cCompleteCb);
  SPI_gatherDataCb(ShimSens_spiCompleteCb);
#elif defined(SHIMMER4_SDK)
  ADC_gatherDataCb(ShimSens_step2Start);
  //I2C_gatherDataInit(ShimSens_step3Start);
  //I2C2_gatherDataInit(ShimSens_step4Start);
  I2cSens_gatherDataCb(ShimSens_step3Start);
  I2cBatt_gatherDataCb(ShimSens_step4Start);
  SPI_gatherDataCb(ShimSens_step5Start);
  temp_cnt1 = temp_cnt2 = temp_cnt3 = temp_cnt4 = 0;
#endif
}

void ShimSens_adcCompleteCb(void)
{
  ShimSens_stageCompleteCb(STAT_PERI_ADC);
}

void ShimSens_i2cCompleteCb(void)
{
  ShimSens_stageCompleteCb(STAT_PERI_I2C_SENS);
}

void ShimSens_spiCompleteCb(void)
{
  ShimSens_stageCompleteCb(STAT_PERI_SPI_SENS);
}

void ShimSens_stageCompleteCb(uint8_t stage)
{
  currentCbFlags |= stage;
  if (currentCbFlags == expectedCbFlags)
  {
#if SAVE_DATA_FROM_RTC_INT
    sensing.isSampling = SAMPLING_COMPLETE;
#else  /* SAVE_DATA_FROM_RTC_INT */
    ShimTask_set(TASK_SAVEDATA);
#endif /* SAVE_DATA_FROM_RTC_INT */
  }
}

#if defined(SHIMMER4_SDK)
void ShimSens_step1Start(void)
{
  PeriStat_Set(STAT_PERI_ADC | STAT_PERI_I2C_SENS | STAT_PERI_I2C_BATT | STAT_PERI_SPI_SENS);
  ShimSens_streamData();
  ADC_gatherDataStart();
  if (temp_cnt2 == 1000)
  {
    __NOP();
    __NOP();
    __NOP();
  }
}

void ShimSens_step2Start(void)
{
  PeriStat_Clr(STAT_PERI_ADC);
  temp_cnt2++;
  //I2C_gatherDataStart();
  I2cSens_gatherDataStart();
}

void ShimSens_step3Start(void)
{
  PeriStat_Clr(STAT_PERI_I2C_SENS);
  temp_cnt3++;
  I2cBatt_gatherDataStart();
}

void ShimSens_step4Start(void)
{
  PeriStat_Clr(STAT_PERI_I2C_BATT);
  //SPI_gatherDataStart();
  ShimSens_step5Start();
}

void ShimSens_step5Start(void)
{
  PeriStat_Clr(STAT_PERI_SPI_SENS);
  temp_cnt4++;
  //ShimSens_streamData();
  ShimSens_stepDone();
}
#endif
void ShimSens_stepDone(void)
{
#if defined(SHIMMER3R)

#endif
}

#if defined(SHIMMER4_SDK)
//void ShimSens_step1End(void){ShimSens_step2Start();}
//void ShimSens_step2End(void){ShimSens_step3Start();}
//void ShimSens_step3End(void){ShimSens_step4Start();}
//void ShimSens_step4End(void){ShimSens_step5Start();}
//void ShimSens_step5End(void){ShimSens_step6Start();}
//void ShimSens_step6End(void){}
#endif

void ShimSens_saveData(void)
{
#if USE_SD
  if (shimmerStatus.sdLogging && !ShimSens_checkStopLoggingConditions())
  {
    PeriStat_Set(STAT_PERI_SDMMC);
    ShimSdDataFile_writeToBuff(sensing.dataBuf + 1, sensing.dataLen - 1);
    PeriStat_Clr(STAT_PERI_SDMMC);
  }
#endif
#if USE_BT
  if (shimmerStatus.btStreaming && !ShimSens_checkStopStreamingConditions())
  {
    uint8_t crcMode = ShimBt_getCrcMode();
    if (crcMode != CRC_OFF)
    {
      calculateCrcAndInsert(crcMode, sensing.dataBuf, sensing.dataLen);
    }

    ShimBt_writeToTxBufAndSend(sensing.dataBuf, sensing.dataLen + crcMode, SENSOR_DATA);
  }
#endif

  /* Data packet has moved off dataBuf, device is free to start new packet */
  sensing.isSampling = SAMPLING_COMPLETE;
}

uint8_t ShimSens_getNumEnabledChannels(void)
{
  return sensing.nbrMcuAdcChans + sensing.nbrI2cChans + sensing.nbrSpiChans
#if defined(SHIMMER3R)
      + ShimConfig_isMicrophoneEnabled()
#endif
      ;
}

void ShimSens_startLoggingIfUndockStartEnabled(void)
{
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();
#if IS_SUPPORTED_SINGLE_TOUCH
  if (!storedConfigPtr->singleTouchStart && !storedConfigPtr->userButtonEnable)
#else
  if (!storedConfigPtr->userButtonEnable)
#endif
  {
    ShimTask_setStartLoggingIfReady();
  }
}

uint8_t ShimSens_checkAutostopLoggingCondition(void)
{
  if (shimmerStatus.sdLogging && maxExpLenSecs)
  {
    if (currentExpLenSecs < maxExpLenSecs)
    {
      currentExpLenSecs++;
      return 0; //Continue sensing
    }
    else
    {
      ShimSens_currentExperimentLengthReset();
      return 1; //Trigger auto-stop
    }
  }
  return 0; //No action
}

void ShimSens_currentExperimentLengthReset(void)
{
  currentExpLenSecs = 0;
}

void ShimSens_maxExperimentLengthSecsSet(uint16_t maxExpLenMins)
{
  maxExpLenSecs = maxExpLenMins * 60;
}
