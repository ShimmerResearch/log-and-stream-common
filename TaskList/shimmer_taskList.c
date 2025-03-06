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

#include <TaskList/shimmer_taskList.h>

#include <Battery/shimmer_battery.h>
#include <Boards/shimmer_boards.h>
#include <Calibration/shimmer_calibration.h>
#include <Comms/shimmer_bt_uart.h>
#include <Comms/shimmer_dock_usart.h>
#include <Configuration/shimmer_config.h>
#include <SDCard/shimmer_sd.h>
#include <SDCard/shimmer_sd_header.h>
#include <SDSync/shimmer_sd_sync.h>
#include <Sensing/shimmer_sensing.h>
#include <log_and_stream_externs.h>

#include "hal_FactoryTest.h"

#if defined(SHIMMER3R)
#include "shimmer_definitions.h"
#endif

volatile uint32_t taskList = 0;
uint32_t taskCurrent;

extern void SetupDock(void);

void ShimTask_NORM_init(void)
{
  taskList = 0;
}

void ShimTask_NORM_manage(void)
{
  taskCurrent = ShimTask_getCurrent();

#if USE_USBX
  USBX_Device_Process();
#endif

  if (!taskCurrent)
  {
    sleepWhenNoTask();
    //#if defined(SHIMMER3R)
    //      /* Only wake MCU when new Task is set. See corresponding
    //       * HAL_PWR_DisableSleepOnExit() in ShimTask_set() */
    //      HAL_PWR_EnableSleepOnExit();
    //
    //      Power_SleepUntilInterrupt();
    //
    //      //if(shimmerStatus.isBtConnected && !shimmerStatus.isSensing){
    //      //   Power_SleepUntilInterrupt();
    //      //
    //      //   __NOP();
    //      //   __NOP();
    //      //   __NOP();
    //      //}else{
    //      //   if(shimmerStatus.periStat == 0)
    //      //   {
    //      ////            static uint8_t green1_cnt = 0;
    //      ////            if(!green1_cnt++){
    //      ////               Board_ledToggle(LED_GREEN1);
    //      ////            }
    //      //Power_StopUntilInterrupt();
    //      //}
    //      //else
    //      //{
    //      //static uint8_t blue_cnt = 0;
    //      //if(!blue_cnt++){
    //      //   Board_ledToggle(LED_BLUE);
    //      //}
    //      //__NOP();
    //      //__NOP();
    //      //__NOP();
    //      //Power_SleepUntilInterrupt();
    //      //}
    //      //}
    //#endif
  }
  else
  {
    switch (taskCurrent)
    {
    case TASK_SETUP_DOCK:
#if defined(SHIMMER3)
      checkSetupDock();
#else
      SetupDock();
#endif
      break;
    case TASK_DOCK_PROCESS_CMD:
      DockUart_processCmd();
      break;
    case TASK_DOCK_RESPOND:
      DockUart_sendRsp();
      break;
    case TASK_BT_PROCESS_CMD:
      BtUart_processCmd();
      break;
    case TASK_BT_RESPOND:
      BtUart_sendRsp();
      break;
    case TASK_RCCENTERR1:
      /* SD Sync - Center */
      SyncCenterR1();
      break;
    case TASK_RCNODER10:
      /* SD Sync - Node */
      SyncNodeR10();
      break;
    case TASK_STREAMDATA:
      //TODO reduce down to one shared function
#if defined(SHIMMER3)
      checkStreamData();
#else
      S4Sens_streamData();
#endif
      break;
#if defined(SHIMMER3)
//    case TASK_CFGCH:
//      ConfigureChannels();
//      break;
    case TASK_SAMPLE_MPU9150_MAG:
      MPU9150_startMagMeasurement();
      break;
    case TASK_SAMPLE_BMPX80_PRESS:
      BMPX80_startMeasurement();
      break;
#endif
#if defined(SHIMMER3R)
    case TASK_SAVEDATA:
      saveData();
      break;
#endif
    case TASK_STARTSENSING:
      //TODO reduce down to one shared function
#if defined(SHIMMER3)
      checkStartSensing();
#elif defined(SHIMMER3R)
      S4Sens_startSensing();
#endif
      break;
#if defined(SHIMMER3R)
    case TASK_STOPSENSING:
      S4Sens_stopSensing();
      break;
#endif
    case TASK_SDWRITE:
      //TODO reduce down to one shared function
#if defined(SHIMMER3)
      Write2SD();
#elif defined(SHIMMER3R)
      SD_writeToCard();
#endif
      break;
    case TASK_SDLOG_CFG_UPDATE:
      if (!shimmerStatus.docked && !shimmerStatus.sensing && CheckSdInslot() && GetSdCfgFlag())
      {
        shimmerStatus.configuring = 1;
        ShimConfig_readRam();
        UpdateSdConfig();
        SetSdCfgFlag(0);
        shimmerStatus.configuring = 0;
      }
      break;
    case TASK_BATT_READ:
#if defined(SHIMMER3)
      /* use adc channel2 and mem4, read back battery status every certain period */
      if (!shimmerStatus.sensing)
      {
        manageReadBatt(1);
      }
#elif defined(SHIMMER3R)
      manageReadBatt(0);
      setupNextRtcMinuteAlarm();
#elif defined(SHIMMER4_SDK)
      S4_ADC_readBatt();
      I2C_readBatt();
#endif
      break;
    case TASK_FACTORY_TEST:
      run_factory_test();
      break;
#if defined(SHIMMER3R) || defined(SHIMMER4_SDK)
    case TASK_USB_SETUP:
      vbusPinStateCheck();
      break;
#endif
    default:
      break;
    }
  }
}

uint32_t ShimTask_NORM_getCurrent()
{
  uint8_t i;
  uint32_t task;
  if (taskList)
  {
    for (i = 0; i < TASK_SIZE; i++)
    {
      task = 0x00000001UL << i;
      if (taskList & task)
      {
        ShimTask_clear(task);
        return task;
      }
    }
  }
  return 0;
}

void ShimTask_NORM_clear(uint32_t task_id)
{
  taskList &= ~task_id;
}

uint8_t ShimTask_NORM_set(uint32_t task_id)
{
  uint8_t is_sleeping = 0;
  if (!taskList && !taskCurrent)
  {
    is_sleeping = 1;
  }
  taskList |= task_id;
#if defined(SHIMMER3R)
  HAL_PWR_DisableSleepOnExit();
#endif
  return is_sleeping;
}

uint32_t ShimTask_NORM_getList()
{
  return taskList;
}

uint8_t ShimTask_setNewBtCmdToProcess(void)
{
  return ShimTask_set(TASK_BT_PROCESS_CMD);
}

void ShimTask_setStartSensing(void)
{
  ShimTask_set(TASK_SDLOG_CFG_UPDATE);
  ShimTask_set(TASK_STARTSENSING);
}

void ShimTask_setStopSensing(void)
{
#if defined(SHIMMER3)
  if (shimmerStatus.sensing)
  {
    setStopSensingFlag(1U);
  }
#else
  ShimTask_set(TASK_STOPSENSING);
#endif
}

void ShimTask_setStopLogging(void)
{
#if defined(SHIMMER3)
  if (shimmerStatus.sdLogging)
  {
    setStopLoggingFlag(1U);
  }
#else
  ShimTask_set(TASK_STOPSENSING);
#endif
}

void ShimTask_setStopStreaming(void)
{
#if defined(SHIMMER3)
  if (shimmerStatus.btStreaming)
  {
    setStopStreamingFlag(1U);
  }
#else
  ShimTask_set(TASK_STOPSENSING);
#endif
}
