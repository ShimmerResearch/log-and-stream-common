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

#include "shimmer_taskList.h"

#include <log_and_stream_includes.h>
#include <log_and_stream_externs.h>

#include "hal_FactoryTest.h"

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
        ShimDock_processCmd();
        break;
      case TASK_DOCK_RESPOND:
        ShimDock_sendRsp();
        break;
      case TASK_BT_PROCESS_CMD:
        ShimBt_processCmd();
        break;
      case TASK_BT_RESPOND:
        ShimBt_sendRsp();
        break;
      case TASK_RCCENTERR1:
        /* SD Sync - Center */
        ShimSdSync_centerR1();
        break;
      case TASK_RCNODER10:
        /* SD Sync - Node */
        ShimSdSync_nodeR10();
        break;
      case TASK_STREAMDATA:
        ShimSens_streamData();
        break;
#if defined(SHIMMER3)
      case TASK_SAMPLE_MPU9150_MAG:
        MPU9150_startMagMeasurement();
        break;
      case TASK_SAMPLE_BMPX80_PRESS:
        BMPX80_startMeasurement();
        break;
#endif
      case TASK_SAVEDATA:
        ShimSens_saveData();
        break;
      case TASK_STARTSENSING:
        ShimSens_startSensing();
        break;
      case TASK_STOPSENSING:
        ShimSens_stopSensing();
        break;
      case TASK_SDWRITE:
        ShimSdDataFile_writeToCard();
        break;
      case TASK_SDLOG_CFG_UPDATE:
        if (!shimmerStatus.docked && !shimmerStatus.sensing && CheckSdInslot()
            && ShimConfig_getSdCfgFlag())
        {
          shimmerStatus.configuring = 1;
          ShimConfig_readRam();
          ShimSdCfgFile_generate();
          ShimConfig_setSdCfgFlag(0);
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
      case TASK_BT_TX_BUF_CLEAR:
        ShimBt_clearBtTxBuf(1U);
        break;
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

/* TODO this won't work in it's current form as since neither
 * shimmerStatus.sdlogCmd nor shimmerStatus.btstreamCmd are modified and written
 * back, no action will actually be taken. This shouldn't be needed as settings
 * should never be allowed to be modified over Bluetooth while sensing is
 * on-going but still could be needed if some is calling BT commands directly
 * outside of our provided software. */
void ShimTask_setRestartSensing(void)
{
  if (shimmerStatus.sensing)
  {
    ShimTask_setStopSensing();
    ShimTask_setStartSensing();
  }
}

void ShimTask_setStartSensing(void)
{
  ShimTask_set(TASK_SDLOG_CFG_UPDATE);
  ShimTask_set(TASK_STARTSENSING);
}

void ShimTask_setStopSensing(void)
{
  ShimTask_set(TASK_STOPSENSING);
}

void ShimTask_setStopLogging(void)
{
  ShimTask_set(TASK_STOPSENSING);
}

void ShimTask_setStopStreaming(void)
{
  ShimTask_set(TASK_STOPSENSING);
}
