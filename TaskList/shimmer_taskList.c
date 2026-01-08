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

#include "log_and_stream_externs.h"
#include "log_and_stream_includes.h"

#include "hal_FactoryTest.h"

static volatile uint32_t taskList = 0;
static volatile TaskId_t executingTask = TASK_NONE;

#if TEST_TASK_MONITOR
/* new state for task watchdog */
static volatile uint32_t execStartTick = 0;
#endif //TEST_TASK_MONITOR

void ShimTask_NORM_init(void)
{
  taskList = 0;
  executingTask = TASK_NONE;
#if TEST_TASK_MONITOR
  execStartTick = 0;
#endif
}

void ShimTask_NORM_manage(void)
{
  executingTask = ShimTask_popNext();

#if USE_USBX
  USBX_Device_Process();
#endif

  if (executingTask == TASK_NONE)
  {
    sleepWhenNoTask();
  }
  else
  {
#if TEST_TASK_MONITOR
    /* mark start of task execution for watchdog */
    ShimTask_executionStart();
#endif //TEST_TASK_MONITOR

    switch (executingTask)
    {
      case TASK_SETUP_DOCK:
        LogAndStream_checkSetupDockUnDock();
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
      case TASK_GATHER_DATA:
        ShimSens_gatherData();
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
        ShimBt_instreamStatusRespSendIfNotBtCmd();
        break;
      case TASK_STOPSENSING:
        ShimSens_stopSensing(1);
        ShimBt_instreamStatusRespSendIfNotBtCmd();
        break;
      case TASK_SDWRITE:
        ShimSdDataFile_writeToCard();
        break;
      case TASK_SDLOG_CFG_UPDATE:
        if (!shimmerStatus.docked && !shimmerStatus.sensing
            && LogAndStream_checkSdInSlot() && ShimConfig_getFlagWriteCfgToSd())
        {
          shimmerStatus.configuring = 1;
          ShimConfig_readRam();
          ShimSdCfgFile_generate();
          ShimConfig_setFlagWriteCfgToSd(0, 1);
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
        /* Don't overwrite the alarm set for jumping to BSL */
        if (!shimmerStatus.bslRebootPending)
        {
          RTC_setAlarmBattRead();
        }
#elif defined(SHIMMER4_SDK)
        S4_ADC_readBatt();
        I2C_readBatt();
#endif
        break;
      case TASK_FACTORY_TEST:
        ShimFactoryTest_run();
        break;
#if defined(SHIMMER3R) || defined(SHIMMER4_SDK)
      case TASK_USB_SETUP:
        vbusPinStateCheck();
        LogAndStream_setupDockUndock();
        break;
#endif
      case TASK_BT_TX_BUF_CLEAR:
        ShimBt_clearBtTxBuf(1U);
        break;

      case TASK_BT_TURN_ON_AFTER_BOOT:
        InitialiseBtAfterBoot();
        break;

#if defined(SHIMMER3R)
      case TASK_JUMP_TO_BOOT_LOADER:
        JumpToBootloader();
        break;
#endif

#if defined(SHIMMER3)
      case TASK_WRITE_RADIO_DETAILS:
        if (ShimEeprom_isPresent())
        {
          ShimEeprom_writeRadioDetails();
        }
        break;
#endif

      default:
        break;
    }

#if TEST_TASK_MONITOR
    /* mark end of task execution */
    ShimTask_executionEnd();
#endif //TEST_TASK_MONITOR

    executingTask = TASK_NONE;
  }
}

TaskId_t ShimTask_NORM_popNext(void)
{
  uint8_t i;
  TaskId_t task;
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

void ShimTask_NORM_clear(TaskId_t task_id)
{
  taskList &= ~task_id;
}

uint8_t ShimTask_NORM_set(TaskId_t task_id)
{
  uint8_t is_sleeping = 0;
  if (!taskList && !executingTask)
  {
    is_sleeping = 1;
  }
  taskList |= task_id;
#if defined(SHIMMER3R)
  /* if called from an ISR, force main context to run:
     - Pend PendSV to wake the scheduler / main loop
     - optionally disable Sleep-on-Exit so the main executes after IRQ return */
  if (__get_IPSR() != 0)
  {
    /* If your code uses HAL_PWR_EnableSleepOnExit() elsewhere, disable it so main runs once */
    HAL_PWR_DisableSleepOnExit();

    /* Pend PendSV to ensure main context runs to handle the task */
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
  }
#endif
  return is_sleeping;
}

uint32_t ShimTask_NORM_getList()
{
  return taskList;
}

#if TEST_TASK_MONITOR
void ShimTask_executionStart(void)
{
  execStartTick = platform_getTick();
}

void ShimTask_executionEnd(void)
{
  execStartTick = 0;
}

uint32_t ShimTask_getExecStartTick(void)
{
  return execStartTick;
}
#endif //TEST_TASK_MONITOR

uint32_t ShimTask_getExecutingTask(void)
{
  return executingTask;
}

void ShimTask_setStartLoggingIfReady(void)
{
  if (ShimSens_checkStartLoggingConditions())
  {
    shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_START;
    ShimTask_set(TASK_STARTSENSING);
  }
}

void ShimTask_setStartStreamingIfReady(void)
{
  if (ShimSens_checkStartStreamingConditions())
  {
    shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_START;
    ShimTask_set(TASK_STARTSENSING);
  }
}

void ShimTask_setStartStreamingAndLoggingIfReady(void)
{
  if (ShimSens_checkStartLoggingConditions())
  {
    shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_START;
  }
  if (ShimSens_checkStartStreamingConditions())
  {
    shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_START;
  }

  if (shimmerStatus.sdlogCmd == SD_LOG_CMD_STATE_START
      || shimmerStatus.btstreamCmd == BT_STREAM_CMD_STATE_START)
  {
    ShimTask_set(TASK_STARTSENSING);
  }
}

void ShimTask_setStopSensing(void)
{
  if (shimmerStatus.sdLogging)
  {
    ShimTask_setStopLogging();
  }
  if (shimmerStatus.btStreaming)
  {
    ShimTask_setStopStreaming();
  }
}

void ShimTask_setStopLogging(void)
{
  shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_STOP;
  ShimTask_set(TASK_STOPSENSING);
}

void ShimTask_setStopStreaming(void)
{
  shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_STOP;
  ShimTask_set(TASK_STOPSENSING);
}

void ShimTask_setInitialiseBluetooth(void)
{
  ShimTask_set(TASK_BT_TURN_ON_AFTER_BOOT);
}
