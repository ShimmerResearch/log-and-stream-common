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

volatile uint32_t taskList = 0;
uint32_t taskCurrent;

extern void SetupDock(uint8_t src);

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

#if defined(SHIMMER3)
  if (!taskCurrent)
#elif defined(SHIMMER3R)
  if (!taskCurrent || shimmerStatus.pendingRebootForDfu)
#endif
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
        SetupDock(SD_BT_LOG_STREAM_CMD_SRC_HW);
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
        uint8_t sendStatus = 0;
        ShimSens_startSensing();
        if ((shimmerStatus.btstreamCmd ==  BT_STREAM_CMD_STATE_START_HW) ||
            (shimmerStatus.sdlogCmd == SD_LOG_CMD_STATE_START_HW))
        {
          sendStatus = 1;
        }
        if(sendStatus)
        {
          ShimBt_instreamStatusRespSend();
        }
        break;
      case TASK_STOPSENSING:
        sendStatus = 0;
        ShimSens_stopSensing(1);
        if ((shimmerStatus.btstreamCmd ==  BT_STREAM_CMD_STATE_STOP_HW) ||
            (shimmerStatus.sdlogCmd == SD_LOG_CMD_STATE_STOP_HW))
        {
          sendStatus = 1;
        }
        if (sendStatus)
        {
          ShimBt_instreamStatusRespSend();
        }
        break;
      case TASK_SDWRITE:
        ShimSdDataFile_writeToCard();
        break;
      case TASK_SDLOG_CFG_UPDATE:
        if (!shimmerStatus.docked && !shimmerStatus.sensing && CheckSdInslot()
            && ShimConfig_getFlagWriteCfgToSd())
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
        RTC_setAlarmBattRead();
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
        SetupDock(SD_BT_LOG_STREAM_CMD_SRC_HW);
        break;
#endif
      case TASK_BT_TX_BUF_CLEAR:
        ShimBt_clearBtTxBuf(1U);
        break;

      case TASK_BT_TURN_ON_AFTER_BOOT:
        InitialiseBtAfterBoot();
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

void ShimTask_setStartLoggingIfReady(uint8_t src)
{

  if (ShimSens_checkStartLoggingConditions())
  {
    if (src == SD_BT_LOG_STREAM_CMD_SRC_HW)
      {
        shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_HW;
        shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_START_HW;
      }
      else if (src == SD_BT_LOG_STREAM_CMD_SRC_BT)
      {
        shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_BT;
        shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_START_BT;
      }
      else
      {
        shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_OTH;
        shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_START_OTH;
      }
    ShimTask_set(TASK_STARTSENSING);
  }
  else
  {
    shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREM_CMD_SRC_NONE;
  }
}

void ShimTask_setStartStreamingIfReady(uint8_t src)
{
  if (ShimSens_checkStartStreamingConditions())
  {
    if (src == SD_BT_LOG_STREAM_CMD_SRC_HW)
    {
      shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_HW;
      shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_START_HW;
    }
    else if (src == SD_BT_LOG_STREAM_CMD_SRC_BT)
    {
      shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_BT;
      shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_START_BT;
    }
    else
    {
      shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_OTH;
      shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_START_OTH;
    }

    ShimTask_set(TASK_STARTSENSING);
  }
  else
  {
    shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREM_CMD_SRC_NONE;
  }
}


void ShimTask_setStartStreamingAndLoggingIfReady(uint8_t src)
{
  if (ShimSens_checkStartLoggingConditions())
  {
    if (src == SD_BT_LOG_STREAM_CMD_SRC_HW)
    {
      shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_HW;
      shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_START_HW;
    }
    else if (src == SD_BT_LOG_STREAM_CMD_SRC_BT)
    {
      shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_BT;
      shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_START_BT;
    }
    else
    {
      shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_OTH;
      shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_START_OTH;
    }
  }
  if (ShimSens_checkStartStreamingConditions())
  {
    if (src == SD_BT_LOG_STREAM_CMD_SRC_HW)
    {
      shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_HW;
      shimmerStatus.sdlogCmd = BT_STREAM_CMD_STATE_START_HW;
    }
    else if (src == SD_BT_LOG_STREAM_CMD_SRC_BT)
    {
      shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_BT;
      shimmerStatus.sdlogCmd = BT_STREAM_CMD_STATE_START_BT;
    }
    else
    {
      shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_OTH;
      shimmerStatus.sdlogCmd = BT_STREAM_CMD_STATE_START_OTH;
    }
  }

  if ((shimmerStatus.sdlogCmd == (SD_LOG_CMD_STATE_START_HW || SD_LOG_CMD_STATE_START_BT || SD_LOG_CMD_STATE_START_OTH))
      || (shimmerStatus.btstreamCmd == (BT_STREAM_CMD_STATE_START_BT || BT_STREAM_CMD_STATE_START_HW ||BT_STREAM_CMD_STATE_START_OTH)))
  {
    ShimTask_set(TASK_STARTSENSING);
  }
  else
  {
    shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREM_CMD_SRC_NONE;
  }
}

void ShimTask_setStopSensing(uint8_t src)
{
  if (shimmerStatus.sdLogging)
  {
    ShimTask_setStopLogging(src);
  }
  if (shimmerStatus.btStreaming)
  {
    ShimTask_setStopStreaming(src);
  }
}

void ShimTask_setStopLogging(uint8_t src)
{
  if (src == SD_BT_LOG_STREAM_CMD_SRC_HW)
  {
    shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_HW;
    shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_STOP_HW;
  }
  else if (src == SD_BT_LOG_STREAM_CMD_SRC_BT)
  {
    shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_BT;
    shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_STOP_BT;
  }
  else
  {
    shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_OTH;
    shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_STOP_OTH;
  }
  ShimTask_set(TASK_STOPSENSING);
}

void ShimTask_setStopStreaming(uint8_t src)
{
  if (src == SD_BT_LOG_STREAM_CMD_SRC_HW)
  {
    shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_HW;
    shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_STOP_HW;
  }
  else if (src == SD_BT_LOG_STREAM_CMD_SRC_BT)
  {
    shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_BT;
    shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_STOP_BT;
  }
  else
  {
    shimmerStatus.sdBtCmdSrc = SD_BT_LOG_STREAM_CMD_SRC_OTH;
    shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_STOP_OTH;
  }
  ShimTask_set(TASK_STOPSENSING);
}

void ShimTask_setInitialiseBluetooth(void)
{
  ShimTask_set(TASK_BT_TURN_ON_AFTER_BOOT);
}
