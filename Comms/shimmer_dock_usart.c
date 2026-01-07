/*
 * shimmer_dock_comms.c
 *
 *  Created on: 13 Aug 2024
 *  Author: MarkNolan
 */

#include "shimmer_dock_usart.h"

#include <stdint.h>
#include <string.h>

#include "log_and_stream_externs.h"
#include "log_and_stream_includes.h"

#include "shimmer_definitions.h"
#include "version.h"

#if defined(SHIMMER3)
#include "../5xx_HAL/hal_CRC.h"
#include "../5xx_HAL/hal_FactoryTest.h"
#include "../5xx_HAL/hal_InfoMem.h"
#include "../5xx_HAL/hal_RTC.h"
#include "../5xx_HAL/hal_UCA0.h"
#include "../5xx_HAL/hal_UartA0.h"
#include "../CAT24C16/CAT24C16.h"
#else
#include "stm32u5xx_hal_uart.h"
#endif
#if defined(SHIMMER3R)
#include "usbd_cdc_acm_if.h"
#endif
#define EN_CALIB_DUMP_RSP 0

uint8_t uartSteps, uartArgSize, uartArg2Wait, uartCrc2Wait, uartAction;
uint8_t dockRxBuf[UART_DATA_LEN_MAX];
uint8_t uartSendRspMac, uartSendRspVer, uartSendRspBat,
    uartSendRspRtcConfigTime, uartSendRspCurrentTime, uartSendRspGdi,
    uartSendRspGdm, uartSendRspGim, uartSendRspBtVer, uartSendRspAck,
    uartSendRspBadCmd, uartSendRspBadArg, uartSendRspBadCrc;
#if EN_CALIB_DUMP_RSP
uint8_t uartSendRspCalibDump;
#endif
uint8_t uartRespBuf[UART_RSP_PACKET_SIZE];

uint8_t uartDcMemLength, uartInfoMemLength;
uint16_t uartDcMemOffset, uartInfoMemOffset;
uint64_t uartTimeStart;

void ShimDock_resetVariables(void)
{
  uartSteps = 0;
  uartArgSize = 0;
  uartArg2Wait = 0;
  uartCrc2Wait = 0;

  uartSendRspAck = 0;
  uartSendRspBadCmd = 0;
  uartSendRspBadArg = 0;
  uartSendRspBadCrc = 0;

  uartSendRspMac = 0;
  uartSendRspVer = 0;
  uartSendRspBat = 0;
  uartSendRspRtcConfigTime = 0;
  uartSendRspCurrentTime = 0;
  uartSendRspGdi = 0;
  uartSendRspGdm = 0;
  uartSendRspGim = 0;
#if EN_CALIB_DUMP_RSP
  uartSendRspCalibDump = 0;
#endif
  uartSendRspBtVer = 0;

  uartTimeStart = 0;
}

uint8_t ShimDock_rxCallback(uint8_t data)
{
  if (shimmerStatus.booting)
  {
    return 0;
  }

  //TODO revisit and either make common approach for Shimmer3 and Shimmer3R or ensure HAL_GetTick() works fine for Shimmer3R
#if defined(SHIMMER3)
  uint64_t uart_time = RTC_get64();
#else
  uint64_t uart_time = HAL_GetTick();
#endif
  if (uartTimeStart)
  {
    //Check for 100ms timeout between bytes
#if defined(SHIMMER3)
    if (uart_time - uartTimeStart > TIMEOUT_100_MS)
#else
    if (uart_time - uartTimeStart > 100)
#endif
    {
      uartSteps = 0;
    }
  }
  uartTimeStart = uart_time;

  if (uartSteps)
  { //wait for: cmd, len, data, crc -> process
    if (uartSteps == UART_STEP_WAIT4_CMD)
    {
      uartAction = data;
      uartArgSize = UART_RXBUF_CMD;
      dockRxBuf[uartArgSize++] = data;
      switch (uartAction)
      {
        case UART_SET:
        case UART_GET:
          uartSteps = UART_STEP_WAIT4_LEN;
          return 0;
        default:
          uartSteps = 0;
          uartSendRspBadCmd = 1;
          ShimTask_set(TASK_DOCK_RESPOND);
          return 1;
      }
    }
    else if (uartSteps == UART_STEP_WAIT4_LEN)
    {
      uartSteps = UART_STEP_WAIT4_DATA;
      uartArgSize = UART_RXBUF_LEN;
      dockRxBuf[uartArgSize++] = data;
      uartArg2Wait = data;
      return 0;
    }
    else if (uartSteps == UART_STEP_WAIT4_DATA)
    {
      dockRxBuf[uartArgSize++] = data;
      if (!--uartArg2Wait)
      {
        uartCrc2Wait = 2;
        uartSteps = UART_STEP_WAIT4_CRC;
      }
      return 0;
    }
    else if (uartSteps == UART_STEP_WAIT4_CRC)
    {
      dockRxBuf[uartArgSize++] = data;
      if (!--uartCrc2Wait)
      {
        uartSteps = 0;
        uartArgSize = 0;
        ShimTask_set(TASK_DOCK_PROCESS_CMD);
        uartTimeStart = 0;
        return 1;
      }
      else
      {
        return 0;
      }
    }
    else
    {
      uartSteps = 0;
      uartTimeStart = 0;
      return 0;
    }
  }
  else
  {
    if (data == '$')
    {
      uartAction = 0;
      uartArgSize = UART_RXBUF_START;
      dockRxBuf[UART_RXBUF_START] = '$';
      uartSteps = UART_STEP_WAIT4_CMD;
      return 0;
    }
  }
  return 0;
}

void ShimDock_processCmd(void)
{
  if (uartAction)
  {
    if (ShimDock_uartCheckCrc(dockRxBuf[UART_RXBUF_LEN] + 3))
    {
      if (uartAction == UART_GET)
      { //get
        if (dockRxBuf[UART_RXBUF_COMP] == UART_COMP_SHIMMER)
        { //get shimmer
          switch (dockRxBuf[UART_RXBUF_PROP])
          {
            case UART_PROP_MAC:
              if (dockRxBuf[UART_RXBUF_LEN] == 2)
              {
                uartSendRspMac = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
            case UART_PROP_VER:
              if (dockRxBuf[UART_RXBUF_LEN] == 2)
              {
                uartSendRspVer = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
            case UART_PROP_RWC_CFG_TIME:
              if (dockRxBuf[UART_RXBUF_LEN] == 2)
              {
                uartSendRspRtcConfigTime = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
            case UART_PROP_CURR_LOCAL_TIME:
              if (dockRxBuf[UART_RXBUF_LEN] == 2)
              {
                uartSendRspCurrentTime = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
            case UART_PROP_INFOMEM:
              uartInfoMemLength = dockRxBuf[UART_RXBUF_DATA];
              uartInfoMemOffset = (uint16_t) dockRxBuf[UART_RXBUF_DATA + 1]
                  + (((uint16_t) dockRxBuf[UART_RXBUF_DATA + 2]) << 8);
              if ((uartInfoMemLength <= 0x80) && (uartInfoMemOffset <= 0x01ff)
                  && (uartInfoMemLength + uartInfoMemOffset <= 0x0200))
              {
                uartSendRspGim = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
#if EN_CALIB_DUMP_RSP
            case UART_PROP_CALIB_DUMP:
              uartCalibRamLength = dockRxBuf[UART_RXBUF_DATA];
              uartCalibRamOffset = (uint16_t) dockRxBuf[UART_RXBUF_DATA + 1]
                  + (((uint16_t) dockRxBuf[UART_RXBUF_DATA + 2]) << 8);
              if ((uartCalibRamLength <= 128)
                  && (uartCalibRamOffset <= SHIMMER_CALIB_RAM_MAX - 1)
                  && (uartCalibRamLength + uartCalibRamOffset <= SHIMMER_CALIB_RAM_MAX))
              {
                uartSendRspCalibDump = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
#endif
            default:
              uartSendRspBadCmd = 1;
              break;
          }
        }
        else if (dockRxBuf[UART_RXBUF_COMP] == UART_COMP_BAT)
        { //get battery
          switch (dockRxBuf[UART_RXBUF_PROP])
          {
            case UART_PROP_VALUE:
              if (dockRxBuf[UART_RXBUF_LEN] == 2)
              {
                uartSendRspBat = 1; //already in the callback function
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
            default:
              uartSendRspBadCmd = 1;
              break;
          }
        }
        else if (dockRxBuf[UART_RXBUF_COMP] == UART_COMP_DAUGHTER_CARD)
        { //get daughter card
          switch (dockRxBuf[UART_RXBUF_PROP])
          {
            case UART_PROP_CARD_ID:
              uartDcMemLength = dockRxBuf[UART_RXBUF_DATA];
              uartDcMemOffset = (uint16_t) dockRxBuf[UART_RXBUF_DATA + 1];
              if ((uartDcMemLength <= 16) && (uartDcMemOffset <= 15)
                  && ((uint16_t) uartDcMemLength + uartDcMemOffset <= 16))
              {
                uartSendRspGdi = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
            case UART_PROP_CARD_MEM:
              uartDcMemLength = dockRxBuf[UART_RXBUF_DATA];
              uartDcMemOffset = (uint16_t) dockRxBuf[UART_RXBUF_DATA + 1]
                  + (((uint16_t) dockRxBuf[UART_RXBUF_DATA + 2]) << 8);
              if ((uartDcMemLength <= 128) && (uartDcMemOffset <= 2031)
                  && ((uint16_t) uartDcMemLength + uartDcMemOffset <= 2032))
              {
                uartSendRspGdm = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
            default:
              uartSendRspBadCmd = 1;
              break;
          }
        }
        else if (dockRxBuf[UART_RXBUF_COMP] == UART_COMP_BT)
        {
          switch (dockRxBuf[UART_RXBUF_PROP])
          {
            case UART_PROP_VER:
              if (dockRxBuf[UART_RXBUF_LEN] == 2)
              {
                uartSendRspBtVer = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
            default:
              uartSendRspBadCmd = 1;
              break;
          }
        }
        else
        {
          uartSendRspBadCmd = 1;
        }
      }
      else if (uartAction == UART_SET)
      { //set
        if (dockRxBuf[UART_RXBUF_COMP] == UART_COMP_SHIMMER)
        { //set shimmer
          switch (dockRxBuf[UART_RXBUF_PROP])
          {
            case UART_PROP_RWC_CFG_TIME:
              if (dockRxBuf[UART_RXBUF_LEN] == 10)
              {
                RTC_setTimeFromTicksPtr(dockRxBuf + UART_RXBUF_DATA);
                ShimRtc_rwcErrorCheck();

                ShimConfig_getStoredConfig()->rtcSetByBt = 0;
                InfoMem_write(NV_SD_TRIAL_CONFIG0,
                    &ShimConfig_getStoredConfig()->rawBytes[NV_SD_TRIAL_CONFIG0], 1);
                ShimSdHead_sdHeadTextSetByte(SDH_TRIAL_CONFIG0,
                    ShimConfig_getStoredConfig()->rawBytes[NV_SD_TRIAL_CONFIG0]);

                uartSendRspAck = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
            case UART_PROP_INFOMEM:
              uartInfoMemLength = dockRxBuf[UART_RXBUF_DATA];
              uartInfoMemOffset = (uint16_t) dockRxBuf[UART_RXBUF_DATA + 1]
                  + (((uint16_t) dockRxBuf[UART_RXBUF_DATA + 2]) << 8);
              if ((uartInfoMemLength <= 128)
                  && (uartInfoMemOffset <= (STOREDCONFIG_SIZE - 1))
                  && (uartInfoMemLength + uartInfoMemOffset <= STOREDCONFIG_SIZE))
              {
                ShimConfig_storedConfigSet(dockRxBuf + UART_RXBUF_DATA + 3,
                    uartInfoMemOffset, uartInfoMemLength);

                ShimConfig_checkAndCorrectConfig();
                ShimConfig_setFlagWriteCfgToSd(1, 0);

                LogAndStream_infomemUpdate();

                uartSendRspAck = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
#if EN_CALIB_DUMP_RSP
            case UART_PROP_CALIB_DUMP:
              uartCalibRamLength = dockRxBuf[UART_RXBUF_DATA];
              uartCalibRamOffset = (uint16_t) dockRxBuf[UART_RXBUF_DATA + 1]
                  + (((uint16_t) dockRxBuf[UART_RXBUF_DATA + 2]) << 8);
              if ((uartCalibRamLength <= 128)
                  && (uartCalibRamOffset <= SHIMMER_CALIB_RAM_MAX - 1)
                  && (uartCalibRamLength + uartCalibRamOffset <= SHIMMER_CALIB_RAM_MAX))
              {
                if (ShimmerCalib_ramWrite(dockRxBuf + UART_RXBUF_DATA + 3,
                        uartCalibRamLength, uartCalibRamOffset)
                    == 1)
                {
                }
                uartSendRspAck = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
#endif
#if defined(SHIMMER3R)
            case UART_PROP_ENTER_BOOTLOADER:
              RTC_setAlarmRebootToBootloader(dockRxBuf[UART_RXBUF_DATA]);

              uartSendRspAck = 1;
              break;
#endif
            default:
              uartSendRspBadCmd = 1;
              break;
          }
        }
        else if (dockRxBuf[UART_RXBUF_COMP] == UART_COMP_DAUGHTER_CARD)
        { //set daughter card id
          switch (dockRxBuf[UART_RXBUF_PROP])
          {
            case UART_PROP_CARD_ID:
              uartDcMemLength = dockRxBuf[UART_RXBUF_DATA];
              uartDcMemOffset = dockRxBuf[UART_RXBUF_DATA + 1];
              if ((uartDcMemLength <= 16) && (uartDcMemOffset < 16))
              {
                //Write (up to) 16 bytes to eeprom
                eepromWrite(uartDcMemOffset, (uint16_t) uartDcMemLength,
                    dockRxBuf + UART_RXBUF_DATA + 2U);
                /* Copy new bytes to active daughter card byte array so it can be read back immediately and verified */
                ShimBrd_setDaugherCardIdMemory((uint8_t) uartDcMemOffset,
                    dockRxBuf + UART_RXBUF_DATA + 2, uartDcMemLength);
                uartSendRspAck = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
            case UART_PROP_CARD_MEM:
              uartDcMemLength = dockRxBuf[UART_RXBUF_DATA];
              uartDcMemOffset = (uint16_t) dockRxBuf[UART_RXBUF_DATA + 1]
                  + (((uint16_t) dockRxBuf[UART_RXBUF_DATA + 2]) << 8);

              if (ShimEeprom_writeDaughterCardMem(uartDcMemOffset,
                      uartDcMemLength, dockRxBuf + UART_RXBUF_DATA + 3U))
              {
                uartSendRspAck = 1;
              }
              else
              {
                uartSendRspBadArg = 1;
              }
              break;
            default:
              uartSendRspBadCmd = 1;
              break;
          }
        }
        else if (dockRxBuf[UART_RXBUF_COMP] == UART_COMP_TEST)
        { //set test
          if (dockRxBuf[UART_RXBUF_PROP] < FACTORY_TEST_COUNT)
          {
            ShimFactoryTest_setup(
                PRINT_TO_DOCK_UART, (factory_test_t) dockRxBuf[UART_RXBUF_PROP]);
            ShimTask_set(TASK_FACTORY_TEST);
            uartSendRspAck = 1;
          }
          else
          {
            uartSendRspBadCmd = 1;
          }
        }
        else
        {
          uartSendRspBadCmd = 1;
        }
      }
    }
    else
    {
      uartSendRspBadCrc = 1;
    }
    ShimTask_set(TASK_DOCK_RESPOND);
  }
}

void ShimDock_sendRsp(void)
{
  uint8_t uart_resp_len = 0, cr = 0;
  uint16_t uartRespCrc;

  if (uartSendRspAck)
  {
    uartSendRspAck = 0;
    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_ACK_RESPONSE;
  }
  else if (uartSendRspBadCmd)
  {
    uartSendRspBadCmd = 0;
    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_BAD_CMD_RESPONSE;
  }
  else if (uartSendRspBadArg)
  {
    uartSendRspBadArg = 0;
    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_BAD_ARG_RESPONSE;
  }
  else if (uartSendRspBadCrc)
  {
    uartSendRspBadCrc = 0;
    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_BAD_CRC_RESPONSE;
  }
  else if (uartSendRspMac)
  {
    uartSendRspMac = 0;
    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
    *(uartRespBuf + uart_resp_len++) = 8;
    *(uartRespBuf + uart_resp_len++) = UART_COMP_SHIMMER;
    *(uartRespBuf + uart_resp_len++) = UART_PROP_MAC;
    memcpy(uartRespBuf + uart_resp_len, ShimBt_macIdBytesPtrGet(), 6);
    uart_resp_len += 6;
  }
  else if (uartSendRspVer)
  {
    uartSendRspVer = 0;
    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
    *(uartRespBuf + uart_resp_len++) = 9;
    *(uartRespBuf + uart_resp_len++) = UART_COMP_SHIMMER;
    *(uartRespBuf + uart_resp_len++) = UART_PROP_VER;
    *(uartRespBuf + uart_resp_len++) = DEVICE_VER;
    *(uartRespBuf + uart_resp_len++) = FW_IDENTIFIER & 0xFF;
    *(uartRespBuf + uart_resp_len++) = (FW_IDENTIFIER & 0xFF00) >> 8;
    *(uartRespBuf + uart_resp_len++) = FW_VERSION_MAJOR & 0xFF;
    *(uartRespBuf + uart_resp_len++) = (FW_VERSION_MAJOR & 0xFF00) >> 8;
    *(uartRespBuf + uart_resp_len++) = FW_VERSION_MINOR;
    *(uartRespBuf + uart_resp_len++) = FW_VERSION_PATCH;
  }
  else if (uartSendRspBat)
  {
    uartSendRspBat = 0;
    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
    *(uartRespBuf + uart_resp_len++) = 5;
    *(uartRespBuf + uart_resp_len++) = UART_COMP_BAT;
    *(uartRespBuf + uart_resp_len++) = UART_PROP_VALUE;
    uint8_t i = 0;
    for (i = 0; i < 3; i++)
    {
      uartRespBuf[uart_resp_len] = batteryStatus.battStatusRaw.rawBytes[i];
      uart_resp_len++;
    }
  }
  else if (uartSendRspRtcConfigTime)
  {
    uartSendRspRtcConfigTime = 0;
    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
    *(uartRespBuf + uart_resp_len++) = 10;
    *(uartRespBuf + uart_resp_len++) = UART_COMP_SHIMMER;
    *(uartRespBuf + uart_resp_len++) = UART_PROP_RWC_CFG_TIME;

    uint64_t temp_rtcConfigTime = ShimRtc_getRwcConfigTime();
    memcpy(uartRespBuf + uart_resp_len, (uint8_t *) (&temp_rtcConfigTime), 8);

    uart_resp_len += 8;
  }
  else if (uartSendRspCurrentTime)
  {
    uartSendRspCurrentTime = 0;
    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
    *(uartRespBuf + uart_resp_len++) = 10;
    *(uartRespBuf + uart_resp_len++) = UART_COMP_SHIMMER;
    *(uartRespBuf + uart_resp_len++) = UART_PROP_CURR_LOCAL_TIME;

    uint64_t rwc_curr_time_64 = RTC_getRwcTime();
    memcpy(uartRespBuf + uart_resp_len, (uint8_t *) (&rwc_curr_time_64), 8);
    uart_resp_len += 8;
  }
  else if (uartSendRspGdi)
  {
    uartSendRspGdi = 0;
    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
    *(uartRespBuf + uart_resp_len++) = uartDcMemLength + 2;
    *(uartRespBuf + uart_resp_len++) = UART_COMP_DAUGHTER_CARD;
    *(uartRespBuf + uart_resp_len++) = UART_PROP_CARD_ID;
    if ((uartDcMemLength + uart_resp_len) < UART_RSP_PACKET_SIZE)
    {
      //Read from the cached and processed daughterCardIdPage
      memcpy(uartRespBuf + uart_resp_len,
          ShimBrd_getDaughtCardIdPtr() + uartDcMemOffset, uartDcMemLength);
      uart_resp_len += uartDcMemLength;
    }
  }
  else if (uartSendRspGdm)
  {
    uartSendRspGdm = 0;
    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
    *(uartRespBuf + uart_resp_len++) = uartDcMemLength + 2;
    *(uartRespBuf + uart_resp_len++) = UART_COMP_DAUGHTER_CARD;
    *(uartRespBuf + uart_resp_len++) = UART_PROP_CARD_MEM;
    if ((uartDcMemLength + uart_resp_len) < UART_RSP_PACKET_SIZE)
    {
      eepromRead(uartDcMemOffset + 16U, (uint16_t) uartDcMemLength,
          (uartRespBuf + uart_resp_len));
      uart_resp_len += uartDcMemLength;
    }
  }
  else if (uartSendRspGim)
  {
    uartSendRspGim = 0;
    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
    *(uartRespBuf + uart_resp_len++) = uartInfoMemLength + 2;
    *(uartRespBuf + uart_resp_len++) = UART_COMP_SHIMMER;
    *(uartRespBuf + uart_resp_len++) = UART_PROP_INFOMEM;
    if ((uartInfoMemLength + uart_resp_len) < UART_RSP_PACKET_SIZE)
    {
      InfoMem_read(uartInfoMemOffset, uartRespBuf + uart_resp_len, uartInfoMemLength);
    }
    uart_resp_len += uartInfoMemLength;
  }
#if EN_CALIB_DUMP_RSP
  else if (uartSendRspCalibDump)
  {
    uartSendRspCalibDump = 0;
    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
    *(uartRespBuf + uart_resp_len++) = uartCalibRamLength + 2;
    *(uartRespBuf + uart_resp_len++) = UART_COMP_SHIMMER;
    *(uartRespBuf + uart_resp_len++) = UART_PROP_CALIB_DUMP;
    if ((uartCalibRamLength + uart_resp_len) < UART_RSP_PACKET_SIZE)
    {
      InfoMem_readCalib(uartRespBuf + uart_resp_len, uartCalibRamOffset, uartCalibRamLength);
    }
    uart_resp_len += uartCalibRamLength;
  }
#endif
  else if (uartSendRspBtVer)
  {
    uartSendRspBtVer = 0;

    uint8_t btVerStrLen = ShimBt_getBtVerStrLen();

    *(uartRespBuf + uart_resp_len++) = '$';
    *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
    *(uartRespBuf + uart_resp_len++) = 2U + btVerStrLen;
    *(uartRespBuf + uart_resp_len++) = UART_COMP_BT;
    *(uartRespBuf + uart_resp_len++) = UART_PROP_VER;

    memcpy(uartRespBuf + uart_resp_len, ShimBt_getBtVerStrPtr(), btVerStrLen);
    uart_resp_len += btVerStrLen;
  }

  uartRespCrc = (uint16_t) platform_crcData(uartRespBuf, uart_resp_len);
  *(uartRespBuf + uart_resp_len++) = uartRespCrc & 0xff;
  *(uartRespBuf + uart_resp_len++) = (uartRespCrc & 0xff00) >> 8;
  if (cr)
  { //character return was in the old commands
    *(uartRespBuf + uart_resp_len++) = 0x0d;
    *(uartRespBuf + uart_resp_len++) = 0x0a;
  }
#if defined(SHIMMER3R)
  if (shimmerStatus.usbPluggedIn)
  {
    /* respond to commands via usb */
    CDC_Transmit(CDC_CH_DOCK_COMMS, uartRespBuf, uart_resp_len);
  }
  else
#endif
      if (shimmerStatus.docked)
  {
    /* respond to commands via dock usart */
    DockUart_writeBlocking(uartRespBuf, uart_resp_len);
  }
}

uint8_t ShimDock_uartCheckCrc(uint8_t len)
{
  if (len > UART_DATA_LEN_MAX)
  {
    return 0;
  }
  uint16_t uart_rx_crc, uart_calc_crc;
  uart_calc_crc = (uint16_t) platform_crcData(dockRxBuf, len);
  uart_rx_crc = (uint16_t) dockRxBuf[len];
  uart_rx_crc += ((uint16_t) dockRxBuf[(uint8_t) (len + 1)]) << 8;
  return (uart_rx_crc == uart_calc_crc);
}
