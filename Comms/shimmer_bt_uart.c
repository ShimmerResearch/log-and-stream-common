/*
 * shimmer_bt_comms.c
 *
 *  Created on: 22 Jun 2022
 *      Author: MarkNolan
 */

#include "shimmer_bt_uart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(SHIMMER3)
#include "msp430.h"

#include "../../Shimmer_Driver/RN4X/RN4X.h"
#include "../../shimmer_btsd.h"
#include "../5xx_HAL/hal_CRC.h"
#include "../5xx_HAL/hal_DMA.h"
#include "../5xx_HAL/hal_RTC.h"
#include "../5xx_HAL/hal_board.h"

#elif defined(SHIMMER3R) || defined(SHIMMER4_SDK)
#include "bmp3_defs.h"
#include "hal_FactoryTest.h"
#include "s4_sensing.h"
#include "s4_taskList.h"
#include "shimmer_definitions.h"
#include "shimmer_externs.h"
#endif

#include <log_and_stream_externs.h>
#include <Boards/shimmer_boards.h>
#include <Calibration/shimmer_calibration.h>
#include <Configuration/shimmer_config.h>
#include <SDCard/shimmer_sd.h>
#include <SDCard/shimmer_sd_header.h>
#include <SDSync/shimmer_sd_sync.h>
#include <Sensing/shimmer_sensing.h>
#include <TaskList/shimmer_taskList.h>

uint8_t unwrappedResponse[256] = { 0 };
volatile uint8_t gAction;
uint8_t args[MAX_COMMAND_ARG_SIZE], waitingForArgs, waitingForArgsLength, argsSize;

#if defined(SHIMMER3)
volatile char btStatusStr[BT_STAT_STR_LEN_LARGEST + 1U]; /* +1 to always have a null char */
volatile char btRxBuffFullResponse[BT_VER_RESPONSE_LARGEST + 1U]; /* +1 to always have a null char */
uint8_t btRxBuff[MAX_COMMAND_ARG_SIZE], *btRxExp;
#endif
uint8_t *btRxBuffPtr;
volatile COMMS_CRC_MODE btCrcMode;

uint8_t *gActionPtr;
uint8_t *gArgsPtr;

#if defined(SHIMMER3)
uint8_t btStatusStrIndex;
char btVerStrResponse[BT_VER_RESPONSE_LARGEST + 1U]; /* +1 to always have a null char */
#else
char btVerStrResponse[100U]; //CYW20820 app=v01.04.16.16 requires size = 76 chars
#endif

uint8_t (*newBtCmdToProcess_cb)(void);
void (*setMacId_cb)(uint8_t *);

uint16_t numBytesInBtRxBufWhenLastProcessed = 0;
uint16_t indexOfFirstEol;
uint32_t firstProcessFailTicks = 0;

uint8_t useAckPrefixForInstreamResponses;
uint8_t sendAck, sendNack, getCmdWaitingResponse;
uint8_t infomemLength, dcMemLength, calibRamLength;
uint16_t infomemOffset, dcMemOffset, calibRamOffset;

//ExG
uint8_t exgLength, exgChip, exgStartAddr;

#if defined(SHIMMER3R)
uint16_t btRxWaitByteCount = 0;
#endif

uint8_t btDataRateTestState;
uint32_t btDataRateTestCounter;

#if defined(SHIMMER3)
/* Return of 1 brings MSP out of low-power mode */
uint8_t ShimBt_dmaConversionDone(void)
{
  btRxBuffPtr = &btRxBuff[0];
#else
uint8_t ShimBt_dmaConversionDone(uint8_t *rxBuff)
{
  btRxBuffPtr = rxBuff;
#endif
#if defined(SHIMMER3)
  uint8_t bt_waitForStartCmd, bt_waitForMacAddress, bt_waitForVersion,
      bt_waitForInitialBoot, bt_waitForReturnNewLine;
  uint8_t expectedlen = 0U;

  DMA2AndCtsDisable();
  bt_waitForStartCmd = BT_getWaitForStartCmd();
  bt_waitForMacAddress = BT_getWaitForMacAddress();
  bt_waitForVersion = BT_getWaitForVersion();
  bt_waitForInitialBoot = BT_getWaitForInitialBoot();
  bt_waitForReturnNewLine = BT_getWaitForReturnNewLine();

  if (!*btRxExp
      && (areBtStatusStringsEnabled()
          || (shimmerStatus.btConnected || bt_waitForStartCmd || bt_waitForMacAddress
              || bt_waitForVersion || bt_waitForInitialBoot || bt_waitForReturnNewLine)))
  {
    if (bt_waitForStartCmd)
    {
      /* RN42 responds with "CMD\r\n" and RN4678 with "CMD> " */
      uint8_t len = strlen((char *) btRxBuffPtr);
      if (len == 5U && btRxBuffPtr[0] == 'C' && btRxBuffPtr[1] == 'M'
          && btRxBuffPtr[2] == 'D')
      {
        BT_setWaitForStartCmd(0);
        setRnCommandModeActive(1U);
        memset(btRxBuffPtr, 0, len);
        BT_setGoodCommand();
      }
    }
    else if (bt_waitForReturnNewLine)
    {
      uint8_t btOffset = strlen(btRxBuffFullResponse);
      memcpy(btRxBuffFullResponse + btOffset, btRxBuffPtr, strlen((char *) btRxBuffPtr));
      memset(btRxBuffPtr, 0, strlen((char *) btRxBuffPtr));

      uint8_t responseLen = strlen(btRxBuffFullResponse);

      if (btRxBuffFullResponse[responseLen - 1U] == '\r')
      {
        /* Wait for "\n" */
        setDmaWaitingForResponse(1U);
        return 0;
      }
      //Wait for RN4678_CMD
      else if (isBtDeviceRn4678() && responseLen > RN4X_CMD_LEN
          && btRxBuffFullResponse[responseLen - 5U] == 'C'
          && btRxBuffFullResponse[responseLen - 4U] == 'M'
          && btRxBuffFullResponse[responseLen - 3U] == 'D'
          && btRxBuffFullResponse[responseLen - 2U] == '>'
          && btRxBuffFullResponse[responseLen - 1U] == ' ')
      {
        /* Return/New line received */
        BT_setWaitForReturnNewLine(0);
        BT_setGoodCommand();
      }
      else if (btRxBuffFullResponse[responseLen - 2U] == '\r'
          && btRxBuffFullResponse[responseLen - 1U] == '\n')
      {
        if (isBtDeviceRn41orRN42())
        {
          BT_setWaitForReturnNewLine(0);
          BT_setGoodCommand();
        }
        else
        {
          /* Wait for "CMD> " */
          setDmaWaitingForResponse(RN4X_CMD_LEN);
          return 0;
        }
      }
      else
      {
        /* Wait for "\r\n" */
        setDmaWaitingForResponse(2U);
        return 0;
      }
    }
    else if (bt_waitForInitialBoot)
    {
      if (isBtDeviceRn4678())
      {
        expectedlen = BT_STAT_STR_LEN_RN4678_REBOOT;
      }
      else
      {
        expectedlen = BT_STAT_STR_LEN_RN42_REBOOT;
      }
      memset(btRxBuffPtr, 0, expectedlen);
      BT_setWaitForInitialBoot(0);
      BT_setGoodCommand();
    }
    else if (bt_waitForMacAddress)
    {
      setMacId_cb(btRxBuffPtr);

      expectedlen = 14U;
      if (isBtDeviceRn4678())
      {
        expectedlen += RN4X_CMD_LEN; /* Allow for "CMD> " */
      }
      memset(btRxBuffPtr, 0, expectedlen);
      BT_setWaitForMacAddress(0);
      BT_setGoodCommand();
    }
    else if (bt_waitForVersion)
    {
      uint8_t btVerRemainingChars = 0;
      uint8_t btOffset = strlen(btRxBuffFullResponse);
      memcpy(btRxBuffFullResponse + btOffset, btRxBuffPtr, strlen((char *) btRxBuffPtr));
      memset(btRxBuffPtr, 0, strlen((char *) btRxBuffPtr));

      uint8_t btVerLen = strlen(btRxBuffFullResponse);
      enum BT_FIRMWARE_VERSION btFwVerNew = BT_FW_VER_UNKNOWN;

      /* RN41 or RN42 */
      if (btRxBuffFullResponse[0U] == 'V')
      {
        /* RN41_VERSION_RESPONSE_V4_77 or RN42_VERSION_RESPONSE_V4_77 */
        if (btRxBuffFullResponse[4U] == '4' && btRxBuffFullResponse[5U] == '.')
        {
          if (btRxBuffFullResponse[9U] == 'R' && btRxBuffFullResponse[10U] == 'N')
          {
            btVerRemainingChars = RN42_VERSION_RESPONSE_LEN_V4_77;
            btFwVerNew = RN42_V4_77;
          }
          else
          {
            btVerRemainingChars = RN41_VERSION_RESPONSE_LEN_V4_77;
            btFwVerNew = RN41_V4_77;
          }
        }
        /* RN42_VERSION_RESPONSE_V6_15 */
        else if (btRxBuffFullResponse[4U] == '6' && btRxBuffFullResponse[5U] == '.'
            && btRxBuffFullResponse[6U] == '1' && btRxBuffFullResponse[7U] == '5')
        {
          btVerRemainingChars = RN42_VERSION_RESPONSE_LEN_V6_15;
          btFwVerNew = RN42_V6_15;
        }
        /* V6.30 not supported */
        else if (btRxBuffFullResponse[4U] == '6' && btRxBuffFullResponse[5U] == '.'
            && btRxBuffFullResponse[6U] == '3' && btRxBuffFullResponse[7U] == '0')
        {
          triggerShimmerErrorState();
        }
      }
      /* RN4678 */
      else if (btRxBuffFullResponse[0U] == 'R')
      {
        /* RN4678_VERSION_RESPONSE_V1_00_5 */
        if (btRxBuffFullResponse[10U] == '0' && btRxBuffFullResponse[11U] == '0')
        {
          btVerRemainingChars = RN4678_VERSION_LEN_V1_00_5;
          btFwVerNew = RN4678_V1_00_5;
        }
        /* RN4678_VERSION_RESPONSE_V1_11_0 */
        else if (btRxBuffFullResponse[10U] == '1' && btRxBuffFullResponse[11U] == '1')
        {
          btVerRemainingChars = RN4678_VERSION_LEN_V1_11_0;
          btFwVerNew = RN4678_V1_11_0;
        }
        /* RN4678_VERSION_RESPONSE_V1_13_5 */
        else if (btRxBuffFullResponse[10U] == '1' && btRxBuffFullResponse[11U] == '3')
        {
          btVerRemainingChars = RN4678_VERSION_LEN_V1_13_5;
          btFwVerNew = RN4678_V1_13_5;
        }
        /* RN4678_VERSION_RESPONSE_V1_22_0 */
        else if (btRxBuffFullResponse[10U] == '2' && btRxBuffFullResponse[11U] == '2')
        {
          btVerRemainingChars = RN4678_VERSION_LEN_V1_22_0;
          btFwVerNew = RN4678_V1_22_0;
        }
        /* RN4678_VERSION_RESPONSE_V1_23_0 */
        else if (btRxBuffFullResponse[10U] == '2' && btRxBuffFullResponse[11U] == '3')
        {
          btVerRemainingChars = RN4678_VERSION_LEN_V1_23_0;
          btFwVerNew = RN4678_V1_23_0;
        }
      }
      else
      {
        /* Unkown BT module - bail */
        btVerRemainingChars = btVerLen;
      }

      btVerRemainingChars -= btVerLen;

      if (btVerRemainingChars)
      {
        setDmaWaitingForResponse(btVerRemainingChars);
        return 0;
      }
      else
      {
        setBtFwVersion(btFwVerNew);

        /* When storing the BT version, ignore from "\r" onwards */
        uint8_t btVerLen = strlen(btRxBuffFullResponse);
        uint8_t btVerIdx;
        for (btVerIdx = 0; btVerIdx < btVerLen; btVerIdx++)
        {
          if (btRxBuffFullResponse[btVerIdx] == '\r')
          {
            btVerLen = btVerIdx;
            break;
          }
        }
        memcpy(btVerStrResponse, btRxBuffFullResponse, btVerLen);

        memset(btRxBuffFullResponse, 0, strlen((char *) btRxBuffFullResponse));
        BT_setWaitForVersion(0);
        BT_setGoodCommand();
      }
    }
    else
    {
#endif
      if (waitingForArgs)
      {
        if ((!waitingForArgsLength) && (waitingForArgs == 3)
            && (*(gActionPtr) == SET_INFOMEM_COMMAND || *(gActionPtr) == SET_CALIB_DUMP_COMMAND
                || *(gActionPtr) == SET_DAUGHTER_CARD_MEM_COMMAND
                || *(gActionPtr) == SET_EXG_REGS_COMMAND))
        {
          gArgsPtr[0] = btRxBuffPtr[0];
          gArgsPtr[1] = btRxBuffPtr[1];
          gArgsPtr[2] = btRxBuffPtr[2];
          if (*(gActionPtr) == SET_EXG_REGS_COMMAND)
          {
            waitingForArgsLength = gArgsPtr[2];
          }
          else
          {
            waitingForArgsLength = gArgsPtr[0];
          }
          setDmaWaitingForResponse(waitingForArgsLength);
          return 0;
        }
        else if ((!waitingForArgsLength) && (waitingForArgs == 2)
            && (*(gActionPtr) == SET_DAUGHTER_CARD_ID_COMMAND))
        {
          gArgsPtr[0] = btRxBuffPtr[0];
          gArgsPtr[1] = btRxBuffPtr[1];
          if (gArgsPtr[0])
          {
            waitingForArgsLength = gArgsPtr[0];
            setDmaWaitingForResponse(waitingForArgsLength);
          }
          return 0;
        }
        else if ((!waitingForArgsLength) && (waitingForArgs == 1)
            && (*(gActionPtr) == SET_CENTER_COMMAND || *(gActionPtr) == SET_CONFIGTIME_COMMAND
                || *(gActionPtr) == SET_EXPID_COMMAND || *(gActionPtr) == SET_SHIMMERNAME_COMMAND))
        {
          gArgsPtr[0] = btRxBuffPtr[0];
          if (gArgsPtr[0])
          {
            waitingForArgsLength = gArgsPtr[0];
            setDmaWaitingForResponse(waitingForArgsLength);
            return 0;
          }
        }

#if defined(SHIMMER3)
        else if (*(gActionPtr) == RN4678_STATUS_STRING_SEPARATOR)
        {
          return ShimBt_parseRn4678Status();
        }
#endif

        else if (*(gActionPtr) == ACK_COMMAND_PROCESSED)
        {
          /* If waiting for command byte */
          if (!waitingForArgsLength)
          {
            /* Save command byte */
            gArgsPtr[0] = btRxBuffPtr[0];

            if (btRxBuffPtr[0] == SD_SYNC_RESPONSE)
            {
              /* Wait for flag to be received */
              waitingForArgsLength = 1U;
              setDmaWaitingForResponse(waitingForArgsLength);
              return 0;
            }
          }
        }

        if (waitingForArgsLength)
        {
          memcpy(gArgsPtr + waitingForArgs, btRxBuffPtr, waitingForArgsLength);
        }
        else
        {
          memcpy(gArgsPtr, btRxBuffPtr, waitingForArgs);
        }

        waitingForArgsLength = 0;
        waitingForArgs = 0;
        argsSize = 0;
        setDmaWaitingForResponse(1U);
        if (newBtCmdToProcess_cb)
        {
          return newBtCmdToProcess_cb();
        }
        else
        {
          return 1;
        }
      }
      else
      {
        uint8_t data = btRxBuffPtr[0];
        uint8_t wakeupMcu = 0;

        switch (data)
        {
        case INQUIRY_COMMAND:
        case DUMMY_COMMAND:
        case GET_SAMPLING_RATE_COMMAND:
        case TOGGLE_LED_COMMAND:
        case START_STREAMING_COMMAND:
        case GET_STATUS_COMMAND:
        case GET_VBATT_COMMAND:
        case GET_TRIAL_CONFIG_COMMAND:
        case START_SDBT_COMMAND:
        case GET_CONFIG_SETUP_BYTES_COMMAND:
        case STOP_STREAMING_COMMAND:
        case STOP_SDBT_COMMAND:
        case START_LOGGING_COMMAND:
        case STOP_LOGGING_COMMAND:
        case GET_LN_ACCEL_CALIBRATION_COMMAND:
        case GET_GYRO_CALIBRATION_COMMAND:
        case GET_MAG_CALIBRATION_COMMAND:
        case GET_WR_ACCEL_CALIBRATION_COMMAND:
        case GET_GSR_RANGE_COMMAND:
        case GET_ALL_CALIBRATION_COMMAND:
        case DEPRECATED_GET_DEVICE_VERSION_COMMAND:
        case GET_DEVICE_VERSION_COMMAND:
        case GET_FW_VERSION_COMMAND:
        case GET_CHARGE_STATUS_LED_COMMAND:
        case GET_BUFFER_SIZE_COMMAND:
        case GET_UNIQUE_SERIAL_COMMAND:
        case GET_WR_ACCEL_RANGE_COMMAND:
        case GET_MAG_GAIN_COMMAND:
        case GET_MAG_SAMPLING_RATE_COMMAND:
        case GET_WR_ACCEL_SAMPLING_RATE_COMMAND:
        case GET_WR_ACCEL_LPMODE_COMMAND:
        case GET_WR_ACCEL_HRMODE_COMMAND:
        case GET_GYRO_RANGE_COMMAND:
        case GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND:
        case GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND:
        case GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND:
        case GET_GYRO_SAMPLING_RATE_COMMAND:
        case GET_ALT_ACCEL_RANGE_COMMAND:
        case GET_PRESSURE_OVERSAMPLING_RATIO_COMMAND:
        case GET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
        case RESET_TO_DEFAULT_CONFIGURATION_COMMAND:
        case RESET_CALIBRATION_VALUE_COMMAND:
        case GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND:
        case GET_BT_COMMS_BAUD_RATE:
        case GET_CENTER_COMMAND:
        case GET_SHIMMERNAME_COMMAND:
        case GET_EXPID_COMMAND:
        case GET_MYID_COMMAND:
        case GET_NSHIMMER_COMMAND:
        case GET_CONFIGTIME_COMMAND:
        case GET_DIR_COMMAND:
        case GET_DERIVED_CHANNEL_BYTES:
        case GET_RWC_COMMAND:
        case UPD_SDLOG_CFG_COMMAND:
        case UPD_CALIB_DUMP_COMMAND:
          //case UPD_FLASH_COMMAND:
        case GET_BT_VERSION_STR_COMMAND:
        case GET_ALT_ACCEL_CALIBRATION_COMMAND:
        case GET_ALT_ACCEL_SAMPLING_RATE_COMMAND:
        case GET_ALT_MAG_CALIBRATION_COMMAND:
        case GET_ALT_MAG_SAMPLING_RATE_COMMAND:
          *(gActionPtr) = data;
          if (newBtCmdToProcess_cb)
          {
            newBtCmdToProcess_cb();
          }
          setDmaWaitingForResponse(1U);
          /* Wake-up MCU so that the get command can be processed */
          wakeupMcu = 1U;
          break;
        case SET_WR_ACCEL_RANGE_COMMAND:
        case SET_WR_ACCEL_SAMPLING_RATE_COMMAND:
        case SET_MAG_GAIN_COMMAND:
        case SET_CHARGE_STATUS_LED_COMMAND:
        case SET_MAG_SAMPLING_RATE_COMMAND:
        case SET_WR_ACCEL_LPMODE_COMMAND:
        case SET_WR_ACCEL_HRMODE_COMMAND:
        case SET_GYRO_RANGE_COMMAND:
        case SET_GYRO_SAMPLING_RATE_COMMAND:
        case SET_ALT_ACCEL_RANGE_COMMAND:
        case SET_PRESSURE_OVERSAMPLING_RATIO_COMMAND:
        case SET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
        case SET_GSR_RANGE_COMMAND:
        case SET_BT_COMMS_BAUD_RATE:
        case SET_CENTER_COMMAND:
        case SET_SHIMMERNAME_COMMAND:
        case SET_EXPID_COMMAND:
        case SET_MYID_COMMAND:
        case SET_NSHIMMER_COMMAND:
        case SET_CONFIGTIME_COMMAND:
        case SET_CRC_COMMAND:
        case SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE:
        case SET_DATA_RATE_TEST:
        case SET_FACTORY_TEST:
        case SET_ALT_ACCEL_SAMPLING_RATE_COMMAND:
        case SET_ALT_MAG_SAMPLING_RATE_COMMAND:
          *(gActionPtr) = data;
          waitingForArgs = 1U;
          break;
        case SET_SAMPLING_RATE_COMMAND:
        case GET_DAUGHTER_CARD_ID_COMMAND:
        case SET_DAUGHTER_CARD_ID_COMMAND:
          *(gActionPtr) = data;
          waitingForArgs = 2U;
          break;
        case SET_SENSORS_COMMAND:
        case GET_EXG_REGS_COMMAND:
        case SET_EXG_REGS_COMMAND:
        case GET_DAUGHTER_CARD_MEM_COMMAND:
        case SET_DAUGHTER_CARD_MEM_COMMAND:
        case SET_TRIAL_CONFIG_COMMAND:
        case GET_INFOMEM_COMMAND:
        case SET_INFOMEM_COMMAND:
        case GET_CALIB_DUMP_COMMAND:
        case SET_CALIB_DUMP_COMMAND:
          *(gActionPtr) = data;
          waitingForArgs = 3U;
          break;
        case SET_CONFIG_SETUP_BYTES_COMMAND:
          *(gActionPtr) = data;
          waitingForArgs = 4U;
          break;
        case SET_RWC_COMMAND:
        case SET_DERIVED_CHANNEL_BYTES:
          *(gActionPtr) = data;
          waitingForArgs = 8U;
          break;
        case SET_LN_ACCEL_CALIBRATION_COMMAND:
        case SET_GYRO_CALIBRATION_COMMAND:
        case SET_MAG_CALIBRATION_COMMAND:
        case SET_WR_ACCEL_CALIBRATION_COMMAND:
        case SET_ALT_ACCEL_CALIBRATION_COMMAND:
        case SET_ALT_MAG_CALIBRATION_COMMAND:
          *(gActionPtr) = data;
          //TODO change to SC_DATA_LEN_STD_IMU_CALIB when calib code goes to common
          waitingForArgs = 21;
          break;
#if defined(SHIMMER3)
        case RN4678_STATUS_STRING_SEPARATOR:
          memset(btStatusStr, 0, sizeof(btStatusStr));
          btStatusStr[0U] = RN4678_STATUS_STRING_SEPARATOR;
          btStatusStrIndex = 1U;
          *(gActionPtr) = data;
          /* Minus 1 because we've already received 1 x RN4678_STATUS_STRING_SEPARATOR */
          waitingForArgs = BT_STAT_STR_LEN_SMALLEST - 1U;
          break;
#endif
        case ACK_COMMAND_PROCESSED:
          /* Wait for command byte */
          *(gActionPtr) = data;
          waitingForArgs = 1U;
          break;
        case SET_SD_SYNC_COMMAND:
          /* Store local time as early as possible after sync bytes have been received */
          ShimSdSync_saveLocalTime();

          *(gActionPtr) = data;
          waitingForArgs = SYNC_PACKET_PAYLOAD_SIZE + BT_SD_SYNC_CRC_MODE;
          break;
        default:
          setDmaWaitingForResponse(1U);
          break;
        }

        if (waitingForArgs)
        {
          setDmaWaitingForResponse(waitingForArgs);
        }

        return wakeupMcu;
      }
#if defined(SHIMMER3)
    }
  }
  else
  {
    uint8_t len = strlen((char *) btRxExp);
    if (!memcmp(btRxBuffPtr, btRxExp, len))
    {
      memset(btRxBuffPtr, 0, len);
      BT_setGoodCommand();
    }
    else
    {
      _NOP(); //bad command trap: reaching here = serious BT problem
    }
  }

  setDmaWaitingForResponseIfStatusStrEnabled();
#endif
  return 0;
}

#if defined(SHIMMER3)
uint8_t ShimBt_parseRn4678Status(void)
{
  uint8_t numberOfCharRemaining = 0U;
  uint8_t bringUcOutOfSleep = 0U;

  memcpy(btStatusStr + btStatusStrIndex, btRxBuffPtr, waitingForArgs);
  memset(btRxBuffPtr, 0, waitingForArgs);
  btStatusStrIndex += waitingForArgs;

  enum BT_FIRMWARE_VERSION btFwVer = getBtFwVersion();

  uint8_t firstChar = btStatusStr[1U];
  switch (firstChar)
  {
  case 'A':
    /* "%AUTHENTICATED%" */
    if (btStatusStr[14U] == '%')
    {
      /* TODO */
    }
    /* "%AUTHEN" - Read outstanding bytes */
    else if (btStatusStr[6U] == 'N')
    {
      numberOfCharRemaining = BT_STAT_STR_LEN_AUTHENTICATED - BT_STAT_STR_LEN_SMALLEST;
    }
    /* "%AUTH_FAIL%" */
    else if (btStatusStr[10U] == '%')
    {
      /* TODO */
    }
    /* "%AUTH_FA" - Read outstanding bytes */
    else if (btStatusStr[6U] == 'F')
    {
      numberOfCharRemaining = BT_STAT_STR_LEN_AUTH_FAIL - BT_STAT_STR_LEN_SMALLEST;
    }
    break;
  case 'B':
    /* "%BONDED%" */
    if (btStatusStr[7U] == '%')
    {
      /* TODO */
    }
    /* "%BONDED" - Read outstanding bytes */
    else
    {
      numberOfCharRemaining = BT_STAT_STR_LEN_BONDED - BT_STAT_STR_LEN_SMALLEST;
    }
    break;
  case 'C':
    if (btStatusStr[5U] == 'E')
    {
      /* "%CONNECT,001BDC06A3D5%" - RN4678 */
      if (btStatusStr[21U] == '%')
      {
        setRn4678ConnectionState(RN4678_CONNECTED_CLASSIC);
      }
      /* "%CONNECT" for RN42 v4.77 or "%CONNECT,001BDC06A3D5," - RN42 v6.15 */
      else if ((btFwVer == RN41_V4_77 && btStatusStr[7U] == 'T')
          || (btFwVer == RN42_V4_77 && btStatusStr[7U] == 'T')
          || (btFwVer == RN42_V6_15 && btStatusStr[21U] == ','))
      {
        ShimBt_handleBtRfCommStateChange(TRUE);
        bringUcOutOfSleep = 1U;
      }
      else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
      {
        if (btFwVer == RN41_V4_77 || btFwVer == RN42_V4_77)
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_RN42_v477_CONNECT;
        }
        else if (btFwVer == RN42_V6_15)
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_RN42_v615_CONNECT;
        }
        else
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_RN4678_CONNECT;
        }
        numberOfCharRemaining -= BT_STAT_STR_LEN_SMALLEST;
      }
    }
    else if (btStatusStr[5U] == '_')
    {
      /* "%CONN_PARAM,000C,0000,03C0%" - RN4678 */
      if (btStatusStr[26U] == '%')
      {
        //TODO
      }
      else
      {
        numberOfCharRemaining = BT_STAT_STR_LEN_CONN_PARAM - BT_STAT_STR_LEN_SMALLEST;
      }
    }
    break;
  case 'D':
    /* "%DISCONN%" -> RN4678 */
    if (btStatusStr[8U] == '%')
    {
      /* This if is needed here for BLE connections as a
       * disconnect over BLE does not trigger an
       * RFCOMM_CLOSE status change as it does for classic
       * Bluetooth connections. */
      if (shimmerStatus.btConnected)
      {
        ShimBt_handleBtRfCommStateChange(FALSE);
        bringUcOutOfSleep = 1U;
      }

      setRn4678ConnectionState(RN4678_DISCONNECTED);
    }
    /* "%DISCONNECT" -> RN42 */
    else if (btStatusStr[10U] == 'T')
    {
      ShimBt_handleBtRfCommStateChange(FALSE);
      bringUcOutOfSleep = 1U;
    }
    /* "%DISCON" - Read outstanding bytes */
    else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
    {
      if (isBtDeviceRn41orRN42())
      {
        numberOfCharRemaining = BT_STAT_STR_LEN_RN42_DISCONNECT;
      }
      else
      {
        numberOfCharRemaining = BT_STAT_STR_LEN_RN4678_DISCONN;
      }
      numberOfCharRemaining -= BT_STAT_STR_LEN_SMALLEST;
    }
    break;
  case 'E':
    if (btStatusStr[2U] == 'N')
    {
      if (btStatusStr[5U] == 'I')
      {
        /* "%END_INQ%" */
        if (btStatusStr[8U] == '%')
        {
          /* TODO */
        }
        /* "%END_IN" - Read outstanding bytes */
        else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_END_INQ - BT_STAT_STR_LEN_SMALLEST;
        }
      }
      else if (btStatusStr[5U] == 'S')
      {
        /* "%END_SCN%" */
        if (btStatusStr[8U] == '%')
        {
          /* TODO */
        }
        /* "%END_SC" - Read outstanding bytes */
        else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_END_SCN - BT_STAT_STR_LEN_SMALLEST;
        }
      }
    }
    else if (btStatusStr[2U] == 'R')
    {
      if (btStatusStr[5U] == 'C')
      {
        /* %ERR_CON is common to two status strings. If detected, read two more chars to determine which one it is */
        /* "%ERR_CONN%" */
        if (btStatusStr[9U] == '%')
        {
          /* TODO */
        }
        /* "%ERR_CONN_PARAM%" */
        else if (btStatusStr[15U] == '%')
        {
          /* TODO */
        }
        /* "%ERR_CONN_" - Read outstanding bytes */
        else if (btStatusStr[9U] == '_')
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_ERR_CONN_PARAM - BT_STAT_STR_LEN_ERR_CONN;
        }
        /* "%ERR_CON" - Read outstanding bytes */
        else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_ERR_CONN - BT_STAT_STR_LEN_SMALLEST;
        }
      }
      else if (btStatusStr[5U] == 'L')
      {
        /* "%ERR_LSEC%" */
        if (btStatusStr[9U] == '%')
        {
          /* TODO */
        }
        /* "%ERR_LSE" - Read outstanding bytes */
        else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_ERR_LSEC - BT_STAT_STR_LEN_SMALLEST;
        }
      }
      else if (btStatusStr[5U] == 'S')
      {
        /* "%ERR_SEC%" */
        if (btStatusStr[8U] == '%')
        {
          /* TODO */
        }
        /* "%ERR_SE" - Read outstanding bytes */
        else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_ERR_SEC - BT_STAT_STR_LEN_SMALLEST;
        }
      }
    }
    break;
  case 'F':
    /* "%FACTORY_RESET%" */
    if (btStatusStr[14U] == '%')
    {
      /* TODO */
    }
    /* "%FACTOR" - Read outstanding bytes */
    else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
    {
      numberOfCharRemaining = BT_STAT_STR_LEN_FACTORY_RESET - BT_STAT_STR_LEN_SMALLEST;
    }
    break;
  case 'L':
    if (btStatusStr[2U] == 'C')
    {
      /* "%LCONNECT,001BDC06A3D5,1%" - RN4678 BLE mode */
      if (btStatusStr[24U] == '%')
      {
        setRn4678ConnectionState(RN4678_CONNECTED_BLE);

        /* RN4678 seems to assume charactertic is advice once BLE connected */
        ShimBt_handleBtRfCommStateChange(TRUE);

        bringUcOutOfSleep = 1U;
      }
      else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
      {
        numberOfCharRemaining = BT_STAT_STR_LEN_RN4678_LCONNECT - BT_STAT_STR_LEN_SMALLEST;
      }
      break;
    }
    else if (btStatusStr[2U] == 'B')
    {
      /* "%LBONDED%" */
      if (btStatusStr[8U] == '%')
      {
        /* TODO */
      }
      /* "%LBONDE" - Read outstanding bytes */
      else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
      {
        numberOfCharRemaining = BT_STAT_STR_LEN_LBONDED - BT_STAT_STR_LEN_SMALLEST;
      }
    }
    else if (btStatusStr[2U] == 'S')
    {
      if (btStatusStr[6U] == 'R')
      {
        /* "%LSECURED%" */
        if (btStatusStr[9U] == '%')
        {
          /* TODO */
        }
        /* "%LSECURE_FAIL%" */
        else if (btStatusStr[13U] == '%')
        {
          /* TODO */
        }
        /* "%LSECURE_F" */
        else if (btStatusStr[9U] == 'F')
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_LSECURE_FAIL - BT_STAT_STR_LEN_LSECURED;
        }
        /* "%LSECUR" - Read outstanding bytes */
        else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_LSECURED - BT_STAT_STR_LEN_SMALLEST;
        }
      }
      else if (btStatusStr[6U] == 'A')
      {
        /* "%LSTREAM_OPEN%" */
        if (btStatusStr[13U] == '%')
        {
          /* TODO */
        }
        /* "%LSTREA" - Read outstanding bytes */
        else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_LSTREAM_OPEN - BT_STAT_STR_LEN_SMALLEST;
        }
      }
    }
    break;
  case 'M':
    /* "%MLDP_MODE%" */
    if (btStatusStr[10U] == '%')
    {
      /* TODO */
    }
    /* "%MLDP_M" - Read outstanding bytes */
    else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
    {
      numberOfCharRemaining = BT_STAT_STR_LEN_MLDP_MODE - BT_STAT_STR_LEN_SMALLEST;
    }
    break;
  case 'R':
    if (btStatusStr[2U] == 'E')
    {
      /* "%REBOOT%" -> RN4678 */
      if (btStatusStr[7U] == '%')
      {
        /* TODO */
        _NOP();
      }
      else
      {
        if (isBtDeviceRn41orRN42())
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_RN42_REBOOT;
        }
        else
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_RN4678_REBOOT;
        }
        numberOfCharRemaining -= BT_STAT_STR_LEN_SMALLEST;
      }
    }
    else if (btStatusStr[2U] == 'F')
    {
      if (btStatusStr[8U] == 'C')
      {
        /* "%RFCOMM_CLOSE%" */
        if (btStatusStr[13U] == '%')
        {
          ShimBt_handleBtRfCommStateChange(FALSE);
          bringUcOutOfSleep = 1U;
        }
        /* "%RFCOMM_CLOSE" - Read outstanding bytes */
        else if (btStatusStr[13U] == '\0')
        {
          numberOfCharRemaining = BT_STAT_STR_LEN_RFCOMM_CLOSE - BT_STAT_STR_LEN_RFCOMM_OPEN;
        }
      }
      /* "%RFCOMM_OPEN%" */
      else if (btStatusStr[12U] == '%')
      {
        ShimBt_handleBtRfCommStateChange(TRUE);
        bringUcOutOfSleep = 1U;
      }
      /* "%RFCOMM" - Read outstanding bytes */
      else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
      {
        numberOfCharRemaining = BT_STAT_STR_LEN_RFCOMM_OPEN - BT_STAT_STR_LEN_SMALLEST;
      }
    }
    break;
  case 'S':
    if (btStatusStr[3U] == 'C')
    {
      /* "%SECURED%" */
      if (btStatusStr[8U] == '%')
      {
        /* TODO */
      }
      /* "%SECURE_FAIL%" */
      else if (btStatusStr[12U] == '%')
      {
        /* TODO */
      }
      /* "%SECURE_F" - Read outstanding bytes */
      else if (btStatusStr[7U] == '_')
      {
        numberOfCharRemaining = BT_STAT_STR_LEN_SECURE_FAIL - BT_STAT_STR_LEN_SECURED;
      }
      /* "%SECURE" - Read outstanding bytes */
      else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
      {
        numberOfCharRemaining = BT_STAT_STR_LEN_SECURED - BT_STAT_STR_LEN_SMALLEST;
      }
    }
    else if (btStatusStr[3U] == 'S')
    {
      /* "%SESSION_OPEN%" */
      if (btStatusStr[13U] == '%')
      {
        /* TODO */
      }
      /* "%SESSION_CLOSE%" */
      else if (btStatusStr[14U] == '%')
      {
        /* TODO */
      }
      /* "%SESSION_CLOSE" - Read outstanding bytes */
      else if (btStatusStr[13U] == 'E')
      {
        numberOfCharRemaining = BT_STAT_STR_LEN_SESSION_CLOSE - BT_STAT_STR_LEN_SESSION_OPEN;
      }
      /* "%SESSIO" - Read outstanding bytes */
      else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST] == '\0')
      {
        numberOfCharRemaining = BT_STAT_STR_LEN_SESSION_OPEN - BT_STAT_STR_LEN_SMALLEST;
      }
    }
    break;
  default:
    break;
  }

  if (numberOfCharRemaining)
  {
    waitingForArgs = numberOfCharRemaining;
  }
  else
  {
    waitingForArgs = 0;
    numberOfCharRemaining = 1U;
  }
  setDmaWaitingForResponse(numberOfCharRemaining);
  return bringUcOutOfSleep;
}
#endif

void ShimBt_resetBtRxVariablesOnConnect(void)
{
  /* Reset to unsupported command */
  *(gActionPtr) = ACK_COMMAND_PROCESSED - 1U;
  waitingForArgs = 0;
  waitingForArgsLength = 0;
}

#if defined(SHIMMER3)
void ShimBt_resetBtRxBuff(void)
{
  memset(btRxBuff, 0, sizeof(btRxBuff));
}
#endif

#if defined(SHIMMER3)
void ShimBt_btCommsProtocolInit(uint8_t (*newBtCmdToProcessCb)(void),
    void (*setMacIdCb)(uint8_t *),
    uint8_t *actionPtr,
    uint8_t *argsPtr)
#else
void ShimBt_btCommsProtocolInit(uint8_t (*newBtCmdToProcessCb)(void))
#endif
{
  ShimBt_setCrcMode(CRC_OFF);
  numBytesInBtRxBufWhenLastProcessed = 0;
  indexOfFirstEol = 0;
  firstProcessFailTicks = 0;
  memset(unwrappedResponse, 0, sizeof(unwrappedResponse));

  newBtCmdToProcess_cb = newBtCmdToProcessCb;
#if defined(SHIMMER3)
  setMacId_cb = setMacIdCb;

  gActionPtr = actionPtr;
  gArgsPtr = argsPtr;
#else
  gActionPtr = &gAction;
  gArgsPtr = &args[0];
#endif

#if defined(SHIMMER3)
  btRxExp = BT_getExpResp();
#endif

  waitingForArgs = 0;
  waitingForArgsLength = 0;
  argsSize = 0;

#if defined(SHIMMER3)
  memset(btStatusStr, 0, sizeof(btStatusStr));

  memset(btRxBuffFullResponse, 0x00,
      sizeof(btRxBuffFullResponse) / sizeof(btRxBuffFullResponse[0]));
  setBtRxFullResponsePtr(btRxBuffFullResponse);

  memset(btRxBuff, 0x00, sizeof(btRxBuff) / sizeof(btRxBuff[0]));
  DMA2_init((uint16_t *) &UCA1RXBUF, (uint16_t *) btRxBuff, sizeof(btRxBuff));
  DMA2_transferDoneFunction(&ShimBt_dmaConversionDone);
  //DMA2SZ = 1U;
  //DMA2_enable();
#endif

  memset(btVerStrResponse, 0x00, sizeof(btVerStrResponse) / sizeof(btVerStrResponse[0]));

  ShimBt_setBtDataRateTestState(0);

  ShimBt_resetBtResponseVars();
}

void ShimBt_resetBtResponseVars(void)
{
  sendAck = 0;
  sendNack = 0;
  getCmdWaitingResponse = 0;
  useAckPrefixForInstreamResponses = 1;

#if defined(SHIMMER4_SDK)
  i2cvBattBtRsp = 0;
#endif
}

uint8_t ShimBt_isWaitingForArgs(void)
{
  return waitingForArgs;
}

uint8_t ShimBt_getBtVerStrLen(void)
{
  return strlen(btVerStrResponse);
}

char *ShimBt_getBtVerStrPtr(void)
{
  return &btVerStrResponse[0];
}

#if defined(SHIMMER3R)
void ShimBt_updateBtVer(void)
{
  BT_generateCyw20820FirmwareVersionStr(&btVerStrResponse[0]);
}
#endif

void ShimBt_processCmd(void)
{
#if !defined(SHIMMER3)
  uint64_t temp64;
#endif
  gConfigBytes *storedConfig = ShimConfig_getStoredConfig();
  uint8_t *configTimeTextPtr;

  uint32_t config_time;
  uint8_t my_config_time[4];
  uint8_t name_len;

  uint8_t update_sdconfig = 0, update_calib_dump_file = 0;
  uint8_t fullSyncResp[SYNC_PACKET_MAX_SIZE] = { 0 };
  uint8_t sensorCalibId;

  switch (gAction)
  {
  case INQUIRY_COMMAND:
  case GET_SAMPLING_RATE_COMMAND:
  case GET_WR_ACCEL_RANGE_COMMAND:
  case GET_MAG_GAIN_COMMAND:
  case GET_MAG_SAMPLING_RATE_COMMAND:
  case GET_STATUS_COMMAND:
  case GET_VBATT_COMMAND:
  case GET_TRIAL_CONFIG_COMMAND:
  case GET_CENTER_COMMAND:
  case GET_SHIMMERNAME_COMMAND:
  case GET_EXPID_COMMAND:
  case GET_CONFIGTIME_COMMAND:
  case GET_DIR_COMMAND:
  case GET_NSHIMMER_COMMAND:
  case GET_MYID_COMMAND:
  case GET_WR_ACCEL_SAMPLING_RATE_COMMAND:
  case GET_WR_ACCEL_LPMODE_COMMAND:
  case GET_WR_ACCEL_HRMODE_COMMAND:
  case GET_GYRO_RANGE_COMMAND:
  case GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND:
  case GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND:
  case GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND:
  case GET_GYRO_SAMPLING_RATE_COMMAND:
  case GET_ALT_ACCEL_RANGE_COMMAND:
  case GET_PRESSURE_OVERSAMPLING_RATIO_COMMAND:
  case GET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
  case GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND:
  case GET_CONFIG_SETUP_BYTES_COMMAND:
  case GET_BT_VERSION_STR_COMMAND:
  case GET_GSR_RANGE_COMMAND:

  case DEPRECATED_GET_DEVICE_VERSION_COMMAND:
  case GET_DEVICE_VERSION_COMMAND:

  case GET_FW_VERSION_COMMAND:
  case GET_CHARGE_STATUS_LED_COMMAND:
  case GET_BUFFER_SIZE_COMMAND:
  case GET_UNIQUE_SERIAL_COMMAND:
  case GET_BT_COMMS_BAUD_RATE:
  case GET_DERIVED_CHANNEL_BYTES:
  case GET_RWC_COMMAND:

  case GET_ALL_CALIBRATION_COMMAND:
  case GET_LN_ACCEL_CALIBRATION_COMMAND:
  case GET_GYRO_CALIBRATION_COMMAND:
  case GET_MAG_CALIBRATION_COMMAND:
  case GET_WR_ACCEL_CALIBRATION_COMMAND:
  case GET_ALT_ACCEL_CALIBRATION_COMMAND:
  case GET_ALT_ACCEL_SAMPLING_RATE_COMMAND:
  case GET_ALT_MAG_CALIBRATION_COMMAND:
  case GET_ALT_MAG_SAMPLING_RATE_COMMAND:
    getCmdWaitingResponse = gAction;
    break;

  case DUMMY_COMMAND:
    break;

  case TOGGLE_LED_COMMAND:
    shimmerStatus.toggleLedRedCmd ^= 1;
    break;
  case START_STREAMING_COMMAND:
    shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_START;
    if (!shimmerStatus.sensing)
    {
      ShimTask_setStartSensing();
    }
    break;
  case START_SDBT_COMMAND:
    if (!shimmerStatus.sensing)
    {
      ShimTask_setStartSensing();
    }
    shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_START;
    shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_START;
    if (shimmerStatus.sdlogReady && !shimmerStatus.sdBadFile)
    {
      shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_START;
    }
    break;
  case START_LOGGING_COMMAND:
    shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_START;
    if (!shimmerStatus.sensing)
    {
      ShimTask_setStartSensing();
    }
    break;
  case SET_CRC_COMMAND:
    ShimBt_setCrcMode((COMMS_CRC_MODE) args[0]);
    break;
  case SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE:
    useAckPrefixForInstreamResponses = args[0];
    break;
  case STOP_STREAMING_COMMAND:
    shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_STOP;
    ShimTask_setStopStreaming();
    break;
  case STOP_SDBT_COMMAND:
    shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_STOP;
    shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_STOP;
    ShimTask_setStopSensing();
    break;
  case STOP_LOGGING_COMMAND:
    shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_STOP;
    ShimTask_setStopLogging();
    break;
  case SET_SENSORS_COMMAND:
    ShimConfig_storedConfigSet(&args[0], NV_SENSORS0, 3);
    ShimBt_settingChangeCommon(NV_SENSORS0, SDH_SENSORS0, 3);
    break;
#if defined(SHIMMER4_SDK)
  case GET_I2C_BATT_STATUS_COMMAND:
    getCmdWaitingResponse = gAction;
    break;
  case SET_I2C_BATT_STATUS_FREQ_COMMAND:
    temp16 = args[0] + ((uint16_t) args[1] << 8);
    I2C_readBattSetFreq(temp16);
    break;
#endif
  case SET_TRIAL_CONFIG_COMMAND:
    storedConfig->rawBytes[NV_SD_TRIAL_CONFIG0] = args[0];
    storedConfig->rawBytes[NV_SD_TRIAL_CONFIG1] = args[1];
    storedConfig->rawBytes[NV_SD_BT_INTERVAL] = args[2];

    //Save TRIAL_CONFIG0, TRIAL_CONFIG1 and BT_INTERVAL
    ShimBt_settingChangeCommon(NV_SD_TRIAL_CONFIG0, SDH_TRIAL_CONFIG0, 3);
    break;
  case SET_CENTER_COMMAND:
    storedConfig->masterEnable = args[0] & 0x01;
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE4, SDH_CONFIG_SETUP_BYTE4, 1);
    break;
  case SET_SHIMMERNAME_COMMAND:
    name_len = (args[0] < sizeof(storedConfig->shimmerName)) ?
        args[0] :
        sizeof(storedConfig->shimmerName);
    memset(&storedConfig->shimmerName[0], 0, sizeof(storedConfig->shimmerName));
    memcpy(&storedConfig->shimmerName[0], &args[1], name_len);
    InfoMem_write(NV_SD_SHIMMER_NAME, &storedConfig->shimmerName[0],
        sizeof(storedConfig->shimmerName));
    ShimSd_setShimmerName();
    update_sdconfig = 1;
    break;
  case SET_EXPID_COMMAND:
    name_len = (args[0] < sizeof(storedConfig->expIdName)) ?
        args[0] :
        sizeof(storedConfig->expIdName);
    memset(&storedConfig->expIdName[0], 0, sizeof(storedConfig->expIdName));
    memcpy(&storedConfig->expIdName[0], &args[1], name_len);
    InfoMem_write(NV_SD_EXP_ID_NAME, &storedConfig->expIdName[0],
        sizeof(storedConfig->expIdName));
    ShimSd_setExpIdName();
    update_sdconfig = 1;
    break;
  case SET_CONFIGTIME_COMMAND:
    configTimeTextPtr = ShimSd_configTimeTextPtrGet();
    name_len = args[0] < (MAX_CHARS - 1) ? args[0] : (MAX_CHARS - 1);
    memcpy(configTimeTextPtr, &args[1], name_len);
    configTimeTextPtr[name_len] = 0;
    ShimSd_setName();
    config_time = atol((char *) configTimeTextPtr);
    my_config_time[3] = *((uint8_t *) &config_time);
    my_config_time[2] = *(((uint8_t *) &config_time) + 1);
    my_config_time[1] = *(((uint8_t *) &config_time) + 2);
    my_config_time[0] = *(((uint8_t *) &config_time) + 3);
    ShimSdHead_sdHeadTextSet(&my_config_time[0], SDH_CONFIG_TIME_0, 4);
    ShimConfig_storedConfigSet(&my_config_time[0], NV_SD_CONFIG_TIME, 4);
    InfoMem_write(NV_SD_CONFIG_TIME, &storedConfig->rawBytes[NV_SD_CONFIG_TIME], 4);
    update_sdconfig = 1;
    break;
  case SET_NSHIMMER_COMMAND:
    storedConfig->numberOfShimmers = args[0];
    ShimBt_settingChangeCommon(NV_SD_NSHIMMER, SDH_NSHIMMER, 1);
    break;
  case SET_MYID_COMMAND:
    storedConfig->myTrialID = args[0];
    ShimBt_settingChangeCommon(NV_SD_MYTRIAL_ID, SDH_MYTRIAL_ID, 1);
    break;
  case SET_WR_ACCEL_RANGE_COMMAND:
    storedConfig->wrAccelRange = args[0] < 4 ? (args[0] & 0x03) : 0;
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE0, SDH_CONFIG_SETUP_BYTE0, 1);
    break;
  case GET_EXG_REGS_COMMAND:
    if (args[0] < 2 && args[1] < 10 && args[2] < 11)
    {
      exgChip = args[0];
      exgStartAddr = args[1];
      exgLength = args[2];
    }
    else
    {
      exgLength = 0;
    }
    getCmdWaitingResponse = gAction;
    break;
  case SET_WR_ACCEL_SAMPLING_RATE_COMMAND:
#if defined(SHIMMER3)
    storedConfig->wrAccelRate
        = (args[0] <= LSM303DLHC_ACCEL_1_344kHz) ? args[0] : LSM303DLHC_ACCEL_100HZ;
#elif defined(SHIMMER3R)
    storedConfig->wrAccelRate
        = (args[0] <= LIS2DW12_XL_ODR_1k6Hz) ? args[0] : LIS2DW12_XL_ODR_100Hz;
#endif
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE0, SDH_CONFIG_SETUP_BYTE0, 1);
    break;
  case SET_MAG_GAIN_COMMAND:
#if defined(SHIMMER3)
    storedConfig->magRange = (args[0] <= LSM303DLHC_MAG_8_1G) ? args[0] : LSM303DLHC_MAG_1_3G;
#elif defined(SHIMMER3R)
    storedConfig->magRange = (args[0] <= LIS3MDL_16_GAUSS) ? args[0] : LIS3MDL_4_GAUSS;
#endif
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE2, SDH_CONFIG_SETUP_BYTE2, 1);
    break;
  case SET_MAG_SAMPLING_RATE_COMMAND:
    ShimConfig_configByteMagRateSet(storedConfig, args[0]);
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE2, SDH_CONFIG_SETUP_BYTE2, 1);
    break;
  case SET_WR_ACCEL_LPMODE_COMMAND:
    ShimConfig_wrAccelLpModeSet(storedConfig, args[0] == 1 ? 1 : 0);
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE0, SDH_CONFIG_SETUP_BYTE0, 1);
    break;
  case SET_WR_ACCEL_HRMODE_COMMAND:
    storedConfig->wrAccelHrMode = (args[0] == 1) ? 1 : 0;
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE0, SDH_CONFIG_SETUP_BYTE0, 1);
    break;
  case SET_GYRO_RANGE_COMMAND:
    ShimConfig_gyroRangeSet(storedConfig, args[0]);
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE2, SDH_CONFIG_SETUP_BYTE2, 1);
    break;
  case SET_GYRO_SAMPLING_RATE_COMMAND:
    ShimConfig_gyroRateSet(storedConfig, args[0]);
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE1, SDH_CONFIG_SETUP_BYTE1, 1);
    break;
  case SET_ALT_ACCEL_RANGE_COMMAND:
#if defined(SHIMMER3)
    storedConfig->altAccelRange = (args[0] <= ACCEL_16G) ? (args[0] & 0x03) : ACCEL_2G;
#elif defined(SHIMMER3R)
    storedConfig->lnAccelRange = (args[0] <= LSM6DSV_16g) ? (args[0] & 0x03) : LSM6DSV_2g;
#endif
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE3, SDH_CONFIG_SETUP_BYTE3, 1);
    break;
  case SET_PRESSURE_OVERSAMPLING_RATIO_COMMAND:
    ShimConfig_configBytePressureOversamplingRatioSet(storedConfig, args[0]);
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE3, SDH_CONFIG_SETUP_BYTE3, 1);
    break;
  case SET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
    storedConfig->expansionBoardPower = (args[0] == 1) ? 1 : 0;
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE3, SDH_CONFIG_SETUP_BYTE3, 1);
    break;
  case SET_CONFIG_SETUP_BYTES_COMMAND:
    ShimConfig_storedConfigSet(&args[0], NV_CONFIG_SETUP_BYTE0, 4);
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE0, SDH_CONFIG_SETUP_BYTE0, 4);
    break;
  case SET_SAMPLING_RATE_COMMAND:
    storedConfig->samplingRateTicks = *(uint16_t *) args;
    //ShimConfig_storedConfigSet(&args[0], NV_SAMPLING_RATE, 2);
    ShimBt_settingChangeCommon(NV_SAMPLING_RATE, SDH_SAMPLE_RATE_0, 2);
    break;
  case GET_CALIB_DUMP_COMMAND:
    //usage:
    //0x98, offset, offset, length
    calibRamLength = args[0];
    calibRamOffset = args[1] + (args[2] << 8);
    getCmdWaitingResponse = gAction;
    break;
  case SET_CALIB_DUMP_COMMAND:
    //usage:
    //0x98, offset, offset, length, data[0:127]
    //max length of this command = 132
    calibRamLength = args[0];
    calibRamOffset = args[1] + (args[2] << 8);
    if (ShimCalib_ramWrite(&args[3], calibRamLength, calibRamOffset) == 1)
    {
      ShimCalib_syncFromDumpRamAll();
      update_calib_dump_file = 1;
    }
    break;
  case UPD_CALIB_DUMP_COMMAND:
    ShimCalib_syncFromDumpRamAll();
    update_calib_dump_file = 1;
    break;
  case UPD_SDLOG_CFG_COMMAND:
    ShimTask_set(TASK_SDLOG_CFG_UPDATE);
    break;
    //case UPD_FLASH_COMMAND:
    //   InfoMem_update();
    //   //ShimmerCalibSyncFromDumpRamAll();
    //   //update_calib_dump_file = 1;
    //   break;
  case SET_LN_ACCEL_CALIBRATION_COMMAND:
#if defined(SHIMMER3)
    sensorCalibId = SC_SENSOR_ANALOG_ACCEL;
#elif defined(SHIMMER3R)
    sensorCalibId = SC_SENSOR_LSM6DSV_ACCEL;
#endif
    ShimBt_calibrationChangeCommon(NV_LN_ACCEL_CALIBRATION, SDH_LN_ACCEL_CALIBRATION,
        &storedConfig->lnAccelCalib.rawBytes[0], &args[0], sensorCalibId);
    break;
  case SET_GYRO_CALIBRATION_COMMAND:
#if defined(SHIMMER3)
    sensorCalibId = SC_SENSOR_MPU9X50_ICM20948_GYRO;
#elif defined(SHIMMER3R)
    sensorCalibId = SC_SENSOR_LSM6DSV_GYRO;
#endif
    ShimBt_calibrationChangeCommon(NV_GYRO_CALIBRATION, SDH_GYRO_CALIBRATION,
        &storedConfig->gyroCalib.rawBytes[0], &args[0], sensorCalibId);
    break;
  case SET_MAG_CALIBRATION_COMMAND:
#if defined(SHIMMER3)
    sensorCalibId = SC_SENSOR_LSM303_MAG;
#elif defined(SHIMMER3R)
    sensorCalibId = SC_SENSOR_LIS3MDL_MAG;
#endif
    ShimBt_calibrationChangeCommon(NV_MAG_CALIBRATION, SDH_MAG_CALIBRATION,
        &storedConfig->magCalib.rawBytes[0], &args[0], sensorCalibId);
    break;
  case SET_WR_ACCEL_CALIBRATION_COMMAND:
#if defined(SHIMMER3)
    sensorCalibId = SC_SENSOR_LSM303_ACCEL;
#elif defined(SHIMMER3R)
    sensorCalibId = SC_SENSOR_LIS2DW12_ACCEL;
#endif
    ShimBt_calibrationChangeCommon(NV_WR_ACCEL_CALIBRATION, SDH_WR_ACCEL_CALIBRATION,
        &storedConfig->wrAccelCalib.rawBytes[0], &args[0], sensorCalibId);
    break;
  case SET_GSR_RANGE_COMMAND:
    storedConfig->gsrRange = (args[0] <= 4) ? (args[0] & 0x07) : GSR_AUTORANGE;
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE3, SDH_CONFIG_SETUP_BYTE3, 1);
    break;
  case SET_EXG_REGS_COMMAND:
    if (args[0] < 2 && args[1] < 10 && args[2] < 11)
    {
      exgChip = args[0];
      exgStartAddr = args[1];
      exgLength = args[2];

      uint16_t exgConfigOffset = (exgChip == 0) ? NV_EXG_ADS1292R_1_CONFIG1 :
                                                  NV_EXG_ADS1292R_2_CONFIG1;
      uint16_t exgSdHeadOffset = (exgChip == 0) ? SDH_EXG_ADS1292R_1_CONFIG1 :
                                                  SDH_EXG_ADS1292R_2_CONFIG1;

      ShimConfig_storedConfigSet(&args[3], exgConfigOffset + exgStartAddr, exgLength);

      /* Check if unit is SR47-4 or greater.
       * If so, amend configuration byte 2 of ADS chip 1 to have bit 3 set
       * to 1. This ensures clock lines on ADS chip are correct
       */
      if (exgChip == 0 && (ShimBrd_getDaughtCardId()->exp_brd_id == EXP_BRD_EXG_UNIFIED)
          && (ShimBrd_getDaughtCardId()->exp_brd_major >= 4))
      {
        storedConfig->exgADS1292rRegsCh1.config2 |= 8;
      }

      InfoMem_write(exgConfigOffset + exgStartAddr,
          &storedConfig->rawBytes[exgConfigOffset + exgStartAddr], exgLength);
      ShimSdHead_sdHeadTextSet(&storedConfig->rawBytes[exgConfigOffset], exgSdHeadOffset, exgLength);

      update_sdconfig = 1;
    }
    break;
  case SET_DATA_RATE_TEST:
    /* Stop test before ACK is sent */
    if (args[0] == 0)
    {
      ShimBt_setBtDataRateTestState(0);
      clearBtTxBuf(1);
    }
    getCmdWaitingResponse = gAction;
    break;
  case SET_FACTORY_TEST:
    if (args[0] < FACTORY_TEST_COUNT)
    {
      setup_factory_test(PRINT_TO_BT_UART, (factory_test_t) args[0]);
      ShimTask_set(TASK_FACTORY_TEST);
    }
    break;

  case RESET_TO_DEFAULT_CONFIGURATION_COMMAND:
    ShimConfig_setDefaultConfig();
    ShimSdHead_config2SdHead();
    update_sdconfig = 1;

    //restart sensing to use settings
    if (shimmerStatus.sensing)
    {
      ShimTask_setStopSensing();
      ShimTask_setStartSensing();
    }
    break;
  case RESET_CALIBRATION_VALUE_COMMAND:
    ShimCalib_init();
    ShimCalib_syncFromDumpRamAll();
    update_calib_dump_file = 1;
    break;

  case GET_DAUGHTER_CARD_ID_COMMAND:
    dcMemLength = args[0];
    dcMemOffset = args[1];
    if ((dcMemLength <= 16) && (dcMemOffset <= 15) && (dcMemLength + dcMemOffset <= 16))
    {
      getCmdWaitingResponse = gAction;
    }
    break;
  case SET_DAUGHTER_CARD_ID_COMMAND:
    dcMemLength = args[0];
    dcMemOffset = args[1];
    if ((dcMemLength <= 16) && (dcMemOffset <= 15) && (dcMemLength + dcMemOffset <= 16))
    {
      eepromWrite(dcMemOffset, dcMemLength, &args[2]);
    }
    break;
  case GET_DAUGHTER_CARD_MEM_COMMAND:
    dcMemLength = args[0];
    dcMemOffset = args[1] + (args[2] << 8);
    if ((dcMemLength <= 128) && (dcMemOffset <= 2031) && (dcMemLength + dcMemOffset <= 2032))
    {
      getCmdWaitingResponse = gAction;
    }
    break;
  case SET_DAUGHTER_CARD_MEM_COMMAND:
    dcMemLength = args[0];
    dcMemOffset = args[1] + (args[2] << 8);
    if ((dcMemLength <= 128) && (dcMemOffset <= 2031) && (dcMemLength + dcMemOffset <= 2032))
    {
      eepromWrite(dcMemOffset + 16U, (uint16_t) dcMemLength, &args[3]);
    }
    break;
  case SET_BT_COMMS_BAUD_RATE:
    //TODO changing BAUD rate is not going to be supported
    //if (btArgs[0] != storedConfig->btCommsBaudRate)
    //{
    //  if (args[0] <= BAUD_1000000)
    //  {
    //    changeBtBaudRate = args[0];
    //  }
    //  else
    //  {
    //    changeBtBaudRate = DEFAULT_BT_BAUD_RATE;
    //  }
    //}
    break;
  case SET_DERIVED_CHANNEL_BYTES:
    memcpy(&storedConfig->rawBytes[NV_DERIVED_CHANNELS_0], &args[0], 3);
    memcpy(&storedConfig->rawBytes[NV_DERIVED_CHANNELS_3], &args[3], 5);
    InfoMem_write(NV_DERIVED_CHANNELS_0, &storedConfig->rawBytes[NV_DERIVED_CHANNELS_0], 3);
    InfoMem_write(NV_DERIVED_CHANNELS_3, &storedConfig->rawBytes[NV_DERIVED_CHANNELS_3], 5);
    ShimSdHead_sdHeadTextSet(&storedConfig->rawBytes[NV_DERIVED_CHANNELS_0],
        SDH_DERIVED_CHANNELS_0, 3);
    ShimSdHead_sdHeadTextSet(&storedConfig->rawBytes[NV_DERIVED_CHANNELS_3],
        SDH_DERIVED_CHANNELS_3, 5);
    update_sdconfig = 1;
    break;
  case GET_INFOMEM_COMMAND:
    infomemLength = args[0];
    infomemOffset = args[1] + (args[2] << 8);
    if ((infomemLength <= 128) && (infomemOffset <= (NV_NUM_RWMEM_BYTES - 1))
        && (infomemLength + infomemOffset <= NV_NUM_RWMEM_BYTES))
    {
      getCmdWaitingResponse = gAction;
    }
    break;
  case SET_INFOMEM_COMMAND:
    infomemLength = args[0];
    infomemOffset = args[1] + (args[2] << 8);
    if ((infomemLength <= 128) && (infomemOffset <= (NV_NUM_RWMEM_BYTES - 1))
        && (infomemLength + infomemOffset <= NV_NUM_RWMEM_BYTES))
    {
      ShimConfig_storedConfigSet(&args[3], infomemOffset, infomemLength);

      /* Overwrite MAC ID as read from BT module */
      if (infomemOffset == (INFOMEM_SEG_C_ADDR_MSP430 - INFOMEM_OFFSET_MSP430))
      {
        ShimConfig_storedConfigSet(getMacIdBytesPtr(), NV_MAC_ADDRESS, 6);
      }

      ShimConfig_checkAndCorrectConfig(ShimConfig_getStoredConfig());

      InfoMem_write(infomemOffset, &args[3], infomemLength);
      InfoMem_read(infomemOffset, &storedConfig->rawBytes[infomemOffset], infomemLength);

      /* Save from infomem to calib dump in memory */
      if (infomemOffset == (INFOMEM_SEG_D_ADDR_MSP430 - INFOMEM_OFFSET_MSP430))
      {
#if defined(SHIMMER3)
        ShimCalib_calibSaveFromInfoMemToCalibDump(SC_SENSOR_ANALOG_ACCEL);
        ShimCalib_calibSaveFromInfoMemToCalibDump(SC_SENSOR_MPU9X50_ICM20948_GYRO);
        ShimCalib_calibSaveFromInfoMemToCalibDump(SC_SENSOR_LSM303_MAG);
        ShimCalib_calibSaveFromInfoMemToCalibDump(SC_SENSOR_LSM303_ACCEL);
#elif defined(SHIMMER3R)
        ShimCalib_calibSaveFromInfoMemToCalibDump(SC_SENSOR_LSM6DSV_ACCEL);
        ShimCalib_calibSaveFromInfoMemToCalibDump(SC_SENSOR_LSM6DSV_GYRO);
        ShimCalib_calibSaveFromInfoMemToCalibDump(SC_SENSOR_LIS3MDL_MAG);
        ShimCalib_calibSaveFromInfoMemToCalibDump(SC_SENSOR_LIS2DW12_ACCEL);
#endif
        update_calib_dump_file = 1;
      }
#if defined(SHIMMER3R)
      else if (infomemOffset == (INFOMEM_SEG_C_ADDR_MSP430 - INFOMEM_OFFSET_MSP430))
      {
        ShimCalib_calibSaveFromInfoMemToCalibDump(SC_SENSOR_ADXL371_ACCEL);
        ShimCalib_calibSaveFromInfoMemToCalibDump(SC_SENSOR_LIS2MDL_MAG);
        update_calib_dump_file = 1;
      }
#endif

      ShimSdHead_config2SdHead();
      ShimSd_infomem2Names();
      update_sdconfig = 1;
      if (((infomemOffset >= NV_LN_ACCEL_CALIBRATION) && (infomemOffset <= NV_CALIBRATION_END))
          || (((infomemLength + infomemOffset) >= NV_LN_ACCEL_CALIBRATION)
              && ((infomemLength + infomemOffset) <= NV_CALIBRATION_END))
          || ((infomemOffset <= NV_LN_ACCEL_CALIBRATION)
              && ((infomemLength + infomemOffset) >= NV_CALIBRATION_END)))
      {
        ShimCalib_updateFromInfoAll();
        update_calib_dump_file = 1;
      }
    }
    else
    {
      return;
    }
    break;
  case SET_RWC_COMMAND:
    storedConfig->rtcSetByBt = 1;
    InfoMem_write(NV_SD_TRIAL_CONFIG0, &storedConfig->rawBytes[NV_SD_TRIAL_CONFIG0], 1);
    ShimSdHead_sdHeadTextSetByte(SDH_TRIAL_CONFIG0, storedConfig->rawBytes[NV_SD_TRIAL_CONFIG0]);
#if defined(SHIMMER3)
    setRwcTime(&args[0]);
    RwcCheck();

    uint64_t *rwcTimeDiffPtr = getRwcTimeDiffPtr();
    ShimSdHead_sdHeadTextSetByte(SDH_RTC_DIFF_7, *((uint8_t *) rwcTimeDiffPtr));
    ShimSdHead_sdHeadTextSetByte(SDH_RTC_DIFF_6, *(((uint8_t *) rwcTimeDiffPtr) + 1));
    ShimSdHead_sdHeadTextSetByte(SDH_RTC_DIFF_5, *(((uint8_t *) rwcTimeDiffPtr) + 2));
    ShimSdHead_sdHeadTextSetByte(SDH_RTC_DIFF_4, *(((uint8_t *) rwcTimeDiffPtr) + 3));
    ShimSdHead_sdHeadTextSetByte(SDH_RTC_DIFF_3, *(((uint8_t *) rwcTimeDiffPtr) + 4));
    ShimSdHead_sdHeadTextSetByte(SDH_RTC_DIFF_2, *(((uint8_t *) rwcTimeDiffPtr) + 5));
    ShimSdHead_sdHeadTextSetByte(SDH_RTC_DIFF_1, *(((uint8_t *) rwcTimeDiffPtr) + 6));
    ShimSdHead_sdHeadTextSetByte(SDH_RTC_DIFF_0, *(((uint8_t *) rwcTimeDiffPtr) + 7));
#else
    memcpy((uint8_t *) (&temp64), args, 8); //64bits = 8bytes
    RTC_init(temp64);

    setupNextRtcMinuteAlarm(); //configure RTC alarm after time set from BT.
#endif
    break;

  case SET_ALT_ACCEL_CALIBRATION_COMMAND:
#if defined(SHIMMER3)
    sensorCalibId = SC_SENSOR_MPU9X50_ICM20948_ACCEL;
#elif defined(SHIMMER3R)
    sensorCalibId = SC_SENSOR_ADXL371_ACCEL;
#endif
    ShimBt_calibrationChangeCommon(NV_ALT_ACCEL_CALIBRATION, SDH_ALT_ACCEL_CALIBRATION,
        &storedConfig->altAccelCalib.rawBytes[0], &args[0], sensorCalibId);
    break;
  case SET_ALT_MAG_CALIBRATION_COMMAND:
#if defined(SHIMMER3)
    sensorCalibId = SC_SENSOR_MPU9X50_ICM20948_MAG;
#elif defined(SHIMMER3R)
    sensorCalibId = SC_SENSOR_LIS2MDL_MAG;
#endif
    ShimBt_calibrationChangeCommon(NV_ALT_MAG_CALIBRATION, SDH_ALT_MAG_CALIBRATION,
        &storedConfig->altMagCalib.rawBytes[0], &args[0], sensorCalibId);
    break;
  case SET_ALT_ACCEL_SAMPLING_RATE_COMMAND:
    storedConfig->altAccelRate = args[0] & 0x03;
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE4, SDH_CONFIG_SETUP_BYTE4, 1);
    break;
  case SET_ALT_MAG_SAMPLING_RATE_COMMAND:
    storedConfig->altMagRate = args[0] & 0x03;
    ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE4, SDH_CONFIG_SETUP_BYTE4, 1);
    break;

  case SET_SD_SYNC_COMMAND:
    if (shimmerStatus.btInSyncMode && ShimSdSync_isBtSdSyncRunning())
    {
      /* Reassemble full packet so that original RcNodeR10() will work without modificiation */
      fullSyncResp[0] = gAction;
      memcpy(&fullSyncResp[1], &args[0], SYNC_PACKET_MAX_SIZE - SYNC_PACKET_SIZE_CMD);
      ShimSdSync_syncRespSet(&fullSyncResp[0], SYNC_PACKET_MAX_SIZE);
      ShimTask_set(TASK_RCNODER10);
    }
    else
    {
      sendNack = 1;
    }
  case ACK_COMMAND_PROCESSED:
    if (shimmerStatus.btInSyncMode && ShimSdSync_isBtSdSyncRunning())
    {
      /* Slave response received by Master */
      if (args[0] == SD_SYNC_RESPONSE)
      {
        /* SD Sync Center - get's into this case when the center is waiting for a 0x01 or 0xFF from a node */
        ShimSdSync_syncRespSet(&args[1], 1U);
        ShimTask_set(TASK_RCCENTERR1);
      }
    }
    else
    {
      sendNack = 1;
    }
    break;
  default:
    break;
  }

  /* Send Response back for all commands except when FW has received an ACK */
  /* ACK is sent back as part of SD_SYNC_RESPONSE so no need to send it here */
  if (!(gAction == ACK_COMMAND_PROCESSED || gAction == SET_SD_SYNC_COMMAND) || sendNack)
  {
    if (sendNack == 0)
    {
      /* Send ACK back for all commands except when FW is sending a NACK */
      sendAck = 1;
    }
    ShimTask_set(TASK_BT_RESPOND);
  }

  if (update_sdconfig)
  {
    ShimConfig_setSdCfgFlag(1);
  }
  if (update_calib_dump_file)
  {
    ShimBt_updateCalibDumpFile();
  }
}

void ShimBt_settingChangeCommon(uint16_t configByteIdx, uint16_t sdHeaderIdx, uint16_t len)
{
  gConfigBytes *storedConfig = ShimConfig_getStoredConfig();

  ShimConfig_checkAndCorrectConfig(storedConfig);

  InfoMem_write(configByteIdx, &storedConfig->rawBytes[configByteIdx], len);
  ShimSdHead_sdHeadTextSet(&storedConfig->rawBytes[configByteIdx], sdHeaderIdx, len);
  //TODO don't think below is needed because we're specifically changing what's
  //new above ShimSdHead_config2SdHead();

  //restart sensing to use settings
  if (shimmerStatus.sensing)
  {
    ShimTask_setStopSensing();
    ShimTask_setStartSensing();
  }

  //update sdconfig
  ShimConfig_setSdCfgFlag(1);
}

void ShimBt_calibrationChangeCommon(uint16_t configByteIdx,
    uint16_t sdHeaderIdx,
    uint8_t *configBytePtr,
    uint8_t *newCalibPtr,
    uint8_t sensorCalibId)
{
  memcpy(configBytePtr, newCalibPtr, SC_DATA_LEN_STD_IMU_CALIB);
  InfoMem_write(configByteIdx, configBytePtr, SC_DATA_LEN_STD_IMU_CALIB);
  ShimSdHead_sdHeadTextSet(configBytePtr, sdHeaderIdx, SC_DATA_LEN_STD_IMU_CALIB);

  ShimCalib_calibSaveFromInfoMemToCalibDump(sensorCalibId);

  ShimBt_updateCalibDumpFile();
}

void ShimBt_updateCalibDumpFile(void)
{
  if (CheckSdInslot() && !shimmerStatus.sdBadFile)
  {
    if (!shimmerStatus.docked)
    {
      ShimCalib_ram2File();
    }
    else
    {
      ShimConfig_setRamCalibFlag(1);
    }
  }
}

uint8_t ShimBt_replySingleSensorCalibCmd(uint8_t cmdWaitingResponse, uint8_t *resPacketPtr)
{
  sc_t sc1;
  gConfigBytes *storedConfig = ShimConfig_getStoredConfig();

  if (cmdWaitingResponse == GET_LN_ACCEL_CALIBRATION_COMMAND)
  {
#if defined(SHIMMER3)
    sc1.id = SC_SENSOR_ANALOG_ACCEL;
    sc1.range = SC_SENSOR_RANGE_ANALOG_ACCEL;
#elif defined(SHIMMER3R)
    sc1.id = SC_SENSOR_LSM6DSV_ACCEL;
    sc1.range = storedConfig->lnAccelRange;
#endif
  }
  else if (cmdWaitingResponse == GET_GYRO_CALIBRATION_COMMAND)
  {
#if defined(SHIMMER3)
    sc1.id = SC_SENSOR_MPU9X50_ICM20948_GYRO;
#elif defined(SHIMMER3R)
    sc1.id = SC_SENSOR_LSM6DSV_GYRO;
#endif
    sc1.range = ShimConfig_gyroRangeGet();
  }
  else if (cmdWaitingResponse == GET_MAG_CALIBRATION_COMMAND)
  {
#if defined(SHIMMER3)
    sc1.id = SC_SENSOR_LSM303_MAG;
#elif defined(SHIMMER3R)
    sc1.id = SC_SENSOR_LIS3MDL_MAG;
#endif
    sc1.range = storedConfig->magRange;
  }
  else if (cmdWaitingResponse == GET_WR_ACCEL_CALIBRATION_COMMAND)
  {
#if defined(SHIMMER3)
    sc1.id = SC_SENSOR_LSM303_ACCEL;
#elif defined(SHIMMER3R)
    sc1.id = SC_SENSOR_LIS2DW12_ACCEL;
#endif
    sc1.range = storedConfig->wrAccelRange;
  }
  else if (cmdWaitingResponse == GET_ALT_ACCEL_CALIBRATION_COMMAND)
  {
#if defined(SHIMMER3)
    sc1.id = SC_SENSOR_MPU9X50_ICM20948_ACCEL;
    sc1.range = storedConfig->altAccelRange;
#elif defined(SHIMMER3R)
    sc1.id = SC_SENSOR_ADXL371_ACCEL;
    sc1.range = SC_SENSOR_RANGE_ADXL371_RANGE;
#endif
  }
  else if (cmdWaitingResponse == GET_ALT_MAG_CALIBRATION_COMMAND)
  {
#if defined(SHIMMER3)
    sc1.id = SC_SENSOR_MPU9X50_ICM20948_MAG;
#elif defined(SHIMMER3R)
    sc1.id = SC_SENSOR_LIS2MDL_MAG;
    sc1.range = SC_SENSOR_RANGE_LIS2MDL_RANGE;
#endif
  }
  else
  {
    return 0;
  }

  sc1.data_len = SC_DATA_LEN_STD_IMU_CALIB;
  ShimCalib_singleSensorRead(&sc1);

  memcpy(resPacketPtr, sc1.data.raw, sc1.data_len);
  return sc1.data_len;
}

void ShimBt_sendRsp(void)
{
  uint16_t packet_length = 0;
  //STATTypeDef * stat = GetStatus();
  uint8_t resPacket[RESPONSE_PACKET_SIZE + 2]; //+2 for CRC
  packet_length = 0;

  uint64_t temp_rtcCurrentTime = 0;
  uint8_t *fileNamePtr;
  uint8_t bmpCalibByteLen;
  uint8_t btVerStrLen;

  gConfigBytes *storedConfig = ShimConfig_getStoredConfig();

  if (shimmerStatus.btConnected)
  {
    if (sendAck)
    {
      *(resPacket + packet_length++) = ACK_COMMAND_PROCESSED;
      sendAck = 0;
    }
    if (sendNack)
    {
      *(resPacket + packet_length++) = NACK_COMMAND_PROCESSED;
      sendNack = 0;
    }

    switch (getCmdWaitingResponse)
    {
    case 0:
      break;
    case INQUIRY_COMMAND:
      /* Channel order/packet structure need to be assembled before sending the inquiry response so that the information is correct. */
      ShimSens_configureChannels();

      *(resPacket + packet_length++) = INQUIRY_RESPONSE;
      *(uint16_t *) (resPacket + packet_length) = storedConfig->samplingRateTicks; //ADC sampling rate
      packet_length += 2;

      *(resPacket + packet_length++) = storedConfig->rawBytes[NV_CONFIG_SETUP_BYTE0];
      *(resPacket + packet_length++) = storedConfig->rawBytes[NV_CONFIG_SETUP_BYTE1];
      *(resPacket + packet_length++) = storedConfig->rawBytes[NV_CONFIG_SETUP_BYTE2];
      *(resPacket + packet_length++) = storedConfig->rawBytes[NV_CONFIG_SETUP_BYTE3];
#if defined(SHIMMER3R)
      *(resPacket + packet_length++) = storedConfig->rawBytes[NV_CONFIG_SETUP_BYTE4];
      *(resPacket + packet_length++) = storedConfig->rawBytes[NV_CONFIG_SETUP_BYTE5];
      *(resPacket + packet_length++) = storedConfig->rawBytes[NV_CONFIG_SETUP_BYTE6];
#endif

      *(resPacket + packet_length++) = sensing.nbrAdcChans + sensing.nbrDigiChans; //number of data channels
      *(resPacket + packet_length++) = storedConfig->bufferSize; //buffer size
      memcpy((resPacket + packet_length), sensing.cc,
          (sensing.nbrAdcChans + sensing.nbrDigiChans));
      packet_length += sensing.nbrAdcChans + sensing.nbrDigiChans;
      break;
    case GET_SAMPLING_RATE_COMMAND:
      *(resPacket + packet_length++) = SAMPLING_RATE_RESPONSE;
      *(uint16_t *) (resPacket + packet_length) = storedConfig->samplingRateTicks; //ADC sampling rate
      packet_length += 2;
      break;
    case GET_WR_ACCEL_RANGE_COMMAND:
      *(resPacket + packet_length++) = WR_ACCEL_RANGE_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->wrAccelRange;
      break;
    case GET_MAG_GAIN_COMMAND:
      *(resPacket + packet_length++) = MAG_GAIN_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->magRange;
      break;
    case GET_MAG_SAMPLING_RATE_COMMAND:
      *(resPacket + packet_length++) = MAG_SAMPLING_RATE_RESPONSE;
      *(resPacket + packet_length++) = ShimConfig_configByteMagRateGet();
      break;
    case GET_STATUS_COMMAND:
      *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
      *(resPacket + packet_length++) = STATUS_RESPONSE;
      *(resPacket + packet_length++) = (shimmerStatus.toggleLedRedCmd << 7)
          + (shimmerStatus.sdBadFile << 6) + (shimmerStatus.sdInserted << 5)
          + (shimmerStatus.btStreaming << 4) + (shimmerStatus.sdLogging << 3)
          + (isRwcTimeSet() << 2) + (shimmerStatus.sensing << 1)
          + (shimmerStatus.docked);
      break;
    case GET_VBATT_COMMAND:
      manageReadBatt(1);
      *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
      *(resPacket + packet_length++) = VBATT_RESPONSE;
      uint8_t i = 0;
      for (i = 0; i < 3; i++)
      {
        resPacket[packet_length] = batteryStatus.battStatusRaw.rawBytes[i];
        packet_length++;
      }
      break;
    case GET_TRIAL_CONFIG_COMMAND:
      *(resPacket + packet_length++) = TRIAL_CONFIG_RESPONSE;
      //2 trial config bytes + 1 interval byte
      ShimConfig_storedConfigGet(&resPacket[packet_length], NV_SD_TRIAL_CONFIG0, 3);
      packet_length += 3;
      break;
    case GET_CENTER_COMMAND:
      *(resPacket + packet_length++) = CENTER_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->masterEnable;
      break;
    case GET_SHIMMERNAME_COMMAND:
      ShimSd_setShimmerName();
      uint8_t shimmer_name_len = strlen(ShimConfig_getStoredConfig()->shimmerName);
      *(resPacket + packet_length++) = SHIMMERNAME_RESPONSE;
      *(resPacket + packet_length++) = shimmer_name_len;
      memcpy((resPacket + packet_length), &storedConfig->shimmerName[0], shimmer_name_len);
      packet_length += shimmer_name_len;
      break;
    case GET_EXPID_COMMAND:
      ShimSd_setExpIdName();
      uint8_t exp_id_name_len = strlen((char *) storedConfig->expIdName);
      *(resPacket + packet_length++) = EXPID_RESPONSE;
      *(resPacket + packet_length++) = exp_id_name_len;
      memcpy((resPacket + packet_length), &storedConfig->expIdName[0], exp_id_name_len);
      packet_length += exp_id_name_len;
      break;
    case GET_CONFIGTIME_COMMAND:
      ShimSd_setCfgTime();
      uint8_t *configTimeTextPtr = ShimSd_configTimeTextPtrGet();
      uint8_t cfgtime_name_len = strlen((char *) configTimeTextPtr);
      *(resPacket + packet_length++) = CONFIGTIME_RESPONSE;
      *(resPacket + packet_length++) = cfgtime_name_len;
      memcpy((resPacket + packet_length), configTimeTextPtr, cfgtime_name_len);
      packet_length += cfgtime_name_len;
      break;
    case GET_DIR_COMMAND:
      fileNamePtr = ShimSd_fileNamePtrGet();
      uint8_t dir_len = strlen((char *) fileNamePtr) - 3;
      *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
      *(resPacket + packet_length++) = DIR_RESPONSE;
      *(resPacket + packet_length++) = dir_len;
      memcpy((resPacket + packet_length), fileNamePtr, dir_len);
      packet_length += dir_len;
      break;
    case GET_NSHIMMER_COMMAND:
      *(resPacket + packet_length++) = NSHIMMER_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->numberOfShimmers;
      break;
    case GET_MYID_COMMAND:
      *(resPacket + packet_length++) = MYID_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->myTrialID;
      break;
    case GET_WR_ACCEL_SAMPLING_RATE_COMMAND:
      *(resPacket + packet_length++) = WR_ACCEL_SAMPLING_RATE_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->wrAccelRate;
      break;
    case GET_WR_ACCEL_LPMODE_COMMAND:
      *(resPacket + packet_length++) = WR_ACCEL_LPMODE_RESPONSE;
      *(resPacket + packet_length++) = ShimConfig_wrAccelLpModeGet();
      break;
    case GET_WR_ACCEL_HRMODE_COMMAND:
      *(resPacket + packet_length++) = WR_ACCEL_HRMODE_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->wrAccelHrMode;
      break;
    case GET_GYRO_RANGE_COMMAND:
      *(resPacket + packet_length++) = GYRO_RANGE_RESPONSE;
      *(resPacket + packet_length++) = ShimConfig_gyroRangeGet();
      break;
    case GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND:
      *(resPacket + packet_length++) = BMP180_CALIBRATION_COEFFICIENTS_RESPONSE;
      if (isBmp180InUse())
      {
        memcpy(resPacket + packet_length, get_bmp_calib_data_bytes(), BMP180_CALIB_DATA_SIZE);
      }
      else
      {
        //Dummy bytes sent if incorrect calibration bytes requested.
        memset(resPacket + packet_length, 0x01, BMP180_CALIB_DATA_SIZE);
      }
      packet_length += BMP180_CALIB_DATA_SIZE;
      break;
    case GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND:
      *(resPacket + packet_length++) = BMP280_CALIBRATION_COEFFICIENTS_RESPONSE;
      if (isBmp280InUse())
      {
        memcpy(resPacket + packet_length, get_bmp_calib_data_bytes(), BMP280_CALIB_DATA_SIZE);
      }
      else
      {
        //Dummy bytes sent if incorrect calibration bytes requested.
        memset(resPacket + packet_length, 0x01, BMP280_CALIB_DATA_SIZE);
      }
      packet_length += BMP280_CALIB_DATA_SIZE;
      break;
    case GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND:
      bmpCalibByteLen = get_bmp_calib_data_bytes_len();
      *(resPacket + packet_length++) = PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE;
      *(resPacket + packet_length++) = 1U + bmpCalibByteLen;
      if (isBmp180InUse())
      {
        *(resPacket + packet_length++) = PRESSURE_SENSOR_BMP180;
      }
      else if (isBmp280InUse())
      {
        *(resPacket + packet_length++) = PRESSURE_SENSOR_BMP280;
      }
      else
      {
        *(resPacket + packet_length++) = PRESSURE_SENSOR_BMP390;
      }
      memcpy(resPacket + packet_length, get_bmp_calib_data_bytes(), bmpCalibByteLen);
      packet_length += bmpCalibByteLen;
      break;
    case GET_GYRO_SAMPLING_RATE_COMMAND:
      *(resPacket + packet_length++) = GYRO_SAMPLING_RATE_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->gyroRate;
      break;
    case GET_ALT_ACCEL_RANGE_COMMAND:
#if defined(SHIMMER3)
      *(resPacket + packet_length++) = ALT_ACCEL_RANGE_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->altAccelRange;
#elif defined(SHIMMER3R)
      *(resPacket + packet_length++) = ALT_ACCEL_RANGE_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->lnAccelRange;
#endif
      break;
    case GET_PRESSURE_OVERSAMPLING_RATIO_COMMAND:
      *(resPacket + packet_length++) = PRESSURE_OVERSAMPLING_RATIO_RESPONSE;
      *(resPacket + packet_length++)
          = ShimConfig_configBytePressureOversamplingRatioGet();
      break;
    case GET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
      *(resPacket + packet_length++) = INTERNAL_EXP_POWER_ENABLE_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->expansionBoardPower;
      break;
    case GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND:
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
      //Mag sensitivity adj feature is not present in ICM-20948
      if (ShimBrd_isGyroInUseMpu9x50())
      {
        MPU9150_init();
        MPU9150_wake(1);
        MPU9150_wake(0);
        *(resPacket + packet_length++) = MPU9150_MAG_SENS_ADJ_VALS_RESPONSE;
        MPU9150_getMagSensitivityAdj(resPacket + packet_length);
        packet_length += 3;
      }
      else
      {
        *(resPacket + packet_length++) = ACK_COMMAND_PROCESSED;
      }
#else
      *(resPacket + packet_length++) = ACK_COMMAND_PROCESSED;
#endif
      break;
    case GET_CONFIG_SETUP_BYTES_COMMAND:
      *(resPacket + packet_length++) = CONFIG_SETUP_BYTES_RESPONSE;
      memcpy(resPacket + packet_length, &storedConfig->rawBytes[NV_CONFIG_SETUP_BYTE0], 4);
      packet_length += 4;
      break;
    case GET_BT_VERSION_STR_COMMAND:
      btVerStrLen = ShimBt_getBtVerStrLen();
      *(resPacket + packet_length++) = BT_VERSION_STR_RESPONSE;
      *(resPacket + packet_length++) = btVerStrLen;
      memcpy((resPacket + packet_length), ShimBt_getBtVerStrPtr(), btVerStrLen);
      packet_length += btVerStrLen;
      break;
    case GET_GSR_RANGE_COMMAND:
      *(resPacket + packet_length++) = GSR_RANGE_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->gsrRange;
      break;

    case DEPRECATED_GET_DEVICE_VERSION_COMMAND:
    case GET_DEVICE_VERSION_COMMAND:
      *(resPacket + packet_length++) = DEVICE_VERSION_RESPONSE;
      *(resPacket + packet_length++) = DEVICE_VER;
      break;

    case GET_FW_VERSION_COMMAND:
      *(resPacket + packet_length++) = FW_VERSION_RESPONSE;
      *(resPacket + packet_length++) = FW_IDENTIFIER & 0xFF;
      *(resPacket + packet_length++) = (FW_IDENTIFIER & 0xFF00) >> 8;
      *(resPacket + packet_length++) = FW_VER_MAJOR & 0xFF;
      *(resPacket + packet_length++) = (FW_VER_MAJOR & 0xFF00) >> 8;
      *(resPacket + packet_length++) = FW_VER_MINOR;
      *(resPacket + packet_length++) = FW_VER_REL;
      break;
    case GET_CHARGE_STATUS_LED_COMMAND:
      *(resPacket + packet_length++) = CHARGE_STATUS_LED_RESPONSE;
      *(resPacket + packet_length++) = batteryStatus.battStat;
      break;
    case GET_BUFFER_SIZE_COMMAND:
      *(resPacket + packet_length++) = BUFFER_SIZE_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->bufferSize;
      break;
    case GET_UNIQUE_SERIAL_COMMAND:
      *(resPacket + packet_length++) = UNIQUE_SERIAL_RESPONSE;
#if defined(SHIMMER3)
      memcpy((resPacket + packet_length), HAL_GetUID(), 8);
      packet_length += 8;
#elif defined(SHIMMER3R)
      uint32_t uid[3];
      uid[0] = HAL_GetUIDw0();
      uid[1] = HAL_GetUIDw1();
      uid[2] = HAL_GetUIDw2();
      memcpy((resPacket + packet_length), (uint8_t *) &uid[0], 12);
      packet_length += 12;
#endif
      break;
    case GET_BT_COMMS_BAUD_RATE:
      *(resPacket + packet_length++) = BT_COMMS_BAUD_RATE_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->btCommsBaudRate;
      break;
    case GET_DERIVED_CHANNEL_BYTES:
      *(resPacket + packet_length++) = DERIVED_CHANNEL_BYTES_RESPONSE;
      ShimConfig_storedConfigGet(&resPacket[packet_length], NV_DERIVED_CHANNELS_0, 3);
      packet_length += 3;
      ShimConfig_storedConfigGet(&resPacket[packet_length], NV_DERIVED_CHANNELS_3, 5);
      packet_length += 5;
      break;
    case GET_RWC_COMMAND:
      temp_rtcCurrentTime = RTC_get64();
      *(resPacket + packet_length++) = RWC_RESPONSE;
      memcpy(resPacket + packet_length, (uint8_t *) (&temp_rtcCurrentTime), 8);
      packet_length += 8;
      break;

    case GET_ALL_CALIBRATION_COMMAND:
      *(resPacket + packet_length++) = ALL_CALIBRATION_RESPONSE;

      packet_length += ShimBt_replySingleSensorCalibCmd(
          GET_LN_ACCEL_CALIBRATION_COMMAND, &resPacket[packet_length]);

      packet_length += ShimBt_replySingleSensorCalibCmd(
          GET_GYRO_CALIBRATION_COMMAND, &resPacket[packet_length]);

      packet_length += ShimBt_replySingleSensorCalibCmd(
          GET_MAG_CALIBRATION_COMMAND, &resPacket[packet_length]);

      packet_length += ShimBt_replySingleSensorCalibCmd(
          GET_WR_ACCEL_CALIBRATION_COMMAND, &resPacket[packet_length]);

#if defined(SHIMMER3R)
      packet_length += ShimBt_replySingleSensorCalibCmd(
          GET_ALT_ACCEL_CALIBRATION_COMMAND, &resPacket[packet_length]);

      packet_length += ShimBt_replySingleSensorCalibCmd(
          GET_ALT_MAG_CALIBRATION_COMMAND, &resPacket[packet_length]);
#endif
      break;

    case GET_LN_ACCEL_CALIBRATION_COMMAND:
    case GET_GYRO_CALIBRATION_COMMAND:
    case GET_MAG_CALIBRATION_COMMAND:
    case GET_WR_ACCEL_CALIBRATION_COMMAND:
    case GET_ALT_ACCEL_CALIBRATION_COMMAND:
    case GET_ALT_MAG_CALIBRATION_COMMAND:
      *(resPacket + packet_length++) = ShimBt_getExpectedRspForGetCmd(getCmdWaitingResponse);
      packet_length += ShimBt_replySingleSensorCalibCmd(
          getCmdWaitingResponse, &resPacket[packet_length]);
      break;
    case GET_ALT_ACCEL_SAMPLING_RATE_COMMAND:
      *(resPacket + packet_length++) = ALT_ACCEL_SAMPLING_RATE_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->altAccelRate;
      break;
    case GET_ALT_MAG_SAMPLING_RATE_COMMAND:
      *(resPacket + packet_length++) = ALT_MAG_SAMPLING_RATE_RESPONSE;
      *(resPacket + packet_length++) = storedConfig->altMagRate;
      break;

    case GET_EXG_REGS_COMMAND:
      *(resPacket + packet_length++) = EXG_REGS_RESPONSE;
      *(resPacket + packet_length++) = exgLength;
      if (exgLength)
      {
        if (exgChip)
        {
          memcpy((resPacket + packet_length),
              &storedConfig->exgADS1292rRegsCh2.rawBytes[exgStartAddr], exgLength);
        }
        else
        {
          memcpy((resPacket + packet_length),
              &storedConfig->exgADS1292rRegsCh1.rawBytes[exgStartAddr], exgLength);
        }
        packet_length += exgLength;
      }
      break;
    case GET_CALIB_DUMP_COMMAND:
      *(resPacket + packet_length++) = RSP_CALIB_DUMP_COMMAND;
      *(resPacket + packet_length++) = calibRamLength;
      *(resPacket + packet_length++) = calibRamOffset & 0xff;
      *(resPacket + packet_length++) = (calibRamOffset >> 8) & 0xff;
      ShimCalib_ramRead(resPacket + packet_length, calibRamLength, calibRamOffset);
      packet_length += calibRamLength;
      break;
    case SET_DATA_RATE_TEST:
      /* Start test after ACK is sent - this will be handled by the
       * interrupt after ACK byte is transmitted */
      if (args[0] != 0)
      {
        ShimBt_setBtDataRateTestState(1);
      }
      break;

    case GET_DAUGHTER_CARD_ID_COMMAND:
      *(resPacket + packet_length++) = DAUGHTER_CARD_ID_RESPONSE;
      *(resPacket + packet_length++) = dcMemLength;
      eepromRead(dcMemOffset, dcMemLength, resPacket + packet_length);
      packet_length += dcMemLength;
      break;
    case GET_DAUGHTER_CARD_MEM_COMMAND:
      *(resPacket + packet_length++) = DAUGHTER_CARD_MEM_RESPONSE;
      *(resPacket + packet_length++) = dcMemLength;
      if (!shimmerStatus.sensing)
      {
        eepromRead(dcMemOffset + 16U, dcMemLength, resPacket + packet_length);
      }
      else
      {
        memset(resPacket + packet_length, 0xff, dcMemLength);
      }
      packet_length += dcMemLength;
      break;
    case GET_INFOMEM_COMMAND:
      *(resPacket + packet_length++) = INFOMEM_RESPONSE;
      *(resPacket + packet_length++) = infomemLength;
      ShimConfig_storedConfigGet(&resPacket[packet_length], infomemOffset, infomemLength);
      packet_length += infomemLength;
      break;

#if defined(SHIMMER4_SDK)
    case GET_I2C_BATT_STATUS_COMMAND:
      *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
      *(resPacket + packet_length++) = RSP_I2C_BATT_STATUS_COMMAND;
      memcpy((resPacket + packet_length), (uint8_t *) shimmerStatus.battDigital, STC3100_DATA_LEN);
      packet_length += STC3100_DATA_LEN;
      break
#endif

          default: break;
    }
    getCmdWaitingResponse = 0;

    uint8_t crcMode = ShimBt_getCrcMode();
    if (crcMode != CRC_OFF)
    {
      calculateCrcAndInsert(crcMode, resPacket, packet_length);
      packet_length += crcMode;
    }
#if defined(SHIMMER3)
    BT_write(resPacket, packet_length, SHIMMER_CMD);
#elif defined(SHIMMER3R)
    BT_write(resPacket, packet_length);
#endif
  }
}

uint8_t ShimBt_getExpectedRspForGetCmd(uint8_t getCmd)
{
  switch (getCmd)
  {
  case GET_LN_ACCEL_CALIBRATION_COMMAND:
    return LN_ACCEL_CALIBRATION_RESPONSE;
  case GET_GYRO_CALIBRATION_COMMAND:
    return GYRO_CALIBRATION_RESPONSE;
  case GET_MAG_CALIBRATION_COMMAND:
    return MAG_CALIBRATION_RESPONSE;
  case GET_WR_ACCEL_CALIBRATION_COMMAND:
    return WR_ACCEL_CALIBRATION_RESPONSE;
  case GET_ALT_ACCEL_CALIBRATION_COMMAND:
    return ALT_ACCEL_CALIBRATION_RESPONSE;
  case GET_ALT_MAG_CALIBRATION_COMMAND:
    return ALT_MAG_CALIBRATION_RESPONSE;
  default:
    return 0;
  }
}

#if defined(SHIMMER3R)
void ShimBt_setDmaWaitingForResponse(uint16_t count)
{
  btRxWaitByteCount = count;
}

uint16_t ShimBt_getBtRxShimmerCommsWaitByteCount(void)
{
  return btRxWaitByteCount;
}
#endif

void ShimBt_setCrcMode(COMMS_CRC_MODE btCrcModeNew)
{
  btCrcMode = btCrcModeNew;
#if defined(SHIMMER3R)
  //TODO turn on/off peripheral when needed to save power
  //if (btCrcMode == CRC_OFF)
  //{
  //  HAL_CRC_DeInit(hcrc);
  //}
  //else
  //{
  //  MX_CRC_Init();
  //}
#endif
}

COMMS_CRC_MODE ShimBt_getCrcMode(void)
{
  return btCrcMode;
}

void ShimBt_setBtDataRateTestState(uint8_t state)
{
  btDataRateTestState = state;
  btDataRateTestCounter = 0;

  BT_setSendNextChar_cb(btDataRateTestState == 1 ? ShimBt_loadBtTxBufForDataRateTest : 0);
}

void ShimBt_loadBtTxBufForDataRateTest(void)
{
  uint16_t spaceInTxBuf = getSpaceInBtTxBuf();
  if (spaceInTxBuf > DATA_RATE_TEST_PACKET_SIZE)
  {
    pushByteToBtTxBuf(DATA_RATE_TEST_RESPONSE);
    pushBytesToBtTxBuf((uint8_t *) &btDataRateTestCounter, sizeof(btDataRateTestCounter));
    btDataRateTestCounter++;
  }
}

uint8_t ShimBt_getMacAddressAscii(char *macAscii)
{
#if defined(SHIMMER3)
  memcpy(macAscii, getMacIdStrPtr(), 12);
  return 0;
#elif defined(SHIMMER3R)
  //MAC is stored as 6 byte array in CYW20820 library
  uint8_t *macAddrPtr = BT_getCyw20820MacAddressPtr();
  (void) sprintf(macAscii, "%02X%02X%02X%02X%02X%02X", macAddrPtr[5],
      macAddrPtr[4], macAddrPtr[3], macAddrPtr[2], macAddrPtr[1], macAddrPtr[0]);
  return 1;
#elif defined(SHIMMER4_SDK)
  if (BT_getRn42MacAddressPtr(macAddrPtr))
  {
    memcpy(macAscii, macAddrPtr, 12);
    return 0;
  }
  else
  {
    return 1;
  }
#endif
}

uint8_t ShimBt_getMacAddressHex(uint8_t *macHex)
{
#if defined(SHIMMER3R)
  uint8_t *ptr = BT_getCyw20820MacAddressPtr();
  macHex[0] = *(ptr + 5);
  macHex[1] = *(ptr + 4);
  macHex[2] = *(ptr + 3);
  macHex[3] = *(ptr + 2);
  macHex[4] = *(ptr + 1);
  macHex[5] = *(ptr + 0);
  return 1;
#elif defined(SHIMMER3)
  uint8_t i, pchar[3];
  uint8_t *macAddrPtr = getMacIdBytesPtr();
  if (*macAddrPtr)
  {
    pchar[2] = 0;
    for (i = 0; i < 6; i++)
    {
      pchar[0] = macAddrPtr[i * 2];
      pchar[1] = macAddrPtr[i * 2 + 1];
      macHex[i] = strtoul((char *) pchar, 0, 16);
    }
    return 0;
  }
  else
  {
    return 1;
  }
#endif
}

void ShimBt_btsdSelfcmd(void)
{
  if (shimmerStatus.btConnected)
  {
    uint8_t i = 0;
    uint8_t selfcmd[6]; /* max is 6 bytes */

    if (useAckPrefixForInstreamResponses)
    {
      selfcmd[i++] = ACK_COMMAND_PROCESSED;
    }
    selfcmd[i++] = INSTREAM_CMD_RESPONSE;
    selfcmd[i++] = STATUS_RESPONSE;
    selfcmd[i++] = (shimmerStatus.toggleLedRedCmd << 7)
        | (shimmerStatus.sdBadFile << 6) | (shimmerStatus.sdInserted << 5)
        | (shimmerStatus.btStreaming << 4) | (shimmerStatus.sdLogging << 3)
        | (isRwcTimeSet() << 2) | (shimmerStatus.sensing << 1) | shimmerStatus.docked;

    uint8_t crcMode = ShimBt_getCrcMode();
    if (crcMode != CRC_OFF)
    {
      calculateCrcAndInsert(crcMode, &selfcmd[0], i);
      i += crcMode; //Ordinal of enum is how many bytes are used
    }

#if defined(SHIMMER3)
    BT_write(selfcmd, i, SHIMMER_CMD);
#elif defined(SHIMMER3R)
    BT_write(selfcmd, i);
#endif
  }
}

void ShimBt_handleBtRfCommStateChange(uint8_t isConnected)
{
#if defined(SHIMMER3)
  shimmerStatus.btConnected = isConnected;
  BT_rst_MessageProgress();

  updateBtConnectionStatusInterruptDirection();
#endif

  if (isConnected)
  { //BT is connected
    ShimBt_resetBtRxVariablesOnConnect();

    if (shimmerStatus.sdSyncEnabled)
    {
      shimmerStatus.btstreamReady = 0;
    }
    else
    {
      shimmerStatus.btstreamReady = 1;

#if defined(SHIMMER3)
      setDmaWaitingForResponseIfStatusStrDisabled();
#endif
    }

    if (ShimSdHead_sdHeadTextGetByte(SDH_TRIAL_CONFIG0) & SDH_IAMMASTER)
    {
      //center sends sync packet and is waiting for response
      if (ShimSdSync_isBtSdSyncRunning())
      {
#if defined(SHIMMER3)
        /* Only need to charge up the DMA if status strings aren't enabled. Otherwise this is handled within the setup/DMA code. */
        setDmaWaitingForResponseIfStatusStrDisabled();
#endif
        ShimSdSync_centerT10();
      }
    }
    else
    {
      ShimSdSync_resetSyncRcNodeR10Cnt();
      //node is waiting for 1 byte ROUTINE_COMMUNICATION(0xE0)
#if defined(SHIMMER3)
      /* Only need to charge up the DMA if status strings aren't enabled. Otherwise this is handled within the setup/DMA code. */
      setDmaWaitingForResponseIfStatusStrDisabled();
#endif
    }
  }
  else
  { //BT is disconnected
    shimmerStatus.btstreamReady = 0;
    shimmerStatus.btstreamCmd = BT_STREAM_CMD_STATE_STOP;

    ShimBt_setBtDataRateTestState(0);

    clearBtTxBuf(0);

    ShimBt_setCrcMode(CRC_OFF);
    /* Revert to default state if changed */
    ShimBt_resetBtResponseVars();

    /* Check BT module configuration after disconnection in case
     * sensor configuration (i.e., BT on vs. BT off vs. SD Sync) was changed
     *  during the last connection. */
    ShimConfig_checkBtModeFromConfig();
  }

  if (!shimmerStatus.sensing)
  {
    ShimTask_set(TASK_SDLOG_CFG_UPDATE);
  }
}

volatile uint8_t *ShimBt_getBtActionPtr(void)
{
  return &gAction;
}

uint8_t *ShimBt_getBtArgsPtr(void)
{
  return &args[0];
}
