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

#include "5xx_HAL/hal_Board.h"
#include "5xx_HAL/hal_CRC.h"
#include "5xx_HAL/hal_DMA.h"
#include "5xx_HAL/hal_RTC.h"
#include "RN4X/RN4678.h"
#include "RN4X/RN4X.h"

#elif defined(SHIMMER3R) || defined(SHIMMER4_SDK)
#include "BMP3/BMP3_SensorAPI/bmp3_defs.h"
#include "hal_FactoryTest.h"
#include "hal_Infomem.h"
#include "shimmer_definitions.h"
#endif

#include "log_and_stream_externs.h"
#include "log_and_stream_includes.h"
#include "shimmer_definitions.h"
#include "version.h"

uint8_t unwrappedResponse[256] = { 0 };
volatile uint8_t gAction;
uint8_t args[MAX_COMMAND_ARG_SIZE], waitingForArgs, waitingForArgsLength, argsSize;

#if defined(SHIMMER3)
volatile char btRxBuffFullResponse[BT_VER_RESPONSE_LARGEST + 1U]; /* +1 to always have a null char */
uint8_t btRxBuff[MAX_COMMAND_ARG_SIZE], *btRxExp;
#endif

uint8_t *btRxBuffPtr;
volatile COMMS_CRC_MODE btCrcMode;

/* CYW20820 app=v01.04.16.16 requires size = 76 chars. Shimmer3 requires BT_VER_RESPONSE_LARGEST */
char btVerStrResponse[100U];

uint16_t numBytesInBtRxBufWhenLastProcessed = 0;
uint16_t indexOfFirstEol;
uint32_t firstProcessFailTicks = 0;

uint8_t useAckPrefixForInstreamResponses;
uint8_t sendAck, sendNack, getCmdWaitingResponse;
uint8_t infomemLength, dcMemLength, calibRamLength;
uint16_t infomemOffset, dcMemOffset, calibRamOffset;

//ExG
uint8_t exgLength, exgChip, exgStartAddr;

uint8_t btDataRateTestState;
#if defined(SHIMMER3)
uint32_t btDataRateTestCounter;
#else
uint8_t dataRateTestTxPacket[] = { DATA_RATE_TEST_RESPONSE, 0, 0, 0, 0 };
#endif

volatile uint8_t btTxInProgress;

char macIdStr[12 + 1]; //+1 for null termination
uint8_t macIdBytes[6];

//Enum for Shimmer3. Actual Baud value for Shimmer3R.
uint32_t btBaudRateToUse;

/* Buffer read / write macros                                                 */
#define RINGFIFO_RESET(ringFifo)         \
  {                                      \
    ringFifo.rdIdx = ringFifo.wrIdx = 0; \
  }
#define RINGFIFO_WR(ringFifo, dataIn, mask)            \
  {                                                    \
    ringFifo.data[mask & ringFifo.wrIdx++] = (dataIn); \
  }
#define RINGFIFO_RD(ringFifo, dataOut, mask)              \
  {                                                       \
    ringFifo.rdIdx++;                                     \
    dataOut = ringFifo.data[mask & (ringFifo.rdIdx - 1)]; \
  }
#define RINGFIFO_EMPTY(ringFifo) (ringFifo.rdIdx == ringFifo.wrIdx)
#define RINGFIFO_FULL(ringFifo, mask) \
  ((mask & ringFifo.rdIdx) == (mask & (ringFifo.wrIdx + 1)))
#define RINGFIFO_COUNT(ringFifo, mask) \
  (mask & (ringFifo.wrIdx - ringFifo.rdIdx))

volatile RingFifoTx_t gBtTxFifo;

uint8_t bleCurrentlyEnabled = 1, btClassicCurrentlyEnabled = 1;

uint8_t sensingStateChangeFromBtCmd = 0;

void ShimBt_btCommsProtocolInit(void)
{
  ShimBt_setCrcMode(CRC_OFF);
  numBytesInBtRxBufWhenLastProcessed = 0;
  indexOfFirstEol = 0;
  firstProcessFailTicks = 0;
  memset(unwrappedResponse, 0, sizeof(unwrappedResponse));
  sensingStateChangeFromBtCmd = 0;

#if defined(SHIMMER3)
  btRxExp = BT_getExpResp();
#endif

  waitingForArgs = 0;
  waitingForArgsLength = 0;
  argsSize = 0;

  ShimBt_resetBtRxBuffs();

#if defined(SHIMMER3)
  RN4678_resetStatusString();

  setBtRxFullResponsePtr(btRxBuffFullResponse);

  DMA2_init((uint16_t *) &UCA1RXBUF, (uint16_t *) btRxBuff, sizeof(btRxBuff));
  DMA2_transferDoneFunction(&ShimBt_dmaConversionDone);
  //DMA2SZ = 1U;
  //DMA2_enable();
#endif

  memset(btVerStrResponse, 0x00, sizeof(btVerStrResponse) / sizeof(btVerStrResponse[0]));

  ShimBt_setDataRateTestState(0);

  ShimBt_resetBtResponseVars();
  ShimBt_macIdVarsReset();

  ShimBt_clearBtTxBuf(1U);

  ShimBt_btTxInProgressSet(0);

  ShimBt_setBtMode(1, 1);
}

void ShimBt_startCommon(void)
{
  if (!shimmerStatus.sensing)
  {
    shimmerStatus.configuring = 1;
  }
  shimmerStatus.btInSyncMode = shimmerStatus.sdSyncEnabled;

  if (shimmerStatus.sdSyncEnabled || !ShimEeprom_isPresent())
  {
    /* Enable classic BT only for sync mode or older devices without EEPROM
     * (which use RN42 modules without BLE support) */
    ShimBt_setBtMode(1, 0);
  }
  else
  {
    ShimBt_setBtMode(ShimEeprom_isBtClassicEnabled(), ShimEeprom_isBleEnabled());
  }
}

void ShimBt_stopCommon(uint8_t isCalledFromMain)
{
  ShimTask_clear(TASK_BT_RESPOND);
  ShimTask_clear(TASK_RCCENTERR1);
  ShimTask_clear(TASK_RCNODER10);

  ShimBt_clearBtTxBuf(isCalledFromMain);
  ShimBt_resetBtRxBuffs();

  shimmerStatus.btConnected = 0;
  shimmerStatus.btIsInitialised = 0;
  shimmerStatus.btInSyncMode = 0;
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

void ShimBt_resetBtRxVariablesOnConnect(void)
{
  /* Reset to unsupported command */
  gAction = ACK_COMMAND_PROCESSED - 1U;
  waitingForArgs = 0;
  waitingForArgsLength = 0;
}

void ShimBt_resetBtRxBuffs(void)
{
#if defined(SHIMMER3)
  ShimUtil_memset_v(btRxBuffFullResponse, 0x00,
      sizeof(btRxBuffFullResponse) / sizeof(btRxBuffFullResponse[0]));
  memset(btRxBuff, 0, sizeof(btRxBuff));
#else
  resetBtRxBuff();
#endif
}

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
      uint8_t btOffset
          = ShimUtil_strlen_v(btRxBuffFullResponse, sizeof(btRxBuffFullResponse));
      ShimUtil_memcpy_v(btRxBuffFullResponse + btOffset, btRxBuffPtr,
          strlen((char *) btRxBuffPtr));
      memset(btRxBuffPtr, 0, strlen((char *) btRxBuffPtr));

      uint8_t responseLen
          = ShimUtil_strlen_v(btRxBuffFullResponse, sizeof(btRxBuffFullResponse));

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
      ShimBt_macIdSetFromStr(btRxBuffPtr);

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
      uint8_t btOffset
          = ShimUtil_strlen_v(btRxBuffFullResponse, sizeof(btRxBuffFullResponse));
      ShimUtil_memcpy_v(btRxBuffFullResponse + btOffset, btRxBuffPtr,
          strlen((char *) btRxBuffPtr));
      memset(btRxBuffPtr, 0, strlen((char *) btRxBuffPtr));

      uint8_t btVerLen
          = ShimUtil_strlen_v(btRxBuffFullResponse, sizeof(btRxBuffFullResponse));
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

        /* When storing the BT version, ignore "CMD>" that appears at the end for the RN4678 */
        uint8_t btVerLen
            = ShimUtil_strlen_v(btRxBuffFullResponse, sizeof(btRxBuffFullResponse));
        uint8_t btVerIdx;
        //+3 here to make sure we don't go past the end of the array
        for (btVerIdx = 0; btVerIdx + 3 < btVerLen; btVerIdx++)
        {
          if ((btRxBuffFullResponse[btVerIdx] == 'C')
              && (btRxBuffFullResponse[btVerIdx + 1] == 'M')
              && (btRxBuffFullResponse[btVerIdx + 2] == 'D')
              && (btRxBuffFullResponse[btVerIdx + 3] == '>'))
          {
            btVerLen = btVerIdx;
            break;
          }
        }
        ShimUtil_memcpy_vv(btVerStrResponse, btRxBuffFullResponse, btVerLen);

        ShimUtil_memset_v(btRxBuffFullResponse, 0, strlen((char *) btRxBuffFullResponse));
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
            && (gAction == SET_INFOMEM_COMMAND || gAction == SET_CALIB_DUMP_COMMAND
                || gAction == SET_DAUGHTER_CARD_MEM_COMMAND || gAction == SET_EXG_REGS_COMMAND))
        {
          args[0] = btRxBuffPtr[0];
          args[1] = btRxBuffPtr[1];
          args[2] = btRxBuffPtr[2];
          if (gAction == SET_EXG_REGS_COMMAND)
          {
            waitingForArgsLength = args[2];
          }
          else
          {
            waitingForArgsLength = args[0];
          }
          setDmaWaitingForResponse(waitingForArgsLength);
          return 0;
        }
        else if ((!waitingForArgsLength) && (waitingForArgs == 2)
            && (gAction == SET_DAUGHTER_CARD_ID_COMMAND))
        {
          args[0] = btRxBuffPtr[0];
          args[1] = btRxBuffPtr[1];
          if (args[0])
          {
            waitingForArgsLength = args[0];
            setDmaWaitingForResponse(waitingForArgsLength);
          }
          return 0;
        }
        else if ((!waitingForArgsLength) && (waitingForArgs == 1)
            && (gAction == SET_CENTER_COMMAND || gAction == SET_CONFIGTIME_COMMAND
                || gAction == SET_EXPID_COMMAND || gAction == SET_SHIMMERNAME_COMMAND))
        {
          args[0] = btRxBuffPtr[0];
          if (args[0])
          {
            waitingForArgsLength = args[0];
            setDmaWaitingForResponse(waitingForArgsLength);
            return 0;
          }
        }

#if defined(SHIMMER3)
        else if (gAction == RN4678_STATUS_STRING_SEPARATOR)
        {
          return RN4678_parseStatusString(&waitingForArgs, btRxBuffPtr);
        }
#endif

        else if (gAction == ACK_COMMAND_PROCESSED)
        {
          /* If waiting for command byte */
          if (!waitingForArgsLength)
          {
            /* Save command byte */
            args[0] = btRxBuffPtr[0];

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
          memcpy(&args[waitingForArgs], btRxBuffPtr, waitingForArgsLength);
        }
        else
        {
          memcpy(&args[0], btRxBuffPtr, waitingForArgs);
        }

        waitingForArgsLength = 0;
        waitingForArgs = 0;
        argsSize = 0;
        setDmaWaitingForResponse(1U);
        return ShimTask_set(TASK_BT_PROCESS_CMD);
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
            gAction = data;
            ShimTask_set(TASK_BT_PROCESS_CMD);
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
            gAction = data;
            waitingForArgs = 1U;
            break;
          case SET_SAMPLING_RATE_COMMAND:
          case GET_DAUGHTER_CARD_ID_COMMAND:
          case SET_DAUGHTER_CARD_ID_COMMAND:
            gAction = data;
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
            gAction = data;
            waitingForArgs = 3U;
            break;
          case SET_CONFIG_SETUP_BYTES_COMMAND:
            gAction = data;
            waitingForArgs = 4U;
            break;
          case SET_RWC_COMMAND:
          case SET_DERIVED_CHANNEL_BYTES:
            gAction = data;
            waitingForArgs = 8U;
            break;
          case SET_LN_ACCEL_CALIBRATION_COMMAND:
          case SET_GYRO_CALIBRATION_COMMAND:
          case SET_MAG_CALIBRATION_COMMAND:
          case SET_WR_ACCEL_CALIBRATION_COMMAND:
          case SET_ALT_ACCEL_CALIBRATION_COMMAND:
          case SET_ALT_MAG_CALIBRATION_COMMAND:
            gAction = data;
            waitingForArgs = SC_DATA_LEN_STD_IMU_CALIB;
            break;
#if defined(SHIMMER3)
          case RN4678_STATUS_STRING_SEPARATOR:
            RN4678_startOfNewStatusString();
            gAction = data;
            /* Minus 1 because we've already received 1 x RN4678_STATUS_STRING_SEPARATOR */
            waitingForArgs = BT_STAT_STR_LEN_SMALLEST - 1U;
            break;
#endif
          case ACK_COMMAND_PROCESSED:
            /* Wait for command byte */
            gAction = data;
            waitingForArgs = 1U;
            break;
          case SET_SD_SYNC_COMMAND:
            /* Store local time as early as possible after sync bytes have been received */
            ShimSdSync_saveLocalTime();

            gAction = data;
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

void ShimBt_processCmd(void)
{
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

  uint8_t update_sdconfig = 0, update_calib_dump_file = 0;
  uint8_t fullSyncResp[SYNC_PACKET_MAX_SIZE] = { 0 };
  uint8_t sensorCalibId;

  /* Block non-sync related commands if sync is enabled. Equally, block sync commands if sync is disabled. XOR condition will sendNack if only one is true. */
  if (storedConfigPtr->syncEnable ^ ShimBt_isCmdAllowedWhileSdSyncing(gAction))
  {
    sendNack = 1;
  }
  /* Block set commands if sensing */
  else if (shimmerStatus.sensing && ShimBt_isCmdBlockedWhileSensing(gAction))
  {
    sendNack = 1;
  }
  else
  {
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
      {
        getCmdWaitingResponse = gAction;
        break;
      }
      case DUMMY_COMMAND:
      {
        break;
      }
      case TOGGLE_LED_COMMAND:
      {
        shimmerStatus.toggleLedRedCmd ^= 1;
        break;
      }
      case START_STREAMING_COMMAND:
      {
        sensingStateChangeFromBtCmd = 1;
        ShimTask_setStartStreamingIfReady();
        break;
      }
      case START_SDBT_COMMAND:
      {
        sensingStateChangeFromBtCmd = 1;
        ShimTask_setStartStreamingAndLoggingIfReady();
        break;
      }
      case START_LOGGING_COMMAND:
      {
        sensingStateChangeFromBtCmd = 1;
        ShimTask_setStartLoggingIfReady();
        break;
      }
      case SET_CRC_COMMAND:
      {
        ShimBt_setCrcMode((COMMS_CRC_MODE) args[0]);
        break;
      }
      case SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE:
      {
        useAckPrefixForInstreamResponses = args[0];
        break;
      }
      case STOP_STREAMING_COMMAND:
      {
        sensingStateChangeFromBtCmd = 1;
        ShimTask_setStopStreaming();
        break;
      }
      case STOP_SDBT_COMMAND:
      {
        sensingStateChangeFromBtCmd = 1;
        ShimTask_setStopSensing();
        break;
      }
      case STOP_LOGGING_COMMAND:
      {
        sensingStateChangeFromBtCmd = 1;
        ShimTask_setStopLogging();
        break;
      }
      case SET_SENSORS_COMMAND:
      {
        ShimConfig_storedConfigSet(&args[0], NV_SENSORS0, 3);
        ShimBt_settingChangeCommon(NV_SENSORS0, SDH_SENSORS0, 3);
        break;
      }
#if defined(SHIMMER4_SDK)
      case GET_I2C_BATT_STATUS_COMMAND:
      {
        getCmdWaitingResponse = gAction;
        break;
      }
      case SET_I2C_BATT_STATUS_FREQ_COMMAND:
      {
        temp16 = args[0] + ((uint16_t) args[1] << 8);
        I2C_readBattSetFreq(temp16);
        break;
      }
#endif
      case SET_TRIAL_CONFIG_COMMAND:
      {
        storedConfigPtr->rawBytes[NV_SD_TRIAL_CONFIG0] = args[0];
        storedConfigPtr->rawBytes[NV_SD_TRIAL_CONFIG1] = args[1];
        storedConfigPtr->rawBytes[NV_SD_BT_INTERVAL] = args[2];

        //Save TRIAL_CONFIG0, TRIAL_CONFIG1 and BT_INTERVAL
        ShimBt_settingChangeCommon(NV_SD_TRIAL_CONFIG0, SDH_TRIAL_CONFIG0, 3);
        break;
      }
      case SET_CENTER_COMMAND:
      {
        storedConfigPtr->masterEnable = args[0] & 0x01;
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE4, SDH_CONFIG_SETUP_BYTE4, 1);
        break;
      }
      case SET_SHIMMERNAME_COMMAND:
      {
        ShimConfig_shimmerNameSet(&args[1], args[0]);
        InfoMem_write(NV_SD_SHIMMER_NAME, (uint8_t *) &storedConfigPtr->shimmerName[0],
            sizeof(storedConfigPtr->shimmerName));
        update_sdconfig = 1;
        break;
      }
      case SET_EXPID_COMMAND:
      {
        ShimConfig_expIdSet(&args[1], args[0]);
        InfoMem_write(NV_SD_EXP_ID_NAME, (uint8_t *) &storedConfigPtr->expIdName[0],
            sizeof(storedConfigPtr->expIdName));
        update_sdconfig = 1;
        break;
      }
      case SET_CONFIGTIME_COMMAND:
      {
        ShimConfig_configTimeSetFromStr(&args[1], args[0]);
        ShimSdHead_sdHeadTextSet(&storedConfigPtr->configTime0, SDH_CONFIG_TIME_0, 4);
        InfoMem_write(NV_SD_CONFIG_TIME, &storedConfigPtr->rawBytes[NV_SD_CONFIG_TIME], 4);
        update_sdconfig = 1;
        break;
      }
      case SET_NSHIMMER_COMMAND:
      {
        storedConfigPtr->numberOfShimmers = args[0];
        ShimBt_settingChangeCommon(NV_SD_NSHIMMER, SDH_NSHIMMER, 1);
        break;
      }
      case SET_MYID_COMMAND:
      {
        storedConfigPtr->myTrialID = args[0];
        ShimBt_settingChangeCommon(NV_SD_MYTRIAL_ID, SDH_MYTRIAL_ID, 1);
        break;
      }
      case SET_WR_ACCEL_RANGE_COMMAND:
      {
        storedConfigPtr->wrAccelRange = args[0] < 4 ? (args[0] & 0x03) : 0;
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE0, SDH_CONFIG_SETUP_BYTE0, 1);
        break;
      }
      case GET_EXG_REGS_COMMAND:
      {
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
      }
      case SET_WR_ACCEL_SAMPLING_RATE_COMMAND:
      {
#if defined(SHIMMER3)
        storedConfigPtr->wrAccelRate
            = (args[0] <= LSM303DLHC_ACCEL_1_344kHz) ? args[0] : LSM303DLHC_ACCEL_100HZ;
#elif defined(SHIMMER3R)
        storedConfigPtr->wrAccelRate
            = (args[0] <= LIS2DW12_XL_ODR_1k6Hz) ? args[0] : LIS2DW12_XL_ODR_100Hz;
#endif
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE0, SDH_CONFIG_SETUP_BYTE0, 1);
        break;
      }
      case SET_MAG_GAIN_COMMAND:
      {
#if defined(SHIMMER3)
        storedConfigPtr->magRange
            = (args[0] <= LSM303DLHC_MAG_8_1G) ? args[0] : LSM303DLHC_MAG_1_3G;
#elif defined(SHIMMER3R)
        storedConfigPtr->altMagRange = (args[0] <= LIS3MDL_16_GAUSS) ? args[0] : LIS3MDL_4_GAUSS;
#endif
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE2, SDH_CONFIG_SETUP_BYTE2, 1);
        break;
      }
      case SET_MAG_SAMPLING_RATE_COMMAND:
      {
        ShimConfig_configByteMagRateSet(args[0]);
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE2, SDH_CONFIG_SETUP_BYTE2, 1);
        break;
      }
      case SET_WR_ACCEL_LPMODE_COMMAND:
      {
        ShimConfig_wrAccelLpModeSet(args[0] == 1 ? 1 : 0);
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE0, SDH_CONFIG_SETUP_BYTE0, 1);
        break;
      }
      case SET_WR_ACCEL_HRMODE_COMMAND:
      {
        storedConfigPtr->wrAccelHrMode = (args[0] == 1) ? 1 : 0;
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE0, SDH_CONFIG_SETUP_BYTE0, 1);
        break;
      }
      case SET_GYRO_RANGE_COMMAND:
      {
        ShimConfig_gyroRangeSet(args[0]);
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE2, SDH_CONFIG_SETUP_BYTE2, 1);
        break;
      }
      case SET_GYRO_SAMPLING_RATE_COMMAND:
      {
        ShimConfig_gyroRateSet(args[0]);
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE1, SDH_CONFIG_SETUP_BYTE1, 1);
        break;
      }
      case SET_ALT_ACCEL_RANGE_COMMAND:
      {
#if defined(SHIMMER3)
        storedConfigPtr->altAccelRange = (args[0] <= ACCEL_16G) ? (args[0] & 0x03) : ACCEL_2G;
#elif defined(SHIMMER3R)
        storedConfigPtr->lnAccelRange
            = (args[0] <= LSM6DSV_16g) ? (args[0] & 0x03) : LSM6DSV_2g;
#endif
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE3, SDH_CONFIG_SETUP_BYTE3, 1);
        break;
      }
      case SET_PRESSURE_OVERSAMPLING_RATIO_COMMAND:
      {
        ShimConfig_configBytePressureOversamplingRatioSet(args[0]);
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE3, SDH_CONFIG_SETUP_BYTE3, 1);
        break;
      }
      case SET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
      {
        storedConfigPtr->expansionBoardPower = (args[0] == 1) ? 1 : 0;
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE3, SDH_CONFIG_SETUP_BYTE3, 1);
        break;
      }
      case SET_CONFIG_SETUP_BYTES_COMMAND:
      {
        ShimConfig_storedConfigSet(&args[0], NV_CONFIG_SETUP_BYTE0, 4);
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE0, SDH_CONFIG_SETUP_BYTE0, 4);
        break;
      }
      case SET_SAMPLING_RATE_COMMAND:
      {
        storedConfigPtr->samplingRateTicks = *(uint16_t *) args;
        //ShimConfig_storedConfigSet(&args[0], NV_SAMPLING_RATE, 2);
        ShimBt_settingChangeCommon(NV_SAMPLING_RATE, SDH_SAMPLE_RATE_0, 2);
        break;
      }
      case GET_CALIB_DUMP_COMMAND:
      {
        //usage:
        //0x98, offset, offset, length
        calibRamLength = args[0];
        calibRamOffset = args[1] + (args[2] << 8);
        getCmdWaitingResponse = gAction;
        break;
      }
      case SET_CALIB_DUMP_COMMAND:
      {
        //usage:
        //0x98, offset, offset, length, data[0:127]
        //max length of this command = 132
        calibRamLength = args[0];
        calibRamOffset = args[1] + (args[2] << 8);
        if (ShimCalib_ramWrite(&args[3], calibRamLength, calibRamOffset) == 1)
        {
          ShimCalib_calibDumpToConfigBytesAndSdHeaderAll(1);
          update_calib_dump_file = 1;
        }
        break;
      }
      case UPD_CALIB_DUMP_COMMAND:
      {
        ShimCalib_calibDumpToConfigBytesAndSdHeaderAll(1);
        update_calib_dump_file = 1;
        break;
      }
      case UPD_SDLOG_CFG_COMMAND:
      {
        ShimTask_set(TASK_SDLOG_CFG_UPDATE);
        break;
      }
        //case UPD_FLASH_COMMAND:
        //{
        //  LogAndStream_infomemUpdate();
        //  //ShimmerCalibSyncFromDumpRamAll();
        //  //update_calib_dump_file = 1;
        //  break;
        //}
      case SET_LN_ACCEL_CALIBRATION_COMMAND:
      {
#if defined(SHIMMER3)
        sensorCalibId = SC_SENSOR_ANALOG_ACCEL;
#elif defined(SHIMMER3R)
        sensorCalibId = SC_SENSOR_LSM6DSV_ACCEL;
#endif
        ShimBt_calibrationChangeCommon(NV_LN_ACCEL_CALIBRATION, SDH_LN_ACCEL_CALIBRATION,
            &storedConfigPtr->lnAccelCalib.rawBytes[0], &args[0], sensorCalibId);
        update_calib_dump_file = 1;
        break;
      }
      case SET_GYRO_CALIBRATION_COMMAND:
      {
#if defined(SHIMMER3)
        sensorCalibId = SC_SENSOR_MPU9X50_ICM20948_GYRO;
#elif defined(SHIMMER3R)
        sensorCalibId = SC_SENSOR_LSM6DSV_GYRO;
#endif
        ShimBt_calibrationChangeCommon(NV_GYRO_CALIBRATION, SDH_GYRO_CALIBRATION,
            &storedConfigPtr->gyroCalib.rawBytes[0], &args[0], sensorCalibId);
        update_calib_dump_file = 1;
        break;
      }
      case SET_MAG_CALIBRATION_COMMAND:
      {
#if defined(SHIMMER3)
        sensorCalibId = SC_SENSOR_LSM303_MAG;
#elif defined(SHIMMER3R)
        sensorCalibId = SC_SENSOR_LIS2MDL_MAG;
#endif
        ShimBt_calibrationChangeCommon(NV_MAG_CALIBRATION, SDH_MAG_CALIBRATION,
            &storedConfigPtr->magCalib.rawBytes[0], &args[0], sensorCalibId);
        update_calib_dump_file = 1;
        break;
      }
      case SET_WR_ACCEL_CALIBRATION_COMMAND:
      {
#if defined(SHIMMER3)
        sensorCalibId = SC_SENSOR_LSM303_ACCEL;
#elif defined(SHIMMER3R)
        sensorCalibId = SC_SENSOR_LIS2DW12_ACCEL;
#endif
        ShimBt_calibrationChangeCommon(NV_WR_ACCEL_CALIBRATION, SDH_WR_ACCEL_CALIBRATION,
            &storedConfigPtr->wrAccelCalib.rawBytes[0], &args[0], sensorCalibId);
        update_calib_dump_file = 1;
        break;
      }
      case SET_GSR_RANGE_COMMAND:
      {
        storedConfigPtr->gsrRange = (args[0] <= 4) ? (args[0] & 0x07) : GSR_AUTORANGE;
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE3, SDH_CONFIG_SETUP_BYTE3, 1);
        break;
      }
      case SET_EXG_REGS_COMMAND:
      {
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
            storedConfigPtr->exgADS1292rRegsCh1.config2 |= 8;
          }

          InfoMem_write(exgConfigOffset + exgStartAddr,
              &storedConfigPtr->rawBytes[exgConfigOffset + exgStartAddr], exgLength);
          ShimSdHead_sdHeadTextSet(&storedConfigPtr->rawBytes[exgConfigOffset],
              exgSdHeadOffset, exgLength);

          update_sdconfig = 1;
        }
        break;
      }
      case SET_DATA_RATE_TEST:
      {
        /* Stop test before ACK is sent */
        if (args[0] == 0)
        {
          ShimBt_setDataRateTestState(0);
          ShimBt_clearBtTxBuf(1);
        }
        getCmdWaitingResponse = gAction;
        break;
      }
      case SET_FACTORY_TEST:
      {
        if (args[0] < FACTORY_TEST_COUNT)
        {
          ShimFactoryTest_setup(PRINT_TO_BT_UART, (factory_test_t) args[0]);
          ShimTask_set(TASK_FACTORY_TEST);
        }
        break;
      }
      case RESET_TO_DEFAULT_CONFIGURATION_COMMAND:
      {
        ShimConfig_setDefaultConfig();
        ShimSdHead_config2SdHead();
        update_sdconfig = 1;
        break;
      }
      case RESET_CALIBRATION_VALUE_COMMAND:
      {
        ShimCalib_init();
        ShimCalib_calibDumpToConfigBytesAndSdHeaderAll(1);
        update_calib_dump_file = 1;
        break;
      }
      case GET_DAUGHTER_CARD_ID_COMMAND:
      {
        dcMemLength = args[0];
        dcMemOffset = args[1];
        if ((dcMemLength <= 16) && (dcMemOffset <= 15) && (dcMemLength + dcMemOffset <= 16))
        {
          getCmdWaitingResponse = gAction;
        }
        break;
      }
      case SET_DAUGHTER_CARD_ID_COMMAND:
      {
        dcMemLength = args[0];
        dcMemOffset = args[1];
        if ((dcMemLength <= 16) && (dcMemOffset <= 15) && (dcMemLength + dcMemOffset <= 16))
        {
          eepromWrite(dcMemOffset, dcMemLength, &args[2]);
          LogAndStream_processDaughterCardId();
        }
        break;
      }
      case GET_DAUGHTER_CARD_MEM_COMMAND:
      {
        dcMemLength = args[0];
        dcMemOffset = args[1] + (args[2] << 8);
        if ((dcMemLength <= 128) && (dcMemOffset <= 2031)
            && (dcMemLength + dcMemOffset <= 2032))
        {
          getCmdWaitingResponse = gAction;
        }
        break;
      }
      case SET_DAUGHTER_CARD_MEM_COMMAND:
      {
        dcMemLength = args[0];
        dcMemOffset = args[1] + (args[2] << 8);
        if (!ShimEeprom_writeDaughterCardMem(dcMemOffset, dcMemLength, &args[3]))
        {
          sendNack = 1;
        }
        break;
      }
      case SET_BT_COMMS_BAUD_RATE:
      {
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
      }
      case SET_DERIVED_CHANNEL_BYTES:
      {
        memcpy(&storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_0], &args[0], 3);
        memcpy(&storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_3], &args[3], 5);
        InfoMem_write(NV_DERIVED_CHANNELS_0,
            &storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_0], 3);
        InfoMem_write(NV_DERIVED_CHANNELS_3,
            &storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_3], 5);
        ShimSdHead_sdHeadTextSet(&storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_0],
            SDH_DERIVED_CHANNELS_0, 3);
        ShimSdHead_sdHeadTextSet(&storedConfigPtr->rawBytes[NV_DERIVED_CHANNELS_3],
            SDH_DERIVED_CHANNELS_3, 5);
        update_sdconfig = 1;
        break;
      }
      case GET_INFOMEM_COMMAND:
      {
        infomemLength = args[0];
        infomemOffset = args[1] + (args[2] << 8);
        if ((infomemLength <= 128) && (infomemOffset <= (NV_NUM_RWMEM_BYTES - 1))
            && (infomemLength + infomemOffset <= NV_NUM_RWMEM_BYTES))
        {
          getCmdWaitingResponse = gAction;
        }
        break;
      }
      case SET_INFOMEM_COMMAND:
      {
        infomemLength = args[0];
        infomemOffset = args[1] + (args[2] << 8);
        if ((infomemLength <= 128) && (infomemOffset <= (NV_NUM_RWMEM_BYTES - 1))
            && (infomemLength + infomemOffset <= NV_NUM_RWMEM_BYTES))
        {
          ShimConfig_storedConfigSet(&args[3], infomemOffset, infomemLength);

          if (infomemOffset == (INFOMEM_SEG_C_ADDR_MSP430 - INFOMEM_OFFSET_MSP430))
          {
            /* Always overwrite MAC ID */
            memcpy(&ShimConfig_getStoredConfig()->macAddr[0],
                ShimBt_macIdBytesPtrGet(), 6);
          }

          ShimConfig_checkAndCorrectConfig();

          LogAndStream_infomemUpdate();
          //InfoMem_write(infomemOffset, &args[3], infomemLength);
          //InfoMem_read(infomemOffset, &storedConfigPtr->rawBytes[infomemOffset], infomemLength);

          /* Save from infomem to calib dump in memory */
          if (infomemOffset == (INFOMEM_SEG_D_ADDR_MSP430 - INFOMEM_OFFSET_MSP430))
          {
            ShimCalib_configBytes0To127ToCalibDumpBytes(0);
            update_calib_dump_file = 1;
          }
#if defined(SHIMMER3R)
          else if (infomemOffset == (INFOMEM_SEG_C_ADDR_MSP430 - INFOMEM_OFFSET_MSP430))
          {
            ShimCalib_configBytes128To255ToCalibDumpBytes(0);
            update_calib_dump_file = 1;
          }
#endif

          ShimSdHead_config2SdHead();
          update_sdconfig = 1;
        }
        else
        {
          return;
        }
        break;
      }
      case SET_RWC_COMMAND:
      {
        storedConfigPtr->rtcSetByBt = 1;
        InfoMem_write(NV_SD_TRIAL_CONFIG0,
            &storedConfigPtr->rawBytes[NV_SD_TRIAL_CONFIG0], 1);
        ShimSdHead_sdHeadTextSetByte(
            SDH_TRIAL_CONFIG0, storedConfigPtr->rawBytes[NV_SD_TRIAL_CONFIG0]);

        RTC_setTimeFromTicksPtr(&args[0]);
        ShimRtc_rwcErrorCheck();
#if defined(SHIMMER3R)
        RTC_setAlarmBattRead(); //configure RTC alarm after time set from BT.
#endif
        break;
      }
      case SET_ALT_ACCEL_CALIBRATION_COMMAND:
      {
#if defined(SHIMMER3)
        sensorCalibId = SC_SENSOR_MPU9X50_ICM20948_ACCEL;
#elif defined(SHIMMER3R)
        sensorCalibId = SC_SENSOR_ADXL371_ACCEL;
#endif
        ShimBt_calibrationChangeCommon(NV_ALT_ACCEL_CALIBRATION, SDH_ALT_ACCEL_CALIBRATION,
            &storedConfigPtr->altAccelCalib.rawBytes[0], &args[0], sensorCalibId);
        update_calib_dump_file = 1;
        break;
      }
      case SET_ALT_MAG_CALIBRATION_COMMAND:
      {
#if defined(SHIMMER3)
        sensorCalibId = SC_SENSOR_MPU9X50_ICM20948_MAG;
#elif defined(SHIMMER3R)
        sensorCalibId = SC_SENSOR_LIS3MDL_MAG;
#endif
        ShimBt_calibrationChangeCommon(NV_ALT_MAG_CALIBRATION, SDH_ALT_MAG_CALIBRATION,
            &storedConfigPtr->altMagCalib.rawBytes[0], &args[0], sensorCalibId);
        update_calib_dump_file = 1;
        break;
      }
      case SET_ALT_ACCEL_SAMPLING_RATE_COMMAND:
      {
        storedConfigPtr->altAccelRate = args[0] & 0x03;
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE4, SDH_CONFIG_SETUP_BYTE4, 1);
        break;
      }
      case SET_ALT_MAG_SAMPLING_RATE_COMMAND:
      {
        ShimConfig_configByteAltMagRateSet(args[0]);
        ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE5, SDH_CONFIG_SETUP_BYTE5, 1);
        break;
      }
      case SET_SD_SYNC_COMMAND:
      {
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
        break;
      }
      case ACK_COMMAND_PROCESSED:
      {
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
      }
      default:
      {
        break;
      }
    }
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
    ShimConfig_setFlagWriteCfgToSd(1, 1);
  }
  if (update_calib_dump_file)
  {
    ShimBt_updateCalibDumpFile();
  }
}

void ShimBt_settingChangeCommon(uint16_t configByteIdx, uint16_t sdHeaderIdx, uint16_t len)
{
  gConfigBytes *storedConfig = ShimConfig_getStoredConfig();

  ShimConfig_checkAndCorrectConfig();
  ShimConfig_setFlagWriteCfgToSd(1, 0);

  InfoMem_write(configByteIdx, &storedConfig->rawBytes[configByteIdx], len);
  ShimSdHead_sdHeadTextSet(&storedConfig->rawBytes[configByteIdx], sdHeaderIdx, len);
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

  ShimCalib_configBytesToCalibDump(sensorCalibId, 0);
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
    sc1.range = storedConfig->magRange;
#elif defined(SHIMMER3R)
    sc1.id = SC_SENSOR_LIS2MDL_MAG;
    sc1.range = SC_SENSOR_RANGE_LIS2MDL_RANGE;
#endif
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
    sc1.range = 0;
#elif defined(SHIMMER3R)
    sc1.id = SC_SENSOR_LIS3MDL_MAG;
    sc1.range = storedConfig->altMagRange;
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
      {
        break;
      }
      case INQUIRY_COMMAND:
      {
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

        uint8_t numberOfChannels = ShimSens_getNumEnabledChannels();

        *(resPacket + packet_length++) = numberOfChannels; //number of data channels
        *(resPacket + packet_length++) = storedConfig->bufferSize; //buffer size
        memcpy((resPacket + packet_length), sensing.cc, numberOfChannels);
        packet_length += numberOfChannels;
        break;
      }
      case GET_SAMPLING_RATE_COMMAND:
      {
        *(resPacket + packet_length++) = SAMPLING_RATE_RESPONSE;
        *(uint16_t *) (resPacket + packet_length) = storedConfig->samplingRateTicks; //ADC sampling rate
        packet_length += 2;
        break;
      }
      case GET_WR_ACCEL_RANGE_COMMAND:
      {
        *(resPacket + packet_length++) = WR_ACCEL_RANGE_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->wrAccelRange;
        break;
      }
      case GET_MAG_GAIN_COMMAND:
      {
        *(resPacket + packet_length++) = MAG_GAIN_RESPONSE;
#if defined(SHIMMER3)
        *(resPacket + packet_length++) = storedConfig->magRange;
#else
        *(resPacket + packet_length++) = storedConfig->altMagRange;
#endif
        break;
      }
      case GET_MAG_SAMPLING_RATE_COMMAND:
      {
        *(resPacket + packet_length++) = MAG_SAMPLING_RATE_RESPONSE;
        *(resPacket + packet_length++) = ShimConfig_configByteMagRateGet();
        break;
      }
      case GET_STATUS_COMMAND:
      {
        *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
        *(resPacket + packet_length++) = STATUS_RESPONSE;
        packet_length += ShimBt_assembleStatusBytes(&resPacket[packet_length]);
        break;
      }
      case GET_VBATT_COMMAND:
      {
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
      }
      case GET_TRIAL_CONFIG_COMMAND:
      {
        *(resPacket + packet_length++) = TRIAL_CONFIG_RESPONSE;
        //2 trial config bytes + 1 interval byte
        ShimConfig_storedConfigGet(&resPacket[packet_length], NV_SD_TRIAL_CONFIG0, 3);
        packet_length += 3;
        break;
      }
      case GET_CENTER_COMMAND:
      {
        *(resPacket + packet_length++) = CENTER_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->masterEnable;
        break;
      }
      case GET_SHIMMERNAME_COMMAND:
      {
        char *shimmerNameTextPtr = ShimConfig_shimmerNameParseToTxtAndPtrGet();
        uint8_t shimmer_name_len = strlen(shimmerNameTextPtr);
        *(resPacket + packet_length++) = SHIMMERNAME_RESPONSE;
        *(resPacket + packet_length++) = shimmer_name_len;
        memcpy((resPacket + packet_length), shimmerNameTextPtr, shimmer_name_len);
        packet_length += shimmer_name_len;
        break;
      }
      case GET_EXPID_COMMAND:
      {
        char *expIdTextPtr = ShimConfig_expIdParseToTxtAndPtrGet();
        uint8_t exp_id_name_len = strlen((char *) expIdTextPtr);
        *(resPacket + packet_length++) = EXPID_RESPONSE;
        *(resPacket + packet_length++) = exp_id_name_len;
        memcpy((resPacket + packet_length), expIdTextPtr, exp_id_name_len);
        packet_length += exp_id_name_len;
        break;
      }
      case GET_CONFIGTIME_COMMAND:
      {
        char *configTimeTextPtr = ShimConfig_configTimeParseToTxtAndPtrGet();
        uint8_t cfgtime_name_len = strlen(configTimeTextPtr);
        *(resPacket + packet_length++) = CONFIGTIME_RESPONSE;
        *(resPacket + packet_length++) = cfgtime_name_len;
        memcpy((resPacket + packet_length), configTimeTextPtr, cfgtime_name_len);
        packet_length += cfgtime_name_len;
        break;
      }
      case GET_DIR_COMMAND:
      {
        fileNamePtr = ShimSdDataFile_fileNamePtrGet();
        uint8_t dir_len = strlen((char *) fileNamePtr) - 3;
        *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
        *(resPacket + packet_length++) = DIR_RESPONSE;
        *(resPacket + packet_length++) = dir_len;
        memcpy((resPacket + packet_length), fileNamePtr, dir_len);
        packet_length += dir_len;
        break;
      }
      case GET_NSHIMMER_COMMAND:
      {
        *(resPacket + packet_length++) = NSHIMMER_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->numberOfShimmers;
        break;
      }
      case GET_MYID_COMMAND:
      {
        *(resPacket + packet_length++) = MYID_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->myTrialID;
        break;
      }
      case GET_WR_ACCEL_SAMPLING_RATE_COMMAND:
      {
        *(resPacket + packet_length++) = WR_ACCEL_SAMPLING_RATE_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->wrAccelRate;
        break;
      }
      case GET_WR_ACCEL_LPMODE_COMMAND:
      {
        *(resPacket + packet_length++) = WR_ACCEL_LPMODE_RESPONSE;
        *(resPacket + packet_length++) = ShimConfig_wrAccelLpModeGet();
        break;
      }
      case GET_WR_ACCEL_HRMODE_COMMAND:
      {
        *(resPacket + packet_length++) = WR_ACCEL_HRMODE_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->wrAccelHrMode;
        break;
      }
      case GET_GYRO_RANGE_COMMAND:
      {
        *(resPacket + packet_length++) = GYRO_RANGE_RESPONSE;
        *(resPacket + packet_length++) = ShimConfig_gyroRangeGet();
        break;
        case GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND:
#if defined(SHIMMER3)
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
#elif defined(SHIMMER3R)
          sendNack = 1;
#endif
          break;
      }
      case GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND:
      {
#if defined(SHIMMER3)
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
#elif defined(SHIMMER3R)
        sendNack = 1;
#endif
        break;
      }
      case GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND:
      {
        bmpCalibByteLen = get_bmp_calib_data_bytes_len();
        *(resPacket + packet_length++) = PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE;
        *(resPacket + packet_length++) = 1U + bmpCalibByteLen;
#if defined(SHIMMER3)
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
#elif defined(SHIMMER3R)
        *(resPacket + packet_length++) = PRESSURE_SENSOR_BMP390;
#endif
        memcpy(resPacket + packet_length, get_bmp_calib_data_bytes(), bmpCalibByteLen);
        packet_length += bmpCalibByteLen;
        break;
      }
      case GET_GYRO_SAMPLING_RATE_COMMAND:
      {
        *(resPacket + packet_length++) = GYRO_SAMPLING_RATE_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->gyroRate;
        break;
      }
      case GET_ALT_ACCEL_RANGE_COMMAND:
      {
#if defined(SHIMMER3)
        *(resPacket + packet_length++) = ALT_ACCEL_RANGE_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->altAccelRange;
#elif defined(SHIMMER3R)
        *(resPacket + packet_length++) = ALT_ACCEL_RANGE_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->lnAccelRange;
#endif
        break;
      }
      case GET_PRESSURE_OVERSAMPLING_RATIO_COMMAND:
      {
        *(resPacket + packet_length++) = PRESSURE_OVERSAMPLING_RATIO_RESPONSE;
        *(resPacket + packet_length++)
            = ShimConfig_configBytePressureOversamplingRatioGet();
        break;
      }
      case GET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
      {
        *(resPacket + packet_length++) = INTERNAL_EXP_POWER_ENABLE_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->expansionBoardPower;
        break;
      }
      case GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND:
      {
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
      }
      case GET_CONFIG_SETUP_BYTES_COMMAND:
      {
        *(resPacket + packet_length++) = CONFIG_SETUP_BYTES_RESPONSE;
        memcpy(resPacket + packet_length,
            &storedConfig->rawBytes[NV_CONFIG_SETUP_BYTE0], 4);
        packet_length += 4;
        break;
      }
      case GET_BT_VERSION_STR_COMMAND:
      {
        btVerStrLen = ShimBt_getBtVerStrLen();
        *(resPacket + packet_length++) = BT_VERSION_STR_RESPONSE;
        *(resPacket + packet_length++) = btVerStrLen;
        memcpy((resPacket + packet_length), ShimBt_getBtVerStrPtr(), btVerStrLen);
        packet_length += btVerStrLen;
        break;
      }
      case GET_GSR_RANGE_COMMAND:
      {
        *(resPacket + packet_length++) = GSR_RANGE_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->gsrRange;
        break;
      }
      case DEPRECATED_GET_DEVICE_VERSION_COMMAND:
      case GET_DEVICE_VERSION_COMMAND:
      {
        *(resPacket + packet_length++) = DEVICE_VERSION_RESPONSE;
        *(resPacket + packet_length++) = DEVICE_VER;
        break;
      }
      case GET_FW_VERSION_COMMAND:
      {
        *(resPacket + packet_length++) = FW_VERSION_RESPONSE;
        *(resPacket + packet_length++) = FW_IDENTIFIER & 0xFF;
        *(resPacket + packet_length++) = (FW_IDENTIFIER & 0xFF00) >> 8;
        *(resPacket + packet_length++) = FW_VERSION_MAJOR & 0xFF;
        *(resPacket + packet_length++) = (FW_VERSION_MAJOR & 0xFF00) >> 8;
        *(resPacket + packet_length++) = FW_VERSION_MINOR;
        *(resPacket + packet_length++) = FW_VERSION_PATCH;
        break;
      }
      case GET_CHARGE_STATUS_LED_COMMAND:
      {
        *(resPacket + packet_length++) = CHARGE_STATUS_LED_RESPONSE;
        *(resPacket + packet_length++) = batteryStatus.battStat;
        break;
      }
      case GET_BUFFER_SIZE_COMMAND:
      {
        *(resPacket + packet_length++) = BUFFER_SIZE_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->bufferSize;
        break;
      }
      case GET_UNIQUE_SERIAL_COMMAND:
      {
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
      }
      case GET_BT_COMMS_BAUD_RATE:
      {
        *(resPacket + packet_length++) = BT_COMMS_BAUD_RATE_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->btCommsBaudRate;
        break;
      }
      case GET_DERIVED_CHANNEL_BYTES:
      {
        *(resPacket + packet_length++) = DERIVED_CHANNEL_BYTES_RESPONSE;
        ShimConfig_storedConfigGet(&resPacket[packet_length], NV_DERIVED_CHANNELS_0, 3);
        packet_length += 3;
        ShimConfig_storedConfigGet(&resPacket[packet_length], NV_DERIVED_CHANNELS_3, 5);
        packet_length += 5;
        break;
      }
      case GET_RWC_COMMAND:
      {
        *(resPacket + packet_length++) = RWC_RESPONSE;

        uint64_t rwc_curr_time_64 = RTC_getRwcTime();
        memcpy(resPacket + packet_length, (uint8_t *) (&rwc_curr_time_64), 8);
        packet_length += 8;

        break;
      }
      case GET_ALL_CALIBRATION_COMMAND:
      {
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
      }
      case GET_LN_ACCEL_CALIBRATION_COMMAND:
      case GET_GYRO_CALIBRATION_COMMAND:
      case GET_MAG_CALIBRATION_COMMAND:
      case GET_WR_ACCEL_CALIBRATION_COMMAND:
      case GET_ALT_ACCEL_CALIBRATION_COMMAND:
      case GET_ALT_MAG_CALIBRATION_COMMAND:
      {
        *(resPacket + packet_length++)
            = ShimBt_getExpectedRspForGetCmd(getCmdWaitingResponse);
        packet_length += ShimBt_replySingleSensorCalibCmd(
            getCmdWaitingResponse, &resPacket[packet_length]);
        break;
      }
      case GET_ALT_ACCEL_SAMPLING_RATE_COMMAND:
      {
        *(resPacket + packet_length++) = ALT_ACCEL_SAMPLING_RATE_RESPONSE;
        *(resPacket + packet_length++) = storedConfig->altAccelRate;
        break;
      }
      case GET_ALT_MAG_SAMPLING_RATE_COMMAND:
      {
        *(resPacket + packet_length++) = ALT_MAG_SAMPLING_RATE_RESPONSE;
        *(resPacket + packet_length++) = ShimConfig_configByteAltMagRateGet();
        break;
      }
      case GET_EXG_REGS_COMMAND:
      {
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
      }
      case GET_CALIB_DUMP_COMMAND:
      {
        *(resPacket + packet_length++) = RSP_CALIB_DUMP_COMMAND;
        *(resPacket + packet_length++) = calibRamLength;
        *(resPacket + packet_length++) = calibRamOffset & 0xff;
        *(resPacket + packet_length++) = (calibRamOffset >> 8) & 0xff;
        ShimCalib_ramRead(resPacket + packet_length, calibRamLength, calibRamOffset);
        packet_length += calibRamLength;
        break;
      }
      case SET_DATA_RATE_TEST:
      {
        /* Start test after ACK is sent - this will be handled by the
         * interrupt after ACK byte is transmitted */
        if (args[0] != 0)
        {
          ShimBt_setDataRateTestState(1);
        }
        break;
      }
      case GET_DAUGHTER_CARD_ID_COMMAND:
      {
        *(resPacket + packet_length++) = DAUGHTER_CARD_ID_RESPONSE;
        *(resPacket + packet_length++) = dcMemLength;
        // Read from the cached and processed daughterCardIdPage
        memcpy(resPacket + packet_length,
            ShimBrd_getDaughtCardIdPtr() + dcMemOffset, dcMemLength);
        packet_length += dcMemLength;
        break;
      }
      case GET_DAUGHTER_CARD_MEM_COMMAND:
      {
        *(resPacket + packet_length++) = DAUGHTER_CARD_MEM_RESPONSE;
        *(resPacket + packet_length++) = dcMemLength;
        eepromRead(dcMemOffset + 16U, dcMemLength, resPacket + packet_length);
        packet_length += dcMemLength;
        break;
      }
      case GET_INFOMEM_COMMAND:
      {
        *(resPacket + packet_length++) = INFOMEM_RESPONSE;
        *(resPacket + packet_length++) = infomemLength;
        ShimConfig_storedConfigGet(&resPacket[packet_length], infomemOffset, infomemLength);
        packet_length += infomemLength;
        break;
      }
#if defined(SHIMMER4_SDK)
      case GET_I2C_BATT_STATUS_COMMAND:
      {
        *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
        *(resPacket + packet_length++) = RSP_I2C_BATT_STATUS_COMMAND;
        memcpy((resPacket + packet_length),
            (uint8_t *) shimmerStatus.battDigital, STC3100_DATA_LEN);
        packet_length += STC3100_DATA_LEN;
        break
      }
#endif

      default:
      {
        break;
      }
    }
    getCmdWaitingResponse = 0;

    uint8_t crcMode = ShimBt_getCrcMode();
    if (crcMode != CRC_OFF)
    {
      calculateCrcAndInsert(crcMode, resPacket, packet_length);
      packet_length += crcMode;
    }
    ShimBt_writeToTxBufAndSend(resPacket, packet_length, SHIMMER_CMD);
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

void ShimBt_macIdSetFromStr(uint8_t *macIdStrMsbOrder)
{
  ShimBt_macIdVarsReset();
  memcpy(macIdStr, macIdStrMsbOrder, 12);
  uint8_t i, pchar[3];
  pchar[2] = 0;
  for (i = 0; i < 6; i++)
  {
    pchar[0] = macIdStr[i * 2];
    pchar[1] = macIdStr[i * 2 + 1];
    macIdBytes[i] = strtoul((char *) pchar, 0, 16);
  }
}

void ShimBt_macIdSetFromBytes(uint8_t *macIdBytesLsbOrder)
{
  ShimBt_macIdVarsReset();
  memcpy(&macIdBytes[0], macIdBytesLsbOrder, sizeof(macIdBytes));

  ShimUtil_reverseArray(&macIdBytes[0], sizeof(macIdBytes));

  (void) sprintf(macIdStr, "%02X%02X%02X%02X%02X%02X", macIdBytes[0],
      macIdBytes[1], macIdBytes[2], macIdBytes[3], macIdBytes[4], macIdBytes[5]);
}

char *ShimBt_macIdStrPtrGet(void)
{
  return &macIdStr[0];
}

uint8_t *ShimBt_macIdBytesPtrGet(void)
{
  return &macIdBytes[0];
}

void ShimBt_macIdVarsReset(void)
{
  memset(macIdStr, 0x00, sizeof(macIdStr) / sizeof(macIdStr[0]));
  memset(macIdBytes, 0x00, sizeof(macIdBytes) / sizeof(macIdBytes[0]));
}

void ShimBt_instreamStatusRespSendIfNotBtCmd(void)
{
  /* If sensing state changed due to BT command (e.g., start/stop sensing),
   * don't send instream status response. Only send it if it was triggered by a
   * hardware action like dock/undock, button press, trial duration or low
   * battery. */
  if (!sensingStateChangeFromBtCmd)
  {
    ShimBt_instreamStatusRespSend();
  }
  /* Assumption: Sensing operations are processed sequentially.
   * If this changes, ensure the flag is only reset after all pending operations complete. */
  sensingStateChangeFromBtCmd = 0;
}

void ShimBt_instreamStatusRespSend(void)
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
    i += ShimBt_assembleStatusBytes(&selfcmd[i]);

    uint8_t crcMode = ShimBt_getCrcMode();
    if (crcMode != CRC_OFF)
    {
      calculateCrcAndInsert(crcMode, &selfcmd[0], i);
      i += crcMode; //Ordinal of enum is how many bytes are used
    }

    ShimBt_writeToTxBufAndSend(selfcmd, i, SHIMMER_CMD);
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
    ShimTask_setStopStreaming();

    ShimBt_setDataRateTestState(0);

    ShimBt_clearBtTxBuf(0);

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

void ShimBt_clearBtTxBuf(uint8_t isCalledFromMain)
{
  //uint16_t i;
  /* We don't want to be clearing the TX buffer if main is in the middle to
   * streaming bytes to it */
  if (isCalledFromMain)
  {
    RINGFIFO_RESET(gBtTxFifo);

    //Reset all bytes in the buffer -> only used during debugging
    //memset(gBtTxFifo.data, 0x00, sizeof(gBtTxFifo.data) /
    //sizeof(gBtTxFifo.data[0])); for(i=BT_TX_BUF_SIZE-1;i<BT_TX_BUF_SIZE;i--)
    //{
    //*(&gBtTxFifo.data[0]+i) = 0xFF;
    //}

    ShimBt_btTxInProgressSet(0);
  }
  else
  {
    ShimTask_set(TASK_BT_TX_BUF_CLEAR);
  }
}

uint8_t ShimBt_isBtTxBufEmpty(void)
{
  return RINGFIFO_EMPTY(gBtTxFifo);
}

void ShimBt_pushByteToBtTxBuf(uint8_t c)
{
  if (!RINGFIFO_FULL(gBtTxFifo, BT_TX_BUF_MASK))
  {
    RINGFIFO_WR(gBtTxFifo, c, BT_TX_BUF_MASK);
  }
}

void ShimBt_pushBytesToBtTxBuf(uint8_t *buf, uint8_t len)
{
  uint8_t i;
  for (i = 0; i < len; i++)
  {
    ShimBt_pushByteToBtTxBuf(*(buf + i));
  }

  ///* if enough space at after head, copy it in */
  //uint16_t spaceAfterHead = BT_TX_BUF_SIZE - (gBtTxFifo.wrIdx &
  //BT_TX_BUF_MASK); if (spaceAfterHead > len)
  //{
  //  ShimUtil_memcpy_v(&gBtTxFifo.data[(gBtTxFifo.wrIdx & BT_TX_BUF_MASK)],
  //  buf, len); gBtTxFifo.wrIdx += len;
  //}
  //else
  //{
  //  /* Fill from head to end of buf */
  //  ShimUtil_memcpy_v(&gBtTxFifo.data[(gBtTxFifo.wrIdx & BT_TX_BUF_MASK)],
  //  buf, spaceAfterHead); gBtTxFifo.wrIdx += spaceAfterHead;
  //
  //  /* Fill from start of buf. We already checked above whether there is
  //   * enough space in the buf (getSpaceInBtTxBuf()) so we don't need to
  //   * worry about the tail position. */
  //  uint16_t remaining = len - spaceAfterHead;
  //  ShimUtil_memcpy_v(&gBtTxFifo.data[(gBtTxFifo.wrIdx & BT_TX_BUF_MASK)],
  //      buf + spaceAfterHead, remaining);
  //  gBtTxFifo.wrIdx += remaining;
  //}
}

uint8_t ShimBt_popBytefromBtTxBuf(void)
{
  uint8_t txByte = 0;
  RINGFIFO_RD(gBtTxFifo, txByte, BT_TX_BUF_MASK);
  return txByte;
}

uint16_t ShimBt_getUsedSpaceInBtTxBuf(void)
{
  return RINGFIFO_COUNT(gBtTxFifo, BT_TX_BUF_MASK);
}

uint16_t ShimBt_getSpaceInBtTxBuf(void)
{
  //Minus 1 as we always need to leave 1 empty byte in the rolling buffer
  return BT_TX_BUF_SIZE - 1 - ShimBt_getUsedSpaceInBtTxBuf();
}

void ShimBt_TxCpltCallback(void)
{
#if !defined(SHIMMER3)
  /* Advance the fifo buffer read index after TX is complete */
  gBtTxFifo.rdIdx += gBtTxFifo.numBytesBeingRead;
  gBtTxFifo.numBytesBeingRead = 0;
#endif

  if (shimmerStatus.btConnected
#if defined(SHIMMER3)
      || areBtSetupCommandsRunning())
#else
  )
#endif
  {
    if (btDataRateTestState)
    {
      ShimBt_loadTxBufForDataRateTest();
    }
    else
    {
      ShimBt_sendNextChar();
    }
  }
  else
  {
    ShimBt_clearBtTxBuf(0);
  }
}

void ShimBt_sendNextCharIfNotInProgress(void)
{
  if (!ShimBt_btTxInProgressGet())
  {
    ShimBt_sendNextChar();
  }
}

void ShimBt_sendNextChar(void)
{
  if (!ShimBt_isBtTxBufEmpty()
#if defined(SHIMMER3)
#if BT_FLUSH_TX_BUF_IF_RN4678_RTS_LOCK_DETECTED
            && (rn4678RtsLockDetected || !isBtModuleOverflowPinHigh())
#else
      && !isBtModuleOverflowPinHigh())
#endif
#else
  )
#endif
  {
    ShimBt_btTxInProgressSet(1);

#if defined(SHIMMER3)
    /* Shimmer3 sends 1 byte at a time */
    uint8_t buf = ShimBt_popBytefromBtTxBuf();
    BtTransmit(&buf, 1);
#else
    HAL_StatusTypeDefShimmer ret_val;
    uint8_t numBytes;

    uint8_t rdIdx = (gBtTxFifo.rdIdx & BT_TX_BUF_MASK);
    uint8_t wrIdx = (gBtTxFifo.wrIdx & BT_TX_BUF_MASK);

    if (rdIdx < wrIdx)
    {
      numBytes = wrIdx - rdIdx;
    }
    else
    {
      numBytes = BT_TX_BUF_SIZE - rdIdx;
    }
    gBtTxFifo.numBytesBeingRead = numBytes;
    ret_val = BtTransmit((uint8_t *) &gBtTxFifo.data[rdIdx], numBytes);
#endif
  }
  else
  {
    ShimBt_btTxInProgressSet(0); //false
  }
}

void ShimBt_btTxInProgressSet(uint8_t state)
{
  btTxInProgress = state;
}

uint8_t ShimBt_btTxInProgressGet(void)
{
  return btTxInProgress;
}

void ShimBt_setDataRateTestState(uint8_t state)
{
  btDataRateTestState = state;
#if defined(SHIMMER3)
  btDataRateTestCounter = 0;
#else
  *((uint32_t *) &dataRateTestTxPacket[1]) = 0;
#endif
}

uint8_t ShimBt_getDataRateTestState(void)
{
  return btDataRateTestState;
}

void ShimBt_loadTxBufForDataRateTest(void)
{
#if defined(SHIMMER3)
  uint16_t spaceInTxBuf = ShimBt_getSpaceInBtTxBuf();
  if (spaceInTxBuf > DATA_RATE_TEST_PACKET_SIZE)
  {
    ShimBt_pushByteToBtTxBuf(DATA_RATE_TEST_RESPONSE);
    ShimBt_pushBytesToBtTxBuf(
        (uint8_t *) &btDataRateTestCounter, sizeof(btDataRateTestCounter));
    btDataRateTestCounter++;
  }
  ShimBt_sendNextChar();
#else
  HAL_StatusTypeDefShimmer ret_val
      = BtTransmit(&dataRateTestTxPacket[0], sizeof(dataRateTestTxPacket));
  (*((uint32_t *) &dataRateTestTxPacket[1]))++;
#endif
}

#if defined(SHIMMER3R)
uint8_t ShimBt_writeToTxBufAndSend(uint8_t *buf, uint8_t len, btResponseType responseType)
{
  if (ShimBt_getSpaceInBtTxBuf() <= len)
  {
    return 1; //fail
  }

  ShimBt_pushBytesToBtTxBuf(buf, len);

  ShimBt_sendNextCharIfNotInProgress();

  return 0;
}
#endif

uint8_t ShimBt_assembleStatusBytes(uint8_t *bufPtr)
{
  uint8_t statusByteCnt = 1;
  *(bufPtr) = (shimmerStatus.toggleLedRedCmd << 7) | (shimmerStatus.sdBadFile << 6)
      | (shimmerStatus.sdInserted << 5) | (shimmerStatus.btStreaming << 4)
      | (shimmerStatus.sdLogging << 3) | (RTC_isRwcTimeSet() << 2)
      | (shimmerStatus.sensing << 1) | shimmerStatus.docked;

#if defined(SHIMMER3R)
  *(bufPtr + 1) = shimmerStatus.usbPluggedIn;
  statusByteCnt++;
#endif /* SHIMMER3R */

  return statusByteCnt;
}

uint8_t ShimBt_isCmdAllowedWhileSdSyncing(uint8_t command)
{
  return (command == SET_SD_SYNC_COMMAND || command == ACK_COMMAND_PROCESSED);
}

uint8_t ShimBt_isCmdBlockedWhileSensing(uint8_t command)
{
  switch (command)
  {
    case SET_SAMPLING_RATE_COMMAND:               //0x05
    case SET_SENSORS_COMMAND:                     //0x08
    case SET_WR_ACCEL_RANGE_COMMAND:              //0x09
    case SET_CONFIG_SETUP_BYTES_COMMAND:          //0x0E
    case SET_LN_ACCEL_CALIBRATION_COMMAND:        //0x11
    case SET_GYRO_CALIBRATION_COMMAND:            //0x14
    case SET_MAG_CALIBRATION_COMMAND:             //0x17
    case SET_WR_ACCEL_CALIBRATION_COMMAND:        //0x1A
    case SET_GSR_RANGE_COMMAND:                   //0x21
    case SET_MAG_GAIN_COMMAND:                    //0x37
    case SET_MAG_SAMPLING_RATE_COMMAND:           //0x3A
    case SET_WR_ACCEL_SAMPLING_RATE_COMMAND:      //0x40
    case SET_WR_ACCEL_LPMODE_COMMAND:             //0x43
    case SET_WR_ACCEL_HRMODE_COMMAND:             //0x46
    case SET_GYRO_RANGE_COMMAND:                  //0x49
    case SET_GYRO_SAMPLING_RATE_COMMAND:          //0x4C
    case SET_ALT_ACCEL_RANGE_COMMAND:             //0x4F
    case SET_PRESSURE_OVERSAMPLING_RATIO_COMMAND: //0x52

    case RESET_TO_DEFAULT_CONFIGURATION_COMMAND: //0x5A
    case RESET_CALIBRATION_VALUE_COMMAND:        //0x5B

    case SET_INTERNAL_EXP_POWER_ENABLE_COMMAND: //0x5E
    case SET_EXG_REGS_COMMAND:                  //0x61
    case SET_DAUGHTER_CARD_ID_COMMAND:          //0x64
    case SET_DAUGHTER_CARD_MEM_COMMAND:         //0x67
    case SET_BT_COMMS_BAUD_RATE:                //0x6A
    case SET_DERIVED_CHANNEL_BYTES:             //0x6D
    case SET_TRIAL_CONFIG_COMMAND:              //0x73
    case SET_CENTER_COMMAND:                    //0x76
    case SET_SHIMMERNAME_COMMAND:               //0x79
    case SET_EXPID_COMMAND:                     //0x7C
    case SET_MYID_COMMAND:                      //0x7F
    case SET_NSHIMMER_COMMAND:                  //0x82
    case SET_CONFIGTIME_COMMAND:                //0x85
    case SET_INFOMEM_COMMAND:                   //0x8C
    case SET_CALIB_DUMP_COMMAND:                //0x98

    case UPD_CALIB_DUMP_COMMAND: //0x9B
    case UPD_SDLOG_CFG_COMMAND:  //0x9C

    case SET_DATA_RATE_TEST:                  //0xA4
    case SET_FACTORY_TEST:                    //0xA8
    case SET_ALT_ACCEL_CALIBRATION_COMMAND:   //0xA9
    case SET_ALT_ACCEL_SAMPLING_RATE_COMMAND: //0xAC
    case SET_ALT_MAG_CALIBRATION_COMMAND:     //0xAF
    case SET_ALT_MAG_SAMPLING_RATE_COMMAND:   //0xB2

      return 1;
    default:
      return 0;
  }
}

void ShimBt_setBtBaudRateToUse(uint32_t baudRate)
{
#if defined(SHIMMER3)
  if (baudRate <= BAUD_1000000)
  {
    //RN4678 doesn't support 1200, overwrite it with the next highest value
    if (isBtDeviceRn4678() && baudRate == BAUD_1200)
    {
      btBaudRateToUse = BAUD_2400;
    }
    else
    {
      btBaudRateToUse = baudRate;
    }
  }
#else
  btBaudRateToUse = baudRate;
#endif
}

uint32_t ShimBt_getBtBaudRateToUse(void)
{
  return btBaudRateToUse;
}

void ShimBt_setBtMode(uint8_t btClassicEn, uint8_t bleEn)
{
  btClassicCurrentlyEnabled = btClassicEn;
  bleCurrentlyEnabled = bleEn;
  BT_setBtMode(btClassicEn, bleEn);
}

__weak void BT_setBtMode(uint8_t btClassicEn, uint8_t bleEn)
{
  /* Implement in board file if needed */
  (void) btClassicEn;
  (void) bleEn;
}

uint8_t ShimBt_isBleCurrentlyEnabled(void)
{
  return bleCurrentlyEnabled;
}

uint8_t ShimBt_isBtClassicCurrentlyEnabled(void)
{
  return btClassicCurrentlyEnabled;
}
