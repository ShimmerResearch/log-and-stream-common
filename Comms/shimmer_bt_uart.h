/*
 * shimmer_bt_comms.h
 *
 *  Created on: 22 Jun 2022
 *      Author: Mark Nolan
 */

#ifndef SHIMMER3_COMMON_SOURCE_BLUETOOTH_SD_SHIMMER_BT_COMMS_H_
#define SHIMMER3_COMMON_SOURCE_BLUETOOTH_SD_SHIMMER_BT_COMMS_H_

#include <stdint.h>

#if defined(SHIMMER3)
#include "../../shimmer_btsd.h"
#include "../5xx_HAL/hal_CRC.h"
#include "../RN4X/RN4678.h"
#elif defined(SHIMMER3R)
#include "hal_CRC.h"
#include "shimmer_definitions.h"
#include <shimmer_include.h>
#endif

#define BT_TX_BUF_SIZE                              256U /* serial buffer in bytes (power 2)  */
#define BT_TX_BUF_MASK                              (BT_TX_BUF_SIZE - 1UL)

/* maximum number of arguments for any command sent (daughter card mem write) */
#define MAX_COMMAND_ARG_SIZE                        131

//Packet Types
#define DATA_PACKET                                 0x00
#define INQUIRY_COMMAND                             0x01
#define INQUIRY_RESPONSE                            0x02
#define GET_SAMPLING_RATE_COMMAND                   0x03
#define SAMPLING_RATE_RESPONSE                      0x04
#define SET_SAMPLING_RATE_COMMAND                   0x05
#define TOGGLE_LED_COMMAND                          0x06
#define START_STREAMING_COMMAND                     0x07
#define SET_SENSORS_COMMAND                         0x08
#define SET_WR_ACCEL_RANGE_COMMAND                  0x09
#define WR_ACCEL_RANGE_RESPONSE                     0x0A
#define GET_WR_ACCEL_RANGE_COMMAND                  0x0B
#define SET_CONFIG_SETUP_BYTES_COMMAND              0x0E
#define CONFIG_SETUP_BYTES_RESPONSE                 0x0F
#define GET_CONFIG_SETUP_BYTES_COMMAND              0x10
#define SET_LN_ACCEL_CALIBRATION_COMMAND            0x11
#define LN_ACCEL_CALIBRATION_RESPONSE               0x12
#define GET_LN_ACCEL_CALIBRATION_COMMAND            0x13
#define SET_GYRO_CALIBRATION_COMMAND                0x14
#define GYRO_CALIBRATION_RESPONSE                   0x15
#define GET_GYRO_CALIBRATION_COMMAND                0x16
#define SET_MAG_CALIBRATION_COMMAND                 0x17
#define MAG_CALIBRATION_RESPONSE                    0x18
#define GET_MAG_CALIBRATION_COMMAND                 0x19
#define SET_WR_ACCEL_CALIBRATION_COMMAND            0x1A
#define WR_ACCEL_CALIBRATION_RESPONSE               0x1B
#define GET_WR_ACCEL_CALIBRATION_COMMAND            0x1C
#define STOP_STREAMING_COMMAND                      0x20
#define SET_GSR_RANGE_COMMAND                       0x21
#define GSR_RANGE_RESPONSE                          0x22
#define GET_GSR_RANGE_COMMAND                       0x23
/* deprecated because 0x24 ('$' ASCII) as a command is problematic if remote
 * config is enabled in RN42 Bluetooth module. Replaced with 0x3F command */
#define DEPRECATED_GET_DEVICE_VERSION_COMMAND       0x24
#define DEVICE_VERSION_RESPONSE                     0x25
#define GET_ALL_CALIBRATION_COMMAND                 0x2C
#define ALL_CALIBRATION_RESPONSE                    0x2D
#define GET_FW_VERSION_COMMAND                      0x2E
#define FW_VERSION_RESPONSE                         0x2F
#define SET_CHARGE_STATUS_LED_COMMAND               0x30
#define CHARGE_STATUS_LED_RESPONSE                  0x31
#define GET_CHARGE_STATUS_LED_COMMAND               0x32
#define BUFFER_SIZE_RESPONSE                        0x35
#define GET_BUFFER_SIZE_COMMAND                     0x36
#define SET_MAG_GAIN_COMMAND                        0x37
#define MAG_GAIN_RESPONSE                           0x38
#define GET_MAG_GAIN_COMMAND                        0x39
#define SET_MAG_SAMPLING_RATE_COMMAND               0x3A
#define MAG_SAMPLING_RATE_RESPONSE                  0x3B
#define GET_MAG_SAMPLING_RATE_COMMAND               0x3C
#define UNIQUE_SERIAL_RESPONSE                      0x3D
#define GET_UNIQUE_SERIAL_COMMAND                   0x3E
#define GET_DEVICE_VERSION_COMMAND                  0x3F
#define SET_WR_ACCEL_SAMPLING_RATE_COMMAND          0x40
#define WR_ACCEL_SAMPLING_RATE_RESPONSE             0x41
#define GET_WR_ACCEL_SAMPLING_RATE_COMMAND          0x42
#define SET_WR_ACCEL_LPMODE_COMMAND                 0x43
#define WR_ACCEL_LPMODE_RESPONSE                    0x44
#define GET_WR_ACCEL_LPMODE_COMMAND                 0x45
#define SET_WR_ACCEL_HRMODE_COMMAND                 0x46
#define WR_ACCEL_HRMODE_RESPONSE                    0x47
#define GET_WR_ACCEL_HRMODE_COMMAND                 0x48
#define SET_GYRO_RANGE_COMMAND                      0x49
#define GYRO_RANGE_RESPONSE                         0x4A
#define GET_GYRO_RANGE_COMMAND                      0x4B
#define SET_GYRO_SAMPLING_RATE_COMMAND              0x4C
#define GYRO_SAMPLING_RATE_RESPONSE                 0x4D
#define GET_GYRO_SAMPLING_RATE_COMMAND              0x4E
#define SET_ALT_ACCEL_RANGE_COMMAND                 0x4F
#define ALT_ACCEL_RANGE_RESPONSE                    0x50
#define GET_ALT_ACCEL_RANGE_COMMAND                 0x51
#define SET_PRESSURE_OVERSAMPLING_RATIO_COMMAND     0x52
#define PRESSURE_OVERSAMPLING_RATIO_RESPONSE        0x53
#define GET_PRESSURE_OVERSAMPLING_RATIO_COMMAND     0x54
#define BMP180_CALIBRATION_COEFFICIENTS_RESPONSE    0x58
#define GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND 0x59
#define RESET_TO_DEFAULT_CONFIGURATION_COMMAND      0x5A
#define RESET_CALIBRATION_VALUE_COMMAND             0x5B
#define MPU9150_MAG_SENS_ADJ_VALS_RESPONSE          0x5C
#define GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND       0x5D
#define SET_INTERNAL_EXP_POWER_ENABLE_COMMAND       0x5E
#define INTERNAL_EXP_POWER_ENABLE_RESPONSE          0x5F
#define GET_INTERNAL_EXP_POWER_ENABLE_COMMAND       0x60
#define SET_EXG_REGS_COMMAND                        0x61
#define EXG_REGS_RESPONSE                           0x62
#define GET_EXG_REGS_COMMAND                        0x63
#define SET_DAUGHTER_CARD_ID_COMMAND                0x64
#define DAUGHTER_CARD_ID_RESPONSE                   0x65
#define GET_DAUGHTER_CARD_ID_COMMAND                0x66
#define SET_DAUGHTER_CARD_MEM_COMMAND               0x67
#define DAUGHTER_CARD_MEM_RESPONSE                  0x68
#define GET_DAUGHTER_CARD_MEM_COMMAND               0x69
/* DEPRECATED: 11 allowable options: 0=115.2K(default), 1=1200, 2=2400, 3=4800,
 * 4=9600, 5=19.2K, 6=38.4K, 7=57.6K, 8=230.4K, 9=460.8K, 10=921.6K Need to
 * disconnect BT connection before change is active */
#define SET_BT_COMMS_BAUD_RATE                      0x6A
#define BT_COMMS_BAUD_RATE_RESPONSE                 0x6B
#define GET_BT_COMMS_BAUD_RATE                      0x6C
#define SET_DERIVED_CHANNEL_BYTES                   0x6D
#define DERIVED_CHANNEL_BYTES_RESPONSE              0x6E
#define GET_DERIVED_CHANNEL_BYTES                   0x6F
#define START_SDBT_COMMAND                          0x70
#define STATUS_RESPONSE                             0x71
#define GET_STATUS_COMMAND                          0x72
#define SET_TRIAL_CONFIG_COMMAND                    0x73
#define TRIAL_CONFIG_RESPONSE                       0x74
#define GET_TRIAL_CONFIG_COMMAND                    0x75
#define SET_CENTER_COMMAND                          0x76
#define CENTER_RESPONSE                             0x77
#define GET_CENTER_COMMAND                          0x78
#define SET_SHIMMERNAME_COMMAND                     0x79
#define SHIMMERNAME_RESPONSE                        0x7a
#define GET_SHIMMERNAME_COMMAND                     0x7b
#define SET_EXPID_COMMAND                           0x7c
#define EXPID_RESPONSE                              0x7d
#define GET_EXPID_COMMAND                           0x7e
#define SET_MYID_COMMAND                            0x7F
#define MYID_RESPONSE                               0x80
#define GET_MYID_COMMAND                            0x81
#define SET_NSHIMMER_COMMAND                        0x82
#define NSHIMMER_RESPONSE                           0x83
#define GET_NSHIMMER_COMMAND                        0x84
#define SET_CONFIGTIME_COMMAND                      0x85
#define CONFIGTIME_RESPONSE                         0x86
#define GET_CONFIGTIME_COMMAND                      0x87
#define DIR_RESPONSE                                0x88
#define GET_DIR_COMMAND                             0x89
#define INSTREAM_CMD_RESPONSE                       0x8A
#define SET_CRC_COMMAND                             0x8B
#define SET_INFOMEM_COMMAND                         0x8C
#define INFOMEM_RESPONSE                            0x8D
#define GET_INFOMEM_COMMAND                         0x8E
#define SET_RWC_COMMAND                             0x8F
#define RWC_RESPONSE                                0x90
#define GET_RWC_COMMAND                             0x91
#define START_LOGGING_COMMAND                       0x92
#define STOP_LOGGING_COMMAND                        0x93
#define VBATT_RESPONSE                              0x94
#define GET_VBATT_COMMAND                           0x95
#define DUMMY_COMMAND                               0x96
#define STOP_SDBT_COMMAND                           0x97
#define SET_CALIB_DUMP_COMMAND                      0x98
#define RSP_CALIB_DUMP_COMMAND                      0x99
#define GET_CALIB_DUMP_COMMAND                      0x9A
#define UPD_CALIB_DUMP_COMMAND                      0x9B
//#define UPD_FLASH_COMMAND                             0x9B
#if defined(SHIMMER4_SDK)
#define SET_I2C_BATT_STATUS_FREQ_COMMAND 0x9C
#define RSP_I2C_BATT_STATUS_COMMAND      0x9D
#define GET_I2C_BATT_STATUS_COMMAND      0x9E
#else
#define UPD_SDLOG_CFG_COMMAND 0x9C
#endif
#define BMP280_CALIBRATION_COEFFICIENTS_RESPONSE      0x9F
#define GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND   0xA0
#define GET_BT_VERSION_STR_COMMAND                    0xA1
#define BT_VERSION_STR_RESPONSE                       0xA2
#define SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE        0xA3
#define SET_DATA_RATE_TEST                            0xA4
#define DATA_RATE_TEST_RESPONSE                       0xA5
#define PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE    0xA6
#define GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND 0xA7
#define SET_FACTORY_TEST                              0xA8
#define SET_ALT_ACCEL_CALIBRATION_COMMAND             0xA9
#define ALT_ACCEL_CALIBRATION_RESPONSE                0xAA
#define GET_ALT_ACCEL_CALIBRATION_COMMAND             0xAB
#define SET_ALT_ACCEL_SAMPLING_RATE_COMMAND           0xAC
#define ALT_ACCEL_SAMPLING_RATE_RESPONSE              0xAD
#define GET_ALT_ACCEL_SAMPLING_RATE_COMMAND           0xAE
#define SET_ALT_MAG_CALIBRATION_COMMAND               0xAF
#define ALT_MAG_CALIBRATION_RESPONSE                  0xB0
#define GET_ALT_MAG_CALIBRATION_COMMAND               0xB1
#define SET_ALT_MAG_SAMPLING_RATE_COMMAND             0xB2
#define ALT_MAG_SAMPLING_RATE_RESPONSE                0xB3
#define GET_ALT_MAG_SAMPLING_RATE_COMMAND             0xB4
#define SET_SD_SYNC_COMMAND                           0xE0
#define SD_SYNC_RESPONSE                              0xE1
#define NACK_COMMAND_PROCESSED                        0xFE
#define ACK_COMMAND_PROCESSED                         0xFF

//#define BT_RX_COMMS_TIMEOUT_TICKS                     3277U /* 32768*0.1s = 3276.8 */
#define BT_RX_COMMS_TIMEOUT_TICKS                     328U /* 32768*0.01s = 327.68  */

#define DATA_RATE_TEST_PACKET_SIZE                    5U //1 header byte + uint32_t counter value

enum
{
  BT_STREAM_CMD_STATE_IDLE = 0,
  BT_STREAM_CMD_STATE_START = 1,
  BT_STREAM_CMD_STATE_STOP = 2
};

enum
{
  SD_LOG_CMD_STATE_IDLE = 0,
  SD_LOG_CMD_STATE_START = 1,
  SD_LOG_CMD_STATE_STOP = 2
};

enum
{
  PRESSURE_SENSOR_BMP180 = 0,
  PRESSURE_SENSOR_BMP280 = 1,
  PRESSURE_SENSOR_BMP390 = 2
};

typedef enum
{
  BT_SETUP,
  SHIMMER_CMD,
  SENSOR_DATA
} btResponseType;

#if defined(SHIMMER3)
/* Order here needs to be maintained as it's saved to the EEPROM */
enum BT_BAUD_RATE
{
  BAUD_115200 = 0U,
  BAUD_1200 = 1U, //Only supported in RN42
  BAUD_2400 = 2U,
  BAUD_4800 = 3U,
  BAUD_9600 = 4U,
  BAUD_19200 = 5U,
  BAUD_38400 = 6U,
  BAUD_57600 = 7U,
  BAUD_230400 = 8U,  //Only supported in RN42
  BAUD_460800 = 9U,  //Only supported in RN42
  BAUD_921600 = 10U, //Only supported in RN42
  BAUD_1000000 = 11U, //Only supported in RN4678 v1.23 (issues with v1.13.5 & v1.22)
  BAUD_2000000 = 12U, //Only supported on CYW20820
  BAUD_NO_CHANGE_NEEDED = 0xFF,
};
#endif

typedef struct
{
  uint8_t data[BT_TX_BUF_SIZE];
  //tail points to the buffer index for the oldest byte that was added to it
  uint16_t rdIdx;
  //head points to the index of the next empty byte in the buffer
  uint16_t wrIdx;
#if !defined(SHIMMER3)
  uint16_t numBytesBeingRead;
#endif
} RingFifoTx_t;

void ShimBt_btCommsProtocolInit(void);
void ShimBt_startCommon(void);
void ShimBt_stopCommon(uint8_t isCalledFromMain);
void ShimBt_resetBtResponseVars(void);
void ShimBt_resetBtRxVariablesOnConnect(void);
void ShimBt_resetBtRxBuffs(void);
#if defined(SHIMMER3)
uint8_t ShimBt_dmaConversionDone(void);
#elif defined(SHIMMER3R)
uint8_t ShimBt_dmaConversionDone(uint8_t *rxBuff);
#endif
uint8_t ShimBt_isWaitingForArgs(void);
uint8_t ShimBt_getBtVerStrLen(void);
char *ShimBt_getBtVerStrPtr(void);

void ShimBt_processCmd(void);
void ShimBt_settingChangeCommon(uint16_t configByteIdx, uint16_t sdHeaderIdx, uint16_t len);
void ShimBt_calibrationChangeCommon(uint16_t configByteIdx,
    uint16_t sdHeaderIdx,
    uint8_t *configBytePtr,
    uint8_t *newCalibPtr,
    uint8_t sensorCalibId);
void ShimBt_updateCalibDumpFile(void);
uint8_t ShimBt_replySingleSensorCalibCmd(uint8_t cmdWaitingResponse, uint8_t *resPacketPtr);
void ShimBt_sendRsp(void);
uint8_t ShimBt_getExpectedRspForGetCmd(uint8_t getCmd);

void ShimBt_setCrcMode(COMMS_CRC_MODE btCrcModeNew);
COMMS_CRC_MODE ShimBt_getCrcMode(void);

void ShimBt_macIdSetFromStr(uint8_t *macIdStrMsbOrder);
void ShimBt_macIdSetFromBytes(uint8_t *macIdBytesLsbOrder);
char *ShimBt_macIdStrPtrGet(void);
uint8_t *ShimBt_macIdBytesPtrGet(void);
void ShimBt_macIdVarsReset(void);
void ShimBt_instreamStatusRespSend(void);
void ShimBt_handleBtRfCommStateChange(uint8_t isConnected);
volatile uint8_t *ShimBt_getBtActionPtr(void);
uint8_t *ShimBt_getBtArgsPtr(void);

void ShimBt_clearBtTxBuf(uint8_t isCalledFromMain);
uint8_t ShimBt_isBtTxBufEmpty(void);
void ShimBt_pushByteToBtTxBuf(uint8_t b);
void ShimBt_pushBytesToBtTxBuf(uint8_t *buf, uint8_t len);
uint8_t ShimBt_popBytefromBtTxBuf(void);
uint16_t ShimBt_getUsedSpaceInBtTxBuf(void);
uint16_t ShimBt_getSpaceInBtTxBuf(void);

void ShimBt_TxCpltCallback(void);
void ShimBt_sendNextCharIfNotInProgress(void);
void ShimBt_sendNextChar(void);

void ShimBt_btTxInProgressSet(uint8_t state);
uint8_t ShimBt_btTxInProgressGet(void);
void ShimBt_setDataRateTestState(uint8_t state);
uint8_t ShimBt_getDataRateTestState(void);
void ShimBt_loadTxBufForDataRateTest(void);
#if defined(SHIMMER3R)
uint8_t ShimBt_writeToTxBufAndSend(uint8_t *buf, uint8_t len, btResponseType responseType);
#endif
uint8_t ShimBt_assembleStatusBytes(uint8_t *bufPtr);

uint8_t ShimBt_isCmdAllowedWhileSdSyncing(uint8_t command);
uint8_t ShimBt_isCmdBlockedWhileSensing(uint8_t command);

void ShimBt_setBtBaudRateToUse(uint32_t baudRate);
uint32_t ShimBt_getBtBaudRateToUse(void);

#endif /* SHIMMER3_COMMON_SOURCE_BLUETOOTH_SD_SHIMMER_BT_COMMS_H_ */
