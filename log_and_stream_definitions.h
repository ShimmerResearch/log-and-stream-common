/*
 * shimmer_definitions.h
 *
 *  Created on: 7 Oct 2024
 *      Author: MarkNolan
 */

#ifndef SHIMMER3_COMMON_SOURCE_SHIMMER_DEFINITIONS_H_
#define SHIMMER3_COMMON_SOURCE_SHIMMER_DEFINITIONS_H_

#include <stdint.h>

#define FW_IS_LOGANDSTREAM        1

#define IS_SUPPORTED_SINGLE_TOUCH 0
#define USE_FATFS                 1
#define USE_SD                    1

#define MAX_NODES                 20
#define MAX_CHARS                 13
#define UINT32_LEN                11 //10+1, where the last byte should be 0x00
#define UINT64_LEN                21 //20+1, where the last byte should be 0x00
#if defined(SHIMMER3)
#define RESPONSE_PACKET_SIZE 133
#else
#define RESPONSE_PACKET_SIZE 1024 //133
#endif

typedef volatile struct STATTypeDef_t
{ //STATUS
  uint8_t initialising     : 1;
  uint8_t docked           : 1;
  uint8_t sensing          : 1;
  uint8_t configuring      : 1;
  uint8_t buttonPressed    : 1;

  uint8_t btConnected      : 1;
  uint8_t btPowerOn        : 1;
  uint8_t btSupportEnabled : 1;
  uint8_t btStreaming      : 1;
  uint8_t btstreamReady    : 1;
  uint8_t btstreamCmd      : 2;
  uint8_t btInSyncMode     : 1;

#if defined(SHIMMER3R)
  uint8_t sdPeripheralInit : 1;
#endif
  uint8_t sdInserted : 1;
  uint8_t sdPowerOn  : 1;
#if defined(SHIMMER3R)
  uint8_t sdMcu0Pc1 : 1;
#endif
  uint8_t sdLogging              : 1;
  uint8_t sdlogReady             : 1;
  uint8_t sdlogCmd               : 2;
  uint8_t sdBadFile              : 1;
  uint8_t sdSyncEnabled          : 1;
  uint8_t sdSyncCommTimerRunning : 1;

  uint8_t toggleLedRedCmd        : 1;
#if defined(SHIMMER3R)
  uint32_t testResult;
  uint8_t pinPvI2c;
  uint8_t pinPvSd;
  uint8_t pinPvExt;
  uint8_t periStat;
#endif
} STATTypeDef;

typedef enum
{
  BOOT_STAGE_START,
  BOOT_STAGE_I2C,
  BOOT_STAGE_BLUETOOTH,
  BOOT_STAGE_BLUETOOTH_FAILURE,
  BOOT_STAGE_CONFIGURATION,
  BOOT_STAGE_END
} boot_stage_t;

typedef enum
{
  HAL_SHIM_OK = 0x00,
  HAL_SHIM_ERROR = 0x01,
  HAL_SHIM_BUSY = 0x02,
  HAL_SHIM_TIMEOUT = 0x03
} HAL_StatusTypeDefShimmer;

#endif /* SHIMMER3_COMMON_SOURCE_SHIMMER_DEFINITIONS_H_ */
