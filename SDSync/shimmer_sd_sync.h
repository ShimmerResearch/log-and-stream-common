/*
 * sd_sync.h
 *
 *  Created on: 11 Oct 2022
 *      Author: Mark Nolan
 */

#ifndef SHIMMER3_COMMON_SOURCE_BLUETOOTH_SD_SD_SYNC_H_
#define SHIMMER3_COMMON_SOURCE_BLUETOOTH_SD_SD_SYNC_H_

#include <stdint.h>

#if defined(SHIMMER3)
#include "../../version.h"
#include "../5xx_HAL/hal_CRC.h"
#include "msp430.h"
#elif defined(SHIMMER3R)
#include "crc.h"
#endif

#define BT_SD_SYNC_CRC_MODE CRC_1BYTES_ENABLED

//BT routine communication
//all node time must *2 in use
//all center time must *4 in use
//#define RC_AHD          3
//#define RC_WINDOW_N     13
//#define RC_WINDOW_C     27
//#define RC_INT_N        27
#define SYNC_INT_C          54 //240
//#define RC_CLK_N        16384   //16384=2hz;//32768=1hz;8192=4hz
//#define RC_CLK_C        8192   //16384=2hz;//32768=1hz;8192=4hz
//#define RC_FACTOR_N     (32768/RC_CLK_N)   //16384=2hz;//32768=1hz;8192=4hz
//#define RC_FACTOR_C     (32768/RC_CLK_C)   //16384=2hz;//32768=1hz;8192=4hz

//Node to Center Sync Packet - ACK(1 byte):SD_SYNC_RESPONSE(1 byte):Flag(1 byte)
//Center to Node Sync Packet - SET_SD_SYNC_COMMAND(1 byte):Status(1 byte):Time(8 bytes, LSB order):CRC(0/1/2 bytes, LSB order)
#define SYNC_PACKET_MAX_SIZE \
  (SYNC_PACKET_SIZE_CMD + SYNC_PACKET_PAYLOAD_SIZE + CRC_2BYTES_ENABLED)
#define SYNC_PACKET_PAYLOAD_SIZE 9
#define SYNC_PACKET_CMD_IDX      0
#define SYNC_PACKET_SIZE_CMD     1
#define SYNC_PACKET_FLG_IDX      1
#define SYNC_PACKET_TIME_IDX     2
#define SYNC_PACKET_TIME_SIZE    8

//new sync
#define SYNC_PERIOD              32768
#define SYNC_FACTOR              1 //32768/SYNC_PERIOD
#define SYNC_BOOT                3
#define SYNC_CD                  2
#define SYNC_EXTEND              4 //should have SYNC_BOOT>SYNC_CD
//below were params in sdlog.cfg
// Allow 12 seconds for the center to try and connect to each node
#define SYNC_T_EACHNODE_C        12
#define SYNC_WINDOW_C            800
#define SYNC_WINDOW_N            50
#define SYNC_NODE_REBOOT         17
#define SYNC_TRANS_IN_ONE_COMM   50
#define SYNC_NEXT2MATCH          2

#define SYNC_PACKET_RESEND       0x01
#define SYNC_FINISHED            0xFF

void ShimSdSync_init(void (*btStart_cb)(void), void (*btStop_cb)(uint8_t));
void ShimSdSync_resetMyTimeDiff(void);
void ShimSdSync_resetMyTimeDiffArrays(void);
void ShimSdSync_resetMyTimeDiffLongMin(void);
uint8_t *ShimSdSync_myTimeDiffPtrGet(void);
void ShimSdSync_syncRespSet(uint8_t *args, uint8_t count);
uint8_t ShimSdSync_isBtSdSyncRunning(void);
uint8_t ShimSdSync_syncNodeNumGet(void);
uint8_t *ShimSdSync_syncNodeNamePtrForIndexGet(uint8_t index);
uint8_t *ShimSdSync_syncCenterNamePtrGet(void);
uint8_t ShimSdSync_rcFirstOffsetRxedGet(void);
uint8_t ShimSdSync_syncSuccCenterGet(void);
uint8_t ShimSdSync_syncSuccNodeGet(void);
uint8_t ShimSdSync_syncCntGet(void);
void ShimSdSync_saveLocalTime(void);
void ShimSdSync_resetSyncRcNodeR10Cnt(void);
void ShimSdSync_resetSyncVariablesBeforeParseConfig(void);
void ShimSdSync_resetSyncVariablesDuringSyncStart(void);
void ShimSdSync_resetSyncVariablesCenter(void);
void ShimSdSync_resetSyncVariablesNode(void);
void ShimSdSync_resetSyncNodeArray(void);
uint16_t ShimSdSync_parseSyncEstExpLen(uint8_t estExpLenLsb, uint8_t estExpLenMsb);
void ShimSdSync_setSyncEstExpLen(uint32_t est_exp_len);
void ShimSdSync_parseSyncNodeNamesFromConfig(uint8_t *storedConfigPtr);
void ShimSdSync_parseSyncCenterNameFromConfig(uint8_t *storedConfigPtr);
void ShimSdSync_parseSyncCenterNameFromCfgFile(uint8_t *storedConfigPtr, char *equals);
void ShimSdSync_parseSyncNodeNameFromCfgFile(uint8_t *storedConfigPtr, char *equals);
void ShimSdSync_checkSyncCenterName(void);

void ShimSdSync_stop(void);
void ShimSdSync_start(uint8_t iAmSyncCenterToSet, uint16_t experimentLengthEstimatedInSecToSet);

void ShimSdSync_centerT10(void);
void ShimSdSync_centerR1(void);
uint32_t ShimSdSync_nodeShift(uint8_t shift_value);
void ShimSdSync_nodeR10(void);
void ShimSdSync_nodeT1(uint8_t val);
uint8_t ShimSdSync_rcFindSmallest(void);

void ShimSdSync_handleSyncTimerTrigger(void);
void ShimSdSync_handleSyncTimerTriggerCenter(void);
void ShimSdSync_handleSyncTimerTriggerNode(void);
void ShimSdSync_startBtForSync(void);

void ShimSdSync_CommTimerStart(void);
void ShimSdSync_CommTimerStop(void);

#if defined(SHIMMER3)
uint16_t GetTA0(void);
#endif

#endif /* SHIMMER3_COMMON_SOURCE_BLUETOOTH_SD_SD_SYNC_H_ */
