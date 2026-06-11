/*
 * log_and_stream_common.h
 *
 *  Created on: 26 Mar 2025
 *      Author: MarkNolan
 */

#ifndef LOG_AND_STREAM_COMMON_LOG_AND_STREAM_COMMON_H_
#define LOG_AND_STREAM_COMMON_LOG_AND_STREAM_COMMON_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "log_and_stream_definitions.h"

/* Battery-read policy. ADC sensor data is prioritised over an up-to-date battery
 * value: a fresh battery measurement (which borrows/uses the ADC) is only taken
 * when it cannot disturb the ADC sensor stream. See LogAndStream_getBattReadAction(). */
typedef enum
{
  BATT_READ_NEW = 0,    //take a fresh battery measurement now
  BATT_READ_USE_STREAM, //sensing + VBatt streamed: use the latest streamed sample
  BATT_READ_REPEAT_LAST //Shimmer3 sensing + other ADC channel(s), no VBatt: keep the
                        //last value - a read on the shared ADC would disturb the data
} battReadAction_t;

void LogAndStream_init(void);
void LogAndStream_setBootStage(boot_stage_t bootStageNew);
boot_stage_t LogAndStream_getBootStage(void);
void LogAndStream_syncConfigAndCalibOnSd(void);
uint8_t LogAndStream_isSdInfoSyncDelayed(void);
void LogAndStream_setSdInfoSyncDelayed(uint8_t state);
void LogAndStream_blinkTimerCommon(void);
uint8_t LogAndStream_isDockedOrUsbIn(void);
void LogAndStream_dockedStateChange(void);
void LogAndStream_infomemUpdate(void);
uint8_t LogAndStream_updateDockedStateAndCheckChanged(void);
void LogAndStream_dockOrUsbStateUpdate(void);
void LogAndStream_checkSetupDockUnDock(void);
void LogAndStream_setupDockUndock(void);
void LogAndStream_setupDock(void);
void LogAndStream_setupUndock(void);
void LogAndStream_assignSdToDock(void);
void LogAndStream_releaseSdToMcu(void);
uint8_t LogAndStream_checkSdInSlot(void);
battReadAction_t LogAndStream_getBattReadAction(void);
void LogAndStream_processDaughterCardId(void);
void LogAndStream_buildShimmerMacSuffix(char *outBuf, size_t outBufLen);
void LogAndStream_buildShimmerPrefix(char *outBuf, size_t outBufLen);
bool LogAndStream_allowDockComms(void);

#endif /* LOG_AND_STREAM_COMMON_LOG_AND_STREAM_COMMON_H_ */
