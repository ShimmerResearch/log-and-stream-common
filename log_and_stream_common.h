/*
 * log_and_stream_common.h
 *
 *  Created on: 26 Mar 2025
 *      Author: MarkNolan
 */

#ifndef LOG_AND_STREAM_COMMON_LOG_AND_STREAM_COMMON_H_
#define LOG_AND_STREAM_COMMON_LOG_AND_STREAM_COMMON_H_

#include <stdint.h>

#include "log_and_stream_definitions.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

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

__weak void delay_ms(const uint32_t delay_time_ms);

#endif /* LOG_AND_STREAM_COMMON_LOG_AND_STREAM_COMMON_H_ */
