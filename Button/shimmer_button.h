/*
 * shimmer_button.h
 *
 *  Created on: 30 Jun 2025
 *      Author: MarkNolan
 */

#ifndef LOG_AND_STREAM_COMMON_BUTTON_SHIMMER_BUTTON_H_
#define LOG_AND_STREAM_COMMON_BUTTON_SHIMMER_BUTTON_H_

#include "stdint.h"

//5 seconds = 163840 ticks @ 32768Hz
#define TICKS_5_SECONDS   163840
//0.5 seconds = 16384 ticks @ 32768Hz
#define TICKS_0_5_SECONDS 16384

void ShimBtn_init(void);
uint8_t ShimBtn_pressReleaseAction(void);

#endif /* LOG_AND_STREAM_COMMON_BUTTON_SHIMMER_BUTTON_H_ */
