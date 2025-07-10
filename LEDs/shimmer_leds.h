/*
 * led.h
 *
 *  Created on: 25 Mar 2025
 *      Author: MarkNolan
 */

#ifndef LED_H_
#define LED_H_

#include "stdint.h"

#include <log_and_stream_definitions.h>

/* Original approach for 5 individual LEDs on Shimmer3. The Shimmer3R has two
 * RGB LEDs instead but the original approach is still supported. */
#define LED_LWR_RED    0x01
#define LED_LWR_GREEN  0x02
#define LED_LWR_YELLOW 0x04
#define LED_UPR_GREEN  0x08
#define LED_UPR_BLUE   0x10
#define LED_ALL        0xFF
#define LED_ALL_OFF    0x00

void ShimLeds_varsInit(void);
void ShimLeds_incrementCounters(void);
void ShimLeds_controlDuringBoot(boot_stage_t bootStageCurrent);
void ShimLeds_blink(void);
uint8_t ShimLeds_isBlinkTimerCnt200ms(void);
uint8_t ShimLeds_isBlinkTimerCnt1s(void);
uint8_t ShimLeds_isBlinkTimerCnt2s(void);
void ShimLeds_setRtcErrorFlash(uint8_t state);

#endif /* LED_H_ */
