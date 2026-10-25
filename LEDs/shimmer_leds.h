/*
 * led.h
 *
 *  Created on: 25 Mar 2025
 *      Author: MarkNolan
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>

#include "log_and_stream_definitions.h"

#define SHIMMER_BLINK_TIMER_PERIOD_MS 100

/* Original approach for 5 individual LEDs on Shimmer3. The Shimmer3R has two
 * RGB LEDs instead but the original approach is still supported. */
#define LED_LWR_RED                   0x01
#define LED_LWR_GREEN                 0x02
#define LED_LWR_YELLOW                0x04
#define LED_UPR_GREEN                 0x08
#define LED_UPR_BLUE                  0x10
#define LED_ALL                       0xFF
#define LED_ALL_OFF                   0x00

#if defined(SHIMMER3R)
/* LED_SET (dock): one mask per RGB LED. Any other value is refused, except
 * LED_SET_RELEASE for both, which hands the LEDs back. */
#define LED_SET_RED             0x01
#define LED_SET_GREEN           0x02
#define LED_SET_BLUE            0x04
#define LED_SET_ALL             (LED_SET_RED | LED_SET_GREEN | LED_SET_BLUE)
#define LED_SET_RELEASE         0xFF
/* An override not released by the host ends by itself after this long */
#define LED_OVERRIDE_TIMEOUT_MS 60000U
#endif

void ShimLeds_varsInit(void);
void ShimLeds_incrementCounters(void);
uint8_t ShimLeds_isPhaseSyncDue(void);
void ShimLeds_phaseSync(void);
void ShimLeds_controlDuringBoot(boot_stage_t bootStageCurrent);
void ShimLeds_blink(void);
uint8_t ShimLeds_isBlinkTimerCnt200ms(void);
uint8_t ShimLeds_isBlinkTimerCnt500ms(void);
uint8_t ShimLeds_isBlinkTimerCnt1s(void);
uint8_t ShimLeds_isBlinkTimerCnt2s(void);
uint8_t ShimLeds_isBlinkTimerCnt5s(void);
void ShimLeds_setRtcErrorFlash(uint8_t state);
uint8_t ShimLeds_getRtcErrorFlash(void);
#if defined(SHIMMER3R)
/* Returns 1 if the LEDs were set (or released), 0 if the masks were refused */
uint8_t ShimLeds_overrideSet(uint8_t lwrMask, uint8_t uprMask);
void ShimLeds_overrideRelease(void);
uint8_t ShimLeds_isOverrideActive(void);
#endif

#endif /* LED_H_ */
