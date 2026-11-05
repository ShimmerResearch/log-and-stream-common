/*
 * shimmer_autostop.h
 *
 * The max_exp_len auto-stop (InfoMem bytes 222-223): logging stops once it has
 * run for that many minutes. Extracted from shimmer_sensing.c so that a host
 * compiler can run it.
 *
 * WHAT IS COUNTED. Real-world clock time, read at each once-a-second check.
 * Until DEV-1417 each check added one second, so the limit was only as good as
 * the blink timer that makes the checks. On Shimmer3 that timer runs from the
 * RTC's own crystal. On Shimmer3R it is TIM6 on the MSI, held to the RTC only
 * by the LED phase lock, and a Shimmer3R stopped a 7-minute log 3% early when
 * the lock counted seconds twice. Adding up the clock between checks makes the
 * limit exact to the crystal however often, and however evenly, they come.
 *
 * WHY EACH STEP IS CLAMPED. On Shimmer3R the clock is the calendar, which
 * SET_RWC moves, so the gap between two checks can be a clock set rather than
 * elapsed time. Each check adds between 0 and AUTOSTOP_MAX_STEP_TICKS, so a set
 * forward ends the log at most about a second early, and a set back extends it
 * by at most about a second. Shimmer3's RTC_get32() counts from boot and is
 * never set, so it does not jump at all.
 *
 * THE LOW 32 BITS ARE ENOUGH. They wrap every 36 hours, and a difference taken
 * modulo 2^32 is right across the wrap.
 *
 * THE TWO CONTEXTS. ShimAutostop_check() runs in the blink-timer interrupt on
 * both platforms. ShimAutostop_setLimitMins() and ShimAutostop_start() run in
 * the main loop before logging is flagged as started, so the interrupt does not
 * check while they write; shimmer_sensing.c keeps that order. The restart after
 * a stop runs in the interrupt itself.
 *
 * Pure arithmetic on 32768 Hz ticks: no hardware, no globals.
 */

#ifndef SHIMMER_AUTOSTOP_H_
#define SHIMMER_AUTOSTOP_H_

#include <stdint.h>

#define AUTOSTOP_TICKS_PER_SEC  32768UL

/* The most one check may add. Checks come once a second, so 2 s leaves room
 * for one that runs late or not at all, and bounds what a clock set can do. */
#define AUTOSTOP_MAX_STEP_TICKS (2UL * AUTOSTOP_TICKS_PER_SEC)

typedef struct
{
  uint32_t limitSecs; /* 0: no limit */
  uint32_t elapsedSecs;
  uint32_t lastRtc;      /* RTC_get32() at the start or at the previous check */
  uint16_t elapsedTicks; /* the part second on top of elapsedSecs, 0-32767 */
} Autostop_t;

/* Sets the limit. 0 turns the auto-stop off. */
void ShimAutostop_setLimitMins(volatile Autostop_t *a, uint16_t mins);

/* Counts from zero, starting at rtcNow. */
void ShimAutostop_start(volatile Autostop_t *a, uint32_t rtcNow);

/* Adds the clock time since the previous check. Returns 1 once the total has
 * reached the limit, and 0 before that or with no limit set. */
uint8_t ShimAutostop_check(volatile Autostop_t *a, uint32_t rtcNow);

#endif /* SHIMMER_AUTOSTOP_H_ */
