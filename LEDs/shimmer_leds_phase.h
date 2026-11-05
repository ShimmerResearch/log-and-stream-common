/*
 * shimmer_leds_phase.h
 *
 * LED blink phase derived from the real-world clock, so that every sensor
 * whose clock was set from the same source flashes on the same frame
 * boundaries. On an SD-sync node the centre's clock is used instead, through
 * the offset that sync already measures.
 *
 * Everything here is pure arithmetic on 32768 Hz ticks: no hardware, no
 * globals. The LED module and the platform timers do the rest.
 */

#ifndef SHIMMER_LEDS_PHASE_H_
#define SHIMMER_LEDS_PHASE_H_

#include <stdint.h>

/* One LED frame is 60 s of 100 ms tenths. 60 s is a multiple of every pattern
 * period in shimmer_leds.c (0.2, 0.4, 1, 2, 3, 4 and 5 s), so every pattern
 * is a pure function of the frame tenth. */
#define LED_PHASE_TENTHS_PER_FRAME 600U

/* One tenth is 3276.8 ticks. Working in fifths of a tick makes it exactly
 * 16384, so the tenth index is a shift and the position within it a mask. */
#define LED_PHASE_FIFTHS_PER_TENTH 16384U
#define LED_PHASE_MID_TENTH_FIFTHS (LED_PHASE_FIFTHS_PER_TENTH / 2U)

/* Phase errors smaller than this (about 5 ms) are left alone, so a timer that
 * can only be moved in coarse steps does not hunt around the target. */
#define LED_PHASE_DEADBAND_TICKS   164

/* Size of the SD-sync offset that ShimSdSync_myTimeDiffPtrGet() points to:
 * one sign byte, then a 64-bit magnitude in LSB order. */
#define LED_PHASE_SYNC_OFFSET_SIZE 9U

/* Tenth of the 60 s frame (0-599) that a clock reading falls in. */
uint16_t ShimLedsPhase_frameTenth(uint64_t ticks);

/* How far a clock reading is past the middle of its tenth, in ticks
 * (-1638 to +1638). Positive means the LED tick fired late. */
int16_t ShimLedsPhase_ticksPastMidTenth(uint64_t ticks);

/* Converts a node's clock reading to the centre's clock using the 9-byte
 * offset SD sync recorded. Flag 0 means the node is ahead of the centre. */
uint64_t ShimLedsPhase_toCentreTime(uint64_t localTicks, const uint8_t *syncOffset);

/* What the once-a-second sync does with the time the last blink tick fired:
 * the tenth the frame counter should hold, and how far to move the next tick
 * in ticks (positive = later). The shift is 0 inside the deadband. */
typedef struct
{
  uint16_t frameTenth;
  int16_t shiftTicks;
} LedPhaseCorrection_t;

LedPhaseCorrection_t ShimLedsPhase_correction(uint64_t tickTime);

/* For a blink timer that counts up from 0 and fires as it reloads after
 * `reload`, as Shimmer3R's TIM6 does. Earlier steps the counter forward.
 * Later cannot step it back: the sync runs just after a tick, when the counter
 * is 0 or 1, so there is nothing to step back over. A timer running fast then
 * outruns the lock, and each time its tick slips into the previous tenth the
 * sync renumbers it, so one whole-second tick fires twice (DEV-1417). A delay
 * therefore lengthens the next period instead.
 *
 * `counter` is the value to leave in the timer: the caller writes it only if
 * it differs from the one read, so a delay never writes the counter at all.
 * `periodExtra` is counts to add to the next period, 0 for none. */
typedef struct
{
  uint16_t counter;
  uint16_t periodExtra;
} LedPhaseTimerShift_t;

LedPhaseTimerShift_t
ShimLedsPhase_counterTimerShift(uint16_t counter, uint16_t reload, int16_t shiftCounts);

/* Pattern helpers. Each replaces a toggle whose phase depended on when the
 * state was entered with a level that depends only on the frame tenth. */
uint8_t ShimLedsPhase_isOnAlternate200ms(uint16_t frameTenth);
uint8_t ShimLedsPhase_isOnAlternateSecond(uint16_t frameTenth);

/* Connected and logging: blue for two seconds, green for one. */
uint8_t ShimLedsPhase_isGreenConnectedAndLogging(uint16_t frameTenth);

/* Logging and streaming: green, off, blue, off, one second each. */
#define LED_PHASE_COLOUR_OFF   0U
#define LED_PHASE_COLOUR_GREEN 1U
#define LED_PHASE_COLOUR_BLUE  2U
uint8_t ShimLedsPhase_loggingAndStreamingColour(uint16_t frameTenth);

#endif /* SHIMMER_LEDS_PHASE_H_ */
