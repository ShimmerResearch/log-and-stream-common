/*
 * shimmer_leds_phase.c
 *
 * See shimmer_leds_phase.h. Written for the MSP430's 16-bit int as much as
 * the STM32's 32-bit one: every intermediate wider than 16 bits is cast.
 */

#include "shimmer_leds_phase.h"

uint16_t ShimLedsPhase_frameTenth(uint64_t ticks)
{
  /* ticks * 5 is the time in fifths of a tick; >> 14 divides by 16384, the
   * fifths in one tenth, giving tenths since the epoch */
  uint64_t tenths = (ticks * 5U) >> 14;

  /* tenths % 600 without a 64-bit division. On the MSP430 that is a software
   * loop long enough that, run once a second in the main loop, it held up
   * sample completion: at 1024 Hz Shimmer3 lost a sample every second
   * (DEV-1386). Reduce the two 32-bit halves instead; 2^32 % 600 = 496. */
  uint32_t hi = (uint32_t) (tenths >> 32);
  uint32_t lo = (uint32_t) tenths;
  uint32_t r = (hi % LED_PHASE_TENTHS_PER_FRAME) * LED_PHASE_2POW32_MOD_FRAME
      + (lo % LED_PHASE_TENTHS_PER_FRAME);
  return (uint16_t) (r % LED_PHASE_TENTHS_PER_FRAME);
}

int16_t ShimLedsPhase_ticksPastMidTenth(uint64_t ticks)
{
  /* Only the low 14 bits of ticks * 5 are kept, and they depend only on the
   * low 14 bits of ticks, so a 32-bit product gives the same answer */
  int32_t fifths
      = (int32_t) (((uint32_t) ticks * 5U) & (LED_PHASE_FIFTHS_PER_TENTH - 1U));
  return (int16_t) ((fifths - (int32_t) LED_PHASE_MID_TENTH_FIFTHS) / 5);
}

uint64_t ShimLedsPhase_toCentreTime(uint64_t localTicks, const uint8_t *syncOffset)
{
  uint64_t magnitude = 0;
  uint8_t i;

  for (i = LED_PHASE_SYNC_OFFSET_SIZE - 1U; i > 0U; i--)
  {
    magnitude = (magnitude << 8) | syncOffset[i];
  }

  /* Flag 0: the node read ahead of the centre, so the centre is behind */
  if (syncOffset[0] == 0U)
  {
    return localTicks - magnitude;
  }
  return localTicks + magnitude;
}

LedPhaseCorrection_t ShimLedsPhase_correction(uint64_t tickTime)
{
  LedPhaseCorrection_t c;
  int16_t late = ShimLedsPhase_ticksPastMidTenth(tickTime);

  c.frameTenth = ShimLedsPhase_frameTenth(tickTime);
  c.shiftTicks = 0;
  if (late > LED_PHASE_DEADBAND_TICKS || late < -LED_PHASE_DEADBAND_TICKS)
  {
    c.shiftTicks = (int16_t) -late;
  }
  return c;
}

LedPhaseTimerShift_t
ShimLedsPhase_counterTimerShift(uint16_t counter, uint16_t reload, int16_t shiftCounts)
{
  LedPhaseTimerShift_t s;

  s.counter = counter;
  s.periodExtra = 0;

  if (shiftCounts > 0)
  {
    /* One extra period at most. The lock never asks for more than half a
     * tenth, so this only bounds a caller's mistake. */
    s.periodExtra = ((uint16_t) shiftCounts > reload) ? reload : (uint16_t) shiftCounts;
  }
  else if (shiftCounts < 0 && counter < reload)
  {
    /* A counter already at the reload value may wrap before the caller's write
     * lands, and the stale value would then undo the reload. That case is
     * skipped; the next second's sync applies the shift instead. Negated as
     * unsigned, which a 16-bit int cannot overflow. */
    uint16_t room = (uint16_t) (reload - counter);
    uint16_t step = (uint16_t) (0U - (uint16_t) shiftCounts);
    s.counter = (uint16_t) (counter + ((step > room) ? room : step));
  }
  return s;
}

uint8_t ShimLedsPhase_isOnAlternate200ms(uint16_t frameTenth)
{
  return (uint8_t) ((frameTenth >> 1) & 1U);
}

uint8_t ShimLedsPhase_isOnAlternateSecond(uint16_t frameTenth)
{
  return (uint8_t) (((frameTenth / 10U) & 1U) == 0U);
}

uint8_t ShimLedsPhase_isGreenConnectedAndLogging(uint16_t frameTenth)
{
  return (uint8_t) ((frameTenth / 10U) % 3U == 2U);
}

uint8_t ShimLedsPhase_loggingAndStreamingColour(uint16_t frameTenth)
{
  switch ((frameTenth / 10U) % 4U)
  {
    case 0U:
      return LED_PHASE_COLOUR_GREEN;
    case 2U:
      return LED_PHASE_COLOUR_BLUE;
    default:
      return LED_PHASE_COLOUR_OFF;
  }
}
