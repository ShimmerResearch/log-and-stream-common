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
  return (uint16_t) (tenths % LED_PHASE_TENTHS_PER_FRAME);
}

int16_t ShimLedsPhase_ticksPastMidTenth(uint64_t ticks)
{
  int32_t fifths = (int32_t) ((ticks * 5U) & (LED_PHASE_FIFTHS_PER_TENTH - 1U));
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
