/*
 * shimmer_autostop.c
 *
 * See shimmer_autostop.h. Written for the MSP430's 16-bit int as much as the
 * STM32's 32-bit one: every intermediate wider than 16 bits is cast.
 */

#include "shimmer_autostop.h"

void ShimAutostop_setLimitMins(volatile Autostop_t *a, uint16_t mins)
{
  /* Widened before the multiply. On MSP430 int is 16 bits, so mins * 60 would
   * be computed in 16 bits and truncate every limit above 1092 minutes. */
  a->limitSecs = (uint32_t) mins * 60U;
}

void ShimAutostop_start(volatile Autostop_t *a, uint32_t rtcNow)
{
  a->lastRtc = rtcNow;
  a->elapsedTicks = 0;
  a->elapsedSecs = 0;
}

uint8_t ShimAutostop_check(volatile Autostop_t *a, uint32_t rtcNow)
{
  uint32_t step = rtcNow - a->lastRtc;
  uint32_t ticks;

  a->lastRtc = rtcNow;
  if (a->limitSecs == 0U)
  {
    return 0;
  }

  /* The top bit set is a step back: the clock was set earlier */
  if (step & 0x80000000UL)
  {
    step = 0;
  }
  else if (step > AUTOSTOP_MAX_STEP_TICKS)
  {
    step = AUTOSTOP_MAX_STEP_TICKS;
  }

  /* >> 15 and & 0x7FFF divide by 32768 without a 32-bit division, which the
   * MSP430 does in software */
  ticks = (uint32_t) a->elapsedTicks + step;
  a->elapsedSecs += ticks >> 15;
  a->elapsedTicks = (uint16_t) (ticks & 0x7FFFUL);

  return (uint8_t) (a->elapsedSecs >= a->limitSecs);
}
