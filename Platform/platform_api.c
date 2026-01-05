/*
 * platform_api.c
 *
 *  Created on: Oct 16, 2025
 *      Author: MarkNolan
 *
 *  These functions can be overridden by the main application to provide a
 *  custom initialization implementation.
 *
 */

#include "platform_api.h"

#include <stdint.h>

#include "CRC/shimmer_swCrc.h"
#include "Sensing/shimmer_sensing.h"

PLATFORM_WEAK void platform_reset(void)
{
  //default no-op
}

PLATFORM_WEAK void platform_delayMs(const uint32_t delay_time_ms)
{
  //This function can be overridden by the main application to provide a custom
  //delay implementation. The default implementation does nothing.
  (void) delay_time_ms; //Suppress unused parameter warning
}

PLATFORM_WEAK uint32_t platform_getTick(void)
{
  //default no-op
  return 0;
}

PLATFORM_WEAK void platform_processHwRevision(void)
{
  //default no-op
}

PLATFORM_WEAK void platform_initGpioForRevision(void)
{
  //default no-op
}

PLATFORM_WEAK uint8_t platform_gatherData(void)
{
  ShimSens_resetCurrentCbFlags();
  ShimSens_gatherData();
  return 0;
}

PLATFORM_WEAK uint32_t platform_CrcData(uint8_t *buf, uint8_t len)
{
  /* This function can be overridden by the main application to provide hardware
   based CRC calculation. */
  return ShimSwCrc_calc(buf, len);
}
