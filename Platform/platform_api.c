/*
 * platform_api.c
 *
 *  Created on: Oct 16, 2025
 *      Author: MarkNolan
 */

#include "platform_api.h"

#include <stdint.h>

PLATFORM_WEAK void platform_delayMs(const uint32_t delay_time_ms)
{
  //This function can be overridden by the main application to provide a custom
  //delay implementation. The default implementation does nothing.
  (void) delay_time_ms; //Suppress unused parameter warning
}

PLATFORM_WEAK void platform_processHwRevision(void)
{
  // default no-op
}

PLATFORM_WEAK void platform_initGpioForRevision(void)
{
  // default no-op
}
