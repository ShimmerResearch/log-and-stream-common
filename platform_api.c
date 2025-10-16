/*
 * platform_api.c
 *
 *  Created on: Oct 16, 2025
 *      Author: MarkNolan
 */

#include "platform_api.h"

#include <stdint.h>

#include "log_and_stream_definitions.h"
#if defined(SHIMMER3R)
#include "stm32u5xx_hal.h"
#endif

__weak void delay_ms(const uint32_t delay_time_ms)
{
  //This function can be overridden by the main application to provide a custom
  //delay implementation. The default implementation does nothing.
  (void) delay_time_ms; //Suppress unused parameter warning
}

__weak void ProcessHwRevision(void)
{
  __NOP();
}


__weak void Board_initGpioForRevision(void)
{
  __NOP();
}
