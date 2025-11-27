/*
 * platform_api.h
 *
 *  Created on: Oct 16, 2025
 *      Author: MarkNolan
 */

#ifndef PLATFORM_API_H_
#define PLATFORM_API_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

//Compiler-agnostic weak attribute
#if defined(SHIMMER3)
/*TODO Special case for SHIMMER3 shouldn't be needed but for some reason Code composer thinks is already defined so just overwriting it here*/
#define PLATFORM_WEAK __attribute__((weak))
#else
#if !defined(PLATFORM_WEAK)
#if defined(__GNUC__) || defined(__clang__)
#define PLATFORM_WEAK __attribute__((weak))
#elif defined(__ICCARM__) || defined(__IAR_SYSTEMS_ICC__)
#define PLATFORM_WEAK __weak
#elif defined(__ARMCC_VERSION)
#define PLATFORM_WEAK __weak
#else
#define PLATFORM_WEAK
#endif
#endif
#endif

  PLATFORM_WEAK void platform_delayMs(const uint32_t delay_time_ms);
  PLATFORM_WEAK void platform_processHwRevision(void);
  PLATFORM_WEAK void platform_initGpioForRevision(void);
  PLATFORM_WEAK uint8_t platform_gatherData(void);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_API_H_ */
