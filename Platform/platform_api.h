/*
 * platform_api.h
 *
 *  Created on: Oct 16, 2025
 *      Author: MarkNolan
 */

#ifndef PLATFORM_API_H_
#define PLATFORM_API_H_

#include <stdbool.h>
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

  /* PLATFORM_WEAK belongs on the default definitions in platform_api.c and
   * never on these prototypes. A weak declaration makes every definition that
   * sees it weak, the platform's own overrides included; the override then
   * wins only by link order, or the link fails on toolchains that reject two
   * weak definitions. The same applies to every __weak hook in this library. */
  void platform_reset(void);
  void platform_delayMs(const uint32_t delay_time_ms);
  uint32_t platform_getTick(void);
  void platform_processHwRevision(void);
  void platform_initGpioForRevision(void);
  uint8_t platform_gatherData(void);
  uint32_t platform_crcData(uint8_t *buf, uint8_t len);
  /* 16-bit-length variant for payloads >255 bytes (e.g. SD file-transfer
   * data frames). Same CRC conventions as platform_crcData(); override in
   * the main application to offload to a hardware CRC peripheral. */
  uint32_t platform_crcData16(uint8_t *buf, uint16_t len);
  bool platform_isDockUartInitialised(void);
  bool platform_isUsbUartInitialised(void);
  /* Called by ShimTask_NORM_manage() with interrupts masked, once it has found
   * the task list empty. Sleep in a way that an interrupt already pending, or
   * raised while masked, still wakes the part (MSP430: LPM3 | GIE in one
   * instruction; Cortex-M: WFI), and unmask before returning. Unmasking before
   * the sleep instead reopens the window this closes: a task queued in it is
   * slept through. */
  void platform_sleepWhenNoTask(void);
  /* LED phase lock (LEDs/shimmer_leds.c). The first returns the time since the
   * LED blink timer last fired, in 32768 Hz ticks; the second moves its next
   * firing later (positive) or earlier (negative) by that many ticks. The
   * defaults report 0 and do nothing, which still aligns the blink frame to
   * the clock but leaves the tick itself up to 100 ms off. */
  uint16_t platform_ledTickElapsedRtcTicks(void);
  void platform_ledTickShift(int16_t rtcTicks);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_API_H_ */
