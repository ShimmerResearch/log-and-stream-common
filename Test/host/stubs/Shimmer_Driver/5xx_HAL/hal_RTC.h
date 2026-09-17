/*
 * Host-test stub for the MSP430 RTC HAL (Shimmer3 only).
 * Real header: shimmer3-firmware Shimmer_Driver/5xx_HAL/hal_RTC.h.
 * Declarations only - the portable timekeeping maths under test lives in
 * RTC/shimmer_rtc.c, and host_stubs.c supplies the platform clock.
 */
#ifndef HOST_TEST_STUB_HAL_RTC_H
#define HOST_TEST_STUB_HAL_RTC_H

#include <stdint.h>

void RTC_init(uint64_t rtc_val);
void RTC_set64(uint64_t rtc_val);
uint64_t RTC_get64(void);
uint32_t RTC_get32(void);

#endif /* HOST_TEST_STUB_HAL_RTC_H */
