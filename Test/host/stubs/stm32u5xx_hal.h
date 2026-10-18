/*
 * Host-test stub for the Shimmer3R firmware's stm32u5xx_hal.h, which
 * SDCard/shimmer_sd_data_file.c includes under SHIMMER3R.
 *
 * Only assert_param(), as the firmware defines it in
 * Core/Inc/stm32u5xx_hal_conf.h with USE_FULL_ASSERT set (it is). There
 * assert_failed() is an empty function that returns, so a failed check does not
 * stop the firmware - a failing f_write() carries on into its recovery path,
 * which is exactly what a host test needs to reach. A suite that compiles a
 * module using it defines assert_failed() itself, and may count the calls.
 */
#ifndef HOST_TEST_STUB_STM32U5XX_HAL_H
#define HOST_TEST_STUB_STM32U5XX_HAL_H

#include <stdint.h>

void assert_failed(uint8_t *file, uint32_t line);
#define assert_param(expr) \
  ((expr) ? (void) 0U : assert_failed((uint8_t *) __FILE__, __LINE__))

#endif
