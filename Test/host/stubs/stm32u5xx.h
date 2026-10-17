/*
 * Host-test stub for ST's STM32U5 device header (Shimmer3R only).
 *
 * The real stm32u5xx.h is CMSIS
 * (shimmer3r-firmware Drivers/CMSIS/Device/ST/STM32U5xx/Include/), and with
 * USE_HAL_DRIVER defined it includes stm32u5xx_hal.h, which declares
 * HAL_Delay(). Calibration/shimmer_calibration.c includes it for that one call,
 * a 50 ms wait after closing a file. host_stubs.c defines it as a no-op.
 */
#ifndef HOST_TEST_STUB_STM32U5XX_H
#define HOST_TEST_STUB_STM32U5XX_H

#include <stdint.h>

void HAL_Delay(uint32_t Delay);

#endif /* HOST_TEST_STUB_STM32U5XX_H */
