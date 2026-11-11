/*
 * Host-test stub for the Shimmer3 RN4X Bluetooth driver.
 *
 * Real header: shimmer3-firmware Shimmer_Driver/RN4X/RN4X.h.
 * Calibration/shimmer_calibration.c includes it under SHIMMER3 but takes
 * nothing from it. LEDs/shimmer_leds.c takes the one symbol below. If a host
 * build starts failing on another symbol from the real one, add just that one.
 */
#ifndef HOST_TEST_STUB_RN4X_H
#define HOST_TEST_STUB_RN4X_H

#include <stdint.h>

/* RN4678 link up but no RFCOMM channel open. Defined by a suite that links
 * LEDs/shimmer_leds.c. */
uint8_t isRn4678ConnectionEstablished(void);

#endif /* HOST_TEST_STUB_RN4X_H */
