/*
 * Host-test stub for the Shimmer3 pressure-sensor driver.
 *
 * Boards/shimmer_boards.c includes this only under SHIMMER3, for the one
 * predicate below - ShimBrd_are2ndGenImuSensorsPresent() asks whether a BMP280
 * is fitted. host_stubs.c defines it; hostStub_setBmp280InUse() sets the answer.
 */
#ifndef HOST_TEST_STUB_BMPX80_H
#define HOST_TEST_STUB_BMPX80_H

#include <stdint.h>

uint8_t isBmp180InUse(void);
uint8_t isBmp280InUse(void);

#endif /* HOST_TEST_STUB_BMPX80_H */
