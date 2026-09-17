/*
 * Host-test stub for the MSP430 hardware CRC unit (Shimmer3 only).
 *
 * Real header: shimmer3-firmware Shimmer_Driver/5xx_HAL/hal_CRC.h. Declarations
 * only - the software CRC in CRC/shimmer_swCrc.c is what the host tests cover,
 * and Test/host/test_swcrc.c pins it to the wire-format vectors.
 *
 * This directory is also the include-path anchor that makes Boards/'s
 * "../BMPX80/bmpX80.h" resolve, mirroring how the firmware builds pass
 * -I Shimmer_Driver -I Shimmer_Driver/5xx_HAL.
 */
#ifndef HOST_TEST_STUB_HAL_CRC_H
#define HOST_TEST_STUB_HAL_CRC_H

#include <stdint.h>

uint16_t CRC_data(uint8_t *buf, uint8_t len);

#endif /* HOST_TEST_STUB_HAL_CRC_H */
