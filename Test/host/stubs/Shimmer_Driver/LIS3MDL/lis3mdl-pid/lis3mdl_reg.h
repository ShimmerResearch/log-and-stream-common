/*
 * Host-test stub for the ST LIS3MDL platform-independent driver (Shimmer3R).
 *
 * The real header is a git submodule of shimmer3r-firmware
 * (Shimmer_Driver/LIS3MDL/lis3mdl-pid/), and the firmware's shimmer_include.h
 * includes it. Calibration/shimmer_calibration.c keys each default LIS3MDL
 * calibration on these full-scale values, so they are mirrored exactly, from
 * the submodule at its v2.0.0 tag - the commit shimmer3r-firmware pins.
 */
#ifndef HOST_TEST_STUB_LIS3MDL_REG_H
#define HOST_TEST_STUB_LIS3MDL_REG_H

typedef enum
{
  LIS3MDL_4_GAUSS = 0,
  LIS3MDL_8_GAUSS = 1,
  LIS3MDL_12_GAUSS = 2,
  LIS3MDL_16_GAUSS = 3,
} lis3mdl_fs_t;

#endif /* HOST_TEST_STUB_LIS3MDL_REG_H */
