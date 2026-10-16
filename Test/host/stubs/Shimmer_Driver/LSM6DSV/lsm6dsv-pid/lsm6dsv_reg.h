/*
 * Host-test stub for the ST LSM6DSV platform-independent driver (Shimmer3R).
 *
 * The real header is a git submodule of shimmer3r-firmware
 * (Shimmer_Driver/LSM6DSV/lsm6dsv-pid/), and the firmware's shimmer_include.h
 * includes it. Calibration/shimmer_calibration.c keys each default LSM6DSV
 * calibration on these full-scale values, so they are mirrored exactly, from
 * the submodule at its v2.0.0 tag - the commit shimmer3r-firmware pins. Note
 * the gyro's 4000 dps is 0xc, not 5.
 */
#ifndef HOST_TEST_STUB_LSM6DSV_REG_H
#define HOST_TEST_STUB_LSM6DSV_REG_H

typedef enum
{
  LSM6DSV_125dps = 0x0,
  LSM6DSV_250dps = 0x1,
  LSM6DSV_500dps = 0x2,
  LSM6DSV_1000dps = 0x3,
  LSM6DSV_2000dps = 0x4,
  LSM6DSV_4000dps = 0xc,
} lsm6dsv_gy_full_scale_t;

typedef enum
{
  LSM6DSV_2g = 0x0,
  LSM6DSV_4g = 0x1,
  LSM6DSV_8g = 0x2,
  LSM6DSV_16g = 0x3,
} lsm6dsv_xl_full_scale_t;

#endif /* HOST_TEST_STUB_LSM6DSV_REG_H */
