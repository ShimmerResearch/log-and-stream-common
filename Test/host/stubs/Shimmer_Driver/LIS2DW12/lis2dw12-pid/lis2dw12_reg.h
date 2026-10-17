/*
 * Host-test stub for the ST LIS2DW12 platform-independent driver.
 *
 * The real header is a git submodule of shimmer3r-firmware
 * (Shimmer_Driver/LIS2DW12/lis2dw12-pid/), so a host build cannot see it.
 * Two types are needed from it:
 *   - lis2dw12_mode_t, for ShimConfig_wrAccelModeSet()/Get() in
 *     Configuration/shimmer_config.h;
 *   - lis2dw12_fs_t, whose full-scale values Calibration/shimmer_calibration.c
 *     uses as the range of each default LIS2DW12 calibration.
 *
 * The MODE enumerators are DELIBERATELY not mirrored. Nothing under host test
 * selects a mode, and declaring the type without them means any code that
 * reaches for a real mode constant fails to compile here - loudly - rather than
 * silently running against invented values. If a host test ever needs to
 * exercise wr-accel mode selection, check the submodule out and drop this stub.
 *
 * The FULL-SCALE enumerators are mirrored, because a calibration entry is
 * keyed on them. They are copied from the submodule at its v2.0.0 tag, the
 * commit shimmer3r-firmware pins.
 */
#ifndef HOST_TEST_STUB_LIS2DW12_REG_H
#define HOST_TEST_STUB_LIS2DW12_REG_H

#include <stdint.h>

typedef enum
{
  LIS2DW12_MODE_STUB_UNSET = 0
} lis2dw12_mode_t;

typedef enum
{
  LIS2DW12_2g = 0,
  LIS2DW12_4g = 1,
  LIS2DW12_8g = 2,
  LIS2DW12_16g = 3,
} lis2dw12_fs_t;

#endif /* HOST_TEST_STUB_LIS2DW12_REG_H */
