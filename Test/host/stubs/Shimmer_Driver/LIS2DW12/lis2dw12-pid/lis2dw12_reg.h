/*
 * Host-test stub for the ST LIS2DW12 platform-independent driver.
 *
 * The real header is a git submodule of shimmer3r-firmware
 * (Shimmer_Driver/LIS2DW12/lis2dw12-pid/), so a host build cannot see it.
 * Configuration/shimmer_config.h needs exactly one type from it, for
 * ShimConfig_wrAccelModeSet()/Get().
 *
 * The enumerators are DELIBERATELY not mirrored. Their register values are the
 * vendor's and guessing them would make a host test agree with itself while
 * disagreeing with the part. Declaring the type without them means any code
 * that reaches for a real mode constant fails to compile here - loudly - rather
 * than silently running against invented values. If a host test ever needs to
 * exercise wr-accel mode selection, check the submodule out and drop this stub.
 */
#ifndef HOST_TEST_STUB_LIS2DW12_REG_H
#define HOST_TEST_STUB_LIS2DW12_REG_H

#include <stdint.h>

typedef enum
{
  LIS2DW12_MODE_STUB_UNSET = 0
} lis2dw12_mode_t;

#endif /* HOST_TEST_STUB_LIS2DW12_REG_H */
