/*
 * Host-test stub for the Shimmer3R firmware's own aggregate header.
 *
 * The real one lives in shimmer3r-firmware
 * (LogAndStream_Shimmer3R/Shimmer_Driver/shimmer_include.h) and pulls in CMSIS
 * and the STM32 HAL. Configuration/shimmer_config.h, Sensing/shimmer_sensing.h
 * and Calibration/shimmer_calibration.h include it under SHIMMER3R.
 *
 * Almost nothing is mirrored here deliberately - see
 * stubs/shimmer_definitions.h for why. The exceptions are below.
 */
#ifndef HOST_TEST_STUB_SHIMMER_INCLUDE_H
#define HOST_TEST_STUB_SHIMMER_INCLUDE_H

/* Four of the headers the real aggregate includes, for what
 * Calibration/shimmer_calibration.h and .c take from them: the calibration RAM
 * size (INFOMEM_CALIB_SIZE) and the sensor full-scale values each default
 * calibration is keyed on. Each is a stub under Shimmer_Driver/, and each says
 * where its values come from. */
#include "LIS2DW12/lis2dw12-pid/lis2dw12_reg.h"
#include "LIS3MDL/lis3mdl-pid/lis3mdl_reg.h"
#include "LSM6DSV/lsm6dsv-pid/lsm6dsv_reg.h"
#include "hal_Infomem.h"

/* __NOP() is a CMSIS intrinsic on Shimmer3R and a macro over __no_operation()
 * on Shimmer3 (see log_and_stream_definitions.h). Button/shimmer_button.c uses
 * it in its debounce path. A host has neither, and an empty statement is the
 * right stand-in: the tests here assert state transitions, not timing. */
#ifndef __NOP
#define __NOP() \
  do            \
  {             \
  } while (0)
#endif

#endif /* HOST_TEST_STUB_SHIMMER_INCLUDE_H */
