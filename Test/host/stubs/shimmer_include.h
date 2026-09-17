/*
 * Host-test stub for the Shimmer3R firmware's own aggregate header.
 *
 * The real one lives in shimmer3r-firmware
 * (LogAndStream_Shimmer3R/Shimmer_Driver/shimmer_include.h) and pulls in CMSIS
 * and the STM32 HAL. Configuration/shimmer_config.h and
 * Sensing/shimmer_sensing.h include it under SHIMMER3R.
 *
 * Almost nothing is mirrored here deliberately - see
 * stubs/shimmer_definitions.h for why. The one exception is below.
 */
#ifndef HOST_TEST_STUB_SHIMMER_INCLUDE_H
#define HOST_TEST_STUB_SHIMMER_INCLUDE_H

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
