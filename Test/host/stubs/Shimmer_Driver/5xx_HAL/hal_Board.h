/*
 * Host-test stub for the Shimmer3 board HAL.
 *
 * Real header: shimmer3-firmware Shimmer_Driver/5xx_HAL/hal_Board.h.
 * Calibration/shimmer_calibration.c includes it as "../5xx_HAL/hal_Board.h"
 * under SHIMMER3 but takes nothing from it, so this is empty - an include
 * anchor and no more.
 *
 * Not to be confused with ../../hal_Board.h, the stub Battery/ resolves as
 * plain "hal_Board.h" for its LED colour constants. -I stubs comes first, so
 * that one is still what Battery/ sees.
 */
#ifndef HOST_TEST_STUB_5XX_HAL_BOARD_H
#define HOST_TEST_STUB_5XX_HAL_BOARD_H
#endif /* HOST_TEST_STUB_5XX_HAL_BOARD_H */
