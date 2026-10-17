/*
 * Host-test stub for the Shimmer3R InfoMem (on-chip flash) HAL.
 *
 * Real header: shimmer3r-firmware
 * LogAndStream_Shimmer3R/Shimmer_Driver/hal_Infomem.h. The firmware's
 * shimmer_include.h includes it, and so does Calibration/shimmer_calibration.c
 * directly. Calibration/shimmer_calibration.h sizes the calibration RAM by
 * INFOMEM_CALIB_SIZE on Shimmer3R, so that value is load-bearing: every bound
 * the calibration suite checks is measured against it.
 *
 * InfoMem_write() is declared in log_and_stream_externs.h with the same
 * signature as the real header's, and host_stubs.c defines it.
 *
 * On a case-insensitive file system "hal_Infomem.h" finds the Shimmer3 stub
 * ../hal_InfoMem.h first; that file forwards here. See its comment.
 */
#ifndef HOST_TEST_STUB_HAL_INFOMEM_S3R_H
#define HOST_TEST_STUB_HAL_INFOMEM_S3R_H

#define INFOMEM_CALIB_SIZE 0x400 /* 1024 B */

#endif /* HOST_TEST_STUB_HAL_INFOMEM_S3R_H */
