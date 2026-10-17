/*
 * Host-test stub for the firmware version header.
 *
 * Each firmware generates its own version.h at release
 * (shimmer3-firmware LogAndStream_Shimmer3/Shimmer_Driver/version.h,
 * shimmer3r-firmware LogAndStream_Shimmer3R/Shimmer_Driver/version.h), and
 * Calibration/shimmer_calibration.c includes it for the version bytes it writes
 * into the calibration dump header. The values below are the ones each
 * firmware's main branch carried when this stub was written. They move with
 * every release, and that drift is harmless: nothing under host test asserts
 * a version number, and a test that reads one reads it back through these same
 * macros.
 */
#ifndef HOST_TEST_STUB_VERSION_H
#define HOST_TEST_STUB_VERSION_H

#if defined(SHIMMER3)
#define FW_VERSION_MAJOR  1
#define FW_VERSION_MINOR  1
#define FW_VERSION_PATCH  6
#define FW_VERSION_STRING "v1.01.006"
#elif defined(SHIMMER3R)
#define FW_VERSION_MAJOR  1
#define FW_VERSION_MINOR  1
#define FW_VERSION_PATCH  21
#define FW_VERSION_STRING "v1.01.021"
#endif

#endif /* HOST_TEST_STUB_VERSION_H */
