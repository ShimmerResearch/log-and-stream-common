/*
 * Host-test stub.
 *
 * Both firmwares carry a real shimmer_definitions.h
 * (shimmer3-firmware/LogAndStream_Shimmer3/Shimmer_Driver/ and
 * shimmer3r-firmware/LogAndStream_Shimmer3R/Shimmer_Driver/).
 * Configuration/shimmer_config.h includes it under SHIMMER3R, and
 * Calibration/shimmer_calibration.c on both platforms. Almost nothing is
 * mirrored: mirroring a firmware header that nothing here reads would only
 * create drift.
 *
 * If a host build starts failing on an undeclared symbol that lives in the real
 * one, add just that symbol, with a comment naming where it came from.
 */
#ifndef HOST_TEST_STUB_SHIMMER_DEFINITIONS_H
#define HOST_TEST_STUB_SHIMMER_DEFINITIONS_H

/* Both real headers include it. */
#include "version.h"

/* Hardware and firmware IDs, from each firmware's own shimmer_definitions.h.
 * Calibration/ writes them into the calibration dump header. */
#if defined(SHIMMER3)
#define DEVICE_VER    3
#define FW_IDENTIFIER 3
#elif defined(SHIMMER3R)
#define DEVICE_VER    10
#define FW_IDENTIFIER 3
#endif

#endif
