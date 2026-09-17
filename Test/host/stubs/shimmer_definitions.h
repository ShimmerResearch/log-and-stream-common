/*
 * Host-test stub.
 *
 * The real shimmer_definitions.h is firmware-side
 * (shimmer3r-firmware/LogAndStream_Shimmer3R/Shimmer_Driver/), and
 * Configuration/shimmer_config.h includes it under SHIMMER3R. Deliberately
 * empty: nothing this repository compiles on a host takes a symbol from it, and
 * mirroring a firmware header that nothing here reads would only create drift.
 *
 * If a host build starts failing on an undeclared symbol that lives in the real
 * one, add just that symbol, with a comment naming where it came from.
 */
#ifndef HOST_TEST_STUB_SHIMMER_DEFINITIONS_H
#define HOST_TEST_STUB_SHIMMER_DEFINITIONS_H
#endif
