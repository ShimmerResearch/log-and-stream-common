/*
 * shimmer_globals.h
 *
 *  Created on: Sep 23, 2024
 *      Author: MarkNolan
 */

#ifndef SHIMMER_GLOBALS_H_
#define SHIMMER_GLOBALS_H_

/* Declarations only: shimmerStatus and batteryStatus are defined once, in
 * log_and_stream_common.c. A definition here would be repeated in every file
 * that includes this header, which links only while the toolchain merges
 * duplicate tentative definitions and fails under program-level or link-time
 * optimisation. */
#include "log_and_stream_externs.h"

#endif /* SHIMMER_GLOBALS_H_ */
