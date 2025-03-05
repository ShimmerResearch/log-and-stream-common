/*
 * shimmer_util.h
 *
 *  Created on: 4 Mar 2025
 *      Author: MarkNolan
 */

#ifndef LOG_AND_STREAM_COMMON_UTIL_SHIMMER_UTIL_H_
#define LOG_AND_STREAM_COMMON_UTIL_SHIMMER_UTIL_H_

#include <stdint.h>

void ShimUtil_ItoaWith0(uint64_t num, uint8_t *buf, uint8_t len);
void ShimUtil_ItoaNo0(uint64_t num, uint8_t *buf, uint8_t max_len);

#endif /* LOG_AND_STREAM_COMMON_UTIL_SHIMMER_UTIL_H_ */
