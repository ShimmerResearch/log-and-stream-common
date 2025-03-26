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
void ShimUtil_ItoaNo0(uint64_t num, char *buf, uint8_t max_len);
uint64_t ShimUtil_Atol64(uint8_t *buf);
volatile void *ShimUtil_memset_v(volatile void *dest, uint8_t value, size_t n);
volatile void *ShimUtil_memcpy_v(volatile void *dest, const void *src, size_t n);
volatile void *ShimUtil_memcpy_vv(volatile void *dest, volatile void *src, size_t n);
size_t ShimUtil_strlen_v(volatile void *dest, size_t maxSize);

#endif /* LOG_AND_STREAM_COMMON_UTIL_SHIMMER_UTIL_H_ */
