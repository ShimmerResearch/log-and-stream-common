/*
 * Host-test stub. log_and_stream_externs.h includes this under SHIMMER3.
 * InfoMem_read()/InfoMem_write() are declared as externs there; host_stubs.c
 * defines them as no-ops.
 */
#ifndef HOST_TEST_STUB_HAL_INFOMEM_H
#define HOST_TEST_STUB_HAL_INFOMEM_H

#include <stdint.h>

#define INFOMEM_SEG_D_ADDR 0x1800
#define INFOMEM_SEG_C_ADDR 0x1880
#define INFOMEM_SEG_B_ADDR 0x1900
#define INFOMEM_SEG_A_ADDR 0x1980
#define INFOMEM_SEG_SIZE   128

#endif /* HOST_TEST_STUB_HAL_INFOMEM_H */
