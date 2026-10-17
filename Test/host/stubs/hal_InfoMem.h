/*
 * Host-test stub. log_and_stream_externs.h includes this under SHIMMER3.
 * InfoMem_read()/InfoMem_write() are declared as externs there; host_stubs.c
 * defines them as no-ops.
 *
 * SHIMMER3R'S IS A DIFFERENT FILE WITH NEARLY THE SAME NAME. Shimmer3R's
 * InfoMem header is hal_Infomem.h, lower-case m, and its stub is
 * Shimmer_Driver/hal_Infomem.h. Linux keeps the two apart. Windows and macOS
 * file systems ignore case, so there "hal_Infomem.h" finds this file first,
 * through -I stubs. The forward below sends a Shimmer3R build on to its own
 * stub, so both platforms see the same definitions on every host OS.
 */
#ifndef HOST_TEST_STUB_HAL_INFOMEM_H
#define HOST_TEST_STUB_HAL_INFOMEM_H

#if defined(SHIMMER3R)
#include "Shimmer_Driver/hal_Infomem.h"
#else

#include <stdint.h>

#define INFOMEM_SEG_D_ADDR 0x1800
#define INFOMEM_SEG_C_ADDR 0x1880
#define INFOMEM_SEG_B_ADDR 0x1900
#define INFOMEM_SEG_A_ADDR 0x1980
#define INFOMEM_SEG_SIZE   128

#endif

#endif /* HOST_TEST_STUB_HAL_INFOMEM_H */
