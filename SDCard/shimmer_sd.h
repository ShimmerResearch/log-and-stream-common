/*
 * shimmer_sd.h
 *
 *  Created on: 23 Oct 2025
 *      Author: MarkNolan
 */

#ifndef LOG_AND_STREAM_COMMON_SDCARD_SHIMMER_SD_H_
#define LOG_AND_STREAM_COMMON_SDCARD_SHIMMER_SD_H_

#include <stdint.h>

#if defined(SHIMMER3)
#include "ff.h"
#elif defined(SHIMMER3R)
#if USE_FATFS
#include "ff.h"
#else
#include "fx_api.h"
#endif
#endif

#define FATFS_V_0_08B       8237
#define FATFS_V_0_12C       68300

/* Tuneable delay for the USB-SD bridge to settle before CD assert */
#define SD_PC_STABILIZE_MS  300 /* 300–500ms has proven robust on macOS */
#define SD_MCU_STABILIZE_MS 50  /* 50ms has proven robust */

typedef enum
{
  SD_UNMOUNT,
  SD_MOUNT
} sd_mount_state_t;

enum
{
  DOCK_CARD_PRESENT,
  DOCK_CARD_NOT_PRESENT
};

void ShimSd_init(void);
uint8_t ShimSd_test1(void);
uint8_t ShimSd_test2(void);
FRESULT ShimSd_mount(sd_mount_state_t val);
FRESULT ShimSd_setFileTimestamp(char *path);
void ShimSd_findError(uint8_t err, uint8_t *name);

#endif /* LOG_AND_STREAM_COMMON_SDCARD_SHIMMER_SD_H_ */
