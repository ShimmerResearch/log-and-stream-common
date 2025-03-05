/*
 * s4sd.h
 *
 *  Created on: Mar 21, 2024
 *      Author: MarkNolan
 */

#ifndef SHIMMER_SD_H_
#define SHIMMER_SD_H_

#include <stdint.h>

#include "ff.h"

#define FATFS_V_0_08B 8237
#define FATFS_V_0_12C 68300

// sd card write buffer size
#define SD_WRITE_BUF_SIZE               512

#if defined(SHIMMER3)
#define NUM_SDWRBUF           1
#elif defined(SHIMMER3R)
#define NUM_SDWRBUF           64
#endif

#define BIN_FILE_SPLIT_TIME_TICKS (32768 * 3600) // 1 hr
#define BIN_FILE_SYNC_TIME_TICKS  (32768 * 60)  // 1 minute

void SD_init(void);
uint8_t SD_test1(void);
uint8_t SD_test2(void);
void SD_setShimmerName(void);
void SD_setExpIdName(void);
void SD_setCfgTime(void);
void SetName(void);
void SD_infomem2Names(void);
uint8_t SD_setBasedir(void);
uint8_t SD_makeBasedir(void);
void SD_makeFileName(char *name_buf);
void SD_fileInit(void);
void SD_close(void);
void SD_writeToBuff(uint8_t *buf, uint16_t len);
void SD_writeToCard(void);
//void SD_config2SdHead(void);
FRESULT SD_mount(uint8_t val);
void UpdateSdConfig(void);
void ParseConfig(void);

uint8_t isFileStatusOk(void);
uint8_t isSdInfoSyncDelayed(void);
void setSdInfoSyncDelayed(uint8_t state);
uint8_t *getConfigTimeTextPtr(void);
uint8_t *getFileNamePtr(void);
uint8_t *getShimmerNamePtr(void);
uint8_t *getExpIdPtr(void);
FRESULT set_file_timestamp(char *path);
void FindError(uint8_t err, uint8_t *name);

#endif /* SHIMMER_SD_H_ */
