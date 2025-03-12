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

#define FATFS_V_0_08B     8237
#define FATFS_V_0_12C     68300

//sd card write buffer size
#define SD_WRITE_BUF_SIZE 512

#if defined(SHIMMER3)
#define NUM_SDWRBUF 1
#elif defined(SHIMMER3R)
#define NUM_SDWRBUF 64
#endif

#define BIN_FILE_SPLIT_TIME_TICKS (32768 * 3600) //1 hr
#define BIN_FILE_SYNC_TIME_TICKS  (32768 * 60)   //1 minute

void ShimSd_init(void);
uint8_t ShimSd_test1(void);
uint8_t ShimSd_test2(void);
void ShimSd_setShimmerName(void);
void ShimSd_setExpIdName(void);
void ShimSd_setCfgTime(void);
void ShimSd_setName(void);
void ShimSd_infomem2Names(void);
uint8_t ShimSd_setBasedir(void);
uint8_t ShimSd_makeBasedir(void);
void ShimSd_makeFileName(char *name_buf);
void ShimSd_fileInit(void);
void ShimSd_close(void);
void ShimSd_writeToBuff(uint8_t *buf, uint16_t len);
void ShimSd_writeToCard(void);
FRESULT ShimSd_mount(uint8_t val);
void ShimSd_updateSdConfig(void);
void ShimSd_parseConfig(void);

uint8_t ShimSd_isFileStatusOk(void);
uint8_t ShimSd_isSdInfoSyncDelayed(void);
void ShimSd_setSdInfoSyncDelayed(uint8_t state);
uint8_t *ShimSd_configTimeTextPtrGet(void);
uint8_t *ShimSd_fileNamePtrGet(void);
uint8_t *ShimSd_shimmerNamePtrGet(void);
uint8_t *ShimSd_expIdPtrGet(void);
FRESULT ShimSd_setFileTimestamp(char *path);
void ShimSd_findError(uint8_t err, uint8_t *name);
void ShimSd_sdInfoSync();
void ShimSd_readSdConfiguration(void);

#endif /* SHIMMER_SD_H_ */
