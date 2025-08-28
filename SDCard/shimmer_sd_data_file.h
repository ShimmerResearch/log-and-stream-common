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

#define TEST_TEXT_LEN 40

void ShimSdDataFile_init(void);
void ShimSdDataFile_resetVars(void);
void ShimSdDataFile_setFileNameIfEmpty(void);
uint8_t ShimSdDataFile_setBasedir(void);
uint8_t ShimSdDataFile_makeBasedir(void);
void ShimSdDataFile_makeFileName(char *name_buf);
void ShimSdDataFile_fileInit(void);
void ShimSdDataFile_close(void);
void ShimSdDataFile_writeToBuff(uint8_t *buf, uint16_t len);
void ShimSdDataFile_writeToCard(void);
void ShimSdDataFile_writeAllBufsToSd(void);
void ShimSdDataFile_advanceSensingBuf(void);
uint8_t ShimSdDataFile_getNumberOfFullBuffers(void);
uint16_t ShimSdDataFile_getBytesInCurrentSensingBuffer(void);

uint8_t ShimSdDataFile_isFileStatusOk(void);

uint8_t *ShimSdDataFile_fileNamePtrGet(void);
void ShimSdDataFile_prepareSDBuffHead(void);

//TODO move to general ShimSd source file?
uint8_t ShimSd_test1(void);
uint8_t ShimSd_test2(void);
FRESULT ShimSd_mount(uint8_t val);
FRESULT ShimSd_setFileTimestamp(char *path);
void ShimSd_findError(uint8_t err, uint8_t *name);

#endif /* SHIMMER_SD_H_ */
