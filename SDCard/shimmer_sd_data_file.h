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

//sd card write buffer size
#define SD_WRITE_BUF_SIZE 512

#if defined(SHIMMER3)
#define NUM_SDWRBUF 1
#elif defined(SHIMMER3R)
#define NUM_SDWRBUF 4
#endif

#define BIN_FILE_SPLIT_TIME_TICKS (32768 * 3600) //1 hr
#define BIN_FILE_SYNC_TIME_TICKS  (32768 * 60)   //1 minute

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

#endif /* SHIMMER_SD_H_ */
