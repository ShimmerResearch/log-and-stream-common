/*
 * shimmer_sd_cfg_file.h
 *
 *  Created on: Mar 26, 2025
 *      Author: MarkNolan
 */

#ifndef SDCARD_SHIMMER_SD_CFG_FILE_H_
#define SDCARD_SHIMMER_SD_CFG_FILE_H_

#include "ff.h"

void ShimSdCfgFile_init(void);
void ShimSdCfgFile_generate(void);
void ShimSdCfgFile_parse(void);
void ShimSdCfgFile_readSdConfiguration(void);
/* The event file borrows sdlog.cfg's file object for a logging session, when
 * sdlog.cfg is never read or written; Shimmer3 has no RAM for another. While
 * it is lent, generate() and parse() leave the card alone. Returns 0 if it is
 * already lent. */
FIL *ShimSdCfgFile_lendFil(void);
void ShimSdCfgFile_returnFil(void);

#endif /* SDCARD_SHIMMER_SD_CFG_FILE_H_ */
