/*
 * shimmer_sd_cfg_file.h
 *
 *  Created on: Mar 26, 2025
 *      Author: MarkNolan
 */

#ifndef SDCARD_SHIMMER_SD_CFG_FILE_H_
#define SDCARD_SHIMMER_SD_CFG_FILE_H_

#include <stdint.h>

void ShimSdCfgFile_init(void);
void ShimSdCfgFile_generate(void);
void ShimSdCfgFile_parse(void);
void ShimSdCfgFile_sync();
void ShimSdCfgFile_readSdConfiguration(void);
uint8_t ShimSdCfgFile_isSdInfoSyncDelayed(void);
void ShimSdCfgFile_setSdInfoSyncDelayed(uint8_t state);

#endif /* SDCARD_SHIMMER_SD_CFG_FILE_H_ */
