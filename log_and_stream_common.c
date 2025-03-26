/*
 * log_and_stream_common.c
 *
 *  Created on: 26 Mar 2025
 *      Author: MarkNolan
 */

#include "log_and_stream_common.h"

#include "log_and_stream_includes.h"

uint8_t sdInfoSyncDelayed = 0;

void LogAndStream_init(void)
{
  ShimTask_init();
  ShimBatt_init();
  ShimConfig_reset();
  ShimSdDataFile_init();
  ShimSdCfgFile_init();
  ShimSdHead_reset();
  ShimSens_init();
  ShimDock_resetVariables();
  ShimBrd_resetDaughterCardId();

  LogAndStream_setSdInfoSyncDelayed(0);

  memset((uint8_t *) &shimmerStatus, 0, sizeof(STATTypeDef));

  ShimTask_set(TASK_BATT_READ);
}

void LogAndStream_syncConfigAndCalibOnSd(void)
{
  LogAndStream_setSdInfoSyncDelayed(0);
  if (ShimConfig_getSdCfgFlag())
  { //info > sdcard
    ShimConfig_readRam();
    ShimSdCfgFile_generate();
    ShimConfig_setSdCfgFlag(0);
  }
  else
  {
    ShimSdCfgFile_readSdConfiguration();
  }

  if (ShimConfig_getRamCalibFlag())
  {
    ShimCalib_ram2File();
    ShimConfig_setRamCalibFlag(0);
  }
  else
  {
    if (ShimCalib_file2Ram())
    {
      ShimCalib_ram2File();
    }
    else
    {
      //only need to do this when file2Ram succeeds
      ShimCalib_calibDumpToConfigBytesAndSdHeaderAll();
    }
  }

  ShimSens_configureChannels();
  ShimSens_checkOnDefault();
}

uint8_t LogAndStream_isSdInfoSyncDelayed(void)
{
  return sdInfoSyncDelayed;
}

void LogAndStream_setSdInfoSyncDelayed(uint8_t state)
{
  sdInfoSyncDelayed = state;
}
