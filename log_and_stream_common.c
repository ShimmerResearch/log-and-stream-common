/*
 * log_and_stream_common.c
 *
 *  Created on: 26 Mar 2025
 *      Author: MarkNolan
 */

#include "log_and_stream_common.h"

#include "hal_Board.h"
#include "log_and_stream_includes.h"

boot_stage_t bootStage;

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
  ShimLeds_varsInit();

  LogAndStream_setSdInfoSyncDelayed(0);

  memset((uint8_t *) &shimmerStatus, 0, sizeof(STATTypeDef));

  ShimTask_set(TASK_BATT_READ);
}

void setBootStage(boot_stage_t bootStageNew)
{
  bootStage = bootStageNew;

  switch (bootStage)
  {
    case BOOT_STAGE_START:
      Board_ledOn(LED_ALL);
      break;
    case BOOT_STAGE_I2C:
      Board_ledOff(LED_ALL);
      break;
    case BOOT_STAGE_BLUETOOTH:
      Board_ledOn(LED_ALL);
      break;
    case BOOT_STAGE_BLUETOOTH_FAILURE:
      Board_ledOff(LED_ALL);
      break;
    case BOOT_STAGE_CONFIGURATION:
      Board_ledOn(LED_ALL);
      break;
    case BOOT_STAGE_END:
      Board_ledOff(LED_ALL);
      break;
    default:
      break;
  }
  return;
}

boot_stage_t getBootStage(void)
{
  return bootStage;
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

void LogAndStream_blinkTimerCommon(void)
{
  ShimLeds_incrementCounters();

  if (shimmerStatus.initialising)
  {
    ShimLeds_controlDuringBoot(bootStage);
  }
  else
  {
    if (ShimLeds_isBlinkTimerCnt1s() && ShimConfig_checkAutostopCondition())
    {
      ShimTask_setStopSensing();
      ShimBt_instreamStatusRespPendingSet(1);
    }

    if (shimmerStatus.timerBlinkEnabled)
    {
      ShimLeds_blink();
    }
  }
}

uint8_t LogAndStream_isDockedOrUsbIn(void)
{
  return shimmerStatus.docked || shimmerStatus.usbPluggedIn;
}
