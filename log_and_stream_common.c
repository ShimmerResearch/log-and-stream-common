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
  ShimBrd_init();
  ShimEeprom_init();
  ShimLeds_varsInit();
  ShimBtn_init();
  ShimRtc_init();

  LogAndStream_setSdInfoSyncDelayed(0);

  memset((uint8_t *) &shimmerStatus, 0, sizeof(STATTypeDef));

  ShimTask_set(TASK_BATT_READ);
}

void LogAndStream_setBootStage(boot_stage_t bootStageNew)
{
  bootStage = bootStageNew;

  //Reset the boot stage time
  shimmerStatus.bootTimePerStageMs = 0;
}

boot_stage_t LogAndStream_getBootStage(void)
{
  return bootStage;
}

void LogAndStream_syncConfigAndCalibOnSd(void)
{
  LogAndStream_setSdInfoSyncDelayed(0);
  if (ShimConfig_getFlagWriteCfgToSd())
  { //info > sdcard
    ShimConfig_readRam();
    ShimSdCfgFile_generate();
    ShimConfig_setFlagWriteCfgToSd(0, 1);
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
      ShimCalib_calibDumpToConfigBytesAndSdHeaderAll(1);
    }
  }

  ShimSens_configureChannels();
  ShimSens_startLoggingIfUndockStartEnabled();
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
    if (ShimLeds_isBlinkTimerCnt1s() && ShimSens_checkAutostopLoggingCondition())
    {
      ShimTask_setStopLogging();
      ShimTask_setStopSensing();
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

void LogAndStream_dockedStateChange(void)
{
  /* Reset battery charging status on dock/undock so that we don't display an
   * invalid state. Called directly here to set correct LED state while waiting
   * for TASK_SETUP_DOCK to trigger. */
  ShimBatt_resetBatteryChargingStatus();
  ShimTask_set(TASK_SETUP_DOCK);
}

void LogAndStream_infomemUpdate(void)
{
#if defined(SHIMMER3)
  InfoMem_update(ShimConfig_getStoredConfig()->rawBytes);
#else
  InfoMem_update(ShimConfig_getStoredConfig()->rawBytes, ShimCalib_getBytesPtr());
#endif
}

void LogAndStream_setupDock(void)
{
  shimmerStatus.configuring = 1;

  /* Reset battery charging status on dock/undock so that we don't display an
   * invalid state. */
  ShimBatt_resetBatteryChargingStatus();

  if (LogAndStream_isDockedOrUsbIn())
  {
    //Only one of these conditions needs to be true to stop loggin
    shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_STOP;
    shimmerStatus.sdlogReady = 0;
    ShimSens_stopSensing(0);

    /* Prioritise dock over USB for SD card access */
    if (shimmerStatus.docked)
    {
      if (CheckSdInslot())
      {
#if defined(SHIMMER3)
        DockSdPowerCycle();
#else
        Board_sd2Pc();
#endif
      }
      if (!shimmerStatus.sensing)
      {
        DockUart_init();
      }
    }
    else
    {
      DockUart_deinit();
    }

    ShimBatt_setBatteryInterval(BATT_INTERVAL_DOCKED);
    /* Reset battery critical count on dock to allow logging to begin again if
     * auto-stop on low-power is enabled. */
    ShimBatt_resetBatteryCriticalCount();

    ShimBt_instreamStatusRespSend();
  }
  else
  {
    ShimBatt_setBatteryInterval(BATT_INTERVAL_UNDOCKED);
    DockUart_deinit();

#if defined(SHIMMER3)
    Board_detectN(1); //Set DETECT_N high
#endif
    ShimBt_instreamStatusRespSend();
#if defined(SHIMMER3)
    SdPowerOff();
#endif
    if (CheckSdInslot())
    {
#if defined(SHIMMER3R)
      Board_sd2Arm();
#endif

      //Set sdlogReady flag if SD card is present and no bad file
      shimmerStatus.sdlogReady = !shimmerStatus.sdBadFile;

      if (!shimmerStatus.sensing)
      {
        delay_ms(120); //120ms
#if defined(SHIMMER3)
        SdPowerOn();
#endif
        LogAndStream_syncConfigAndCalibOnSd();
      }
      else
      {
        LogAndStream_setSdInfoSyncDelayed(1);
      }
    }
  }

#if defined(SHIMMER3R)
  //Setup RTC alarm for battery read after dock/undock
  RTC_setAlarmBattReadAfterDockUnDock();
#endif

  shimmerStatus.configuring = 0;
}

__weak void delay_ms(const uint32_t delay_time_ms)
{
  //This function can be overridden by the main application to provide a custom
  //delay implementation. The default implementation does nothing.
  (void) delay_time_ms; //Suppress unused parameter warning
}
