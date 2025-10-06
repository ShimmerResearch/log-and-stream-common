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

/* variables used for delayed undock-start */
uint8_t undockEvent;
uint64_t time_newUnDockEvent;

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

  /* variables used for delayed undock-start */
  undockEvent = 0;
  time_newUnDockEvent = 0;

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

  if (shimmerStatus.booting)
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

  if (!undockEvent)
  {
    if (!shimmerStatus.docked) //undocked
    {
      undockEvent = 1;
      ShimBatt_setBattCritical(0);
      time_newUnDockEvent = RTC_get64();
    }
  }

  if (shimmerStatus.docked)
  {
    shimmerStatus.sdlogReady = 0;
  }
  else
  {
    shimmerStatus.sdlogReady = LogAndStream_checkSdInSlot() && !shimmerStatus.sdBadFile;
  }
}

void LogAndStream_infomemUpdate(void)
{
#if defined(SHIMMER3)
  InfoMem_update(ShimConfig_getStoredConfig()->rawBytes);
#else
  InfoMem_update(ShimConfig_getStoredConfig()->rawBytes, ShimCalib_getBytesPtr());
#endif
}

void LogAndStream_checkSetupDockUnDock(void)
{
  uint8_t dockedStateSaved = shimmerStatus.docked;
  if (!shimmerStatus.configuring && !sensing.inSdWr
      && (dockedStateSaved || ((RTC_get64() - time_newUnDockEvent) > TIMEOUT_100_MS)))
  {
    LogAndStream_setupDockUndock();

    /* If Shimmer is docking status has changed while dock was being setup, set
     * task again. */
    if (dockedStateSaved != BOARD_IS_DOCKED)
    {
      ShimTask_set(TASK_SETUP_DOCK);
    }

    undockEvent = 0;
  }
  else
  {
    ShimTask_set(TASK_SETUP_DOCK);
  }

  ShimRtc_rwcErrorCheck();
}

void LogAndStream_setupDockUndock(void)
{
  shimmerStatus.configuring = 1;

#if TEST_UNDOCKED
  if (0)
#else
  if (LogAndStream_isDockedOrUsbIn())
#endif
  {
    LogAndStream_setupDock();
  }
  else
  {
    LogAndStream_setupUndock();
  }

  ShimTask_set(TASK_BATT_READ);

#if defined(SHIMMER3R)
  //Setup RTC alarm for battery read after dock/undock
  RTC_setAlarmBattReadAfterDockUnDock();
#endif

  shimmerStatus.configuring = 0;
}

void LogAndStream_setupDock(void)
{
  //Only one of these conditions needs to be true to stop loggin
  shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_STOP;
  shimmerStatus.sdlogReady = 0;
  ShimSens_stopSensing(0);

  /* Prioritise dock over USB for SD card access */
  if (shimmerStatus.docked)
  {
    if (LogAndStream_checkSdInSlot())
    {
      Board_sd2Pc();
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

  ShimBatt_setBatteryInterval(BATT_INTERVAL_SECS_DOCKED);
  /* Reset battery critical count on dock to allow logging to begin again if
   * auto-stop on low-power is enabled. */
  ShimBatt_resetBatteryCriticalCount();

  ShimBt_instreamStatusRespSend();
}

void LogAndStream_setupUndock(void)
{
  ShimBatt_setBatteryInterval(BATT_INTERVAL_SECS_UNDOCKED);
  DockUart_deinit();

  /* Set dock detect high to let dock know SD card is not available and kill
   * power to SD card. It will get turned back on as part of power cycle if
   * SD is still inserted */
  Board_dockDetectN(1);
  Board_setSdPower(0);

  ShimBt_instreamStatusRespSend();

  if (LogAndStream_checkSdInSlot())
  {
    Board_sd2Mcu();

    //Set sdlogReady flag if SD card is present and no bad file
    shimmerStatus.sdlogReady = !shimmerStatus.sdBadFile;

    //If in the middle of booting,
    if (!shimmerStatus.booting)
    {
      if (!shimmerStatus.sensing)
      {
        delay_ms(120); //120ms
        LogAndStream_syncConfigAndCalibOnSd();
      }
      else
      {
        LogAndStream_setSdInfoSyncDelayed(1);
      }
    }
  }
}

uint8_t LogAndStream_checkSdInSlot(void)
{
  //Check if card is inserted and enable interrupt for SD_DETECT_N
  shimmerStatus.sdInserted = Board_isSdInserted();
  return shimmerStatus.sdInserted;
}

__weak void delay_ms(const uint32_t delay_time_ms)
{
  //This function can be overridden by the main application to provide a custom
  //delay implementation. The default implementation does nothing.
  (void) delay_time_ms; //Suppress unused parameter warning
}
