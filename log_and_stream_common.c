/*
 * log_and_stream_common.c
 *
 *  Created on: 26 Mar 2025
 *      Author: MarkNolan
 */

#include "log_and_stream_common.h"

#include <stdio.h>
#include <string.h>

#include "Platform/platform_api.h"
#include "hal_Board.h"
#include "log_and_stream_includes.h"

boot_stage_t bootStage;

uint8_t sdInfoSyncDelayed = 0;

/* variables used for delayed undock-start */
uint8_t undockEvent;
uint64_t time_newUnDockEvent;

static uint32_t g_dock_usb_last_tick = 0U;
#define DOCK_USB_DEBOUNCE_MS 50U

void LogAndStream_init(void)
{
  ShimTask_init();
  ShimBatt_init();
  ShimConfig_reset();
  ShimSd_init();
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
#if TEST_TASK_MONITOR
  static uint8_t stuckCount = 0;
#endif //TEST_TASK_MONITOR

  ShimLeds_incrementCounters();

  if (shimmerStatus.booting)
  {
    ShimLeds_controlDuringBoot(bootStage);
  }
  else
  {
#if TEST_TASK_MONITOR
    /* If a task appears to be executing for too long, blinking stops
       and escalate to a reset after repeated timer ticks. */
    if (ShimTask_getExecutingTask())
    {
      uint32_t start = ShimTask_getExecStartTick();
      uint32_t elapsed = (platform_getTick() - start);

      if (elapsed > TASK_STUCK_TIMEOUT)
      {
        /* first hit: turn off LEDs except lower red */
        if (stuckCount == 0)
        {
          Board_ledOff(LED_ALL);
          Board_ledOn(LED_LWR_RED);
        }
        else
        {
          /* subsequent hits: toggle lower red and green */
          Board_ledToggle(LED_LWR_RED);
          Board_ledToggle(LED_LWR_GREEN);
        }

        stuckCount++;

        /* escalate to system reset after further period of time */
        if (elapsed > TASK_STUCK_RESET_TIMEOUT)
        {
          platform_reset(); /* recover by reboot */
        }

        /* do not run normal blink processing while stuck */
        return;
      }
      else
      {
        /* task is within allowed time, clear counter */
        stuckCount = 0;
      }
    }
#endif //TEST_TASK_MONITOR

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

  if (!undockEvent && !LogAndStream_isDockedOrUsbIn())
  {
    undockEvent = 1;
    ShimBatt_setBattCritical(0);
    time_newUnDockEvent = RTC_get64();
  }

  if (shimmerStatus.docked || shimmerStatus.usbPluggedIn)
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

/**
 * @brief  Check docked state and update shimmerStatus.docked, return whether state has changed
 * @param  none
 * @return 1 if docked state has changed, 0 if not
 */
uint8_t LogAndStream_updateDockedStateAndCheckChanged(void)
{
  uint8_t prevDocked = shimmerStatus.docked;
#if TEST_UNDOCKED
  shimmerStatus.docked = 0;
#else  //TEST_UNDOCKED
  shimmerStatus.docked = Board_isDocked();
#endif //TEST_UNDOCKED
  return prevDocked != shimmerStatus.docked;
}

/**
 * @brief Unified debounced handler for dock (and USB-VBUS on Shimmer3R)
 *        state changes.
 *
 * Called from the task runner (TASK_DOCK_OR_USB_STATE_CHANGE) so that
 * DOCK_DETECT (and USB_VBUS on Shimmer3R) interrupts are funnelled through
 * a single deferred path with debounce.  Re-reads the relevant pin(s) after
 * the debounce window, updates shimmerStatus, and triggers the dock/undock
 * sequence only when something has actually changed (or on first boot).
 */
void LogAndStream_dockOrUsbStateUpdate(void)
{
  const uint32_t now = platform_getTick();

  /* Only apply time-based debounce when the platform tick source is valid.
   * Some builds may fall back to a weak platform_getTick() that returns 0,
   * and repeatedly re-queueing in that case can livelock the main loop.
   */
  if (now != 0U)
  {
    if ((now - g_dock_usb_last_tick) < DOCK_USB_DEBOUNCE_MS)
    {
      /* Still inside the debounce window – reschedule so we come back later */
      ShimTask_setDockOrUsbStateChange();
      return;
    }
    g_dock_usb_last_tick = now;
  }

  /* --- Sample pin(s) --- */
  /* Check docked state and update shimmerStatus.docked, also return whether state has changed since last check. */
  uint8_t dockChanged = LogAndStream_updateDockedStateAndCheckChanged();

#if defined(SHIMMER3R)
  uint8_t prevUsb = shimmerStatus.usbPluggedIn;
  shimmerStatus.usbPluggedIn = Board_isUsbPluggedIn();
  uint8_t usbChanged = (prevUsb != shimmerStatus.usbPluggedIn);
#endif

  if (dockChanged
#if defined(SHIMMER3R)
      || usbChanged
#endif
      || shimmerStatus.booting)
  {
    LogAndStream_dockedStateChange();
  }
}

void LogAndStream_checkSetupDockUnDock(void)
{
  uint8_t dockedOrUsbStateSaved = LogAndStream_isDockedOrUsbIn();
  uint8_t undockTimeoutEvent = !dockedOrUsbStateSaved
      && ((RTC_get64() - time_newUnDockEvent) > TIMEOUT_100_MS);

  if (!shimmerStatus.configuring && !sensing.inSdWr && (dockedOrUsbStateSaved || undockTimeoutEvent))
  {
    LogAndStream_setupDockUndock();

    /* If docked/USB status has changed while dock was being setup, set
     * task again. */
    if (dockedOrUsbStateSaved != LogAndStream_isDockedOrUsbIn())
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

#if defined(SHIMMER3R)

/**
 * @brief  Wait for any in-flight SD transfer to complete, then abort.
 *
 * Safe to call even when the peripheral is not initialised — returns
 * immediately in that case.  SHIMMER3R only (uses STM32 SDMMC HAL).
 */
static void LogAndStream_sdWaitAndAbort(void)
{
  if (hsd1.Instance != NULL && shimmerStatus.sdPeripheralInit)
  {
    uint32_t start_tick = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER
        && ((HAL_GetTick() - start_tick) < 1500U))
    {
      platform_delayMs(10);
    }
    HAL_SD_Abort(&hsd1);
  }
}

/** Hand SD card to USB-C (USBX MSC + CDC).  SHIMMER3R only. */
static void LogAndStream_assignSdToUsb(void)
{
  /* Tear down dock if it had ownership */
  DockUart_deinit();
  Board_dockDetectN(DOCK_CARD_NOT_PRESENT);

  /* Tear down USB if it was already running (clean re-init) */
  USB_deinit();

  /* Route SD to MCU so USBX MSC can access it */
  Board_sd2Mcu();

  /* Bring up USB device only if SD card initialised successfully */
  if (shimmerStatus.sdPeripheralInit)
  {
    USB_init();
  }

  shimmerStatus.sdOwner = SD_OWNER_USB;
}
#endif /* SHIMMER3R – USB-only helpers */

/**
 * @brief Hand SD card to the physical dock's USB-SD bridge.
 *
 * Shared by both Shimmer3 and Shimmer3R.  On Shimmer3R the USB device stack
 * is torn down first and the SDMMC transfer is aborted before switching.
 */
void LogAndStream_assignSdToDock(void)
{
#if defined(SHIMMER3R)
  /* Tear down USB device first — must happen before touching SDMMC so that
   * the MSC class stops issuing SD commands. */
  USB_deinit();

  /* Wait for any in-flight SD transfer to complete and abort so the card
   * returns to transfer state before we power-cycle it for the dock. */
  LogAndStream_sdWaitAndAbort();
#endif

  if (LogAndStream_checkSdInSlot())
  {
    Board_sd2Pc();
  }
  if (!shimmerStatus.sensing)
  {
    DockUart_init();
  }

#if defined(SHIMMER3R)
  shimmerStatus.sdOwner = SD_OWNER_DOCK;
#endif
}

/**
 * @brief Release SD card back to MCU (no external owner).
 *
 * Shared by both Shimmer3 and Shimmer3R.  On Shimmer3R the USB device stack
 * is torn down first.
 */
void LogAndStream_releaseSdToMcu(void)
{
#if defined(SHIMMER3R)
  if (USBX_IsInitialised())
  {
    MX_USBX_Device_DeInit();
  }
#endif
  DockUart_deinit();

#if defined(SHIMMER3R)
  shimmerStatus.sdOwner = SD_OWNER_MCU;
#endif
}

void LogAndStream_setupDock(void)
{
  //Only one of these conditions needs to be true to stop logging
  shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_STOP;
  shimmerStatus.sdlogReady = 0;
  ShimSens_stopSensing(0);

#if defined(SHIMMER3R)
  /* --- SD ownership logic ---
   *
   * The SD card can only be owned by one entity at a time: MCU (for sensor
   * recording), USB-C (USBX MSC) or physical dock (USB-SD bridge).
   *
   * Rules:
   *   1. USB has priority over dock when both are connected.
   *   2. If the current owner disconnects, re-evaluate and hand SD to whoever
   *      is still connected.
   *   3. If the current owner is still connected, a new connection by the other
   *      does NOT steal the SD card.
   */
  uint8_t currentOwnerStillConnected = 0;

  switch (shimmerStatus.sdOwner)
  {
    case SD_OWNER_USB:
      currentOwnerStillConnected = shimmerStatus.usbPluggedIn;
      break;
    case SD_OWNER_DOCK:
      currentOwnerStillConnected = shimmerStatus.docked;
      break;
    case SD_OWNER_MCU:
    default:
      /* Nobody external owns it yet — always re-evaluate */
      currentOwnerStillConnected = 0;
      break;
  }

  if (!currentOwnerStillConnected)
  {
    /* Current owner disconnected (or first time) — re-evaluate.
     * USB has priority over dock when choosing a new owner. */
    if (shimmerStatus.usbPluggedIn)
    {
      LogAndStream_assignSdToUsb();
    }
    else if (shimmerStatus.docked)
    {
      LogAndStream_assignSdToDock();
    }
    else
    {
      /* Should not happen in setupDock (called when isDockedOrUsbIn),
       * but handle defensively */
      LogAndStream_releaseSdToMcu();
    }
  }
  /* else: current owner still connected — no SD switchover needed. */

#else
  /* Non-SHIMMER3R: dock-only, no SD owner arbitration needed */
  if (shimmerStatus.docked)
  {
    LogAndStream_assignSdToDock();
  }
#endif

  ShimBatt_setBatteryInterval(BATT_INTERVAL_SECS_DOCKED);
  /* Reset battery critical count on dock to allow logging to begin again if
   * auto-stop on low-power is enabled. */
  ShimBatt_resetBatteryCriticalCount();

  ShimBt_instreamStatusRespSend();
}

void LogAndStream_setupUndock(void)
{
  ShimBatt_setBatteryInterval(BATT_INTERVAL_SECS_UNDOCKED);

  /* Release any external SD ownership before reclaiming for sensor recording */
  LogAndStream_releaseSdToMcu();

  /* Set dock detect high to let dock know SD card is not available and kill
   * power to SD card. It will get turned back on as part of power cycle if
   * SD is still inserted */
  Board_dockDetectN(1);
  Board_setSdPower(0);

  /* Send instream response here if user button is enabled. Otherwise it will be send later when undock start logging is started. */
  if (ShimConfig_getStoredConfig()->userButtonEnable || !LogAndStream_checkSdInSlot())
  {
    ShimBt_instreamStatusRespSend();
  }

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
        platform_delayMs(120); //120ms
        LogAndStream_syncConfigAndCalibOnSd();
      }
      else
      {
        LogAndStream_setSdInfoSyncDelayed(1);
      }
    }
    ShimSens_startLoggingIfUndockStartEnabled();
  }
}

uint8_t LogAndStream_checkSdInSlot(void)
{
  //Check if card is inserted and enable interrupt for SD_DETECT_N
  shimmerStatus.sdInserted = Board_isSdInserted();
  return shimmerStatus.sdInserted;
}

void LogAndStream_processDaughterCardId(void)
{
  /* Read all from EEPROM if present */
  if (ShimEeprom_isPresent())
  {
    ShimEeprom_readAll();
  }
  /* Process the hardware revision in case any FW overrides are needed. */
  platform_processHwRevision();
  /* Parse the daughter card ID to a String. */
  ShimBrd_parseDaughterCardId();
  /* Initialise any GPIOs that depend on the hardware revision. */
  platform_initGpioForRevision();
}

void LogAndStream_buildShimmerMacSuffix(char *outBuf, size_t outBufLen)
{
  char *mac = ShimBt_macIdStrPtrGet();
  char c8 = 'X', c9 = 'X', c10 = 'X', c11 = 'X';

  /* Validate output buffer before use, similar to LogAndStream_buildShimmerPrefix */
  if ((outBuf == NULL) || (outBufLen == 0))
  {
    return;
  }

  /* Start with an empty string in case of truncation or early return */
  outBuf[0] = '\0';

  if (mac != NULL)
  {
    size_t macLen = strlen(mac);
    if (macLen >= 12)
    {
      c8 = mac[8];
      c9 = mac[9];
      c10 = mac[10];
      c11 = mac[11];
    }
  }

  /* suffix is 4 characters plus terminating NUL (e.g. "AAAA") */
  snprintf(outBuf, outBufLen, "%c%c%c%c", c8, c9, c10, c11);
}

void LogAndStream_buildShimmerPrefix(char *outBuf, size_t outBufLen)
{
  /* prefix fits within SHIMMER_PREFIX_LEN (e.g. "Shimmer AAAA") */
  char suffix[8]; /* enough for "AAAA" + NUL */

  if ((outBuf == NULL) || (outBufLen == 0))
  {
    return;
  }

  /* Start with an empty string in case of truncation or formatting errors */
  outBuf[0] = '\0';

  LogAndStream_buildShimmerMacSuffix(suffix, sizeof(suffix));
  snprintf(outBuf, outBufLen, "Shimmer %s", suffix);
}
