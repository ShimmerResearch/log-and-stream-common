/*
 * led.c
 *
 *  Created on: 25 Mar 2025
 *      Author: MarkNolan
 */

#include "shimmer_leds.h"
#include "log_and_stream_includes.h"

#if defined(SHIMMER3)
#include "../5xx_HAL/hal_RTC.h"
#include "../RN4X/RN4X.h"
#include "hal_Board.h"
#else
#endif

uint16_t blinkCnt20, blinkCnt50;
uint8_t lastLedToggleUpr, lastLedToggleCnt, lastLedToggleLwr, rwcErrorFlash;

static int bootLedIndex = 0;
static int bootLedDirection = 1; //1: forward, -1: backward
const uint8_t bootLedOrder[3] = { LED_LWR_GREEN, LED_LWR_YELLOW, LED_LWR_RED };

void ShimLeds_blinkSetUprState(void);
void ShimLeds_blinkSetUprRtcNotSet(void);
void ShimLeds_blinkSetUprConfiguring(void);
void ShimLeds_blinkSetUpLogAndStreamMode(void);
void ShimLeds_blinkSetUprStreamingOnly(void);
void ShimLeds_blinkSetUprConnectedAndLogging(void);
void ShimLeds_blinkSetUprLoggingOnly(void);
void ShimLeds_blinkSetUprLoggingAndStreaming(void);
void ShimLeds_blinkSetUprAdvertisingLoggingIdle(void);
void ShimLeds_blinkSetUprLoggingOrSdSyncMode(void);
void ShimLeds_blinkSetUprLogging(void);
void ShimLeds_blinkSetUprLoggingIdle(void);

void ShimLeds_blinkSetLwrState(void);
void ShimLeds_blinkSetLwrBtnPress(void);
void ShimLeds_blinkSetLwrRedOn(void);
void ShimLeds_blinkSetLwrErrorSensorBus(void);
void ShimLeds_blinkSetLwrErrorBluetooth(void);
void ShimLeds_blinkSetLwrErrorSdCard(void);
void ShimLeds_blinkSetLwrBattStatus(void);
#if defined(SHIMMER3R)
void ShimLeds_blinkSetLwrEnteringBslMode(void);
#endif

void ShimLeds_varsInit(void)
{
  blinkCnt20 = blinkCnt50 = 0;
  lastLedToggleUpr = 0;
  lastLedToggleCnt = 0;
  lastLedToggleLwr = 0;
  rwcErrorFlash = 0;
  ShimLeds_setRtcErrorFlash(0);

  bootLedIndex = 0;
  bootLedDirection = 1;
}

void ShimLeds_incrementCounters(void)
{
  //Note: each count is 0.1s
  if (blinkCnt50++ == 49)
  {
    blinkCnt50 = 0;
  }

  if (blinkCnt20++ == 19)
  {
    blinkCnt20 = 0;
  }
}

void ShimLeds_controlDuringBoot(boot_stage_t bootStageCurrent)
{
#if defined(SHIMMER3R)
  //If the bootloader is initialising, toggle the LEDs to indicate this
  if (shimmerStatus.bslCheckTimeMs > 0)
  {
    ShimLeds_blinkSetLwrEnteringBslMode();
    return;
  }
#endif

  //If timeout has occurred, show boot stage failure point by toggling individual LEDs
  if (bootStageCurrent == BOOT_STAGE_I2C && shimmerStatus.bootTimePerStageMs > BOOT_STAGE_TIMEOUT_MS_I2C)
  {
    ShimLeds_blinkSetLwrErrorSensorBus();
  }
  else if (bootStageCurrent == BOOT_STAGE_BLUETOOTH_FAILURE)
  {
    ShimLeds_blinkSetLwrErrorBluetooth();
  }
  else if (bootStageCurrent == BOOT_STAGE_CONFIGURATION
      && shimmerStatus.bootTimePerStageMs > BOOT_STAGE_TIMEOUT_MS_CONFIGURATION)
  {
    ShimLeds_blinkSetLwrErrorSdCard();
  }
  else
  {
    //If the boot stage is not yet complete, toggle the LEDs to indicate booting
    shimmerStatus.bootTimePerStageMs += SHIMMER_BLINK_TIMER_PERIOD_MS;

    //Note: This is a placeholder for legacy behavior, can be removed if not needed
    //// Turn on LEDs as per legacy behavior in Shimmer3
    //Board_ledOn(LED_UPR_GREEN + LED_UPR_BLUE + LED_LWR_GREEN + LED_LWR_YELLOW + LED_LWR_RED);

    //Cycle through the LEDs in a specific order
    if (ShimLeds_isBlinkTimerCnt500ms())
    {
      //Turn off all LEDs first
      Board_ledOff(LED_ALL);

      //Turn on the current LED
      Board_ledOn(bootLedOrder[bootLedIndex]);

      //Update index for next call
      bootLedIndex += bootLedDirection;
      if (bootLedIndex == (sizeof(bootLedOrder) / sizeof(bootLedOrder[0]) - 1))
      {
        bootLedDirection = -1; //Reverse at red
      }
      else if (bootLedIndex == 0)
      {
        bootLedDirection = 1; //Forward at green1
      }
    }
  }
}

void ShimLeds_blink(void)
{
  /* Upper LED control */
  ShimLeds_blinkSetUprState();

  /* Lower LED control */
  ShimLeds_blinkSetLwrState();
}

void ShimLeds_blinkSetUprState(void)
{
  if (rwcErrorFlash && !shimmerStatus.sensing)
  {
    ShimLeds_blinkSetUprRtcNotSet();
  }
  else if (shimmerStatus.configuring)
  {
    ShimLeds_blinkSetUprConfiguring();
  }
  else if (shimmerStatus.btSupportEnabled && !shimmerStatus.sdSyncEnabled)
  {
    ShimLeds_blinkSetUpLogAndStreamMode();
  }
  else
  {
    ShimLeds_blinkSetUprLoggingOrSdSyncMode();
  }
}

void ShimLeds_blinkSetUprRtcNotSet(void)
{
  if (ShimLeds_isBlinkTimerCnt200ms())
  {
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
    Board_ledOn(LED_UPR_GREEN);
    Board_ledOn(LED_UPR_BLUE);
#else
    Board_ledUprSetColourRgb(LED_PWM_OFF, LED_PWM_ON, LED_PWM_ON); //Cyan
#endif
  }
  else
  {
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
    Board_ledOff(LED_UPR_GREEN);
    Board_ledOff(LED_UPR_BLUE);
#else
    Board_ledUprSetColourRgb(LED_PWM_OFF, LED_PWM_OFF, LED_PWM_OFF); //Off
#endif
  }
}

void ShimLeds_blinkSetUprConfiguring(void)
{
  if (ShimLeds_isBlinkTimerCnt200ms())
  {
    Board_ledToggle(LED_UPR_GREEN);
  }
}

void ShimLeds_blinkSetUpLogAndStreamMode(void)
{
  if (shimmerStatus.sensing)
  {
    //btstream only
    if ((shimmerStatus.btstreamReady && shimmerStatus.btStreaming)
        && !(shimmerStatus.sdlogReady && shimmerStatus.sdLogging))
    {
      ShimLeds_blinkSetUprStreamingOnly();
    }
    //sdlog only
    else if (!(shimmerStatus.btstreamReady && shimmerStatus.btStreaming)
        && (shimmerStatus.sdlogReady && shimmerStatus.sdLogging))
    {
      if (shimmerStatus.btConnected)
      {
        ShimLeds_blinkSetUprConnectedAndLogging();
      }
      else
      {
        ShimLeds_blinkSetUprLoggingOnly();
      }
    }
    //btstream & sdlog
    else if ((shimmerStatus.btstreamReady && shimmerStatus.btStreaming)
        && (shimmerStatus.sdlogReady && shimmerStatus.sdLogging))
    {
      ShimLeds_blinkSetUprLoggingAndStreaming();
    }
    else
    {
      Board_ledOff(LED_UPR_GREEN + LED_UPR_BLUE); //nothing to show
    }
  }
  else if (shimmerStatus.btConnected)
  {
    Board_ledOn(LED_UPR_BLUE);
    Board_ledOff(LED_UPR_GREEN); //nothing to show
  }
#if defined(SHIMMER3)
  else if (isRn4678ConnectionEstablished())
  {
    /* BT connection established but RFComm not open */
    Board_ledToggle(LED_UPR_BLUE);
    Board_ledOff(LED_UPR_GREEN); //nothing to show
  }
#endif
  else
  {
    /* BT advertising and logging idle*/
    ShimLeds_blinkSetUprAdvertisingLoggingIdle();
  }
}

void ShimLeds_blinkSetUprStreamingOnly(void)
{
  if (ShimLeds_isBlinkTimerCnt1s())
  {
    Board_ledToggle(LED_UPR_BLUE);
  }
  Board_ledOff(LED_UPR_GREEN); //nothing to show
}

void ShimLeds_blinkSetUprConnectedAndLogging(void)
{
  if (ShimLeds_isBlinkTimerCnt1s())
  {
    lastLedToggleCnt++;
    if (lastLedToggleCnt >= 3)
    {
      lastLedToggleCnt = 0;
    }

    if (lastLedToggleCnt == 2)
    {
      Board_ledOn(LED_UPR_GREEN);
      Board_ledOff(LED_UPR_BLUE);
    }
    else
    {
      Board_ledOff(LED_UPR_GREEN);
      Board_ledOn(LED_UPR_BLUE);
    }
  }
}

void ShimLeds_blinkSetUprLoggingOnly(void)
{
  if (ShimLeds_isBlinkTimerCnt1s())
  {
    Board_ledToggle(LED_UPR_GREEN);
  }
  Board_ledOff(LED_UPR_BLUE); //nothing to show
}

void ShimLeds_blinkSetUprLoggingAndStreaming(void)
{
  if (ShimLeds_isBlinkTimerCnt1s())
  {
    if (Board_isLedOnUprBlue() || Board_isLedOnUprGreen())
    {
      Board_ledOff(LED_UPR_BLUE + LED_UPR_GREEN);
    }
    else
    {
      if (lastLedToggleUpr)
      {
        Board_ledOn(LED_UPR_BLUE);
      }
      else
      {
        Board_ledOn(LED_UPR_GREEN);
      }
      lastLedToggleUpr ^= 1;
    }
  }
}

void ShimLeds_blinkSetUprAdvertisingLoggingIdle(void)
{
  Board_ledOff(LED_UPR_GREEN);
  if (shimmerStatus.btPowerOn && ShimLeds_isBlinkTimerCnt2s())
  {
    Board_ledOn(LED_UPR_BLUE);
  }
  else
  {
    Board_ledOff(LED_UPR_BLUE);
  }
}

void ShimLeds_blinkSetUprLoggingOrSdSyncMode(void)
{
  //SDLogging or SD sync - upper green LED:
  if (shimmerStatus.sensing)
  { //sensing
    ShimLeds_blinkSetUprLogging();
  }
  else
  { //standby
    ShimLeds_blinkSetUprLoggingIdle();
  }

  //SDLogging or SD sync - upper blue LED:
  /* Toggle blue LED while a connection is established */
  if (shimmerStatus.btPowerOn && shimmerStatus.btConnected)
  {
    /* turn off green LED to avoid visual clash */
    Board_ledOff(LED_UPR_GREEN);
    Board_ledToggle(LED_UPR_BLUE);
  }
  /* Leave blue LED on solid if it's a node and a sync hasn't occurred yet (first 'outlier' not included) */
  else if (!ShimSdSync_rcFirstOffsetRxedGet() && shimmerStatus.sensing
      && shimmerStatus.sdSyncEnabled
      && !(ShimSdHead_sdHeadTextGetByte(SDH_TRIAL_CONFIG0) & SDH_IAMMASTER))
  {
    /*Avoid clashing with the green LED*/
    if (Board_isLedOnUprGreen())
    {
      Board_ledOff(LED_UPR_BLUE);
    }
    else
    {
      Board_ledOn(LED_UPR_BLUE);
    }
  }
  else
  {
    /* Quick double flash if BT is on */
    if (shimmerStatus.btPowerOn && (blinkCnt20 == 12 || blinkCnt20 == 14))
    {
      Board_ledOn(LED_UPR_BLUE);
    }
    else
    {
      Board_ledOff(LED_UPR_BLUE);
    }

    //TODO original SD sync blink code is below, trying to simplify things above. Decide whether to keep above and then remove below.

    ///* Flash twice if sync is not successfull */
    //if (((blinkCnt20 == 12) || (blinkCnt20 == 14))
    //        && !shimmerStatus.docked
    //        && shimmerStatus.sensing
    //        && shimmerStatus.sdSyncEnabled
    //        && ((!getSyncSuccC() && (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER))
    //                || (!getSyncSuccN() && !(sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER))))
    //{
    //    if (getSyncCnt() > 3)
    //    {
    //        Board_ledOn(LED_BLUE);
    //    }
    //    //TODO should there be an else here?
    //}
    //else
    //{
    //    Board_ledOff(LED_BLUE);
    //}
  }
}

void ShimLeds_blinkSetUprLogging(void)
{
  if (blinkCnt20 < 10)
  {
    Board_ledOn(LED_UPR_GREEN);
  }
  else
  {
    Board_ledOff(LED_UPR_GREEN);
  }
}

void ShimLeds_blinkSetUprLoggingIdle(void)
{
  if (ShimLeds_isBlinkTimerCnt2s())
  {
    Board_ledOn(LED_UPR_GREEN);
  }
  else
  {
    Board_ledOff(LED_UPR_GREEN);
  }
}

void ShimLeds_blinkSetLwrState(void)
{
  if (shimmerStatus.buttonPressed)
  {
    //keep lower green LED on when user button pressed
    ShimLeds_blinkSetLwrBtnPress();
  }
  else if (shimmerStatus.toggleLedRedCmd)
  {
    ShimLeds_blinkSetLwrRedOn();
  }
  else if (!shimmerStatus.docked && (shimmerStatus.sdBadFile || !shimmerStatus.sdInserted)
      && ShimConfig_getStoredConfig()->sdErrorEnable)
  {
    ShimLeds_blinkSetLwrErrorSdCard();
  }
  else
  {
    ShimLeds_blinkSetLwrBattStatus();
  }
}

void ShimLeds_blinkSetLwrBtnPress(void)
{
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
  Board_ledOn(LED_LWR_GREEN);
#else
  Board_ledLwrSetColourRgb(-1, LED_PWM_ON, -1);
#endif
}

void ShimLeds_blinkSetLwrRedOn(void)
{
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
  Board_ledOff(LED_LWR_GREEN + LED_LWR_YELLOW);
  Board_ledOn(LED_LWR_RED);
#else
  Board_ledLwrSetColourRgb(LED_PWM_ON, LED_PWM_OFF, LED_PWM_OFF); //Red
#endif
}

void ShimLeds_blinkSetLwrErrorSensorBus(void)
{
  //Alternate Red/Yellow
  if (ShimLeds_isBlinkTimerCnt200ms())
  {
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
    Board_ledOn(LED_LWR_RED);
    Board_ledOff(LED_LWR_GREEN + LED_LWR_YELLOW);
#else
    Board_ledLwrSetColourRgb(LED_PWM_ON, LED_PWM_ON, LED_PWM_OFF); //Yellow
#endif
  }
  else
  {
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
    Board_ledOff(LED_LWR_RED);
    Board_ledOff(LED_LWR_GREEN + LED_LWR_YELLOW);
#else
    Board_ledLwrSetColourRgb(LED_PWM_ON, LED_PWM_OFF, LED_PWM_OFF); //Red
#endif
  }
}

void ShimLeds_blinkSetLwrErrorBluetooth(void)
{
  if (ShimLeds_isBlinkTimerCnt200ms())
  {
    Board_ledToggle(LED_LWR_YELLOW);
  }
}

void ShimLeds_blinkSetLwrErrorSdCard(void)
{
  if (ShimLeds_isBlinkTimerCnt200ms())
  {
    Board_ledToggle(LED_LWR_GREEN);
  }
}

void ShimLeds_blinkSetLwrBattStatus(void)
{
  //battery charge/state status LEDs
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
  Board_ledOff(LED_LWR_GREEN + LED_LWR_YELLOW + LED_LWR_RED);
#endif
  uint32_t batt_led = 0;
  if (batteryStatus.battStatLedFlash)
  {
    if (ShimLeds_isBlinkTimerCnt200ms())
    {
      batt_led = batteryStatus.battStatLedCharging;
    }
    else
    {
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
      batt_led = LED_ALL_OFF;
#else
      batt_led = LED_RGB_ALL_OFF;
#endif
    }
  }
  else
  {
    if (LogAndStream_isDockedOrUsbIn())
    {
      batt_led = batteryStatus.battStatLedCharging;
    }
    else if (ShimLeds_isBlinkTimerCnt5s())
    {
      batt_led = batteryStatus.battStatLed;
    }
  }
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
  Board_ledOn(batt_led);
#else
  Board_ledLwrSetColour(batt_led);
#endif
}

#if defined(SHIMMER3R)
void ShimLeds_blinkSetLwrEnteringBslMode(void)
{
  if (ShimLeds_isBlinkTimerCnt200ms())
  {
    Board_ledUprSetColourRgb(0, 0, 0);
    //Toggle upper and lower LEDs purple every 100ms
    if (lastLedToggleLwr)
    {
      Board_ledLwrSetColourRgb(128, 0, 128);
    }
    else
    {
      Board_ledLwrSetColourRgb(0, 0, 0);
    }
    lastLedToggleLwr ^= 1;
  }
}
#endif

uint8_t ShimLeds_isBlinkTimerCnt200ms(void)
{
  return (blinkCnt20 % 2);
}

uint8_t ShimLeds_isBlinkTimerCnt500ms(void)
{
  return (blinkCnt20 % 5 == 0);
}

uint8_t ShimLeds_isBlinkTimerCnt1s(void)
{
  return (blinkCnt20 % 10 == 0);
}

uint8_t ShimLeds_isBlinkTimerCnt2s(void)
{
  return (blinkCnt20 == 0);
}

uint8_t ShimLeds_isBlinkTimerCnt5s(void)
{
  return (blinkCnt50 == 0);
}

void ShimLeds_setRtcErrorFlash(uint8_t state)
{
  rwcErrorFlash = state;
}

uint8_t ShimLeds_getRtcErrorFlash(void)
{
  return rwcErrorFlash;
}
