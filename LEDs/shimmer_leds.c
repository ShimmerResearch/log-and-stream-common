/*
 * led.c
 *
 *  Created on: 25 Mar 2025
 *      Author: MarkNolan
 */

#include <LEDs/shimmer_leds.h>
#include <log_and_stream_includes.h>

#if defined(SHIMMER3)
#include "../5xx_HAL/hal_RTC.h"
#include "../RN4X/RN4X.h"
#include "hal_Board.h"
#else
#endif

uint16_t blinkCnt20, blinkCnt50;
uint8_t lastLedGroup2, rwcErrorFlash;

static int ledIndex = 0;
static int direction = 1; //1: forward, -1: backward
const uint8_t ledOrder[3]
    = { LED_LWR_GREEN, LED_LWR_YELLOW, LED_LWR_RED };

void ShimLeds_blinkSetLwrRedOn(void);
void ShimLeds_blinkSetLwrSdError(void);
void ShimLeds_blinkSetLwrBattStatus(void);
void ShimLeds_blinkSetLwrBtnPress(void);
void ShimLeds_blinkSetUprRtcNotSet(void);
void ShimLeds_blinkSetUprDeviceStatus(void);

void ShimLeds_varsInit(void)
{
  blinkCnt20 = blinkCnt50 = 0;
  lastLedGroup2 = 0;
  rwcErrorFlash = 0;

  ledIndex = 0;
  direction = 1;
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
    //Toggle upper and lower LEDs purple every 200ms
    if (ShimLeds_isBlinkTimerCnt200ms())
    {
      Board_ledUprSetColourRgb(0, 0, 0);
      Board_ledLwrSetColourRgb(128, 0, 128);
    }
    else
    {
      Board_ledUprSetColourRgb(128, 0, 128);
      Board_ledLwrSetColourRgb(0, 0, 0);
    }
    return;
  }
#endif

  //If timeout has occurred, show boot stage failure point by toggling individual LEDs
  if (bootStageCurrent == BOOT_STAGE_I2C && shimmerStatus.bootTimePerStageMs > BOOT_STAGE_TIMEOUT_MS_I2C)
  {
    Board_ledToggle(LED_LWR_RED);
  }
  else if (bootStageCurrent == BOOT_STAGE_BLUETOOTH_FAILURE)
  {
    Board_ledToggle(LED_LWR_YELLOW);
  }
  else if (bootStageCurrent == BOOT_STAGE_CONFIGURATION
      && shimmerStatus.bootTimePerStageMs > BOOT_STAGE_TIMEOUT_MS_CONFIGURATION)
  {
    Board_ledToggle(LED_LWR_GREEN);
  }
  else
  {
    //If the boot stage is not yet complete, toggle the LEDs to indicate booting
    shimmerStatus.bootTimePerStageMs += SHIMMER_BLINK_TIMER_PERIOD_MS;

    //Note: This is a placeholder for legacy behavior, can be removed if not needed

    //// Toggle upper and lower LEDs green every 200ms to indicate booting
    //if(ShimLeds_isBlinkTimerCnt200ms())
    //{
    //  if(Board_isLedOnUprGreen())
    //  {
    //    Board_ledOn(LED_LWR_GREEN);
    //    Board_ledOff(LED_UPR_GREEN);
    //  }
    //  else
    //  {
    //    Board_ledOff(LED_LWR_GREEN);
    //    Board_ledOn(LED_UPR_GREEN);
    //  }
    //}

    //Cycle through the LEDs in a specific order
    if (ShimLeds_isBlinkTimerCnt200ms())
    {
      //Turn off all LEDs first
      Board_ledOff(LED_ALL);

      //Turn on the current LED
      Board_ledOn(ledOrder[ledIndex]);

      //Update index for next call
      ledIndex += direction;
      if (ledIndex == (sizeof(ledOrder)/sizeof(ledOrder[0]) - 1))
      {
        direction = -1; //Reverse at red
      }
      else if (ledIndex == 0)
      {
        direction = 1; //Forward at green1
      }
    }

    //// Turn on LEDs as per legacy behavior in Shimmer3
    //Board_ledOn(LED_UPR_GREEN + LED_UPR_BLUE + LED_LWR_GREEN + LED_LWR_YELLOW + LED_LWR_RED);
  }
}

void ShimLeds_blink(void)
{
  /* Lower LED control */
  if (shimmerStatus.toggleLedRedCmd)
  {
    ShimLeds_blinkSetLwrRedOn();
  }
  else if (!shimmerStatus.docked && (shimmerStatus.sdBadFile || !shimmerStatus.sdInserted)
      && ShimConfig_getStoredConfig()->sdErrorEnable)
  {
    ShimLeds_blinkSetLwrSdError();
  }
  else
  {
    ShimLeds_blinkSetLwrBattStatus();
  }

  //code for keeping LED_LWR_GREEN on when user button pressed
  if (shimmerStatus.buttonPressed)
  {
    ShimLeds_blinkSetLwrBtnPress();
  }

  //below are settings for green1, blue, yellow and red leds

  if (rwcErrorFlash && !shimmerStatus.sensing)
  {
    ShimLeds_blinkSetUprRtcNotSet();
  }
  else
  {
    ShimLeds_blinkSetUprDeviceStatus();
  }
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

void ShimLeds_blinkSetLwrSdError(void)
{
  //Alternate Red/Yellow for SD error
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

void ShimLeds_blinkSetLwrBtnPress(void)
{
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
  Board_ledOn(LED_LWR_GREEN);
#else
  Board_ledLwrSetColourRgb(-1, LED_PWM_ON, -1);
#endif
}

void ShimLeds_blinkSetUprRtcNotSet(void)
{
  if (ShimLeds_isBlinkTimerCnt200ms())
  {
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
    Board_ledOff(LED_UPR_GREEN);
    Board_ledOn(LED_UPR_BLUE);
#else
    Board_ledUprSetColourRgb(LED_PWM_OFF, LED_PWM_OFF, LED_PWM_ON); //Blue
#endif
  }
  else
  {
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
    Board_ledOn(LED_UPR_GREEN);
    Board_ledOff(LED_UPR_BLUE);
#else
    Board_ledUprSetColourRgb(LED_PWM_OFF, LED_PWM_ON, LED_PWM_OFF); //Green
#endif
  }
}

//TODO merge Shimmer3 and Shimmer3R approaches
void ShimLeds_blinkSetUprDeviceStatus(void)
{
  //#if defined(SHIMMER3)
  if (shimmerStatus.btSupportEnabled && !shimmerStatus.sdSyncEnabled)
  {
    if (shimmerStatus.sensing)
    { //sensing
      //shimmerStatus.sdlogReady, shimmerStatus.btstreamReady, enableSdlog,
      //enableBtstream btstream only
      if ((shimmerStatus.btstreamReady && shimmerStatus.btStreaming)
          && !(shimmerStatus.sdlogReady && shimmerStatus.sdLogging))
      {
        if (ShimLeds_isBlinkTimerCnt1s())
        {
          Board_ledToggle(LED_UPR_BLUE);
        }
        Board_ledOff(LED_UPR_GREEN); //nothing to show
      }
      //sdlog only
      else if (!(shimmerStatus.btstreamReady && shimmerStatus.btStreaming)
          && (shimmerStatus.sdlogReady && shimmerStatus.sdLogging))
      {
        if (ShimLeds_isBlinkTimerCnt1s())
        {
          Board_ledToggle(LED_UPR_GREEN);
        }
        Board_ledOff(LED_UPR_BLUE); //nothing to show
      }
      //btstream & sdlog
      else if ((shimmerStatus.btstreamReady && shimmerStatus.btStreaming)
          && (shimmerStatus.sdlogReady && shimmerStatus.sdLogging))
      {
        if (ShimLeds_isBlinkTimerCnt1s())
        {
          if (Board_isLedOnUprBlue() || Board_isLedOnUprGreen())
          {
            Board_ledOff(LED_UPR_BLUE + LED_UPR_GREEN);
          }
          else
          {
            if (lastLedGroup2)
            {
              Board_ledOn(LED_UPR_BLUE);
              lastLedGroup2 ^= 1;
            }
            else
            {
              Board_ledOn(LED_UPR_GREEN);
              lastLedGroup2 ^= 1;
            }
          }
        }
      }
      else
      {
        Board_ledOff(LED_UPR_GREEN + LED_UPR_BLUE); //nothing to show
      }
    }
    else //standby or configuring
    {
      if (shimmerStatus.configuring)
      {
        Board_ledToggle(LED_UPR_GREEN);
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
        /* BT advertising */
        if (shimmerStatus.btPowerOn && ShimLeds_isBlinkTimerCnt2s())
        {
          Board_ledOn(LED_UPR_BLUE);
        }
        else
        {
          Board_ledOff(LED_UPR_BLUE);
        }
        Board_ledOff(LED_UPR_GREEN);
      }
    }
  }
  else
  {
    //SDLogging or SD sync - upper green LED:
    if (!shimmerStatus.sensing)
    { //standby or configuring
      if (shimmerStatus.configuring)
      { //configuring
        Board_ledToggle(LED_UPR_GREEN);
      }
      else
      { //standby
        if (ShimLeds_isBlinkTimerCnt2s())
        {
          Board_ledOn(LED_UPR_GREEN);
        }
        else
        {
          Board_ledOff(LED_UPR_GREEN);
        }
      }
    }
    else
    { //sensing
      if (blinkCnt20 < 10)
      {
        Board_ledOn(LED_UPR_GREEN);
      }
      else
      {
        Board_ledOff(LED_UPR_GREEN);
      }
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
}

uint8_t ShimLeds_isBlinkTimerCnt200ms(void)
{
  return (blinkCnt20 % 2);
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
