/*
 * shimmer_button.c
 *
 *  Created on: 30 Jun 2025
 *      Author: MarkNolan
 */

#include "shimmer_button.h"

#include "stdint.h"

#include "log_and_stream_includes.h"

//Time-stamps
uint64_t buttonPressTs, buttonReleaseCurrentTs, buttonReleasePrevTs;
//Time difference calculations
uint64_t buttonReleaseTd, buttonPressReleaseTd;

void ShimBtn_init(void)
{
  buttonPressTs = 0;
  buttonReleaseCurrentTs = 0;
  buttonReleasePrevTs = 0;
  buttonReleaseTd = 0;
  buttonPressReleaseTd = 0;
}

/* Returns 1 to indicate MCU should be woken to carry out a task. */
uint8_t ShimBtn_pressReleaseAction(void)
{
  uint8_t wake = 0;
  shimmerStatus.buttonPressed = Board_isBtnPressed();
  if (shimmerStatus.buttonPressed)
  { //button pressed
    buttonPressTs = RTC_get64();
  }
  else
  { //button released
    buttonReleaseCurrentTs = RTC_get64();

    //Guard against bogus long-press when no prior press was recorded
    if (buttonPressTs != 0 && buttonReleaseCurrentTs >= buttonPressTs)
    {
      buttonPressReleaseTd = buttonReleaseCurrentTs - buttonPressTs;
    }
    else
    {
      buttonPressReleaseTd = 0;
    }

    buttonReleaseTd = buttonReleaseCurrentTs - buttonReleasePrevTs;

    /* long button press: >=5s */
    if (buttonPressReleaseTd >= TICKS_5_SECONDS)
    {
      //Consume the event; update the last-release timestamp to prevent repeats
      //(No long-press action defined yet)
      buttonReleasePrevTs = buttonReleaseCurrentTs;
    }
    /* 0.5s < Press > 5s: take action as long as the device isn't currently in
     * the middle of configuring and SD SYNC isn't in the middle of an
     * operation. */
    else if ((buttonReleaseTd > TICKS_0_5_SECONDS) && !shimmerStatus.configuring
        && !(shimmerStatus.btConnected && shimmerStatus.sdSyncCommTimerRunning))
    {
      buttonReleasePrevTs = buttonReleaseCurrentTs;
#if TEST_PRESS2UNDOCK
      if (shimmerStatus.docked)
      {
        shimmerStatus.docked = 0;
      }
      else
      {
        shimmerStatus.docked = 1;
      }
      LogAndStream_dockedStateChange();
      if (!shimmerStatus.sensing)
      {
        wake = 1;
      }
#else
      if (ShimConfig_getStoredConfig()->userButtonEnable)
      {
        //toggles sensing and refresh BT timers (for the centre)
        if (shimmerStatus.sdLogging == 0)
        {
          ShimTask_setStartLoggingIfReady();
        }
        else
        {
          ShimTask_setStopLogging();
        }
        wake = 1;
      }
#endif
    }
    /* Short press, take no action */
    else
    {
      //Always update last release to avoid stale deltas causing spurious triggers later
      buttonReleasePrevTs = buttonReleaseCurrentTs;
      __NOP();
    }

    //Single place to clear the press timestamp after handling a release
    buttonPressTs = 0;
  }
  return wake;
}
