/*
 * shimmer_button.c
 *
 *  Created on: 30 Jun 2025
 *      Author: MarkNolan
 */

#include "shimmer_button.h"

#include "stdint.h"

#include <log_and_stream_includes.h>

uint64_t buttonPressTs, buttonReleaseTs, buttonLastReleaseTs;
//Time difference calculations
uint64_t buttonReleaseTd, buttonPressReleaseTd;

void ShimBtn_init(void)
{
  buttonPressTs = 0;
  buttonReleaseTs = 0;
  buttonLastReleaseTs = 0;
  buttonReleaseTd = 0;
  buttonPressReleaseTd = 0;
}

/* Returns 1 to indicate MCU should be woken to carry out a task. */
uint8_t ShimBtn_pressReleaseAction(void)
{
  if (Board_isBtnPressed())
  { //button pressed
    shimmerStatus.buttonPressed = 1;
    buttonPressTs = RTC_get64();
  }
  else
  { //button released
    shimmerStatus.buttonPressed = 0;
    buttonReleaseTs = RTC_get64();
    buttonPressReleaseTd = buttonReleaseTs - buttonPressTs;
    buttonReleaseTd = buttonReleaseTs - buttonLastReleaseTs;
    if (buttonPressReleaseTd >= TICKS_5_SECONDS)
    { //long button press: 5s
    }
    else if ((buttonReleaseTd > TICKS_0_5_SECONDS) && !shimmerStatus.configuring
        && !shimmerStatus.btConnected)
    {
      buttonLastReleaseTs = buttonReleaseTs;
#if TEST_PRESS2UNDOCK
      if (shimmerStatus.docked)
      {
        shimmerStatus.docked = 0;
      }
      else
      {
        shimmerStatus.docked = 1;
      }
      ShimTask_set(TASK_SETUP_DOCK);
      if (!shimmerStatus.sensing)
      {
        return 1;
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
        return 1;
      }
#endif
    }
    else
    {
      _NOP();
    }
  }
  return 0;
}
