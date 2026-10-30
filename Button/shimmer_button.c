/*
 * shimmer_button.c
 *
 *  Created on: 30 Jun 2025
 *      Author: MarkNolan
 */

#include "shimmer_button.h"

#include "stdint.h"

#include "log_and_stream_includes.h"

/* Main loop only */
static ShimBtnGesture gesture;

void ShimBtn_init(void)
{
  ShimBtnGesture_init(&gesture);
}

/* Not while the device is being configured, nor while SD sync is mid-way
 * through an exchange */
static uint8_t isActionAllowed(void)
{
  return !shimmerStatus.configuring
      && !(shimmerStatus.btConnected && shimmerStatus.sdSyncCommTimerRunning);
}

static void toggleLogging(void)
{
#if TEST_PRESS2UNDOCK
  shimmerStatus.docked = shimmerStatus.docked ? 0 : 1;
  LogAndStream_dockedStateChange();
#else
  if (ShimConfig_doesPressToggleLogging())
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
  }
#endif
}

/* Configuration byte 232 decides both whether presses are recorded and which
 * gesture starts and stops logging (Configuration/shimmer_button_log_mode.h) */
static void applyResult(const ShimBtnGestureResult *res)
{
  uint8_t i;
  uint8_t record = ShimConfig_doesButtonRecordPresses();

  for (i = 0; i < res->numEdges; i++)
  {
    /* Written only here, in the main loop, so the LEDs follow the debounced
     * level rather than every bounce */
    shimmerStatus.buttonPressed = res->edge[i].level;
    if (record)
    {
      ShimEvt_recordButtonEdge(res->edge[i].ts, res->edge[i].level,
          res->edge[i].synthesized ? EVT_FLAG_SYNTHESIZED : 0U);
    }
  }

  if (res->toggleLogging && isActionAllowed())
  {
    toggleLogging();
  }
}

void ShimBtn_processEdge(uint64_t ts, uint8_t isPress)
{
  ShimBtnGestureResult res;

  ShimBtnGesture_edge(&gesture, ts, isPress, ShimConfig_isDoublePress(), &res);
  applyResult(&res);
}

uint8_t ShimBtn_isRecheckPending(void)
{
  return gesture.bouncePending;
}

void ShimBtn_recheckIfDue(uint64_t now)
{
  ShimBtnGestureResult res;

  if (!ShimBtnGesture_isRecheckDue(&gesture, now))
  {
    return;
  }
  ShimBtnGesture_recheck(&gesture, Board_isBtnPressed(), ShimConfig_isDoublePress(), &res);
  applyResult(&res);
}

uint8_t ShimBtn_isPressed(void)
{
  return gesture.level;
}
