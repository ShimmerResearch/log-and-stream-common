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

static uint8_t isMarkerMode(void)
{
  return ShimConfig_getStoredConfig()->buttonEventMarkerEnable;
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

static void applyResult(const ShimBtnGestureResult *res, uint8_t markerMode)
{
  uint8_t i;
  for (i = 0; i < res->numEdges; i++)
  {
    /* Written only here, in the main loop, so the LEDs follow the debounced
     * level rather than every bounce */
    shimmerStatus.buttonPressed = res->edge[i].level;
    if (markerMode)
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
  uint8_t markerMode = isMarkerMode();

  ShimBtnGesture_edge(&gesture, ts, isPress, markerMode, &res);
  applyResult(&res, markerMode);
}

uint8_t ShimBtn_isRecheckPending(void)
{
  return gesture.bouncePending;
}

void ShimBtn_recheckIfDue(uint64_t now)
{
  ShimBtnGestureResult res;
  uint8_t markerMode;

  if (!ShimBtnGesture_isRecheckDue(&gesture, now))
  {
    return;
  }
  markerMode = isMarkerMode();
  ShimBtnGesture_recheck(&gesture, Board_isBtnPressed(), markerMode, &res);
  applyResult(&res, markerMode);
}

uint8_t ShimBtn_isPressed(void)
{
  return gesture.level;
}
