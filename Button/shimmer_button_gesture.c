/*
 * shimmer_button_gesture.c
 *
 *  Created on: 30 Oct 2026
 *      Author: MarkNolan
 */

#include "shimmer_button_gesture.h"

#include <string.h>

void ShimBtnGesture_init(ShimBtnGesture *g)
{
  memset(g, 0, sizeof(*g));
}

static void clearResult(ShimBtnGestureResult *res)
{
  res->numEdges = 0;
  res->toggleLogging = 0;
}

static void releaseAction(ShimBtnGesture *g, uint64_t ts, uint8_t doublePress, ShimBtnGestureResult *res)
{
  /* 0 when the press was not seen, which counts as neither short nor long */
  uint64_t held = (g->pressTs != 0 && ts >= g->pressTs) ? ts - g->pressTs : 0;
  g->pressTs = 0;

  if (doublePress)
  {
    if (held != 0 && held <= BTN_SHORT_PRESS_MAX_TICKS)
    {
      if (g->firstTapTs != 0 && ts - g->firstTapTs <= BTN_DOUBLE_GAP_TICKS)
      {
        res->toggleLogging = 1;
        /* A third tap starts a new pair rather than toggling again */
        g->firstTapTs = 0;
      }
      else
      {
        g->firstTapTs = ts;
      }
    }
    else
    {
      g->firstTapTs = 0;
    }
  }
  else if (held < BTN_LONG_PRESS_TICKS && ts - g->releasePrevTs > BTN_REPEAT_GAP_TICKS)
  {
    /* A press of 5 s or more is consumed: no long-press action is defined */
    res->toggleLogging = 1;
  }
  g->releasePrevTs = ts;
}

static void accept(ShimBtnGesture *g,
    uint64_t ts,
    uint8_t level,
    uint8_t synthesized,
    uint8_t doublePress,
    ShimBtnGestureResult *res)
{
  ShimBtnEdge *e = &res->edge[res->numEdges++];
  e->ts = ts;
  e->level = level;
  e->synthesized = synthesized;

  g->level = level;
  g->lastAcceptedTs = ts;
  if (level)
  {
    g->pressTs = ts;
  }
  else
  {
    releaseAction(g, ts, doublePress, res);
  }
}

void ShimBtnGesture_edge(ShimBtnGesture *g, uint64_t ts, uint8_t level, uint8_t doublePress, ShimBtnGestureResult *res)
{
  clearResult(res);
  level = level ? 1U : 0U;

  /* Unsigned on purpose: a clock set backwards makes this huge, and an edge
   * after a clock change is taken rather than dropped */
  if (ts - g->lastAcceptedTs < BTN_DEBOUNCE_TICKS)
  {
    g->bouncePending = 1;
    g->lastIgnoredTs = ts;
    g->lastIgnoredLevel = level;
    return;
  }

  if (level == g->level)
  {
    /* The edge between this one and the last was missed. If the last edge
     * ignored as bounce went that way, that was it; otherwise all that is
     * known is that it came before this one. */
    uint8_t missed = (uint8_t) !level;
    uint64_t missedTs
        = (g->bouncePending && g->lastIgnoredLevel == missed) ? g->lastIgnoredTs : ts;
    accept(g, missedTs, missed, 1, doublePress, res);
  }
  g->bouncePending = 0;
  accept(g, ts, level, 0, doublePress, res);
}

uint8_t ShimBtnGesture_isRecheckDue(const ShimBtnGesture *g, uint64_t now)
{
  return g->bouncePending && (now - g->lastIgnoredTs >= BTN_DEBOUNCE_TICKS);
}

void ShimBtnGesture_recheck(ShimBtnGesture *g, uint8_t pinLevel, uint8_t doublePress, ShimBtnGestureResult *res)
{
  clearResult(res);
  if (!g->bouncePending)
  {
    return;
  }
  g->bouncePending = 0;

  pinLevel = pinLevel ? 1U : 0U;
  if (pinLevel != g->level)
  {
    /* The last ignored edge was real: it is when the line last changed */
    accept(g, g->lastIgnoredTs, pinLevel, 1, doublePress, res);
  }
}
