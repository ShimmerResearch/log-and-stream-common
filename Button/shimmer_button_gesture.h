/*
 * shimmer_button_gesture.h
 *
 * What the user button's edges mean. Edges arrive from the interrupt with
 * their times; this debounces them and decides when a press starts or stops
 * logging. Pure logic on timestamps with no platform dependencies, so the host
 * test suite can drive it edge by edge.
 *
 * Debounce: an edge less than BTN_DEBOUNCE_TICKS after the last accepted one
 * is ignored as bounce. The last ignored edge may still be the real one - a
 * tap shorter than the debounce, or a bounce that ends at the other level - so
 * the caller rechecks the pin once the line has been quiet for the debounce
 * time, and a difference becomes an edge at the time of the last ignored one.
 * An edge in the same direction as the last accepted one means the edge
 * between them was missed; it is put back, marked as synthesized.
 *
 * Two ways a press can start or stop logging:
 * - Single press, the button's original behaviour: a release more than 0.5 s
 *   after the previous release, unless the press lasted 5 s or more.
 * - Double press, configuration byte 232's button modes 3 and 4. Mode 4 also
 *   records every press as an event marker, and the double press is what keeps
 *   a marker from stopping a recording: two presses of at most 300 ms each,
 *   released at most 400 ms apart.
 */

#ifndef SHIMMER_BUTTON_GESTURE_H_
#define SHIMMER_BUTTON_GESTURE_H_

#include <stdint.h>

/* All in RTC ticks at 32768 Hz */
#define BTN_DEBOUNCE_TICKS        819UL    /* 25 ms */
#define BTN_REPEAT_GAP_TICKS      16384UL  /* 0.5 s, single press */
#define BTN_LONG_PRESS_TICKS      163840UL /* 5 s, single press */
#define BTN_SHORT_PRESS_MAX_TICKS 9830UL   /* 300 ms, double press */
#define BTN_DOUBLE_GAP_TICKS      13107UL  /* 400 ms, double press */

typedef struct
{
  uint8_t level; /* debounced: 1 = pressed */
  uint8_t bouncePending;
  uint8_t lastIgnoredLevel;
  uint64_t lastAcceptedTs;
  uint64_t lastIgnoredTs;
  uint64_t pressTs;       /* 0 = no press seen */
  uint64_t releasePrevTs; /* single press: the previous release */
  uint64_t firstTapTs;    /* double press: the first short press's release */
} ShimBtnGesture;

typedef struct
{
  uint64_t ts;
  uint8_t level;
  uint8_t synthesized;
} ShimBtnEdge;

typedef struct
{
  /* Accepted edges, oldest first: a synthesized one may precede the edge */
  uint8_t numEdges;
  ShimBtnEdge edge[2];
  /* A gesture asks to start or stop logging; the caller decides if it may */
  uint8_t toggleLogging;
} ShimBtnGestureResult;

void ShimBtnGesture_init(ShimBtnGesture *g);
void ShimBtnGesture_edge(ShimBtnGesture *g,
    uint64_t ts,
    uint8_t level,
    uint8_t doublePress,
    ShimBtnGestureResult *res);
uint8_t ShimBtnGesture_isRecheckDue(const ShimBtnGesture *g, uint64_t now);
void ShimBtnGesture_recheck(ShimBtnGesture *g,
    uint8_t pinLevel,
    uint8_t doublePress,
    ShimBtnGestureResult *res);

#endif /* SHIMMER_BUTTON_GESTURE_H_ */
