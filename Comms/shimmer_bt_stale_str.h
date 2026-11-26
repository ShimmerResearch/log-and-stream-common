/*
 * shimmer_bt_stale_str.h
 *
 * Spotting a module status string that the module has stopped sending
 * (Shimmer3, RN4678 status strings).
 *
 * The module sends each status string back to back, so a string the parser
 * is still reading a whole tick after it last saw it, with not a byte added,
 * is not one the module is sending. It was opened by the closing '%' of a
 * string the parser lost its place in, read in command position.
 *
 * Header-only so that the decision can be tested on a host without the
 * receive path's registers; ShimBt_rxDropStaleStatusString() supplies them.
 */

#ifndef SHIMMER_BT_STALE_STR_H_
#define SHIMMER_BT_STALE_STR_H_

#include <stdint.h>

typedef struct
{
  uint8_t seenOpen;
  uint8_t lastIndex;
  uint16_t lastRemaining;
} ShimBtStaleStr_t;

/* Call once per tick with whether a status string is open, the number of
 * bytes the receive is still waiting for, and the string's length so far.
 * Returns 1 when the same open string, with no byte added, was also seen on
 * the previous call. The watch then starts afresh, so a string is reported
 * at most once every two calls. */
static inline uint8_t
ShimBt_staleStatusStrTick(ShimBtStaleStr_t *watch, uint8_t isOpen, uint16_t remaining, uint8_t index)
{
  if (!isOpen)
  {
    watch->seenOpen = 0;
    return 0;
  }
  if (!watch->seenOpen || remaining != watch->lastRemaining || index != watch->lastIndex)
  {
    watch->seenOpen = 1;
    watch->lastRemaining = remaining;
    watch->lastIndex = index;
    return 0;
  }
  watch->seenOpen = 0;
  return 1;
}

#endif /* SHIMMER_BT_STALE_STR_H_ */
