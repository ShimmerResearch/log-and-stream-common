/*
 * shimmer_bt_reply.c
 *
 *  Created on: 16 Oct 2026
 *      Author: MarkNolan
 */

#include "shimmer_bt_reply.h"

#include <string.h>

/* Held replies, in the order they were sent. Main loop only. */
static uint8_t pendingReply[BT_PENDING_REPLY_SIZE];
uint16_t gBtReplyPendingLen = 0;
static uint16_t replyDropCount = 0;

uint8_t ShimBtReply_send(uint8_t *buf, uint16_t len)
{
  if (len == 0U)
  {
    return 0;
  }

  /* Something is already held: this one goes behind it, never ahead, or the
   * host would see replies out of order */
  if (gBtReplyPendingLen == 0U && ShimBt_writeReplyToTxBuf(buf, len) == 0U)
  {
    return 0;
  }

  if ((uint16_t) (gBtReplyPendingLen + len) > BT_PENDING_REPLY_SIZE)
  {
    replyDropCount++;
    return 1;
  }

  memcpy(&pendingReply[gBtReplyPendingLen], buf, len);
  gBtReplyPendingLen += len;

  /* The ring may have room for the held bytes already; if not, keep trying */
  ShimBtReply_retry();
  return 0;
}

void ShimBtReply_retry(void)
{
  /* All or nothing: the held bytes can be several replies, and a reply split
   * across two writes could have a status push land inside it */
  if (gBtReplyPendingLen == 0U)
  {
    return;
  }
  if (ShimBt_writeReplyToTxBuf(pendingReply, gBtReplyPendingLen) == 0U)
  {
    gBtReplyPendingLen = 0;
  }
  else
  {
    /* Poll from the main loop until it is out. Deliberately not from the TX
     * path or the data path: on Shimmer3 the RN42 link's throughput proved
     * sensitive to the timing of both (shimmer_bt_reply.h). A reply is held
     * for milliseconds, and only when the ring is full. */
    ShimBt_scheduleReplyRetry();
  }
}

uint8_t ShimBtReply_isPending(void)
{
  return gBtReplyPendingLen != 0U;
}

void ShimBtReply_reset(void)
{
  gBtReplyPendingLen = 0;
}

uint16_t ShimBtReply_getDropCount(void)
{
  return replyDropCount;
}
