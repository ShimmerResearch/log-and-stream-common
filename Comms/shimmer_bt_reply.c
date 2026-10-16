/*
 * shimmer_bt_reply.c
 *
 *  Created on: 16 Oct 2026
 *      Author: MarkNolan
 */

#include "shimmer_bt_reply.h"

#include <string.h>

/* Held replies, in the order they were sent. Touched only from the main loop:
 * the TX-complete interrupt only reads pendingReplyLen, through
 * ShimBtReply_isPending(), to decide whether to schedule a retry. */
static uint8_t pendingReply[BT_PENDING_REPLY_SIZE];
static volatile uint16_t pendingReplyLen = 0;
static uint16_t replyDropCount = 0;

uint8_t ShimBtReply_send(uint8_t *buf, uint16_t len)
{
  if (len == 0U)
  {
    return 0;
  }

  /* Something is already held: this one goes behind it, never ahead, or the
   * host would see replies out of order */
  if (pendingReplyLen == 0U && ShimBt_writeReplyToTxBuf(buf, len) == 0U)
  {
    return 0;
  }

  if ((uint16_t) (pendingReplyLen + len) > BT_PENDING_REPLY_SIZE)
  {
    replyDropCount++;
    return 1;
  }

  memcpy(&pendingReply[pendingReplyLen], buf, len);
  pendingReplyLen += len;

  /* The ring may have room for the held bytes already */
  ShimBtReply_retry();
  return 0;
}

void ShimBtReply_retry(void)
{
  /* All or nothing: the held bytes can be several replies, and a reply split
   * across two writes could have a status push land inside it */
  if (pendingReplyLen != 0U && ShimBt_writeReplyToTxBuf(pendingReply, pendingReplyLen) == 0U)
  {
    pendingReplyLen = 0;
  }
}

uint8_t ShimBtReply_isPending(void)
{
  return pendingReplyLen != 0U;
}

uint8_t ShimBtReply_isRoomForSensorData(uint16_t len)
{
  return pendingReplyLen == 0U
      && (uint32_t) ShimBt_getSpaceInBtTxBuf() >= (uint32_t) len + BT_TX_REPLY_RESERVE;
}

void ShimBtReply_reset(void)
{
  pendingReplyLen = 0;
}

uint16_t ShimBtReply_getDropCount(void)
{
  return replyDropCount;
}
