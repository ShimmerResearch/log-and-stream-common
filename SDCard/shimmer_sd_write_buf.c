/*
 * shimmer_sd_write_buf.c
 *
 *  Created on: 16 Sep 2026
 *      Author: MarkNolan
 */

#include "SDCard/shimmer_sd_write_buf.h"

#include <string.h>

/* Room left in the buffer records are being appended to.
 *
 * Written as a subtraction from the buffer size rather than as
 * "used + recLen > SD_WRITE_BUF_SIZE", because int is 16 bits on the MSP430
 * and the sum form is one careless widening of recLen away from overflowing.
 * The subtraction cannot: len[] is never above SD_WRITE_BUF_SIZE. */
static uint16_t roomInCurrent(const SdWriteBuf *bufsPtr)
{
  return (uint16_t) (SD_WRITE_BUF_SIZE - bufsPtr->len[bufsPtr->sensIdx]);
}

static void advanceSensIdx(SdWriteBuf *bufsPtr)
{
  bufsPtr->numQueued++;
  bufsPtr->sensIdx++;
  if (bufsPtr->sensIdx >= NUM_SDWRBUF)
  {
    bufsPtr->sensIdx = 0;
  }
}

void SdWrBuf_init(SdWriteBuf *bufsPtr)
{
  memset(bufsPtr, 0, sizeof(SdWriteBuf));
}

/* Keeps diag: the counters exist to be read after a run, and this is called
 * from the stop path. Only SdWrBuf_init clears them. */
void SdWrBuf_reset(SdWriteBuf *bufsPtr)
{
  memset(&bufsPtr->buf[0][0], 0, sizeof(bufsPtr->buf));
  memset(&bufsPtr->len[0], 0, sizeof(bufsPtr->len));
  bufsPtr->sensIdx = 0;
  bufsPtr->wrIdx = 0;
  bufsPtr->numQueued = 0;
}

SdWrBufPutResult SdWrBuf_put(SdWriteBuf *bufsPtr,
    const uint8_t *headPtr,
    uint8_t headLen,
    const uint8_t *recPtr,
    uint16_t recLen)
{
  SdWrBufPutResult result;

  if (recLen == 0)
  {
    /* Nothing to store, and nothing to refuse. */
    return SDWRBUF_PUT_APPENDED;
  }

  /* The check the old code did after the copy rather than before it. A record
   * that cannot fit an empty buffer can never be stored, so fail here instead
   * of running off the end of one. */
  if (recLen > (uint16_t) (SD_WRITE_BUF_SIZE - headLen))
  {
    bufsPtr->diag.putsRefusedOversize++;
    return SDWRBUF_PUT_REFUSED;
  }

  if (bufsPtr->numQueued >= NUM_SDWRBUF)
  {
    /* No buffer is open: the card has not kept up. */
    bufsPtr->diag.putsRefusedFull++;
    return SDWRBUF_PUT_REFUSED;
  }

  /* Close the buffer in progress when this record will not fit it. Queueing is
   * deliberately driven by the record that does not fit, not by a look-ahead
   * from the one that did: the look-ahead was only ever right because the
   * record length is fixed for a logging session. Same inequality, one record
   * later, so buffers still end after exactly the same record. */
  if ((bufsPtr->len[bufsPtr->sensIdx] > 0) && (recLen > roomInCurrent(bufsPtr)))
  {
    advanceSensIdx(bufsPtr);
    if (bufsPtr->numQueued >= NUM_SDWRBUF)
    {
      /* The full buffer is queued - the caller still has a write to schedule -
       * but this record has nowhere to go. */
      bufsPtr->diag.putsRefusedFull++;
      return SDWRBUF_PUT_REFUSED;
    }
  }

  if (bufsPtr->len[bufsPtr->sensIdx] == 0)
  {
    if (headLen > 0)
    {
      memcpy(&bufsPtr->buf[bufsPtr->sensIdx][0], headPtr, headLen);
      bufsPtr->len[bufsPtr->sensIdx] = headLen;
    }
    result = SDWRBUF_PUT_STARTED_FRESH;
  }
  else
  {
    result = SDWRBUF_PUT_APPENDED;
  }

  memcpy(&bufsPtr->buf[bufsPtr->sensIdx][bufsPtr->len[bufsPtr->sensIdx]], recPtr, recLen);
  bufsPtr->len[bufsPtr->sensIdx] += recLen;

  return result;
}

/* Refusing to queue when every buffer is already queued is the whole point.
 * At that moment sensIdx is aliased onto the oldest queued buffer, so queueing
 * "the current buffer" would count a buffer twice and leave the drain loop
 * with a count it can never work down to zero. */
uint8_t SdWrBuf_queueCurrent(SdWriteBuf *bufsPtr)
{
  if ((bufsPtr->numQueued >= NUM_SDWRBUF) || (bufsPtr->len[bufsPtr->sensIdx] == 0))
  {
    return 0;
  }

  advanceSensIdx(bufsPtr);
  return 1;
}

uint8_t SdWrBuf_peekWrite(const SdWriteBuf *bufsPtr, const uint8_t **dataPtrOut, uint16_t *lenPtrOut)
{
  if (bufsPtr->numQueued == 0)
  {
    return 0;
  }

  *dataPtrOut = &bufsPtr->buf[bufsPtr->wrIdx][0];
  *lenPtrOut = bufsPtr->len[bufsPtr->wrIdx];
  return 1;
}

/* Releases whatever is at the write index, whatever its length. The old code
 * returned early on a zero-length buffer without decrementing the count, which
 * is how a flush could spin forever. */
void SdWrBuf_writeDone(SdWriteBuf *bufsPtr)
{
  if (bufsPtr->numQueued == 0)
  {
    return;
  }

  bufsPtr->len[bufsPtr->wrIdx] = 0;
  bufsPtr->wrIdx++;
  if (bufsPtr->wrIdx >= NUM_SDWRBUF)
  {
    bufsPtr->wrIdx = 0;
  }
  bufsPtr->numQueued--;
}

uint8_t SdWrBuf_numQueued(const SdWriteBuf *bufsPtr)
{
  return bufsPtr->numQueued;
}

/* 0 when every buffer is queued: there is no open buffer then, and sensIdx is
 * aliased onto a queued one whose length would be a lie. */
uint16_t SdWrBuf_bytesInCurrent(const SdWriteBuf *bufsPtr)
{
  if (bufsPtr->numQueued >= NUM_SDWRBUF)
  {
    return 0;
  }
  return bufsPtr->len[bufsPtr->sensIdx];
}
