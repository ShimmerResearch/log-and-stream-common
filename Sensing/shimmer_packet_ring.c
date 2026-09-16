/*
 * shimmer_packet_ring.c
 *
 *  Created on: 16 Sep 2026
 *      Author: MarkNolan
 */

#include "Sensing/shimmer_packet_ring.h"

#include <string.h>

void PktRing_init(PacketRing *ringPtr, uint8_t headerByte)
{
  memset(ringPtr, 0, sizeof(PacketRing));
  ringPtr->headerByte = headerByte;
  ringPtr->stallLimitPeriods = PKT_RING_STALL_LIMIT_DEFAULT;
  PktRing_reset(ringPtr);
}

void PktRing_reset(PacketRing *ringPtr)
{
  uint8_t i;

  ringPtr->rdIdx = ringPtr->wrIdx = 0;
  ringPtr->stallCount = 0;
  for (i = 0; i < DATA_BUF_QTY; i++)
  {
    PktRing_resetSlot(ringPtr, i, 1);
  }
}

/* index must already be masked. resetAll clears the whole sensor payload as
 * well, which is only wanted when sampling is not running - the partial reset
 * leaves the payload alone because the slot is about to be refilled. */
void PktRing_resetSlot(PacketRing *ringPtr, uint8_t index, uint8_t resetAll)
{
  PACKETBufferTypeDef *slotPtr = &ringPtr->slots[index];

  slotPtr->samplingStatus = SAMPLING_PACKET_IDLE;
  slotPtr->timestampTicks = 0;
  if (resetAll)
  {
    memset(&slotPtr->dataBuf[0], 0, DATA_BUF_SIZE);
  }
  else
  {
    slotPtr->dataBuf[PACKET_HEADER_IDX] = ringPtr->headerByte;
    slotPtr->dataBuf[PACKET_TIMESTAMP_IDX] = 0;
    slotPtr->dataBuf[PACKET_TIMESTAMP_IDX + 1] = 0;
    slotPtr->dataBuf[PACKET_TIMESTAMP_IDX + 2] = 0;
  }
}

PACKETBufferTypeDef *PktRing_wrSlot(PacketRing *ringPtr)
{
  return &ringPtr->slots[DATA_BUF_MASK & ringPtr->wrIdx];
}

PACKETBufferTypeDef *PktRing_rdSlot(PacketRing *ringPtr)
{
  return &ringPtr->slots[DATA_BUF_MASK & ringPtr->rdIdx];
}

PACKETBufferTypeDef *PktRing_nextWrSlot(PacketRing *ringPtr)
{
  return &ringPtr->slots[DATA_BUF_MASK & (ringPtr->wrIdx + 1)];
}

/* The slot before the write index. The write index points at the sample being
 * filled now, so the previous slot is never the current target and is safe to
 * read without a critical section. Always non-NULL, but it only holds a
 * completed sample once at least one sample has been captured - before that
 * (right after PktRing_reset, where "previous" wraps to the last slot) it is
 * an idle buffer. Callers must only use it while sampling is active. */
PACKETBufferTypeDef *PktRing_prevWrSlot(PacketRing *ringPtr)
{
  return &ringPtr->slots[DATA_BUF_MASK & (ringPtr->wrIdx - 1)];
}

uint8_t PktRing_count(const PacketRing *ringPtr)
{
  uint16_t wrIdx = ringPtr->wrIdx;
  uint16_t rdIdx = ringPtr->rdIdx;
  return (uint8_t) (DATA_BUF_MASK & (wrIdx - rdIdx));
}

uint8_t PktRing_isEmpty(const PacketRing *ringPtr)
{
  return ringPtr->rdIdx == ringPtr->wrIdx;
}

uint8_t PktRing_isFull(const PacketRing *ringPtr)
{
  uint16_t wrIdx = ringPtr->wrIdx;
  uint16_t rdIdx = ringPtr->rdIdx;
  return ((DATA_BUF_MASK & rdIdx)
      == (DATA_BUF_MASK & (wrIdx + (DATA_BUF_QTY - DATA_BUF_QTY_IN_USE))));
}

PktTickAction PktRing_onTick(PacketRing *ringPtr)
{
  PACKETBufferTypeDef *slotPtr = PktRing_wrSlot(ringPtr);

  if (PktRing_isFull(ringPtr))
  {
    /* Nothing can start until the drain catches up. */
    ringPtr->diag.startsRefusedFull++;
    return PKT_TICK_FULL;
  }

  if (slotPtr->samplingStatus == SAMPLING_PACKET_IDLE)
  {
    ringPtr->stallCount = 0;
    return PKT_TICK_START;
  }

  if (slotPtr->samplingStatus == SAMPLING_IN_PROGRESS)
  {
    ringPtr->stallCount++;
    if ((ringPtr->stallLimitPeriods != PKT_RING_STALL_LIMIT_DISABLED)
        && (ringPtr->stallCount > ringPtr->stallLimitPeriods))
    {
      /* Release the slot so the next tick can start a new packet. The payload
       * is left alone: a gather may still be writing into it. */
      slotPtr->samplingStatus = SAMPLING_PACKET_IDLE;
      ringPtr->diag.stallResets++;
      return PKT_TICK_STALL_RESET;
    }
    return PKT_TICK_BUSY;
  }

  return PKT_TICK_IGNORE;
}

void PktRing_markInProgress(PacketRing *ringPtr)
{
  PktRing_wrSlot(ringPtr)->samplingStatus = SAMPLING_IN_PROGRESS;
}

uint8_t PktRing_onComplete(PacketRing *ringPtr)
{
  PktRing_wrSlot(ringPtr)->samplingStatus = SAMPLING_COMPLETE;

  if (!PktRing_isFull(ringPtr))
  {
    ringPtr->wrIdx++;
  }
  return 1;
}

uint8_t PktRing_drainNext(PacketRing *ringPtr, PACKETBufferTypeDef **slotPtrOut)
{
  if (PktRing_count(ringPtr) == 0)
  {
    return 0;
  }

  *slotPtrOut = PktRing_rdSlot(ringPtr);
  return 1;
}

void PktRing_drainDone(PacketRing *ringPtr)
{
  PktRing_resetSlot(ringPtr, (uint8_t) (DATA_BUF_MASK & ringPtr->rdIdx), 0);
  ringPtr->rdIdx++;
}
