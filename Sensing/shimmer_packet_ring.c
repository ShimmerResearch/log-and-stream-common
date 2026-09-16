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

/* Deliberately a >= rather than an equality on the indices. The two are the
 * same while the ring's own invariant holds - starts are refused here, so the
 * count can never pass DATA_BUF_QTY_IN_USE - but an equality fails OPEN if it
 * ever does not: it would report "not full", allow a start, and let the write
 * index lap the read index, which aliases the whole ring. Failing closed costs
 * nothing and is the same class of mistake this module exists to remove. */
uint8_t PktRing_isFull(const PacketRing *ringPtr)
{
  return (uint8_t) (PktRing_count(ringPtr) >= DATA_BUF_QTY_IN_USE);
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

/* A gather may only run against a slot a tick actually started. Without this a
 * gather left queued from a packet the fail-safe already released - or one
 * re-queued by the restart - runs against whatever the write index points at
 * by then, which may be a slot nothing has stamped. */
uint8_t PktRing_gatherMayProceed(PacketRing *ringPtr)
{
  if (PktRing_wrSlot(ringPtr)->samplingStatus != SAMPLING_IN_PROGRESS)
  {
    ringPtr->diag.gathersRefused++;
    return 0;
  }
  return 1;
}

uint8_t PktRing_onComplete(PacketRing *ringPtr)
{
  PACKETBufferTypeDef *slotPtr = PktRing_wrSlot(ringPtr);

  /* Only a packet that is actually in progress can be completed. A completion
   * arriving for anything else is stale - the fail-safe released the slot, or
   * the gather was refused - and closing the slot on it would publish a packet
   * no tick ever stamped. */
  if (slotPtr->samplingStatus != SAMPLING_IN_PROGRESS)
  {
    ringPtr->diag.completionsDropped++;
    return 0;
  }

  /* COMPLETE before the index moves. Reversed, a drain running between the two
   * writes would find a slot inside [rd, wr) that is not yet complete and
   * discard a good packet.
   *
   * The increment is unconditional. It used to be skipped when the ring was
   * full, which could leave a COMPLETE slot at the write index with the index
   * unmoved - after that no tick can ever start again. It cannot overflow:
   * PktRing_onTick refuses to start a packet at DATA_BUF_QTY_IN_USE, so the
   * count here is at most DATA_BUF_QTY_IN_USE - 1. */
  slotPtr->samplingStatus = SAMPLING_COMPLETE;
  ringPtr->wrIdx++;
  return 1;
}

uint8_t PktRing_drainNext(PacketRing *ringPtr, PACKETBufferTypeDef **slotPtrOut)
{
  while (PktRing_count(ringPtr) > 0)
  {
    PACKETBufferTypeDef *slotPtr = PktRing_rdSlot(ringPtr);

    if (slotPtr->samplingStatus == SAMPLING_COMPLETE)
    {
      *slotPtrOut = slotPtr;
      return 1;
    }

    /* Anything else in the drain window was never completed, so it carries no
     * timestamp and must not reach the card or the Bluetooth link. Drop it and
     * keep going - leaving it in place would stall the drain.
     *
     * Deliberately keyed on the status and not on timestampTicks == 0: the
     * tick counter is ticks-since-boot on Shimmer3 and the low 32 bits of the
     * real-world clock on Shimmer3R, and both pass through zero legitimately
     * every 2^32 ticks (36.4 hours). */
    PktRing_resetSlot(ringPtr, (uint8_t) (DATA_BUF_MASK & ringPtr->rdIdx), 0);
    ringPtr->rdIdx++;
    ringPtr->diag.drainSkipped++;
  }
  return 0;
}

void PktRing_drainDone(PacketRing *ringPtr)
{
  PktRing_resetSlot(ringPtr, (uint8_t) (DATA_BUF_MASK & ringPtr->rdIdx), 0);
  ringPtr->rdIdx++;
}
