/*
 * shimmer_packet_ring.h
 *
 * The sample packet ring that sits between the sample timer and the SD/BT
 * writers, extracted from shimmer_sensing.c so that its state machine can be
 * exercised by a host compiler.
 *
 * This translation unit depends on nothing but <stdint.h>: no HAL, no platform
 * headers, no globals. Everything that needs the platform - reading the RTC,
 * starting a gather, queueing a task - stays in shimmer_sensing.c and calls in
 * here. That is what lets Test/host/test_packet_ring.c replay an interleaving
 * of timer and completion events deterministically on a PC.
 *
 * WHY A RING AT ALL. One packet is filled at a time; a sample tick starts it,
 * the platform gathers the enabled channels asynchronously, and a completion
 * callback closes it. The ring decouples that producer from the SD/BT drain,
 * which runs from the task loop and can block for the length of an SD write.
 *
 * THE THREE CONTEXTS. Read this before changing anything here:
 *   - the sample tick runs in a timer ISR on both platforms;
 *   - the gather completion runs in the main loop on Shimmer3 (I2C/SPI) or an
 *     ISR (its ADC DMA), and always in an ISR on Shimmer3R;
 *   - the drain runs in the main loop on both.
 * Each index has exactly one writer: wrIdx is written only by
 * PktRing_onComplete (completion context), rdIdx only by PktRing_drainDone
 * (drain context). Both are 16-bit and aligned, so a load is atomic on the
 * MSP430 and the STM32 alike and PktRing_count() can never see a torn index.
 * There are no critical sections here and there must not need to be.
 *
 *  Created on: 16 Sep 2026
 *      Author: MarkNolan
 */

#ifndef SHIMMER_PACKET_RING_H
#define SHIMMER_PACKET_RING_H

#include <stdint.h>

#define DATA_BUF_SIZE                 128U
#define DATA_BUF_QTY                  8U /* packet buffer (power 2)  */
#define DATA_BUF_QTY_IN_USE           6U /* must be < DATA_BUF_QTY */
#define DATA_BUF_MASK                 (DATA_BUF_QTY - 1UL)

/* Packet structure: [Header(1)][Timestamp(3)][Sensor Data(N)][CRC(N)] */
#define PACKET_HEADER_IDX             0
#define PACKET_HEADER_LEN             1
#define PACKET_TIMESTAMP_IDX          1
#define PACKET_TIMESTAMP_LEN          3
#define FIRST_CH_BYTE_IDX             (PACKET_HEADER_LEN + PACKET_TIMESTAMP_LEN)

/* Number of sample periods a gather may be outstanding before the fail-safe
 * gives up on it and lets the next tick start a fresh packet. Without this a
 * lost completion - a dropped HAL callback, or the deliberate
 * ShimTask_clear(TASK_GATHER_DATA) in ShimSdCfgFile_readSdConfiguration() -
 * strands the slot IN_PROGRESS and sampling never restarts.
 *
 * This is only the value the ring starts with. ShimSens_startSensing()
 * replaces it with the number of periods that make up a fixed WALL-CLOCK
 * timeout, because what the fail-safe is really waiting on - an SD write - has
 * a duration that has nothing to do with the sample rate. Counting periods
 * instead meant the timeout shrank as the rate rose: three periods is 6 ms at
 * 504 Hz, so an ordinary f_write looked like a hang. */
#define PKT_RING_STALL_LIMIT_DEFAULT  3U
/* Sentinel for "never give up", used by the host tests. */
#define PKT_RING_STALL_LIMIT_DISABLED 0xFFFFU

typedef enum
{
  SAMPLING_PACKET_IDLE = 0x00,
  SAMPLING_IN_PROGRESS = 0x01,
  SAMPLING_COMPLETE = 0x02
} samplingStatus_t;

typedef struct
{ //sensor data
  volatile samplingStatus_t samplingStatus;
  volatile uint32_t timestampTicks;
  uint8_t dataBuf[DATA_BUF_SIZE];
} PACKETBufferTypeDef;

/* What the caller must do about this sample tick. */
typedef enum
{
  /* Ring is full - nothing started. Drain before sampling can resume. */
  PKT_TICK_FULL,
  /* Slot is free - the caller stamps it, marks it in progress and gathers. */
  PKT_TICK_START,
  /* A gather is still outstanding - this sample is skipped. */
  PKT_TICK_BUSY,
  /* A gather was outstanding for too long; the slot has been released so the
   * next tick can start fresh. This sample is skipped too. */
  PKT_TICK_STALL_RESET,
  /* Nothing to do. */
  PKT_TICK_IGNORE
} PktTickAction;

typedef struct
{
  /* Free-running, masked at use. Written by one context each - see the file
   * header comment. */
  volatile uint16_t wrIdx;
  volatile uint16_t rdIdx;
  /* Sample periods the current gather has been outstanding. */
  uint16_t stallCount;
  uint16_t stallLimitPeriods;
  /* Written into dataBuf[PACKET_HEADER_IDX] on every slot reset. Held here
   * rather than #included so that this module stays free of the Bluetooth
   * protocol headers. */
  uint8_t headerByte;
  PACKETBufferTypeDef slots[DATA_BUF_QTY];

  /* Never reset while sensing, so a debugger can read them after a run. Each
   * has a single writer context, as the indices do. */
  struct
  {
    volatile uint16_t startsRefusedFull;
    volatile uint16_t stallResets;
    volatile uint16_t gathersRefused;
    volatile uint16_t completionsDropped;
    volatile uint16_t drainSkipped;
  } diag;
} PacketRing;

void PktRing_init(PacketRing *ringPtr, uint8_t headerByte);
void PktRing_reset(PacketRing *ringPtr);
void PktRing_resetSlot(PacketRing *ringPtr, uint8_t index, uint8_t resetAll);

PACKETBufferTypeDef *PktRing_wrSlot(PacketRing *ringPtr);
PACKETBufferTypeDef *PktRing_rdSlot(PacketRing *ringPtr);
PACKETBufferTypeDef *PktRing_nextWrSlot(PacketRing *ringPtr);
PACKETBufferTypeDef *PktRing_prevWrSlot(PacketRing *ringPtr);

uint8_t PktRing_count(const PacketRing *ringPtr);
uint8_t PktRing_isEmpty(const PacketRing *ringPtr);
uint8_t PktRing_isFull(const PacketRing *ringPtr);

PktTickAction PktRing_onTick(PacketRing *ringPtr);
void PktRing_markInProgress(PacketRing *ringPtr);
/* Returns 0 when the write slot is not in progress, i.e. no gather should run
 * against it. */
uint8_t PktRing_gatherMayProceed(PacketRing *ringPtr);
uint8_t PktRing_onComplete(PacketRing *ringPtr);

/* Returns 1 and sets *slotPtrOut when there is a packet to emit. */
uint8_t PktRing_drainNext(PacketRing *ringPtr, PACKETBufferTypeDef **slotPtrOut);
void PktRing_drainDone(PacketRing *ringPtr);

#endif //SHIMMER_PACKET_RING_H
