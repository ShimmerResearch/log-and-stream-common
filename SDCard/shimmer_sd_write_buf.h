/*
 * shimmer_sd_write_buf.h
 *
 * The SD write buffers that sit between the sample drain and the card,
 * extracted from shimmer_sd_data_file.c so that their state machine can be
 * exercised by a host compiler.
 *
 * This translation unit depends on nothing but <stdint.h> and <string.h>: no
 * FatFs, no HAL, no globals. Everything that needs the platform - f_write, the
 * file split and sync timers, the sync-offset header, queueing TASK_SDWRITE -
 * stays in shimmer_sd_data_file.c and calls in here. That is what lets
 * Test/host/test_sd_write_buf.c replay a sequence of records, writes and stops
 * deterministically on a PC.
 *
 * WHY EXTRACT IT. Three defects lived in the forty lines this replaces, and
 * none of them was reachable by any test:
 *   - a record offered while every buffer was queued was dropped silently, with
 *     no counter and no flag. On Shimmer3, with one buffer, that was every
 *     record between a buffer filling and its write reaching the card;
 *   - the copy happened before the room check, which was a look-ahead for the
 *     NEXT record. Correct only because the record length cannot change within
 *     a logging session - the guarantee lived in the caller, not here;
 *   - the stop-time flush queued the current buffer unconditionally. With every
 *     buffer already queued the sensing index is aliased onto the oldest queued
 *     one, so that pushed the count one past the number of buffers and the
 *     drain loop could never reach zero.
 *
 * ONE CONTEXT. Every entry point here is reached from the task loop
 * (ShimTask_NORM_manage) and never from an interrupt: records arrive on
 * TASK_SAVEDATA, writes on TASK_SDWRITE, the flush on TASK_STOPSENSING. That is
 * why nothing here is volatile and there are no critical sections. If you ever
 * call SdWrBuf_put from an ISR, change this comment first, then the code.
 *
 * THE INVARIANTS, maintained by every function that mutates the struct:
 *   - sensIdx == (wrIdx + numQueued) % NUM_SDWRBUF;
 *   - a queued buffer always holds at least one byte, which is what makes the
 *     stop-time drain loop terminate;
 *   - len[i] <= SD_WRITE_BUF_SIZE for every i, for any record length.
 *
 *  Created on: 16 Sep 2026
 *      Author: MarkNolan
 */

#ifndef SHIMMER_SD_WRITE_BUF_H
#define SHIMMER_SD_WRITE_BUF_H

#include <stdint.h>

/* One buffer, and the most a single f_write ever moves. */
#define SD_WRITE_BUF_SIZE 512

/* How many buffers rotate. Compile-time per platform, because the storage is
 * a fixed array and on the MSP430 each buffer is 512 B of a 16 KB budget.
 * Overridable with -DNUM_SDWRBUF=n so the host test can build the same code at
 * 1 (what Shimmer3 shipped), 2 and 4. */
#if !defined(NUM_SDWRBUF)
#if defined(SHIMMER3)
#define NUM_SDWRBUF 2
#elif defined(SHIMMER3R)
#define NUM_SDWRBUF 4
#else
/* Host build with no platform selected: the Shimmer3 value. */
#define NUM_SDWRBUF 2
#endif
#endif

#if (NUM_SDWRBUF < 1) || (NUM_SDWRBUF > 255)
#error "NUM_SDWRBUF must be 1..255 - the buffer indices are uint8_t"
#endif

typedef struct
{
  uint8_t buf[NUM_SDWRBUF][SD_WRITE_BUF_SIZE];
  /* Bytes used in each buffer. A queued buffer always has len > 0. */
  uint16_t len[NUM_SDWRBUF];
  /* The buffer records are appended to. Meaningless while numQueued reaches
   * NUM_SDWRBUF, because then no buffer is open - see SdWrBuf_bytesInCurrent. */
  uint8_t sensIdx;
  /* The oldest queued buffer, the next one to go to the card. */
  uint8_t wrIdx;
  /* Buffers full and waiting for the card. 0..NUM_SDWRBUF. */
  uint8_t numQueued;

  /* Never reset while logging, so a debugger can read them after a run.
   * SdWrBuf_init clears them, SdWrBuf_reset deliberately does not. */
  struct
  {
    /* A record offered while every buffer was queued. This is the card not
     * keeping up: the record is lost, and the file shows a gap of whole sample
     * periods at a block boundary. */
    uint16_t putsRefusedFull;
    /* head + record could not fit an empty buffer. Cannot happen with the
     * record lengths this firmware produces; counted rather than trusted,
     * because the alternative is writing past the end of a buffer. */
    uint16_t putsRefusedOversize;
  } diag;
} SdWriteBuf;

/* What SdWrBuf_put did with the record. */
typedef enum
{
  /* Nothing was stored. diag says which limit was hit. */
  SDWRBUF_PUT_REFUSED = 0,
  /* Appended to the buffer already in progress. */
  SDWRBUF_PUT_APPENDED,
  /* First record of a buffer; the head, if one was given, precedes it. */
  SDWRBUF_PUT_STARTED_FRESH
} SdWrBufPutResult;

/* Zero everything, diagnostics included. Boot, and the start of logging. */
void SdWrBuf_init(SdWriteBuf *bufsPtr);
/* Drop whatever is buffered and start again, KEEPING the diagnostics so they
 * can be read after a run. Stop and file close. */
void SdWrBuf_reset(SdWriteBuf *bufsPtr);

/* Offer one record. headLen 0 means no head; a head is written only at the
 * start of a fresh buffer, which is exactly when the result is
 * SDWRBUF_PUT_STARTED_FRESH. Queues a full buffer as a side effect, so check
 * SdWrBuf_numQueued afterwards whatever the result. */
SdWrBufPutResult SdWrBuf_put(SdWriteBuf *bufsPtr,
    const uint8_t *headPtr,
    uint8_t headLen,
    const uint8_t *recPtr,
    uint16_t recLen);

/* Queue a partially filled buffer so a stop can flush it. Returns 0 when there
 * is nothing to queue - either the current buffer is empty, or every buffer is
 * queued already and there is no open buffer to queue. */
uint8_t SdWrBuf_queueCurrent(SdWriteBuf *bufsPtr);

/* The oldest queued buffer. Returns 0, leaving the outputs untouched, when
 * nothing is queued. */
uint8_t SdWrBuf_peekWrite(const SdWriteBuf *bufsPtr, const uint8_t **dataPtrOut, uint16_t *lenPtrOut);
/* Release the buffer SdWrBuf_peekWrite returned. Unconditional: a queued
 * buffer is always released, so a drain loop always terminates. */
void SdWrBuf_writeDone(SdWriteBuf *bufsPtr);

uint8_t SdWrBuf_numQueued(const SdWriteBuf *bufsPtr);
/* Bytes in the open buffer, and 0 when no buffer is open. */
uint16_t SdWrBuf_bytesInCurrent(const SdWriteBuf *bufsPtr);

#endif //SHIMMER_SD_WRITE_BUF_H
