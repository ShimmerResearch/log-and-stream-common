/*
 * Host-side test for SDCard/shimmer_sd_write_buf.c.
 *
 * That module includes nothing but <stdint.h> and <string.h>, so this needs no
 * MCU toolchain, no HAL and no target hardware, and no stubs either. Run by
 * Test/host/Makefile.
 *
 * WHAT THIS IS FOR. The states worth testing need a card slower than the sample
 * rate: a buffer filling while the previous one is still going to the card, a
 * stop arriving while every buffer is queued. You cannot ask hardware for those
 * on demand, and when they happened the firmware lost samples without saying
 * so. Here each one is a handful of calls.
 *
 * The buffer count is a compile-time constant, so the Makefile builds this file
 * once per count in SD_WRBUF_COUNTS: 1 is what Shimmer3 shipped and pins down
 * the record loss that caused, 4 is what both platforms ship now, 2 sits
 * between them. Scenarios whose outcome depends on the count are guarded on
 * NUM_SDWRBUF.
 *
 * HOW LOSS IS DETECTED. Every record the module accepts is appended to a ring
 * of expected bytes, and every byte of every buffer written out is compared
 * against the front of that ring. A dropped, duplicated or reordered record
 * shows up as a mismatch, and anything still in the ring at the end was
 * accepted but never written. The ring is small on purpose: only a few buffers
 * can be outstanding, so it never has to hold a whole run and the cases can
 * iterate as long as they like.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Without the
 * guard it collides with the firmware's own main() at link time, in both the
 * STM32 and the MSP430 build. Only the host-test build passes
 * -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "SDCard/shimmer_sd_write_buf.h"
#include "host_test.h"

/* The 21-byte record and 9-byte sync head of the recordings this was found in:
 * 504 Hz, gyro + wide-range accel + magnetometer, logging as a sync node. */
#define FIELD_REC_LEN  21
#define FIELD_HEAD_LEN 9

/* Comfortably more than the buffers that can be outstanding at once, plus the
 * largest single record. */
#define TAPE_RING      ((NUM_SDWRBUF + 2) * SD_WRITE_BUF_SIZE)

typedef struct
{
  SdWriteBuf w;
  uint8_t canary[16];

  /* The most any buffer has ever held. Bytes past it were never written, so
   * they must still be poison - that is what catches a copy running off the
   * end of a buffer, which on a 2-D array lands in the next one. Comparing
   * against len[] instead would not work: releasing a buffer sets its length
   * to zero and leaves the bytes alone. */
  uint16_t hwm[NUM_SDWRBUF];

  /* Bytes accepted but not yet seen on the card, oldest first. */
  uint8_t tape[TAPE_RING];
  uint32_t tapeIn;
  uint32_t tapeOut;
  uint32_t mismatches;

  uint8_t head[FIELD_HEAD_LEN];
  uint8_t headLen;
  uint32_t blocksWritten;
  uint32_t headsSeen;
  uint8_t nextRecByte;
} Sim;

static void sim_init(Sim *sim, uint8_t headLen)
{
  uint8_t i;

  memset(sim, 0, sizeof(Sim));
  memset(sim->canary, 0xA5, sizeof(sim->canary));
  SdWrBuf_init(&sim->w);
  /* Poison the storage so a write past a buffer's end is visible. */
  memset(&sim->w.buf[0][0], 0xA5, sizeof(sim->w.buf));
  sim->headLen = headLen;
  for (i = 0; i < FIELD_HEAD_LEN; i++)
  {
    sim->head[i] = (uint8_t) (0xE0 + i);
  }
}

/* Checked after every call that can touch the state. */
static void sim_checkInvariants(Sim *sim)
{
  uint8_t i;
  uint16_t j;

  expectTrue("numQueued never exceeds the number of buffers", sim->w.numQueued <= NUM_SDWRBUF);
  if (sim->w.numQueued > NUM_SDWRBUF)
  {
    return;
  }

  expectU("sensIdx == (wrIdx + numQueued) % NUM_SDWRBUF", sim->w.sensIdx,
      (uint8_t) ((sim->w.wrIdx + sim->w.numQueued) % NUM_SDWRBUF));

  for (i = 0; i < NUM_SDWRBUF; i++)
  {
    if (sim->w.len[i] > SD_WRITE_BUF_SIZE)
    {
      printf("         buffer %u length %u exceeds the buffer\n", (unsigned) i,
          (unsigned) sim->w.len[i]);
      expectTrue("no buffer is longer than SD_WRITE_BUF_SIZE", 0);
      return;
    }
    if (sim->w.len[i] > sim->hwm[i])
    {
      sim->hwm[i] = sim->w.len[i];
    }
    for (j = sim->hwm[i]; j < SD_WRITE_BUF_SIZE; j++)
    {
      if (sim->w.buf[i][j] != 0xA5)
      {
        printf(
            "         buffer %u written past its high-water mark at byte %u\n",
            (unsigned) i, (unsigned) j);
        expectTrue("nothing is written past a buffer's high-water mark", 0);
        return;
      }
    }
  }
  for (i = 0; i < sizeof(sim->canary); i++)
  {
    if (sim->canary[i] != 0xA5)
    {
      expectTrue("nothing is written past the end of the struct", 0);
      return;
    }
  }
}

/* One record, contents derived from a counter so consecutive records differ. */
static SdWrBufPutResult sim_put(Sim *sim, uint16_t recLen)
{
  /* Zeroed rather than left to the stack: a zero-length record is one of the
   * cases under test, and gcc cannot otherwise prove the array is initialised
   * before it is passed. */
  uint8_t rec[SD_WRITE_BUF_SIZE + 4] = { 0 };
  SdWrBufPutResult r;
  uint16_t i;

  for (i = 0; i < recLen && i < sizeof(rec); i++)
  {
    rec[i] = (uint8_t) (sim->nextRecByte + i);
  }
  sim->nextRecByte++;

  r = SdWrBuf_put(&sim->w, sim->headLen ? sim->head : 0, sim->headLen, rec, recLen);
  sim_checkInvariants(sim);

  if (r != SDWRBUF_PUT_REFUSED && recLen > 0)
  {
    expectTrue("the module hands records over before the test ring fills",
        (sim->tapeIn - sim->tapeOut) + recLen <= TAPE_RING);
    if ((sim->tapeIn - sim->tapeOut) + recLen > TAPE_RING)
    {
      return r;
    }
    for (i = 0; i < recLen; i++)
    {
      sim->tape[sim->tapeIn % TAPE_RING] = rec[i];
      sim->tapeIn++;
    }
  }
  return r;
}

/* What TASK_SDWRITE does: take the oldest queued buffer, write it, release it.
 * Returns 0 when there was nothing queued. */
static uint8_t sim_taskSdWrite(Sim *sim)
{
  const uint8_t *data;
  uint16_t len;
  uint16_t offset = 0;
  uint16_t i;

  if (!SdWrBuf_peekWrite(&sim->w, &data, &len))
  {
    return 0;
  }

  expectTrue("a queued buffer is never empty, so the stop-time flush "
             "terminates",
      len != 0);

  if (sim->headLen > 0)
  {
    if (len < sim->headLen || memcmp(data, sim->head, sim->headLen) != 0)
    {
      expectTrue("every written buffer starts with the sync head", 0);
    }
    else
    {
      sim->headsSeen++;
      offset = sim->headLen;
    }
  }

  for (i = offset; i < len; i++)
  {
    if (sim->tapeOut == sim->tapeIn)
    {
      if (sim->mismatches == 0)
      {
        expectTrue("no more bytes are written than were accepted", 0);
      }
      sim->mismatches++;
      break;
    }
    if (data[i] != sim->tape[sim->tapeOut % TAPE_RING])
    {
      if (sim->mismatches == 0)
      {
        expectTrue("every written byte is the next byte that was accepted", 0);
      }
      sim->mismatches++;
    }
    sim->tapeOut++;
  }
  sim->blocksWritten++;

  SdWrBuf_writeDone(&sim->w);
  sim_checkInvariants(sim);
  return 1;
}

/* ShimSdDataFile_writeAllBufsToSd. The iteration cap is the point: a
 * regression of the stop-time flush defect must fail the test, not hang it. */
static uint32_t sim_stop(Sim *sim)
{
  uint32_t writes = 0;
  uint32_t cap = NUM_SDWRBUF + 2;

  SdWrBuf_queueCurrent(&sim->w);
  sim_checkInvariants(sim);

  while (SdWrBuf_numQueued(&sim->w) > 0)
  {
    if (writes >= cap)
    {
      expectTrue("the stop-time flush terminates rather than writing more "
                 "buffers than exist",
          0);
      break;
    }
    if (!sim_taskSdWrite(sim))
    {
      expectTrue("a queued buffer is always accepted by the write path", 0);
      break;
    }
    writes++;
  }
  return writes;
}

/* Everything accepted has been written, in order. */
static void sim_checkNothingLost(Sim *sim)
{
  if (sim->tapeOut != sim->tapeIn)
  {
    printf("         %lu accepted bytes never reached the card\n",
        (unsigned long) (sim->tapeIn - sim->tapeOut));
  }
  expectU("every accepted byte reached the card", sim->tapeIn - sim->tapeOut, 0);
  expectU("no byte was written out of order", sim->mismatches, 0);
}

/* Fill the open buffer until one more field record would not fit. */
static void sim_fillCurrent(Sim *sim)
{
  while (SdWrBuf_bytesInCurrent(&sim->w) + FIELD_REC_LEN <= SD_WRITE_BUF_SIZE)
  {
    if (sim_put(sim, FIELD_REC_LEN) == SDWRBUF_PUT_REFUSED)
    {
      return;
    }
  }
}

/* Queue every buffer, leaving nothing open. */
static void sim_queueEverything(Sim *sim)
{
  uint8_t i;

  for (i = 0; i < NUM_SDWRBUF; i++)
  {
    sim_fillCurrent(sim);
    sim_put(sim, FIELD_REC_LEN);
  }
}

/* The ordinary cycle: records in, one block out, nothing lost. */
static void test_normal_cycle(void)
{
  Sim sim;

  testCase("the ordinary fill-and-write cycle");
  sim_init(&sim, 0);
  expectU("first record starts a buffer", sim_put(&sim, FIELD_REC_LEN), SDWRBUF_PUT_STARTED_FRESH);
  expectU("second record appends", sim_put(&sim, FIELD_REC_LEN), SDWRBUF_PUT_APPENDED);
  expectU("nothing queued yet", SdWrBuf_numQueued(&sim.w), 0);

  sim_fillCurrent(&sim);
  expectU("still nothing queued before the record that will not fit",
      SdWrBuf_numQueued(&sim.w), 0);

  /* The record that does not fit closes the buffer. Where it goes then depends
   * on whether there is another buffer for it to go into. */
#if NUM_SDWRBUF >= 2
  expectU("the record that does not fit starts the next buffer",
      sim_put(&sim, FIELD_REC_LEN), SDWRBUF_PUT_STARTED_FRESH);
  expectU("no refusals", sim.w.diag.putsRefusedFull, 0);
#else
  expectU("with one buffer there is nowhere for it to go",
      sim_put(&sim, FIELD_REC_LEN), SDWRBUF_PUT_REFUSED);
  expectU("and the loss is counted", sim.w.diag.putsRefusedFull, 1);
#endif
  expectU("the full buffer is queued", SdWrBuf_numQueued(&sim.w), 1);

  expectU("the write drains it", sim_taskSdWrite(&sim), 1);
  expectU("nothing left queued", SdWrBuf_numQueued(&sim.w), 0);

  sim_stop(&sim);
  expectU("everything reached the card", SdWrBuf_numQueued(&sim.w), 0);
  sim_checkNothingLost(&sim);
}

/* THE DEFECT. The drain hands over several records in a row without the task
 * loop running in between, so a batch can cross the 512-byte boundary. With one
 * buffer every record from the crossing on was dropped; with more the batch
 * fits, and what is offered is what gets written. */
static void test_batch_crossing_boundary(void)
{
  Sim sim;
  int i;
  int refused = 0;

  testCase("a drain batch that crosses the buffer boundary");
  sim_init(&sim, 0);
  sim_fillCurrent(&sim);

  /* One ShimSens_saveData run: six records, no write in between. */
  for (i = 0; i < 6; i++)
  {
    if (sim_put(&sim, FIELD_REC_LEN) == SDWRBUF_PUT_REFUSED)
    {
      refused++;
    }
  }

#if NUM_SDWRBUF >= 2
  expectU("no record in the batch was refused", (uint32_t) refused, 0);
  expectU("none counted as refused", sim.w.diag.putsRefusedFull, 0);
#else
  /* What Shimmer3 shipped: the buffer fills mid-batch and the rest is gone. */
  expectU("the whole batch was dropped from the crossing on", (uint32_t) refused, 6);
  expectU("and every drop was counted", sim.w.diag.putsRefusedFull, (uint32_t) refused);
#endif

  sim_stop(&sim);
  sim_checkNothingLost(&sim);
}

/* When the card really is too slow the record is still lost - but counted, and
 * the machine recovers as soon as a buffer frees. */
static void test_refusal_is_counted_and_recovers(void)
{
  Sim sim;
  uint16_t refusedSoFar;

  testCase("a refusal when every buffer is queued");
  sim_init(&sim, 0);
  sim_queueEverything(&sim);
  expectU("every buffer is queued", SdWrBuf_numQueued(&sim.w), NUM_SDWRBUF);
  expectU("no buffer is open", SdWrBuf_bytesInCurrent(&sim.w), 0);

  refusedSoFar = sim.w.diag.putsRefusedFull;

  expectU("a record offered now is refused", sim_put(&sim, FIELD_REC_LEN), SDWRBUF_PUT_REFUSED);
  expectU("and counted", sim.w.diag.putsRefusedFull, refusedSoFar + 1);
  expectU("so is the next", sim_put(&sim, FIELD_REC_LEN), SDWRBUF_PUT_REFUSED);
  expectU("counted again", sim.w.diag.putsRefusedFull, refusedSoFar + 2);
  expectU("the refusals changed nothing else", SdWrBuf_numQueued(&sim.w), NUM_SDWRBUF);

  sim_taskSdWrite(&sim);
  expectU("one buffer freed", SdWrBuf_numQueued(&sim.w), NUM_SDWRBUF - 1);
  expectU("and the next record goes into it", sim_put(&sim, FIELD_REC_LEN),
      SDWRBUF_PUT_STARTED_FRESH);
  expectU("the counter still reads what happened", sim.w.diag.putsRefusedFull,
      refusedSoFar + 2);

  SdWrBuf_reset(&sim.w);
  expectU("a reset keeps the counter for the debugger",
      sim.w.diag.putsRefusedFull, refusedSoFar + 2);
  SdWrBuf_init(&sim.w);
  expectU("a new logging session clears it", sim.w.diag.putsRefusedFull, 0);
}

/* Both indices wrap. Long enough to lap the buffers many times, with the write
 * lagging by a different amount each cycle. */
static void test_indices_wrap(void)
{
  Sim sim;
  int cycle;

  testCase("1000 fill-and-drain cycles");
  sim_init(&sim, 0);
  for (cycle = 0; cycle < 1000; cycle++)
  {
    sim_fillCurrent(&sim);
    sim_put(&sim, FIELD_REC_LEN);

    /* Let the queue build up short of full, then drain it. */
    if ((SdWrBuf_numQueued(&sim.w) + 1 >= NUM_SDWRBUF) || ((cycle % 3) == 0))
    {
      while (SdWrBuf_numQueued(&sim.w) > 0)
      {
        sim_taskSdWrite(&sim);
      }
    }

    if (sim.w.sensIdx >= NUM_SDWRBUF || sim.w.wrIdx >= NUM_SDWRBUF)
    {
      expectTrue("neither index ever leaves the array", 0);
      return;
    }
  }
  sim_stop(&sim);
  sim_checkNothingLost(&sim);
#if NUM_SDWRBUF >= 2
  expectU("nothing was refused", sim.w.diag.putsRefusedFull, 0);
#else
  /* One buffer, so every cycle loses the record that crosses the boundary -
   * one sample per 512 bytes of file, for the whole recording. */
  expectU("one record lost per block", sim.w.diag.putsRefusedFull, 1000);
#endif
}

/* The bound must hold for any record length, not only the constant one the old
 * look-ahead silently depended on. */
static void test_variable_record_lengths(void)
{
  Sim sim;
  uint16_t recLen;
  int cycle;
  uint8_t headLen;

  testCase("record lengths from 1 to 254, with and without a sync head");
  for (headLen = 0; headLen <= FIELD_HEAD_LEN; headLen = (uint8_t) (headLen + FIELD_HEAD_LEN))
  {
    sim_init(&sim, headLen);

    recLen = 1;
    for (cycle = 0; cycle < 4000; cycle++)
    {
      sim_put(&sim, recLen);
      recLen++;
      if (recLen > 254)
      {
        recLen = 1;
      }
      while (SdWrBuf_numQueued(&sim.w) > 0)
      {
        sim_taskSdWrite(&sim);
      }
    }
    sim_stop(&sim);
    sim_checkNothingLost(&sim);
#if NUM_SDWRBUF >= 2
    expectU("nothing refused for lack of room", sim.w.diag.putsRefusedFull, 0);
#else
    /* One buffer: the record that closes a buffer has nowhere to go, whatever
     * its length. Counted, which is the point - it used to be silent. */
    expectTrue("the boundary records were lost, and counted",
        sim.w.diag.putsRefusedFull > 0);
#endif
    expectU("nothing refused as oversize", sim.w.diag.putsRefusedOversize, 0);

    /* A zero-length record changes nothing. */
    expectU("zero-length record is a no-op", sim_put(&sim, 0), SDWRBUF_PUT_APPENDED);
    expectU("no buffer opened for it", SdWrBuf_bytesInCurrent(&sim.w), 0);

    /* One that could not fit an empty buffer fails closed. */
    expectU("oversize record is refused",
        sim_put(&sim, (uint16_t) (SD_WRITE_BUF_SIZE - headLen + 1)), SDWRBUF_PUT_REFUSED);
    expectU("and counted separately", sim.w.diag.putsRefusedOversize, 1);
    expectU("and nothing was written", SdWrBuf_bytesInCurrent(&sim.w), 0);
    expectU("the largest record that does fit is accepted",
        sim_put(&sim, (uint16_t) (SD_WRITE_BUF_SIZE - headLen)), SDWRBUF_PUT_STARTED_FRESH);
    sim_stop(&sim);
    sim_checkNothingLost(&sim);
  }
}

/* THE HANG. A stop is a higher-priority task than the SD write, so it can
 * arrive with every buffer queued. The flush must still finish, and must write
 * each buffer exactly once. */
static void test_stop_flush_terminates(void)
{
  Sim sim;
  uint8_t i;
  uint32_t writes;

  testCase("a stop arriving with every buffer queued");
  sim_init(&sim, 0);
  sim_queueEverything(&sim);
  expectU("every buffer queued before the stop", SdWrBuf_numQueued(&sim.w), NUM_SDWRBUF);
  writes = sim_stop(&sim);
  expectU("each queued buffer written exactly once", writes, NUM_SDWRBUF);
  expectU("nothing left queued", SdWrBuf_numQueued(&sim.w), 0);
  sim_checkNothingLost(&sim);

  testCase("a stop with one buffer partly filled");
  sim_init(&sim, 0);
  for (i = 0; i + 1 < NUM_SDWRBUF; i++)
  {
    sim_fillCurrent(&sim);
    sim_put(&sim, FIELD_REC_LEN);
  }
  sim_put(&sim, FIELD_REC_LEN);
  expectTrue("one buffer is open with data in it", SdWrBuf_bytesInCurrent(&sim.w) > 0);
  writes = sim_stop(&sim);
  expectU("the partial buffer is written too", writes, NUM_SDWRBUF);
  expectU("nothing left queued", SdWrBuf_numQueued(&sim.w), 0);
  sim_checkNothingLost(&sim);

  testCase("a stop with nothing buffered");
  sim_init(&sim, 0);
  writes = sim_stop(&sim);
  expectU("a stop with nothing buffered writes nothing", writes, 0);
}

/* The sync offset leads every buffer and is consumed exactly once per buffer,
 * which is what lets the caller reset it on STARTED_FRESH. */
static void test_sync_head(void)
{
  Sim sim;
  int cycle;
  uint32_t fresh = 0;

  testCase("the sync offset at the head of every block");
  sim_init(&sim, FIELD_HEAD_LEN);
  for (cycle = 0; cycle < 200; cycle++)
  {
    if (sim_put(&sim, FIELD_REC_LEN) == SDWRBUF_PUT_STARTED_FRESH)
    {
      fresh++;
    }
    while (SdWrBuf_numQueued(&sim.w) > 0)
    {
      sim_taskSdWrite(&sim);
    }
  }
  sim_stop(&sim);

  expectU("one head per block written", sim.headsSeen, sim.blocksWritten);
  expectU("one fresh start per block", fresh, sim.blocksWritten);
  sim_checkNothingLost(&sim);

  /* With no head the whole buffer is record data, which the byte-for-byte
   * comparison in sim_taskSdWrite covers. */
  testCase("no sync offset when none is supplied");
  sim_init(&sim, 0);
  for (cycle = 0; cycle < 200; cycle++)
  {
    sim_put(&sim, FIELD_REC_LEN);
    while (SdWrBuf_numQueued(&sim.w) > 0)
    {
      sim_taskSdWrite(&sim);
    }
  }
  sim_stop(&sim);
  expectU("no heads when none was given", sim.headsSeen, 0);
  sim_checkNothingLost(&sim);
}

/* A block still holds the same records it always did. Host parsers slice files
 * on this arithmetic, so moving the queue decision must not move it. */
static void test_block_layout_unchanged(void)
{
  Sim sim;
  uint16_t recLen;
  uint8_t headLen;

  testCase("records per block, for every record length a session can have");
  for (headLen = 0; headLen <= FIELD_HEAD_LEN; headLen = (uint8_t) (headLen + FIELD_HEAD_LEN))
  {
    for (recLen = 12; recLen <= 64; recLen++)
    {
      uint16_t perBlock = (uint16_t) ((SD_WRITE_BUF_SIZE - headLen) / recLen);
      uint16_t i;
      uint8_t closedEarly = 0;

      sim_init(&sim, headLen);
      for (i = 0; i < perBlock; i++)
      {
        if (SdWrBuf_numQueued(&sim.w) != 0)
        {
          printf("         block closed after %u records, expected %u "
                 "(record %u, head %u)\n",
              (unsigned) i, (unsigned) perBlock, (unsigned) recLen, (unsigned) headLen);
          closedEarly = 1;
          break;
        }
        sim_put(&sim, recLen);
      }
      expectTrue("a block holds every record that fits", !closedEarly);
      if (closedEarly)
      {
        return;
      }

      sim_put(&sim, recLen);
      expectU("the block closes on the record that does not fit",
          SdWrBuf_numQueued(&sim.w), 1);
      expectU("and is head + perBlock * record bytes long",
          sim.w.len[sim.w.wrIdx], (uint16_t) (headLen + perBlock * recLen));
    }
  }
}

int main(void)
{
  printf("SD write buffers, NUM_SDWRBUF = %u\n\n", (unsigned) NUM_SDWRBUF);
  hostTestSilenceUnused();

  test_normal_cycle();
  test_batch_crossing_boundary();
  test_refusal_is_counted_and_recovers();
  test_indices_wrap();
  test_variable_record_lengths();
  test_stop_flush_terminates();
  test_sync_head();
  test_block_layout_unchanged();

  return hostTestReport("sd_write_buf");
}

#endif /* SHIMMER_HOST_TEST */
