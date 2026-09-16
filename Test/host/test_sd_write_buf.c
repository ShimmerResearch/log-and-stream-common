/*
 * Host-side test for the SD write buffers.
 *
 * shimmer_sd_write_buf.c depends on nothing but <stdint.h> and <string.h>, so
 * the buffer state machine can be driven on a PC. Run by
 * .github/workflows/host-tests.yml.
 *
 * WHAT THIS IS FOR. The interesting states need a card slower than the sample
 * rate: a buffer filling while the previous one is still being written, a stop
 * arriving while every buffer is queued. You cannot ask real hardware for those
 * on demand, and when they did happen the firmware lost samples without saying
 * so. Here each one is a handful of calls and runs on every push.
 *
 * The buffer count is a compile-time constant, so the workflow builds this file
 * once per count: 1 is what Shimmer3 shipped and pins down the record loss that
 * caused, 2 is Shimmer3 now, 4 is Shimmer3R. Scenarios whose outcome depends on
 * the count are guarded on NUM_SDWRBUF.
 *
 * HOW LOSS IS DETECTED. Every record the module accepts is appended to a ring
 * of expected bytes, and every byte of every buffer written out is compared
 * against the front of that ring. A dropped, duplicated or reordered record
 * shows up as a mismatch, and anything still in the ring at the end was
 * accepted but never written. The ring is small on purpose: at most
 * NUM_SDWRBUF + 1 buffers can be outstanding, so it never needs to hold a whole
 * run, and the tests can iterate for as long as they like.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions (see the
 * log-and-stream-common <entry kind="sourcePath"> in the Shimmer3R .cproject),
 * so every .c file under it is discovered and compiled into the firmware. This
 * file defines main(). Without the guard it collides with the firmware's own
 * main() at link time, in both the STM32 and the MSP430 build. The guard makes
 * the translation unit empty for anyone who does not ask for it; only the CI
 * workflow passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "SDCard/shimmer_sd_write_buf.h"

/* The 21-byte record and 9-byte sync head of the recordings this was found in:
 * 504 Hz, gyro + wide-range accel + magnetometer, logging as a sync node. */
#define FIELD_REC_LEN  21
#define FIELD_HEAD_LEN 9

/* Comfortably more than the (NUM_SDWRBUF + 1) buffers that can be outstanding,
 * plus the largest single record. */
#define TAPE_RING      ((NUM_SDWRBUF + 2) * SD_WRITE_BUF_SIZE)

static int failures;

static void expect(const char *what, uint32_t got, uint32_t want)
{
  if (got != want)
  {
    printf("  FAIL %-52s got %lu want %lu\n", what, (unsigned long) got,
        (unsigned long) want);
    failures++;
  }
}

static void fail(const char *what)
{
  printf("  FAIL %s\n", what);
  failures++;
}

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
  /* Poison the storage so a write past a buffer's length is visible. */
  memset(&sim->w.buf[0][0], 0xA5, sizeof(sim->w.buf));
  sim->headLen = headLen;
  for (i = 0; i < FIELD_HEAD_LEN; i++)
  {
    sim->head[i] = (uint8_t) (0xE0 + i);
  }
}

/* Checked after every call that can touch the state. */
static void sim_checkInvariants(Sim *sim, const char *where)
{
  uint8_t i;
  uint16_t j;

  if (sim->w.numQueued > NUM_SDWRBUF)
  {
    printf("  FAIL %s: numQueued %u > NUM_SDWRBUF\n", where, (unsigned) sim->w.numQueued);
    failures++;
    return;
  }
  if (sim->w.sensIdx != ((sim->w.wrIdx + sim->w.numQueued) % NUM_SDWRBUF))
  {
    printf("  FAIL %s: sensIdx %u != (wrIdx %u + numQueued %u) %% %u\n", where,
        (unsigned) sim->w.sensIdx, (unsigned) sim->w.wrIdx,
        (unsigned) sim->w.numQueued, (unsigned) NUM_SDWRBUF);
    failures++;
  }
  for (i = 0; i < NUM_SDWRBUF; i++)
  {
    if (sim->w.len[i] > SD_WRITE_BUF_SIZE)
    {
      printf("  FAIL %s: buffer %u length %u exceeds the buffer\n", where,
          (unsigned) i, (unsigned) sim->w.len[i]);
      failures++;
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
        printf("  FAIL %s: buffer %u written past its high-water mark at "
               "byte %u\n",
            where, (unsigned) i, (unsigned) j);
        failures++;
        return;
      }
    }
  }
  for (i = 0; i < sizeof(sim->canary); i++)
  {
    if (sim->canary[i] != 0xA5)
    {
      printf("  FAIL %s: wrote past the end of the struct\n", where);
      failures++;
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
  sim_checkInvariants(sim, "after put");

  if (r != SDWRBUF_PUT_REFUSED && recLen > 0)
  {
    if ((sim->tapeIn - sim->tapeOut) + recLen > TAPE_RING)
    {
      fail("more accepted than can be outstanding - the module is holding on "
           "to records it should have handed over");
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

  if (len == 0)
  {
    fail("a queued buffer was empty - the stop-time flush relies on it not "
         "being");
  }

  if (sim->headLen > 0)
  {
    if (len < sim->headLen || memcmp(data, sim->head, sim->headLen) != 0)
    {
      fail("a written buffer did not start with the sync head");
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
        fail("more bytes written than were accepted");
      }
      sim->mismatches++;
      break;
    }
    if (data[i] != sim->tape[sim->tapeOut % TAPE_RING])
    {
      if (sim->mismatches == 0)
      {
        fail("a written byte is not the next byte that was accepted - a record "
             "was lost, duplicated or reordered");
      }
      sim->mismatches++;
    }
    sim->tapeOut++;
  }
  sim->blocksWritten++;

  SdWrBuf_writeDone(&sim->w);
  sim_checkInvariants(sim, "after writeDone");
  return 1;
}

/* ShimSdDataFile_writeAllBufsToSd. The iteration cap is the point: a
 * regression of the stop-time flush defect must fail the test, not hang it. */
static uint32_t sim_stop(Sim *sim)
{
  uint32_t writes = 0;
  uint32_t cap = NUM_SDWRBUF + 2;

  SdWrBuf_queueCurrent(&sim->w);
  sim_checkInvariants(sim, "after queueCurrent");

  while (SdWrBuf_numQueued(&sim->w) > 0)
  {
    if (writes >= cap)
    {
      fail("stop-time flush did not terminate - it wrote more buffers than "
           "exist");
      break;
    }
    if (!sim_taskSdWrite(sim))
    {
      fail("a buffer was queued but the write path would not take it");
      break;
    }
    writes++;
  }
  return writes;
}

/* Everything accepted has been written, in order. */
static void sim_checkNothingLost(Sim *sim, const char *what)
{
  if (sim->tapeOut != sim->tapeIn)
  {
    printf("  FAIL %s: %lu accepted bytes never reached the card\n", what,
        (unsigned long) (sim->tapeIn - sim->tapeOut));
    failures++;
  }
  expect(what, sim->mismatches, 0);
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

/* 1. The ordinary cycle: records in, one block out, nothing lost. */
static void test_normal_cycle(void)
{
  Sim sim;

  sim_init(&sim, 0);
  expect("first record starts a buffer", sim_put(&sim, FIELD_REC_LEN), SDWRBUF_PUT_STARTED_FRESH);
  expect("second record appends", sim_put(&sim, FIELD_REC_LEN), SDWRBUF_PUT_APPENDED);
  expect("nothing queued yet", SdWrBuf_numQueued(&sim.w), 0);

  sim_fillCurrent(&sim);
  expect("still nothing queued before the record that will not fit",
      SdWrBuf_numQueued(&sim.w), 0);

  /* The record that does not fit closes the buffer. Where it goes then depends
   * on whether there is another buffer to go into. */
#if NUM_SDWRBUF >= 2
  expect("the record that does not fit starts the next buffer",
      sim_put(&sim, FIELD_REC_LEN), SDWRBUF_PUT_STARTED_FRESH);
  expect("no refusals", sim.w.diag.putsRefusedFull, 0);
#else
  expect("with one buffer there is nowhere for it to go",
      sim_put(&sim, FIELD_REC_LEN), SDWRBUF_PUT_REFUSED);
  expect("and the loss is counted", sim.w.diag.putsRefusedFull, 1);
#endif
  expect("the full buffer is queued", SdWrBuf_numQueued(&sim.w), 1);

  expect("the write drains it", sim_taskSdWrite(&sim), 1);
  expect("nothing left queued", SdWrBuf_numQueued(&sim.w), 0);

  sim_stop(&sim);
  expect("everything reached the card", SdWrBuf_numQueued(&sim.w), 0);
  sim_checkNothingLost(&sim, "normal cycle");
}

/* 2. THE DEFECT. The drain hands over several records in a row without the task
 * loop running in between, so a batch can cross the 512-byte boundary. With one
 * buffer every record from the crossing on was dropped; with two the batch
 * fits, and what is offered is what gets written. */
static void test_batch_crossing_boundary(void)
{
  Sim sim;
  int i;
  int refused = 0;

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
  expect("no record in the batch was refused", (uint32_t) refused, 0);
  expect("none counted as refused", sim.w.diag.putsRefusedFull, 0);
#else
  /* What Shimmer3 shipped: the buffer fills mid-batch and the rest is gone. */
  expect("the whole batch was dropped from the crossing on", (uint32_t) refused, 6);
  expect("and every drop was counted", sim.w.diag.putsRefusedFull, (uint32_t) refused);
#endif

  sim_stop(&sim);
  sim_checkNothingLost(&sim, "batch crossing the boundary");
}

/* 3. When the card really is too slow the record is still lost - but counted,
 * and the machine recovers as soon as a buffer frees. */
static void test_refusal_is_counted_and_recovers(void)
{
  Sim sim;

  sim_init(&sim, 0);
  sim_queueEverything(&sim);
  expect("every buffer is queued", SdWrBuf_numQueued(&sim.w), NUM_SDWRBUF);
  expect("no buffer is open", SdWrBuf_bytesInCurrent(&sim.w), 0);

  {
    uint16_t refusedSoFar = sim.w.diag.putsRefusedFull;

    expect("a record offered now is refused", sim_put(&sim, FIELD_REC_LEN), SDWRBUF_PUT_REFUSED);
    expect("and counted", sim.w.diag.putsRefusedFull, refusedSoFar + 1);
    expect("so is the next", sim_put(&sim, FIELD_REC_LEN), SDWRBUF_PUT_REFUSED);
    expect("counted again", sim.w.diag.putsRefusedFull, refusedSoFar + 2);
    expect("the refusals changed nothing else", SdWrBuf_numQueued(&sim.w), NUM_SDWRBUF);

    sim_taskSdWrite(&sim);
    expect("one buffer freed", SdWrBuf_numQueued(&sim.w), NUM_SDWRBUF - 1);
    expect("and the next record goes into it", sim_put(&sim, FIELD_REC_LEN),
        SDWRBUF_PUT_STARTED_FRESH);
    expect("the counter still reads what happened", sim.w.diag.putsRefusedFull,
        refusedSoFar + 2);

    SdWrBuf_reset(&sim.w);
    expect("a reset keeps the counter for the debugger",
        sim.w.diag.putsRefusedFull, refusedSoFar + 2);
    SdWrBuf_init(&sim.w);
    expect("a new logging session clears it", sim.w.diag.putsRefusedFull, 0);
  }
}

/* 4. Both indices wrap. Long enough to lap the buffers many times, with the
 * write lagging by a different amount each cycle. */
static void test_indices_wrap(void)
{
  Sim sim;
  int cycle;

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
      fail("an index left the array");
      return;
    }
  }
  sim_stop(&sim);
  sim_checkNothingLost(&sim, "1000 cycles");
#if NUM_SDWRBUF >= 2
  expect("nothing was refused", sim.w.diag.putsRefusedFull, 0);
#else
  /* One buffer, so every cycle loses the record that crosses the boundary -
   * one sample per 512 bytes of file, for the whole recording. */
  expect("one record lost per block", sim.w.diag.putsRefusedFull, 1000);
#endif
}

/* 5. The bound must hold for any record length, not only the constant one the
 * old look-ahead silently depended on. */
static void test_variable_record_lengths(void)
{
  Sim sim;
  uint16_t recLen;
  int cycle;
  uint8_t headLen;

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
    sim_checkNothingLost(&sim, "varying record lengths");
#if NUM_SDWRBUF >= 2
    expect("nothing refused for lack of room", sim.w.diag.putsRefusedFull, 0);
#else
    /* One buffer: the record that closes a buffer has nowhere to go, whatever
     * its length. Counted, which is the point - it used to be silent. */
    expect("the boundary records were lost, and counted",
        sim.w.diag.putsRefusedFull > 0, 1);
#endif
    expect("nothing refused as oversize", sim.w.diag.putsRefusedOversize, 0);

    /* A zero-length record changes nothing. */
    expect("zero-length record is a no-op", sim_put(&sim, 0), SDWRBUF_PUT_APPENDED);
    expect("no buffer opened for it", SdWrBuf_bytesInCurrent(&sim.w), 0);

    /* One that could not fit an empty buffer fails closed. */
    expect("oversize record is refused",
        sim_put(&sim, (uint16_t) (SD_WRITE_BUF_SIZE - headLen + 1)), SDWRBUF_PUT_REFUSED);
    expect("and counted separately", sim.w.diag.putsRefusedOversize, 1);
    expect("and nothing was written", SdWrBuf_bytesInCurrent(&sim.w), 0);
    expect("the largest record that does fit is accepted",
        sim_put(&sim, (uint16_t) (SD_WRITE_BUF_SIZE - headLen)), SDWRBUF_PUT_STARTED_FRESH);
    sim_stop(&sim);
    sim_checkNothingLost(&sim, "after the oversize record");
  }
}

/* 6. THE HANG. A stop is a higher-priority task than the SD write, so it can
 * arrive with every buffer queued. The flush must still finish, and must write
 * each buffer exactly once. */
static void test_stop_flush_terminates(void)
{
  Sim sim;
  uint8_t i;
  uint32_t writes;

  /* Every buffer queued, nothing open. */
  sim_init(&sim, 0);
  sim_queueEverything(&sim);
  expect("every buffer queued before the stop", SdWrBuf_numQueued(&sim.w), NUM_SDWRBUF);
  writes = sim_stop(&sim);
  expect("each queued buffer written exactly once", writes, NUM_SDWRBUF);
  expect("nothing left queued", SdWrBuf_numQueued(&sim.w), 0);
  sim_checkNothingLost(&sim, "stop with every buffer queued");

  /* Every buffer queued but one, which is partly filled. */
  sim_init(&sim, 0);
  for (i = 0; i + 1 < NUM_SDWRBUF; i++)
  {
    sim_fillCurrent(&sim);
    sim_put(&sim, FIELD_REC_LEN);
  }
  sim_put(&sim, FIELD_REC_LEN);
  expect("one buffer is open with data in it", SdWrBuf_bytesInCurrent(&sim.w) > 0, 1);
  writes = sim_stop(&sim);
  expect("the partial buffer is written too", writes, NUM_SDWRBUF);
  expect("nothing left queued", SdWrBuf_numQueued(&sim.w), 0);
  sim_checkNothingLost(&sim, "stop with a partial buffer");

  /* Nothing buffered at all. */
  sim_init(&sim, 0);
  writes = sim_stop(&sim);
  expect("a stop with nothing buffered writes nothing", writes, 0);
}

/* 7. The sync offset leads every buffer and is consumed exactly once per
 * buffer, which is what lets the caller reset it on STARTED_FRESH. */
static void test_sync_head(void)
{
  Sim sim;
  int cycle;
  uint32_t fresh = 0;

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

  expect("one head per block written", sim.headsSeen, sim.blocksWritten);
  expect("one fresh start per block", fresh, sim.blocksWritten);
  sim_checkNothingLost(&sim, "with the sync head");

  /* With no head the whole buffer is record data, which the byte-for-byte
   * comparison in sim_taskSdWrite covers. */
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
  expect("no heads when none was given", sim.headsSeen, 0);
  sim_checkNothingLost(&sim, "without the sync head");
}

/* 8. A block still holds the same records it always did. Host parsers slice
 * files on this arithmetic, so moving the queue decision must not move it. */
static void test_block_layout_unchanged(void)
{
  Sim sim;
  uint16_t recLen;
  uint8_t headLen;

  for (headLen = 0; headLen <= FIELD_HEAD_LEN; headLen = (uint8_t) (headLen + FIELD_HEAD_LEN))
  {
    for (recLen = 12; recLen <= 64; recLen++)
    {
      uint16_t perBlock = (uint16_t) ((SD_WRITE_BUF_SIZE - headLen) / recLen);
      uint16_t i;

      sim_init(&sim, headLen);
      for (i = 0; i < perBlock; i++)
      {
        if (SdWrBuf_numQueued(&sim.w) != 0)
        {
          printf("  FAIL block closed after %u records, expected %u "
                 "(record %u, head %u)\n",
              (unsigned) i, (unsigned) perBlock, (unsigned) recLen, (unsigned) headLen);
          failures++;
          break;
        }
        sim_put(&sim, recLen);
      }
      sim_put(&sim, recLen);

      if (SdWrBuf_numQueued(&sim.w) != 1)
      {
        printf("  FAIL block did not close after %u records (record %u, "
               "head %u)\n",
            (unsigned) perBlock, (unsigned) recLen, (unsigned) headLen);
        failures++;
      }
      else if (sim.w.len[sim.w.wrIdx] != (uint16_t) (headLen + perBlock * recLen))
      {
        printf("  FAIL block is %u bytes, expected %u (record %u, head %u)\n",
            (unsigned) sim.w.len[sim.w.wrIdx], (unsigned) (headLen + perBlock * recLen),
            (unsigned) recLen, (unsigned) headLen);
        failures++;
      }
    }
  }
}

int main(void)
{
  printf("SD write buffers, NUM_SDWRBUF = %u\n", (unsigned) NUM_SDWRBUF);

  test_normal_cycle();
  test_batch_crossing_boundary();
  test_refusal_is_counted_and_recovers();
  test_indices_wrap();
  test_variable_record_lengths();
  test_stop_flush_terminates();
  test_sync_head();
  test_block_layout_unchanged();

  if (failures)
  {
    printf("\nSD write buffers: %d FAILURE(S)\n", failures);
    return 1;
  }
  printf("\nSD write buffers: all checks passed\n");
  return 0;
}

#endif /* SHIMMER_HOST_TEST */
