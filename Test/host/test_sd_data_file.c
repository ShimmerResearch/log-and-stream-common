/*
 * Host-side test for the Shimmer3R SD writer, ShimSdDataFile_writeToCard() in
 * SDCard/shimmer_sd_data_file.c, built against the stub platform with a fake
 * FatFs card defined below. Run by Test/host/Makefile, Shimmer3R only: the
 * batching, the alignment offset, the split guard and the short-write
 * recovery under test are all Shimmer3R code.
 *
 * test_sd_write_buf.c covers the buffer state machine on its own. This covers
 * what the writer does with it: which buffers go into one f_write, where in
 * memory that write starts, what is released after a short or failed write,
 * and when the hourly split happens. None of that is reachable on hardware on
 * demand - a card that writes 300 of 4000 bytes and returns FR_OK is not
 * something a bench can ask for.
 *
 * HOW LOSS IS DETECTED. Every record carries its sequence number, and is
 * written in through ShimSdDataFile_writeToBuff() behind the same
 * back-pressure test the drain uses (ShimSdDataFile_canAcceptRecord), with a
 * FIFO standing in for the packet ring. At the end the files the fake card
 * holds are read back, header stripped, and the records must be exactly
 * 0, 1, 2, ... with nothing missing, repeated or reordered, and no record
 * split across two files.
 *
 * NOT COVERED. The writer refuses to judge the split from a write shorter than
 * one record (the retried tail of a short write), because the timestamp it
 * would read lies before the buffer. Without that check the outcome depends on
 * stale bytes in memory, so no assertion here can pin it down.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "host_stubs.h"
#include "host_test.h"
#include "log_and_stream_includes.h"

#if !defined(SHIMMER3R)
#error "test_sd_data_file covers the Shimmer3R writer; build it with -DSHIMMER3R"
#endif

/* An odd record length, so the file size - and with it the alignment offset
 * the writer applies - takes every value mod 4. */
#define REC_LEN          23U
#define TICKS_PER_SAMPLE 32U /* 1024 Hz */
/* The packet ring in front of the SD buffers (DATA_BUF_QTY_IN_USE on
 * Shimmer3R): about 1 s at 1024 Hz. */
#define RING_RECS        1022U
#define RECS_PER_BUF     (SD_WRITE_BUF_SIZE / REC_LEN)
#define BATCH_MAX_BYTES  (16U * SD_WRITE_BUF_SIZE)

#define CARD_MAX_FILES   4
#define CARD_FILE_CAP    (8UL * 1024UL * 1024UL)
#define MAX_WRITES       200000U
/* Far more data writes than any case makes. Past it the writer is looping
 * without releasing buffers, which on the device hangs the stop: fail rather
 * than hang the suite. */
#define RUNAWAY_WRITES   2000000U

/* ---- the fake card -------------------------------------------------------- */

typedef struct
{
  uint8_t *data;
  uint32_t len;
  uint32_t headLen;
} CardFile;

typedef struct
{
  uint8_t file;
  uint32_t start; /* byte offset in that file */
  uint32_t len;   /* bytes that reached it */
  uint8_t batch;
} CardWrite;

static struct
{
  CardFile files[CARD_MAX_FILES];
  int nFiles;
  int cur;
  /* Behaviour, set per test. 1 in shortEvery data writes stores only part of
   * what it was given and returns FR_OK; 1 in failEvery stores part and
   * returns FR_DISK_ERR. Both store at least one byte unless zeroProgressOk.
   * dead: every data write stores nothing and fails. */
  uint32_t shortEvery;
  uint32_t failEvery;
  uint8_t zeroProgressOk;
  uint8_t dead;
  /* Once: the first single-buffer write whose last record is at or after this
   * timestamp stores only half its bytes. 0 is off. Lands a short write on the
   * write that would otherwise trigger the hourly split. */
  uint32_t shortOnceAtTs;
  uint32_t recBaseTs; /* record 0's timestamp, to decode the trigger */
  /* What it saw. */
  uint32_t dataWrites;
  uint32_t batchWrites;
  uint32_t misalignedBatches;
  uint32_t maxWrite;
  uint32_t shortWrites;
  uint32_t failedWrites;
  CardWrite writes[MAX_WRITES];
  uint32_t nWrites;
} card;

/* Defined in shimmer_sd_data_file.c and declared in no header. The card reads
 * its address range to tell a single-buffer write from a batched one. */
extern SdWriteBuf sdWrBuf;

static uint32_t assertsFailed;
static uint32_t rng = 1;

static uint32_t rnd(void)
{
  rng = rng * 1103515245UL + 12345UL;
  return (rng >> 8) & 0xFFFFFFUL;
}

void assert_failed(uint8_t *file, uint32_t line)
{
  (void) file;
  (void) line;
  assertsFailed++;
}

static void card_reset(void)
{
  int i;
  for (i = 0; i < CARD_MAX_FILES; i++)
  {
    free(card.files[i].data);
  }
  memset(&card, 0, sizeof(card));
  card.cur = -1;
  assertsFailed = 0;
}

FRESULT f_open(FIL *fp, const TCHAR *path, BYTE mode)
{
  (void) path;
  (void) mode;
  if (card.nFiles >= CARD_MAX_FILES)
  {
    return FR_TOO_MANY_OPEN_FILES;
  }
  card.cur = card.nFiles++;
  card.files[card.cur].data = malloc(CARD_FILE_CAP);
  card.files[card.cur].len = 0;
  card.files[card.cur].headLen = 0;
  memset(fp, 0, sizeof(*fp));
  return (card.files[card.cur].data != NULL) ? FR_OK : FR_NOT_ENOUGH_CORE;
}

FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw)
{
  CardFile *f;
  const uint8_t *src = (const uint8_t *) buff;
  /* Addresses, compared as integers: ordering pointers into different
   * objects is undefined in C. */
  const uintptr_t bufsLo = (uintptr_t) &sdWrBuf.buf[0][0];
  const uintptr_t bufsHi = bufsLo + sizeof(sdWrBuf.buf);
  uint32_t n = btw;
  FRESULT res = FR_OK;
  uint8_t isBatch;
  uint8_t forcedShort = 0;

  *bw = 0;
  if (card.cur < 0)
  {
    return FR_DISK_ERR;
  }
  f = &card.files[card.cur];
  if (f->len + btw > CARD_FILE_CAP)
  {
    return FR_DISK_ERR;
  }

  /* The file header, written once straight after f_open. */
  if ((f->len == 0) && (btw == SD_HEAD_SIZE))
  {
    memcpy(f->data, src, btw);
    f->len = btw;
    f->headLen = btw;
    fp->obj.objsize = btw;
    fp->fptr = btw;
    *bw = btw;
    return FR_OK;
  }

  /* A batched write comes from the writer's own staging buffer; a single
   * buffer is written straight out of sdWrBuf. */
  isBatch = ((uintptr_t) src < bufsLo) || ((uintptr_t) src >= bufsHi);
  card.dataWrites++;
  if (card.dataWrites > RUNAWAY_WRITES)
  {
    printf("  FAIL the writer kept writing without releasing buffers\n");
    exit(2);
  }
  if (isBatch)
  {
    card.batchWrites++;
    /* The data must start at an address congruent to the file position mod 4,
     * so every sector boundary of the file lands word-aligned. */
    if ((((uintptr_t) src - (uintptr_t) fp->obj.objsize) & 3U) != 0U)
    {
      card.misalignedBatches++;
    }
  }
  if (btw > card.maxWrite)
  {
    card.maxWrite = btw;
  }

  if ((card.shortOnceAtTs != 0U) && !isBatch && (btw >= REC_LEN))
  {
    const uint8_t *last = &src[btw - REC_LEN];
    uint32_t seq = (uint32_t) last[3] | ((uint32_t) last[4] << 8)
        | ((uint32_t) last[5] << 16) | ((uint32_t) last[6] << 24);
    forcedShort = (card.recBaseTs + seq * TICKS_PER_SAMPLE >= card.shortOnceAtTs);
  }

  if (forcedShort)
  {
    card.shortOnceAtTs = 0;
    /* Mid-record, so a split taken now would cut a record in two. */
    n = (btw / 2U) + (REC_LEN / 2U);
    card.shortWrites++;
  }
  else if (card.dead)
  {
    n = 0;
    res = FR_DISK_ERR;
  }
  else if ((card.failEvery != 0U) && ((rnd() % card.failEvery) == 0U) && (btw > 1U))
  {
    n = card.zeroProgressOk ? (rnd() % btw) : (1U + (rnd() % (btw - 1U)));
    res = FR_DISK_ERR;
    card.failedWrites++;
  }
  else if ((card.shortEvery != 0U) && ((rnd() % card.shortEvery) == 0U) && (btw > 1U))
  {
    n = card.zeroProgressOk ? (rnd() % btw) : (1U + (rnd() % (btw - 1U)));
    card.shortWrites++;
  }

  if (card.nWrites < MAX_WRITES)
  {
    CardWrite *w = &card.writes[card.nWrites++];
    w->file = (uint8_t) card.cur;
    w->start = f->len;
    w->len = n;
    w->batch = isBatch;
  }
  memcpy(&f->data[f->len], src, n);
  f->len += n;
  fp->obj.objsize += n;
  fp->fptr += n;
  *bw = n;
  return res;
}

FRESULT f_close(FIL *fp)
{
  (void) fp;
  return FR_OK;
}

FRESULT f_read(FIL *fp, void *buff, UINT btr, UINT *br)
{
  (void) fp;
  (void) buff;
  (void) btr;
  *br = 0;
  return FR_DISK_ERR;
}

FRESULT f_lseek(FIL *fp, FSIZE_t ofs)
{
  fp->fptr = ofs;
  return FR_OK;
}

FRESULT f_sync(FIL *fp)
{
  (void) fp;
  return FR_OK;
}

FRESULT f_opendir(DIR *dp, const TCHAR *path)
{
  (void) path;
  memset(dp, 0, sizeof(*dp));
  return FR_OK;
}

FRESULT f_closedir(DIR *dp)
{
  (void) dp;
  return FR_OK;
}

/* An empty experiment directory: the first entry ends the listing. */
FRESULT f_readdir(DIR *dp, FILINFO *fno)
{
  (void) dp;
  memset(fno, 0, sizeof(*fno));
  return FR_OK;
}

FRESULT f_mkdir(const TCHAR *path)
{
  (void) path;
  return FR_OK;
}

FRESULT f_stat(const TCHAR *path, FILINFO *fno)
{
  (void) path;
  memset(fno, 0, sizeof(*fno));
  return FR_OK;
}

/* ---- the rest of the platform the writer touches -------------------------- */

SENSINGTypeDef sensing;

void Board_setSdPower(uint8_t state)
{
  shimmerStatus.sdPowerOn = state;
}

static uint32_t tasksSet;

uint8_t ShimTask_NORM_set(TaskId_t task_id)
{
  tasksSet |= (uint32_t) task_id;
  return 1;
}

char *ShimConfig_shimmerNameParseToTxtAndPtrGet(void)
{
  return "host";
}

char *ShimConfig_expIdParseToTxtAndPtrGet(void)
{
  return "test";
}

char *ShimConfig_configTimeParseToTxtAndPtrGet(void)
{
  return "0";
}

void ShimSd_findError(uint8_t err, uint8_t *name)
{
  (void) err;
  (void) name;
}

void ShimSdHead_config2SdHead(void)
{
}

uint8_t ShimSdHead_sdHeadTextGet(uint8_t *buf, uint16_t offset, uint16_t length)
{
  (void) offset;
  memset(buf, 0, length);
  return 1;
}

/* The event file follows the data file's lifecycle; it has its own suite */
void ShimEvt_onSessionStart(uint64_t fileOpenTs)
{
  (void) fileOpenTs;
}

void ShimEvt_onFileSplit(uint64_t fileOpenTs)
{
  (void) fileOpenTs;
}

void ShimEvt_onSyncTick(void)
{
}

void ShimEvt_onSessionStop(void)
{
}

/* With SD sync on, every buffer starts with a 9-byte head: the sync offset the
 * writer takes from ShimSdSync_myTimeDiffPtrGet() and then resets. Each head
 * here carries its own number, so read-back can tell them apart. */
static uint8_t syncHead[SYNC_PACKET_PAYLOAD_SIZE];
static uint32_t syncHeadsIssued;

static void head_make(uint32_t n, uint8_t *head)
{
  uint8_t i;

  head[0] = 0xC3;
  for (i = 0; i < 4U; i++)
  {
    head[1U + i] = (uint8_t) (n >> (8U * i));
    head[5U + i] = (uint8_t) ~(n >> (8U * i));
  }
}

uint8_t *ShimSdSync_myTimeDiffPtrGet(void)
{
  return syncHead;
}

void ShimSdSync_resetMyTimeDiff(void)
{
  head_make(++syncHeadsIssued, syncHead);
}

/* ---- a logging session ---------------------------------------------------- */

typedef struct
{
  uint32_t baseTs;   /* timestamp of record 0 */
  uint32_t produced; /* records sampled */
  uint32_t fifo[RING_RECS];
  uint32_t head;
  uint32_t count;
  uint32_t ringOverflows;
  /* A stall forced to start at this record, for stallLen records. */
  uint32_t stallAt;
  uint32_t stallLen;
  /* SD sync on: every buffer starts with a sync head. */
  uint8_t sync;
} Session;

static uint32_t rec_ts(const Session *s, uint32_t seq)
{
  return s->baseTs + seq * TICKS_PER_SAMPLE;
}

static void rec_make(const Session *s, uint32_t seq, uint8_t *rec)
{
  uint32_t ts = rec_ts(s, seq);
  uint8_t i;

  rec[0] = (uint8_t) ts;
  rec[1] = (uint8_t) (ts >> 8);
  rec[2] = (uint8_t) (ts >> 16);
  rec[3] = (uint8_t) seq;
  rec[4] = (uint8_t) (seq >> 8);
  rec[5] = (uint8_t) (seq >> 16);
  rec[6] = (uint8_t) (seq >> 24);
  for (i = 7; i < REC_LEN; i++)
  {
    rec[i] = (uint8_t) (seq * 7U + i);
  }
}

static void set_clock(uint32_t ts)
{
  sensing.latestTs = ts;
  /* RTC_get64() reads the stub tick in ms. */
  hostStub_setTick((uint32_t) (((uint64_t) ts * 1000U) / 32768U));
}

/* Start a session with the file created at fileCrTs. */
static void session_start(Session *s, uint32_t baseTs, uint32_t fileCrTs)
{
  memset(s, 0, sizeof(*s));
  s->baseTs = baseTs;
  s->stallAt = UINT32_MAX;
  card_reset();
  card.recBaseTs = baseTs;
  hostStub_reset();
  memset(&sensing, 0, sizeof(sensing));
  sensing.dataLen = (uint8_t) (REC_LEN + 1U);
  shimmerStatus.sdSyncEnabled = 0;
  syncHeadsIssued = 0;
  head_make(0, syncHead);

  ShimSdDataFile_init();
  set_clock(fileCrTs);
  ShimSdDataFile_fileInit();
}

/* Move what the "ring" holds into the SD buffers, as ShimSens_saveData does:
 * stop at the first record the buffers would refuse. */
static void drain(Session *s)
{
  uint8_t rec[REC_LEN];

  while ((s->count > 0U) && ShimSdDataFile_canAcceptRecord(REC_LEN))
  {
    rec_make(s, s->fifo[s->head], rec);
    ShimSdDataFile_writeToBuff(rec, REC_LEN);
    s->head = (s->head + 1U) % RING_RECS;
    s->count--;
  }
}

/* One sample: into the ring, then drained as far as the buffers allow. */
static void sample(Session *s)
{
  uint32_t seq = s->produced++;

  set_clock(rec_ts(s, seq));
  if (s->count >= RING_RECS)
  {
    s->ringOverflows++;
  }
  else
  {
    s->fifo[(s->head + s->count) % RING_RECS] = seq;
    s->count++;
  }
  drain(s);
}

/* Sample n records against a card that stalls now and then: between stalls it
 * takes one write every other sample, which keeps up. */
static void run(Session *s, uint32_t n)
{
  uint32_t i;
  uint32_t stallLeft = 0;

  for (i = 0; i < n; i++)
  {
    if (s->produced == s->stallAt)
    {
      stallLeft = s->stallLen;
    }
    sample(s);
    if (stallLeft > 0U)
    {
      stallLeft--;
    }
    else if ((rnd() % 1500U) == 0U)
    {
      stallLeft = 100U + (rnd() % 600U); /* 0.1 - 0.7 s busy */
    }
    else if (((i & 1U) == 0U) && (ShimSdDataFile_getNumberOfFullBuffers() > 0U))
    {
      ShimSdDataFile_writeToCard();
    }
  }
}

/* The stop path: everything sampled goes to the card, then the open buffer. */
static void session_stop(Session *s)
{
  uint32_t guard = 0;

  while ((s->count > 0U) && (guard++ < 100000U))
  {
    drain(s);
    if (s->count > 0U)
    {
      ShimSdDataFile_writeToCard();
    }
  }
  ShimSdDataFile_writeAllBufsToSd();
}

/* ---- reading the card back ------------------------------------------------ */

/* Records in a full buffer that starts with a sync head. */
#define RECS_PER_SYNC_BUF \
  ((SD_WRITE_BUF_SIZE - SYNC_PACKET_PAYLOAD_SIZE) / REC_LEN)

typedef struct
{
  uint32_t heads;     /* sync heads read back, in order */
  uint32_t records;   /* records read back, all files */
  uint32_t bad;       /* records out of sequence or corrupt */
  uint32_t straddles; /* files whose data is not whole records */
  uint32_t firstSeq[CARD_MAX_FILES];
  uint32_t nRecs[CARD_MAX_FILES];
} ReadBack;

static void read_back(const Session *s, ReadBack *rb)
{
  int f;
  uint32_t expect = 0;
  uint8_t want[REC_LEN];

  memset(rb, 0, sizeof(*rb));
  for (f = 0; f < card.nFiles; f++)
  {
    const CardFile *cf = &card.files[f];
    uint32_t dataLen = cf->len - cf->headLen;
    uint32_t off;

    rb->firstSeq[f] = expect;
    if (s->sync)
    {
      /* A head, then RECS_PER_SYNC_BUF records - fewer only in the buffer the
       * stop flush closed, which is the last thing in the last file. */
      uint8_t wantHead[SYNC_PACKET_PAYLOAD_SIZE];

      off = cf->headLen;
      while (off < cf->len)
      {
        uint32_t left;
        uint32_t n;
        uint32_t r;

        head_make(rb->heads, wantHead);
        if ((off + SYNC_PACKET_PAYLOAD_SIZE > cf->len)
            || (memcmp(&cf->data[off], wantHead, SYNC_PACKET_PAYLOAD_SIZE) != 0))
        {
          if (rb->bad == 0U)
          {
            printf("         file %d offset %lu: expected sync head %lu\n", f,
                (unsigned long) off, (unsigned long) rb->heads);
          }
          rb->bad++;
          break;
        }
        rb->heads++;
        off += SYNC_PACKET_PAYLOAD_SIZE;
        left = cf->len - off;
        n = (left >= RECS_PER_SYNC_BUF * REC_LEN) ? RECS_PER_SYNC_BUF : (left / REC_LEN);
        if ((n < RECS_PER_SYNC_BUF) && ((left % REC_LEN) != 0U))
        {
          rb->straddles++;
        }
        for (r = 0; r < n; r++, off += REC_LEN)
        {
          rec_make(s, expect, want);
          if (memcmp(&cf->data[off], want, REC_LEN) != 0)
          {
            if (rb->bad == 0U)
            {
              printf("         file %d offset %lu: expected record %lu\n", f,
                  (unsigned long) off, (unsigned long) expect);
            }
            rb->bad++;
          }
          expect++;
          rb->nRecs[f]++;
        }
        if (n < RECS_PER_SYNC_BUF)
        {
          break;
        }
      }
      continue;
    }
    if ((dataLen % REC_LEN) != 0U)
    {
      rb->straddles++;
    }
    for (off = cf->headLen; off + REC_LEN <= cf->len; off += REC_LEN)
    {
      rec_make(s, expect, want);
      if (memcmp(&cf->data[off], want, REC_LEN) != 0)
      {
        if (rb->bad == 0U)
        {
          printf("         file %d offset %lu: expected record %lu\n", f,
              (unsigned long) off, (unsigned long) expect);
        }
        rb->bad++;
      }
      expect++;
      rb->nRecs[f]++;
    }
  }
  rb->records = expect;
}

/* The newest record any batched write put in file f, as a timestamp. */
static uint32_t newest_batched_ts(const Session *s, int f, uint8_t *anyOut)
{
  uint32_t i;
  uint32_t newest = 0;
  const CardFile *cf = &card.files[f];

  *anyOut = 0;
  for (i = 0; i < card.nWrites; i++)
  {
    const CardWrite *w = &card.writes[i];
    uint32_t lastByte;
    uint32_t seq;

    if ((w->file != f) || !w->batch || (w->len == 0U))
    {
      continue;
    }
    lastByte = w->start + w->len - 1U - cf->headLen;
    seq = lastByte / REC_LEN;
    *anyOut = 1;
    if (rec_ts(s, seq) > newest)
    {
      newest = rec_ts(s, seq);
    }
  }
  return newest;
}

/* ---- the cases ------------------------------------------------------------ */

static void expect_lossless(const Session *s, uint32_t files)
{
  ReadBack rb;

  read_back(s, &rb);
  expectU("the ring never overflowed", s->ringOverflows, 0);
  expectU("every record sampled reached the card", rb.records, s->produced);
  expectU("in order, none missing, repeated or corrupt", rb.bad, 0);
  expectU("no record is split across two files", rb.straddles, 0);
  expectU("files written", (uint64_t) card.nFiles, files);
  expectU("nothing left queued", ShimSdDataFile_getNumberOfFullBuffers(), 0);
}

static void test_batched_drain_is_lossless(void)
{
  Session s;

  testCase("stalls are written out in batches, losslessly");
  rng = 1;
  session_start(&s, 0, 0);
  run(&s, 120000U);
  session_stop(&s);
  expect_lossless(&s, 1);
  expectTrue("the backlog after a stall went out batched", card.batchWrites > 0U);
  expectTrue("no write is over 16 buffers", card.maxWrite <= BATCH_MAX_BYTES);
  expectU("every batch starts word-aligned to the file position", card.misalignedBatches, 0);
  expectU("no FatFs call failed", assertsFailed, 0);
  expectU("the file is good", shimmerStatus.sdBadFile, 0);
}

static void test_short_and_failed_writes_keep_the_file_whole(void)
{
  Session s;

  testCase("short and part-failed writes lose nothing");
  rng = 7;
  session_start(&s, 0, 0);
  card.shortEvery = 6;
  card.failEvery = 9;
  run(&s, 120000U);
  card.shortEvery = 0;
  card.failEvery = 0;
  session_stop(&s);
  expectTrue("short writes happened", card.shortWrites > 10U);
  expectTrue("failed writes happened", card.failedWrites > 10U);
  expectTrue("some of them batched", card.batchWrites > 0U);
  expect_lossless(&s, 1);
  expectU("every batch starts word-aligned to the file position", card.misalignedBatches, 0);
  expectU("the file is good again once writes succeed", shimmerStatus.sdBadFile, 0);
}

static void test_dead_card_flush_ends(void)
{
  Session s;
  uint32_t i;
  uint8_t queued;

  testCase("a stop flush on a dead card ends");
  rng = 3;
  session_start(&s, 0, 0);
  /* A stall long enough to queue every buffer. */
  for (i = 0; i < 2000U; i++)
  {
    sample(&s);
  }
  /* Back-pressure stops one short: the record that would close the last open
   * buffer is held back, so the stop flush is what queues it. */
  expectU("every buffer but the open one is queued",
      ShimSdDataFile_getNumberOfFullBuffers(), NUM_SDWRBUF - 1U);
  expectTrue("and that one has data", ShimSdDataFile_getBytesInCurrentSensingBuffer() > 0U);
  queued = NUM_SDWRBUF;
  card.dead = 1;
  ShimSdDataFile_writeAllBufsToSd();
  expectU("and all were released", ShimSdDataFile_getNumberOfFullBuffers(), 0);
  expectTrue("one write per buffer at most", card.dataWrites <= (uint32_t) queued + 1U);
  expectU("the file is marked bad", shimmerStatus.sdBadFile, 1);
}

static void test_hourly_split(uint32_t shortEvery, uint32_t failEvery, uint8_t shortOnTheHour)
{
  Session s;
  ReadBack rb;
  uint32_t i;
  uint32_t lateInFirst = 0;
  uint32_t earlyInSecond = 0;
  uint8_t anyBatch;
  const uint32_t split = (uint32_t) BIN_FILE_SPLIT_TIME_TICKS;

  /* File created at 0; sampling picks up 30 s before the hour and runs 60 s,
   * through stalls, past it. */
  session_start(&s, split - 30U * 32768U, 0);
  /* And a stall that straddles it: 0.3 s before the hour to 0.4 s after, so
   * the backlog written when it clears holds records from both sides. That is
   * when a batch could carry post-hour records into the old file, and when
   * splitting on the sample clock would leave pre-hour buffers queued for the
   * new one. */
  s.stallAt = 30U * 1024U - 300U;
  s.stallLen = 700U;
  card.shortEvery = shortEvery;
  card.failEvery = failEvery;
  card.shortOnceAtTs = shortOnTheHour ? split : 0U;
  run(&s, 60U * 1024U);
  if (shortOnTheHour)
  {
    expectU("the write on the hour came back short", card.shortOnceAtTs, 0);
  }
  card.shortEvery = 0;
  card.failEvery = 0;
  session_stop(&s);
  expect_lossless(&s, 2);

  read_back(&s, &rb);
  for (i = 0; i < rb.nRecs[0]; i++)
  {
    if (rec_ts(&s, rb.firstSeq[0] + i) >= split)
    {
      lateInFirst++;
    }
  }
  for (i = 0; i < rb.nRecs[1]; i++)
  {
    if (rec_ts(&s, rb.firstSeq[1] + i) < split)
    {
      earlyInSecond++;
    }
  }
  expectU("the second file holds nothing sampled before the hour", earlyInSecond, 0);
  expectTrue("the first overshoots the hour by at most one buffer", lateInFirst <= RECS_PER_BUF);
  (void) newest_batched_ts(&s, 0, &anyBatch);
  expectTrue("the first file was batched before the guard", anyBatch);
  expectTrue("no batch put a record from past the hour into it",
      !anyBatch || (newest_batched_ts(&s, 0, &anyBatch) < split));
  expectTrue("the second file starts with its header", card.files[1].headLen == SD_HEAD_SIZE);
}

static void test_split_on_the_hour(void)
{
  testCase("the hourly split lands on the hour");
  rng = 11;
  test_hourly_split(0, 0, 0);
}

/* The write that would split comes back short. Its unwritten tail belongs to
 * the old file, so the split must wait for the next whole write. */
static void test_split_waits_out_a_short_write(void)
{
  testCase("a short write on the hour delays the split");
  rng = 19;
  test_hourly_split(0, 0, 1);
}

static void test_split_through_short_writes(void)
{
  testCase("the hourly split lands on the hour through short writes");
  rng = 13;
  test_hourly_split(5, 8, 0);
  expectTrue("short writes happened", card.shortWrites > 5U);
}

/* The regression c02dbda fixed: the first records of a file are the backlog
 * sampled while it was being created, older than sdFileCrTs. An unsigned age
 * wrapped and split the file on its first write. */
/* The file format with SD sync on: every buffer starts with its sync head, so
 * batching and short-write recovery must carry the heads through as well as
 * the records - each head exactly once, in front of the records it led. */
static void test_sync_heads(uint32_t shortEvery, uint32_t failEvery)
{
  Session s;
  ReadBack rb;

  session_start(&s, 0, 0);
  s.sync = 1;
  shimmerStatus.sdSyncEnabled = 1;
  card.shortEvery = shortEvery;
  card.failEvery = failEvery;
  run(&s, 120000U);
  card.shortEvery = 0;
  card.failEvery = 0;
  session_stop(&s);
  expect_lossless(&s, 1);
  read_back(&s, &rb);
  expectU("every buffer's sync head reached the card, in order", rb.heads, syncHeadsIssued);
  expectTrue("the backlog after a stall went out batched", card.batchWrites > 0U);
  expectU("every batch starts word-aligned to the file position", card.misalignedBatches, 0);
}

static void test_sync_heads_survive_batching(void)
{
  testCase("sync heads survive batched writes");
  rng = 29;
  test_sync_heads(0, 0);
}

static void test_sync_heads_survive_short_writes(void)
{
  testCase("sync heads survive short and part-failed batched writes");
  rng = 31;
  test_sync_heads(6, 9);
  expectTrue("short writes happened", card.shortWrites > 10U);
  expectTrue("failed writes happened", card.failedWrites > 10U);
}

static void test_backlog_older_than_file_does_not_split(void)
{
  Session s;

  testCase("records older than the file do not split it");
  rng = 17;
  /* File created 10 s into sampling, which started at 0. */
  session_start(&s, 0, 10U * 32768U);
  run(&s, 30U * 1024U);
  session_stop(&s);
  expect_lossless(&s, 1);
}

/* ShimSens_saveData() stops draining the ring when every buffer is queued and
 * does not re-queue itself; the writer restarts it each time it frees one. */
static void test_freed_buffer_resumes_the_drain(void)
{
  Session s;
  uint32_t i;

  testCase("a freed buffer resumes a paused drain");
  rng = 23;
  session_start(&s, 0, 0);
  for (i = 0; i < 2000U; i++)
  {
    sample(&s);
  }
  /* Packets waiting in the real ring. */
  sensing.ring.wrIdx = (uint16_t) (sensing.ring.rdIdx + 5U);
  tasksSet = 0;
  ShimSdDataFile_writeToCard();
  expectTrue("the drain is re-queued", (tasksSet & TASK_SAVEDATA) != 0U);
  expectTrue("and so is the next write", (tasksSet & TASK_SDWRITE) != 0U);

  sensing.ring.wrIdx = sensing.ring.rdIdx;
  tasksSet = 0;
  ShimSdDataFile_writeToCard();
  expectTrue("not when the ring is empty", (tasksSet & TASK_SAVEDATA) == 0U);
  session_stop(&s);
}

int main(void)
{
  printf("SD writer (Shimmer3R), NUM_SDWRBUF = %u, record %u B\n\n",
      (unsigned) NUM_SDWRBUF, (unsigned) REC_LEN);
  hostTestSilenceUnused();

  test_batched_drain_is_lossless();
  test_short_and_failed_writes_keep_the_file_whole();
  test_dead_card_flush_ends();
  test_split_on_the_hour();
  test_split_through_short_writes();
  test_split_waits_out_a_short_write();
  test_backlog_older_than_file_does_not_split();
  test_sync_heads_survive_batching();
  test_sync_heads_survive_short_writes();
  test_freed_buffer_resumes_the_drain();

  card_reset();
  return hostTestReport("sd_data_file");
}

#endif /* SHIMMER_HOST_TEST */
