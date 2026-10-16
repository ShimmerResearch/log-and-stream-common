/*
 * Host-side test for the length and offset bounds in
 * Calibration/shimmer_calibration.c.
 *
 * Built against the stub platform in Test/host/stubs - see stubs/README.md.
 * Run by .github/workflows/host-tests.yml.
 *
 * WHAT THIS IS FOR. The calibration dump is a 1024-byte blob that arrives from
 * outside the firmware: a Bluetooth host writes it in 128-byte chunks
 * (SET_CALIB_DUMP), the SD card's calibration file is read back into it at
 * boot, and every record inside it carries its own length byte. Each of those
 * lengths used to size a memcpy() or memset() with nothing to check it against
 * the buffer behind it. A host, or a corrupt card, could write past the
 * calibration RAM or the caller's sc_t, or hang file2Ram on a dead card.
 *
 * Every case below drives one of those bounds from both sides: the largest
 * input that is still accepted, and the smallest that is not. Each one fails
 * with its fix taken out.
 *
 * HOW AN OVERFLOW IS SEEN WITHOUT CAUSING ONE. A broken build would write past
 * an object, and a test of that is only as good as what it can observe safely.
 * So:
 *   - a caller's sc_t sits in guarded_sc_t, with a guard band straight after
 *     it, and the test checks the band;
 *   - the fake SD card below never copies past the calibration RAM. It counts
 *     the bytes a read asked for there instead, and it ends a read loop that
 *     would otherwise never stop.
 *
 * Built TWICE, once per platform. The RAM is 1024 bytes on both, but it is
 * sized from a different header on each (the literal on Shimmer3,
 * INFOMEM_CALIB_SIZE on Shimmer3R), the two read the SD file's size from
 * different FatFs releases, and they seed different default sensors.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Without the
 * guard it collides with the firmware's own main() at link time, in both the
 * STM32 and the MSP430 build. Only the host-test build passes
 * -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "host_stubs.h"
#include "host_test.h"
#include "log_and_stream_includes.h"
#include "shimmer_definitions.h" /* DEVICE_VER, as the module itself includes it */

#if defined(SHIMMER3)
#define PLATFORM_NAME "Shimmer3"
#else
#define PLATFORM_NAME "Shimmer3R"
#endif

#define RAM_MAX     SHIMMER_CALIB_RAM_MAX
#define CHUNK       SHIMMER_CALIB_COPY_SIZE
#define HDR         SC_OFFSET_SENSOR_DATA /* a record's 12-byte header */
#define IMU_LEN     SC_DATA_LEN_STD_IMU_CALIB

/* The record the cases read and write. SC_SENSOR_HOST_ECG is declared on both
 * platforms and seeded on neither, so it is never among the defaults. */
#define TEST_SENSOR SC_SENSOR_HOST_ECG

/* The sensor whose calibration the config bytes keep in lnAccelCalib. */
#if defined(SHIMMER3)
#define LN_ACCEL_SENSOR SC_SENSOR_ANALOG_ACCEL
#else
#define LN_ACCEL_SENSOR SC_SENSOR_LSM6DSV_ACCEL
#endif

/* The buffer SET_CALIB_DUMP chunks are staged in. The module keeps it as a
 * plain global with no accessor; a refused chunk must leave it alone, so the
 * suite looks at it directly. */
extern uint8_t shimmerCalib_ramTemp[SHIMMER_CALIB_RAM_MAX];
/* Padding records, there only to move the next one to a chosen offset. */
#define FILLER_ID 0xFFF0
#define GUARD     0x5A

/* --- building a dump -------------------------------------------------------- */

static uint8_t *ram(void)
{
  return ShimCalib_getBytesPtr();
}

/* The dump's length field counts the bytes after its own two. */
static void setLenField(uint8_t *dump, uint16_t len)
{
  dump[SC_OFFSET_LENGTH_L] = len & 0xFF;
  dump[SC_OFFSET_LENGTH_H] = len >> 8;
}

static uint16_t ramLenField(void)
{
  return (uint16_t) (ram()[SC_OFFSET_LENGTH_L] | (ram()[SC_OFFSET_LENGTH_H] << 8));
}

/* One record at `at`: header, then a payload counting up from `first`. Written
 * only as far as the end of the RAM, so a record can be placed to run off it. */
static void
putRecord(uint8_t *dump, uint16_t at, uint16_t id, uint8_t range, uint8_t dataLen, uint8_t first)
{
  uint8_t rec[HDR + 255];
  unsigned i;

  memset(rec, 0, sizeof(rec));
  rec[SC_OFFSET_SENSOR_ID_L] = id & 0xFF;
  rec[SC_OFFSET_SENSOR_ID_H] = id >> 8;
  rec[SC_OFFSET_SENSOR_RANGE] = range;
  rec[SC_OFFSET_SENSOR_LENGTH] = dataLen;
  for (i = 0; i < dataLen; i++)
  {
    rec[HDR + i] = (uint8_t) (first + i);
  }
  for (i = 0; (i < HDR + (unsigned) dataLen) && (at + i < RAM_MAX); i++)
  {
    dump[at + i] = rec[i];
  }
}

/* Pad with filler records from the first record slot up to `end`, so the next
 * record starts there. A record is 12 to 267 bytes, so `end` must be the first
 * slot itself or at least 12 bytes past it. */
static void fillTo(uint8_t *dump, uint16_t end)
{
  uint16_t at = SC_OFFSET_FIRST_SENSOR;

  while (at < end)
  {
    uint16_t rec = end - at;
    if (rec > HDR + 255)
    {
      rec = HDR + 255;
      if (end - at - rec < HDR)
      {
        rec = end - at - HDR;
      }
    }
    putRecord(dump, at, FILLER_ID, 0, (uint8_t) (rec - HDR), 0x33);
    at += rec;
  }
}

/* A dump holding nothing but its 10-byte header, and zeros after it. */
static void ramEmpty(void)
{
  memset(ram(), 0, RAM_MAX);
  setLenField(ram(), SC_OFFSET_FIRST_SENSOR - 2);
}

static int allBytes(const uint8_t *p, unsigned n, uint8_t value)
{
  unsigned i;

  for (i = 0; i < n; i++)
  {
    if (p[i] != value)
    {
      return 0;
    }
  }
  return 1;
}

static int isRamp(const uint8_t *p, unsigned n, uint8_t first)
{
  unsigned i;

  for (i = 0; i < n; i++)
  {
    if (p[i] != (uint8_t) (first + i))
    {
      return 0;
    }
  }
  return 1;
}

/* --- a caller's sc_t, with a guard band behind it ---------------------------- */

typedef struct
{
  sc_t sc;
  uint8_t after[256];
} guarded_sc_t;

/* The guard band only means something if it starts where sc_t ends. */
_Static_assert(offsetof(guarded_sc_t, after) == sizeof(sc_t),
    "the guard band must follow sc_t directly");
_Static_assert(sizeof(sc_t) == HDR + SHIMMER_CALIB_DATA_MAX,
    "sc_t is a record header and SHIMMER_CALIB_DATA_MAX payload bytes");

/* A caller asking for TEST_SENSOR, range 0, into a slot of `slotLen` bytes.
 * Timestamp and payload start as 0xAA, so anything left unwritten shows. */
static void guardInit(guarded_sc_t *g, uint8_t slotLen)
{
  memset(&g->sc, 0xAA, sizeof(g->sc));
  memset(g->after, GUARD, sizeof(g->after));
  g->sc.id = TEST_SENSOR;
  g->sc.range = 0;
  g->sc.data_len = slotLen;
}

static int guardIntact(const guarded_sc_t *g)
{
  return allBytes(g->after, sizeof(g->after), GUARD);
}

/* --- a fake SD card ----------------------------------------------------------- *
 * FatFs as ShimCalib_file2Ram() uses it: one calibration file, read in chunks.
 * Shimmer3 (FatFs R0.08b) takes the file size from FIL; Shimmer3R (R0.12c)
 * takes it from f_stat(). Both are served from card.fileLen.
 * --------------------------------------------------------------------------- */

#define CARD_FILE_MAX    2048
/* A file read here takes at most a dozen reads. Past this many, the module is
 * looping on a card that has stopped answering. */
#define CARD_READ_BUDGET 64

static struct
{
  uint8_t file[CARD_FILE_MAX];
  uint32_t fileLen;
  uint8_t present;
  /* 0: every read succeeds. Otherwise every read from this one (1-based) on
   * returns failWith with nothing read - the card has gone away. */
  uint32_t failFromRead;
  FRESULT failWith;
  uint32_t reads;
  uint32_t closes;
  /* Opens for writing, and f_write calls: ShimCalib_ram2File() at work. */
  uint32_t writeOpens;
  uint32_t writes;
  /* Bytes a read was asked to put past the end of the calibration RAM. */
  uint32_t pastRamEnd;
  /* Set when CARD_READ_BUDGET ran out: the read loop would not have ended. */
  uint8_t stuck;
  char path[64];
} card;

static void cardReset(void)
{
  memset(&card, 0, sizeof(card));
  card.failWith = FR_DISK_ERR;
}

/* A calibration file of `len` bytes: a dump whose length field says so, then a
 * byte pattern that shows where every byte came from. */
static void cardInsert(uint32_t len)
{
  uint32_t i;

  for (i = 0; i < len; i++)
  {
    card.file[i] = (uint8_t) (i * 7U + 3U);
  }
  setLenField(card.file, (uint16_t) (len - 2U));
  card.fileLen = len;
  card.present = 1;
}

#if defined(SHIMMER3)
FRESULT f_opendir(DIRS *dp, const TCHAR *path)
#else
FRESULT f_opendir(DIR *dp, const TCHAR *path)
#endif
{
  (void) dp;
  return (strcmp(path, "/Calibration") == 0) ? FR_OK : FR_NO_PATH;
}

FRESULT f_mkdir(const TCHAR *path)
{
  (void) path;
  return FR_OK;
}

FRESULT f_open(FIL *fp, const TCHAR *path, BYTE mode)
{
  snprintf(card.path, sizeof(card.path), "%s", path);
  memset(fp, 0, sizeof(*fp));
  if (mode & FA_WRITE)
  {
    card.writeOpens++; /* ShimCalib_ram2File(), counted but not modelled */
    return FR_OK;
  }
  if (!card.present)
  {
    return FR_NO_FILE;
  }
#if defined(SHIMMER3)
  fp->fsize = card.fileLen;
#endif
  return FR_OK;
}

FRESULT f_stat(const TCHAR *path, FILINFO *fno)
{
  (void) path;
  fno->fsize = card.fileLen;
  return card.present ? FR_OK : FR_NO_FILE;
}

FRESULT f_read(FIL *fp, void *buff, UINT btr, UINT *br)
{
  uintptr_t ramStart = (uintptr_t) ShimCalib_getBytesPtr();
  uintptr_t at = (uintptr_t) buff - ramStart;
  UINT n = btr;
  UINT keep;

  *br = 0;
  card.reads++;
  if (card.reads > CARD_READ_BUDGET)
  {
    card.stuck = 1;
    fp->fptr = card.fileLen; /* end the loop, so the suite reports */
    return FR_DISK_ERR;
  }
  if (card.failFromRead && (card.reads >= card.failFromRead))
  {
    return card.failWith;
  }

  if (n > card.fileLen - fp->fptr)
  {
    n = (UINT) (card.fileLen - fp->fptr);
  }
  /* Every read here targets the calibration RAM. Copy only the part that is
   * inside it and count the rest, so a broken build reports an overrun rather
   * than committing one. */
  keep = n;
  if (at >= RAM_MAX)
  {
    keep = 0;
  }
  else if (at + n > RAM_MAX)
  {
    keep = (UINT) (RAM_MAX - at);
  }
  card.pastRamEnd += n - keep;
  memcpy(buff, card.file + fp->fptr, keep);
  fp->fptr += n;
  *br = n;
  return FR_OK;
}

FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw)
{
  (void) buff;
  card.writes++;
  fp->fptr += btw;
  *bw = btw;
  return FR_OK;
}

FRESULT f_close(FIL *fp)
{
  (void) fp;
  card.closes++;
  return FR_OK;
}

/* --- the cases ---------------------------------------------------------------- */

/* Power-on state: the firmware defaults in the RAM, no transfer under way, an
 * empty card slot. */
static void reset(void)
{
  hostStub_reset();
  cardReset();
  ShimCalib_init();
  ShimCalib_ramTempInit();
}

/*
 * GET_CALIB_DUMP. Out-of-range requests are answered with zeros, and the
 * length to zero is the host's. The caller's buffer is sized for the 128 bytes
 * a chunk can be, so a length above that must not be zeroed in full.
 */
static void test_ram_read_bounds(void)
{
  uint8_t buf[256];
  unsigned i;

  testCase("test_ram_read_bounds");
  reset();
  for (i = 0; i < RAM_MAX; i++)
  {
    ram()[i] = (uint8_t) (i ^ 0x5AU);
  }

  memset(buf, 0xAA, sizeof(buf));
  expectU("a read ending on the last byte succeeds",
      ShimCalib_ramRead(buf, CHUNK, RAM_MAX - CHUNK), 0);
  expectX("from its offset", buf[0], (uint8_t) ((RAM_MAX - CHUNK) ^ 0x5AU));
  expectX("to the RAM's last byte", buf[CHUNK - 1], (uint8_t) ((RAM_MAX - 1) ^ 0x5AU));
  expectX("and writes no further", buf[CHUNK], 0xAA);

  memset(buf, 0xAA, sizeof(buf));
  expectU("a read ending past the RAM is refused",
      ShimCalib_ramRead(buf, CHUNK, RAM_MAX - CHUNK + 1), 1);
  expectTrue("and answered with zeros", allBytes(buf, CHUNK, 0));
  expectX("only for its length", buf[CHUNK], 0xAA);

  memset(buf, 0xAA, sizeof(buf));
  expectU("a 129-byte read is refused", ShimCalib_ramRead(buf, CHUNK + 1, 0), 1);
  expectTrue("its first 128 bytes are zeroed", allBytes(buf, CHUNK, 0));
  expectX("but not the 129th", buf[CHUNK], 0xAA);

  memset(buf, 0xAA, sizeof(buf));
  expectU("a 255-byte read is refused", ShimCalib_ramRead(buf, 255, 0), 1);
  expectTrue("nothing past 128 bytes is zeroed", allBytes(buf + CHUNK, 255 - CHUNK, 0xAA));
}

/*
 * SET_CALIB_DUMP, first chunk. Its length field sets how many bytes the
 * transfer will copy into the 1024-byte RAM when it completes, so a dump that
 * would not fit is refused at the start. A field of 0xFFFF used to wrap the
 * 16-bit expected size to 1 and commit the dump on the spot.
 */
static void test_ram_write_declared_length(void)
{
  uint8_t chunk[CHUNK];
  uint8_t before[RAM_MAX];

  testCase("test_ram_write_declared_length");
  reset();
  memcpy(before, ram(), RAM_MAX);
  memset(chunk, 0x42, sizeof(chunk));

  setLenField(chunk, RAM_MAX - 1); /* 1023 + its own 2 = 1025 */
  expectX("a dump declaring 1025 bytes is refused",
      ShimCalib_ramWrite(chunk, sizeof(chunk), 0), 0xFF);

  setLenField(chunk, 0xFFFF);
  expectX("a dump declaring 65537 bytes is refused",
      ShimCalib_ramWrite(chunk, sizeof(chunk), 0), 0xFF);

  /* The other end: the field has to cover the 8-byte version header after
   * it. A shorter dump would have that header stamped outside it, and its
   * next appended record written over it. */
  setLenField(chunk, SC_OFFSET_FIRST_SENSOR - 3);
  expectX("a dump declaring 7 bytes, short of its version header, is refused",
      ShimCalib_ramWrite(chunk, sizeof(chunk), 0), 0xFF);
  setLenField(chunk, 0);
  expectX("and so is one declaring 0", ShimCalib_ramWrite(chunk, sizeof(chunk), 0), 0xFF);
  expectTrue("and the calibration RAM is untouched", memcmp(before, ram(), RAM_MAX) == 0);

  setLenField(chunk, SC_OFFSET_FIRST_SENSOR - 2);
  expectU(
      "a dump of just the header, 8 bytes, is accepted and completes at once",
      ShimCalib_ramWrite(chunk, SC_OFFSET_FIRST_SENSOR, 0), 1);
  expectU("its length field reads back as 8", ramLenField(), SC_OFFSET_FIRST_SENSOR - 2);

  setLenField(chunk, RAM_MAX - 2);
  expectU("a dump declaring exactly 1024 bytes is accepted",
      ShimCalib_ramWrite(chunk, sizeof(chunk), 0), 0);
}

/*
 * SET_CALIB_DUMP, a chunk past the start with no transfer in progress. With
 * none under way the expected total is 0, so such a chunk used to "complete" a
 * transfer at once: an empty copy, then the old dump re-applied to the
 * configuration and saved again. It is refused, and the staging buffer is not
 * written, and nothing reaches the configuration or the SD header. Three ways
 * to get there: a stray chunk before any transfer, the chunk after a refused
 * start, and one after a finished transfer. A proper transfer still goes
 * through after them.
 */
static void test_ram_write_no_transfer(void)
{
  static uint8_t ramBefore[RAM_MAX];
  static uint8_t stagingBefore[RAM_MAX];
  static gConfigBytes configBefore;
  static uint8_t sdHeadBefore[SD_HEAD_SIZE];
  static uint8_t image[CHUNK];
  gConfigBytes *config;
  uint8_t chunk[CHUNK];

  testCase("test_ram_write_no_transfer");
  reset();
  memset(chunk, 0x42, sizeof(chunk));
  /* A completed transfer copies the dump on into the configuration bytes and
   * the SD header. Mark both so that a refused chunk visibly reached neither. */
  config = ShimConfig_getStoredConfig();
  memset(config, 0xEE, sizeof(*config));
  memset(ShimSdHead_getSdHeadText(), 0xEE, SD_HEAD_SIZE);
  memcpy(&configBefore, config, sizeof(configBefore));
  memcpy(sdHeadBefore, ShimSdHead_getSdHeadText(), SD_HEAD_SIZE);

  memcpy(ramBefore, ram(), RAM_MAX);
  memcpy(stagingBefore, shimmerCalib_ramTemp, RAM_MAX);
  expectX("a chunk at offset 128 before any transfer is refused",
      ShimCalib_ramWrite(chunk, CHUNK, CHUNK), 0xFF);
  expectX("and so is one at offset 2", ShimCalib_ramWrite(chunk, CHUNK, 2), 0xFF);
  expectTrue("the staging buffer is untouched",
      memcmp(stagingBefore, shimmerCalib_ramTemp, RAM_MAX) == 0);
  expectTrue("and so is the calibration RAM", memcmp(ramBefore, ram(), RAM_MAX) == 0);
  expectTrue("and the config bytes",
      memcmp(&configBefore, config, sizeof(configBefore)) == 0);
  expectTrue("and the SD header",
      memcmp(sdHeadBefore, ShimSdHead_getSdHeadText(), SD_HEAD_SIZE) == 0);

  /* A start declaring too much is refused, and that leaves no transfer open. */
  setLenField(chunk, RAM_MAX - 1);
  expectX("an oversized start is refused", ShimCalib_ramWrite(chunk, CHUNK, 0), 0xFF);
  memcpy(stagingBefore, shimmerCalib_ramTemp, RAM_MAX);
  memset(chunk, 0x43, sizeof(chunk));
  expectX("the chunk after a refused start is refused",
      ShimCalib_ramWrite(chunk, CHUNK, CHUNK), 0xFF);
  expectTrue("without touching the staging buffer",
      memcmp(stagingBefore, shimmerCalib_ramTemp, RAM_MAX) == 0);
  expectTrue("or the calibration RAM", memcmp(ramBefore, ram(), RAM_MAX) == 0);

  expectTrue("or the config bytes",
      memcmp(&configBefore, config, sizeof(configBefore)) == 0);
  expectTrue("or the SD header",
      memcmp(sdHeadBefore, ShimSdHead_getSdHeadText(), SD_HEAD_SIZE) == 0);

  /* After all those refusals a proper transfer still goes through. A 128-byte
   * dump completes in its one chunk, and closes the transfer. */
  memset(image, 0, sizeof(image));
  setLenField(image, CHUNK - 2);
  fillTo(image, CHUNK);
  expectU("a one-chunk dump from offset 0 still completes",
      ShimCalib_ramWrite(image, CHUNK, 0), 1);
  expectTrue("and lands in the calibration RAM",
      memcmp(ram() + SC_OFFSET_FIRST_SENSOR, image + SC_OFFSET_FIRST_SENSOR, CHUNK - SC_OFFSET_FIRST_SENSOR)
          == 0);
  memcpy(ramBefore, ram(), RAM_MAX);
  memcpy(stagingBefore, shimmerCalib_ramTemp, RAM_MAX);
  expectX("a chunk after a finished transfer is refused",
      ShimCalib_ramWrite(chunk, CHUNK, CHUNK), 0xFF);
  expectTrue("without touching the staging buffer",
      memcmp(stagingBefore, shimmerCalib_ramTemp, RAM_MAX) == 0);
  expectTrue("or the calibration RAM", memcmp(ramBefore, ram(), RAM_MAX) == 0);
}

/*
 * SET_CALIB_DUMP, a start without its whole length header. The dump's first
 * two bytes are its declared length. A start that does not carry both - length
 * 0 or 1 at offset 0, or any start at offset 1 - used to take the missing byte
 * from the staging buffer, which still holds the previous transfer, and open a
 * transfer of that stale length. Such a start is refused, and it closes any
 * transfer in progress.
 *
 * A start that does carry the header clears what the previous transfer left in
 * the staging buffer. Completion counts bytes received, not bytes covered, so a
 * host that repeats a chunk can complete a transfer with a gap in it, and that
 * gap used to commit the previous dump's bytes.
 */
static void test_ram_write_start_header(void)
{
  static uint8_t image[RAM_MAX];
  uint8_t chunk[CHUNK];
  const uint16_t total = 300; /* declared 298 + the 2 length bytes */

  testCase("test_ram_write_start_header");
  reset();
  memset(chunk, 0x66, sizeof(chunk));

  /* Leave a 300-byte transfer open, with its header in the staging buffer. */
  memset(image, 0x55, sizeof(image));
  setLenField(image, total - 2);
  expectU("a 300-byte transfer starts", ShimCalib_ramWrite(image, CHUNK, 0), 0);

  expectX("a zero-length start is refused", ShimCalib_ramWrite(chunk, 0, 0), 0xFF);
  expectX("and it closes the open transfer",
      ShimCalib_ramWrite(image + CHUNK, CHUNK, CHUNK), 0xFF);
  expectX("a one-byte start is refused", ShimCalib_ramWrite(chunk, 1, 0), 0xFF);
  expectX("so is a start at offset 1", ShimCalib_ramWrite(chunk, CHUNK, 1), 0xFF);

  /* A whole 300-byte dump of 0x55 goes through... */
  expectU("chunk 1 of the 0x55 dump", ShimCalib_ramWrite(image, CHUNK, 0), 0);
  expectU("chunk 2", ShimCalib_ramWrite(image + CHUNK, CHUNK, CHUNK), 0);
  expectU("chunk 3 completes it",
      ShimCalib_ramWrite(image + 2 * CHUNK, total - 2 * CHUNK, 2 * CHUNK), 1);
  expectTrue("its tail is in the RAM", allBytes(ram() + 2 * CHUNK, total - 2 * CHUNK, 0x55));

  /* ...then a 300-byte dump of 0x66 whose second chunk is sent twice and whose
   * third never is. It completes on byte count, and its unwritten tail is zero,
   * not the 0x55 dump's. */
  memset(image, 0x66, sizeof(image));
  setLenField(image, total - 2);
  expectU("chunk 1 of the 0x66 dump", ShimCalib_ramWrite(image, CHUNK, 0), 0);
  expectU("chunk 2", ShimCalib_ramWrite(image + CHUNK, CHUNK, CHUNK), 0);
  expectU("chunk 2 again completes it on count",
      ShimCalib_ramWrite(image + CHUNK, CHUNK, CHUNK), 1);
  expectTrue("its second chunk is in the RAM", allBytes(ram() + CHUNK, CHUNK, 0x66));
  expectTrue("and the tail it never sent is zero, not the old dump's",
      allBytes(ram() + 2 * CHUNK, total - 2 * CHUNK, 0));
}

/* SET_CALIB_DUMP, the transfer that should go through: chunks at increasing
 * offsets, committed when the declared length has arrived. */
static void test_ram_write_multi_chunk(void)
{
  static uint8_t image[RAM_MAX];
  sc_t sc;
  unsigned i;

  testCase("test_ram_write_multi_chunk");
  reset();

  /* 300 bytes: one record, then a filler to the end. */
  memset(image, 0, sizeof(image));
  setLenField(image, 300 - 2);
  putRecord(image, SC_OFFSET_FIRST_SENSOR, TEST_SENSOR, 1, IMU_LEN, 0x60);
  putRecord(image, SC_OFFSET_FIRST_SENSOR + HDR + IMU_LEN, FILLER_ID, 0,
      300 - (SC_OFFSET_FIRST_SENSOR + HDR + IMU_LEN) - HDR, 0x33);

  expectU("chunk 1 of 3 is taken", ShimCalib_ramWrite(image, CHUNK, 0), 0);
  expectU("chunk 2 of 3 is taken", ShimCalib_ramWrite(image + CHUNK, CHUNK, CHUNK), 0);
  expectU("chunk 3 completes the dump",
      ShimCalib_ramWrite(image + 2 * CHUNK, 300 - 2 * CHUNK, 2 * CHUNK), 1);
  expectU("its length field is the host's", ramLenField(), 300 - 2);
  expectTrue("every record byte arrived",
      memcmp(ram() + SC_OFFSET_FIRST_SENSOR, image + SC_OFFSET_FIRST_SENSOR, 300 - SC_OFFSET_FIRST_SENSOR)
          == 0);
  expectX("and the version bytes are the firmware's own",
      ram()[SC_OFFSET_VER_HW_ID_L], DEVICE_VER & 0xFF);

  memset(&sc, 0, sizeof(sc));
  sc.id = TEST_SENSOR;
  sc.range = 1;
  sc.data_len = IMU_LEN;
  expectU("the record reads back", ShimCalib_singleSensorRead(&sc), 0);
  expectTrue("with its payload", isRamp(sc.data.raw, IMU_LEN, 0x60));

  /* The largest dump there is, eight full chunks, still goes through. */
  memset(image, 0, sizeof(image));
  setLenField(image, RAM_MAX - 2);
  fillTo(image, RAM_MAX);
  for (i = 0; i < RAM_MAX / CHUNK - 1; i++)
  {
    expectU("a chunk of the 1024-byte dump is taken",
        ShimCalib_ramWrite(image + i * CHUNK, CHUNK, (uint16_t) (i * CHUNK)), 0);
  }
  expectU("the eighth chunk completes a 1024-byte dump",
      ShimCalib_ramWrite(image + RAM_MAX - CHUNK, CHUNK, RAM_MAX - CHUNK), 1);
  expectU("whose length field is 1022", ramLenField(), RAM_MAX - 2);
}

/*
 * singleSensorRead, the slot. The caller's data_len is the size of the slot it
 * copies the result into - InfoMem, the SD header, a BT response - so it is
 * the most that may come back, whatever length the record's own byte claims.
 * And no slot can be larger than the sc_t it lives in.
 */
static void test_single_sensor_read_slot(void)
{
  guarded_sc_t g;

  testCase("test_single_sensor_read_slot");
  reset();
  ramEmpty();
  putRecord(ram(), SC_OFFSET_FIRST_SENSOR, TEST_SENSOR, 0, 40, 0x70);
  setLenField(ram(), SC_OFFSET_FIRST_SENSOR + HDR + 40 - 2);

  guardInit(&g, IMU_LEN);
  expectU("a 40-byte record is found", ShimCalib_singleSensorRead(&g.sc), 0);
  expectU("into a 21-byte slot it reads back as 21 bytes", g.sc.data_len, IMU_LEN);
  expectTrue("the record's first 21", isRamp(g.sc.data.raw, IMU_LEN, 0x70));
  expectX("the slot's spare byte is untouched", g.sc.data.raw[IMU_LEN], 0xAA);
  expectTrue("and nothing past the caller's sc_t is written", guardIntact(&g));

  guardInit(&g, 255);
  expectU("into a 255-byte slot it is found", ShimCalib_singleSensorRead(&g.sc), 0);
  expectU("cut to the 22 bytes sc_t holds", g.sc.data_len, SHIMMER_CALIB_DATA_MAX);
  expectTrue("and nothing past the caller's sc_t is written", guardIntact(&g));
}

/* singleSensorRead, the RAM end. A record placed so its payload runs off the
 * end of the 1024-byte RAM is read only as far as the RAM goes. */
static void test_single_sensor_read_ram_end(void)
{
  guarded_sc_t g;
  const uint16_t at = 1000; /* payload at 1012-1032; the RAM ends at 1023 */

  testCase("test_single_sensor_read_ram_end");
  reset();
  ramEmpty();
  fillTo(ram(), at);
  putRecord(ram(), at, TEST_SENSOR, 0, IMU_LEN, 0x40);
  setLenField(ram(), RAM_MAX - 2);

  guardInit(&g, IMU_LEN);
  expectU("a record running off the RAM is found", ShimCalib_singleSensorRead(&g.sc), 0);
  expectU("its payload is cut at the RAM end", g.sc.data_len, RAM_MAX - (at + HDR));
  expectTrue("to the bytes that are there",
      isRamp(g.sc.data.raw, RAM_MAX - (at + HDR), 0x40));
  expectTrue("and zeros for the rest of the slot",
      allBytes(g.sc.data.raw + (RAM_MAX - (at + HDR)), IMU_LEN - (RAM_MAX - (at + HDR)), 0));

  /* A record whose 12-byte header itself is cut by the end is not a record. */
  ramEmpty();
  fillTo(ram(), RAM_MAX - 8);
  putRecord(ram(), RAM_MAX - 8, TEST_SENSOR, 0, IMU_LEN, 0x40);
  setLenField(ram(), RAM_MAX - 2);

  guardInit(&g, IMU_LEN);
  expectU("a record with only 8 header bytes in the RAM is not matched",
      ShimCalib_singleSensorRead(&g.sc), 1);
  expectTrue("and the slot reads as zeros", allBytes(g.sc.data.raw, IMU_LEN, 0));
}

/*
 * singleSensorRead and singleSensorWrite, the declared end. The dump is as long
 * as its own length field says, and that can end well before the RAM does.
 * What lies after it is not part of the dump: the rest of an earlier, longer
 * dump, or of a calibration file longer than its own length field. So a record
 * whose payload runs past the declared end is read, and updated, only up to it.
 */
static void test_single_sensor_declared_end(void)
{
  guarded_sc_t g;
  const uint16_t data = SC_OFFSET_FIRST_SENSOR + HDR;
  const uint16_t inDump = 5; /* payload bytes inside the declared length */
  unsigned i;

  testCase("test_single_sensor_declared_end");
  reset();
  ramEmpty();
  putRecord(ram(), SC_OFFSET_FIRST_SENSOR, TEST_SENSOR, 0, IMU_LEN, 0x40);
  setLenField(ram(), data + inDump - 2);

  guardInit(&g, IMU_LEN);
  expectU("a record cut by the declared end is found",
      ShimCalib_singleSensorRead(&g.sc), 0);
  expectU("its payload is cut at the declared end", g.sc.data_len, inDump);
  expectTrue("to the bytes inside the dump", isRamp(g.sc.data.raw, inDump, 0x40));
  expectTrue("and zeros for the rest of the slot, not the bytes after the dump",
      allBytes(g.sc.data.raw + inDump, IMU_LEN - inDump, 0));

  guardInit(&g, IMU_LEN);
  for (i = 0; i < sizeof(g.sc.data.raw); i++)
  {
    g.sc.data.raw[i] = (uint8_t) (0xC0 + i);
  }
  expectU("the same record is updated", ShimCalib_singleSensorWrite(&g.sc), 0);
  expectTrue("inside the dump", isRamp(ram() + data, inDump, 0xC0));
  expectTrue("but not past its declared end",
      isRamp(ram() + data + inDump, IMU_LEN - inDump, (uint8_t) (0x40 + inDump)));
  expectU("and the dump keeps its length", ramLenField(), data + inDump - 2);
}

/* singleSensorRead, a sensor the dump does not hold. It reads back as the
 * all-zero block and the all-zero timestamp of a default calibration. The
 * timestamp used to be left as it was, and the SD header got stack bytes. */
static void test_single_sensor_read_missing(void)
{
  guarded_sc_t g;

  testCase("test_single_sensor_read_missing");
  reset(); /* the defaults, which never include TEST_SENSOR */

  guardInit(&g, IMU_LEN);
  expectU("a sensor missing from the dump is not found",
      ShimCalib_singleSensorRead(&g.sc), 1);
  expectTrue("its timestamp reads as zero", allBytes(g.sc.ts, SC_TIMESTAMP_LENGTH, 0));
  expectTrue("and its payload too", allBytes(g.sc.data.raw, IMU_LEN, 0));
  expectU("at the slot's length", g.sc.data_len, IMU_LEN);
}

/*
 * singleSensorWrite, a new record. It is appended only if all of it fits in
 * the RAM; a full dump keeps what it has. A record claiming a payload larger
 * than sc_t holds is refused too, because the append copies that many bytes
 * out of the caller's sc_t.
 */
static void test_single_sensor_write_full_ram(void)
{
  static uint8_t before[RAM_MAX];
  guarded_sc_t g;
  /* The largest dump a 33-byte record still fits: 2 + 989 + 33 = 1024. */
  const uint16_t fitsLen = RAM_MAX - 2 - (HDR + IMU_LEN);

  testCase("test_single_sensor_write_full_ram");
  reset();

  ramEmpty();
  fillTo(ram(), fitsLen + 2);
  setLenField(ram(), fitsLen);
  guardInit(&g, IMU_LEN);
  memset(g.sc.ts, 0, sizeof(g.sc.ts));
  memset(g.sc.data.raw, 0x21, sizeof(g.sc.data.raw));
  expectU("a record that exactly fills the RAM is appended",
      ShimCalib_singleSensorWrite(&g.sc), 0);
  expectU("and the dump is full", ramLenField(), RAM_MAX - 2);
  expectTrue("with its payload in the last 21 bytes",
      allBytes(ram() + RAM_MAX - IMU_LEN, IMU_LEN, 0x21));

  ramEmpty();
  fillTo(ram(), 1000 + 2);
  setLenField(ram(), 1000);
  memcpy(before, ram(), RAM_MAX);
  expectU("a record that does not fit is refused", ShimCalib_singleSensorWrite(&g.sc), 1);
  expectU("the dump keeps its length", ramLenField(), 1000);
  expectTrue("and its bytes", memcmp(before, ram(), RAM_MAX) == 0);

  ramEmpty();
  g.sc.data_len = SHIMMER_CALIB_DATA_MAX + 1;
  expectU("a record longer than sc_t holds is refused",
      ShimCalib_singleSensorWrite(&g.sc), 1);
  expectU("and not appended", ramLenField(), SC_OFFSET_FIRST_SENSOR - 2);
}

/*
 * singleSensorWrite, an existing record. The new payload is copied over the
 * old one for the length the STORED record gives, and that length came from a
 * host or the SD card. It is capped at the caller's payload (data_len), and
 * that at what the caller's sc_t holds; a longer stored record keeps the rest
 * of its bytes.
 */
static void test_single_sensor_write_stored_length(void)
{
  guarded_sc_t g;
  const uint16_t data = SC_OFFSET_FIRST_SENSOR + HDR;
  unsigned i;

  testCase("test_single_sensor_write_stored_length");
  reset();
  ramEmpty();
  putRecord(ram(), SC_OFFSET_FIRST_SENSOR, TEST_SENSOR, 0, 200, 0);
  memset(ram() + data, 0x11, 200);
  setLenField(ram(), data + 200 - 2);

  guardInit(&g, IMU_LEN);
  for (i = 0; i < SC_TIMESTAMP_LENGTH; i++)
  {
    g.sc.ts[i] = (uint8_t) (i + 1);
  }
  for (i = 0; i < sizeof(g.sc.data.raw); i++)
  {
    g.sc.data.raw[i] = (uint8_t) (0xC0 + i);
  }
  expectU("the stored 200-byte record is updated", ShimCalib_singleSensorWrite(&g.sc), 0);
  expectTrue("with the caller's timestamp",
      memcmp(ram() + SC_OFFSET_FIRST_SENSOR + SC_OFFSET_SENSOR_TIMESTAMP, g.sc.ts, SC_TIMESTAMP_LENGTH)
          == 0);
  expectTrue("and the caller's 21 payload bytes", isRamp(ram() + data, IMU_LEN, 0xC0));
  expectTrue("but nothing past the caller's payload",
      allBytes(ram() + data + IMU_LEN, 200 - IMU_LEN, 0x11));

  /* A caller claiming more than sc_t holds is still held to sc_t. */
  memset(ram() + data, 0x11, 200);
  guardInit(&g, 255);
  for (i = 0; i < sizeof(g.sc.data.raw); i++)
  {
    g.sc.data.raw[i] = (uint8_t) (0xC0 + i);
  }
  expectU("a 255-byte payload claim still updates the record",
      ShimCalib_singleSensorWrite(&g.sc), 0);
  expectTrue("with the 22 bytes sc_t holds",
      isRamp(ram() + data, SHIMMER_CALIB_DATA_MAX, 0xC0));
  expectTrue("and nothing from past the caller's sc_t",
      allBytes(ram() + data + SHIMMER_CALIB_DATA_MAX, 200 - SHIMMER_CALIB_DATA_MAX, 0x11));
}

/*
 * file2Ram, a file larger than the RAM. The size is whatever is on the card,
 * and the read used to run on to the end of the file, past the calibration
 * RAM. It stops at the RAM's 1024 bytes.
 */
static void test_file2ram_truncates(void)
{
  static uint8_t before[RAM_MAX];

  testCase("test_file2ram_truncates");
  reset();

  memcpy(before, ram(), RAM_MAX);
  expectU("a missing file is reported", ShimCalib_file2Ram(), 1);
  expectTrue("and leaves the defaults", memcmp(before, ram(), RAM_MAX) == 0);

  cardInsert(1500);
  expectU("a 1500-byte file is read", ShimCalib_file2Ram(), 0);
  expectStr("from the file named for the MAC", card.path, "/Calibration/calib_5A1C");
  expectU("no read reaches past the calibration RAM", card.pastRamEnd, 0);
  expectU("which takes eight 128-byte reads", card.reads, RAM_MAX / CHUNK);
  expectTrue("and holds the file's first 1024 bytes",
      memcmp(ram(), card.file, RAM_MAX) == 0);
  expectU("the file is closed", card.closes, 1);
}

/*
 * file2Ram, a card that stops answering part-way. A failed read leaves the
 * file pointer where it was, and the loop, which runs until the pointer
 * reaches the file size, used to spin there for ever. So does a read that
 * succeeds with nothing in it, which is what a file shorter than its directory
 * entry gives. Either one ends the read.
 *
 * The RAM then holds part of a file, and the callers would apply it to the
 * configuration. So it is rebuilt from the configuration bytes instead, the
 * way boot builds it before it reads the file: ShimCalib_init(), then
 * ShimCalib_initFromConfigBytesAll(). And file2Ram still reports success,
 * because on failure its callers write the RAM back to the card, over a file
 * that may read cleanly next time.
 */
static void test_file2ram_failed_read(void)
{
  static const FRESULT failures[] = { FR_DISK_ERR, FR_OK };
  static const char *const names[]
      = { "test_file2ram_failed_read (FR_DISK_ERR)", "test_file2ram_failed_read (FR_OK, 0 bytes)" };
  static uint8_t booted[RAM_MAX];
  gConfigBytes *config;
  sc_t sc;
  unsigned f;
  unsigned i;

  for (f = 0; f < sizeof(failures) / sizeof(failures[0]); f++)
  {
    testCase(names[f]);
    reset();

    /* A calibration in the config bytes, for the rebuild to carry over. */
    config = ShimConfig_getStoredConfig();
    for (i = 0; i < sizeof(config->lnAccelCalib.rawBytes); i++)
    {
      config->lnAccelCalib.rawBytes[i] = (uint8_t) (0x30 + i);
    }
    /* What boot builds from that configuration, before any file is read. */
    ShimCalib_init();
    ShimCalib_initFromConfigBytesAll();
    memcpy(booted, ram(), RAM_MAX);

    cardInsert(300);
    card.failFromRead = 2;
    card.failWith = failures[f];

    expectU("file2Ram still reports success", ShimCalib_file2Ram(), 0);
    expectFalse("does not leave the read loop spinning", card.stuck);
    expectU("the read stops at it", card.reads, 2);
    expectU("and the file is closed", card.closes, 1);

    expectTrue("the RAM is rebuilt as boot builds it", memcmp(booted, ram(), RAM_MAX) == 0);
    expectFalse("with nothing of the file's first chunk",
        memcmp(ram(), card.file, CHUNK) == 0);
    expectTrue("its length field is the rebuilt dump's, not the file's",
        ramLenField() != 300 - 2);
    expectX("its version bytes are the firmware's",
        ram()[SC_OFFSET_VER_HW_ID_L], DEVICE_VER & 0xFF);

    memset(&sc, 0xAA, sizeof(sc));
    sc.id = LN_ACCEL_SENSOR;
    sc.range = 0;
    sc.data_len = IMU_LEN;
    expectU("the config bytes' accel calibration is in it",
        ShimCalib_singleSensorRead(&sc), 0);
    expectTrue("with their payload", isRamp(sc.data.raw, IMU_LEN, 0x30));
    expectTrue("and the zero timestamp of a boot rebuild",
        allBytes(sc.ts, SC_TIMESTAMP_LENGTH, 0));

    expectU("nothing is opened for writing on the card", card.writeOpens, 0);
    expectU("and nothing written to it", card.writes, 0);
  }
}

/*
 * file2Ram, a file whose length field is too short for its own version
 * header. The file reads cleanly, but a dump declaring fewer than 8 bytes has
 * its header outside itself, so it is not a dump the firmware can use or save
 * back. It is handled like a failed read: the RAM is rebuilt as boot builds
 * it, the call still reports success, and the card is left alone.
 */
static void test_file2ram_short_header(void)
{
  static uint8_t booted[RAM_MAX];

  testCase("test_file2ram_short_header");
  reset();
  ShimCalib_init();
  ShimCalib_initFromConfigBytesAll();
  memcpy(booted, ram(), RAM_MAX);

  cardInsert(300);
  setLenField(card.file, SC_OFFSET_FIRST_SENSOR - 3); /* 7, one short */

  expectU("file2Ram reports success", ShimCalib_file2Ram(), 0);
  expectTrue("the RAM is rebuilt as boot builds it", memcmp(booted, ram(), RAM_MAX) == 0);
  expectU("with the rebuilt dump's length field, not the file's 7", ramLenField(),
      (uint16_t) ((booted[SC_OFFSET_LENGTH_H] << 8) | booted[SC_OFFSET_LENGTH_L]));
  expectU("nothing is opened for writing on the card", card.writeOpens, 0);
}

#if defined(SHIMMER3R)
/*
 * dump to config bytes and SD header, a sensor with no slot. On Shimmer3R the
 * switch that picks the destination offsets had an empty default, so an
 * unlisted sensor was copied using offsets nobody had set. It now copies
 * nothing. (Shimmer3's default falls back to the analog accelerometer's slot,
 * which is a separate behaviour and not pinned here.)
 */
static void test_dump_to_config_unknown_sensor(void)
{
  static gConfigBytes configBefore;
  static uint8_t sdHeadBefore[SD_HEAD_SIZE];
  gConfigBytes *config;

  testCase("test_dump_to_config_unknown_sensor");
  reset();
  config = ShimConfig_getStoredConfig();
  memset(config, 0xEE, sizeof(*config));
  memset(ShimSdHead_getSdHeadText(), 0xEE, SD_HEAD_SIZE);
  memcpy(&configBefore, config, sizeof(configBefore));
  memcpy(sdHeadBefore, ShimSdHead_getSdHeadText(), SD_HEAD_SIZE);

  ShimCalib_calibDumpToConfigBytesAndSdHeaderSingleSensor(TEST_SENSOR, 1);
  expectTrue("the config bytes are untouched",
      memcmp(&configBefore, config, sizeof(configBefore)) == 0);
  expectTrue("and so is the SD header",
      memcmp(sdHeadBefore, ShimSdHead_getSdHeadText(), SD_HEAD_SIZE) == 0);
}
#endif

int main(void)
{
  printf("shimmer_calibration host tests (" PLATFORM_NAME ")\n\n");
  hostTestSilenceUnused();

  test_ram_read_bounds();
  test_ram_write_declared_length();
  test_ram_write_no_transfer();
  test_ram_write_start_header();
  test_ram_write_multi_chunk();
  test_single_sensor_read_slot();
  test_single_sensor_read_ram_end();
  test_single_sensor_declared_end();
  test_single_sensor_read_missing();
  test_single_sensor_write_full_ram();
  test_single_sensor_write_stored_length();
  test_file2ram_truncates();
  test_file2ram_failed_read();
  test_file2ram_short_header();
#if defined(SHIMMER3R)
  test_dump_to_config_unknown_sensor();
#endif

  return hostTestReport("shimmer_calibration (" PLATFORM_NAME ")");
}

#endif /* SHIMMER_HOST_TEST */
