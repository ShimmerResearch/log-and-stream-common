/*
 * Host-side test for EEPROM/cat24c16.c, the CAT24C16 driver both firmwares use
 * (DEV-1496).
 *
 * The bus is a fake CAT24C16 behind the driver's own callbacks, modelled on the
 * datasheet: 16-byte pages that wrap rather than cross, the block number in the
 * low 3 bits of the I2C address, an address counter that reads on across pages
 * and blocks, and a 5 ms write cycle (tWR max) that starts at the STOP of a
 * write carrying data, during which the part acknowledges nothing. Time only
 * moves when the driver calls mdelay(), so a missing wait shows up as a NACK
 * rather than as a flaky test.
 *
 * Faults are injected by call number: a write that fails before its data
 * lands, one whose data lands but which reports an error anyway (so a write
 * cycle is running when it returns), a read that fails, and a read that comes
 * back with a bit flipped.
 *
 * The self-test cases are DEV-1494/DEV-1495's: whatever happens after the
 * first test write, the bytes it used are put back, and only when they cannot
 * be is EEPROM_TEST_FAIL_NOT_RESTORED set.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include "host_test.h"

#include "EEPROM/cat24c16.h"
#include "Test/shimmer_test.h"

#define MAX_FAULTS 4

static uint8_t mem[CAT24C16_TOTAL_SIZE];
static uint8_t original[CAT24C16_TOTAL_SIZE];

static uint32_t nowMs;
static uint32_t busyUntilMs;
static uint32_t writeCalls, readCalls, nacksWhileBusy;
static uint8_t lastDevAddr;
static int badAddress;
static int badHandle;
static void *expectedHandle;

static uint32_t failWriteAt[MAX_FAULTS]; /* 1-based write call numbers */
static uint32_t failWriteAfterAt[MAX_FAULTS]; /* data lands, then it reports an error */
static uint32_t failReadAt[MAX_FAULTS];
static uint32_t corruptReadAt[MAX_FAULTS];
static uint32_t failWritesFrom; /* every write from this call on fails, 0 = never */

/* Every data byte the driver sent, in order */
static uint8_t writeLog[1024];
static uint16_t writeLogLen;

static int inList(const uint32_t *list, uint32_t n)
{
  int i;

  for (i = 0; i < MAX_FAULTS; i++)
  {
    if (list[i] == n)
    {
      return 1;
    }
  }
  return 0;
}

static int isBusy(void)
{
  return nowMs < busyUntilMs;
}

static void checkBus(void *handle, uint8_t devAddr)
{
  lastDevAddr = devAddr;
  if ((devAddr & 0xF8) != CAT24C16_ADDR)
  {
    badAddress = 1;
  }
  if (handle != expectedHandle)
  {
    badHandle = 1;
  }
}

static int32_t fakeWrite(void *handle, uint8_t devAddr, uint8_t *buf, uint16_t len)
{
  uint16_t addr, pageBase, i;

  writeCalls++;
  checkBus(handle, devAddr);
  if (isBusy())
  {
    nacksWhileBusy++;
    return 1; /* the address is not acknowledged during a write cycle */
  }
  if (inList(failWriteAt, writeCalls) || (failWritesFrom && writeCalls >= failWritesFrom))
  {
    return 1;
  }

  addr = (uint16_t) ((devAddr & 0x07) * CAT24C16_BLOCK_SIZE + buf[0]);
  pageBase = (uint16_t) (addr & ~(CAT24C16_PAGE_SIZE - 1));
  for (i = 0; i + 1 < len; i++)
  {
    mem[pageBase + ((addr + i) % CAT24C16_PAGE_SIZE)] = buf[1 + i];
    if (writeLogLen < sizeof(writeLog))
    {
      writeLog[writeLogLen++] = buf[1 + i];
    }
  }
  if (len > 1)
  {
    busyUntilMs = nowMs + CAT24C16_WRITE_CYCLE_MS;
  }
  return inList(failWriteAfterAt, writeCalls) ? 1 : 0;
}

static int32_t
fakeRead(void *handle, uint8_t devAddr, uint8_t wordAddr, uint8_t *outBuffer, uint16_t len)
{
  uint16_t addr, i;

  readCalls++;
  checkBus(handle, devAddr);
  /* Scribble first, so a failure that left the buffer alone is caught */
  memset(outBuffer, 0x5A, len);
  if (isBusy())
  {
    nacksWhileBusy++;
    return 1;
  }
  if (inList(failReadAt, readCalls))
  {
    return 1;
  }

  addr = (uint16_t) ((devAddr & 0x07) * CAT24C16_BLOCK_SIZE + wordAddr);
  for (i = 0; i < len; i++)
  {
    outBuffer[i] = mem[(addr + i) % CAT24C16_TOTAL_SIZE];
  }
  if (inList(corruptReadAt, readCalls))
  {
    outBuffer[len / 2] ^= 0x01;
  }
  return 0;
}

static void fakeDelay(uint32_t ms)
{
  nowMs += ms;
}

static uint8_t handleA, handleB;
static cat24c16dev_ctx_t ctx;

static void reset(void)
{
  uint16_t i;

  for (i = 0; i < CAT24C16_TOTAL_SIZE; i++)
  {
    mem[i] = (uint8_t) (i * 37 + 11);
  }
  memcpy(original, mem, sizeof(mem));
  nowMs = 1000;
  busyUntilMs = 0;
  writeCalls = readCalls = nacksWhileBusy = 0;
  failWritesFrom = 0;
  badAddress = badHandle = 0;
  writeLogLen = 0;
  memset(failWriteAt, 0, sizeof(failWriteAt));
  memset(failWriteAfterAt, 0, sizeof(failWriteAfterAt));
  memset(failReadAt, 0, sizeof(failReadAt));
  memset(corruptReadAt, 0, sizeof(corruptReadAt));

  ctx.write = fakeWrite;
  ctx.read = fakeRead;
  ctx.mdelay = fakeDelay;
  ctx.handle = &handleA;
  expectedHandle = &handleA;
}

static int regionHolds(const uint8_t *want)
{
  return memcmp(&mem[CAT24C16_TEST_OFFSET], &want[CAT24C16_TEST_OFFSET], CAT24C16_TEST_SIZE)
      == 0;
}

static int outsideRegionUntouched(void)
{
  return memcmp(mem, original, CAT24C16_TEST_OFFSET) == 0
      && memcmp(&mem[CAT24C16_TEST_OFFSET + CAT24C16_TEST_SIZE],
             &original[CAT24C16_TEST_OFFSET + CAT24C16_TEST_SIZE],
             CAT24C16_TOTAL_SIZE - CAT24C16_TEST_OFFSET - CAT24C16_TEST_SIZE)
      == 0;
}

/* Run the self-test with the faults already set and check the outcome */
static void runSelfTest(const char *name, uint8_t want, int wantRestored)
{
  uint8_t got;

  testCase(name);
  got = CAT24C16_test(&ctx);
  expectX("result", got, want);
  expectU("test region holds the backup", regionHolds(original), (uint64_t) wantRestored);
  expectTrue("bytes outside the test region untouched", outsideRegionUntouched());
  expectFalse("only 0x50-0x57 addressed", badAddress);
  expectFalse("the context's handle reached the bus", badHandle);
  expectFalse("left the part mid write cycle", isBusy());
}

/* 128 bytes from offset 16 are 8 page writes a pass and one read */
#define PAGES 8

static void test_self_test(void)
{
  uint16_t i;
  int samePosition12 = 0, samePosition23 = 0;

  reset();
  runSelfTest("healthy part", EEPROM_TEST_PASS, 1);
  expectU("writes: 3 passes and the restore", writeCalls, 4 * PAGES);
  expectU("reads: backup, 3 passes and the restore", readCalls, 5);
  expectU("no transfer met a write cycle", nacksWhileBusy, 0);
  /* Each pass must write a different value to every byte, or a pass whose
   * write never happened would read back the previous pass's data as a pass */
  for (i = 0; i < CAT24C16_TEST_SIZE; i++)
  {
    samePosition12 += writeLog[i] == writeLog[CAT24C16_TEST_SIZE + i];
    samePosition23 += writeLog[CAT24C16_TEST_SIZE + i]
        == writeLog[2 * CAT24C16_TEST_SIZE + i];
  }
  expectU("bytes pass 2 wrote as pass 1 did", (uint64_t) samePosition12, 0);
  expectU("bytes pass 3 wrote as pass 2 did", (uint64_t) samePosition23, 0);

  reset();
  failReadAt[0] = 1;
  runSelfTest("backup read fails", EEPROM_TEST_FAIL_INITIAL_BACKUP, 1);
  expectU("nothing written", writeCalls, 0);

  reset();
  failWriteAt[0] = 3; /* pass 1, page 3: pages 1-2 already hold test data */
  runSelfTest("write fails mid-pass 1", EEPROM_TEST_FAIL_WRITING_BUF, 1);

  reset();
  failWriteAfterAt[0] = 12; /* pass 2, page 4: lands, reports an error, cycle running */
  runSelfTest("write lands but reports an error", EEPROM_TEST_FAIL_WRITING_BUF, 1);
  expectU("the restore did not meet that cycle", nacksWhileBusy, 0);

  reset();
  failReadAt[0] = 3; /* backup = read 1, pass 1 = read 2, pass 2 = read 3 */
  runSelfTest("read-back fails on pass 2", EEPROM_TEST_FAIL_READING_BUF, 1);

  reset();
  corruptReadAt[0] = 4; /* pass 3's read-back */
  runSelfTest("read-back mismatch on pass 3", EEPROM_TEST_FAIL_BUF_COMPARISON, 1);

  reset();
  corruptReadAt[0] = 2;       /* pass 1 mismatches */
  failWritesFrom = PAGES + 1; /* and no write after pass 1 works */
  runSelfTest("mismatch, then the restore cannot write",
      EEPROM_TEST_FAIL_BUF_COMPARISON | EEPROM_TEST_FAIL_NOT_RESTORED, 0);
  expectU("the restore tried 3 times", writeCalls, PAGES + CAT24C16_TEST_RESTORE_TRIES);

  reset();
  failWritesFrom = 3 * PAGES + 1; /* every pass fine, the restore cannot write */
  runSelfTest("passes, the restore cannot write", EEPROM_TEST_FAIL_RESTORING_BACKUP, 0);

  reset();
  failWriteAt[0] = 3 * PAGES + 2; /* the restore's 2nd page fails once */
  runSelfTest("passes, the restore fails once then works", EEPROM_TEST_PASS, 1);

  reset();
  corruptReadAt[0] = 5; /* the restore's own read-back is wrong once */
  runSelfTest("passes, the restore's read-back is wrong once", EEPROM_TEST_PASS, 1);

  reset();
  failReadAt[0] = 5; /* the restore's own read-back fails once */
  runSelfTest("passes, the restore's read-back fails once", EEPROM_TEST_PASS, 1);

  /* Shimmer3R's factory test runs it on a second part, on I2C4 */
  reset();
  ctx.handle = &handleB;
  expectedHandle = &handleB;
  runSelfTest("a second context reaches its own bus", EEPROM_TEST_PASS, 1);
}

static void test_write(void)
{
  uint8_t data[64];
  uint8_t back[64];
  uint16_t i;
  int32_t r;

  for (i = 0; i < sizeof(data); i++)
  {
    data[i] = (uint8_t) (0xA0 + i);
  }

  reset();
  testCase("write across a page and a block boundary");
  /* 250-255 in block 0, 256-271 and 272-287 in block 1, then 288-289 */
  r = CAT24C16_write(&ctx, 250, 40, data);
  expectU("result", (uint64_t) r, 0);
  expectU("one transfer a page", writeCalls, 4);
  expectX("last page addressed to block 1", lastDevAddr, CAT24C16_ADDR | 1);
  expectTrue("bytes 250-289 written", memcmp(&mem[250], data, 40) == 0);
  expectTrue("byte 249 untouched", mem[249] == original[249]);
  expectTrue("byte 290 untouched", mem[290] == original[290]);
  expectU("no page met the last one's write cycle", nacksWhileBusy, 0);
  expectFalse("returns with the part idle", isBusy());

  testCase("read straight after a write");
  r = CAT24C16_read(&ctx, 250, 40, back);
  expectU("result", (uint64_t) r, 0);
  expectTrue("reads back what was written", memcmp(back, data, 40) == 0);
  expectU("no NACK", nacksWhileBusy, 0);

  reset();
  testCase("a page that fails after its data landed");
  failWriteAfterAt[0] = 2;
  r = CAT24C16_write(&ctx, 0, 48, data);
  expectTrue("reports the failure", r != 0);
  expectU("stops at the failed page", writeCalls, 2);
  expectFalse("still returns with the part idle", isBusy());

  reset();
  testCase("the last byte");
  r = CAT24C16_write(&ctx, CAT24C16_TOTAL_SIZE - 1, 1, data);
  expectU("result", (uint64_t) r, 0);
  expectX("block 7", lastDevAddr, CAT24C16_ADDR | 7);
  expectX("byte 2047", mem[CAT24C16_TOTAL_SIZE - 1], data[0]);
}

static void test_bounds(void)
{
  uint8_t buf[CAT24C16_PAGE_SIZE];

  reset();
  testCase("out of bounds");
  memset(buf, 0, sizeof(buf));
  expectU("write, zero length", (uint64_t) CAT24C16_write(&ctx, 0, 0, buf),
      CAT24C16_OUT_OF_BOUNDS_ERROR);
  expectU("read, zero length", (uint64_t) CAT24C16_read(&ctx, 0, 0, buf),
      CAT24C16_OUT_OF_BOUNDS_ERROR);
  expectU("write past the end", (uint64_t) CAT24C16_write(&ctx, 2040, 9, buf),
      CAT24C16_OUT_OF_BOUNDS_ERROR);
  expectU("read past the end", (uint64_t) CAT24C16_read(&ctx, 2047, 2, buf),
      CAT24C16_OUT_OF_BOUNDS_ERROR);
  /* 0xFFFF + 2 wraps to 1 in the MSP430's 16-bit arithmetic */
  expectU("write at 0xFFFF", (uint64_t) CAT24C16_write(&ctx, 0xFFFF, 2, buf),
      CAT24C16_OUT_OF_BOUNDS_ERROR);
  expectU("read at 0xFFFF", (uint64_t) CAT24C16_read(&ctx, 0xFFFF, 2, buf),
      CAT24C16_OUT_OF_BOUNDS_ERROR);
  expectU("read at 2048", (uint64_t) CAT24C16_read(&ctx, CAT24C16_TOTAL_SIZE, 1, buf),
      CAT24C16_OUT_OF_BOUNDS_ERROR);
  expectU("nothing reached the bus", writeCalls + readCalls, 0);
}

static void test_read(void)
{
  uint8_t buf[32];
  uint16_t i;
  int allFF = 1;
  int32_t r;

  reset();
  testCase("read across a block boundary");
  r = CAT24C16_read(&ctx, 0xF0, sizeof(buf), buf);
  expectU("result", (uint64_t) r, 0);
  expectTrue("bytes 0xF0-0x10F", memcmp(buf, &mem[0xF0], sizeof(buf)) == 0);
  expectX("addressed to block 0", lastDevAddr, CAT24C16_ADDR);

  reset();
  testCase("a failed read reads as unprogrammed");
  failReadAt[0] = 1;
  r = CAT24C16_read(&ctx, 0, sizeof(buf), buf);
  expectTrue("reports the failure", r != 0);
  for (i = 0; i < sizeof(buf); i++)
  {
    allFF &= buf[i] == 0xFF;
  }
  expectTrue("buffer filled with 0xFF", allFF);
}

static void test_report_text(void)
{
  testCase("factory report text");
  expectStr("pass", CAT24C16_testFailReason(EEPROM_TEST_PASS), SELF_TEST_STR_EMPTY);
  expectStr("backup read",
      CAT24C16_testFailReason(EEPROM_TEST_FAIL_INITIAL_BACKUP), " - Read failed");
  expectStr("write", CAT24C16_testFailReason(EEPROM_TEST_FAIL_WRITING_BUF), " - Write failed");
  expectStr("read-back", CAT24C16_testFailReason(EEPROM_TEST_FAIL_READING_BUF),
      " - Read-back failed");
  expectStr("mismatch", CAT24C16_testFailReason(EEPROM_TEST_FAIL_BUF_COMPARISON),
      " - Read-back mismatch");
  expectStr("restore only",
      CAT24C16_testFailReason(EEPROM_TEST_FAIL_RESTORING_BACKUP), SELF_TEST_STR_EMPTY);
  expectStr("write, not restored",
      CAT24C16_testFailReason(EEPROM_TEST_FAIL_WRITING_BUF | EEPROM_TEST_FAIL_NOT_RESTORED),
      " - Write failed");
  expectStr("mismatch, not restored",
      CAT24C16_testFailReason(EEPROM_TEST_FAIL_BUF_COMPARISON | EEPROM_TEST_FAIL_NOT_RESTORED),
      " - Read-back mismatch");
  expectStr("unknown code", CAT24C16_testFailReason(6), SELF_TEST_STR_UNKNOWN);

  expectU("pass left test data", CAT24C16_testLeftTestData(EEPROM_TEST_PASS), 0);
  expectU("mismatch left test data",
      CAT24C16_testLeftTestData(EEPROM_TEST_FAIL_BUF_COMPARISON), 0);
  expectU("restore only left test data",
      CAT24C16_testLeftTestData(EEPROM_TEST_FAIL_RESTORING_BACKUP), 1);
  expectU("read-back, not restored, left test data",
      CAT24C16_testLeftTestData(EEPROM_TEST_FAIL_READING_BUF | EEPROM_TEST_FAIL_NOT_RESTORED),
      1);
}

int main(void)
{
  printf("cat24c16 host tests\n\n");
  hostTestSilenceUnused();

  test_self_test();
  test_write();
  test_bounds();
  test_read();
  test_report_text();

  return hostTestReport("cat24c16");
}

#endif /* SHIMMER_HOST_TEST */
