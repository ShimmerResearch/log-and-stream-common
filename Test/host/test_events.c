/*
 * Host-side test for Events/shimmer_events.c, with the button module behind it
 * (Button/shimmer_button.c and shimmer_button_gesture.c).
 *
 * The event file follows the data file's lifecycle but never delays it: its
 * open, split and sync wait for TASK_EVENT_CHANNEL. Pinned here, against a fake
 * FatFs that keeps every file in memory:
 * - nothing is written unless buttonEventMarkerEnable is set;
 * - the file opens beside the data file, under events/, only when the task runs,
 *   and starts with the gesture that started logging and the state at the open;
 * - a heartbeat is written at the one-minute sync only after 60 s of nothing;
 * - at a split the old file ends with a final record, the new one carries on
 *   the sequence, and an edge from before the split stays in the old file;
 * - the close writes a final record and gives sdlog.cfg its file object back;
 * - a card error stops the event file and nothing else.
 *
 * Built against the stub platform as Shimmer3R. Run by
 * .github/workflows/host-tests.yml.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main() and FatFs
 * functions. Only the host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include "host_test.h"

#include "host_stubs.h"
#include "log_and_stream_includes.h"

/* ---- fake FatFs ---------------------------------------------------------- */

#define FAKE_FILES    4U
#define FAKE_FILE_MAX 2048U
#define FAKE_DIRS     8U

typedef struct
{
  char name[EVT_PATH_LEN];
  uint8_t data[FAKE_FILE_MAX];
  uint32_t len;
  uint8_t isOpen;
} FakeFile;

static FakeFile files[FAKE_FILES];
static uint8_t fileCount;
static FakeFile *current;
static char dirs[FAKE_DIRS][EVT_PATH_LEN];
static uint8_t dirCount;
static uint32_t syncCount;
/* Writes left before every write fails; negative for never */
static int32_t writesBeforeFailure;

static FakeFile *findFile(const char *name)
{
  uint8_t i;
  for (i = 0; i < fileCount; i++)
  {
    if (strcmp(files[i].name, name) == 0)
    {
      return &files[i];
    }
  }
  return 0;
}

FRESULT f_open(FIL *fp, const TCHAR *path, BYTE mode)
{
  FakeFile *f = findFile(path);
  (void) fp;
  (void) mode;
  if (f == 0)
  {
    if (fileCount == FAKE_FILES)
    {
      return FR_DENIED;
    }
    f = &files[fileCount++];
    strncpy(f->name, path, sizeof(f->name) - 1U);
  }
  f->len = 0; /* FA_CREATE_ALWAYS */
  f->isOpen = 1;
  current = f;
  return FR_OK;
}

FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw)
{
  (void) fp;
  *bw = 0;
  if (current == 0 || writesBeforeFailure == 0 || current->len + btw > FAKE_FILE_MAX)
  {
    return FR_DISK_ERR;
  }
  if (writesBeforeFailure > 0)
  {
    writesBeforeFailure--;
  }
  memcpy(&current->data[current->len], buff, btw);
  current->len += btw;
  *bw = btw;
  return FR_OK;
}

FRESULT f_close(FIL *fp)
{
  (void) fp;
  if (current != 0)
  {
    current->isOpen = 0;
    current = 0;
  }
  return FR_OK;
}

FRESULT f_sync(FIL *fp)
{
  (void) fp;
  syncCount++;
  return FR_OK;
}

FRESULT f_mkdir(const TCHAR *path)
{
  uint8_t i;
  for (i = 0; i < dirCount; i++)
  {
    if (strcmp(dirs[i], path) == 0)
    {
      return FR_EXIST;
    }
  }
  if (dirCount < FAKE_DIRS)
  {
    strncpy(dirs[dirCount++], path, EVT_PATH_LEN - 1U);
  }
  return FR_OK;
}

/* ---- the rest of the firmware this touches ------------------------------- */

static FIL cfgFil;
static uint8_t cfgFilLent;
static uint32_t cfgFilLends;

FIL *ShimSdCfgFile_lendFil(void)
{
  if (cfgFilLent)
  {
    return 0;
  }
  cfgFilLent = 1;
  cfgFilLends++;
  return &cfgFil;
}

void ShimSdCfgFile_returnFil(void)
{
  cfgFilLent = 0;
}

static char dataFileName[80];

const char *ShimSdDataFile_getDataFileName(void)
{
  return dataFileName;
}

uint32_t ShimConfig_configTimeGet(void)
{
  return 1793350800UL;
}

static uint8_t pinPressed;

uint8_t Board_isBtnPressed(void)
{
  return pinPressed;
}

uint8_t ShimTask_NORM_set(TaskId_t task_id)
{
  (void) task_id;
  return 0;
}

/* ---- helpers ------------------------------------------------------------- */

#define DIR_SESSION "DefaultTrial_1793350800/Shimmer_A1B2-004"

static uint64_t rtc(uint32_t atMs)
{
  return (uint64_t) atMs * 32768U / 1000U;
}

static void reset(uint8_t markers)
{
  hostStub_reset();
  memset(files, 0, sizeof(files));
  fileCount = 0;
  current = 0;
  memset(dirs, 0, sizeof(dirs));
  dirCount = 0;
  syncCount = 0;
  writesBeforeFailure = -1;
  cfgFilLent = 0;
  cfgFilLends = 0;
  pinPressed = 0;
  strcpy(dataFileName, "data/" DIR_SESSION "/000");
  ShimConfig_getStoredConfig()->buttonEventMarkerEnable = markers;
  ShimBtn_init();
  ShimEvt_init();
}

/* An edge at atMs, then the task */
static void edgeAt(uint32_t atMs, uint8_t isPress)
{
  hostStub_setTick(atMs);
  pinPressed = isPress;
  (void) ShimEvt_isrPushButtonEdge(isPress);
  ShimEvt_process();
}

static uint32_t recordCount(const FakeFile *f)
{
  return (f->len < EVT_FILE_HEADER_LEN) ? 0U : (f->len - EVT_FILE_HEADER_LEN) / EVT_RECORD_LEN;
}

static uint64_t le(const uint8_t *p, uint8_t len)
{
  uint64_t v = 0;
  while (len--)
  {
    v = (v << 8) | p[len];
  }
  return v;
}

static const uint8_t *rec(const FakeFile *f, uint32_t i)
{
  return &f->data[EVT_FILE_HEADER_LEN + i * EVT_RECORD_LEN];
}

#define REC_TYPE(r)    ((r)[0])
#define REC_FLAGS(r)   ((r)[1])
#define REC_SEQ(r)     le(&(r)[2], 2)
#define REC_TS(r)      le(&(r)[4], 8)
#define REC_CHANNEL(r) ((r)[12])
#define REC_STATE(r)   ((r)[13])
#define REC_DROPPED(r) le(&(r)[14], 2)

/* ---- tests --------------------------------------------------------------- */

static void test_nothing_without_the_config_bit(void)
{
  testCase("markers off: no event file, sdlog.cfg keeps its file object");
  reset(0);
  edgeAt(1000, 1);
  edgeAt(1100, 0);
  ShimEvt_onSessionStart(rtc(2000));
  hostStub_setTick(2010);
  ShimEvt_process();
  edgeAt(3000, 1);
  edgeAt(3100, 0);
  ShimEvt_onSessionStop();
  expectU("files", fileCount, 0);
  expectU("file object lent", cfgFilLends, 0);
  expectU("button level still tracked", ShimBtn_isPressed(), 0);
}

static void test_session(void)
{
  FakeFile *f, *g;
  const uint8_t *r;
  uint64_t openTs = rtc(10600);
  uint64_t splitTs;
  uint32_t n;

  testCase("a session: open, markers, heartbeat, split, close");
  reset(1);

  /* The double press that starts logging */
  edgeAt(10000, 1);
  edgeAt(10100, 0);
  edgeAt(10250, 1);
  edgeAt(10380, 0);

  ShimEvt_onSessionStart(openTs);
  expectU("no file until the task runs", fileCount, 0);
  expectU("file object lent", cfgFilLent, 1);

  hostStub_setTick(10620);
  ShimEvt_process();
  f = findFile("events/" DIR_SESSION "/000");
  expectTrue("event file beside the data file", f != 0);
  if (f == 0)
  {
    return;
  }
  expectU("directories made", dirCount, 3);
  expectStr("first directory", dirs[0], "events");
  expectStr("session directory", dirs[2], "events/" DIR_SESSION);

  expectTrue("magic", memcmp(f->data, "SEVT", 4) == 0);
  expectU("file open ts", le(&f->data[8], 8), openTs);
  expectU("config time", le(&f->data[24], 4), 1793350800UL);
  expectU("hardware id", f->data[7], 10);
  expectU("first seq", le(&f->data[34], 2), 0);
  expectU("open reason", f->data[36], EVT_OPEN_SESSION_START);
  expectU("enables", f->data[37], EVT_ENABLE_BUTTON);

  expectU("gesture + initial state", recordCount(f), 5);
  r = rec(f, 0);
  expectU("gesture press first", REC_TYPE(r), EVT_TYPE_EDGE);
  expectU("its state", REC_STATE(r), EVT_STATE_BUTTON);
  expectU("its time", REC_TS(r), rtc(10000));
  expectU("its channel", REC_CHANNEL(r), EVT_CHANNEL_BUTTON);
  r = rec(f, 4);
  expectU("initial state", REC_TYPE(r), EVT_TYPE_INITIAL_STATE);
  expectU("at the data file's start", REC_TS(r), openTs);
  expectU("released then", REC_STATE(r), 0);
  expectU("seq", REC_SEQ(r), 4);

  /* A marker during the session */
  edgeAt(22400, 1);
  edgeAt(22580, 0);
  expectU("marker press and release", recordCount(f), 7);

  /* The data file's sync: no heartbeat 47 s after the last record */
  hostStub_setTick(70000);
  ShimEvt_onSyncTick();
  ShimEvt_process();
  expectU("no heartbeat yet", recordCount(f), 7);
  expectU("synced", syncCount, 1);

  /* ...but one after 60 s */
  hostStub_setTick(130000);
  ShimEvt_onSyncTick();
  ShimEvt_process();
  expectU("heartbeat", recordCount(f), 8);
  r = rec(f, 7);
  expectU("heartbeat type", REC_TYPE(r), EVT_TYPE_HEARTBEAT);
  expectU("not final", REC_FLAGS(r), 0);

  /* A press just before the split, still queued when the split comes */
  hostStub_setTick(139990);
  pinPressed = 1;
  (void) ShimEvt_isrPushButtonEdge(1);
  splitTs = rtc(140000);
  strcpy(dataFileName, "data/" DIR_SESSION "/001");
  hostStub_setTick(140000);
  ShimEvt_onFileSplit(splitTs);
  ShimEvt_process();

  n = recordCount(f);
  expectU("the queued press stays in the old file, then a final record", n, 10);
  r = rec(f, n - 2U);
  expectU("press in the old file", REC_TS(r), rtc(139990));
  r = rec(f, n - 1U);
  expectU("final type", REC_TYPE(r), EVT_TYPE_HEARTBEAT);
  expectU("final flag", REC_FLAGS(r), EVT_FLAG_FINAL);
  expectU("final at the split", REC_TS(r), splitTs);
  expectFalse("old file closed", f->isOpen);

  g = findFile("events/" DIR_SESSION "/001");
  expectTrue("new file beside the new data file", g != 0);
  if (g == 0)
  {
    return;
  }
  expectU("no directories made for a split", dirCount, 3);
  expectU("open reason", g->data[36], EVT_OPEN_SPLIT);
  expectU("sequence carries on", le(&g->data[34], 2), REC_SEQ(rec(f, n - 1U)) + 1U);
  expectU("file open ts", le(&g->data[8], 8), splitTs);
  r = rec(g, 0);
  expectU("initial state", REC_TYPE(r), EVT_TYPE_INITIAL_STATE);
  expectU("pressed at the split", REC_STATE(r), EVT_STATE_BUTTON);

  edgeAt(140300, 0);
  expectU("the release goes in the new file", recordCount(g), 2);

  hostStub_setTick(150000);
  ShimEvt_onSessionStop();
  n = recordCount(g);
  r = rec(g, n - 1U);
  expectU("final at the close", REC_FLAGS(r), EVT_FLAG_FINAL);
  expectFalse("new file closed", g->isOpen);
  expectU("file object returned", cfgFilLent, 0);
  expectFalse("no error", ShimEvt_isFileError());
}

static void test_old_presses_are_not_written(void)
{
  FakeFile *f;

  testCase("presses more than 2 s before the data file opened stay out");
  reset(1);
  edgeAt(1000, 1);
  edgeAt(1100, 0);
  ShimEvt_onSessionStart(rtc(5000));
  hostStub_setTick(5010);
  ShimEvt_process();
  f = findFile("events/" DIR_SESSION "/000");
  expectTrue("file", f != 0);
  if (f != 0)
  {
    expectU("only the initial state", recordCount(f), 1);
  }
  ShimEvt_onSessionStop();
}

static void test_card_error_stops_the_event_file(void)
{
  testCase("a write error stops the event file and returns the file object");
  reset(1);
  ShimEvt_onSessionStart(rtc(1000));
  writesBeforeFailure = 1; /* the header, then nothing */
  hostStub_setTick(1010);
  ShimEvt_process();
  expectTrue("error latched", ShimEvt_isFileError());
  edgeAt(2000, 1);
  edgeAt(2100, 0);
  ShimEvt_onSessionStop();
  expectU("file object returned", cfgFilLent, 0);
  expectFalse("file closed", files[0].isOpen);
}

static void test_ring_overflow_is_counted(void)
{
  FakeFile *f;
  uint32_t i;

  testCase("edges the ring could not hold are counted in later records");
  reset(1);
  ShimEvt_onSessionStart(rtc(1000));
  hostStub_setTick(1010);
  ShimEvt_process();
  for (i = 0; i < EVT_RING_LEN + 4U; i++)
  {
    hostStub_setTick(2000U + 100U * i);
    (void) ShimEvt_isrPushButtonEdge((uint8_t) ((i & 1U) == 0U));
  }
  ShimEvt_process();
  edgeAt(10000, 1);
  f = findFile("events/" DIR_SESSION "/000");
  expectTrue("file", f != 0);
  if (f != 0)
  {
    expectU("dropped", REC_DROPPED(rec(f, recordCount(f) - 1U)), 4);
  }
  ShimEvt_onSessionStop();
}

int main(void)
{
  hostTestSilenceUnused();

  test_nothing_without_the_config_bit();
  test_session();
  test_old_presses_are_not_written();
  test_card_error_stops_the_event_file();
  test_ring_overflow_is_counted();

  return hostTestReport("test_events");
}

#endif /* SHIMMER_HOST_TEST */
