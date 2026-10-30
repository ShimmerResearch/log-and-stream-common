/*
 * shimmer_events.c
 *
 *  Created on: 30 Oct 2026
 *      Author: MarkNolan
 */

#include "shimmer_events.h"

#include <string.h>

#include "log_and_stream_includes.h"
/* DEVICE_VER, FW_IDENTIFIER and the version, for the file header - included
 * directly, as SDCard/shimmer_sd_header.c does */
#include "shimmer_definitions.h"
#include "version.h"

#if defined(__MSP430__)
#define EVT_IRQ_SAVE()                            \
  uint16_t evtIrqState = __get_interrupt_state(); \
  __disable_interrupt()
#define EVT_IRQ_RESTORE() __set_interrupt_state(evtIrqState)
#elif defined(__arm__) || defined(__ARM_ARCH)
#define EVT_IRQ_SAVE()                    \
  uint32_t evtIrqState = __get_PRIMASK(); \
  __disable_irq()
#define EVT_IRQ_RESTORE() __set_PRIMASK(evtIrqState)
#else
#define EVT_IRQ_SAVE()
#define EVT_IRQ_RESTORE()
#endif

typedef struct
{
  uint64_t ts;
  uint8_t isPress;
} ShimEvtRaw;

/* One producer at a time - interrupt handlers do not nest on Shimmer3, and the
 * ones that queue here share a priority on Shimmer3R - and one consumer, the
 * main loop. Each index is written by one side only. */
static volatile ShimEvtRaw ring[EVT_RING_LEN];
static volatile uint8_t ringHead;   /* interrupt */
static volatile uint8_t ringTail;   /* main loop */
static volatile uint16_t ringDrops; /* interrupt; taken by the main loop */

/* Everything below is main loop only. Records kept while no event file is
 * open, oldest first. */
static ShimEvtRecord preroll[EVT_PREROLL_LEN];
static uint8_t prerollCount;

static struct
{
  uint8_t sessionActive; /* recording into an event file this session */
  uint8_t isOpen;
  uint8_t openPending;
  uint8_t openReason;
  uint8_t splitPending;
  uint8_t syncPending;
  uint8_t error;
  uint8_t filLent;
  uint16_t seq;
  uint16_t dropped;
  uint64_t openTs;
  uint64_t splitTs;
  uint64_t lastWriteTs;
#if USE_FATFS
  FIL *fil;
#endif
  char path[EVT_PATH_LEN];
} evtFile;

void ShimEvt_init(void)
{
  ringHead = 0;
  ringTail = 0;
  ringDrops = 0;
  prerollCount = 0;
  memset(&evtFile, 0, sizeof(evtFile));
}

uint8_t ShimEvt_isrPushButtonEdge(uint8_t isPress)
{
  uint8_t head = ringHead;

  if ((uint8_t) (head - ringTail) < EVT_RING_LEN)
  {
    ring[head % EVT_RING_LEN].ts = RTC_get64();
    ring[head % EVT_RING_LEN].isPress = isPress ? 1U : 0U;
    ringHead = (uint8_t) (head + 1U);
  }
  else if (ringDrops < 0xFFFFU)
  {
    ringDrops++;
  }
  return ShimTask_set(TASK_EVENT_CHANNEL);
}

uint8_t ShimEvt_tick(void)
{
  if (ShimBtn_isRecheckPending())
  {
    return ShimTask_set(TASK_EVENT_CHANNEL);
  }
  return 0;
}

static void addDropped(uint16_t count)
{
  uint32_t total = (uint32_t) evtFile.dropped + count;
  evtFile.dropped = (total > 0xFFFFUL) ? 0xFFFFU : (uint16_t) total;
}

static uint8_t currentState(void)
{
  return ShimBtn_isPressed() ? EVT_STATE_BUTTON : 0U;
}

static uint8_t channelStateBit(uint8_t channel)
{
  return (channel == EVT_CHANNEL_BUTTON) ? EVT_STATE_BUTTON :
      (channel == EVT_CHANNEL_TTL)       ? EVT_STATE_TTL :
                                           0U;
}

/* The state just before a record: an edge changed its own channel's bit */
static uint8_t stateBefore(const ShimEvtRecord *rec)
{
  if (rec->type == EVT_TYPE_EDGE)
  {
    return rec->state ^ channelStateBit(rec->channel);
  }
  return rec->state;
}

static void keep(const ShimEvtRecord *rec)
{
  if (prerollCount == EVT_PREROLL_LEN)
  {
    /* The oldest goes. While a session waits for its file, that is a record
     * lost; otherwise it was only kept in case logging started. */
    memmove(&preroll[0], &preroll[1], sizeof(preroll[0]) * (EVT_PREROLL_LEN - 1U));
    prerollCount--;
    if (evtFile.sessionActive)
    {
      addDropped(1);
    }
  }
  preroll[prerollCount++] = *rec;
}

#if USE_FATFS
/* Shimmer3 gives the card to the dock when docked: never write it then */
static uint8_t isCardWritable(void)
{
#if defined(SHIMMER3)
  return !shimmerStatus.docked;
#else
  return 1;
#endif
}

static void fail(void)
{
  evtFile.error = 1;
  if (evtFile.isOpen)
  {
    evtFile.isOpen = 0;
    (void) f_close(evtFile.fil);
  }
}

static void putRecord(ShimEvtRecord *rec)
{
  uint8_t buf[EVT_RECORD_LEN];
  UINT bw = 0;

  if (!evtFile.isOpen)
  {
    return;
  }
  rec->seq = evtFile.seq++;
  rec->dropped = evtFile.dropped;
  ShimEvtRec_encodeRecord(rec, buf);
  if (f_write(evtFile.fil, buf, EVT_RECORD_LEN, &bw) != FR_OK || bw != EVT_RECORD_LEN)
  {
    fail();
    return;
  }
  evtFile.lastWriteTs = rec->ts;
}

static void putStateRecord(uint8_t type, uint8_t flags, uint64_t ts, uint8_t state)
{
  ShimEvtRecord rec;
  rec.type = type;
  rec.flags = flags;
  rec.seq = 0;
  rec.ts = ts;
  rec.channel = EVT_CHANNEL_NONE;
  rec.state = state;
  rec.dropped = 0;
  putRecord(&rec);
}

/* events/<experiment>/<session>/<NNN> beside data/<experiment>/<session>/<NNN> */
static uint8_t makePath(void)
{
  const char *data = ShimSdDataFile_getDataFileName();

  if (strncmp(data, "data/", 5U) != 0 || strlen(data) - 5U + 7U >= EVT_PATH_LEN)
  {
    return 0;
  }
  strcpy(evtFile.path, "events/");
  strcat(evtFile.path, data + 5U);
  return 1;
}

/* Every directory in the path: events, the experiment's, the session's */
static uint8_t makeDirs(void)
{
  char *start = evtFile.path;
  char *slash;
  FRESULT res;

  while ((slash = strchr(start, '/')) != 0)
  {
    *slash = '\0';
    res = f_mkdir(evtFile.path);
    *slash = '/';
    if (res != FR_OK && res != FR_EXIST)
    {
      return 0;
    }
    start = slash + 1;
  }
  return 1;
}

static void closeFile(void)
{
  if (!evtFile.isOpen)
  {
    return;
  }
  evtFile.isOpen = 0;
  if (f_close(evtFile.fil) != FR_OK)
  {
    evtFile.error = 1;
    return;
  }
  ShimSd_setFileTimestamp(evtFile.path);
}

/* next: the record that triggered a split, which belongs in the new file */
static void openFile(const ShimEvtRecord *next)
{
  ShimEvtHeader hdr;
  uint8_t buf[EVT_FILE_HEADER_LEN];
  UINT bw = 0;
  uint8_t i;
  uint8_t state = currentState();

  evtFile.openPending = 0;
  if (!isCardWritable() || !makePath()
      || (evtFile.openReason == EVT_OPEN_SESSION_START && !makeDirs())
      || f_open(evtFile.fil, evtFile.path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
  {
    fail();
    return;
  }
  evtFile.isOpen = 1;

  hdr.hwId = DEVICE_VER;
  hdr.fileOpenTs = evtFile.openTs;
#if defined(SHIMMER3)
  hdr.rwcOffset = *RTC_getRwcTimeDiffPtr();
#else
  hdr.rwcOffset = 0;
#endif
  hdr.configTime = ShimConfig_configTimeGet();
  hdr.fwId = FW_IDENTIFIER;
  hdr.fwMajor = FW_VERSION_MAJOR;
  hdr.fwMinor = FW_VERSION_MINOR;
  hdr.fwInternal = FW_VERSION_PATCH;
  hdr.firstSeq = evtFile.seq;
  hdr.openReason = evtFile.openReason;
  hdr.enables = EVT_ENABLE_BUTTON;
  ShimEvtRec_encodeHeader(&hdr, buf);
  if (f_write(evtFile.fil, buf, EVT_FILE_HEADER_LEN, &bw) != FR_OK || bw != EVT_FILE_HEADER_LEN)
  {
    fail();
    return;
  }
  evtFile.lastWriteTs = evtFile.openTs;

  /* The state at the moment the data file opened: just before the first
   * record at or after it, or now if there is none */
  for (i = 0; i < prerollCount; i++)
  {
    if (preroll[i].ts >= evtFile.openTs)
    {
      state = stateBefore(&preroll[i]);
      break;
    }
  }
  if (i == prerollCount && next != 0)
  {
    state = stateBefore(next);
  }

  /* Kept records from just before the data file opened - the gesture that
   * started logging - then the state at the open, then the rest */
  for (i = 0; i < prerollCount; i++)
  {
    if (preroll[i].ts < evtFile.openTs && evtFile.openTs - preroll[i].ts <= EVT_PREROLL_TICKS)
    {
      putRecord(&preroll[i]);
    }
  }
  putStateRecord(EVT_TYPE_INITIAL_STATE, 0U, evtFile.openTs, state);
  for (i = 0; i < prerollCount; i++)
  {
    if (preroll[i].ts >= evtFile.openTs)
    {
      putRecord(&preroll[i]);
    }
  }
  prerollCount = 0;
}

static void doSplit(const ShimEvtRecord *next)
{
  evtFile.splitPending = 0;
  if (!evtFile.isOpen)
  {
    return;
  }
  putStateRecord(EVT_TYPE_HEARTBEAT, EVT_FLAG_FINAL, evtFile.splitTs,
      (next != 0) ? stateBefore(next) : currentState());
  closeFile();
  evtFile.openTs = evtFile.splitTs;
  evtFile.openReason = EVT_OPEN_SPLIT;
  openFile(next);
}

static void fileService(void)
{
  uint64_t now;

  if (!evtFile.sessionActive || evtFile.error)
  {
    return;
  }
  if (evtFile.openPending)
  {
    openFile(0);
  }
  if (evtFile.splitPending)
  {
    doSplit(0);
  }
  if (evtFile.syncPending)
  {
    evtFile.syncPending = 0;
    if (evtFile.isOpen)
    {
      now = RTC_get64();
      if (now - evtFile.lastWriteTs >= EVT_HEARTBEAT_TICKS)
      {
        putStateRecord(EVT_TYPE_HEARTBEAT, 0U, now, currentState());
      }
      if (evtFile.isOpen && f_sync(evtFile.fil) != FR_OK)
      {
        fail();
      }
    }
  }
}
#endif /* USE_FATFS */

static void emit(ShimEvtRecord *rec)
{
#if USE_FATFS
  if (evtFile.sessionActive && !evtFile.openPending)
  {
    if (evtFile.splitPending && rec->ts >= evtFile.splitTs)
    {
      doSplit(rec);
    }
    putRecord(rec);
    return;
  }
#endif
  keep(rec);
}

void ShimEvt_recordButtonEdge(uint64_t ts, uint8_t level, uint8_t flags)
{
  ShimEvtRecord rec;

  rec.type = EVT_TYPE_EDGE;
  rec.flags = flags;
  rec.seq = 0;
  rec.ts = ts;
  rec.channel = EVT_CHANNEL_BUTTON;
  /* This edge's level, not the button's latest: two edges can arrive at once */
  rec.state = (uint8_t) ((currentState() & (uint8_t) ~EVT_STATE_BUTTON)
      | (level ? EVT_STATE_BUTTON : 0U));
  rec.dropped = 0;
  emit(&rec);
}

void ShimEvt_process(void)
{
  uint16_t drops;

  /* Edges first, so the file work below sees every record before it */
  while (ringTail != ringHead)
  {
    uint8_t i = (uint8_t) (ringTail % EVT_RING_LEN);
    uint64_t ts = ring[i].ts;
    uint8_t isPress = ring[i].isPress;

    ringTail = (uint8_t) (ringTail + 1U);
    ShimBtn_processEdge(ts, isPress);
  }

  {
    EVT_IRQ_SAVE();
    drops = ringDrops;
    ringDrops = 0;
    EVT_IRQ_RESTORE();
  }
  if (drops != 0U)
  {
    addDropped(drops);
  }

  ShimBtn_recheckIfDue(RTC_get64());

#if USE_FATFS
  fileService();
#endif
}

void ShimEvt_onSessionStart(uint64_t fileOpenTs)
{
#if USE_FATFS
  evtFile.sessionActive = 0;
  evtFile.isOpen = 0;
  evtFile.splitPending = 0;
  evtFile.syncPending = 0;
  evtFile.error = 0;
  evtFile.seq = 0;
  evtFile.dropped = 0;
  if (!ShimConfig_doesButtonRecordPresses())
  {
    prerollCount = 0;
    return;
  }

  evtFile.sessionActive = 1;
  evtFile.fil = ShimSdCfgFile_lendFil();
  evtFile.filLent = (evtFile.fil != 0);
  if (!evtFile.filLent)
  {
    evtFile.error = 1;
    return;
  }
  evtFile.openPending = 1;
  evtFile.openReason = EVT_OPEN_SESSION_START;
  evtFile.openTs = fileOpenTs;
  (void) ShimTask_set(TASK_EVENT_CHANNEL);
#else
  (void) fileOpenTs;
#endif
}

void ShimEvt_onFileSplit(uint64_t fileOpenTs)
{
  if (!evtFile.sessionActive)
  {
    return;
  }
  if (evtFile.openPending)
  {
    /* The first file was never opened: open beside the new one instead */
    evtFile.openTs = fileOpenTs;
  }
  else
  {
    evtFile.splitPending = 1;
    evtFile.splitTs = fileOpenTs;
  }
  (void) ShimTask_set(TASK_EVENT_CHANNEL);
}

void ShimEvt_onSyncTick(void)
{
  if (!evtFile.sessionActive)
  {
    return;
  }
  evtFile.syncPending = 1;
  (void) ShimTask_set(TASK_EVENT_CHANNEL);
}

void ShimEvt_onSessionStop(void)
{
  if (!evtFile.sessionActive)
  {
    return;
  }

  /* Edges still queued, and an open or split still waiting for the task */
  ShimEvt_process();

#if USE_FATFS
  if (evtFile.isOpen && isCardWritable())
  {
    putStateRecord(EVT_TYPE_HEARTBEAT, EVT_FLAG_FINAL, RTC_get64(), currentState());
  }
  closeFile();
  if (evtFile.filLent)
  {
    ShimSdCfgFile_returnFil();
    evtFile.filLent = 0;
  }
  evtFile.fil = 0;
#endif
  evtFile.sessionActive = 0;
  evtFile.openPending = 0;
  evtFile.splitPending = 0;
  evtFile.syncPending = 0;
  prerollCount = 0;
}

uint8_t ShimEvt_isFileError(void)
{
  return evtFile.error;
}
