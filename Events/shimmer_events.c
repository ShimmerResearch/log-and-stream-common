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
#include "Comms/shimmer_bt_reply.h"
#include "Comms/shimmer_bt_uart.h"
#include "shimmer_definitions.h"
#include "version.h"

#if EVT_BT_FRAME_TYPE != INSTREAM_CMD_RESPONSE || EVT_BT_FRAME_SUBTYPE != EVENT_RECORD_RESPONSE
#error "Events/shimmer_event_record.h and Comms/shimmer_bt_uart.h disagree on the event frame's opcodes"
#endif

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
  uint8_t channel;
  uint8_t level;
  uint8_t flags;
} ShimEvtRaw;

/* One producer at a time - interrupt handlers do not nest on Shimmer3, and the
 * ones that queue here share a priority on Shimmer3R - and one consumer, the
 * main loop. Each index is written by one side only. */
static volatile ShimEvtRaw ring[EVT_RING_LEN];
static volatile uint8_t ringHead;   /* interrupt */
static volatile uint8_t ringTail;   /* main loop */
static volatile uint16_t ringDrops; /* interrupt; taken by the main loop */

/* TTL flood control: counted by the capture interrupt, reset and counted down
 * by the blink tick */
static volatile uint8_t ttlEdgesThisTick;
static volatile uint8_t ttlThrottled;
static volatile uint8_t ttlThrottleTicks;

/* Bluetooth: what the host asked for (any context), and a frame waiting for
 * TX space (set by the task, cleared by the TX-complete interrupt) */
static volatile uint8_t btStreamWanted;
static volatile uint8_t btWaitingForTxSpace;

/* Everything below is main loop only. Records kept while no event file is
 * open, oldest first. */
static ShimEvtRecord preroll[EVT_PREROLL_LEN];
static uint8_t prerollCount;
static uint8_t ttlArmed;
static uint8_t ttlLevel;

/* The link's own state, from the opt-in until it ends */
static struct
{
  uint8_t on;
  uint8_t count; /* records in queue, oldest first */
  uint16_t seq;
  uint16_t dropped;    /* edges lost, as in the event file */
  uint16_t framesLost; /* records the queue had no room for */
  ShimEvtRecord queue[EVT_BT_QUEUE_LEN];
} btLink;

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
  uint8_t enables; /* EVT_ENABLE_*, from the configuration at session start */
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
  ttlEdgesThisTick = 0;
  ttlThrottled = 0;
  ttlThrottleTicks = 0;
  btStreamWanted = 0;
  btWaitingForTxSpace = 0;
  prerollCount = 0;
  ttlArmed = 0;
  ttlLevel = 0;
  memset(&btLink, 0, sizeof(btLink));
  memset(&evtFile, 0, sizeof(evtFile));
}

void ShimEvt_setBtStream(uint8_t enable)
{
  btStreamWanted = enable ? 1U : 0U;
  (void) ShimTask_set(TASK_EVENT_CHANNEL);
}

void ShimEvt_btTxSpaceAvailable(void)
{
  if (btWaitingForTxSpace)
  {
    btWaitingForTxSpace = 0;
    (void) ShimTask_set(TASK_EVENT_CHANNEL);
  }
}

static uint8_t isrPush(uint8_t channel, uint8_t level, uint8_t flags, uint64_t ts)
{
  uint8_t head = ringHead;

  if ((uint8_t) (head - ringTail) < EVT_RING_LEN)
  {
    ring[head % EVT_RING_LEN].ts = ts;
    ring[head % EVT_RING_LEN].channel = channel;
    ring[head % EVT_RING_LEN].level = level ? 1U : 0U;
    ring[head % EVT_RING_LEN].flags = flags;
    ringHead = (uint8_t) (head + 1U);
  }
  else if (ringDrops < 0xFFFFU)
  {
    ringDrops++;
  }
  return ShimTask_set(TASK_EVENT_CHANNEL);
}

uint8_t ShimEvt_isrPushButtonEdge(uint8_t isPress)
{
  return isrPush(EVT_CHANNEL_BUTTON, isPress, 0U, RTC_get64());
}

uint8_t ShimEvt_isrPushTtlEdge(uint8_t level, uint64_t ts, uint8_t flags)
{
  if (++ttlEdgesThisTick > EVT_TTL_MAX_EDGES_PER_TICK)
  {
    /* The blink tick counts this down and the main loop re-arms the input */
    Board_evtTtlArm(0);
    ttlThrottled = 1;
    ttlThrottleTicks = EVT_TTL_THROTTLE_TICKS;
    return 0;
  }
  return isrPush(EVT_CHANNEL_TTL, level, flags, ts);
}

uint8_t ShimEvt_tick(void)
{
  uint8_t wake = 0;

  ttlEdgesThisTick = 0;
  if (ttlThrottleTicks != 0U)
  {
    ttlThrottleTicks--;
    if (ttlThrottleTicks == 0U)
    {
      wake |= ShimTask_set(TASK_EVENT_CHANNEL);
    }
  }
  if (ShimBtn_isRecheckPending())
  {
    wake |= ShimTask_set(TASK_EVENT_CHANNEL);
  }
  return wake;
}

static uint16_t addSaturating(uint16_t a, uint16_t b)
{
  uint32_t total = (uint32_t) a + b;
  return (total > 0xFFFFUL) ? 0xFFFFU : (uint16_t) total;
}

static void addDropped(uint16_t count)
{
  evtFile.dropped = addSaturating(evtFile.dropped, count);
  btLink.dropped = addSaturating(btLink.dropped, count);
}

static uint8_t currentState(void)
{
  return (uint8_t) ((ShimBtn_isPressed() ? EVT_STATE_BUTTON : 0U)
      | (ttlLevel ? EVT_STATE_TTL : 0U));
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
  hdr.enables = evtFile.enables;
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

/* A copy for the host, with the link's own sequence number: a gap in it is a
 * record the link lost, while the dropped count is edges lost before any
 * record was made */
static void btQueue(const ShimEvtRecord *rec)
{
  uint16_t seq;

  if (!btLink.on || !shimmerStatus.btConnected)
  {
    return;
  }
  seq = btLink.seq++;
  if (btLink.count == EVT_BT_QUEUE_LEN)
  {
    btLink.framesLost = addSaturating(btLink.framesLost, 1U);
    return;
  }
  btLink.queue[btLink.count] = *rec;
  btLink.queue[btLink.count].seq = seq;
  btLink.queue[btLink.count].dropped = btLink.dropped;
  btLink.count++;
}

static void btFlush(void)
{
  uint8_t frame[EVT_BT_FRAME_LEN];
  uint16_t crc;
  uint8_t sent = 0;

  while (btLink.count != 0U)
  {
    if (!btLink.on || !shimmerStatus.btConnected)
    {
      btLink.count = 0;
      return;
    }
    if (sent == EVT_BT_FRAMES_PER_PASS)
    {
      /* The rest on the next pass, after any data packet waiting */
      (void) ShimTask_set(TASK_EVENT_CHANNEL);
      return;
    }

    /* A held command reply goes first, and the room kept for replies stays
     * free (Comms/shimmer_bt_reply.h). The flag is set before the check, so a
     * TX-complete between the two still brings the task back. */
    btWaitingForTxSpace = 1;
    if (gBtReplyPendingLen != 0U
        || ShimBt_getSpaceInBtTxBuf()
            < (uint16_t) (EVT_BT_FRAME_LEN + BT_TX_REPLY_RESERVE + 1U))
    {
      return;
    }

    ShimEvtRec_encodeBtFrame(&btLink.queue[0], frame);
    crc = (uint16_t) platform_crcData16(frame, EVT_BT_FRAME_CRC_OFFSET);
    frame[EVT_BT_FRAME_CRC_OFFSET] = (uint8_t) crc;
    frame[EVT_BT_FRAME_CRC_OFFSET + 1U] = (uint8_t) (crc >> 8);
    /* Written as a command reply is: SHIMMER_CMD, not SENSOR_DATA, since on
     * Shimmer3's RN4678 paths data is held back to be grouped, which would
     * strand an event while nothing streams. Not ShimBt_writeToTxBufAndSend()
     * directly: on Shimmer3 that is a function pointer declared in RN4X.h. */
    if (ShimBt_writeReplyToTxBuf(frame, EVT_BT_FRAME_LEN))
    {
      return;
    }
    btWaitingForTxSpace = 0;

    btLink.count--;
    memmove(&btLink.queue[0], &btLink.queue[1], sizeof(btLink.queue[0]) * btLink.count);
    sent++;
  }
}

static void emit(ShimEvtRecord *rec)
{
  btQueue(rec);
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

static void recordTtl(uint8_t type, uint8_t flags, uint64_t ts, uint8_t level)
{
  ShimEvtRecord rec;

  rec.type = type;
  rec.flags = flags;
  rec.seq = 0;
  rec.ts = ts;
  rec.channel = EVT_CHANNEL_TTL;
  rec.state = (uint8_t) ((ShimBtn_isPressed() ? EVT_STATE_BUTTON : 0U)
      | (level ? EVT_STATE_TTL : 0U));
  rec.dropped = 0;
  emit(&rec);
}

static void processTtlEdge(uint64_t ts, uint8_t level, uint8_t flags)
{
  if (flags & EVT_RAW_EDGES_LOST)
  {
    /* The timer overwrote an edge before reading this one: say so where it
     * happened, at the level last known */
    addDropped(1);
    recordTtl(EVT_TYPE_OVERFLOW, 0U, ts, ttlLevel);
  }
  ttlLevel = level;
  recordTtl(EVT_TYPE_EDGE, (uint8_t) (flags & (uint8_t) ~EVT_RAW_EDGES_LOST), ts, level);
}

/* After a throttle second, take the input back: what it did in that second is
 * lost, so an overflow record marks the gap and gives its level now */
static void ttlRearmIfDue(void)
{
  if (!ttlThrottled || ttlThrottleTicks != 0U)
  {
    return;
  }
  ttlThrottled = 0;
  if (!ttlArmed)
  {
    return;
  }
  addDropped(1);
  ttlLevel = Board_evtTtlLevel();
  recordTtl(EVT_TYPE_OVERFLOW, 0U, RTC_get64(), ttlLevel);
  Board_evtTtlArm(1);
}

static void ttlArm(uint8_t enable)
{
  if (enable)
  {
    ttlThrottled = 0;
    ttlThrottleTicks = 0;
    ttlLevel = Board_evtTtlLevel();
    ttlArmed = 1;
    Board_evtTtlArm(1);
  }
  else if (ttlArmed)
  {
    /* ttlLevel stays as last seen, for the records still to be written */
    ttlArmed = 0;
    ttlThrottled = 0;
    ttlThrottleTicks = 0;
    Board_evtTtlArm(0);
  }
}

/* Armed while a session records the TTL input, or while a host has opted in
 * to the stream and the input is enabled */
static void ttlUpdateArming(void)
{
  uint8_t want = ShimConfig_isTtlInputEnabled()
      && ((evtFile.sessionActive && !evtFile.error && (evtFile.enables & EVT_ENABLE_TTL))
          || btLink.on);

  if (want && !ttlArmed)
  {
    ttlArm(1);
  }
  else if (!want && ttlArmed)
  {
    ttlArm(0);
    if (!evtFile.sessionActive)
    {
      ttlLevel = 0;
    }
  }
}

/* The host's opt-in, applied in the task: a fresh link, sequence from 0, and
 * the inputs' state as its first record so the host need not wait for an edge */
static void btApplyWanted(void)
{
  uint8_t want = btStreamWanted;
  ShimEvtRecord rec;

  if (want == btLink.on)
  {
    return;
  }
  memset(&btLink, 0, sizeof(btLink));
  btLink.on = want;
  btWaitingForTxSpace = 0;
  ttlUpdateArming();
  if (want)
  {
    memset(&rec, 0, sizeof(rec));
    rec.type = EVT_TYPE_INITIAL_STATE;
    rec.ts = RTC_get64();
    rec.channel = EVT_CHANNEL_NONE;
    rec.state = currentState();
    btQueue(&rec);
  }
}

void ShimEvt_process(void)
{
  uint16_t drops;

  btApplyWanted();

  /* Edges first, so the file work below sees every record before it */
  while (ringTail != ringHead)
  {
    uint8_t i = (uint8_t) (ringTail % EVT_RING_LEN);
    uint64_t ts = ring[i].ts;
    uint8_t channel = ring[i].channel;
    uint8_t level = ring[i].level;
    uint8_t flags = ring[i].flags;

    ringTail = (uint8_t) (ringTail + 1U);
    if (channel == EVT_CHANNEL_TTL)
    {
      processTtlEdge(ts, level, flags);
    }
    else
    {
      ShimBtn_processEdge(ts, level);
    }
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

  ttlRearmIfDue();
  ShimBtn_recheckIfDue(RTC_get64());

#if USE_FATFS
  fileService();
#endif
  btFlush();
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
  evtFile.enables
      = (uint8_t) ((ShimConfig_doesButtonRecordPresses() ? EVT_ENABLE_BUTTON : 0U)
          | (ShimConfig_isTtlInputEnabled() ? EVT_ENABLE_TTL : 0U));
  if (evtFile.enables == 0U)
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
  /* Armed now, so an edge from the data file's first moment is captured; its
   * record waits with the button's for the file to open */
  ttlUpdateArming();
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

  /* No TTL edges for the file after this, unless a host is still taking them;
   * then the ones still queued, and an open or split still waiting for the
   * task */
  if (!btLink.on)
  {
    ttlArm(0);
  }
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
  if (!ttlArmed)
  {
    ttlLevel = 0;
  }
  ttlUpdateArming();
}

uint8_t ShimEvt_isFileError(void)
{
  return evtFile.error;
}

static void putLe16(uint8_t *out, uint16_t v)
{
  out[0] = (uint8_t) v;
  out[1] = (uint8_t) (v >> 8);
}

uint8_t ShimEvt_buildStatus(uint8_t *out)
{
  out[0] = (uint8_t) ((ShimConfig_doesButtonRecordPresses() ? EVT_ENABLE_BUTTON : 0U)
      | (ShimConfig_isTtlInputEnabled() ? EVT_ENABLE_TTL : 0U));
  /* The opt-in as asked for: the task may not have applied it yet */
  out[1] = (uint8_t) ((btStreamWanted ? 0x01U : 0U)
      | (evtFile.isOpen ? 0x02U : 0U) | (evtFile.error ? 0x04U : 0U)
      | (ttlArmed ? 0x08U : 0U) | (ttlThrottled ? 0x10U : 0U));
  out[2] = currentState();
  putLe16(&out[3], btLink.seq);
  putLe16(&out[5], evtFile.dropped);
  putLe16(&out[7], btLink.framesLost);
  return EVT_STATUS_LEN;
}
