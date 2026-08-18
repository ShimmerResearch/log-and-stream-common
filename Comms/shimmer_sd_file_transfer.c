/*
 * shimmer_sd_file_transfer.c
 *
 *  Created on: 18 Aug 2026
 *      Author: MarkNolan
 */

#include "shimmer_sd_file_transfer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "log_and_stream_externs.h"
#include "log_and_stream_includes.h"

#if defined(SHIMMER3R)
#include "main.h"
#include "shimmer_definitions.h"
#include "shimmer_include.h"

#if USE_FATFS
#include "fatfs.h"
#include "ff.h"
#endif
#endif

#if defined(SHIMMER3R) && USE_FATFS

/* Budget for the LIST response (opcode + payload). Chosen so that
 * ACK + response + 2 CRC bytes stays below 253 bytes: this keeps the
 * one-shot responses within the reach of platforms whose response path
 * still uses uint8_t lengths (SHIMMER3), so the wire format ports without
 * change. Larger listings are served via startIdx paging. */
#define SD_FT_LIST_RSP_BUDGET   240
#define SD_FT_LIST_RSP_HDR_LEN  8 /* opcode, status, startIdx u16, entriesLen u16, nEntries, flags */
#define SD_FT_LIST_ENTRY_FIXED  10 /* attr, size u32, fdate u16, ftime u16, nameLen */

typedef enum
{
  SD_FT_STATE_IDLE = 0,
  SD_FT_STATE_STREAMING = 1
} sdFtState_t;

typedef struct
{
  uint8_t sessionId;
  uint8_t status;
  uint32_t nextOffset;
} sdFtPendingStatus_t;

/* One-shot command staging (copied out of the volatile BT args buffer in
 * ShimBt_processCmd(); consumed in ShimBt_sendRsp()) */
static char stagedPath[SD_FT_MAX_PATH_LEN + 1];
static uint8_t stagedPathValid;
static uint16_t stagedListStartIdx;
static uint8_t stagedListMaxEntries;

/* Windowed-read context */
static uint8_t sdFtState = SD_FT_STATE_IDLE;
static uint8_t sessionIdCounter;  /* increments on every SD_FILE_READ_COMMAND */
static uint8_t windowSessionId;   /* session id of the active/last window */
static uint16_t blockSeq;
static uint16_t blockPayloadLen = SD_FT_BLOCK_PAYLOAD_DEFAULT;
static uint32_t readOffset;
static uint32_t windowEnd;
static char readPath[SD_FT_MAX_PATH_LEN + 1];

/* Cached read handle: kept open between windows on the same file to avoid
 * FatFs cluster-chain walks on every re-open/seek */
static FIL xferFil;
static uint8_t xferFilOpen;
static char xferFilPath[SD_FT_MAX_PATH_LEN + 1];

/* Set when the TX ring had no room for the next frame; the BT TX-complete
 * path re-arms TASK_SD_FILE_TRANSFER when space frees up */
static volatile uint8_t waitingForTxSpace;

/* Set once this module has requested SD power so ShimSdFileTransfer_reset()
 * knows to release it again when the device is otherwise idle */
static uint8_t sdPowerRequested;

/* Worst case two frames are owed at once: SUPERSEDED for an in-flight
 * window plus the verdict for the READ that displaced it */
static sdFtPendingStatus_t pendingStatus[2];
static uint8_t pendingStatusCount;

static uint8_t xferBuf[SD_FT_DATA_FRAME_HEADER_LEN + SD_FT_BLOCK_PAYLOAD_MAX
    + SD_FT_FRAME_CRC_LEN];

static uint16_t u16FromLe(const uint8_t *p)
{
  return (uint16_t) p[0] | ((uint16_t) p[1] << 8);
}

static uint32_t u32FromLe(const uint8_t *p)
{
  return (uint32_t) p[0] | ((uint32_t) p[1] << 8) | ((uint32_t) p[2] << 16)
      | ((uint32_t) p[3] << 24);
}

static void u16ToLe(uint8_t *p, uint16_t v)
{
  p[0] = v & 0xFF;
  p[1] = (v >> 8) & 0xFF;
}

static void u32ToLe(uint8_t *p, uint32_t v)
{
  p[0] = v & 0xFF;
  p[1] = (v >> 8) & 0xFF;
  p[2] = (v >> 16) & 0xFF;
  p[3] = (v >> 24) & 0xFF;
}

/* Frame CRCs go through the platform layer so the main application can
 * offload them to a hardware CRC peripheral; the weak default matches
 * ShimSwCrc_calc() conventions (CRC_INIT seed, odd-length zero pad) */
static uint16_t sdFtCalcCrc(const uint8_t *buf, uint16_t len)
{
  return (uint16_t) platform_crcData16((uint8_t *) buf, len);
}

static void sdFtCloseXferFil(void)
{
  if (xferFilOpen)
  {
    f_close(&xferFil);
    xferFilOpen = 0;
    xferFilPath[0] = '\0';
  }
}

/* Gate every SD access on card ownership and the v1 idle-only policy.
 * Powers the card up on first use (mirrors the sdlog.cfg read/write idiom). */
static uint8_t sdFtAccessCheck(void)
{
  LogAndStream_checkSdInSlot();

  if (shimmerStatus.docked || shimmerStatus.usbPluggedIn
      || shimmerStatus.sdOwner != SD_OWNER_MCU || !shimmerStatus.sdInserted
      || shimmerStatus.sdBadFile)
  {
    return SD_FT_STATUS_SD_UNAVAILABLE;
  }
  if (shimmerStatus.sensing || shimmerStatus.sdLogging || shimmerStatus.btStreaming)
  {
    return SD_FT_STATUS_BUSY;
  }
  if (!sdPowerRequested)
  {
    Board_setSdPower(1);
    sdPowerRequested = 1;
  }
  return SD_FT_STATUS_OK;
}

static uint8_t sdFtCopyPathArg(const uint8_t *src, uint8_t pathLen, char *dst)
{
  if (pathLen == 0 || pathLen > SD_FT_MAX_PATH_LEN)
  {
    return 0;
  }
  memcpy(dst, src, pathLen);
  dst[pathLen] = '\0';
  return 1;
}

/* Deletion is restricted to content strictly under the data/ logging
 * directory so a host bug can never take out sdlog.cfg or /Calibration */
static uint8_t sdFtIsDeletablePath(const char *path)
{
  const char *p = path;
  if (*p == '/')
  {
    p++;
  }
  if (!((p[0] == 'd' || p[0] == 'D') && (p[1] == 'a' || p[1] == 'A')
        && (p[2] == 't' || p[2] == 'T') && (p[3] == 'a' || p[3] == 'A')
        && p[4] == '/' && p[5] != '\0'))
  {
    return 0;
  }
  if (strstr(path, "..") != NULL)
  {
    return 0;
  }
  return 1;
}

static void sdFtQueueStatus(uint8_t sid, uint8_t status, uint32_t nextOffset)
{
  uint8_t idx = pendingStatusCount;
  if (idx >= sizeof(pendingStatus) / sizeof(pendingStatus[0]))
  {
    idx = (sizeof(pendingStatus) / sizeof(pendingStatus[0])) - 1;
  }
  else
  {
    pendingStatusCount++;
  }
  pendingStatus[idx].sessionId = sid;
  pendingStatus[idx].status = status;
  pendingStatus[idx].nextOffset = nextOffset;
  ShimTask_set(TASK_SD_FILE_TRANSFER);
}

/* Returns 1 when the pending queue is fully drained */
static uint8_t sdFtEmitPendingStatusFrames(void)
{
  uint8_t frame[SD_FT_STATUS_FRAME_LEN];

  while (pendingStatusCount)
  {
    if (ShimBt_getSpaceInBtTxBuf() < (uint16_t) (SD_FT_STATUS_FRAME_LEN + 1))
    {
      waitingForTxSpace = 1;
      return 0;
    }

    frame[0] = INSTREAM_CMD_RESPONSE;
    frame[1] = SD_FILE_STATUS_RESPONSE;
    frame[2] = pendingStatus[0].sessionId;
    frame[3] = pendingStatus[0].status;
    u32ToLe(&frame[4], pendingStatus[0].nextOffset);
    u16ToLe(&frame[8], sdFtCalcCrc(frame, 8));

    if (ShimBt_writeToTxBufAndSend(frame, SD_FT_STATUS_FRAME_LEN, SHIMMER_CMD))
    {
      waitingForTxSpace = 1;
      return 0;
    }

    pendingStatusCount--;
    if (pendingStatusCount)
    {
      pendingStatus[0] = pendingStatus[1];
    }
  }
  return 1;
}

void ShimSdFileTransfer_init(void)
{
  stagedPath[0] = '\0';
  stagedPathValid = 0;
  stagedListStartIdx = 0;
  stagedListMaxEntries = SD_FT_LIST_MAX_ENTRIES;

  sdFtState = SD_FT_STATE_IDLE;
  sessionIdCounter = 0;
  windowSessionId = 0;
  blockSeq = 0;
  blockPayloadLen = SD_FT_BLOCK_PAYLOAD_DEFAULT;
  readOffset = 0;
  windowEnd = 0;
  readPath[0] = '\0';

  xferFilOpen = 0;
  xferFilPath[0] = '\0';

  waitingForTxSpace = 0;
  sdPowerRequested = 0;
  pendingStatusCount = 0;
}

void ShimSdFileTransfer_reset(void)
{
  sdFtCloseXferFil();
  sdFtState = SD_FT_STATE_IDLE;
  pendingStatusCount = 0;
  waitingForTxSpace = 0;

  if (sdPowerRequested)
  {
    sdPowerRequested = 0;
    /* Only actively power the card down when nothing else could be using
     * it; the dock/USB paths manage card power themselves */
    if (!shimmerStatus.sensing && !shimmerStatus.docked
        && !shimmerStatus.usbPluggedIn && shimmerStatus.sdOwner == SD_OWNER_MCU)
    {
      Board_setSdPower(0);
    }
  }
}

void ShimSdFileTransfer_abort(uint8_t xferStatus)
{
  sdFtCloseXferFil();

  if (xferStatus == SD_FT_XFER_SD_LOST)
  {
    /* Card is being handed to the dock/USB which power-cycles it itself */
    sdPowerRequested = 0;
  }

  if (sdFtState == SD_FT_STATE_STREAMING)
  {
    sdFtState = SD_FT_STATE_IDLE;
    sdFtQueueStatus(windowSessionId, xferStatus, readOffset);
  }
}

void ShimSdFileTransfer_txSpaceAvailableEvent(void)
{
  if (waitingForTxSpace)
  {
    waitingForTxSpace = 0;
    ShimTask_set(TASK_SD_FILE_TRANSFER);
  }
}

void ShimSdFileTransfer_stageListDir(uint8_t *argsPtr)
{
  stagedListStartIdx = u16FromLe(&argsPtr[0]);
  stagedListMaxEntries = argsPtr[2];
  if (stagedListMaxEntries == 0 || stagedListMaxEntries > SD_FT_LIST_MAX_ENTRIES)
  {
    stagedListMaxEntries = SD_FT_LIST_MAX_ENTRIES;
  }
  stagedPathValid = sdFtCopyPathArg(&argsPtr[4], argsPtr[3], stagedPath);
}

void ShimSdFileTransfer_stagePath(uint8_t *argsPtr)
{
  stagedPathValid = sdFtCopyPathArg(&argsPtr[1], argsPtr[0], stagedPath);
}

void ShimSdFileTransfer_startRead(uint8_t *argsPtr)
{
  uint32_t offset = u32FromLe(&argsPtr[0]);
  uint32_t windowLen = u32FromLe(&argsPtr[4]);
  uint16_t blockLen = u16FromLe(&argsPtr[8]);
  uint8_t pathLen = argsPtr[10];
  uint8_t access;

  sessionIdCounter++;

  if (sdFtState == SD_FT_STATE_STREAMING)
  {
    sdFtState = SD_FT_STATE_IDLE;
    sdFtQueueStatus(windowSessionId, SD_FT_XFER_SUPERSEDED, readOffset);
  }
  windowSessionId = sessionIdCounter;

  access = sdFtAccessCheck();
  if (access != SD_FT_STATUS_OK)
  {
    sdFtQueueStatus(windowSessionId,
        (access == SD_FT_STATUS_SD_UNAVAILABLE) ? SD_FT_XFER_SD_LOST :
                                                  SD_FT_XFER_DENIED,
        offset);
    return;
  }

  if (!sdFtCopyPathArg(&argsPtr[11], pathLen, readPath))
  {
    sdFtQueueStatus(windowSessionId, SD_FT_XFER_DENIED, offset);
    return;
  }

  if (blockLen == 0)
  {
    blockLen = SD_FT_BLOCK_PAYLOAD_DEFAULT;
  }
  else if (blockLen < SD_FT_BLOCK_PAYLOAD_MIN)
  {
    blockLen = SD_FT_BLOCK_PAYLOAD_MIN;
  }
  else if (blockLen > SD_FT_BLOCK_PAYLOAD_MAX)
  {
    blockLen = SD_FT_BLOCK_PAYLOAD_MAX;
  }

  readOffset = offset;
  windowEnd = (windowLen > (0xFFFFFFFFUL - offset)) ? 0xFFFFFFFFUL :
                                                      (offset + windowLen);
  blockSeq = 0;
  blockPayloadLen = blockLen;
  sdFtState = SD_FT_STATE_STREAMING;

  ShimTask_set(TASK_SD_FILE_TRANSFER);
}

void ShimSdFileTransfer_run(void)
{
  uint8_t access;
  uint8_t blocksThisPass = 0;
  FRESULT res;

  if (!shimmerStatus.btConnected)
  {
    ShimSdFileTransfer_reset();
    return;
  }

  if (!sdFtEmitPendingStatusFrames())
  {
    return; /* re-armed from the BT TX-complete path */
  }

  if (sdFtState != SD_FT_STATE_STREAMING)
  {
    return;
  }

  access = sdFtAccessCheck();
  if (access != SD_FT_STATUS_OK)
  {
    sdFtCloseXferFil();
    sdFtState = SD_FT_STATE_IDLE;
    sdFtQueueStatus(windowSessionId,
        (access == SD_FT_STATUS_SD_UNAVAILABLE) ? SD_FT_XFER_SD_LOST :
                                                  SD_FT_XFER_DENIED,
        readOffset);
    sdFtEmitPendingStatusFrames();
    return;
  }

  if (!xferFilOpen || strcmp(xferFilPath, readPath) != 0)
  {
    sdFtCloseXferFil();
    res = f_open(&xferFil, readPath, FA_READ | FA_OPEN_EXISTING);
    if (res != FR_OK)
    {
      sdFtState = SD_FT_STATE_IDLE;
      sdFtQueueStatus(windowSessionId,
          (res == FR_NO_FILE || res == FR_NO_PATH || res == FR_INVALID_NAME) ?
              SD_FT_XFER_NOT_FOUND :
              SD_FT_XFER_FS_ERROR,
          readOffset);
      sdFtEmitPendingStatusFrames();
      return;
    }
    xferFilOpen = 1;
    strcpy(xferFilPath, readPath);
  }

  if ((uint32_t) f_tell(&xferFil) != readOffset)
  {
    /* In read-only mode FatFs clamps a seek past EOF to the file size; the
     * subsequent zero-byte read then reports EOF to the host */
    res = f_lseek(&xferFil, readOffset);
    if (res != FR_OK)
    {
      sdFtCloseXferFil();
      sdFtState = SD_FT_STATE_IDLE;
      sdFtQueueStatus(windowSessionId, SD_FT_XFER_FS_ERROR, readOffset);
      sdFtEmitPendingStatusFrames();
      return;
    }
  }

  while (blocksThisPass < SD_FT_BLOCKS_PER_PASS)
  {
    uint32_t remaining;
    uint16_t toRead;
    uint16_t frameLen;
    UINT bytesRead = 0;

    if (readOffset >= windowEnd)
    {
      sdFtState = SD_FT_STATE_IDLE;
      sdFtQueueStatus(windowSessionId, SD_FT_XFER_WINDOW_COMPLETE, readOffset);
      sdFtEmitPendingStatusFrames();
      return;
    }

    remaining = windowEnd - readOffset;
    toRead = (remaining < blockPayloadLen) ? (uint16_t) remaining : blockPayloadLen;
    frameLen = SD_FT_DATA_FRAME_HEADER_LEN + toRead + SD_FT_FRAME_CRC_LEN;

    /* Keep SD_FT_TX_RESERVE free so command responses and status frames
     * always have room while a window is in flight */
    if (ShimBt_getSpaceInBtTxBuf() < (uint16_t) (frameLen + SD_FT_TX_RESERVE))
    {
      waitingForTxSpace = 1;
      return; /* re-armed from the BT TX-complete path */
    }

    /* A failed/unsent frame is re-read on the next pass: readOffset is only
     * advanced after a successful push, and the f_tell()/f_lseek() check
     * above re-synchronises the file pointer */
    res = f_read(&xferFil, &xferBuf[SD_FT_DATA_FRAME_HEADER_LEN], toRead, &bytesRead);
    if (res != FR_OK)
    {
      sdFtCloseXferFil();
      sdFtState = SD_FT_STATE_IDLE;
      sdFtQueueStatus(windowSessionId, SD_FT_XFER_FS_ERROR, readOffset);
      sdFtEmitPendingStatusFrames();
      return;
    }

    if (bytesRead > 0)
    {
      xferBuf[0] = INSTREAM_CMD_RESPONSE;
      xferBuf[1] = SD_FILE_DATA_RESPONSE;
      xferBuf[2] = windowSessionId;
      u16ToLe(&xferBuf[3], blockSeq);
      u16ToLe(&xferBuf[5], (uint16_t) bytesRead);
      u16ToLe(&xferBuf[SD_FT_DATA_FRAME_HEADER_LEN + bytesRead],
          sdFtCalcCrc(xferBuf, SD_FT_DATA_FRAME_HEADER_LEN + (uint16_t) bytesRead));

      if (ShimBt_writeToTxBufAndSend(xferBuf,
              SD_FT_DATA_FRAME_HEADER_LEN + (uint16_t) bytesRead + SD_FT_FRAME_CRC_LEN,
              SENSOR_DATA))
      {
        waitingForTxSpace = 1;
        return;
      }

      readOffset += bytesRead;
      blockSeq++;
      blocksThisPass++;
    }

    if (bytesRead < toRead)
    {
      sdFtState = SD_FT_STATE_IDLE;
      sdFtQueueStatus(windowSessionId, SD_FT_XFER_EOF, readOffset);
      sdFtEmitPendingStatusFrames();
      return;
    }
  }

  /* Per-pass budget spent; yield so lower-priority tasks are not starved
   * and re-arm to continue the window */
  ShimTask_set(TASK_SD_FILE_TRANSFER);
}

uint16_t ShimSdFileTransfer_buildListDirRsp(uint8_t *dst)
{
  uint16_t len = 0;
  uint16_t entriesLen = 0;
  uint8_t nEntries = 0;
  uint8_t flags = 0;
  uint8_t status;
  uint16_t entryIdx = 0;
  DIR dirObj;
  FILINFO fno;
  FRESULT res;

  status = stagedPathValid ? sdFtAccessCheck() : SD_FT_STATUS_BAD_ARGS;

  dst[len++] = SD_LIST_DIR_RESPONSE;
  dst[len++] = status;
  u16ToLe(&dst[len], stagedListStartIdx);
  len += 2;
  /* entriesLen, nEntries and flags are back-filled below */
  len += 4;

  if (status == SD_FT_STATUS_OK)
  {
    res = f_opendir(&dirObj, stagedPath);
    if (res != FR_OK)
    {
      dst[1] = (uint8_t) res;
    }
    else
    {
      for (;;)
      {
        uint8_t nameLen;
        uint8_t attr = 0;

        res = f_readdir(&dirObj, &fno);
        if (res != FR_OK)
        {
          dst[1] = (uint8_t) res;
          break;
        }
        if (fno.fname[0] == '\0')
        {
          break; /* end of directory */
        }
        if (fno.fname[0] == '.')
        {
          continue; /* defensively skip dot entries */
        }
        if (entryIdx++ < stagedListStartIdx)
        {
          continue;
        }
        if (nEntries >= stagedListMaxEntries)
        {
          flags |= 0x01; /* hasMore */
          break;
        }

        nameLen = (uint8_t) strlen(fno.fname);
        if (nameLen > SD_FT_LIST_NAME_MAX)
        {
          nameLen = SD_FT_LIST_NAME_MAX;
          attr |= SD_FT_ATTR_NAME_TRUNCATED;
        }
        if ((uint16_t) (SD_FT_LIST_RSP_HDR_LEN + entriesLen
                + SD_FT_LIST_ENTRY_FIXED + nameLen)
            > SD_FT_LIST_RSP_BUDGET)
        {
          flags |= 0x01; /* hasMore */
          break;
        }

        if (fno.fattrib & AM_DIR)
        {
          attr |= SD_FT_ATTR_DIR;
        }
        dst[len++] = attr;
        u32ToLe(&dst[len], (uint32_t) fno.fsize);
        len += 4;
        u16ToLe(&dst[len], (uint16_t) fno.fdate);
        len += 2;
        u16ToLe(&dst[len], (uint16_t) fno.ftime);
        len += 2;
        dst[len++] = nameLen;
        memcpy(&dst[len], fno.fname, nameLen);
        len += nameLen;

        entriesLen += SD_FT_LIST_ENTRY_FIXED + nameLen;
        nEntries++;
      }
#if _FATFS != FATFS_V_0_08B
      f_closedir(&dirObj);
#endif
    }
  }

  u16ToLe(&dst[4], entriesLen);
  dst[6] = nEntries;
  dst[7] = flags;

  return len;
}

uint16_t ShimSdFileTransfer_buildStatRsp(uint8_t *dst)
{
  uint16_t len = 0;
  uint8_t status;
  uint8_t attr = 0;
  FILINFO fno;
  FRESULT res;

  memset(&fno, 0, sizeof(fno));

  status = stagedPathValid ? sdFtAccessCheck() : SD_FT_STATUS_BAD_ARGS;
  if (status == SD_FT_STATUS_OK)
  {
    res = f_stat(stagedPath, &fno);
    if (res != FR_OK)
    {
      status = (uint8_t) res;
      memset(&fno, 0, sizeof(fno));
    }
    else if (fno.fattrib & AM_DIR)
    {
      attr |= SD_FT_ATTR_DIR;
    }
  }

  dst[len++] = SD_FILE_STAT_RESPONSE;
  dst[len++] = status;
  u32ToLe(&dst[len], (uint32_t) fno.fsize);
  len += 4;
  u16ToLe(&dst[len], (uint16_t) fno.fdate);
  len += 2;
  u16ToLe(&dst[len], (uint16_t) fno.ftime);
  len += 2;
  dst[len++] = attr;

  return len;
}

uint16_t ShimSdFileTransfer_buildFreeSpaceRsp(uint8_t *dst)
{
  uint16_t len = 0;
  uint8_t status;
  uint32_t freeKb = 0;
  uint32_t totalKb = 0;

  status = sdFtAccessCheck();
  if (status == SD_FT_STATUS_OK)
  {
    FATFS *fs;
    DWORD freeClusters = 0;
    FRESULT res = f_getfree("", &freeClusters, &fs);
    if (res != FR_OK)
    {
      status = (uint8_t) res;
    }
    else
    {
      /* 512-byte sectors -> KB = clusters * sectors-per-cluster / 2.
       * 64-bit intermediate so large exFAT cards cannot overflow. */
      uint64_t freeKb64 = ((uint64_t) freeClusters * fs->csize) / 2U;
      uint64_t totalKb64 = ((uint64_t) (fs->n_fatent - 2) * fs->csize) / 2U;
      freeKb = (freeKb64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t) freeKb64;
      totalKb = (totalKb64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t) totalKb64;
    }
  }

  dst[len++] = SD_FREE_SPACE_RESPONSE;
  dst[len++] = status;
  u32ToLe(&dst[len], freeKb);
  len += 4;
  u32ToLe(&dst[len], totalKb);
  len += 4;

  return len;
}

uint16_t ShimSdFileTransfer_buildDeleteRsp(uint8_t *dst)
{
  uint16_t len = 0;
  uint8_t status;

  status = stagedPathValid ? sdFtAccessCheck() : SD_FT_STATUS_BAD_ARGS;
  if (status == SD_FT_STATUS_OK)
  {
    if (!sdFtIsDeletablePath(stagedPath))
    {
      status = SD_FT_STATUS_BAD_ARGS;
    }
    else
    {
      FRESULT res;
      /* Release the cached read handle first: with _FS_LOCK enabled FatFs
       * refuses to unlink a file that is still open (FR_LOCKED) */
      sdFtCloseXferFil();
      res = f_unlink(stagedPath);
      status = (uint8_t) res; /* FR_OK == 0 == SD_FT_STATUS_OK */
    }
  }

  dst[len++] = SD_DELETE_RESPONSE;
  dst[len++] = status;

  return len;
}

#else /* !(defined(SHIMMER3R) && USE_FATFS) */

/* Not supported on this platform (see the SHIMMER3 assessment in DEV-948):
 * command opcodes are reserved protocol-wide but only served on SHIMMER3R.
 * Stubs keep the shared sources building everywhere. */

void ShimSdFileTransfer_init(void)
{
}

void ShimSdFileTransfer_reset(void)
{
}

void ShimSdFileTransfer_abort(uint8_t xferStatus)
{
  (void) xferStatus;
}

void ShimSdFileTransfer_run(void)
{
}

void ShimSdFileTransfer_txSpaceAvailableEvent(void)
{
}

void ShimSdFileTransfer_stageListDir(uint8_t *argsPtr)
{
  (void) argsPtr;
}

void ShimSdFileTransfer_stagePath(uint8_t *argsPtr)
{
  (void) argsPtr;
}

void ShimSdFileTransfer_startRead(uint8_t *argsPtr)
{
  (void) argsPtr;
}

uint16_t ShimSdFileTransfer_buildListDirRsp(uint8_t *dst)
{
  (void) dst;
  return 0;
}

uint16_t ShimSdFileTransfer_buildStatRsp(uint8_t *dst)
{
  (void) dst;
  return 0;
}

uint16_t ShimSdFileTransfer_buildFreeSpaceRsp(uint8_t *dst)
{
  (void) dst;
  return 0;
}

uint16_t ShimSdFileTransfer_buildDeleteRsp(uint8_t *dst)
{
  (void) dst;
  return 0;
}

#endif /* defined(SHIMMER3R) && USE_FATFS */
