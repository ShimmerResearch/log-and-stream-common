/*
 * shimmer_audio_file.c
 *
 * See shimmer_audio_file.h. Offsets are into the file, little-endian.
 *
 * The WAV:
 *    0  "RIFF", u32 file size - 8, "WAVE"
 *   12  "fmt ", u32 16, PCM, mono, 16000 Hz, 32000 B/s, align 2, 16 bits
 *   36  "shim", u32 460
 *   44  "SAUD"
 *   48  u8 version, u8 hwId, u8 mode, u8 openReason
 *   52  u16 fileIndex, u16 reserved
 *   56  u64 dataFileTs
 *   64  u32 configTime
 *   68  u16 fwId, u16 fwMajor, u8 fwMinor, u8 fwInternal
 *   74  u8[6] mac
 *   80  u64 firstSampleTicks
 *   88  u32 decimation, i32 gain
 *       the closing summary:
 *   96  u8 closed, u8 fault, u16 reserved
 *  100  u32 samples
 *  104  u64 lastSampleTicks
 *  112  u32 silenceSamples
 *  116  u16 escapeDrains, u16 writeErrors
 *  120  u32 adfErrorEvents, u32 adfSaturation, u32 adfOverflow
 *  132  zero to 503
 *  504  "data", u32 sample bytes
 *  512  the samples
 *
 * The fingerprint file (.AFP):
 *    0  "SAFP"
 *    4  u16 chunk header bytes 16, u16 record bytes 4
 *    8  u32 chunk samples 80000, u32 chunk hop samples 64000
 *   16  u16 frame hop samples 256, u16 sample rate 16000
 *   20  zero to 43
 *   44  "SAUD" and the identity, as the WAV's to 95; 80 is the first chunk's
 *       Shimmer clock
 *       the closing summary:
 *   96  u8 closed, u8 fault, u16 reserved
 *  100  u32 chunks
 *  104  u64 the last chunk's Shimmer clock
 *  112  u32 records
 *  116  u16 escapeDrains, u16 writeErrors
 *  120  u32 adfErrorEvents, u32 adfSaturation, u32 adfOverflow
 *  132  u32 chunksSkipped
 *  136  zero to 511
 *  512  chunks: a 16-byte chunk header, then its records
 *
 * A chunk header: u16 mark 0x4843, u16 records, u32 chunk, u64 start ticks.
 */

#include <string.h>

#include "shimmer_audio_file.h"

#define SHIM_CHUNK_OFFSET 36U
#define DATA_CHUNK_OFFSET (AUDIO_FILE_HEADER_LEN - 8U)

static void put16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t) v;
  p[1] = (uint8_t) (v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
  put16(p, (uint16_t) v);
  put16(p + 2, (uint16_t) (v >> 16));
}

static void put64(uint8_t *p, uint64_t v)
{
  put32(p, (uint32_t) v);
  put32(p + 4, (uint32_t) (v >> 32));
}

/* Bytes 44-95, the same in both files */
static void packIdentity(uint8_t *out, const ShimAudioFileInfo *info)
{
  memcpy(out + 44, "SAUD", 4);
  out[48] = AUDIO_FILE_FORMAT_VERSION;
  out[49] = info->hwId;
  out[50] = info->mode;
  out[51] = info->openReason;
  put16(out + 52, info->fileIndex);
  put64(out + 56, info->dataFileTs);
  put32(out + 64, info->configTime);
  put16(out + 68, info->fwId);
  put16(out + 70, info->fwMajor);
  out[72] = info->fwMinor;
  out[73] = info->fwInternal;
  memcpy(out + 74, info->mac, 6);
  put64(out + 80, info->firstSampleTicks);
  put32(out + 88, info->decimation);
  put32(out + 92, (uint32_t) info->gain);
}

/* The session's counters, at 96-97 and 116-131 in both files */
static void packSessionSummary(uint8_t *out, const ShimAudioFileInfo *info)
{
  out[96] = info->closed;
  out[97] = info->fault;
  put16(out + 116, info->escapeDrains);
  put16(out + 118, info->writeErrors);
  put32(out + 120, info->adfErrorEvents);
  put32(out + 124, info->adfSaturation);
  put32(out + 128, info->adfOverflow);
}

void ShimAudioFile_packHeader(uint8_t *out, const ShimAudioFileInfo *info)
{
  uint32_t dataBytes = info->samples * 2U;

  memset(out, 0, AUDIO_FILE_HEADER_LEN);

  memcpy(out, "RIFF", 4);
  put32(out + 4, AUDIO_FILE_HEADER_LEN - 8U + dataBytes);
  memcpy(out + 8, "WAVE", 4);

  memcpy(out + 12, "fmt ", 4);
  put32(out + 16, 16U);
  put16(out + 20, 1U); /* PCM */
  put16(out + 22, 1U); /* mono */
  put32(out + 24, AUDIO_FILE_RATE_HZ);
  put32(out + 28, AUDIO_FILE_RATE_HZ * 2U);
  put16(out + 32, 2U);
  put16(out + 34, 16U);

  memcpy(out + SHIM_CHUNK_OFFSET, "shim", 4);
  put32(out + SHIM_CHUNK_OFFSET + 4, DATA_CHUNK_OFFSET - SHIM_CHUNK_OFFSET - 8U);
  packIdentity(out, info);

  packSessionSummary(out, info);
  put32(out + 100, info->samples);
  put64(out + 104, info->lastSampleTicks);
  put32(out + 112, info->silenceSamples);

  memcpy(out + DATA_CHUNK_OFFSET, "data", 4);
  put32(out + DATA_CHUNK_OFFSET + 4, dataBytes);
}

void ShimAudioFile_packFpHeader(uint8_t *out, const ShimAudioFileInfo *info, const ShimAudioFpSummary *fp)
{
  memset(out, 0, AUDIO_FP_FILE_HEADER_LEN);

  memcpy(out, "SAFP", 4);
  put16(out + 4, AUDIO_FP_CHUNK_HEADER_LEN);
  put16(out + 6, AUDIO_FP_RECORD_LEN);
  put32(out + 8, AUDIO_FP_CHUNK_SAMPLES);
  put32(out + 12, AUDIO_FP_CHUNK_HOP_SAMPLES);
  put16(out + 16, AUDIO_FP_FRAME_HOP_SAMPLES);
  put16(out + 18, AUDIO_FILE_RATE_HZ);
  packIdentity(out, info);

  packSessionSummary(out, info);
  put32(out + 100, fp->chunks);
  put64(out + 104, fp->lastChunkTicks);
  put32(out + 112, fp->records);
  put32(out + 132, fp->chunksSkipped);
}

void ShimAudioFile_packFpChunkHeader(uint8_t *out, uint32_t chunk, uint64_t startTicks, uint16_t records)
{
  put16(out, AUDIO_FP_CHUNK_MARK);
  put16(out + 2, records);
  put32(out + 4, chunk);
  put64(out + 8, startTicks);
}

uint8_t ShimAudioFile_packFpRecord(uint8_t *out, uint16_t t1, uint8_t f1, uint8_t f2, uint8_t dt)
{
  uint32_t df = (uint32_t) ((int) f2 - (int) f1) & 0x3FU;

  if (t1 > 0x1FFU || dt > 0x3FU)
  {
    return 0;
  }
  put32(out, (uint32_t) t1 | ((uint32_t) dt << 9) | (df << 15) | ((uint32_t) (f1 & 0x7FU) << 21));
  return 1;
}
