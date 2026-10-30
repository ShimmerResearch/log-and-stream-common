/*
 * shimmer_event_record.c
 *
 *  Created on: 30 Oct 2026
 *      Author: MarkNolan
 */

#include "shimmer_event_record.h"

#include <string.h>

/* Byte by byte rather than through a packed struct: the MSP430 and Cortex-M
 * compilers align a uint64_t differently, and the file must not depend on
 * either. */
static void putLe(uint8_t *out, uint64_t val, uint8_t len)
{
  uint8_t i;
  for (i = 0; i < len; i++)
  {
    out[i] = (uint8_t) (val >> (8U * i));
  }
}

void ShimEvtRec_encodeHeader(const ShimEvtHeader *hdr, uint8_t *out)
{
  memset(out, 0, EVT_FILE_HEADER_LEN);
  memcpy(&out[0], EVT_FILE_MAGIC, 4U);
  out[4] = EVT_FILE_FORMAT_VERSION;
  out[5] = EVT_FILE_HEADER_LEN;
  out[6] = EVT_RECORD_LEN;
  out[7] = hdr->hwId;
  putLe(&out[8], hdr->fileOpenTs, 8U);
  putLe(&out[16], hdr->rwcOffset, 8U);
  putLe(&out[24], hdr->configTime, 4U);
  putLe(&out[28], hdr->fwId, 2U);
  putLe(&out[30], hdr->fwMajor, 2U);
  out[32] = hdr->fwMinor;
  out[33] = hdr->fwInternal;
  putLe(&out[34], hdr->firstSeq, 2U);
  out[36] = hdr->openReason;
  out[37] = hdr->enables;
  /* 38-63 reserved, zero */
}

void ShimEvtRec_encodeRecord(const ShimEvtRecord *rec, uint8_t *out)
{
  out[0] = rec->type;
  out[1] = rec->flags;
  putLe(&out[2], rec->seq, 2U);
  putLe(&out[4], rec->ts, 8U);
  out[12] = rec->channel;
  out[13] = rec->state;
  putLe(&out[14], rec->dropped, 2U);
}
