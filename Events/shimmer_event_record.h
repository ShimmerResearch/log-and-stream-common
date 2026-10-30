/*
 * shimmer_event_record.h
 *
 * The layout of an event file: a 64-byte header followed by 16-byte records,
 * all little-endian (docs/SHIMMER3_EVENT_CHANNEL.md). The header and records
 * are multiples of 16 bytes, so no record straddles a 512-byte sector.
 *
 * Pure encoding with no platform dependencies, so the host test suite can pin
 * the byte layout.
 */

#ifndef SHIMMER_EVENT_RECORD_H_
#define SHIMMER_EVENT_RECORD_H_

#include <stdint.h>

#define EVT_FILE_MAGIC          "SEVT"
#define EVT_FILE_FORMAT_VERSION 1U
#define EVT_FILE_HEADER_LEN     64U
#define EVT_RECORD_LEN          16U

/* Record types */
#define EVT_TYPE_EDGE           0x01U
#define EVT_TYPE_INITIAL_STATE  0x02U
#define EVT_TYPE_HEARTBEAT      0x03U
#define EVT_TYPE_OVERFLOW       0x04U

/* Record flags */
#define EVT_FLAG_HW_CAPTURE     0x01U /* time latched by a timer at the edge */
#define EVT_FLAG_SYNTHESIZED    0x02U /* an edge inferred, not seen */
#define EVT_FLAG_LATE           0x04U /* the edge's time is less certain */
#define EVT_FLAG_STATE_STALE    0x08U
#define EVT_FLAG_FINAL          0x80U /* the last record of a closed file */

/* Channels */
#define EVT_CHANNEL_BUTTON      0U
#define EVT_CHANNEL_TTL         1U
#define EVT_CHANNEL_NONE        0xFFU

/* State bits: the level of every input when the record was made */
#define EVT_STATE_BUTTON        0x01U
#define EVT_STATE_TTL           0x02U

/* Why a file was opened */
#define EVT_OPEN_SESSION_START  1U
#define EVT_OPEN_SPLIT          2U

/* Header enables: which inputs this file records */
#define EVT_ENABLE_BUTTON       0x01U
#define EVT_ENABLE_TTL          0x02U

typedef struct
{
  uint8_t hwId;
  /* The data file's initial timestamp: pairs the two files */
  uint64_t fileOpenTs;
  /* Shimmer3's offset from boot time to real-world time; 0 on Shimmer3R */
  uint64_t rwcOffset;
  uint32_t configTime;
  uint16_t fwId;
  uint16_t fwMajor;
  uint8_t fwMinor;
  uint8_t fwInternal;
  uint16_t firstSeq;
  uint8_t openReason;
  uint8_t enables;
} ShimEvtHeader;

typedef struct
{
  uint8_t type;
  uint8_t flags;
  uint16_t seq;
  /* RTC ticks at 32768 Hz, the same timeline as the data file's samples */
  uint64_t ts;
  uint8_t channel;
  uint8_t state;
  /* Edges lost this session so far, stopping at 0xFFFF */
  uint16_t dropped;
} ShimEvtRecord;

/* A record over Bluetooth, to a host that opted in: [INSTREAM_CMD_RESPONSE]
 * [EVENT_RECORD_RESPONSE][record length][record][CRC-16 of all before it, LE].
 * No ACK prefix and no session CRC: the frame carries its own, like the SD
 * file transfer frames. The length byte lets a host step over a longer record
 * from later firmware. */
#define EVT_BT_FRAME_TYPE       0x8AU /* INSTREAM_CMD_RESPONSE */
#define EVT_BT_FRAME_SUBTYPE    0xBBU /* EVENT_RECORD_RESPONSE */
#define EVT_BT_FRAME_HEAD_LEN   3U
#define EVT_BT_FRAME_CRC_OFFSET (EVT_BT_FRAME_HEAD_LEN + EVT_RECORD_LEN)
#define EVT_BT_FRAME_LEN        (EVT_BT_FRAME_CRC_OFFSET + 2U)

void ShimEvtRec_encodeHeader(const ShimEvtHeader *hdr, uint8_t *out);
void ShimEvtRec_encodeRecord(const ShimEvtRecord *rec, uint8_t *out);
/* All but the CRC: EVT_BT_FRAME_CRC_OFFSET bytes */
void ShimEvtRec_encodeBtFrame(const ShimEvtRecord *rec, uint8_t *out);

#endif /* SHIMMER_EVENT_RECORD_H_ */
