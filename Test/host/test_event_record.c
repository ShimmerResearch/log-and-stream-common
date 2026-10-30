/*
 * Host-side test for Events/shimmer_event_record.c.
 *
 * Pins the event file's byte layout against the worked example in
 * docs/SHIMMER3_EVENT_CHANNEL.md, so the document, the firmware and any host
 * parser written from the document agree to the byte.
 *
 * shimmer_event_record.c includes nothing but the C library, so this needs no
 * stubs. The Bluetooth frame's CRC comes from CRC/shimmer_swCrc.c, which is
 * what platform_crcData16() runs on both platforms. Run by
 * .github/workflows/host-tests.yml.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include "host_test.h"

#include "CRC/shimmer_swCrc.h"
#include "Events/shimmer_event_record.h"

static void expectBytes(const char *what, const uint8_t *got, const uint8_t *want, uint16_t len)
{
  uint16_t i;
  char msg[64];

  for (i = 0; i < len; i++)
  {
    if (got[i] != want[i])
    {
      snprintf(msg, sizeof(msg), "%s, byte %u", what, (unsigned) i);
      expectX(msg, got[i], want[i]);
      return;
    }
  }
}

/* The example's header: a Shimmer3R on v1.01.025 with both inputs enabled,
 * the data file opened at 10:15:00.250 UTC on 30 Oct 2026 */
static const uint8_t exampleHeader[EVT_FILE_HEADER_LEN] = {
  0x53,
  0x45,
  0x56,
  0x54,
  0x01,
  0x40,
  0x10,
  0x0A,
  0x00,
  0x20,
  0x12,
  0x37,
  0x72,
  0x35,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x90,
  0x5C,
  0xE4,
  0x6A,
  0x03,
  0x00,
  0x01,
  0x00,
  0x01,
  0x19,
  0x00,
  0x00,
  0x01,
  0x03,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
};

static void test_header_matches_the_documented_example(void)
{
  ShimEvtHeader hdr;
  uint8_t out[EVT_FILE_HEADER_LEN];

  testCase("header matches the documented example");
  hdr.hwId = 10;
  hdr.fileOpenTs = 0x0000357237122000ULL;
  hdr.rwcOffset = 0;
  hdr.configTime = 1793350800UL;
  hdr.fwId = 3;
  hdr.fwMajor = 1;
  hdr.fwMinor = 1;
  hdr.fwInternal = 25;
  hdr.firstSeq = 0;
  hdr.openReason = EVT_OPEN_SESSION_START;
  hdr.enables = EVT_ENABLE_BUTTON | EVT_ENABLE_TTL;

  /* Garbage first, so a byte the encoder leaves alone shows up */
  memset(out, 0xAA, sizeof(out));
  ShimEvtRec_encodeHeader(&hdr, out);
  expectBytes("header", out, exampleHeader, EVT_FILE_HEADER_LEN);
}

static void test_header_fields_land_where_documented(void)
{
  ShimEvtHeader hdr;
  uint8_t out[EVT_FILE_HEADER_LEN];

  testCase("header fields land where documented");
  memset(&hdr, 0, sizeof(hdr));
  hdr.rwcOffset = 0x0102030405060708ULL;
  hdr.firstSeq = 0xBEEF;
  hdr.openReason = EVT_OPEN_SPLIT;
  ShimEvtRec_encodeHeader(&hdr, out);
  expectX("RWC offset LSB at 16", out[16], 0x08);
  expectX("RWC offset MSB at 23", out[23], 0x01);
  expectX("first seq LSB at 34", out[34], 0xEF);
  expectX("first seq MSB at 35", out[35], 0xBE);
  expectX("open reason at 36", out[36], EVT_OPEN_SPLIT);
}

static void test_bt_frame_layout(void)
{
  ShimEvtRecord rec;
  uint8_t out[EVT_BT_FRAME_LEN];
  uint8_t bare[EVT_RECORD_LEN];

  testCase("Bluetooth frame: 0x8A 0xBB, record length, record, then the CRC");
  memset(&rec, 0, sizeof(rec));
  rec.type = EVT_TYPE_EDGE;
  rec.seq = 0x0102;
  rec.ts = 0x0000357237122000ULL;
  rec.channel = EVT_CHANNEL_TTL;
  memset(out, 0xAA, sizeof(out));
  ShimEvtRec_encodeBtFrame(&rec, out);
  ShimEvtRec_encodeRecord(&rec, bare);
  expectU("frame length", EVT_BT_FRAME_LEN, 21);
  expectX("in-stream response", out[0], 0x8A);
  expectX("event record subtype", out[1], 0xBB);
  expectU("record length", out[2], EVT_RECORD_LEN);
  expectBytes("record as in the file", &out[3], bare, EVT_RECORD_LEN);
  expectU("CRC at", EVT_BT_FRAME_CRC_OFFSET, 19);
  expectX("CRC left for the caller", out[EVT_BT_FRAME_CRC_OFFSET], 0xAA);
}

static void test_bt_frame_matches_the_documented_example(void)
{
  ShimEvtRecord rec;
  uint8_t out[EVT_BT_FRAME_LEN];
  uint16_t crc;
  /* SHIMMER3_BT_COMMUNICATION_PROTOCOL.md section 7.15: the worked example's
   * marker press, as the link's second record */
  static const uint8_t frame[EVT_BT_FRAME_LEN]
      = { 0x8A, 0xBB, 0x10, 0x01, 0x00, 0x01, 0x00, 0x33, 0x53, 0x18, 0x37,
          0x72, 0x35, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x09, 0x20 };

  testCase("Bluetooth frame matches the documented example");
  rec.type = EVT_TYPE_EDGE;
  rec.flags = 0;
  rec.seq = 1;
  rec.ts = 0x0000357237185333ULL;
  rec.channel = EVT_CHANNEL_BUTTON;
  rec.state = EVT_STATE_BUTTON;
  rec.dropped = 0;
  ShimEvtRec_encodeBtFrame(&rec, out);
  crc = ShimSwCrc_calc16(out, EVT_BT_FRAME_CRC_OFFSET);
  out[EVT_BT_FRAME_CRC_OFFSET] = (uint8_t) crc;
  out[EVT_BT_FRAME_CRC_OFFSET + 1U] = (uint8_t) (crc >> 8);
  expectBytes("frame", out, frame, EVT_BT_FRAME_LEN);
}

static void test_records_match_the_documented_example(void)
{
  ShimEvtRecord rec;
  uint8_t out[EVT_RECORD_LEN];
  static const uint8_t press[EVT_RECORD_LEN] = { 0x01, 0x00, 0x00, 0x00, 0xA4,
    0xD0, 0x11, 0x37, 0x72, 0x35, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00 };
  static const uint8_t ttlRise[EVT_RECORD_LEN] = { 0x01, 0x01, 0x07, 0x00, 0x04,
    0x20, 0x21, 0x37, 0x72, 0x35, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00 };
  static const uint8_t final[EVT_RECORD_LEN] = { 0x03, 0x80, 0x10, 0x00, 0xAE,
    0x87, 0x5D, 0x37, 0x72, 0x35, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00 };

  testCase("records match the documented example");

  rec.type = EVT_TYPE_EDGE;
  rec.flags = 0;
  rec.seq = 0;
  rec.ts = 0x000035723711D0A4ULL;
  rec.channel = EVT_CHANNEL_BUTTON;
  rec.state = EVT_STATE_BUTTON;
  rec.dropped = 0;
  ShimEvtRec_encodeRecord(&rec, out);
  expectBytes("record 0, button press", out, press, EVT_RECORD_LEN);

  rec.flags = EVT_FLAG_HW_CAPTURE;
  rec.seq = 7;
  rec.ts = 0x0000357237212004ULL;
  rec.channel = EVT_CHANNEL_TTL;
  rec.state = EVT_STATE_TTL;
  ShimEvtRec_encodeRecord(&rec, out);
  expectBytes("record 7, TTL rising", out, ttlRise, EVT_RECORD_LEN);

  rec.type = EVT_TYPE_HEARTBEAT;
  rec.flags = EVT_FLAG_FINAL;
  rec.seq = 16;
  rec.ts = 0x00003572375D87AEULL;
  rec.channel = EVT_CHANNEL_NONE;
  rec.state = 0;
  ShimEvtRec_encodeRecord(&rec, out);
  expectBytes("record 16, final", out, final, EVT_RECORD_LEN);
}

static void test_dropped_is_little_endian(void)
{
  ShimEvtRecord rec;
  uint8_t out[EVT_RECORD_LEN];

  testCase("dropped count is little-endian at 14");
  memset(&rec, 0, sizeof(rec));
  rec.dropped = 0x1234;
  ShimEvtRec_encodeRecord(&rec, out);
  expectX("dropped LSB", out[14], 0x34);
  expectX("dropped MSB", out[15], 0x12);
}

int main(void)
{
  hostTestSilenceUnused();

  test_header_matches_the_documented_example();
  test_header_fields_land_where_documented();
  test_records_match_the_documented_example();
  test_dropped_is_little_endian();
  test_bt_frame_layout();
  test_bt_frame_matches_the_documented_example();

  return hostTestReport("test_event_record");
}

#endif /* SHIMMER_HOST_TEST */
