/*
 * Host-side test for Test/shimmer_factory_record.c: the byte layouts a
 * production test reads back over the dock and over Bluetooth.
 *
 * A host parses these by offset, so each layout is pinned here byte for byte
 * against a hand-written vector rather than against the encoder's own idea of
 * it. shimmer_factory_record.c includes nothing but the C library, so this
 * needs no stubs.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include "host_test.h"

#include "Test/shimmer_factory_record.h"

static void expectBytes(const char *what, const uint8_t *got, const uint8_t *want, unsigned len)
{
  unsigned i;
  for (i = 0; i < len; i++)
  {
    if (got[i] != want[i])
    {
      printf("  FAIL [%s] %s: byte %u is 0x%02X, want 0x%02X\n", hostTestCase,
          what, i, got[i], want[i]);
      hostTestFailures++;
      return;
    }
  }
}

static void test_boot_report_layout(void)
{
  boot_report_t r;
  uint8_t out[BOOT_REPORT_LEN + 4];
  unsigned i;
  static const uint8_t want[BOOT_REPORT_LEN] = {
    0x02, /* layout */
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, /* UID */
    0x0B,       /* flags */
    0xE9, 0x03, /* I2C 1001 ms */
    0x34, 0x12, /* BT 0x1234 ms */
    0xFF, 0xFF, /* config, saturated */
    0x02,       /* baud attempts */
    0x02,       /* LSE drive */
    0x1B, 0x00, /* parts found: LIS3MDL, ADXL371, BMP390, BMP581 */
    0x2D, 0x80, /* parts expected: LIS3MDL, ADS7028, BMP390, ADS1292R, bit 15 */
  };

  testCase("boot report: 26 bytes, layout 2, little-endian, documented order");
  for (i = 0; i < BOOT_REPORT_UID_LEN; i++)
  {
    r.uid[i] = (uint8_t) (0x10U + i);
  }
  r.flags = BOOT_REPORT_FLAG_I2C_SLOW | BOOT_REPORT_FLAG_BT_FAILED | BOOT_REPORT_FLAG_LSE_FALLBACK;
  r.i2cMs = 1001U;
  r.btMs = 0x1234U;
  r.configMs = ShimFactoryRecord_saturateMs(70000UL);
  r.btBaudAttempts = 2U;
  r.lseDrive = 2U;
  r.partsFound = PP_PART_LIS3MDL | PP_PART_ADXL371 | PP_PART_BMP390 | PP_PART_BMP581;
  r.partsExpected = (uint16_t) (PP_PART_LIS3MDL | PP_PART_ADS7028
      | PP_PART_BMP390 | PP_PART_ADS1292R | 0x8000U);

  memset(out, 0xA5, sizeof(out));
  expectU("length", ShimFactoryRecord_encodeBootReport(&r, out), BOOT_REPORT_LEN);
  expectBytes("bytes", out, want, BOOT_REPORT_LEN);
  expectX("nothing written past the end", out[BOOT_REPORT_LEN], 0xA5);
  expectU("fits a dock reply", BOOT_REPORT_LEN <= 128U, 1);
  expectU("layout version", BOOT_REPORT_LAYOUT_VER, 2);
}

static void test_flag_values(void)
{
  testCase("boot report flags: one bit each, as documented");
  expectX("I2C slow", BOOT_REPORT_FLAG_I2C_SLOW, 0x01);
  expectX("BT failed", BOOT_REPORT_FLAG_BT_FAILED, 0x02);
  expectX("config slow", BOOT_REPORT_FLAG_CONFIG_SLOW, 0x04);
  expectX("LSE fallback", BOOT_REPORT_FLAG_LSE_FALLBACK, 0x08);
}

static void test_saturate(void)
{
  testCase("ms: stops at 65535 rather than wrapping");
  expectU("0", ShimFactoryRecord_saturateMs(0UL), 0);
  expectU("65535", ShimFactoryRecord_saturateMs(65535UL), 65535);
  expectU("65536", ShimFactoryRecord_saturateMs(65536UL), 65535);
  expectU("80 s", ShimFactoryRecord_saturateMs(80000UL), 65535);
  expectU("4294967295", ShimFactoryRecord_saturateMs(0xFFFFFFFFUL), 65535);
}

static void test_button_state(void)
{
  uint8_t out[BUTTON_STATE_LEN + 1];
  static const uint8_t want[BUTTON_STATE_LEN] = { 0x01, 0x02, 0x01, 0xDC, 0x05 };

  testCase("button state: level, presses u16, last press ms u16");
  memset(out, 0xA5, sizeof(out));
  expectU("length", ShimFactoryRecord_encodeButtonState(7U, 0x0102U, 1500U, out),
      BUTTON_STATE_LEN);
  expectBytes("bytes", out, want, BUTTON_STATE_LEN);
  expectX("nothing written past the end", out[BUTTON_STATE_LEN], 0xA5);
}

/* The CRC bytes are computed by Extras' shimmer_crc.py, the in-repo host
 * reference, which shares no code with the firmware's ShimSwCrc_calc() */
static const uint8_t ppRecordVector[PP_RECORD_LEN] = {
  0x50, 0x50,             /* magic */
  0x01,                   /* layout */
  0x03,                   /* DONE */
  0x16,                   /* run_seq 22 */
  0x34, 0x12,             /* nonce */
  0x78, 0x56, 0x34, 0x12, /* step_ok */
  0x1B, 0x00,             /* parts_found */
  0x01,                   /* schedule version */
  0xF2, 0xBF,             /* CRC */
};

static pp_record_t vectorRecord(void)
{
  pp_record_t r;
  r.state = PP_STATE_DONE;
  r.runSeq = 22U;
  r.nonce = 0x1234U;
  r.stepOk = 0x12345678UL;
  r.partsFound = PP_PART_LIS3MDL | PP_PART_ADXL371 | PP_PART_BMP390 | PP_PART_BMP581;
  r.scheduleVer = 1U;
  return r;
}

static void test_pp_record_encode(void)
{
  pp_record_t r = vectorRecord();
  uint8_t out[PP_RECORD_LEN + 1];

  testCase("power-profile record: 16 bytes, CRC from the host reference");
  memset(out, 0xA5, sizeof(out));
  ShimFactoryRecord_encodePpRecord(&r, out);
  expectBytes("bytes", out, ppRecordVector, PP_RECORD_LEN);
  expectX("nothing written past the page", out[PP_RECORD_LEN], 0xA5);
  expectU("one EEPROM page", PP_RECORD_LEN, 16);
}

static void test_pp_record_decode(void)
{
  pp_record_t r;
  uint8_t page[PP_RECORD_LEN];
  unsigned i;

  testCase("power-profile record: decodes, and refuses a damaged page");
  memset(&r, 0, sizeof(r));
  expectU("valid", ShimFactoryRecord_decodePpRecord(ppRecordVector, &r), 1);
  expectU("state", r.state, PP_STATE_DONE);
  expectU("run_seq", r.runSeq, 22);
  expectX("nonce", r.nonce, 0x1234);
  expectX("step_ok", r.stepOk, 0x12345678UL);
  expectX("parts_found", r.partsFound, 0x1B);
  expectU("schedule", r.scheduleVer, 1);

  /* Any one byte changed is refused, CRC bytes included */
  for (i = 0; i < PP_RECORD_LEN; i++)
  {
    memcpy(page, ppRecordVector, sizeof(page));
    page[i] ^= 0x01U;
    expectU("one byte flipped", ShimFactoryRecord_decodePpRecord(page, &r), 0);
  }

  memset(page, 0xFF, sizeof(page));
  expectU("erased EEPROM", ShimFactoryRecord_decodePpRecord(page, &r), 0);
  memset(page, 0x00, sizeof(page));
  expectU("zeroed EEPROM", ShimFactoryRecord_decodePpRecord(page, &r), 0);
}

static void test_pp_record_round_trip(void)
{
  static const uint8_t states[] = { PP_STATE_DISARMED, PP_STATE_ARMED,
    PP_STATE_RUNNING, PP_STATE_DONE, PP_STATE_SKIPPED, PP_STATE_ABORTED };
  pp_record_t r = vectorRecord();
  pp_record_t back;
  uint8_t page[PP_RECORD_LEN];
  unsigned i;

  testCase("power-profile record: every state round-trips, extremes too");
  for (i = 0; i < sizeof(states); i++)
  {
    r.state = states[i];
    r.nonce = (uint16_t) (i == 0 ? 0x0000U : 0xFFFFU);
    r.stepOk = (i == 0) ? 0UL : 0xFFFFFFFFUL;
    ShimFactoryRecord_encodePpRecord(&r, page);
    memset(&back, 0, sizeof(back));
    expectU("valid", ShimFactoryRecord_decodePpRecord(page, &back), 1);
    expectU("state", back.state, states[i]);
    expectX("nonce", back.nonce, r.nonce);
    expectX("step_ok", back.stepOk, r.stepOk);
  }
  expectX("state values are protocol", PP_STATE_ABORTED, 5);
  expectX("no record", PP_STATE_NONE, 0xFF);
}

static void test_pp_readback(void)
{
  pp_record_t r = vectorRecord();
  pp_schedule_t s;
  uint8_t out[PP_READBACK_MAX_LEN + 4];
  static const uint8_t want[PP_READBACK_HEADER_LEN + 6] = {
    0x01,                   /* layout */
    0x03,                   /* DONE */
    0x16,                   /* run_seq */
    0x34, 0x12,             /* nonce */
    0x78, 0x56, 0x34, 0x12, /* step_ok */
    0x1B, 0x00,             /* parts_found */
    0x23, 0x00,             /* parts expected */
    0xE8, 0x03,             /* window 1000 ms */
    0x01,                   /* marker */
    0x02,                   /* n */
    0x01, 0x34, 0x08,       /* MARKER 2100 ms */
    0x02, 0x88, 0x13,       /* BASELINE 5000 ms */
  };

  testCase("power-profile readback: header, then 3 bytes per step");
  memset(&s, 0, sizeof(s));
  s.scheduleVer = 1U;
  s.markerId = PP_MARKER_NONCE16;
  s.baselineWindowMs = 1000U;
  s.partsExpected = PP_PART_LIS3MDL | PP_PART_ADXL371 | PP_PART_ADS1292R;
  expectU("add", ShimFactoryRecord_addPpStep(&s, PP_STEP_MARKER, PP_MARKER_DURATION_MS), 1);
  expectU("add", ShimFactoryRecord_addPpStep(&s, PP_STEP_BASELINE, 5000U), 1);

  memset(out, 0xA5, sizeof(out));
  expectU("length", ShimFactoryRecord_encodePpReadback(&r, &s, out), sizeof(want));
  expectBytes("bytes", out, want, sizeof(want));
  expectX("nothing written past the end", out[sizeof(want)], 0xA5);
  expectU("marker lasts 2.1 s", PP_MARKER_DURATION_MS, 2100);
  expectU("run time: steps + windows + one pulse", ShimFactoryRecord_ppRunMs(&s),
      2100U + 5000U + 2U * 1000U + PP_MARKER_PULSE_MS);
}

static void test_pp_readback_without_record(void)
{
  pp_schedule_t s;
  uint8_t out[PP_READBACK_MAX_LEN];
  unsigned i;

  testCase("power-profile readback: no record reads NONE, fields zero");
  memset(&s, 0, sizeof(s));
  s.baselineWindowMs = 1000U;
  s.markerId = PP_MARKER_NONCE16;
  memset(out, 0xA5, sizeof(out));
  expectU("length", ShimFactoryRecord_encodePpReadback(NULL, &s, out), PP_READBACK_HEADER_LEN);
  expectX("state", out[1], PP_STATE_NONE);
  for (i = 2; i < 11; i++)
  {
    expectX("record field", out[i], 0x00);
  }
  expectX("n", out[16], 0);
}

static void test_pp_readback_fits(void)
{
  pp_schedule_t s;
  uint8_t out[PP_READBACK_MAX_LEN + 8];
  unsigned i;

  testCase("power-profile readback: never more than 128 bytes");
  expectU("37 steps at most", PP_MAX_STEPS, 37);
  memset(&s, 0, sizeof(s));
  for (i = 0; i < PP_MAX_STEPS; i++)
  {
    expectU("add", ShimFactoryRecord_addPpStep(&s, PP_STEP_LED_LWR_RED, 1000U), 1);
  }
  expectU("the 38th is refused",
      ShimFactoryRecord_addPpStep(&s, PP_STEP_BLUETOOTH, 1000U), 0);
  memset(out, 0xA5, sizeof(out));
  expectU("full length", ShimFactoryRecord_encodePpReadback(NULL, &s, out),
      PP_READBACK_HEADER_LEN + 3U * PP_MAX_STEPS);
  expectTrue("fits a dock reply",
      PP_READBACK_HEADER_LEN + 3U * PP_MAX_STEPS <= PP_READBACK_MAX_LEN);
  expectX("nothing past 128", out[PP_READBACK_MAX_LEN], 0xA5);

  /* A corrupt count is clamped, not trusted */
  s.n = 200U;
  expectU("clamped", ShimFactoryRecord_encodePpReadback(NULL, &s, out),
      PP_READBACK_HEADER_LEN + 3U * PP_MAX_STEPS);
  expectU("n clamped", out[16], PP_MAX_STEPS);
}

int main(void)
{
  hostTestSilenceUnused();

  test_boot_report_layout();
  test_flag_values();
  test_saturate();
  test_button_state();
  test_pp_record_encode();
  test_pp_record_decode();
  test_pp_record_round_trip();
  test_pp_readback();
  test_pp_readback_without_record();
  test_pp_readback_fits();

  return hostTestReport("test_factory_record");
}

#endif /* SHIMMER_HOST_TEST */
