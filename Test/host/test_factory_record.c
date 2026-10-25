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
    0x01, /* layout */
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, /* UID */
    0x0B,       /* flags */
    0xE9, 0x03, /* I2C 1001 ms */
    0x34, 0x12, /* BT 0x1234 ms */
    0xFF, 0xFF, /* config, saturated */
    0x02,       /* baud attempts */
    0x02,       /* LSE drive */
  };

  testCase("boot report: 22 bytes, little-endian, in the documented order");
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

  memset(out, 0xA5, sizeof(out));
  expectU("length", ShimFactoryRecord_encodeBootReport(&r, out), BOOT_REPORT_LEN);
  expectBytes("bytes", out, want, BOOT_REPORT_LEN);
  expectX("nothing written past the end", out[BOOT_REPORT_LEN], 0xA5);
  expectU("fits a dock reply", BOOT_REPORT_LEN <= 128U, 1);
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

int main(void)
{
  hostTestSilenceUnused();

  test_boot_report_layout();
  test_flag_values();
  test_saturate();
  test_button_state();

  return hostTestReport("test_factory_record");
}

#endif /* SHIMMER_HOST_TEST */
