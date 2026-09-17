/*
 * Host-side test for the CRC mode dispatcher in CRC/shimmer_crc.c.
 *
 * Built against the stub platform in Test/host/stubs - see stubs/README.md.
 * Run by .github/workflows/host-tests.yml.
 *
 * WHAT THIS IS FOR, AND HOW IT DIFFERS FROM test_swcrc.c. That file covers the
 * polynomial: given these bytes, this checksum. This one covers the layer above
 * it - which bytes get a checksum at all, where it is written, and what happens
 * when it does not match. Those are decided by COMMS_CRC_MODE, which is
 * negotiated per connection, so the same firmware runs all three modes
 * depending on what the host asked for.
 *
 * The mode that deserves the most attention is CRC_OFF, because it FAILS OPEN:
 * checkCrc() returns "valid" without looking at anything. That is correct - a
 * host that did not ask for CRCs did not send any - but it means a mode
 * negotiated wrongly does not produce checksum errors, it produces silent
 * acceptance of whatever arrives. It is asserted here so that the behaviour is
 * deliberate and stays that way.
 *
 * This file also runs the firmware's own testCrcDriver() self-test. That
 * function already exists and already carries the wire-format vectors; until
 * now it only ran on a device, during factory test. There is no reason for CI
 * not to run it on every push as well.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Without the
 * guard it collides with the firmware's own main() at link time, in both the
 * STM32 and the MSP430 build. Only the host-test build passes
 * -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "host_stubs.h"
#include "host_test.h"
#include "log_and_stream_includes.h"

#define PAYLOAD_LEN 8

/* 0x01..0x08, whose CRC is 0x48AA - the vector test_swcrc.c pins. */
static void fillPayload(uint8_t *buf, uint8_t len)
{
  uint8_t i;
  memset(buf, 0xEE, len + 4); /* 0xEE so an unwritten CRC byte is obvious */
  for (i = 0; i < len; i++)
  {
    buf[i] = (uint8_t) (i + 1);
  }
}

/*
 * The firmware's own self-test, run here rather than only on a device.
 *
 * It covers both weak CRC hooks - platform_crcData() for the Bluetooth and dock
 * links, platform_crcData16() for SD file transfer frames longer than 255 bytes
 * - against fixed wire-format vectors. Anything that moves one of those changes
 * what goes out on a link, so this failing is never a test problem.
 */
static void test_firmware_self_test(void)
{
  testCase("test_firmware_self_test");
  expectU("testCrcDriver() passes", testCrcDriver(), 0);
}

/* Where the CRC bytes land, per mode. */
static void test_insert_per_mode(void)
{
  uint8_t buf[PAYLOAD_LEN + 4];

  testCase("test_insert_per_mode");

  /* CRC_OFF writes nothing at all - the caller's length must not change. */
  fillPayload(buf, PAYLOAD_LEN);
  calculateCrcAndInsert(CRC_OFF, buf, PAYLOAD_LEN);
  expectX("CRC_OFF leaves the first trailing byte alone", buf[PAYLOAD_LEN], 0xEE);
  expectX("CRC_OFF leaves the second alone", buf[PAYLOAD_LEN + 1], 0xEE);

  /* One byte: the low half of the CRC, and nothing after it. */
  fillPayload(buf, PAYLOAD_LEN);
  calculateCrcAndInsert(CRC_1BYTE_ENABLED, buf, PAYLOAD_LEN);
  expectX("1-byte mode writes the CRC low byte", buf[PAYLOAD_LEN], 0xAA);
  expectX("and writes nothing after it", buf[PAYLOAD_LEN + 1], 0xEE);

  /* Two bytes: low then high - little-endian on the wire. */
  fillPayload(buf, PAYLOAD_LEN);
  calculateCrcAndInsert(CRC_2BYTES_ENABLED, buf, PAYLOAD_LEN);
  expectX("2-byte mode writes the low byte first", buf[PAYLOAD_LEN], 0xAA);
  expectX("then the high byte", buf[PAYLOAD_LEN + 1], 0x48);
  expectX("and nothing after that", buf[PAYLOAD_LEN + 2], 0xEE);

  /* The payload itself is never touched. */
  {
    uint8_t i;
    for (i = 0; i < PAYLOAD_LEN; i++)
    {
      expectU("payload is unmodified", buf[i], (uint8_t) (i + 1));
    }
  }
}

/* checkCrc() against what calculateCrcAndInsert() just wrote, in each mode. */
static void test_check_round_trip(void)
{
  uint8_t buf[PAYLOAD_LEN + 4];

  testCase("test_check_round_trip");

  fillPayload(buf, PAYLOAD_LEN);
  calculateCrcAndInsert(CRC_1BYTE_ENABLED, buf, PAYLOAD_LEN);
  expectTrue("1-byte: accepts its own CRC", checkCrc(CRC_1BYTE_ENABLED, buf, PAYLOAD_LEN));

  fillPayload(buf, PAYLOAD_LEN);
  calculateCrcAndInsert(CRC_2BYTES_ENABLED, buf, PAYLOAD_LEN);
  expectTrue("2-byte: accepts its own CRC", checkCrc(CRC_2BYTES_ENABLED, buf, PAYLOAD_LEN));

  /* A 2-byte packet read as 1-byte still passes, because the low byte is the
   * same and the high byte is simply not looked at. Worth knowing: a mode
   * mismatch in this direction does not announce itself. */
  expectTrue("a 2-byte packet passes a 1-byte check",
      checkCrc(CRC_1BYTE_ENABLED, buf, PAYLOAD_LEN));
}

/* What a corrupted packet has to look like to be rejected. */
static void test_check_rejects_corruption(void)
{
  uint8_t buf[PAYLOAD_LEN + 4];
  uint8_t bit;

  testCase("test_check_rejects_corruption");

  /* Every single-bit flip in the payload must be caught. */
  {
    uint8_t byteIdx;
    uint8_t missed = 0;
    for (byteIdx = 0; byteIdx < PAYLOAD_LEN; byteIdx++)
    {
      for (bit = 0; bit < 8; bit++)
      {
        fillPayload(buf, PAYLOAD_LEN);
        calculateCrcAndInsert(CRC_2BYTES_ENABLED, buf, PAYLOAD_LEN);
        buf[byteIdx] ^= (uint8_t) (1U << bit);
        if (checkCrc(CRC_2BYTES_ENABLED, buf, PAYLOAD_LEN))
        {
          missed++;
        }
      }
    }
    expectU("every single-bit payload flip is caught", missed, 0);
  }

  /* A corrupted CRC byte must be caught too - a checksum that only validates
   * the payload would accept a packet whose checksum was mangled in transit. */
  fillPayload(buf, PAYLOAD_LEN);
  calculateCrcAndInsert(CRC_2BYTES_ENABLED, buf, PAYLOAD_LEN);
  buf[PAYLOAD_LEN] ^= 0x01;
  expectFalse("a corrupted low CRC byte is caught",
      checkCrc(CRC_2BYTES_ENABLED, buf, PAYLOAD_LEN));

  fillPayload(buf, PAYLOAD_LEN);
  calculateCrcAndInsert(CRC_2BYTES_ENABLED, buf, PAYLOAD_LEN);
  buf[PAYLOAD_LEN + 1] ^= 0x01;
  expectFalse("a corrupted high CRC byte is caught in 2-byte mode",
      checkCrc(CRC_2BYTES_ENABLED, buf, PAYLOAD_LEN));

  /* ...but NOT in 1-byte mode, which never reads it. This is the cost of the
   * shorter checksum, stated rather than assumed. */
  expectTrue("a corrupted high CRC byte is invisible in 1-byte mode",
      checkCrc(CRC_1BYTE_ENABLED, buf, PAYLOAD_LEN));
}

/*
 * CRC_OFF fails open, deliberately. Asserted so that the behaviour is a
 * decision on the record rather than something nobody looked at.
 */
static void test_crc_off_fails_open(void)
{
  uint8_t buf[PAYLOAD_LEN + 4];

  testCase("test_crc_off_fails_open");

  fillPayload(buf, PAYLOAD_LEN);
  expectTrue("CRC_OFF accepts a packet with no CRC at all",
      checkCrc(CRC_OFF, buf, PAYLOAD_LEN));

  memset(buf, 0x00, sizeof(buf));
  expectTrue("CRC_OFF accepts an all-zero packet", checkCrc(CRC_OFF, buf, PAYLOAD_LEN));

  memset(buf, 0xFF, sizeof(buf));
  expectTrue("CRC_OFF accepts an all-ones packet", checkCrc(CRC_OFF, buf, PAYLOAD_LEN));

  /* PINNED, NOT ENDORSED - the zero-length payload is degenerate, and not in
   * the way you would guess. It does not compute the CRC of nothing: the
   * dispatcher routes to ShimSwCrc_calc(), which seeds with msg[0] BEFORE its
   * loop and so reads one byte even when told there are none (see the comment
   * on ShimSwCrc_calc16 - that divergence is deliberate and load-bearing for
   * 2-byte packets). calculateCrcAndInsert() then writes the CRC low byte to
   * buf[0] - over the very byte it just checksummed - so the packet cannot be
   * re-verified, and checkCrc() rejects a packet the same call produced.
   *
   * No caller passes a zero payload length; every protocol frame on both links
   * carries at least a command byte. This is asserted rather than fixed because
   * the asymmetry it rests on is relied upon elsewhere, and because a silent
   * change here would move the wire format. If a zero-length frame ever becomes
   * reachable, this assertion is what should stop it. */
  fillPayload(buf, PAYLOAD_LEN);
  calculateCrcAndInsert(CRC_2BYTES_ENABLED, buf, 0);
  expectFalse("a zero-length payload cannot be re-verified",
      checkCrc(CRC_2BYTES_ENABLED, buf, 0));

  /* One byte of payload is the shortest frame that does round-trip. */
  fillPayload(buf, PAYLOAD_LEN);
  calculateCrcAndInsert(CRC_2BYTES_ENABLED, buf, 1);
  expectTrue("a one-byte payload round-trips", checkCrc(CRC_2BYTES_ENABLED, buf, 1));
  buf[0] ^= 0x01;
  expectFalse("and corrupting it is caught", checkCrc(CRC_2BYTES_ENABLED, buf, 1));
}

/*
 * CRC_MAX_SUPPORTED_BYTES is a sentinel for bounds checking, not a mode - the
 * header says so. But the dispatcher's tests are "!= CRC_OFF" and
 * "== CRC_2BYTES_ENABLED", so the sentinel behaves as a one-byte mode rather
 * than being rejected. PINNED, NOT ENDORSED: if mode validation is ever
 * tightened at the call sites, this is the assertion that should change with
 * it, rather than the behaviour changing unnoticed.
 */
static void test_sentinel_mode_behaviour(void)
{
  uint8_t buf[PAYLOAD_LEN + 4];

  testCase("test_sentinel_mode_behaviour");

  fillPayload(buf, PAYLOAD_LEN);
  calculateCrcAndInsert(CRC_MAX_SUPPORTED_BYTES, buf, PAYLOAD_LEN);
  expectX("the sentinel writes one CRC byte", buf[PAYLOAD_LEN], 0xAA);
  expectX("and not a second", buf[PAYLOAD_LEN + 1], 0xEE);
  expectTrue("and its own packet checks out",
      checkCrc(CRC_MAX_SUPPORTED_BYTES, buf, PAYLOAD_LEN));
}

int main(void)
{
  printf("shimmer_crc host tests\n\n");
  hostTestSilenceUnused();
  hostStub_reset();

  test_firmware_self_test();
  test_insert_per_mode();
  test_check_round_trip();
  test_check_rejects_corruption();
  test_crc_off_fails_open();
  test_sentinel_mode_behaviour();

  return hostTestReport("shimmer_crc");
}

#endif /* SHIMMER_HOST_TEST */
