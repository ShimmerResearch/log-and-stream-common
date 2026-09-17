/*
 * Host-side test for Util/shimmer_util.c.
 *
 * shimmer_util.c includes nothing but <stdint.h>, <stdlib.h> and <string.h>, so
 * this needs no MCU toolchain, no HAL and no target hardware, and no stubs
 * either. Run by .github/workflows/host-tests.yml.
 *
 * WHAT THIS IS FOR. These eight functions look too small to be worth testing,
 * which is exactly why they are not. They sit under things that are expensive
 * to get wrong and slow to notice:
 *
 *   ShimUtil_ItoaWith0   names every directory on the SD card
 *   ShimUtil_ItoaNo0     writes the 64-bit fields of the config file
 *   ShimUtil_Atol64      reads them back
 *   ShimUtil_strlen_v    measures every Bluetooth response
 *   ShimUtil_reverseArray puts the MAC address the right way round
 *
 * A fault in any of them shows up as a corrupt recording or a config file that
 * will not round-trip, days later and a long way from the cause. The cases
 * below are written against those five real callers, not against the functions
 * in the abstract, and several of them pin behaviour at a limit rather than
 * assert that it is right - where that is so, the comment says which.
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

#include "Util/shimmer_util.h"
#include "host_test.h"

/*
 * ShimUtil_ItoaWith0(num, buf, len) writes len-1 zero-padded digits and a NUL.
 *
 * The caller that matters is ShimSdDataFile_makeBasedir(), which passes a
 * 4-byte buffer for a uint16_t dirCounter - three digits. Directory names are
 * "<name>-NNN", so these three digits are the only thing separating one
 * recording session's directory from the next.
 */
static void test_itoa_with_zeros(void)
{
  uint8_t buf[8];

  testCase("test_itoa_with_zeros");

  /* The real call: 3 digits for the SD directory counter. */
  ShimUtil_ItoaWith0(0, buf, 4);
  expectStr("counter 0 -> 000", (char *) buf, "000");
  ShimUtil_ItoaWith0(1, buf, 4);
  expectStr("counter 1 -> 001", (char *) buf, "001");
  ShimUtil_ItoaWith0(42, buf, 4);
  expectStr("counter 42 -> 042", (char *) buf, "042");
  ShimUtil_ItoaWith0(999, buf, 4);
  expectStr("counter 999 -> 999", (char *) buf, "999");

  /* PINNED, NOT ENDORSED. dirCounter is a uint16_t and nothing clamps it, so
   * the 1000th directory in one experiment folder is named "000" again - the
   * same name as the first. f_mkdir() then fails with FR_EXIST and
   * ShimSdDataFile_makeBasedir() returns 0. It is a fail-stop rather than a
   * silent overwrite, but it is a limit worth knowing about, and if the field
   * is ever widened this assertion is what will notice. */
  ShimUtil_ItoaWith0(1000, buf, 4);
  expectStr("counter 1000 wraps to 000", (char *) buf, "000");
  ShimUtil_ItoaWith0(1234, buf, 4);
  expectStr("counter 1234 keeps the low 3 digits", (char *) buf, "234");

  /* The NUL is written and the byte past the field is not touched. */
  memset(buf, 0xAA, sizeof(buf));
  ShimUtil_ItoaWith0(7, buf, 4);
  expectU("terminator at buf[3]", buf[3], 0);
  expectX("buf[4] untouched", buf[4], 0xAA);
}

/*
 * ShimUtil_ItoaNo0(num, buf, max_len) writes up to max_len-1 digits with no
 * leading zeros.
 *
 * Its caller is the SD config file: ShimSdCfgFile_generate() passes 21 for the
 * 64-bit derived-channels field, and ShimConfig writes the config time the same
 * way. 21 is exactly right - UINT64_MAX is 20 digits - so the boundary case
 * here is the one the firmware actually reaches.
 */
static void test_itoa_no_zeros(void)
{
  char buf[24];

  testCase("test_itoa_no_zeros");

  ShimUtil_ItoaNo0(0, buf, 21);
  expectStr("zero is written, not left blank", buf, "0");
  ShimUtil_ItoaNo0(7, buf, 21);
  expectStr("single digit", buf, "7");
  ShimUtil_ItoaNo0(1234567890U, buf, 21);
  expectStr("ten digits", buf, "1234567890");

  /* The real field width. A derived-channels mask with the top bit set is
   * 20 digits, which is the whole of the buffer bar the terminator. */
  ShimUtil_ItoaNo0(UINT64_MAX, buf, 21);
  expectStr("UINT64_MAX fits exactly in 21", buf, "18446744073709551615");
  expectU("and is NUL terminated", (uint8_t) buf[20], 0);

  ShimUtil_ItoaNo0(0x8000000000000000ULL, buf, 21);
  expectStr("top bit only", buf, "9223372036854775808");

  /* PINNED, NOT ENDORSED. Given too small a buffer it keeps the LOW digits, in
   * order, and reports no error. Callers pass a literal 21 today so this is
   * unreachable; the assertion exists so that shrinking a buffer changes a test
   * result rather than silently truncating a config value. */
  ShimUtil_ItoaNo0(12345, buf, 4);
  expectStr("overlong input keeps the low digits", buf, "345");
}

/*
 * ShimUtil_Atol64() is the inverse of a 20-digit ShimUtil_ItoaWith0(): it reads
 * a fixed 20 characters as four 5-digit groups. It has no caller inside this
 * repository - both firmwares use it to parse 64-bit values off the dock and
 * Bluetooth links - so the round-trip is the contract worth asserting.
 */
static void test_atol64(void)
{
  uint8_t buf[24];
  const uint64_t vectors[] = {
    0U, 1U, 99999U, /* exactly fills the last group */
    100000U,        /* first carry into the next group */
    1234567890123U, /* a plausible RWC tick count */
    UINT64_MAX,     /* 20 digits - the widest input it can be given */
  };
  unsigned i;

  testCase("test_atol64");

  for (i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i++)
  {
    char what[64];
    ShimUtil_ItoaWith0(vectors[i], buf, 21);
    snprintf(what, sizeof(what), "round-trips %" PRIu64, vectors[i]);
    expectU(what, ShimUtil_Atol64(buf), vectors[i]);
  }

  /* Leading zeros must not be read as octal - atol() would not, but this is the
   * property the group-of-five scheme depends on. */
  memcpy(buf, "00000000000000000010", 20);
  expectU("leading zeros are decimal", ShimUtil_Atol64(buf), 10U);

  /* It reads exactly 20 bytes and stops. A 21st byte cannot reach the result. */
  memcpy(buf, "00000000000000000001", 20);
  buf[20] = '9';
  expectU("reads exactly 20 bytes", ShimUtil_Atol64(buf), 1U);
}

/*
 * The volatile-safe memory helpers. These exist because memcpy() on a volatile
 * object is not something the standard defines, and the buffers they serve are
 * written by DMA and by interrupt handlers.
 *
 * All three copy BACKWARDS, from the top byte down. That is not an
 * implementation detail a caller can ignore, because it decides what happens
 * when the regions overlap: a top-down copy is safe when the destination is
 * ABOVE the source, and smears the top byte across the whole range when it is
 * below. memmove() handles both; these do not. Both directions are asserted
 * below, so that reversing the loop - or "simplifying" one of these into a
 * plain memcpy() - fails here rather than in the field.
 */
static void test_volatile_memory_helpers(void)
{
  static volatile uint8_t vdst[16];
  static volatile uint8_t vsrc[16];
  uint8_t src[16];
  uint8_t i;

  testCase("test_volatile_memory_helpers");

  for (i = 0; i < 16; i++)
  {
    src[i] = (uint8_t) (i + 1);
  }

  ShimUtil_memset_v(vdst, 0xA5, sizeof(vdst));
  expectX("memset_v writes the first byte", vdst[0], 0xA5);
  expectX("memset_v writes the last byte", vdst[15], 0xA5);

  ShimUtil_memset_v(vdst, 0, sizeof(vdst));
  ShimUtil_memcpy_v(vdst, src, 16);
  for (i = 0; i < 16; i++)
  {
    expectU("memcpy_v copies every byte", vdst[i], src[i]);
  }

  ShimUtil_memset_v(vsrc, 0, sizeof(vsrc));
  ShimUtil_memcpy_v(vsrc, src, 16);
  ShimUtil_memset_v(vdst, 0, sizeof(vdst));
  ShimUtil_memcpy_vv(vdst, vsrc, 16);
  for (i = 0; i < 16; i++)
  {
    expectU("memcpy_vv copies every byte", vdst[i], vsrc[i]);
  }

  /* Zero length must touch nothing. */
  ShimUtil_memset_v(vdst, 0x00, 16);
  ShimUtil_memcpy_v(vdst, src, 0);
  expectU("zero-length copy writes nothing", vdst[0], 0);

  /* Overlap, destination ABOVE source - the safe direction. The loop writes
   * index n only after it has read index n, and works downwards, so every
   * source byte is still intact when it is read. The result is a clean shift
   * up by one. */
  for (i = 0; i < 8; i++)
  {
    vdst[i] = (uint8_t) (i + 1);
  }
  ShimUtil_memcpy_vv(&vdst[1], &vdst[0], 7);
  expectU("overlap upwards shifts cleanly [1]", vdst[1], 1);
  expectU("overlap upwards shifts cleanly [4]", vdst[4], 4);
  expectU("overlap upwards shifts cleanly [7]", vdst[7], 7);

  /* PINNED, NOT ENDORSED. Overlap with the destination BELOW the source is the
   * unsafe direction: working downwards, vdst[6] is overwritten before it is
   * read as the source for vdst[5], so the top byte smears over the range. No
   * caller overlaps today. This assertion is here so that if one ever does, it
   * is a failing test rather than a corrupted buffer - and so nobody "fixes"
   * the loop direction without seeing the case above break. */
  for (i = 0; i < 8; i++)
  {
    vdst[i] = (uint8_t) (i + 1);
  }
  ShimUtil_memcpy_vv(&vdst[0], &vdst[1], 7);
  expectU("overlap downwards smears the top byte [0]", vdst[0], 8);
  expectU("overlap downwards smears the top byte [3]", vdst[3], 8);
  expectU("overlap downwards smears the top byte [6]", vdst[6], 8);
}

/*
 * ShimUtil_strlen_v() is a bounded strlen for volatile buffers. Every Bluetooth
 * response length in Comms/shimmer_bt_uart.c comes from it, always with
 * sizeof(btRxBuffFullResponse) as the bound, so the unterminated case is not
 * hypothetical - it is what a truncated or garbled module response looks like.
 */
static void test_strlen_v(void)
{
  static volatile char buf[32];

  testCase("test_strlen_v");

  ShimUtil_memset_v(buf, 0, sizeof(buf));
  expectU("empty string is 0", ShimUtil_strlen_v(buf, sizeof(buf)), 0);

  ShimUtil_memcpy_v(buf, "AOK", 4);
  expectU("counts up to the NUL", ShimUtil_strlen_v(buf, sizeof(buf)), 3);

  /* No terminator anywhere: it must stop at the bound rather than run off the
   * end of the buffer. This is the case that protects the BT receive path. */
  ShimUtil_memset_v(buf, 'x', sizeof(buf));
  expectU("unterminated stops at the bound",
      ShimUtil_strlen_v(buf, sizeof(buf)), sizeof(buf));

  /* A NUL exactly at the bound is out of scope - the scan never reads it. */
  ShimUtil_memset_v(buf, 'x', sizeof(buf));
  buf[8] = 0;
  expectU("respects a shorter bound", ShimUtil_strlen_v(buf, 4), 4);
  expectU("finds the NUL within a longer bound", ShimUtil_strlen_v(buf, 32), 8);

  /* NOTE for anyone widening a buffer: the loop counter inside
   * ShimUtil_strlen_v() is a uint16_t while maxSize is a size_t. On Shimmer3R,
   * where size_t is 32-bit, a bound above 65535 would never terminate. No
   * caller is near that today - the largest is RESPONSE_PACKET_SIZE, 1024 - but
   * if one ever is, the counter has to be widened with it. */
}

/*
 * ShimUtil_reverseArray()'s caller is the Bluetooth MAC address: the module
 * reports it in one byte order and the host protocol wants the other.
 */
static void test_reverse_array(void)
{
  uint8_t mac[6] = { 0x00, 0x06, 0x66, 0x12, 0x34, 0x56 };
  uint8_t odd[5] = { 1, 2, 3, 4, 5 };
  uint8_t one[1] = { 9 };
  uint8_t expectedMac[6] = { 0x56, 0x34, 0x12, 0x66, 0x06, 0x00 };
  uint8_t i;

  testCase("test_reverse_array");

  ShimUtil_reverseArray(mac, sizeof(mac));
  for (i = 0; i < 6; i++)
  {
    expectX("MAC bytes reversed", mac[i], expectedMac[i]);
  }

  /* Odd length: the middle element stays put. */
  ShimUtil_reverseArray(odd, sizeof(odd));
  expectU("odd length [0]", odd[0], 5);
  expectU("odd length keeps the middle", odd[2], 3);
  expectU("odd length [4]", odd[4], 1);

  ShimUtil_reverseArray(one, 1);
  expectU("single element is unchanged", one[0], 9);

  /* Zero length must not touch the buffer or underflow the loop bound. */
  ShimUtil_reverseArray(odd, 0);
  expectU("zero length is a no-op", odd[0], 5);
}

int main(void)
{
  printf("shimmer_util host tests\n\n");
  hostTestSilenceUnused();

  test_itoa_with_zeros();
  test_itoa_no_zeros();
  test_atol64();
  test_volatile_memory_helpers();
  test_strlen_v();
  test_reverse_array();

  return hostTestReport("shimmer_util");
}

#endif /* SHIMMER_HOST_TEST */
