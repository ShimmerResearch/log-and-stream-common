/*
 * Host-side test for the software CRC.
 *
 * Builds with a plain host compiler: shimmer_swCrc.c depends on nothing but
 * <stdint.h>, so this needs no ARM toolchain, no HAL and no target hardware.
 * Run by .github/workflows/host-tests.yml.
 *
 * Two modes:
 *   (no args)  run the assertions, exit non-zero if any failed
 *   --dump     print "len,crc" for a reproducible corpus, for the Python
 *              cross-check in crosscheck_swcrc.py to compare against the
 *              host reference implementation
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions (see the
 * log-and-stream-common <entry kind="sourcePath"> in the Shimmer3R .cproject),
 * so every .c file under it is discovered and compiled into the firmware. This
 * file defines main(). Without the guard it collides with the firmware's own
 * main() at link time, in both the STM32 and the MSP430 build. The guard makes
 * the translation unit empty for anyone who does not ask for it; only the CI
 * workflow passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "shimmer_crc.h"
#include "shimmer_swCrc.h"

#define CORPUS_MAX 600

/* Deterministic and trivially reproducible in any language - deliberately not
 * a PRNG, so the Python side cannot drift from this by seeding differently. */
static void fill_corpus(uint8_t *buf, int n)
{
  int i;
  for (i = 0; i < n; i++)
  {
    buf[i] = (uint8_t) ((i * 7 + 13) & 0xFF);
  }
}

static int failures;

/* uint16_t, with explicit casts at the printf: uint32_t is unsigned long on
 * ARM and unsigned int on x86-64, so a bare %04X on a uint32_t is only
 * accidentally correct on whichever of the two you happen to build for. */
static void expect(const char *what, uint16_t got, uint16_t want)
{
  if (got != want)
  {
    printf("  FAIL %-28s got 0x%04X want 0x%04X\n", what, (unsigned) got, (unsigned) want);
    failures++;
  }
}

int main(int argc, char **argv)
{
  static uint8_t buf[CORPUS_MAX];
  uint8_t seq[16];
  int i;

  for (i = 0; i < 16; i++)
  {
    seq[i] = (uint8_t) (i + 1);
  }
  fill_corpus(buf, CORPUS_MAX);

  if (argc > 1 && strcmp(argv[1], "--dump") == 0)
  {
    for (i = 1; i <= CORPUS_MAX; i++)
    {
      printf("%d,%04X\n", i, (unsigned) ShimSwCrc_calc16(buf, (uint16_t) i));
    }
    return 0;
  }

  /* 1. The wire-format vectors. These are the contract: anything that moves
   * one of them changes what goes out on the Bluetooth and dock links. */
  expect("calc len 8", ShimSwCrc_calc(seq, 8), 0x48AA);
  expect("calc len 9", ShimSwCrc_calc(seq, 9), 0x2A5D);
  expect("calc len 10", ShimSwCrc_calc(seq, 10), 0x8B17);
  expect("calc len 11", ShimSwCrc_calc(seq, 11), 0x794E);

  /* 2. The 16-bit path, including the lengths a uint8_t cannot express. */
  expect("calc16 len 0", ShimSwCrc_calc16(seq, 0), 0xB0CA);
  expect("calc16 len 1", ShimSwCrc_calc16(seq, 1), 0x553A);
  expect("calc16 len 8", ShimSwCrc_calc16(seq, 8), 0x48AA);
  expect("calc16 len 9", ShimSwCrc_calc16(seq, 9), 0x2A5D);

  /* 3. calc16 must agree with calc wherever both are defined - len 1..255.
   * This is what stops the two implementations drifting apart. */
  for (i = 1; i <= 255; i++)
  {
    uint16_t a = ShimSwCrc_calc(buf, (uint8_t) i);
    uint16_t b = ShimSwCrc_calc16(buf, (uint16_t) i);
    if (a != b)
    {
      printf("  FAIL calc/calc16 disagree at len %d: 0x%04X vs 0x%04X\n", i,
          (unsigned) a, (unsigned) b);
      failures++;
      break;
    }
  }

  /* 4. The one place they must NOT agree. ShimSwCrc_calc() seeds with msg[0]
   * before its loop, so it reads a byte even at len 0; ShimSwCrc_calc16()
   * returns the bare seed. ShimSwCrc_check() calls calc(msg, len - 2), so a
   * 2-byte packet lands exactly here. Asserted so that "simplifying" one into
   * the other fails loudly rather than silently changing 2-byte packets. */
  if (ShimSwCrc_calc(seq, 0) == ShimSwCrc_calc16(seq, 0))
  {
    printf("  FAIL calc and calc16 now agree at len 0 - see the comment on "
           "ShimSwCrc_calc16, this divergence is load-bearing\n");
    failures++;
  }

  /* 5. ShimSwCrc_check() round-trip: append a CRC, verify it reads back, and
   * verify a single flipped bit is rejected. */
  {
    uint8_t pkt[32];
    uint16_t crc;
    for (i = 0; i < 30; i++)
    {
      pkt[i] = (uint8_t) (i + 1);
    }
    crc = ShimSwCrc_calc(pkt, 30);
    pkt[30] = (uint8_t) (crc & 0xFF);
    pkt[31] = (uint8_t) ((crc >> 8) & 0xFF);
    if (!ShimSwCrc_check(pkt, 32))
    {
      printf("  FAIL ShimSwCrc_check rejected a CRC it had just produced\n");
      failures++;
    }
    pkt[5] ^= 0x01;
    if (ShimSwCrc_check(pkt, 32))
    {
      printf("  FAIL ShimSwCrc_check accepted a corrupted packet\n");
      failures++;
    }
  }

  if (failures)
  {
    printf("\nsoftware CRC: %d FAILURE(S)\n", failures);
    return 1;
  }
  printf("\nsoftware CRC: all checks passed\n");
  return 0;
}

#endif /* SHIMMER_HOST_TEST */
