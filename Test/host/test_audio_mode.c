/*
 * Host-side test for Configuration/shimmer_audio_mode.c.
 *
 * Configuration byte 233 says whether a Shimmer3R records audio while it logs,
 * and what it keeps (DEV-99). Its rules are safety rules more than decoding: a
 * byte that no host meant to write - erased flash, a host that zeroes what it
 * does not know, a value from a newer host - must read as off, so a unit never
 * records audio nobody asked for. And a mode this firmware does not record
 * must read back as off, so a host that writes it learns the unit cannot. Each
 * of those is pinned here, over every one of the 256 values.
 *
 * The field values are also a contract with every host that writes the byte
 * and every parser that reads its copy in the SD header, so they are pinned
 * as numbers.
 *
 * shimmer_audio_mode.c includes nothing but <stdint.h>, so this needs no
 * stubs. Run by .github/workflows/host-tests.yml.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include "host_test.h"

#include "Configuration/shimmer_audio_mode.h"

/* Restated rather than built from the header's masks, so a mask that moves
 * fails here instead of moving the test with it */
static int standsAsWritten(uint8_t raw)
{
  uint8_t mode = raw & 0x03U;
  uint8_t gain = (uint8_t) ((raw >> 2) & 0x03U);

  return !(raw & 0x80U) && (gain == 0U) && (AUDIO_MODES_BUILT & (1U << mode));
}

static void test_constants(void)
{
  testCase("the field values are the documented ones");
  expectX("off", AUDIO_MODE_OFF, 0);
  expectX("fingerprint only", AUDIO_MODE_FINGERPRINT, 1);
  expectX("fingerprint and raw audio", AUDIO_MODE_FINGERPRINT_RAW, 2);
  expectX("raw audio only", AUDIO_MODE_RAW, 3);
  expectX("default gain", AUDIO_GAIN_DEFAULT, 0);
  expectX("mode mask", AUDIO_MODE_MASK, 0x03);
  expectX("gain mask", AUDIO_MODE_GAIN_MASK, 0x0C);
  expectX("gain shift", AUDIO_MODE_GAIN_SHIFT, 2);
  expectX("must-be-0 bit", AUDIO_MODE_INVALID, 0x80);
  expectX("off is always built", AUDIO_MODES_BUILT & 1U, 1);
}

static void test_correct_every_value(void)
{
  unsigned v;
  uint8_t raw, fixed;

  testCase("every value either stands or reads as off");
  for (v = 0; v < 256U; v++)
  {
    raw = (uint8_t) v;
    fixed = ShimAudioMode_correct(raw);
    if (standsAsWritten(raw))
    {
      /* Bits 4-6 are not used, so they go */
      expectX("valid built value kept, bits 4-6 cleared", fixed, raw & 0x0FU);
    }
    else
    {
      expectX("anything else reads as off", fixed, 0);
    }
    expectX("never more than bits 0-3", fixed & 0xF0U, 0);
    expectX("correcting twice changes nothing", ShimAudioMode_correct(fixed), fixed);
  }
}

static void test_values_no_host_meant(void)
{
  testCase("erased flash, a zeroing host and reserved values all read as off");
  expectX("0xFF (erased flash)", ShimAudioMode_correct(0xFF), 0);
  expectX("0x00 (host that writes 0)", ShimAudioMode_correct(0x00), 0);
  expectX("raw audio, bit 7 set", ShimAudioMode_correct(0x83), 0);
  expectX("fingerprint, gain preset 1 (reserved)", ShimAudioMode_correct(0x05), 0);
  expectX("raw audio, gain preset 3 (reserved)", ShimAudioMode_correct(0x0F), 0);
}

static void test_queries_apply_the_correction(void)
{
  unsigned v;
  uint8_t raw, mode;

  testCase("queries read the corrected byte, never a mode that is not built");
  for (v = 0; v < 256U; v++)
  {
    raw = (uint8_t) v;
    mode = ShimAudioMode_mode(raw);
    expectX("mode is the corrected byte's", mode, ShimAudioMode_correct(raw) & 0x03U);
    expectU("only built modes come back", (AUDIO_MODES_BUILT >> mode) & 1U, 1);
    expectX("gain preset is the corrected byte's", ShimAudioMode_gainPreset(raw),
        (ShimAudioMode_correct(raw) >> 2) & 0x03U);
    expectU("fingerprint in modes 1 and 2 only",
        ShimAudioMode_recordsFingerprint(raw), (mode == 1U) || (mode == 2U));
    expectU("raw audio in modes 2 and 3 only",
        ShimAudioMode_recordsRawAudio(raw), (mode == 2U) || (mode == 3U));
  }
  expectU("off records no fingerprint", ShimAudioMode_recordsFingerprint(0), 0);
  expectU("off records no raw audio", ShimAudioMode_recordsRawAudio(0), 0);
}

static void test_sdlog_cfg_round_trip(void)
{
  unsigned v;
  uint8_t fixed, parsed;

  testCase("the sdlog.cfg key sets the mode and nothing else");
  expectX("audio_mode=3", ShimAudioMode_withMode(0, 3), 0x03);
  expectX("audio_mode=1 replaces, not ORs", ShimAudioMode_withMode(0x03, 1), 0x01);
  expectX("the gain preset stays", ShimAudioMode_withMode(0x04, 2), 0x06);

  testCase("a value outside the field makes the whole byte off");
  expectX("audio_mode=4", ShimAudioMode_correct(ShimAudioMode_withMode(0x01, 4)), 0);
  expectX("audio_mode=-1", ShimAudioMode_correct(ShimAudioMode_withMode(0x01, -1)), 0);
  /* atoi() of anything that is not a number */
  expectX("audio_mode=0 from text", ShimAudioMode_withMode(0x02, 0), 0x00);

  testCase("what generate writes, parse reads back");
  for (v = 0; v < 256U; v++)
  {
    fixed = ShimAudioMode_correct((uint8_t) v);
    /* The parse starts from ShimConfig_createBlankConfigBytes(), which sets
     * the byte to 0. The gain preset has no key until one is built. */
    parsed = ShimAudioMode_withMode(0, ShimAudioMode_mode(fixed));
    expectX("round trip", ShimAudioMode_correct(parsed), fixed);
  }
}

int main(void)
{
  printf("shimmer_audio_mode host tests\n\n");
  hostTestSilenceUnused();

  test_constants();
  test_correct_every_value();
  test_values_no_host_meant();
  test_queries_apply_the_correction();
  test_sdlog_cfg_round_trip();

  return hostTestReport("shimmer_audio_mode");
}

#endif /* SHIMMER_HOST_TEST */
