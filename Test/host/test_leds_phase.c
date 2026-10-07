/*
 * Host-side test for LEDs/shimmer_leds_phase.c (DEV-338).
 *
 * The LED blink phase is derived from the real-world clock so that sensors set
 * from the same source flash on the same frame boundaries, and SD-sync nodes
 * flash with their centre. What is pinned here is the arithmetic every sensor
 * must agree on: which tenth of the 60 s frame a clock reading falls in, how far
 * a tick landed from the middle of its tenth, how a node's reading converts to
 * the centre's, and that every pattern repeats cleanly across the frame wrap.
 *
 * shimmer_leds_phase.c includes nothing but <stdint.h>, so this needs no stubs.
 * Run by .github/workflows/host-tests.yml.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include "host_test.h"

#include "LEDs/shimmer_leds_phase.h"

#define TICKS_PER_SEC   32768ULL
#define TICKS_PER_FRAME (60ULL * TICKS_PER_SEC)

/* Clock readings spread across the range a sensor can hold: boot-relative
 * (Shimmer3 before its clock is set), present day, the 32-bit seconds wrap in
 * 2106, and far beyond it. */
static const uint64_t sampleTicks[] = {
  0ULL,
  12345ULL,
  1791331217ULL * TICKS_PER_SEC + 11469ULL,
  (0x100000000ULL * TICKS_PER_SEC) - 1ULL,
  0x100000000ULL * TICKS_PER_SEC,
  0x0000FFFFFFFFFFFFULL,
};
#define SAMPLE_COUNT (sizeof(sampleTicks) / sizeof(sampleTicks[0]))

static void test_frame_tenth_boundaries(void)
{
  testCase("frame tenth: the tenth boundary is 3276.8 ticks");
  expectU("0 ticks", ShimLedsPhase_frameTenth(0), 0);
  expectU("3276 ticks is still tenth 0", ShimLedsPhase_frameTenth(3276), 0);
  expectU("3277 ticks is tenth 1", ShimLedsPhase_frameTenth(3277), 1);
  expectU("1 s", ShimLedsPhase_frameTenth(TICKS_PER_SEC), 10);
  expectU("one tick before the frame wraps",
      ShimLedsPhase_frameTenth(TICKS_PER_FRAME - 1), 599);
  expectU("60 s wraps to 0", ShimLedsPhase_frameTenth(TICKS_PER_FRAME), 0);

  /* 1791331200 is a whole minute, so 17.35 s past it is tenth 173 */
  expectU("present-day reading",
      ShimLedsPhase_frameTenth(1791331217ULL * TICKS_PER_SEC + 11469ULL), 173);
}

static void test_frame_tenth_is_continuous(void)
{
  unsigned i;

  testCase(
      "frame tenth: one second on is ten tenths on, one minute on is the same");
  for (i = 0; i < SAMPLE_COUNT; i++)
  {
    uint64_t t = sampleTicks[i];
    uint16_t now = ShimLedsPhase_frameTenth(t);
    expectU("+1 s", ShimLedsPhase_frameTenth(t + TICKS_PER_SEC), (now + 10U) % 600U);
    expectU("+60 s", ShimLedsPhase_frameTenth(t + TICKS_PER_FRAME), now);
    expectTrue("in range", now < LED_PHASE_TENTHS_PER_FRAME);
  }
}

static void test_ticks_past_mid_tenth(void)
{
  testCase("phase error: signed ticks from the middle of the tenth");
  expectU("start of a tenth is 1638 early",
      (uint64_t) (-ShimLedsPhase_ticksPastMidTenth(0)), 1638);
  expectU("1638 ticks is the middle", (uint64_t) ShimLedsPhase_ticksPastMidTenth(1638), 0);
  expectU("end of a tenth is 1637 late",
      (uint64_t) ShimLedsPhase_ticksPastMidTenth(3276), 1637);
  /* 1803 ticks is 9015 fifths, 823 past the middle: 164.6 ticks, truncated */
  expectU("5 ms late", (uint64_t) ShimLedsPhase_ticksPastMidTenth(1803), 164);
  expectU("the next tenth starts early again",
      (uint64_t) (-ShimLedsPhase_ticksPastMidTenth(3277)), 1638);

  /* The same position within a tenth, a long way from the epoch */
  expectU("mid-tenth on a present-day reading",
      (uint64_t) ShimLedsPhase_ticksPastMidTenth(1791331200ULL * TICKS_PER_SEC + 1638ULL), 0);
}

static void test_to_centre_time(void)
{
  /* Sign byte, then the magnitude LSB first - the layout ShimSdSync_rcFindSmallest
   * publishes. 0x0102030405060708 puts a distinct value in every byte, so a byte
   * read from the wrong place cannot cancel out. */
  const uint8_t nodeAhead[LED_PHASE_SYNC_OFFSET_SIZE]
      = { 0, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01 };
  const uint8_t nodeBehind[LED_PHASE_SYNC_OFFSET_SIZE]
      = { 1, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01 };
  const uint8_t noOffset[LED_PHASE_SYNC_OFFSET_SIZE] = { 0 };
  const uint64_t local = 0x1000000000000000ULL;

  testCase("SD-sync offset: a node converts its reading to the centre's");
  expectX("node ahead: centre is behind it",
      ShimLedsPhase_toCentreTime(local, nodeAhead), local - 0x0102030405060708ULL);
  expectX("node behind: centre is ahead of it",
      ShimLedsPhase_toCentreTime(local, nodeBehind), local + 0x0102030405060708ULL);
  expectX("zero offset", ShimLedsPhase_toCentreTime(local, noOffset), local);
}

static void test_node_and_centre_agree(void)
{
  /* The point of the whole feature: a node whose clock reads 7.3 s ahead of
   * the centre lands in the same tenth once the offset is applied. */
  const uint64_t centre = 1791331217ULL * TICKS_PER_SEC + 11469ULL;
  const uint64_t ahead = 7ULL * TICKS_PER_SEC + 9830ULL;
  const uint8_t offset[LED_PHASE_SYNC_OFFSET_SIZE] = { 0, (uint8_t) (ahead & 0xFF),
    (uint8_t) ((ahead >> 8) & 0xFF), (uint8_t) ((ahead >> 16) & 0xFF), 0, 0, 0, 0, 0 };

  testCase("SD-sync offset: node and centre land in the same tenth");
  expectTrue("without the offset they differ",
      ShimLedsPhase_frameTenth(centre + ahead) != ShimLedsPhase_frameTenth(centre));
  expectU("with it they agree",
      ShimLedsPhase_frameTenth(ShimLedsPhase_toCentreTime(centre + ahead, offset)),
      ShimLedsPhase_frameTenth(centre));
}

static void test_patterns(void)
{
  testCase("patterns: the level each replaced toggle produces");
  expectU("200 ms: tenth 0 off", ShimLedsPhase_isOnAlternate200ms(0), 0);
  expectU("200 ms: tenth 1 off", ShimLedsPhase_isOnAlternate200ms(1), 0);
  expectU("200 ms: tenth 2 on", ShimLedsPhase_isOnAlternate200ms(2), 1);
  expectU("200 ms: tenth 3 on", ShimLedsPhase_isOnAlternate200ms(3), 1);

  expectU("1 s: second 0 on", ShimLedsPhase_isOnAlternateSecond(0), 1);
  expectU("1 s: tenth 9 still on", ShimLedsPhase_isOnAlternateSecond(9), 1);
  expectU("1 s: second 1 off", ShimLedsPhase_isOnAlternateSecond(10), 0);

  expectU("connected+logging: second 0 blue",
      ShimLedsPhase_isGreenConnectedAndLogging(0), 0);
  expectU("connected+logging: second 1 blue",
      ShimLedsPhase_isGreenConnectedAndLogging(10), 0);
  expectU("connected+logging: second 2 green",
      ShimLedsPhase_isGreenConnectedAndLogging(20), 1);

  expectU("logging+streaming: second 0 green",
      ShimLedsPhase_loggingAndStreamingColour(0), LED_PHASE_COLOUR_GREEN);
  expectU("logging+streaming: second 1 off",
      ShimLedsPhase_loggingAndStreamingColour(10), LED_PHASE_COLOUR_OFF);
  expectU("logging+streaming: second 2 blue",
      ShimLedsPhase_loggingAndStreamingColour(20), LED_PHASE_COLOUR_BLUE);
  expectU("logging+streaming: second 3 off",
      ShimLedsPhase_loggingAndStreamingColour(30), LED_PHASE_COLOUR_OFF);
}

static void test_patterns_survive_the_frame_wrap(void)
{
  /* Each pattern must repeat with its own period straight across tenth 599 to
   * tenth 0; otherwise every sensor would show a glitch once a minute. That
   * holds only because 600 tenths is a multiple of every period. */
  uint16_t t;

  testCase("patterns: every period divides the frame, so the wrap is seamless");
  for (t = 0; t < LED_PHASE_TENTHS_PER_FRAME; t++)
  {
    expectU("200 ms period", ShimLedsPhase_isOnAlternate200ms(t),
        ShimLedsPhase_isOnAlternate200ms((uint16_t) ((t + 4U) % 600U)));
    expectU("2 s period", ShimLedsPhase_isOnAlternateSecond(t),
        ShimLedsPhase_isOnAlternateSecond((uint16_t) ((t + 20U) % 600U)));
    expectU("3 s period", ShimLedsPhase_isGreenConnectedAndLogging(t),
        ShimLedsPhase_isGreenConnectedAndLogging((uint16_t) ((t + 30U) % 600U)));
    expectU("4 s period", ShimLedsPhase_loggingAndStreamingColour(t),
        ShimLedsPhase_loggingAndStreamingColour((uint16_t) ((t + 40U) % 600U)));
  }
  expectTrue("frame is a multiple of the 2 s blinkCnt20 cycle",
      LED_PHASE_TENTHS_PER_FRAME % 20U == 0);
  expectTrue("frame is a multiple of the 5 s blinkCnt50 cycle",
      LED_PHASE_TENTHS_PER_FRAME % 50U == 0);
}

int main(void)
{
  hostTestSilenceUnused();

  test_frame_tenth_boundaries();
  test_frame_tenth_is_continuous();
  test_ticks_past_mid_tenth();
  test_to_centre_time();
  test_node_and_centre_agree();
  test_patterns();
  test_patterns_survive_the_frame_wrap();

  return hostTestReport("test_leds_phase");
}

#endif /* SHIMMER_HOST_TEST */
