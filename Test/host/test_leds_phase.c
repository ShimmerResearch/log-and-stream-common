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
 * Then the lock itself, closed loop, against a blink timer that drifts from the
 * clock as Shimmer3R's does (DEV-1417): every whole-second tick must come once
 * per second, which the auto-stop and the 1 s patterns depend on.
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

/* Both helpers avoid 64-bit division and multiplication, which are slow
 * enough on the MSP430 to cost Shimmer3 a sample a second (DEV-1386). They
 * must still give exactly what the plain 64-bit formula gives, anywhere in the
 * range a clock can hold. */
static void test_matches_the_64_bit_formula(void)
{
  uint64_t x = 0x9E3779B97F4A7C15ULL;
  unsigned i, bad = 0;

  testCase("frame tenth and phase error match the 64-bit formula");
  for (i = 0; i < 200000U; i++)
  {
    uint64_t t;
    int32_t fifths;

    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    /* Mostly readings up to 2^48 ticks (272 years), some anywhere a product
     * by 5 still fits */
    t = (i & 7U) ? (x >> 16) : (x / 5U);
    if (i < SAMPLE_COUNT)
    {
      t = sampleTicks[i];
    }
    fifths = (int32_t) ((t * 5U) & (LED_PHASE_FIFTHS_PER_TENTH - 1U));
    if (ShimLedsPhase_frameTenth(t) != (uint16_t) (((t * 5U) >> 14) % 600U)
        || ShimLedsPhase_ticksPastMidTenth(t)
            != (int16_t) ((fifths - (int32_t) LED_PHASE_MID_TENTH_FIFTHS) / 5))
    {
      bad++;
    }
  }
  expectU("readings that disagree", bad, 0);
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

static void test_correction(void)
{
  LedPhaseCorrection_t c;

  testCase("correction: the tenth to hold and the shift, with a deadband");
  c = ShimLedsPhase_correction(TICKS_PER_SEC + 1638ULL);
  expectU("mid-tenth: tenth 10", c.frameTenth, 10);
  expectU("mid-tenth: no shift", (uint64_t) (uint16_t) c.shiftTicks, 0);

  /* 164 ticks late is inside the deadband, 165 is not. The middle is 1638.4
   * ticks in, so those are 1803 and 1804 ticks into the tenth. */
  c = ShimLedsPhase_correction(TICKS_PER_SEC + 1803ULL);
  expectU("164 ticks late: no shift", (uint64_t) (uint16_t) c.shiftTicks, 0);
  c = ShimLedsPhase_correction(TICKS_PER_SEC + 1804ULL);
  expectU("165 ticks late: moved 165 earlier", (uint64_t) (-c.shiftTicks), 165);
  c = ShimLedsPhase_correction(TICKS_PER_SEC + 1638ULL - 200ULL);
  expectU("200 ticks early: moved 200 later", (uint64_t) c.shiftTicks, 200);

  /* A tick early enough to fall in the previous tenth is renumbered into it.
   * This is what turns a fast timer into a second counted twice. */
  c = ShimLedsPhase_correction(TICKS_PER_SEC - 10ULL);
  expectU("just before a whole second: tenth 9", c.frameTenth, 9);
}

static void test_counter_timer_shift(void)
{
  LedPhaseTimerShift_t s;

  testCase(
      "counter timer: a delay lengthens one period, the counter is left alone");
  s = ShimLedsPhase_counterTimerShift(0, 79, 24);
  expectU("counter 0, 24 later: counter unchanged", s.counter, 0);
  expectU("counter 0, 24 later: next period +24", s.periodExtra, 24);
  s = ShimLedsPhase_counterTimerShift(30, 79, 24);
  expectU("counter 30, 24 later: counter unchanged", s.counter, 30);
  expectU("counter 30, 24 later: next period +24", s.periodExtra, 24);
  s = ShimLedsPhase_counterTimerShift(0, 79, 200);
  expectU("more than a period: capped at one period", s.periodExtra, 79);

  testCase(
      "counter timer: earlier steps the counter forward, up to the reload");
  s = ShimLedsPhase_counterTimerShift(2, 79, -24);
  expectU("counter 2, 24 earlier: counter 26", s.counter, 26);
  expectU("counter 2, 24 earlier: period unchanged", s.periodExtra, 0);
  s = ShimLedsPhase_counterTimerShift(70, 79, -24);
  expectU("counter 70, 24 earlier: clipped at the reload", s.counter, 79);
  s = ShimLedsPhase_counterTimerShift(79, 79, -24);
  expectU("counter on its last count: left alone", s.counter, 79);
  s = ShimLedsPhase_counterTimerShift(90, 79, -24);
  expectU("inside a lengthened period: left alone", s.counter, 90);
  s = ShimLedsPhase_counterTimerShift(5, 79, -32768);
  expectU("the most negative shift: clipped, no overflow", s.counter, 79);
  s = ShimLedsPhase_counterTimerShift(5, 79, 0);
  expectU("no shift: counter unchanged", s.counter, 5);
  expectU("no shift: period unchanged", s.periodExtra, 0);
}

/* --- closed loop -------------------------------------------------------------
 *
 * The lock against a blink timer that does not run from the RTC's crystal:
 * Shimmer3R's TIM6, on the MSI (DEV-1417). The model is a counter at 800 Hz,
 * scaled by the timer's error, reloading after 80 counts with ARR preload, a
 * tick at each reload, and the sync a short latency after each whole-second
 * tick, as TASK_LED_PHASE_SYNC runs. What the tick and the sync do mirrors
 * LogAndStream_blinkTimerCommon(), ShimLeds_phaseSync() and tim.c's
 * platform_ledTickElapsedRtcTicks(), ledBlinkTimerCallback() and
 * platform_ledTickShift(), with the decisions made by the two functions under
 * test above.
 *
 * Time is in millionths of an RTC tick, so a timer error of a few ppm is not
 * lost to rounding. */
#define TIM_RELOAD         79U
#define TIM_RTC_TICKS_X100 4096 /* tim.c TIM6_RTC_TICKS_PER_COUNT_X100 */
#define UTICKS_PER_TICK    1000000ULL
#define UTICKS_PER_SEC     (TICKS_PER_SEC * UTICKS_PER_TICK)

typedef enum
{
  SHIFT_FIXED,  /* ShimLedsPhase_counterTimerShift() */
  SHIFT_CLIPPED /* what platform_ledTickShift did before DEV-1417 */
} shift_model_t;

typedef struct
{
  uint64_t elapsedUticks; /* RTC time across secsWanted whole-second ticks */
  unsigned mislabelled;   /* ticks whose tenth disagreed with the clock */
} lock_run_t;

/* Runs the lock with the timer `ppm` fast (negative: slow) and the sync
 * `latencyUs` after each whole-second tick. Lets it settle for 10 s, then
 * measures how much RTC time `secsWanted` whole-second ticks span. */
static lock_run_t runLock(int32_t ppm, uint32_t latencyUs, shift_model_t model, uint32_t secsWanted)
{
  const uint64_t t0 = 1791331217ULL * TICKS_PER_SEC + 11469ULL;
  const uint64_t countPeriod = (uint64_t) TIM_RTC_TICKS_X100 * UTICKS_PER_TICK
      / 100ULL * 1000000ULL / (uint64_t) (1000000 + ppm);
  const uint64_t latency = (uint64_t) latencyUs * UTICKS_PER_SEC / 1000000ULL;
  uint64_t t = 0, syncAt = 0, firstSecond = 0;
  uint16_t counter = 0, reloadActive = TIM_RELOAD, reloadPreload = TIM_RELOAD;
  uint16_t frame = ShimLedsPhase_frameTenth(t0);
  uint32_t secs = 0;
  uint8_t syncPending = 0, measuring = 0;
  lock_run_t run = { 0, 0 };

  for (;;)
  {
    uint64_t edge = t + countPeriod;

    if (syncPending && syncAt < edge)
    {
      /* ShimLeds_phaseSync() */
      uint64_t now = t0 + syncAt / UTICKS_PER_TICK;
      uint16_t elapsed = (uint16_t) (((uint32_t) counter * TIM_RTC_TICKS_X100) / 100U);
      LedPhaseCorrection_t c = ShimLedsPhase_correction(now - elapsed);

      syncPending = 0;
      frame = c.frameTenth;
      if (c.shiftTicks != 0)
      {
        /* platform_ledTickShift() */
        int32_t counts = ((int32_t) c.shiftTicks * 100) / TIM_RTC_TICKS_X100;
        if (counts == 0)
        {
          continue;
        }
        if (model == SHIFT_FIXED)
        {
          LedPhaseTimerShift_t s
              = ShimLedsPhase_counterTimerShift(counter, TIM_RELOAD, (int16_t) counts);
          counter = s.counter;
          if (s.periodExtra != 0U)
          {
            reloadPreload = (uint16_t) (TIM_RELOAD + s.periodExtra);
          }
        }
        else if (counter < TIM_RELOAD)
        {
          int32_t cnt = (int32_t) counter - counts;
          if (cnt < 0)
          {
            cnt = 0;
          }
          else if (cnt > (int32_t) TIM_RELOAD)
          {
            cnt = (int32_t) TIM_RELOAD;
          }
          counter = (uint16_t) cnt;
        }
      }
      continue;
    }

    t = edge;
    if (counter < reloadActive)
    {
      counter++;
      continue;
    }

    /* The reload: the preload takes effect, then ledBlinkTimerCallback() */
    counter = 0;
    reloadActive = reloadPreload;
    if (model == SHIFT_FIXED)
    {
      reloadPreload = TIM_RELOAD;
    }
    frame = (uint16_t) ((frame + 1U) % LED_PHASE_TENTHS_PER_FRAME);

    if (measuring && frame != ShimLedsPhase_frameTenth(t0 + t / UTICKS_PER_TICK))
    {
      run.mislabelled++;
    }

    if (frame % 10U == 0U)
    {
      syncPending = 1;
      syncAt = t + latency;

      if (!measuring && t >= 10ULL * UTICKS_PER_SEC)
      {
        measuring = 1;
        firstSecond = t;
      }
      else if (measuring && ++secs == secsWanted)
      {
        run.elapsedUticks = t - firstSecond;
        return run;
      }
    }
  }
}

static void test_closed_loop_counts_each_second_once(void)
{
  static const int32_t ppms[] = { -30000, -3000, 0, 100, 1500, 3000, 30000 };
  static const uint32_t latenciesUs[] = { 500, 2000, 20000 };
  unsigned i, j;

  testCase("closed loop: 600 whole-second ticks span 600 s, timer -3% to +3%");
  for (i = 0; i < sizeof(ppms) / sizeof(ppms[0]); i++)
  {
    for (j = 0; j < sizeof(latenciesUs) / sizeof(latenciesUs[0]); j++)
    {
      lock_run_t r = runLock(ppms[i], latenciesUs[j], SHIFT_FIXED, 600);
      char what[80];

      /* Every tick stays in its own tenth, so the first and the last are each
       * less than a tenth from their whole seconds */
      snprintf(what, sizeof(what), "%+ld ppm, sync %lu us late: within 100 ms of 600 s",
          (long) ppms[i], (unsigned long) latenciesUs[j]);
      expectTrue(what,
          r.elapsedUticks > 600ULL * UTICKS_PER_SEC - UTICKS_PER_SEC / 10ULL
              && r.elapsedUticks < 600ULL * UTICKS_PER_SEC + UTICKS_PER_SEC / 10ULL);
      snprintf(what, sizeof(what), "%+ld ppm, sync %lu us late: no tick in the wrong tenth",
          (long) ppms[i], (unsigned long) latenciesUs[j]);
      expectU(what, r.mislabelled, 0);
    }
  }
}

static void test_closed_loop_the_clip_counted_twice(void)
{
  lock_run_t r;

  /* What DEV-1417 found on a bench unit: with the sync this soon after the
   * tick, a timer only 0.15% fast double-counted about one second in 35 */
  testCase("closed loop: the clipped delay it replaces counted seconds twice");
  r = runLock(1500, 500, SHIFT_CLIPPED, 600);
  expectTrue("+0.15%: 600 ticks span under 590 s", r.elapsedUticks < 590ULL * UTICKS_PER_SEC);
  expectTrue("+0.15%: ticks landed in the wrong tenth", r.mislabelled > 0);

  /* ...and even a crystal-accurate timer slips, given long enough */
  r = runLock(100, 500, SHIFT_CLIPPED, 3600);
  expectTrue("+100 ppm: an hour of ticks spans under 3599 s",
      r.elapsedUticks < 3599ULL * UTICKS_PER_SEC);

  /* A slow timer was always fine: stepping forward has room */
  r = runLock(-30000, 500, SHIFT_CLIPPED, 600);
  expectU("-3%: no tick in the wrong tenth", r.mislabelled, 0);
}

int main(void)
{
  hostTestSilenceUnused();

  test_frame_tenth_boundaries();
  test_frame_tenth_is_continuous();
  test_ticks_past_mid_tenth();
  test_matches_the_64_bit_formula();
  test_to_centre_time();
  test_node_and_centre_agree();
  test_patterns();
  test_patterns_survive_the_frame_wrap();
  test_correction();
  test_counter_timer_shift();
  test_closed_loop_counts_each_second_once();
  test_closed_loop_the_clip_counted_twice();

  return hostTestReport("test_leds_phase");
}

#endif /* SHIMMER_HOST_TEST */
