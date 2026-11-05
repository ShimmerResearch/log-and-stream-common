/*
 * Host-side test for Sensing/shimmer_autostop.c (DEV-1417).
 *
 * The max_exp_len auto-stop measures the real-world clock between its
 * once-a-second checks instead of counting the checks. What is pinned here:
 * the log stops at the first check at or after the limit, whatever the
 * cadence of the checks (doubled, missed or drifting, as a blink timer that is
 * not on the crystal produces); a clock set mid-log moves the stop by about a
 * second at most; the 32-bit clock wrapping changes nothing; and the limit
 * covers the whole 16-bit range of minutes.
 *
 * shimmer_autostop.c includes nothing but <stdint.h>, so this needs no stubs.
 * Run by .github/workflows/host-tests.yml.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include "host_test.h"

#include "Sensing/shimmer_autostop.h"

#define TICK_HZ   32768UL
#define SECS(s)   ((uint32_t) ((s) * TICK_HZ))
#define TENTHS(t) ((uint32_t) ((t) * TICK_HZ / 10UL))

/* A present-day clock reading's low 32 bits, an arbitrary phase in */
#define START_RTC 0x5A3C1E77UL

typedef enum
{
  CADENCE_STEADY,   /* one check a second */
  CADENCE_DOUBLED,  /* plus an extra check 0.1 s after every 7th */
  CADENCE_MISSED,   /* every 5th check missed: a 2 s gap */
  CADENCE_FAST_3PC, /* a check every 0.97 s */
  CADENCE_SLOW_3PC  /* a check every 1.03 s */
} cadence_t;

/* Real seconds between log start and the check that stopped it, in ticks, or
 * 0 if no check up to `maxSecs` did. The first check comes `phase` ticks after
 * the start, as the blink timer has no relation to when logging began.
 * `setAt`/`setBy` model a clock set: from real time setAt on, the clock reads
 * setBy ticks off (negative: back). */
static uint64_t
runToStop(uint16_t limitMins, cadence_t cadence, uint32_t phase, uint32_t setAt, int64_t setBy, uint32_t maxSecs)
{
  Autostop_t a;
  uint64_t t = phase;
  uint32_t n = 0;
  uint32_t interval = TICK_HZ;

  if (cadence == CADENCE_FAST_3PC)
  {
    interval = TICK_HZ * 97UL / 100UL;
  }
  else if (cadence == CADENCE_SLOW_3PC)
  {
    interval = TICK_HZ * 103UL / 100UL;
  }

  ShimAutostop_setLimitMins(&a, limitMins);
  ShimAutostop_start(&a, START_RTC);

  while (t < (uint64_t) maxSecs * TICK_HZ)
  {
    uint64_t clock = START_RTC + t + ((setAt && t >= setAt) ? (uint64_t) setBy : 0U);

    n++;
    if (!(cadence == CADENCE_MISSED && n % 5U == 0U))
    {
      if (ShimAutostop_check(&a, (uint32_t) clock))
      {
        return t;
      }
      if (cadence == CADENCE_DOUBLED && n % 7U == 0U)
      {
        clock += TENTHS(1);
        if (ShimAutostop_check(&a, (uint32_t) clock))
        {
          return t + TENTHS(1);
        }
      }
    }
    t += interval;
  }
  return 0;
}

static void test_stops_at_the_limit(void)
{
  uint64_t t;

  testCase("stop: at the first check at or after the limit");
  t = runToStop(7, CADENCE_STEADY, TENTHS(3), 0, 0, 1000);
  expectU("7 min, checks 0.3 s past each second: stops at 420.3 s", t,
      SECS(420) + TENTHS(3));
  t = runToStop(7, CADENCE_STEADY, TICK_HZ, 0, 0, 1000);
  expectU("7 min, checks on the second: stops at 420 s exactly", t, SECS(420));
  t = runToStop(1, CADENCE_STEADY, 1, 0, 0, 1000);
  expectU("1 min, first check 1 tick in: stops at 60 s + 1 tick", t, SECS(60) + 1U);
  t = runToStop(10, CADENCE_STEADY, TENTHS(9), 0, 0, 1000);
  expectU("10 min: stops at 600.9 s", t, SECS(600) + TENTHS(9));
}

static void test_cadence_does_not_matter(void)
{
  /* DEV-1417: TIM6 off the crystal, and a phase lock that could count a
   * second twice. None of it may move the stop by more than one check. */
  uint64_t t;

  testCase("cadence: doubled, missed and drifting checks change nothing");
  t = runToStop(7, CADENCE_DOUBLED, TENTHS(3), 0, 0, 1000);
  expectU("doubled checks: still 420.3 s", t, SECS(420) + TENTHS(3));
  t = runToStop(7, CADENCE_MISSED, TENTHS(3), 0, 0, 1000);
  expectU("missed checks: still 420.3 s", t, SECS(420) + TENTHS(3));
  t = runToStop(7, CADENCE_FAST_3PC, TENTHS(3), 0, 0, 1000);
  expectTrue("checks 3% fast: within one check after 420 s",
      t >= SECS(420) && t < SECS(420) + TICK_HZ);
  t = runToStop(7, CADENCE_SLOW_3PC, TENTHS(3), 0, 0, 1000);
  expectTrue("checks 3% slow: within one check after 420 s",
      t >= SECS(420) && t < SECS(420) + TICK_HZ * 103UL / 100UL);
}

static void test_clock_set_mid_log(void)
{
  /* Shimmer3R's RTC_get32() reads the calendar, which SET_RWC moves */
  uint64_t t;

  testCase("clock set: an hour either way moves the stop by about a second");
  t = runToStop(7, CADENCE_STEADY, TENTHS(3), SECS(200) + TENTHS(5),
      (int64_t) SECS(3600), 1000);
  expectU("an hour forward at 200.5 s: stops a second early, at 419.3 s", t,
      SECS(419) + TENTHS(3));
  t = runToStop(7, CADENCE_STEADY, TENTHS(3), SECS(200) + TENTHS(5),
      -(int64_t) SECS(3600), 1000);
  expectU("an hour back at 200.5 s: stops a second late, at 421.3 s", t,
      SECS(421) + TENTHS(3));
  t = runToStop(7, CADENCE_STEADY, TENTHS(3), SECS(200) + TENTHS(5),
      (int64_t) TENTHS(4), 1000);
  expectU("0.4 s forward: still stops at 420.3 s", t, SECS(420) + TENTHS(3));
  t = runToStop(7, CADENCE_STEADY, TENTHS(3), SECS(200) + TENTHS(5),
      (int64_t) SECS(-1), 1000);
  expectU("a second back: stops at 421.3 s", t, SECS(421) + TENTHS(3));
}

static void test_clock_wrap(void)
{
  Autostop_t a;
  uint32_t i;
  uint8_t stopped = 0;

  testCase("wrap: the low 32 bits rolling over mid-log change nothing");
  ShimAutostop_setLimitMins(&a, 1);
  ShimAutostop_start(&a, 0xFFFFFFFFUL - SECS(30));
  for (i = 1; i <= 59U; i++)
  {
    stopped |= ShimAutostop_check(&a, 0xFFFFFFFFUL - SECS(30) + SECS(i));
  }
  expectU("59 s across the wrap: not stopped", stopped, 0);
  expectU("60 s across the wrap: stopped",
      ShimAutostop_check(&a, 0xFFFFFFFFUL - SECS(30) + SECS(60)), 1);
}

static void test_no_limit(void)
{
  testCase("no limit: 0 minutes never stops");
  expectU("a day of checks", runToStop(0, CADENCE_STEADY, TENTHS(3), 0, 0, 86400), 0);
}

static void test_whole_range(void)
{
  Autostop_t a;

  testCase("range: every limit up to 65535 minutes");
  ShimAutostop_setLimitMins(&a, 1093);
  expectU("1093 min: above the old 16-bit wrap", a.limitSecs, 65580);
  ShimAutostop_setLimitMins(&a, 65535);
  expectU("65535 min: 3932100 s", a.limitSecs, 3932100UL);

  /* 45.5 days of checks, across 32-bit clock wraps */
  expectU("65535 min: stops at 3932100.3 s",
      runToStop(65535, CADENCE_STEADY, TENTHS(3), 0, 0, 4000000UL),
      (uint64_t) 3932100UL * TICK_HZ + TENTHS(3));
}

static void test_restart(void)
{
  Autostop_t a;
  uint32_t i;
  uint8_t stopped = 0;

  testCase("restart: counting begins again from the new start");
  ShimAutostop_setLimitMins(&a, 1);
  ShimAutostop_start(&a, 1000);
  for (i = 1; i <= 59U; i++)
  {
    stopped |= ShimAutostop_check(&a, 1000 + SECS(i));
  }
  expectU("59 s in: not stopped", stopped, 0);

  ShimAutostop_start(&a, 1000 + SECS(59));
  for (i = 60; i <= 118U; i++)
  {
    stopped |= ShimAutostop_check(&a, 1000 + SECS(i));
  }
  expectU("restarted, 59 s later: not stopped", stopped, 0);
  expectU("restarted, 60 s later: stopped", ShimAutostop_check(&a, 1000 + SECS(119)), 1);
}

static void test_one_long_gap(void)
{
  Autostop_t a;

  testCase("clamp: one long gap between checks counts as 2 s");
  ShimAutostop_setLimitMins(&a, 1);
  ShimAutostop_start(&a, 1000);
  expectU("a 59 s gap: not stopped", ShimAutostop_check(&a, 1000 + SECS(59)), 0);
  expectU("...it counted 2 s", a.elapsedSecs, 2);
  expectU("2 s exactly is not clamped", ShimAutostop_check(&a, 1000 + SECS(61)), 0);
  expectU("...4 s now", a.elapsedSecs, 4);
}

int main(void)
{
  hostTestSilenceUnused();

  test_stops_at_the_limit();
  test_cadence_does_not_matter();
  test_clock_set_mid_log();
  test_clock_wrap();
  test_no_limit();
  test_whole_range();
  test_restart();
  test_one_long_gap();

  return hostTestReport("test_autostop");
}

#endif /* SHIMMER_HOST_TEST */
