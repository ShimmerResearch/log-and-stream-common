/*
 * Host-side test for the RTC conversions in RTC/shimmer_rtc.c.
 *
 * Built against the stub platform in Test/host/stubs - see stubs/README.md.
 * Run by .github/workflows/host-tests.yml, and cross-checked against Python's
 * datetime by crosscheck_rtc.py.
 *
 * WHAT THIS IS FOR. Four pure functions decide what time every recording claims
 * to have been made at:
 *
 *   ShimRtc_rtc2Unix     calendar -> seconds, when the host sets the clock
 *   ShimRtc_unix2Rtc     seconds -> calendar
 *   ShimRtc_ticks2Rtc    32768 Hz ticks -> calendar, for SD file timestamps
 *   ShimRtc_isDateValid  the gate that rejects a nonsense clock at boot
 *
 * A fault in any of them is invisible on the device - the data looks fine, the
 * LEDs look fine - and only shows up when a recording is lined up against
 * something else and is a day, a month or a year out. The first three are
 * exercised here over the whole 2000-2099 range the hardware can express, which
 * costs a few milliseconds and is the only way the leap-year and month-boundary
 * arithmetic gets covered at all.
 *
 * Two modes:
 *   (no args)  run the assertions, exit non-zero if any failed
 *   --dump     print "unix,YYYY-MM-DD HH:MM:SS,weekday" for crosscheck_rtc.py
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

/* 2000-01-01 00:00:00 UTC. The RTC stores a two-digit year, so this is the
 * earliest instant it can express and the zero point of every case below. */
#define UNIX_2000_01_01 946684800U
/* 2099-12-31 23:59:59 UTC - the latest. */
#define UNIX_2099_12_31 4102444799U

#define TICKS_PER_SEC   32768U

static void setRtc(SHIM_RTC_t *t,
    uint8_t year,
    uint8_t month,
    uint8_t date,
    uint8_t hours,
    uint8_t minutes,
    uint8_t seconds,
    uint8_t weekday)
{
  memset(t, 0, sizeof(*t));
  t->year = year;
  t->month = month;
  t->date = date;
  t->hours = hours;
  t->minutes = minutes;
  t->seconds = seconds;
  t->weekday = weekday;
}

/* The fixed points. Each one is a date whose Unix value is independently known,
 * so these catch a wholesale offset that a round-trip test cannot see - a
 * conversion that is self-consistent but an hour, a day or a year out. */
static void test_known_instants(void)
{
  SHIM_RTC_t t;

  testCase("test_known_instants");

  setRtc(&t, 0, 1, 1, 0, 0, 0, 6); /* 2000-01-01 was a Saturday */
  expectU("2000-01-01 00:00:00", ShimRtc_rtc2Unix(&t), UNIX_2000_01_01);

  setRtc(&t, 0, 2, 29, 12, 0, 0, 2); /* 2000 is a leap year - 400 rule */
  expectU("2000-02-29 12:00:00", ShimRtc_rtc2Unix(&t), 951825600U);

  setRtc(&t, 24, 2, 29, 23, 59, 59, 4); /* 2024-02-29, a leap day */
  expectU("2024-02-29 23:59:59", ShimRtc_rtc2Unix(&t), 1709251199U);

  setRtc(&t, 25, 12, 31, 23, 59, 59, 3);
  expectU("2025-12-31 23:59:59", ShimRtc_rtc2Unix(&t), 1767225599U);

  setRtc(&t, 99, 12, 31, 23, 59, 59, 4);
  expectU("2099-12-31 23:59:59", ShimRtc_rtc2Unix(&t), UNIX_2099_12_31);

  /* The hours term specifically. On a 16-bit-int target the product
   * hours * RTC_SECONDS_PER_HOUR overflows a plain int from 10:00 onwards, so
   * these three are the cases that would diverge if the widening cast in
   * ShimRtc_rtc2Unix() were ever dropped. A host build cannot reproduce that -
   * int is 32-bit here - which is precisely why the Shimmer3 MSP430 build is a
   * required release gate and not something CI can stand in for. */
  setRtc(&t, 0, 1, 1, 9, 0, 0, 6);
  expectU("09:00 on the epoch date", ShimRtc_rtc2Unix(&t), UNIX_2000_01_01 + 32400U);
  setRtc(&t, 0, 1, 1, 10, 0, 0, 6);
  expectU("10:00 on the epoch date", ShimRtc_rtc2Unix(&t), UNIX_2000_01_01 + 36000U);
  setRtc(&t, 0, 1, 1, 23, 0, 0, 6);
  expectU("23:00 on the epoch date", ShimRtc_rtc2Unix(&t), UNIX_2000_01_01 + 82800U);
}

/* unix2Rtc against the same fixed points, in the other direction. */
static void test_unix_to_calendar(void)
{
  SHIM_RTC_t t;

  testCase("test_unix_to_calendar");

  memset(&t, 0, sizeof(t));
  ShimRtc_unix2Rtc(&t, UNIX_2000_01_01);
  expectU("2000-01-01 year", t.year, 0);
  expectU("2000-01-01 month", t.month, 1);
  expectU("2000-01-01 date", t.date, 1);
  expectU("2000-01-01 hours", t.hours, 0);
  expectU("2000-01-01 weekday is Saturday", t.weekday, 6);
  expectU("unix field is stored", t.unix, UNIX_2000_01_01);

  /* 1970-01-01 was a Thursday, and Monday is weekday 1. Below 2000 the struct
   * cannot represent the year, but the weekday arithmetic still has to be right
   * at its own origin or every later weekday is skewed. */
  memset(&t, 0, sizeof(t));
  ShimRtc_unix2Rtc(&t, 0);
  expectU("1970-01-01 weekday is Thursday", t.weekday, 4);

  ShimRtc_unix2Rtc(&t, 951825600U);
  expectU("2000-02-29 is reachable", t.date, 29);
  expectU("2000-02-29 month", t.month, 2);

  /* 2001-03-01: the day after a NON-leap February. The month table is indexed
   * by the leap flag, so this is where an off-by-one in that lookup shows. */
  ShimRtc_unix2Rtc(&t, 983404800U);
  expectU("2001-03-01 year", t.year, 1);
  expectU("2001-03-01 month", t.month, 3);
  expectU("2001-03-01 date", t.date, 1);

  ShimRtc_unix2Rtc(&t, UNIX_2099_12_31);
  expectU("2099-12-31 year", t.year, 99);
  expectU("2099-12-31 month", t.month, 12);
  expectU("2099-12-31 date", t.date, 31);
  expectU("2099-12-31 hours", t.hours, 23);
  expectU("2099-12-31 minutes", t.minutes, 59);
  expectU("2099-12-31 seconds", t.seconds, 59);
}

/*
 * The exhaustive pass. Every midnight and every 23:59:59 from 2000-01-01 to
 * 2099-12-31 is converted to a calendar date and back, and the two must agree.
 *
 * This is the case that actually covers the leap-year rule and all twelve month
 * lengths, across 25 leap years, without anyone having to write out the dates.
 * It runs in well under a second.
 */
static void test_round_trip_every_day(void)
{
  uint32_t unix;
  uint32_t mismatches = 0;
  uint32_t days = 0;

  testCase("test_round_trip_every_day");

  for (unix = UNIX_2000_01_01; unix < UNIX_2099_12_31; unix += 86400U)
  {
    SHIM_RTC_t t;
    uint32_t back;

    memset(&t, 0, sizeof(t));
    ShimRtc_unix2Rtc(&t, unix);
    back = ShimRtc_rtc2Unix(&t);
    days++;
    if (back != unix)
    {
      if (mismatches < 5)
      {
        printf("  FAIL midnight round-trip at unix %lu came back as %lu "
               "(20%02u-%02u-%02u)\n",
            (unsigned long) unix, (unsigned long) back, t.year, t.month, t.date);
      }
      mismatches++;
    }

    /* And the last second of the same day, which exercises the hours, minutes
     * and seconds terms rather than only the day arithmetic. */
    memset(&t, 0, sizeof(t));
    ShimRtc_unix2Rtc(&t, unix + 86399U);
    back = ShimRtc_rtc2Unix(&t);
    if (back != unix + 86399U)
    {
      if (mismatches < 5)
      {
        printf("  FAIL end-of-day round-trip at unix %lu came back as %lu\n",
            (unsigned long) (unix + 86399U), (unsigned long) back);
      }
      mismatches++;
    }
  }

  expectU("every day from 2000 to 2099 round-trips", mismatches, 0);
  /* 100 years x 365 days + 25 leap days (2000, 2004 ... 2096). Asserted so that
   * a loop bound edited by accident shows up as a shrunken range rather than as
   * a quietly weaker test. */
  expectU("and the range covered is a century", days, 36525U);
}

/* The weekday must advance by exactly one per day and wrap 7 -> 1, for the
 * whole range. A weekday that drifts is the classic symptom of a leap-year
 * rule applied in one direction but not the other. */
static void test_weekday_is_monotonic(void)
{
  uint32_t unix;
  uint8_t expected;
  uint32_t breaks = 0;
  SHIM_RTC_t t;

  testCase("test_weekday_is_monotonic");

  memset(&t, 0, sizeof(t));
  ShimRtc_unix2Rtc(&t, UNIX_2000_01_01);
  expected = t.weekday;

  for (unix = UNIX_2000_01_01; unix < UNIX_2099_12_31; unix += 86400U)
  {
    memset(&t, 0, sizeof(t));
    ShimRtc_unix2Rtc(&t, unix);
    if (t.weekday != expected)
    {
      if (breaks < 5)
      {
        printf("  FAIL weekday at unix %lu is %u, expected %u\n",
            (unsigned long) unix, t.weekday, expected);
      }
      breaks++;
      expected = t.weekday;
    }
    expected = (uint8_t) ((expected % 7U) + 1U);
    expectTrue("weekday stays in 1..7", t.weekday >= 1 && t.weekday <= 7);
  }

  expectU("weekday advances by one every day", breaks, 0);
}

/*
 * ticks2Rtc is unix2Rtc with a 32768 Hz divide in front. It is what stamps SD
 * files, so the divide and the sub-second truncation are what matter.
 */
static void test_ticks_to_calendar(void)
{
  SHIM_RTC_t t;
  uint64_t ticks;

  testCase("test_ticks_to_calendar");

  ticks = (uint64_t) UNIX_2000_01_01 * TICKS_PER_SEC;
  memset(&t, 0, sizeof(t));
  ShimRtc_ticks2Rtc(&t, ticks);
  expectU("2000-01-01 from ticks: year", t.year, 0);
  expectU("2000-01-01 from ticks: month", t.month, 1);
  expectU("2000-01-01 from ticks: date", t.date, 1);
  expectU("unix field derived from ticks", t.unix, UNIX_2000_01_01);
  expectU("ticks field is stored whole", t.ticks, ticks);

  /* Sub-second ticks truncate down; they must never round a second forward. */
  memset(&t, 0, sizeof(t));
  ShimRtc_ticks2Rtc(&t, ticks + TICKS_PER_SEC - 1U);
  expectU("32767 sub-second ticks do not advance the second", t.unix, UNIX_2000_01_01);
  memset(&t, 0, sizeof(t));
  ShimRtc_ticks2Rtc(&t, ticks + TICKS_PER_SEC);
  expectU("a whole tick period advances one second", t.unix, UNIX_2000_01_01 + 1U);

  /* A 64-bit tick count well past the 32-bit second range must not be truncated
   * on the way in. 2099-12-31 is 1.3e14 ticks, far beyond a uint32_t. */
  memset(&t, 0, sizeof(t));
  ShimRtc_ticks2Rtc(&t, (uint64_t) UNIX_2099_12_31 * TICKS_PER_SEC);
  expectU("2099 from ticks: year", t.year, 99);
  expectU("2099 from ticks: month", t.month, 12);
  expectU("2099 from ticks: date", t.date, 31);
  expectU("2099 from ticks: hours", t.hours, 23);
}

/*
 * ShimRtc_isDateValid() is the gate the firmware uses to decide whether the
 * clock it read back is believable. Letting a bad date through means a
 * recording stamped with nonsense; rejecting a good one means a device that
 * flashes an RTC error it should not.
 */
static void test_date_validation(void)
{
  SHIM_RTC_t t;

  testCase("test_date_validation");

  setRtc(&t, 25, 6, 15, 12, 30, 45, 1);
  expectTrue("an ordinary date is valid", ShimRtc_isDateValid(&t));

  setRtc(&t, 0, 1, 1, 0, 0, 0, 1);
  expectTrue("the earliest representable date is valid", ShimRtc_isDateValid(&t));
  setRtc(&t, 99, 12, 31, 23, 59, 59, 7);
  expectTrue("the latest representable date is valid", ShimRtc_isDateValid(&t));

  /* Month and date bounds, each side of the limit. */
  setRtc(&t, 25, 0, 15, 0, 0, 0, 1);
  expectFalse("month 0 is rejected", ShimRtc_isDateValid(&t));
  setRtc(&t, 25, 13, 15, 0, 0, 0, 1);
  expectFalse("month 13 is rejected", ShimRtc_isDateValid(&t));
  setRtc(&t, 25, 6, 0, 0, 0, 0, 1);
  expectFalse("date 0 is rejected", ShimRtc_isDateValid(&t));
  setRtc(&t, 25, 6, 31, 0, 0, 0, 1);
  expectFalse("31 June is rejected", ShimRtc_isDateValid(&t));
  setRtc(&t, 25, 7, 31, 0, 0, 0, 1);
  expectTrue("31 July is accepted", ShimRtc_isDateValid(&t));

  /* February, which is the whole reason the month table has two rows. */
  setRtc(&t, 24, 2, 29, 0, 0, 0, 1);
  expectTrue("29 Feb 2024 is accepted", ShimRtc_isDateValid(&t));
  setRtc(&t, 25, 2, 29, 0, 0, 0, 1);
  expectFalse("29 Feb 2025 is rejected", ShimRtc_isDateValid(&t));
  setRtc(&t, 0, 2, 29, 0, 0, 0, 1);
  expectTrue("29 Feb 2000 is accepted - the 400 rule", ShimRtc_isDateValid(&t));

  /* Time-of-day and weekday bounds. */
  setRtc(&t, 25, 6, 15, 24, 0, 0, 1);
  expectFalse("hour 24 is rejected", ShimRtc_isDateValid(&t));
  setRtc(&t, 25, 6, 15, 23, 60, 0, 1);
  expectFalse("minute 60 is rejected", ShimRtc_isDateValid(&t));
  setRtc(&t, 25, 6, 15, 23, 59, 60, 1);
  expectFalse("second 60 is rejected - no leap seconds", ShimRtc_isDateValid(&t));
  setRtc(&t, 25, 6, 15, 0, 0, 0, 0);
  expectFalse("weekday 0 is rejected", ShimRtc_isDateValid(&t));
  setRtc(&t, 25, 6, 15, 0, 0, 0, 8);
  expectFalse("weekday 8 is rejected", ShimRtc_isDateValid(&t));

  /* An all-zero struct is what a failed RTC read leaves behind, and it must not
   * be believed. */
  memset(&t, 0, sizeof(t));
  expectFalse("an all-zero struct is rejected", ShimRtc_isDateValid(&t));

  /* NOTE: isDateValid() does not check that weekday agrees with the date - only
   * that it is in range. Nothing in the firmware relies on it doing so; this is
   * recorded here so the gap is a known one. */
}

/*
 * ShimRtc_rwcErrorCheck() is the one function here with a platform dependency:
 * it asks whether the real-world clock has been set and whether the config
 * wants the error flash, and drives the LED accordingly.
 */
static void test_rwc_error_flash(void)
{
  testCase("test_rwc_error_flash");

  hostStub_reset();
  ShimConfig_getStoredConfig()->rtcErrorEnable = 1;

  hostStub_setRwcTimeSet(0);
  ShimRtc_rwcErrorCheck();
  expectU("clock unset and flash enabled -> flashing", hostStub_getRtcErrorFlash(), 1);

  hostStub_setRwcTimeSet(1);
  ShimRtc_rwcErrorCheck();
  expectU("clock set -> not flashing", hostStub_getRtcErrorFlash(), 0);

  /* The config switch wins over an unset clock. */
  ShimConfig_getStoredConfig()->rtcErrorEnable = 0;
  hostStub_setRwcTimeSet(0);
  ShimRtc_rwcErrorCheck();
  expectU("flash disabled by config -> not flashing", hostStub_getRtcErrorFlash(), 0);
}

/* The config time is stored as a 64-bit tick count, and 0 means "never set". */
static void test_rwc_config_time(void)
{
  testCase("test_rwc_config_time");

  ShimRtc_init();
  expectU("init clears the config time", ShimRtc_getRwcConfigTime(), 0);
  expectFalse("and reports it as unset", ShimRtc_isRwcConfigTimeSet());

  ShimRtc_setRwcConfigTime((uint64_t) UNIX_2000_01_01 * TICKS_PER_SEC);
  expectTrue("a set time reports as set", ShimRtc_isRwcConfigTimeSet());
  expectU("and reads back whole, all 64 bits", ShimRtc_getRwcConfigTime(),
      (uint64_t) UNIX_2000_01_01 * TICKS_PER_SEC);

  /* A value whose low 32 bits are zero would look unset to a 32-bit read. */
  ShimRtc_setRwcConfigTime(0x0000000100000000ULL);
  expectTrue("a value with zero low word still reports as set",
      ShimRtc_isRwcConfigTimeSet());
}

/* Every hour of a handful of representative days, as "unix,date,weekday", for
 * crosscheck_rtc.py to put against Python's datetime. */
static void dumpCorpus(void)
{
  const uint32_t days[] = {
    UNIX_2000_01_01, /* the epoch of the representable range */
    951782400U,      /* 2000-02-29 - leap day, 400 rule */
    951868800U,      /* 2000-03-01 - the day after it */
    1709164800U,     /* 2024-02-29 - leap day, 4 rule */
    1709251200U,     /* 2024-03-01 */
    1740787200U,     /* 2025-03-01 - after a non-leap February */
    1767225600U,     /* 2026-01-01 - a year boundary */
    2524608000U,     /* 2050-01-01 */
    4102358400U,     /* 2099-12-31 - the last representable day */
  };
  unsigned d;
  unsigned h;

  for (d = 0; d < sizeof(days) / sizeof(days[0]); d++)
  {
    for (h = 0; h < 24; h++)
    {
      SHIM_RTC_t t;
      uint32_t unix = days[d] + h * 3600U + 1800U + 30U; /* HH:30:30 */

      memset(&t, 0, sizeof(t));
      ShimRtc_unix2Rtc(&t, unix);
      printf("%lu,20%02u-%02u-%02u %02u:%02u:%02u,%u\n", (unsigned long) unix,
          t.year, t.month, t.date, t.hours, t.minutes, t.seconds, t.weekday);
    }
  }
}

int main(int argc, char **argv)
{
  if (argc > 1 && strcmp(argv[1], "--dump") == 0)
  {
    dumpCorpus();
    return 0;
  }

  printf("shimmer_rtc host tests\n\n");
  hostTestSilenceUnused();
  hostStub_reset();

  test_known_instants();
  test_unix_to_calendar();
  test_round_trip_every_day();
  test_weekday_is_monotonic();
  test_ticks_to_calendar();
  test_date_validation();
  test_rwc_error_flash();
  test_rwc_config_time();

  return hostTestReport("shimmer_rtc");
}

#endif /* SHIMMER_HOST_TEST */
