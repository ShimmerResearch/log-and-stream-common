#!/usr/bin/env python3
"""Cross-check the firmware's RTC conversions against Python's datetime.

WHY THIS EXISTS. test_rtc.c proves ShimRtc_unix2Rtc() and ShimRtc_rtc2Unix() are
each other's inverse over the whole 2000-2099 range, which is a strong property
- but it is a property the pair can satisfy while both being wrong in the same
direction. A conversion that is consistently a day out, or that applies the leap
rule to the wrong year, round-trips perfectly.

So this puts the firmware's answers against an oracle that shares no code with
it: Python's own calendar. Where they disagree, the firmware is wrong, because
the recordings it stamps are read by hosts that use exactly this calendar.

Run by .github/workflows/host-tests.yml, after building test_rtc.

Usage: crosscheck_rtc.py <path-to-test_rtc-binary>
"""
import datetime
import subprocess
import sys

# Monday is 1 in the firmware's SHIM_RTC_t; Python's isoweekday() agrees.
def expected(unix_ts):
    dt = datetime.datetime.fromtimestamp(unix_ts, datetime.timezone.utc)
    return dt.strftime("%Y-%m-%d %H:%M:%S"), dt.isoweekday()


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2

    out = subprocess.run([sys.argv[1], "--dump"], capture_output=True, text=True, check=True)
    lines = [ln for ln in out.stdout.splitlines() if ln.strip() and not ln.startswith("#")]
    if not lines:
        print("FAIL: the C test produced no output")
        return 1

    failures = 0
    for line in lines:
        unix_str, stamp, weekday_str = line.split(",")
        unix_ts = int(unix_str)
        want_stamp, want_weekday = expected(unix_ts)

        if stamp != want_stamp:
            print(f"  FAIL unix {unix_ts}: firmware says {stamp}, datetime says {want_stamp}")
            failures += 1
        if int(weekday_str) != want_weekday:
            print(f"  FAIL unix {unix_ts} ({want_stamp}): firmware weekday "
                  f"{weekday_str}, datetime says {want_weekday}")
            failures += 1
        if failures >= 10:
            print("  ... stopping after 10")
            break

    # A handful of dates asserted directly, so that a regression in this script's
    # own corpus selection cannot make it pass vacuously.
    for unix_ts, want in (
        (946684800, "2000-01-01 00:00:00"),   # the earliest the RTC can express
        (951782400, "2000-02-29 00:00:00"),   # leap day, 400 rule
        (1709164800, "2024-02-29 00:00:00"),  # leap day, 4 rule
        (4102358400, "2099-12-31 00:00:00"),  # the latest
    ):
        got, _ = expected(unix_ts)
        if got != want:
            print(f"  FAIL oracle self-check: {unix_ts} -> {got}, expected {want}")
            failures += 1

    if failures:
        print(f"\nRTC cross-check: {failures} FAILURE(S)")
        return 1
    print(f"\nRTC cross-check: firmware and Python datetime agree over "
          f"{len(lines)} timestamps")
    return 0


if __name__ == "__main__":
    sys.exit(main())
