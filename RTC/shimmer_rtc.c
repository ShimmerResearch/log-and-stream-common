/*
 * shimmer_rtc.c
 *
 *  Created on: Jun 27, 2025
 *      Author: MarkNolan
 */

#include "shimmer_rtc.h"
#include <stdint.h>

#include "log_and_stream_includes.h"

#if defined(SHIMMER3)
#include "../5xx_HAL/hal_RTC.h"
#endif

/* Stores the time at which the RWC was last set. */
uint64_t rwcConfigTime64;

/* Days in a month */
const uint8_t RTC_Months[2][12] = {
  { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }, /* Not leap year */
  { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }  /* Leap year */
};

void ShimRtc_init(void)
{
  ShimRtc_setRwcConfigTime(0);
}

uint8_t ShimRtc_isRwcConfigTimeSet(void)
{
  return rwcConfigTime64 != 0;
}

void ShimRtc_setRwcConfigTime(uint64_t val)
{
  rwcConfigTime64 = val;
} //64bits = 8bytes

uint64_t ShimRtc_getRwcConfigTime(void)
{
  return rwcConfigTime64;
}

uint32_t ShimRtc_rtc2Unix(SHIM_RTC_t *data)
{
  uint32_t days, seconds;
  uint32_t year = (uint32_t) data->year + 2000U;
  uint32_t month = data->month;
  /* Year is below offset year */
  if (year < RTC_OFFSET_YEAR)
  {
    return 0;
  }
  /* Days since 1970-01-01 in constant time, rather than a loop over every year
   * since 1970: Shimmer3R reads the clock this way for every sample timestamp.
   * Counting the year from March puts the leap day last, so the days before
   * each month follow one linear formula and the leap rule applies to the
   * whole years only. 719468 is the day number of 1970-01-01 on that count. */
  if (month <= 2U)
  {
    year--;
    month += 12U;
  }
  days = 365U * year + year / 4U - year / 100U + year / 400U
      + (153U * (month - 3U) + 2U) / 5U + (uint32_t) data->date - 1U - 719468U;
  seconds = days * RTC_SECONDS_PER_DAY;
  /* The cast is load-bearing on Shimmer3. data->hours is a uint8_t and
   * RTC_SECONDS_PER_HOUR is 3600, which fits an int - so on MSP430, where int
   * is 16 bits, the product is computed as a 16-bit int and overflows from
   * 10:00 onwards (10 * 3600 = 36000 > INT_MAX). Widening one operand forces
   * the whole expression to 32 bits. The minutes term needs no cast:
   * 59 * 60 = 3540 fits either width. */
  seconds += (uint32_t) data->hours * RTC_SECONDS_PER_HOUR;
  seconds += data->minutes * RTC_SECONDS_PER_MINUTE;
  seconds += data->seconds;

  /* seconds = days * 86400; */
  return seconds;
}

void ShimRtc_unix2Rtc(SHIM_RTC_t *data, uint32_t unix)
{
  uint16_t year;

  /* Store unix time to unix in struct */
  data->unix = unix;
  /* Get seconds from unix */
  data->seconds = unix % 60;
  /* Go to minutes */
  unix /= 60;
  /* Get minutes */
  data->minutes = unix % 60;
  /* Go to hours */
  unix /= 60;
  /* Get hours */
  data->hours = unix % 24;
  /* Go to days */
  unix /= 24;

  /* Get week day */
  /* Monday is day one */
  data->weekday = (unix + 3) % 7 + 1;

  /* Get year */
  year = 1970;
  while (1)
  {
    if (RTC_LEAP_YEAR(year))
    {
      if (unix >= 366)
      {
        unix -= 366;
      }
      else
      {
        break;
      }
    }
    else if (unix >= 365)
    {
      unix -= 365;
    }
    else
    {
      break;
    }
    year++;
  }
  /* Get year in xx format */
  data->year = (uint8_t) (year - 2000);
  /* Get month */
  for (data->month = 0; data->month < 12; data->month++)
  {
    if (RTC_LEAP_YEAR(year))
    {
      if (unix >= (uint32_t) RTC_Months[1][data->month])
      {
        unix -= RTC_Months[1][data->month];
      }
      else
      {
        break;
      }
    }
    else if (unix >= (uint32_t) RTC_Months[0][data->month])
    {
      unix -= RTC_Months[0][data->month];
    }
    else
    {
      break;
    }
  }
  /* Get month */
  /* Month starts with 1 */
  data->month++;
  /* Get date */
  /* Date starts with 1 */
  data->date = unix + 1;
}

void ShimRtc_ticks2Rtc(SHIM_RTC_t *data, uint64_t ticks)
{
  uint16_t year;
  uint32_t unix;

  data->ticks = ticks;
  /* Store unix time to unix in struct */
  unix = ticks / 32768;
  data->unix = unix;
  /* Get seconds from unix */
  data->seconds = unix % 60;
  /* Go to minutes */
  unix /= 60;
  /* Get minutes */
  data->minutes = unix % 60;
  /* Go to hours */
  unix /= 60;
  /* Get hours */
  data->hours = unix % 24;
  /* Go to days */
  unix /= 24;

  /* Get week day */
  /* Monday is day one */
  data->weekday = (unix + 3) % 7 + 1;

  /* Get year */
  year = 1970;
  while (1)
  {
    if (RTC_LEAP_YEAR(year))
    {
      if (unix >= 366)
      {
        unix -= 366;
      }
      else
      {
        break;
      }
    }
    else if (unix >= 365)
    {
      unix -= 365;
    }
    else
    {
      break;
    }
    year++;
  }
  /* Get year in xx format */
  data->year = (uint8_t) (year - 2000);
  /* Get month */
  for (data->month = 0; data->month < 12; data->month++)
  {
    if (RTC_LEAP_YEAR(year))
    {
      if (unix >= (uint32_t) RTC_Months[1][data->month])
      {
        unix -= RTC_Months[1][data->month];
      }
      else
      {
        break;
      }
    }
    else if (unix >= (uint32_t) RTC_Months[0][data->month])
    {
      unix -= RTC_Months[0][data->month];
    }
    else
    {
      break;
    }
  }
  /* Get month */
  /* Month starts with 1 */
  data->month++;
  /* Get date */
  /* Date starts with 1 */
  data->date = unix + 1;
}

uint8_t ShimRtc_isDateValid(SHIM_RTC_t *data)
{
  if (data->year > 99 || data->month == 0 || data->month > 12 || data->date == 0
      || data->date > RTC_Months[RTC_LEAP_YEAR(2000 + data->year) ? 1 : 0][data->month - 1]
      || data->weekday == 0 || data->weekday > 7 || data->hours > 23
      || data->minutes > 59 || data->seconds > 59)
  {
    /* Invalid date */
    return 0;
  }
  return 1; //Valid date
}

void ShimRtc_rwcErrorCheck(void)
{
  uint8_t state = 0;
#if !TEST_RTC_ERR_FLASH_OFF
  state = (!RTC_isRwcTimeSet()) && ShimConfig_getStoredConfig()->rtcErrorEnable;
#endif //TEST_RTC_ERR_FLASH_OFF
  ShimLeds_setRtcErrorFlash(state);
}

/* Record who last set the clock (InfoMem NV_SD_TRIAL_CONFIG0, rtcSetByBt):
 * 1 for Bluetooth, 0 for the dock. The flag is flushed to InfoMem only when
 * it changes. Any InfoMem write erases flash with interrupts disabled - a
 * 128-byte segment on Shimmer3, an 8 KB page on Shimmer3R - which stalls
 * sampling, and hosts that set the clock at every logging start (the Java
 * driver does) would otherwise pay that mid-stream every time. Returns 1 if
 * it wrote InfoMem. */
uint8_t ShimRtc_setRtcSetByBt(uint8_t setByBt)
{
  gConfigBytes *cfg = ShimConfig_getStoredConfig();
  uint8_t want = setByBt ? 1U : 0U;

  if (cfg->rtcSetByBt == want)
  {
    return 0;
  }
  cfg->rtcSetByBt = want;
  InfoMem_write(NV_SD_TRIAL_CONFIG0, &cfg->rawBytes[NV_SD_TRIAL_CONFIG0], 1);
  return 1;
}

/* DEV-1539: a clock set that arrives while sensing waits for sensing to stop.
 *
 * On Shimmer3R the sample stamps are the clock itself (RTC_getRwcTime is
 * RTC_get64), so setting it mid-session steps every stamp after the set by the
 * difference between the value written and the clock it replaces. Over
 * Bluetooth that difference is mostly the command's transit time, which the
 * firmware cannot see: 19-27 ms on an idle Classic link, about 3.5 ms while
 * streaming. A step back beyond a host's reorder window reads as a counter
 * wrap, 512 s. Shimmer3's stamps are a free-running counter and do not step,
 * but its clock offset still moves under a running session: a log's later
 * hourly files carry a different SDH_RTC_DIFF from its earlier ones. The rule
 * is the same on both, so a host need not know which it is talking to.
 *
 * While sensing (streaming, SD logging or both), a set is accepted and kept as
 * the difference between the time it asked for and the clock when it arrived.
 * Once streaming and logging have both stopped, ShimSens_stopSensing() applies
 * it to the clock as it then reads, which gives the time the host asked for
 * just as an immediate set would have. A later set while still sensing
 * replaces an earlier one. Until then the clock, GET_RWC and the dock's
 * current-time read all carry on unchanged, as the session's stamps do.
 *
 * Who set it (rtcSetByBt and the SD header's copy) waits too: on the first set
 * it changes, and that InfoMem write erases flash with interrupts disabled. */
static uint8_t rwcSetDeferred;
static uint8_t rwcSetDeferredByBt;
/* Requested ticks minus the clock when the set arrived, modulo 2^64: adding it
 * back gives the right answer whichever way the clock is to move. */
static uint64_t rwcSetDeferredOffset;

static void ShimRtc_applyRwc(uint64_t ticks, uint8_t setByBt)
{
  ShimRtc_setRtcSetByBt(setByBt);
  ShimSdHead_sdHeadTextSetByte(SDH_TRIAL_CONFIG0,
      ShimConfig_getStoredConfig()->rawBytes[NV_SD_TRIAL_CONFIG0]);

  RTC_setTimeFromTicksPtr((uint8_t *) &ticks);
  ShimRtc_rwcErrorCheck();
#if defined(SHIMMER3R)
  if (setByBt)
  {
    RTC_setAlarmBattRead(); //configure RTC alarm after time set from BT.
  }
#endif
}

/* Set the real-world clock to ticks (32768 Hz since the Unix epoch, UTC), now
 * or, while sensing, when sensing stops. setByBt is 1 for SET_RWC_COMMAND and
 * 0 for the dock. Returns 1 if the set was deferred. */
uint8_t ShimRtc_setRwc(uint64_t ticks, uint8_t setByBt)
{
  if (shimmerStatus.sensing)
  {
    rwcSetDeferredOffset = ticks - RTC_getRwcTime();
    rwcSetDeferredByBt = setByBt ? 1U : 0U;
    rwcSetDeferred = 1;
    return 1;
  }
  rwcSetDeferred = 0;
  ShimRtc_applyRwc(ticks, setByBt);
  return 0;
}

/* Apply a set deferred while sensing. Called once streaming and logging have
 * both stopped. Returns 1 if there was one. */
uint8_t ShimRtc_applyDeferredRwc(void)
{
  if (!rwcSetDeferred)
  {
    return 0;
  }
  rwcSetDeferred = 0;
  ShimRtc_applyRwc(RTC_getRwcTime() + rwcSetDeferredOffset, rwcSetDeferredByBt);
  return 1;
}

uint8_t ShimRtc_isRwcSetDeferred(void)
{
  return rwcSetDeferred;
}
