/*
 * shimmer_rtc.c
 *
 *  Created on: Jun 27, 2025
 *      Author: MarkNolan
 */

#include "shimmer_rtc.h"
#include <stdint.h>

#include <log_and_stream_includes.h>

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
  uint32_t days = 0, seconds = 0;
  uint16_t i;
  uint16_t year = (uint16_t) (data->year + 2000);
  /* Year is below offset year */
  if (year < RTC_OFFSET_YEAR)
  {
    return 0;
  }
  /* Days in back years */
  for (i = RTC_OFFSET_YEAR; i < year; i++)
  {
    days += RTC_DAYS_IN_YEAR(i);
  }
  /* Days in current year */
  for (i = 1; i < data->month; i++)
  {
    days += RTC_Months[RTC_LEAP_YEAR(year)][i - 1];
  }
  /* Day starts with 1 */
  days += data->date - 1;
  seconds = days * RTC_SECONDS_PER_DAY;
  seconds += data->hours * RTC_SECONDS_PER_HOUR;
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
