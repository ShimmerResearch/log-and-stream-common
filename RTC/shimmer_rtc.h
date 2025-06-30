/*
 * shimmer_rtc.h
 *
 *  Created on: Jun 27, 2025
 *      Author: MarkNolan
 */

#ifndef RTC_SHIMMER_RTC_H_
#define RTC_SHIMMER_RTC_H_

#include <stdint.h>

/* Internal RTC defines */
#define RTC_LEAP_YEAR(year) \
  ((((year) % 4 == 0) && ((year) % 100 != 0)) || ((year) % 400 == 0))
#define RTC_DAYS_IN_YEAR(x)    RTC_LEAP_YEAR(x) ? 366 : 365
#define RTC_OFFSET_YEAR        1970
#define RTC_SECONDS_PER_DAY    86400
#define RTC_SECONDS_PER_HOUR   3600
#define RTC_SECONDS_PER_MINUTE 60
#define RTC_BCD2BIN(x)         ((((x) >> 4) & 0x0F) * 10 + ((x) & 0x0F))
#define RTC_CHAR2NUM(x)        ((x) - '0')
#define RTC_CHARISNUM(x)       ((x) >= '0' && (x) <= '9')

#if defined(SHIMMER3R)
// For Shimmer3R, we use the RTC to get the time from the RWC
#define RTC_getRwcTime RTC_get64
#endif //SHIMMER3R

typedef struct
{
  uint8_t seconds;     /*!< Seconds parameter, from 00 to 59 */
  uint16_t subseconds; /*!< Subsecond downcounter. When it reaches zero, it's reload value is the same as
                                 @ref RTC_SYNC_PREDIV, so in our case 0x3FF = 1023, 1024 steps in one second */
  uint8_t minutes;     /*!< Minutes parameter, from 00 to 59 */
  uint8_t hours;       /*!< Hours parameter, 24Hour mode, 00 to 23 */
  uint8_t weekday;     /*!< Day in a week, from 1 to 7 */
  uint8_t date;        /*!< Date in a month, 1 to 31 */
  uint8_t month;       /*!< Month in a year, 1 to 12 */
  uint8_t year;   /*!< Year parameter, 00 to 99, 00 is 2000 and 99 is 2099 */
  uint32_t unix;  /*!< Seconds from 01.01.1970 00:00:00 */
  uint64_t ticks; /*!< ticks from 01.01.1970 00:00:00 */
} SHIM_RTC_t;

extern const uint8_t RTC_Months[2][12];

void ShimRtc_init(void);
uint8_t ShimRtc_isRwcConfigTimeSet(void);
void ShimRtc_setRwcConfigTime(uint64_t val);
uint64_t ShimRtc_getRwcConfigTime(void);

uint32_t ShimRtc_rtc2Unix(SHIM_RTC_t *data);
void ShimRtc_unix2Rtc(SHIM_RTC_t *data, uint32_t unix);
void ShimRtc_ticks2Rtc(SHIM_RTC_t *data, uint64_t ticks);

uint8_t ShimRtc_isDateValid(SHIM_RTC_t *data);

uint8_t ShimRtc_isTimeSet(void);
void ShimRtc_rwcErrorCheck(void);

#endif /* RTC_SHIMMER_RTC_H_ */
