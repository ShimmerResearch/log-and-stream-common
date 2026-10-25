/*
 * shimmer_factory_record.c
 *
 * See shimmer_factory_record.h. Includes nothing but the C library, so it
 * builds unchanged for both MCUs and for the host tests.
 */

#include "shimmer_factory_record.h"

#include <stdint.h>
#include <string.h>

static void putU16(uint8_t *out, uint16_t v)
{
  out[0] = (uint8_t) (v & 0xFFU);
  out[1] = (uint8_t) (v >> 8);
}

uint16_t ShimFactoryRecord_saturateMs(uint32_t ms)
{
  return (ms > 0xFFFFUL) ? 0xFFFFU : (uint16_t) ms;
}

uint8_t ShimFactoryRecord_encodeBootReport(const boot_report_t *report, uint8_t *out)
{
  out[0] = BOOT_REPORT_LAYOUT_VER;
  memcpy(&out[1], &report->uid[0], BOOT_REPORT_UID_LEN);
  out[13] = report->flags;
  putU16(&out[14], report->i2cMs);
  putU16(&out[16], report->btMs);
  putU16(&out[18], report->configMs);
  out[20] = report->btBaudAttempts;
  out[21] = report->lseDrive;
  return BOOT_REPORT_LEN;
}

uint8_t ShimFactoryRecord_encodeButtonState(uint8_t level,
    uint16_t pressCount,
    uint16_t lastPressMs,
    uint8_t *out)
{
  out[0] = level ? 1U : 0U;
  putU16(&out[1], pressCount);
  putU16(&out[3], lastPressMs);
  return BUTTON_STATE_LEN;
}
