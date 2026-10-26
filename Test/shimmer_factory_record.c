/*
 * shimmer_factory_record.c
 *
 * See shimmer_factory_record.h. Includes nothing but the C library and the
 * software CRC, so it builds unchanged for both MCUs and for the host tests.
 */

#include "shimmer_factory_record.h"

#include <stdint.h>
#include <string.h>

#include "CRC/shimmer_swCrc.h"

static void putU16(uint8_t *out, uint16_t v)
{
  out[0] = (uint8_t) (v & 0xFFU);
  out[1] = (uint8_t) (v >> 8);
}

static uint16_t getU16(const uint8_t *in)
{
  return (uint16_t) (in[0] | ((uint16_t) in[1] << 8));
}

static void putU32(uint8_t *out, uint32_t v)
{
  out[0] = (uint8_t) (v & 0xFFU);
  out[1] = (uint8_t) ((v >> 8) & 0xFFU);
  out[2] = (uint8_t) ((v >> 16) & 0xFFU);
  out[3] = (uint8_t) ((v >> 24) & 0xFFU);
}

static uint32_t getU32(const uint8_t *in)
{
  return (uint32_t) in[0] | ((uint32_t) in[1] << 8) | ((uint32_t) in[2] << 16)
      | ((uint32_t) in[3] << 24);
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
  putU16(&out[22], report->partsFound);
  putU16(&out[24], report->partsExpected);
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

void ShimFactoryRecord_encodePpRecord(const pp_record_t *record, uint8_t *out)
{
  uint16_t crc;

  putU16(&out[0], PP_RECORD_MAGIC);
  out[2] = PP_RECORD_LAYOUT_VER;
  out[3] = record->state;
  out[4] = record->runSeq;
  putU16(&out[5], record->nonce);
  putU32(&out[7], record->stepOk);
  putU16(&out[11], record->partsFound);
  out[13] = record->scheduleVer;
  crc = ShimSwCrc_calc(out, PP_RECORD_LEN - 2U);
  putU16(&out[14], crc);
}

uint8_t ShimFactoryRecord_decodePpRecord(const uint8_t *in, pp_record_t *record)
{
  uint8_t copy[PP_RECORD_LEN];

  if (getU16(&in[0]) != PP_RECORD_MAGIC || in[2] != PP_RECORD_LAYOUT_VER)
  {
    return 0;
  }
  /* ShimSwCrc_calc() takes a non-const buffer */
  memcpy(copy, in, sizeof(copy));
  if (ShimSwCrc_calc(copy, PP_RECORD_LEN - 2U) != getU16(&in[14]))
  {
    return 0;
  }

  record->state = in[3];
  record->runSeq = in[4];
  record->nonce = getU16(&in[5]);
  record->stepOk = getU32(&in[7]);
  record->partsFound = getU16(&in[11]);
  record->scheduleVer = in[13];
  return 1;
}

uint8_t ShimFactoryRecord_encodePpReadback(const pp_record_t *record,
    const pp_schedule_t *schedule,
    uint8_t *out)
{
  uint8_t n = (schedule->n > PP_MAX_STEPS) ? (uint8_t) PP_MAX_STEPS : schedule->n;
  uint8_t len = PP_READBACK_HEADER_LEN;
  uint8_t i;

  memset(out, 0, PP_READBACK_HEADER_LEN);
  out[0] = PP_READBACK_LAYOUT_VER;
  if (record != NULL)
  {
    out[1] = record->state;
    out[2] = record->runSeq;
    putU16(&out[3], record->nonce);
    putU32(&out[5], record->stepOk);
    putU16(&out[9], record->partsFound);
  }
  else
  {
    out[1] = PP_STATE_NONE;
  }
  putU16(&out[11], schedule->partsExpected);
  putU16(&out[13], schedule->baselineWindowMs);
  out[15] = schedule->markerId;
  out[16] = n;

  for (i = 0; i < n; i++)
  {
    out[len++] = schedule->steps[i].stepId;
    putU16(&out[len], schedule->steps[i].durationMs);
    len += 2U;
  }
  return len;
}

uint8_t ShimFactoryRecord_addPpStep(pp_schedule_t *schedule, uint8_t stepId, uint16_t durationMs)
{
  if (schedule->n >= PP_MAX_STEPS)
  {
    return 0;
  }
  schedule->steps[schedule->n].stepId = stepId;
  schedule->steps[schedule->n].durationMs = durationMs;
  schedule->n++;
  return 1;
}

uint32_t ShimFactoryRecord_ppRunMs(const pp_schedule_t *schedule)
{
  uint32_t total = 0;
  uint8_t i;

  for (i = 0; i < schedule->n && i < PP_MAX_STEPS; i++)
  {
    total += schedule->steps[i].durationMs;
    total += schedule->baselineWindowMs;
    if (i > 0U)
    {
      total += PP_MARKER_PULSE_MS;
    }
  }
  return total;
}
