/*
 * shimmer_button_log_mode.c
 *
 * See shimmer_button_log_mode.h for the byte layout.
 */

#include <stdint.h>

#include "shimmer_button_log_mode.h"

static uint8_t isButtonModeBuilt(uint8_t mode)
{
  return (mode >= BTN_MODE_DISABLED) && (mode <= BTN_MODE_RECORDED);
}

static uint8_t canPressStartLogging(uint8_t mode)
{
  return (mode == BTN_MODE_SINGLE_PRESS) || (mode == BTN_MODE_DOUBLE_PRESS)
      || (mode == BTN_MODE_DOUBLE_RECORDED);
}

uint8_t ShimBtnLogMode_correct(uint8_t raw)
{
  uint8_t mode = raw & BTN_LOG_MODE_BUTTON_MASK;
  uint8_t trig = (uint8_t) ((raw & BTN_LOG_MODE_TRIGGER_MASK) >> BTN_LOG_MODE_TRIGGER_SHIFT);
  uint8_t ttl = raw & BTN_LOG_MODE_TTL_INPUT;

  if (raw & BTN_LOG_MODE_INVALID)
  {
    return 0;
  }
  /* BTN_MODE_LEGACY lands here too: its log triggers are not read, so they
   * are cleared rather than kept to mislead a host reading the byte back. The
   * TTL input does not depend on the button, so it stays */
  if (!isButtonModeBuilt(mode) || (trig > LOG_TRIG_NONE))
  {
    return ttl;
  }
  return raw & (BTN_LOG_MODE_BUTTON_MASK | BTN_LOG_MODE_TRIGGER_MASK | BTN_LOG_MODE_TTL_INPUT);
}

uint8_t ShimBtnLogMode_buttonMode(uint8_t raw)
{
  return ShimBtnLogMode_correct(raw) & BTN_LOG_MODE_BUTTON_MASK;
}

uint8_t ShimBtnLogMode_logTriggers(uint8_t raw)
{
  return (uint8_t) ((ShimBtnLogMode_correct(raw) & BTN_LOG_MODE_TRIGGER_MASK)
      >> BTN_LOG_MODE_TRIGGER_SHIFT);
}

uint8_t ShimBtnLogMode_withButtonMode(uint8_t raw, int value)
{
  if ((value < 0) || (value > (int) BTN_LOG_MODE_BUTTON_MASK))
  {
    return raw | BTN_LOG_MODE_INVALID;
  }
  return (uint8_t) ((raw & ~BTN_LOG_MODE_BUTTON_MASK) | (uint8_t) value);
}

uint8_t ShimBtnLogMode_withLogTriggers(uint8_t raw, int value)
{
  if ((value < 0) || (value > (int) (BTN_LOG_MODE_TRIGGER_MASK >> BTN_LOG_MODE_TRIGGER_SHIFT)))
  {
    return raw | BTN_LOG_MODE_INVALID;
  }
  return (uint8_t) ((raw & ~BTN_LOG_MODE_TRIGGER_MASK)
      | (uint8_t) (value << BTN_LOG_MODE_TRIGGER_SHIFT));
}

uint8_t ShimBtnLogMode_withTtlInput(uint8_t raw, int value)
{
  return value ? (uint8_t) (raw | BTN_LOG_MODE_TTL_INPUT) :
                 (uint8_t) (raw & (uint8_t) ~BTN_LOG_MODE_TTL_INPUT);
}

uint8_t ShimBtnLogMode_doesPressToggleLogging(uint8_t raw, uint8_t userButtonEnable)
{
  uint8_t mode = ShimBtnLogMode_buttonMode(raw);

  if (mode == BTN_MODE_LEGACY)
  {
    return userButtonEnable ? 1U : 0U;
  }
  return canPressStartLogging(mode);
}

uint8_t ShimBtnLogMode_doesUndockStartLogging(uint8_t raw, uint8_t userButtonEnable, uint8_t singleTouchStart)
{
  if (singleTouchStart)
  {
    return 0;
  }
  if (ShimBtnLogMode_buttonMode(raw) == BTN_MODE_LEGACY)
  {
    return userButtonEnable ? 0U : 1U;
  }
  return ShimBtnLogMode_logTriggers(raw) == LOG_TRIG_HOST_UNDOCK;
}

uint8_t ShimBtnLogMode_isLoggingDisabled(uint8_t raw)
{
  /* A corrected byte has log triggers only if its button mode is not legacy */
  return ShimBtnLogMode_logTriggers(raw) == LOG_TRIG_NONE;
}

uint8_t ShimBtnLogMode_isButtonDisabled(uint8_t raw)
{
  return ShimBtnLogMode_buttonMode(raw) == BTN_MODE_DISABLED;
}

uint8_t ShimBtnLogMode_isDoublePress(uint8_t raw)
{
  uint8_t mode = ShimBtnLogMode_buttonMode(raw);

  return (mode == BTN_MODE_DOUBLE_PRESS) || (mode == BTN_MODE_DOUBLE_RECORDED);
}

uint8_t ShimBtnLogMode_recordsPresses(uint8_t raw)
{
  uint8_t mode = ShimBtnLogMode_buttonMode(raw);

  return (mode == BTN_MODE_DOUBLE_RECORDED) || (mode == BTN_MODE_RECORDED);
}

uint8_t ShimBtnLogMode_isTtlInputEnabled(uint8_t raw)
{
  return (ShimBtnLogMode_correct(raw) & BTN_LOG_MODE_TTL_INPUT) ? 1U : 0U;
}

uint8_t ShimBtnLogMode_withoutPressRecording(uint8_t raw)
{
  uint8_t fixed = ShimBtnLogMode_correct(raw);
  uint8_t rest = fixed & (uint8_t) ~BTN_LOG_MODE_BUTTON_MASK;

  switch (fixed & BTN_LOG_MODE_BUTTON_MASK)
  {
    case BTN_MODE_DOUBLE_RECORDED:
      return (uint8_t) (rest | BTN_MODE_DOUBLE_PRESS);
    case BTN_MODE_RECORDED:
      return (uint8_t) (rest | BTN_MODE_DISABLED);
    default:
      return fixed;
  }
}

uint8_t ShimBtnLogMode_allowsSingleTouch(uint8_t raw)
{
  uint8_t mode = ShimBtnLogMode_buttonMode(raw);

  /* Legacy: ShimConfig_checkAndCorrectConfig() turns the button on for it */
  if (mode == BTN_MODE_LEGACY)
  {
    return 1;
  }
  return canPressStartLogging(mode) && !ShimBtnLogMode_isLoggingDisabled(raw);
}

uint8_t ShimBtnLogMode_nearestUserButtonEnable(uint8_t raw, uint8_t userButtonEnable)
{
  uint8_t mode = ShimBtnLogMode_buttonMode(raw);

  if (mode == BTN_MODE_LEGACY)
  {
    return userButtonEnable ? 1U : 0U;
  }
  if (!canPressStartLogging(mode) && (ShimBtnLogMode_logTriggers(raw) == LOG_TRIG_HOST_UNDOCK))
  {
    return 0;
  }
  return 1;
}
