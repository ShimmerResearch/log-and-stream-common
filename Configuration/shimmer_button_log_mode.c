/*
 * shimmer_button_log_mode.c
 *
 * See shimmer_button_log_mode.h for the byte layout.
 */

#include <stdint.h>

#include "shimmer_button_log_mode.h"

static uint8_t isButtonModeBuilt(uint8_t mode)
{
  return (mode == BTN_MODE_DISABLED) || (mode == BTN_MODE_SINGLE_PRESS);
}

static uint8_t canPressStartLogging(uint8_t mode)
{
  return mode == BTN_MODE_SINGLE_PRESS;
}

uint8_t ShimBtnLogMode_correct(uint8_t raw)
{
  uint8_t mode = raw & BTN_LOG_MODE_BUTTON_MASK;
  uint8_t trig = (uint8_t) ((raw & BTN_LOG_MODE_TRIGGER_MASK) >> BTN_LOG_MODE_TRIGGER_SHIFT);

  /* BTN_MODE_LEGACY lands here too: its log triggers are not read, so they
   * are cleared rather than kept to mislead a host reading the byte back */
  if ((raw & BTN_LOG_MODE_INVALID) || !isButtonModeBuilt(mode) || (trig > LOG_TRIG_NONE))
  {
    return 0;
  }
  return raw & (BTN_LOG_MODE_BUTTON_MASK | BTN_LOG_MODE_TRIGGER_MASK);
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
