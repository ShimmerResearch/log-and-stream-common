/*
 * Host-side test for Configuration/shimmer_button_log_mode.c.
 *
 * Configuration byte 232 says what the user button does and what else may
 * start SD logging. Its rules are safety rules more than decoding: a byte that
 * no host meant to write - erased flash, a host that zeroes what it does not
 * know, a value from a newer host - must read as the behaviour from before the
 * byte existed, and must never turn logging off. A setting that falls back to
 * byte 217 alone must not add an undock start. Each of those is pinned here,
 * over every one of the 256 values where it can be.
 *
 * The field values are also a contract with every host that writes the byte
 * and every parser that reads its copy in the SD header, so they are pinned
 * as numbers.
 *
 * shimmer_button_log_mode.c includes nothing but <stdint.h>, so this needs no
 * stubs. Run by .github/workflows/host-tests.yml.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include "host_test.h"

#include "Configuration/shimmer_button_log_mode.h"

/* Restated rather than built from the header's masks, so a mask that moves
 * fails here instead of moving the test with it */
static uint8_t byteOf(uint8_t buttonMode, uint8_t logTriggers)
{
  return (uint8_t) (buttonMode | (logTriggers << 3));
}

/* The button mode and log triggers: bits 0-4 */
static int areButtonFieldsValid(uint8_t raw)
{
  uint8_t mode = raw & 0x07U;
  uint8_t trig = (raw >> 3) & 0x03U;
  return mode >= 1U && mode <= 5U && trig <= 2U;
}

static void test_constants(void)
{
  testCase("the field values are the documented ones");
  expectX("legacy", BTN_MODE_LEGACY, 0);
  expectX("button disabled", BTN_MODE_DISABLED, 1);
  expectX("single press", BTN_MODE_SINGLE_PRESS, 2);
  expectX("double press", BTN_MODE_DOUBLE_PRESS, 3);
  expectX("double press, presses recorded", BTN_MODE_DOUBLE_RECORDED, 4);
  expectX("presses recorded only", BTN_MODE_RECORDED, 5);
  expectX("host only", LOG_TRIG_HOST, 0);
  expectX("host or undock", LOG_TRIG_HOST_UNDOCK, 1);
  expectX("logging off", LOG_TRIG_NONE, 2);
  expectX("button mask", BTN_LOG_MODE_BUTTON_MASK, 0x07);
  expectX("trigger mask", BTN_LOG_MODE_TRIGGER_MASK, 0x18);
  expectX("trigger shift", BTN_LOG_MODE_TRIGGER_SHIFT, 3);
  expectX("TTL input bit", BTN_LOG_MODE_TTL_INPUT, 0x20);
  expectX("must-be-0 bit", BTN_LOG_MODE_INVALID, 0x80);
}

static void test_correct_every_value(void)
{
  unsigned v;
  uint8_t raw, fixed;
  unsigned kept = 0;

  testCase("every value either stands or reads as legacy");
  for (v = 0; v < 256U; v++)
  {
    raw = (uint8_t) v;
    fixed = ShimBtnLogMode_correct(raw);
    if (raw & 0x80U)
    {
      expectX("bit 7 set: all of it reads as legacy, TTL off", fixed, 0);
    }
    else if (areButtonFieldsValid(raw))
    {
      kept++;
      /* Bit 6 is not used, so it goes */
      expectX("valid value kept, bit 6 cleared", fixed, raw & 0x3FU);
    }
    else
    {
      /* The TTL input does not depend on the button */
      expectX("invalid button fields read as legacy, TTL kept", fixed, raw & 0x20U);
    }
    expectX("correcting twice changes nothing", ShimBtnLogMode_correct(fixed), fixed);
  }
  /* 5 button modes x 3 log triggers x 4 combinations of bits 5-6 */
  expectU("values that stand", kept, 60);
}

static void test_values_no_host_meant(void)
{
  testCase("erased flash and a zeroing host both read as legacy");
  expectX("0xFF (erased flash)", ShimBtnLogMode_correct(0xFF), 0);
  expectX("0x00 (host that writes 0)", ShimBtnLogMode_correct(0x00), 0);
  /* A legacy button mode does not read its log triggers, so they go */
  expectX("legacy with log triggers set", ShimBtnLogMode_correct(byteOf(0, 2)), 0);
  /* Reserved, so legacy */
  expectX("button mode 6", ShimBtnLogMode_correct(byteOf(6, 0)), 0);
  expectX("button mode 7", ShimBtnLogMode_correct(byteOf(7, 1)), 0);
  expectX("reserved log triggers", ShimBtnLogMode_correct(byteOf(1, 3)), 0);
  expectX("valid fields, bit 7 set", ShimBtnLogMode_correct(0x80U | byteOf(1, 2)), 0);

  testCase("the TTL input stands on its own");
  expectX("legacy button, TTL on", ShimBtnLogMode_correct(0x20), 0x20);
  expectX("reserved button mode, TTL on",
      ShimBtnLogMode_correct(0x20U | byteOf(6, 0)), 0x20);
  expectX("TTL on, bit 7 set", ShimBtnLogMode_correct(0xA0), 0);
}

static void test_queries_apply_the_correction(void)
{
  unsigned v;
  uint8_t raw, fixed, ube, st;

  testCase("an uncorrected byte is never read any other way");
  for (v = 0; v < 256U; v++)
  {
    raw = (uint8_t) v;
    fixed = ShimBtnLogMode_correct(raw);
    for (ube = 0; ube < 2U; ube++)
    {
      expectU("press", ShimBtnLogMode_doesPressToggleLogging(raw, ube),
          ShimBtnLogMode_doesPressToggleLogging(fixed, ube));
      expectU("nearest userButtonEnable", ShimBtnLogMode_nearestUserButtonEnable(raw, ube),
          ShimBtnLogMode_nearestUserButtonEnable(fixed, ube));
      for (st = 0; st < 2U; st++)
      {
        expectU("undock", ShimBtnLogMode_doesUndockStartLogging(raw, ube, st),
            ShimBtnLogMode_doesUndockStartLogging(fixed, ube, st));
      }
    }
    expectU("logging off", ShimBtnLogMode_isLoggingDisabled(raw),
        ShimBtnLogMode_isLoggingDisabled(fixed));
    expectU("button off", ShimBtnLogMode_isButtonDisabled(raw),
        ShimBtnLogMode_isButtonDisabled(fixed));
    expectU("single touch", ShimBtnLogMode_allowsSingleTouch(raw),
        ShimBtnLogMode_allowsSingleTouch(fixed));
    expectU("double press", ShimBtnLogMode_isDoublePress(raw),
        ShimBtnLogMode_isDoublePress(fixed));
    expectU("records presses", ShimBtnLogMode_recordsPresses(raw),
        ShimBtnLogMode_recordsPresses(fixed));
    expectX("without press recording", ShimBtnLogMode_withoutPressRecording(raw),
        ShimBtnLogMode_withoutPressRecording(fixed));
    expectU("TTL input", ShimBtnLogMode_isTtlInputEnabled(raw),
        ShimBtnLogMode_isTtlInputEnabled(fixed));
    expectU("TTL input is bit 5 unless bit 7 is set",
        ShimBtnLogMode_isTtlInputEnabled(raw), (raw & 0x80U) ? 0U : ((raw >> 5) & 1U));
  }
}

static void test_legacy_is_unchanged(void)
{
  testCase("legacy: userButtonEnable decides, as before the byte existed");
  expectU("button on: press toggles", ShimBtnLogMode_doesPressToggleLogging(0, 1), 1);
  expectU("button on: no undock start", ShimBtnLogMode_doesUndockStartLogging(0, 1, 0), 0);
  expectU("button off: press does nothing",
      ShimBtnLogMode_doesPressToggleLogging(0, 0), 0);
  /* The trap the byte exists to get out of, kept as it was */
  expectU("button off: undock starts", ShimBtnLogMode_doesUndockStartLogging(0, 0, 0), 1);
  expectU("single touch: no undock start",
      ShimBtnLogMode_doesUndockStartLogging(0, 0, 1), 0);
  expectU("logging stays on", ShimBtnLogMode_isLoggingDisabled(0), 0);
  expectU("button LED stays on", ShimBtnLogMode_isButtonDisabled(0), 0);
  expectU("single touch allowed", ShimBtnLogMode_allowsSingleTouch(0), 1);
  expectU("single press", ShimBtnLogMode_isDoublePress(0), 0);
  expectU("nothing recorded", ShimBtnLogMode_recordsPresses(0), 0);
  expectU("TTL off", ShimBtnLogMode_isTtlInputEnabled(0), 0);
  expectU("userButtonEnable 0 left alone",
      ShimBtnLogMode_nearestUserButtonEnable(0, 0), 0);
  expectU("userButtonEnable 1 left alone",
      ShimBtnLogMode_nearestUserButtonEnable(0, 1), 1);
}

/* One row per valid combination. userButtonEnable is ignored by all of them,
 * so each is checked with it both ways. */
static void test_mode_table(void)
{
  static const struct
  {
    uint8_t mode, trig;
    uint8_t press, undock, loggingOff, buttonOff, singleTouch, nearestUbe;
    uint8_t doublePress, records;
  } rows[] = {
    /* mode, trig: press undock off btnOff st ube, double records */
    { 1, LOG_TRIG_HOST, 0, 0, 0, 1, 0, 1, 0, 0 },
    { 1, LOG_TRIG_HOST_UNDOCK, 0, 1, 0, 1, 0, 0, 0, 0 },
    { 1, LOG_TRIG_NONE, 0, 0, 1, 1, 0, 1, 0, 0 },
    { 2, LOG_TRIG_HOST, 1, 0, 0, 0, 1, 1, 0, 0 },
    { 2, LOG_TRIG_HOST_UNDOCK, 1, 1, 0, 0, 1, 1, 0, 0 },
    /* Logging off wins: the press still asks, the start check refuses */
    { 2, LOG_TRIG_NONE, 1, 0, 1, 0, 0, 1, 0, 0 },
    { 3, LOG_TRIG_HOST, 1, 0, 0, 0, 1, 1, 1, 0 },
    { 3, LOG_TRIG_HOST_UNDOCK, 1, 1, 0, 0, 1, 1, 1, 0 },
    { 3, LOG_TRIG_NONE, 1, 0, 1, 0, 0, 1, 1, 0 },
    { 4, LOG_TRIG_HOST, 1, 0, 0, 0, 1, 1, 1, 1 },
    { 4, LOG_TRIG_HOST_UNDOCK, 1, 1, 0, 0, 1, 1, 1, 1 },
    /* Presses still recorded, for a host streaming them over Bluetooth */
    { 4, LOG_TRIG_NONE, 1, 0, 1, 0, 0, 1, 1, 1 },
    { 5, LOG_TRIG_HOST, 0, 0, 0, 0, 0, 1, 0, 1 },
    /* The press cannot start logging and leaving the dock does, so byte 217
     * alone says the same with the button off */
    { 5, LOG_TRIG_HOST_UNDOCK, 0, 1, 0, 0, 0, 0, 0, 1 },
    { 5, LOG_TRIG_NONE, 0, 0, 1, 0, 0, 1, 0, 1 },
  };

  unsigned i;
  uint8_t ube, raw;

  testCase("each button mode and log trigger does what the table says");
  for (i = 0; i < sizeof(rows) / sizeof(rows[0]); i++)
  {
    raw = byteOf(rows[i].mode, rows[i].trig);
    for (ube = 0; ube < 2U; ube++)
    {
      expectU("press", ShimBtnLogMode_doesPressToggleLogging(raw, ube), rows[i].press);
      expectU("undock", ShimBtnLogMode_doesUndockStartLogging(raw, ube, 0),
          rows[i].undock);
      expectU("nearest userButtonEnable",
          ShimBtnLogMode_nearestUserButtonEnable(raw, ube), rows[i].nearestUbe);
    }
    expectU("undock never with single touch",
        ShimBtnLogMode_doesUndockStartLogging(raw, 0, 1), 0);
    expectU("logging off", ShimBtnLogMode_isLoggingDisabled(raw), rows[i].loggingOff);
    expectU("button off", ShimBtnLogMode_isButtonDisabled(raw), rows[i].buttonOff);
    expectU("single touch", ShimBtnLogMode_allowsSingleTouch(raw), rows[i].singleTouch);
    expectU("double press", ShimBtnLogMode_isDoublePress(raw), rows[i].doublePress);
    expectU("records presses", ShimBtnLogMode_recordsPresses(raw), rows[i].records);
  }
}

static void test_board_without_button_edges(void)
{
  unsigned v;
  uint8_t raw, without, ube;

  /* Shimmer3R SR48-6-0, whose button drives BOOT0 */
  testCase("taking press recording out changes nothing else");
  expectX("double press, recorded",
      ShimBtnLogMode_withoutPressRecording(byteOf(4, 1)), byteOf(3, 1));
  expectX("recorded only", ShimBtnLogMode_withoutPressRecording(byteOf(5, 2)),
      byteOf(1, 2));
  for (v = 0; v < 256U; v++)
  {
    raw = (uint8_t) v;
    without = ShimBtnLogMode_withoutPressRecording(raw);
    expectU("nothing recorded", ShimBtnLogMode_recordsPresses(without), 0);
    expectU("same log triggers", ShimBtnLogMode_logTriggers(without),
        ShimBtnLogMode_logTriggers(raw));
    expectU("same gesture", ShimBtnLogMode_isDoublePress(without),
        ShimBtnLogMode_isDoublePress(raw));
    expectX("already corrected", ShimBtnLogMode_correct(without), without);
    expectU("same TTL input", ShimBtnLogMode_isTtlInputEnabled(without),
        ShimBtnLogMode_isTtlInputEnabled(raw));
    for (ube = 0; ube < 2U; ube++)
    {
      expectU("same press", ShimBtnLogMode_doesPressToggleLogging(without, ube),
          ShimBtnLogMode_doesPressToggleLogging(raw, ube));
      expectU("same undock", ShimBtnLogMode_doesUndockStartLogging(without, ube, 0),
          ShimBtnLogMode_doesUndockStartLogging(raw, ube, 0));
      expectU("same userButtonEnable",
          ShimBtnLogMode_nearestUserButtonEnable(without, ube),
          ShimBtnLogMode_nearestUserButtonEnable(raw, ube));
    }
  }
}

static void test_fallback_never_adds_an_undock_start(void)
{
  unsigned v;
  uint8_t raw, ube, fallbackUbe;

  /* A host that knows only byte 217 reads userButtonEnable back and writes it
   * again with 0 in byte 232. ShimConfig_checkAndCorrectConfig() holds
   * userButtonEnable at the nearest value, so that is what it writes. */
  testCase("falling back to byte 217 never adds an undock start");
  for (v = 0; v < 256U; v++)
  {
    raw = (uint8_t) v;
    for (ube = 0; ube < 2U; ube++)
    {
      fallbackUbe = ShimBtnLogMode_nearestUserButtonEnable(raw, ube);
      if (ShimBtnLogMode_doesUndockStartLogging(0, fallbackUbe, 0))
      {
        expectTrue("undock start after the fallback was there before it",
            ShimBtnLogMode_doesUndockStartLogging(raw, ube, 0));
      }
    }
  }
}

static void test_sdlog_cfg_round_trip(void)
{
  unsigned v;
  uint8_t fixed, parsed;

  testCase("sdlog.cfg keys set one field each");
  expectX("button_mode=2", ShimBtnLogMode_withButtonMode(0, 2), 0x02);
  expectX("then log_triggers=2", ShimBtnLogMode_withLogTriggers(0x02, 2), 0x12);
  expectX("in either order",
      ShimBtnLogMode_withButtonMode(ShimBtnLogMode_withLogTriggers(0, 2), 2), 0x12);
  expectX("a field replaced, not ORed", ShimBtnLogMode_withButtonMode(0x12, 1), 0x11);
  expectX("ttl_input_enable=1", ShimBtnLogMode_withTtlInput(0x12, 1), 0x32);
  expectX("ttl_input_enable=0", ShimBtnLogMode_withTtlInput(0x32, 0), 0x12);
  expectX("ttl_input_enable=5 is on", ShimBtnLogMode_withTtlInput(0x00, 5), 0x20);

  testCase("a value outside its field makes the whole byte legacy");
  expectX("button_mode=8",
      ShimBtnLogMode_correct(ShimBtnLogMode_withButtonMode(0x12, 8)), 0);
  expectX("button_mode=-1",
      ShimBtnLogMode_correct(ShimBtnLogMode_withButtonMode(0x12, -1)), 0);
  expectX("log_triggers=4",
      ShimBtnLogMode_correct(ShimBtnLogMode_withLogTriggers(0x01, 4)), 0);
  /* atoi() of anything that is not a number */
  expectX("log_triggers=0 from text", ShimBtnLogMode_withLogTriggers(0x02, 0), 0x02);

  testCase("what generate writes, parse reads back");
  for (v = 0; v < 256U; v++)
  {
    fixed = ShimBtnLogMode_correct((uint8_t) v);
    /* The parse starts from ShimConfig_createBlankConfigBytes(), which sets
     * the byte to 0 */
    parsed = ShimBtnLogMode_withButtonMode(0, ShimBtnLogMode_buttonMode(fixed));
    parsed = ShimBtnLogMode_withLogTriggers(parsed, ShimBtnLogMode_logTriggers(fixed));
    parsed = ShimBtnLogMode_withTtlInput(parsed, ShimBtnLogMode_isTtlInputEnabled(fixed));
    expectX("round trip", ShimBtnLogMode_correct(parsed), fixed);
  }
}

int main(void)
{
  printf("shimmer_button_log_mode host tests\n\n");
  hostTestSilenceUnused();

  test_constants();
  test_correct_every_value();
  test_values_no_host_meant();
  test_queries_apply_the_correction();
  test_legacy_is_unchanged();
  test_mode_table();
  test_board_without_button_edges();
  test_fallback_never_adds_an_undock_start();
  test_sdlog_cfg_round_trip();

  return hostTestReport("shimmer_button_log_mode");
}

#endif /* SHIMMER_HOST_TEST */
