/*
 * shimmer_button_log_mode.h
 *
 * Configuration byte 232 (NV_BUTTON_LOG_MODE): what the user button does, and
 * what else may start SD logging. Kept free of any platform dependency so every
 * one of its 256 values can be host-tested
 * (Test/host/test_button_log_mode.c). docs/SHIMMER3_CONFIGURATION_INFOMEM.md
 * section 8.3 has the table and the reasons behind each rule.
 *
 *    bits 0-2  button mode   BTN_MODE_*
 *    bits 3-4  log triggers  LOG_TRIG_*, read only when the button mode is not
 *                            BTN_MODE_LEGACY
 *    bits 5-6  unused, cleared
 *    bit 7     must be 0
 *
 * 0 means "as before": userButtonEnable (byte 217 bit 5) decides both what a
 * press does and whether leaving the dock starts logging, exactly as it did
 * before this byte existed. Hosts that do not know the byte write 0 there or
 * leave it alone, so that is what an older setup reads as. Anything this
 * firmware cannot honour reads as 0 too: bit 7 set (erased flash is 0xFF), a
 * reserved value, or a button mode that is not built yet.
 * ShimBtnLogMode_correct() is that rule; every query below applies it first,
 * so a byte nobody has corrected is never read any other way.
 */

#ifndef LOG_AND_STREAM_COMMON_CONFIGURATION_SHIMMER_BUTTON_LOG_MODE_H_
#define LOG_AND_STREAM_COMMON_CONFIGURATION_SHIMMER_BUTTON_LOG_MODE_H_

#include <stdint.h>

#define BTN_LOG_MODE_BUTTON_MASK   0x07U
#define BTN_LOG_MODE_TRIGGER_SHIFT 3U
#define BTN_LOG_MODE_TRIGGER_MASK  0x18U
#define BTN_LOG_MODE_INVALID       0x80U

/* Bits 0-2: what a press does. Values 3-7 are reserved and read as
 * BTN_MODE_LEGACY; 3-5 are kept for double-press and event-marker modes. */
#define BTN_MODE_LEGACY            0U /* userButtonEnable decides */
#define BTN_MODE_DISABLED          1U /* nothing, and no LED while held */
#define BTN_MODE_SINGLE_PRESS      2U /* a press starts and stops logging */

/* Bits 3-4: what else may start logging. 3 is reserved and reads as
 * BTN_MODE_LEGACY. */
#define LOG_TRIG_HOST              0U /* a host command only */
#define LOG_TRIG_HOST_UNDOCK       1U /* a host command, or leaving the dock */
#define LOG_TRIG_NONE              2U /* nothing: SD logging is off */

/* The byte this firmware would act on: raw itself when it is valid, 0 when
 * it is not. Bits 5 and 6 are cleared. */
uint8_t ShimBtnLogMode_correct(uint8_t raw);

uint8_t ShimBtnLogMode_buttonMode(uint8_t raw);
uint8_t ShimBtnLogMode_logTriggers(uint8_t raw);

/* For the sdlog.cfg keys: raw with one field replaced. A value outside the
 * field sets bit 7, so the whole byte reads as BTN_MODE_LEGACY rather than as
 * whatever the low bits of a typing error happen to be. */
uint8_t ShimBtnLogMode_withButtonMode(uint8_t raw, int value);
uint8_t ShimBtnLogMode_withLogTriggers(uint8_t raw, int value);

/* Whether a press starts and stops logging. Starting still has to pass the
 * logging start check, which LOG_TRIG_NONE fails. */
uint8_t ShimBtnLogMode_doesPressToggleLogging(uint8_t raw, uint8_t userButtonEnable);

/* Whether leaving the dock, or on Shimmer3R unplugging USB-C, starts logging.
 * Never with single touch, where the sync centre's press starts every node. */
uint8_t ShimBtnLogMode_doesUndockStartLogging(uint8_t raw,
    uint8_t userButtonEnable,
    uint8_t singleTouchStart);

uint8_t ShimBtnLogMode_isLoggingDisabled(uint8_t raw);
uint8_t ShimBtnLogMode_isButtonDisabled(uint8_t raw);

/* Whether single touch can work: it needs a press that can start logging. */
uint8_t ShimBtnLogMode_allowsSingleTouch(uint8_t raw);

/* The userButtonEnable that comes closest to the mode, for hosts and firmware
 * that know only byte 217. 0 only where a press cannot start logging and
 * leaving the dock does; anything else gets 1, so falling back to it never
 * adds an undock start. A legacy byte returns userButtonEnable unchanged. */
uint8_t ShimBtnLogMode_nearestUserButtonEnable(uint8_t raw, uint8_t userButtonEnable);

#endif /* LOG_AND_STREAM_COMMON_CONFIGURATION_SHIMMER_BUTTON_LOG_MODE_H_ */
