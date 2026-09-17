/*
 * The PC "platform" - see host_stubs.h.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is discovered and compiled into the firmware. This file defines
 * shimmerStatus and batteryStatus, which log_and_stream_globals.h also defines;
 * without the guard they collide at link time in both the STM32 and the MSP430
 * build. The guard makes the translation unit empty for anyone who does not ask
 * for it; only the host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <stdint.h>
#include <string.h>

#include "host_stubs.h"
#include "log_and_stream_includes.h"

/* The two globals log_and_stream_globals.h defines for the firmware. The
 * firmware includes that header from exactly one translation unit; the host
 * tests define them here instead, for the same reason. */
STATTypeDef shimmerStatus;
BattStatus batteryStatus;

static uint8_t stubDockedOrUsbIn;
static uint8_t stubRwcTimeSet;
static uint8_t stubEepromPresent;
static uint8_t stubBmp280InUse;
static uint32_t stubTick;
static uint8_t stubRtcErrorFlash;
static uint32_t stubStopSensingCount;
static uint32_t stubResetCount;
/* The real gConfigBytes union - see ShimConfig_getStoredConfig() below. */
static gConfigBytes stubStoredConfig;

void hostStub_reset(void)
{
  memset((void *) &shimmerStatus, 0, sizeof(shimmerStatus));
  memset((void *) &batteryStatus, 0, sizeof(batteryStatus));
  memset(&stubStoredConfig, 0, sizeof(stubStoredConfig));
  stubDockedOrUsbIn = 0;
  stubRwcTimeSet = 0;
  stubEepromPresent = 1; /* the common case: SR31-7-0 and later carry one */
  stubBmp280InUse = 0;
  stubTick = 0;
  stubRtcErrorFlash = 0;
  stubStopSensingCount = 0;
  stubResetCount = 0;
}

void hostStub_setDockedOrUsbIn(uint8_t state)
{
  stubDockedOrUsbIn = state;
}

void hostStub_setRwcTimeSet(uint8_t state)
{
  stubRwcTimeSet = state;
}

void hostStub_setEepromPresent(uint8_t present)
{
  stubEepromPresent = present;
}

void hostStub_setTick(uint32_t tick)
{
  stubTick = tick;
}

uint8_t hostStub_getRtcErrorFlash(void)
{
  return stubRtcErrorFlash;
}

uint32_t hostStub_getStopSensingCount(void)
{
  return stubStopSensingCount;
}

uint32_t hostStub_getResetCount(void)
{
  return stubResetCount;
}

/* --- Configuration/shimmer_config.h --------------------------------------- *
 * The REAL gConfigBytes - Configuration/shimmer_config.h is host-clean, so the
 * union, its overlay and every bitfield are the firmware's own. Only the
 * accessor is stubbed, because Configuration/shimmer_config.c itself is not yet
 * host-buildable (it needs the ADXL371 and LSM6DSV driver headers). A test sets
 * the fields it cares about and hostStub_reset() zeroes the rest. */
gConfigBytes *ShimConfig_getStoredConfig(void)
{
  return &stubStoredConfig;
}

/* --- log_and_stream_common.h ---------------------------------------------- */

uint8_t LogAndStream_isDockedOrUsbIn(void)
{
  return stubDockedOrUsbIn;
}

/* --- log_and_stream_externs.h --------------------------------------------- */

uint8_t RTC_isRwcTimeSet(void)
{
  return stubRwcTimeSet;
}

uint32_t RTC_get32(void)
{
  return stubTick / 1000U;
}

uint64_t RTC_get64(void)
{
  return (uint64_t) stubTick * 32768U / 1000U;
}

/* --- LEDs/shimmer_leds.h --------------------------------------------------- *
 * Overriding the real ShimLeds_setRtcErrorFlash() rather than linking
 * LEDs/shimmer_leds.c: that module needs the RN4X and PWM headers, and all a
 * test needs to know is what the caller asked for. */
void ShimLeds_setRtcErrorFlash(uint8_t state)
{
  stubRtcErrorFlash = state;
}

uint8_t ShimLeds_getRtcErrorFlash(void)
{
  return stubRtcErrorFlash;
}

/* --- EEPROM/shimmer_eeprom.h ---------------------------------------------- */

uint8_t ShimEeprom_isPresent(void)
{
  return stubEepromPresent;
}

/* --- TaskList/shimmer_taskList.h ------------------------------------------ */

void ShimTask_setStopSensing(void)
{
  stubStopSensingCount++;
}

void ShimTask_setStopLogging(void)
{
}

void ShimTask_setStopStreaming(void)
{
}

void ShimTask_setStartLoggingIfReady(void)
{
}

void ShimTask_setStartStreamingIfReady(void)
{
}

/* --- Sensing/shimmer_sensing.h -------------------------------------------- */

uint8_t ShimSens_isSamplingRateInvalid(void)
{
  return 0;
}

/* Called by the weak platform_gatherData(). Nothing on a host has sensors to
 * gather from; the sampling path itself is covered by test_packet_ring.c. */
void ShimSens_gatherData(void)
{
}

/* --- Platform/platform_api.h ---------------------------------------------- *
 * These are __weak in platform_api.c, so the definitions here win at link time
 * without the test having to exclude that file. */

void platform_reset(void)
{
  stubResetCount++;
}

void platform_delayMs(const uint32_t delay_time_ms)
{
  (void) delay_time_ms;
}

uint32_t platform_getTick(void)
{
  return stubTick;
}

/* --- BMPX80 (Shimmer3 only) ------------------------------------------------ */

void hostStub_setBmp280InUse(uint8_t inUse)
{
  stubBmp280InUse = inUse;
}

#if defined(SHIMMER3)
uint8_t isBmp280InUse(void)
{
  return stubBmp280InUse;
}

uint8_t isBmp180InUse(void)
{
  return !stubBmp280InUse;
}
#endif

#endif /* SHIMMER_HOST_TEST */
