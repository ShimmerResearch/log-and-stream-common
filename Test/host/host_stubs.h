/*
 * The PC "platform" for the host tests, and the knobs tests use to steer it.
 *
 * log_and_stream_externs.h declares what each platform firmware must implement.
 * Shimmer3 implements it against the MSP430 HAL, Shimmer3R against the STM32
 * HAL, and host_stubs.c against nothing at all - the externs are no-ops holding
 * a value a test can set. Anything a module needs that is not here is telling
 * you the module reached around the abstraction.
 *
 * Call hostStub_reset() at the top of every test case. The globals below are
 * file-scope state shared by every test in a binary, so a case that does not
 * reset inherits whatever the previous one left behind - which is how a suite
 * starts passing in one order and failing in another.
 */
#ifndef HOST_TEST_HOST_STUBS_H
#define HOST_TEST_HOST_STUBS_H

#include <stdint.h>

/* Put every stub and both globals back to their power-on values. */
void hostStub_reset(void);

/* --- platform state a test can steer -------------------------------------- */

/* What LogAndStream_isDockedOrUsbIn() reports. Drives the battery module's
 * choice between the charging and the undocked LED paths. */
void hostStub_setDockedOrUsbIn(uint8_t state);

/* What RTC_isRwcTimeSet() reports - whether the real-world clock has been set
 * since power-on. ShimRtc_rwcErrorCheck() turns the RTC error flash on when
 * this is false and the config enables it. */
void hostStub_setRwcTimeSet(uint8_t state);

/* What ShimEeprom_isPresent() reports. Boards fitted before Shimmer3 SR31-7-0
 * carry no EEPROM, and ShimBrd_isRn4678PresentAndCmdModeSupport() gates on it. */
void hostStub_setEepromPresent(uint8_t present);

/* The free-running millisecond tick platform_getTick() returns. */
void hostStub_setTick(uint32_t tick);

/* What isBmp280InUse() reports (Shimmer3 only). Second-generation Shimmer3
 * boards pair a BMP280 with an LSM303AHTR, which is what
 * ShimBrd_are2ndGenImuSensorsPresent() looks for. */
void hostStub_setBmp280InUse(uint8_t inUse);

/* --- what the module under test did to the platform ----------------------- */

/* Latest state passed to ShimLeds_setRtcErrorFlash(). */
uint8_t hostStub_getRtcErrorFlash(void);

/* Number of times ShimTask_setStopSensing() was called. The low-battery cutoff
 * is supposed to raise it exactly once, on the sample that crosses the count. */
uint32_t hostStub_getStopSensingCount(void);

/* Number of times platform_reset() was called. Nothing under host test should
 * ever reboot the device; a non-zero count is a failure in itself. */
uint32_t hostStub_getResetCount(void);

#endif /* HOST_TEST_HOST_STUBS_H */
