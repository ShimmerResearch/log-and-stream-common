/*
 * Host-side test for the platform seam as the linker sees it: which definition
 * of a platform hook ends up in the image, and that the shared-state header
 * declares its globals rather than defining them.
 *
 * Built against the stub platform in Test/host/stubs - see stubs/README.md.
 * Run by .github/workflows/host-tests.yml.
 *
 * WEAK HOOKS. The library supplies a weak default for every hook a platform
 * may replace (Platform/platform_api.c, Boards/shimmer_boards.c), and each
 * firmware defines its own version of the ones it needs. That works only while
 * the firmware's version is a strong symbol. A weak attribute on the hook's
 * PROTOTYPE makes every definition that sees the header weak, the firmware's
 * included, and the linker then keeps whichever weak copy it meets first. Both
 * firmwares once shipped that way and worked only because main.c happened to
 * be linked ahead of the defaults; GNU ld on Windows refuses the link outright.
 *
 * So this binary is linked with the library's defaults FIRST, ahead of this
 * file and host_stubs.c - the order that exposes the fault. Put the attribute
 * back on a prototype and the cases below fail, or the link does. Every hook
 * here is given a value its default never returns, so a default that wins
 * shows up as a wrong value rather than a coincidental pass.
 *
 * check_weak_decls.py covers the hook headers this binary cannot include.
 *
 * SHARED STATE. log_and_stream_globals.h is included from each firmware's
 * main.c (and, on Shimmer3, from a driver as well). It must only declare
 * shimmerStatus and batteryStatus: host_stubs.c defines both, so a definition
 * in the header collides with it at link time under -fno-common, just as it
 * collides with log_and_stream_common.c in a program-level firmware build.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main() and
 * overrides platform hooks. Without the guard it collides with the firmware's
 * own definitions at link time, in both the STM32 and the MSP430 build. Only
 * the host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "host_stubs.h"
#include "host_test.h"
#include "log_and_stream_globals.h"
#include "log_and_stream_includes.h"

/* --- overrides, as a firmware's main.c would write them -------------------- *
 * Plain definitions, with the library headers in scope and no attribute of
 * their own. Each records that it ran, or returns a value the default never
 * does. platform_reset(), platform_delayMs() and platform_getTick() are
 * overridden in host_stubs.c and checked through it. */

static uint8_t calledProcessHwRevision;
static uint8_t calledInitGpioForRevision;
static uint8_t calledSleepWhenNoTask;
static int16_t ledTickShiftArg;

void platform_processHwRevision(void)
{
  calledProcessHwRevision = 1;
}

void platform_initGpioForRevision(void)
{
  calledInitGpioForRevision = 1;
}

uint8_t platform_gatherData(void)
{
  return 0xA5U; /* the default returns 0 */
}

uint32_t platform_crcData(uint8_t *buf, uint8_t len)
{
  (void) buf;
  (void) len;
  return 0x12345678UL; /* the default returns a 16-bit CRC */
}

uint32_t platform_crcData16(uint8_t *buf, uint16_t len)
{
  (void) buf;
  (void) len;
  return 0x9ABCDEF0UL;
}

bool platform_isDockUartInitialised(void)
{
  return true; /* the default returns false */
}

bool platform_isUsbUartInitialised(void)
{
  return true;
}

void platform_sleepWhenNoTask(void)
{
  calledSleepWhenNoTask = 1;
}

uint16_t platform_ledTickElapsedRtcTicks(void)
{
  return 0x1234U; /* the default returns 0 */
}

void platform_ledTickShift(int16_t rtcTicks)
{
  ledTickShiftArg = rtcTicks;
}

uint8_t ShimBrd_doesDeviceSupportBle(void)
{
  return 1U; /* the default returns 0 */
}

uint8_t ShimBrd_doesDeviceSupportBtClassic(void)
{
  return 1U;
}

/* --- cases ----------------------------------------------------------------- */

static void test_platform_api_overrides_win(void)
{
  uint8_t buf[2] = { 0x01, 0x02 };

  testCase("platform_api.h: a firmware's override beats the weak default");
  hostStub_reset();
  calledProcessHwRevision = 0;
  calledInitGpioForRevision = 0;
  calledSleepWhenNoTask = 0;
  ledTickShiftArg = 0;

  platform_processHwRevision();
  expectTrue("platform_processHwRevision() reached the override", calledProcessHwRevision);
  platform_initGpioForRevision();
  expectTrue("platform_initGpioForRevision() reached the override", calledInitGpioForRevision);
  expectX("platform_gatherData()", platform_gatherData(), 0xA5U);
  expectX("platform_crcData()", platform_crcData(buf, sizeof(buf)), 0x12345678UL);
  expectX("platform_crcData16()", platform_crcData16(buf, sizeof(buf)), 0x9ABCDEF0UL);
  expectTrue("platform_isDockUartInitialised()", platform_isDockUartInitialised());
  expectTrue("platform_isUsbUartInitialised()", platform_isUsbUartInitialised());
  platform_sleepWhenNoTask();
  expectTrue("platform_sleepWhenNoTask() reached the override", calledSleepWhenNoTask);
  expectX("platform_ledTickElapsedRtcTicks()", platform_ledTickElapsedRtcTicks(), 0x1234U);
  platform_ledTickShift(-7);
  expectU("platform_ledTickShift() argument", (uint16_t) ledTickShiftArg, (uint16_t) -7);
}

static void test_overrides_in_another_file_win(void)
{
  testCase("platform_api.h: overrides in another file win too (host_stubs.c)");
  hostStub_reset();

  hostStub_setTick(4321U);
  expectU("platform_getTick()", platform_getTick(), 4321U);
  platform_reset();
  expectU("platform_reset() reached the stub", hostStub_getResetCount(), 1U);
  hostStub_reset(); /* nothing else here should see a reset */
}

static void test_board_overrides_win(void)
{
  testCase("shimmer_boards.h: a firmware's override beats the weak default");
  hostStub_reset();

  expectU("ShimBrd_doesDeviceSupportBle()", ShimBrd_doesDeviceSupportBle(), 1U);
  expectU("ShimBrd_doesDeviceSupportBtClassic()", ShimBrd_doesDeviceSupportBtClassic(), 1U);
}

static void test_globals_are_shared(void)
{
  testCase(
      "log_and_stream_globals.h: declares the globals host_stubs.c defines");
  hostStub_reset();

  /* Linking at all is most of this case. What is left is that the name here
   * and the one hostStub_reset() clears are the same object. */
  shimmerStatus.docked = 1;
  batteryStatus.battStat = BATT_HIGH;
  hostStub_reset();
  expectU("shimmerStatus cleared by host_stubs.c", shimmerStatus.docked, 0U);
  expectU("batteryStatus cleared by host_stubs.c", batteryStatus.battStat, 0U);
}

int main(void)
{
  printf("platform seam host tests\n\n");
  hostTestSilenceUnused();

  test_platform_api_overrides_win();
  test_overrides_in_another_file_win();
  test_board_overrides_win();
  test_globals_are_shared();

  return hostTestReport("platform_seam");
}

#endif /* SHIMMER_HOST_TEST */
