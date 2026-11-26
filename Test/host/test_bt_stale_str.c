/*
 * Host-side test for Comms/shimmer_bt_stale_str.h: when the Shimmer3 drops a
 * module status string the module has stopped sending.
 *
 * On a bench RN4678 unit, a status string that lost a byte while the device
 * logged left the parser one string out of step with the module. A string's
 * closing '%', read in command position, opened a string the module never sent.
 * That phantom took the start of the module's next string, and the end of that
 * string opened the next phantom, connection after connection. The connect's
 * "%RFCOMM_OPEN%" was never seen again, and the device stayed silent until it
 * was reset. The snapshot of a unit in that state was always the same: a
 * status string open at one character, waiting for 6 more bytes that were not
 * coming. Nothing re-armed it, because the receive DMA was armed throughout.
 *
 * ShimBt_rxDropStaleStatusString() calls the decision tested here ten times a
 * second with the receive's registers. These cases are the calls.
 *
 * Guarded like every suite here: both firmware projects compile every .c in
 * this repository, and this file defines main(). Only the host-test build
 * passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include "host_test.h"

#include "Comms/shimmer_bt_stale_str.h"

static ShimBtStaleStr_t watch;

static uint8_t tick(uint8_t isOpen, uint16_t remaining, uint8_t index)
{
  return ShimBt_staleStatusStrTick(&watch, isOpen, remaining, index);
}

static void reset(void)
{
  ShimBtStaleStr_t fresh = { 0 };
  watch = fresh;
}

static void testPhantomFromTheBench(void)
{
  testCase(
      "a phantom string waiting for 6 bytes is dropped on the second tick");
  reset();
  /* The snapshot's state: '%' read, 6 more bytes asked for, none arriving */
  expectU("first tick", tick(1, 6, 1), 0);
  expectU("second tick", tick(1, 6, 1), 1);
}

static void testLiveStringNeverDropped(void)
{
  testCase("a string still gaining bytes is never dropped");
  reset();
  /* The DMA counts down as bytes arrive */
  expectU("6 left", tick(1, 6, 1), 0);
  expectU("4 left", tick(1, 4, 1), 0);
  expectU("1 left", tick(1, 1, 1), 0);
  /* The next step of the same string: more asked for, the string longer */
  expectU("next step", tick(1, 6, 7), 0);
  expectU("2 left", tick(1, 2, 7), 0);
}

static void testNewStepIsProgress(void)
{
  testCase("a string that moved to its next step is progress, not stale");
  reset();
  /* A step can ask for as many bytes as the last one did: only the length
   * of the string so far tells them apart */
  expectU("first step", tick(1, 6, 1), 0);
  expectU("second step, same count", tick(1, 6, 7), 0);
  expectU("still there", tick(1, 6, 7), 1);
}

static void testNothingOpen(void)
{
  testCase("no status string open: nothing is dropped");
  reset();
  expectU("tick 1", tick(0, 1, 9), 0);
  expectU("tick 2", tick(0, 1, 9), 0);
  expectU("tick 3", tick(0, 1, 9), 0);
}

static void testClosedBetweenTicks(void)
{
  testCase("a string that ended between two ticks starts the watch afresh");
  reset();
  expectU("open", tick(1, 6, 1), 0);
  expectU("ended", tick(0, 1, 9), 0);
  /* Another string, caught at the same point as the last one */
  expectU("new string, first sight", tick(1, 6, 1), 0);
  expectU("new string, still there", tick(1, 6, 1), 1);
}

static void testOneDropPerTwoTicks(void)
{
  testCase("after a drop the watch needs another full tick");
  reset();
  expectU("first", tick(1, 6, 1), 0);
  expectU("dropped", tick(1, 6, 1), 1);
  /* Had the drop not happened (a byte arrived during the last look), the
   * string is given a whole tick again rather than dropped at once */
  expectU("seen again", tick(1, 6, 1), 0);
  expectU("dropped again", tick(1, 6, 1), 1);
}

int main(void)
{
  hostTestSilenceUnused();
  testPhantomFromTheBench();
  testLiveStringNeverDropped();
  testNewStepIsProgress();
  testNothingOpen();
  testClosedBetweenTicks();
  testOneDropPerTwoTicks();
  return hostTestReport("test_bt_stale_str");
}

#endif /* SHIMMER_HOST_TEST */
