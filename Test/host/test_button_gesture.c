/*
 * Host-side test for Button/shimmer_button_gesture.c.
 *
 * The user button's edges arrive with their times from the interrupt; this is
 * where they are debounced and turned into "start or stop logging". Pinned
 * here: bounce is ignored, an ignored edge that was real comes back at its own
 * time once the pin is rechecked, a missed edge is put back, the single press
 * keeps the button's original rules, and the double press needs two short
 * presses close together.
 *
 * shimmer_button_gesture.c includes nothing but the C library, so this needs
 * no stubs. Run by .github/workflows/host-tests.yml.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include "host_test.h"

#include "Button/shimmer_button_gesture.h"

#define SINGLE 0U
#define DOUBLE 1U

/* Well clear of zero, as the clock is in the firmware */
static const uint64_t T0 = 100000000ULL;

static uint64_t ms(uint32_t m)
{
  return T0 + (uint64_t) m * 32768ULL / 1000ULL;
}

static ShimBtnGesture g;
static ShimBtnGestureResult res;
static uint32_t toggles;

static void edge(uint32_t atMs, uint8_t level, uint8_t mode)
{
  ShimBtnGesture_edge(&g, ms(atMs), level, mode, &res);
  toggles += res.toggleLogging;
}

static void press(uint32_t downMs, uint32_t upMs, uint8_t mode)
{
  edge(downMs, 1, mode);
  edge(upMs, 0, mode);
}

static void reset(void)
{
  ShimBtnGesture_init(&g);
  memset(&res, 0, sizeof(res));
  toggles = 0;
}

static void test_single_press_toggles_on_release(void)
{
  testCase("single press: a release toggles logging");
  reset();
  edge(0, 1, SINGLE);
  expectU("press accepted", res.numEdges, 1);
  expectU("press does not toggle", res.toggleLogging, 0);
  expectU("level after press", g.level, 1);
  edge(200, 0, SINGLE);
  expectU("release accepted", res.numEdges, 1);
  expectU("release toggles", res.toggleLogging, 1);
  expectU("release not synthesized", res.edge[0].synthesized, 0);
  expectU("release at its own time", res.edge[0].ts, ms(200));
}

static void test_single_press_repeat_gate(void)
{
  testCase("single press: a release within 0.5 s of the last does nothing");
  reset();
  press(0, 100, SINGLE);
  press(200, 400, SINGLE); /* 300 ms after the last release */
  expectU("toggles", toggles, 1);
  press(1000, 1100, SINGLE); /* 700 ms after it */
  expectU("toggles", toggles, 2);
}

static void test_single_press_long_press_is_consumed(void)
{
  testCase("single press: a press of 5 s or more does nothing");
  reset();
  press(0, 5100, SINGLE);
  expectU("toggles", toggles, 0);
  press(5000 + 100 + 600, 5000 + 100 + 700, SINGLE);
  expectU("next ordinary press toggles", toggles, 1);
}

static void test_press_bounce_is_ignored(void)
{
  testCase("bounce on a press is ignored, and the release still toggles once");
  reset();
  edge(0, 1, SINGLE);
  edge(2, 0, SINGLE);
  expectU("bounce release ignored", res.numEdges, 0);
  edge(4, 1, SINGLE);
  expectU("bounce press ignored", res.numEdges, 0);
  expectTrue("recheck pending", g.bouncePending);
  expectFalse("recheck not due 20 ms after", ShimBtnGesture_isRecheckDue(&g, ms(24)));
  expectTrue("recheck due 25 ms after", ShimBtnGesture_isRecheckDue(&g, ms(30)));
  ShimBtnGesture_recheck(&g, 1, SINGLE, &res);
  expectU("pin agrees: no edge", res.numEdges, 0);
  expectFalse("recheck done", g.bouncePending);
  edge(300, 0, SINGLE);
  expectU("release accepted", res.numEdges, 1);
  expectU("toggles", toggles, 1);
}

static void test_fast_tap_comes_back_on_recheck(void)
{
  testCase("a tap shorter than the debounce comes back on the recheck");
  reset();
  edge(0, 1, SINGLE);
  edge(15, 0, SINGLE);
  expectU("release ignored at first", res.numEdges, 0);
  ShimBtnGesture_recheck(&g, 0, SINGLE, &res);
  expectU("release put back", res.numEdges, 1);
  expectU("at the ignored edge's time", res.edge[0].ts, ms(15));
  expectU("marked synthesized", res.edge[0].synthesized, 1);
  expectU("level", g.level, 0);
  expectU("toggles", toggles + res.toggleLogging, 1);
}

static void test_missed_edge_is_put_back(void)
{
  testCase("a missed release is put back before the next press");
  reset();
  edge(0, 1, SINGLE);
  edge(1000, 1, SINGLE); /* the release between was never seen */
  expectU("two edges", res.numEdges, 2);
  expectU("first is a release", res.edge[0].level, 0);
  expectU("first synthesized", res.edge[0].synthesized, 1);
  expectU("second is the press", res.edge[1].level, 1);
  expectU("second not synthesized", res.edge[1].synthesized, 0);
  expectU("level", g.level, 1);
}

static void test_missed_edge_uses_the_ignored_one(void)
{
  testCase("a missed release takes the time of an ignored one, when it went "
           "that way");
  reset();
  edge(0, 1, SINGLE);
  edge(10, 0, SINGLE); /* ignored as bounce - but it was real */
  edge(1000, 1, SINGLE);
  expectU("two edges", res.numEdges, 2);
  expectU("release at the ignored time", res.edge[0].ts, ms(10));
}

static void test_double_press_toggles(void)
{
  testCase(
      "double press: two short presses toggle once, on the second release");
  reset();
  press(0, 100, DOUBLE);
  expectU("first press does not toggle", toggles, 0);
  press(250, 380, DOUBLE);
  expectU("second release toggles", toggles, 1);
  press(1500, 1600, DOUBLE);
  expectU("a lone press later does not", toggles, 1);
}

static void test_triple_press_toggles_once(void)
{
  testCase("double press: a third quick press starts a new pair");
  reset();
  press(0, 100, DOUBLE);
  press(200, 300, DOUBLE);
  press(400, 500, DOUBLE);
  expectU("toggles", toggles, 1);
}

static void test_double_press_too_slow(void)
{
  testCase("double press: releases more than 400 ms apart do not toggle");
  reset();
  press(0, 100, DOUBLE);
  press(450, 550, DOUBLE);
  expectU("toggles", toggles, 0);
}

static void test_double_press_needs_short_presses(void)
{
  testCase("double press: a press over 300 ms does not count");
  reset();
  press(0, 350, DOUBLE);
  press(450, 550, DOUBLE);
  expectU("long first press", toggles, 0);
  reset();
  press(0, 100, DOUBLE);
  press(150, 490, DOUBLE);
  expectU("long second press", toggles, 0);
}

static void test_single_press_never_toggles_in_double_mode(void)
{
  testCase("double press mode: single presses are markers only");
  reset();
  press(0, 100, DOUBLE);
  press(2000, 2100, DOUBLE);
  press(4000, 9500, DOUBLE);
  expectU("toggles", toggles, 0);
}

int main(void)
{
  hostTestSilenceUnused();

  test_single_press_toggles_on_release();
  test_single_press_repeat_gate();
  test_single_press_long_press_is_consumed();
  test_press_bounce_is_ignored();
  test_fast_tap_comes_back_on_recheck();
  test_missed_edge_is_put_back();
  test_missed_edge_uses_the_ignored_one();
  test_double_press_toggles();
  test_triple_press_toggles_once();
  test_double_press_too_slow();
  test_double_press_needs_short_presses();
  test_single_press_never_toggles_in_double_mode();

  return hostTestReport("test_button_gesture");
}

#endif /* SHIMMER_HOST_TEST */
