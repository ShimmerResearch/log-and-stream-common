/*
 * The bit of scaffolding every host test needs: an assertion that keeps going.
 *
 * A test binary here runs to the end and reports every failure it found, rather
 * than stopping at the first. One firmware change usually breaks a family of
 * related cases, and the shape of the whole family says more about what went
 * wrong than its first member does.
 *
 * Nothing here is a framework. There is no runner, no registration, no
 * discovery - main() calls the cases in order, which keeps a test readable as
 * the sequence of firmware calls it is meant to describe.
 *
 * WHY THE GUARD IN EVERY .c HERE - do not remove it. Both consuming firmware
 * projects add this repository as a source-path root with no exclusions, so
 * every .c file under it is compiled into the firmware. The test files define
 * main(). Without the guard they collide with the firmware's own main() at link
 * time, on both the STM32 and the MSP430. Only the host-test build passes
 * -DSHIMMER_HOST_TEST. This header has no guard of its own because it declares
 * nothing - it is only ever included from a file that has one.
 */
#ifndef HOST_TEST_HOST_TEST_H
#define HOST_TEST_HOST_TEST_H

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int hostTestFailures;
static const char *hostTestCase = "";

/* Name the case being run. Printed with each failure, so a report identifies
 * itself without the reader counting lines. */
static void testCase(const char *name)
{
  hostTestCase = name;
  printf("%s\n", name);
}

/* Widened to uint64_t at the call so one helper serves uint8_t through
 * uint32_t. PRIu64 rather than %lu: uint32_t is unsigned long on ARM and
 * unsigned int on x86-64, so a bare length modifier is only accidentally right
 * on whichever of the two you happen to build for. */
static void expectU(const char *what, uint64_t got, uint64_t want)
{
  if (got != want)
  {
    printf("  FAIL [%s] %-44s got %" PRIu64 " want %" PRIu64 "\n", hostTestCase,
        what, got, want);
    hostTestFailures++;
  }
}

/* Same, printed in hex - for register values, colour codes and CRCs, where the
 * decimal form of a wrong answer tells you nothing about how it is wrong. */
static void expectX(const char *what, uint64_t got, uint64_t want)
{
  if (got != want)
  {
    printf("  FAIL [%s] %-44s got 0x%" PRIX64 " want 0x%" PRIX64 "\n",
        hostTestCase, what, got, want);
    hostTestFailures++;
  }
}

static void expectStr(const char *what, const char *got, const char *want)
{
  if (strcmp(got, want) != 0)
  {
    printf("  FAIL [%s] %-44s got \"%s\" want \"%s\"\n", hostTestCase, what, got, want);
    hostTestFailures++;
  }
}

/* For a predicate, where "got 0 want 1" is less use than the claim itself. */
static void expectTrue(const char *what, int cond)
{
  if (!cond)
  {
    printf("  FAIL [%s] %s\n", hostTestCase, what);
    hostTestFailures++;
  }
}

static void expectFalse(const char *what, int cond)
{
  expectTrue(what, !cond);
}

/* Print the tally and hand back the process exit code. */
static int hostTestReport(const char *suite)
{
  printf("\n%s: %s (%d failure%s)\n", suite, hostTestFailures ? "FAILED" : "PASSED",
      hostTestFailures, hostTestFailures == 1 ? "" : "s");
  return hostTestFailures ? 1 : 0;
}

/* Every helper above is static in a single-translation-unit test binary, so any
 * one a given suite does not use draws -Wunused-function under -Werror. This
 * keeps them all live without a test having to reference them. */
static void hostTestSilenceUnused(void)
{
  (void) expectU;
  (void) expectX;
  (void) expectStr;
  (void) expectTrue;
  (void) expectFalse;
  (void) testCase;
  (void) hostTestReport;
  (void) hostTestSilenceUnused;
}

#endif /* HOST_TEST_HOST_TEST_H */
