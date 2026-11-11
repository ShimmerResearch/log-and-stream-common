/*
 * Host-side test for LEDs/shimmer_leds.c.
 *
 * Built against the stub platform in Test/host/stubs - see stubs/README.md.
 * Run by .github/workflows/host-tests.yml.
 *
 * WHAT THIS IS FOR. The lower LED's priority order: which of the button, the
 * host override, the SD-card error and the battery or charging display wins.
 * docs/SHIMMER3_LED_FEEDBACK.md §4 describes it. The case that started this
 * suite is the SD-card error on a Shimmer3R that is on USB-C but out of the
 * dock: that error used to be gated on the dock alone, so it showed instead of
 * the charging colour on USB, where the card belongs to the PC anyway.
 *
 * The real module is linked, and the LED driver below records what it drew. A
 * pattern is read at two ticks of the 100 ms blink timer, so a 5 Hz flash
 * (off, on), a single blip (on, off) and a steady colour (on, on) each look
 * different.
 *
 * Built TWICE, once per platform: Shimmer3 drives discrete LEDs through a bit
 * mask, Shimmer3R a pair of RGB LEDs.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main() and the
 * LED driver. Without the guard they collide with the firmware's own at link
 * time, in both the STM32 and the MSP430 build. Only the host-test build passes
 * -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "LEDs/shimmer_leds_phase.h"
#include "hal_Board.h"
#include "host_stubs.h"
#include "host_test.h"
#include "log_and_stream_includes.h"

/* Not in the module's header: the lower-LED half of ShimLeds_blink(). */
void ShimLeds_blinkSetLwrState(void);

#if defined(SHIMMER3)
#define COLOUR_RED    LED_LWR_RED
#define COLOUR_YELLOW LED_LWR_YELLOW
#define COLOUR_GREEN  LED_LWR_GREEN
#define COLOUR_OFF    LED_ALL_OFF
#define PLATFORM_NAME "Shimmer3"
#else
#define COLOUR_RED    LED_RGB_RED
#define COLOUR_YELLOW LED_RGB_YELLOW
#define COLOUR_GREEN  LED_RGB_GREEN
#define COLOUR_OFF    LED_RGB_ALL_OFF
#define PLATFORM_NAME "Shimmer3R"
#endif

/* --- the LED driver, recording what was drawn ----------------------------- */

static uint8_t ledMask; /* Shimmer3's discrete LEDs; Shimmer3R's mask calls */
static uint32_t lwrColour; /* Shimmer3R's lower RGB LED, as 0xRRGGBB */
static uint32_t uprColour; /* Shimmer3R's upper RGB LED */

void Board_ledOn(uint8_t mask)
{
  ledMask |= mask;
}

void Board_ledOff(uint8_t mask)
{
  ledMask &= (uint8_t) ~mask;
}

void Board_ledToggle(uint8_t mask)
{
  ledMask ^= mask;
}

uint8_t Board_isLedOnUprGreen(void)
{
  return (ledMask & LED_UPR_GREEN) ? 1U : 0U;
}

#if defined(SHIMMER3R)
static uint32_t packRgb(int16_t red, int16_t green, int16_t blue)
{
  return ((uint32_t) (uint8_t) red << 16) | ((uint32_t) (uint8_t) green << 8)
      | (uint32_t) (uint8_t) blue;
}

void Board_ledLwrSetColour(uint32_t colour)
{
  lwrColour = colour;
}

void Board_ledLwrSetColourRgb(int16_t red, int16_t green, int16_t blue)
{
  lwrColour = packRgb(red, green, blue);
}

void Board_ledUprSetColour(uint32_t colour)
{
  uprColour = colour;
}

void Board_ledUprSetColourRgb(int16_t red, int16_t green, int16_t blue)
{
  uprColour = packRgb(red, green, blue);
}
#endif

/* What the lower LED shows now, as the platform's colour constant. */
static uint32_t lowerLed(void)
{
#if defined(SHIMMER3)
  return ledMask & (LED_LWR_RED | LED_LWR_GREEN | LED_LWR_YELLOW);
#else
  return lwrColour;
#endif
}

/* --- the rest of what the module links against ---------------------------- */

#if defined(SHIMMER3)
uint8_t isRn4678ConnectionEstablished(void)
{
  return 0;
}
#endif

uint8_t ShimSdSync_rcFirstOffsetRxedGet(void)
{
  return 0;
}

uint8_t *ShimSdSync_ledPhaseOffsetPtrGet(void)
{
  static uint8_t offset[LED_PHASE_SYNC_OFFSET_SIZE];
  return offset;
}

uint16_t platform_ledTickElapsedRtcTicks(void)
{
  return 0;
}

void platform_ledTickShift(int16_t rtcTicks)
{
  (void) rtcTicks;
}

/* --- helpers ---------------------------------------------------------------- */

/* Where the sensor is and what its card is doing. USB is a Shimmer3R thing; on
 * Shimmer3 the same flag stays clear in the firmware, but the decision under
 * test is shared code, so both builds run every case. */
typedef struct
{
  uint8_t docked;
  uint8_t usb;
  uint8_t sdInserted;
  uint8_t sdBadFile;
  uint8_t sdErrorEnable;
} where_t;

static void setUp(where_t w)
{
  hostStub_reset();
  ShimLeds_varsInit(); /* frame 0 */

  ledMask = 0;
  lwrColour = 0;
  uprColour = 0;

  shimmerStatus.docked = w.docked;
  shimmerStatus.usbPluggedIn = w.usb;
  hostStub_setDockedOrUsbIn(w.docked || w.usb);
  shimmerStatus.sdInserted = w.sdInserted;
  shimmerStatus.sdBadFile = w.sdBadFile;
  ShimConfig_getStoredConfig()->sdErrorEnable = w.sdErrorEnable;

  /* Charging, steady red, docked or on USB; battery high, green blip, out. */
  batteryStatus.battStatLedFlash = 0;
  batteryStatus.battStatLedCharging = COLOUR_RED;
  batteryStatus.battStatLed = COLOUR_GREEN;
}

/* Draw the lower LED at frame 0 and frame 1. The SD error flashes on odd
 * ticks; the battery blip shows on frame 0 only; a steady colour on both. */
static void drawTwoTicks(uint32_t *atTick0, uint32_t *atTick1)
{
  ShimLeds_blinkSetLwrState();
  *atTick0 = lowerLed();
  ShimLeds_incrementCounters();
  ShimLeds_blinkSetLwrState();
  *atTick1 = lowerLed();
}

static void expectPattern(const char *what, where_t w, uint32_t want0, uint32_t want1)
{
  uint32_t got0, got1;
  char label[96];

  setUp(w);
  drawTwoTicks(&got0, &got1);
  snprintf(label, sizeof(label), "%s, tick 0", what);
  expectX(label, got0, want0);
  snprintf(label, sizeof(label), "%s, tick 1", what);
  expectX(label, got1, want1);
}

/* --- cases ------------------------------------------------------------------ */

static void test_sd_error_out_of_dock(void)
{
  testCase("test_sd_error_out_of_dock");

  /* Out of the dock and off USB: the card is the MCU's, and a fault flashes. */
  expectPattern("no card", (where_t){ 0, 0, 0, 0, 1 }, COLOUR_OFF, COLOUR_GREEN);
  expectPattern("bad file", (where_t){ 0, 0, 1, 1, 1 }, COLOUR_OFF, COLOUR_GREEN);

  /* A healthy card shows the battery blip instead. */
  expectPattern("healthy card", (where_t){ 0, 0, 1, 0, 1 }, COLOUR_GREEN, COLOUR_OFF);

  /* With sdErrorEnable clear there is no card indication at all. */
  expectPattern("no card, SD error disabled", (where_t){ 0, 0, 0, 0, 0 },
      COLOUR_GREEN, COLOUR_OFF);
}

static void test_sd_error_hidden_docked_or_on_usb(void)
{
  testCase("test_sd_error_hidden_docked_or_on_usb");

  /* Docked: the charging colour, whatever the card is doing. */
  expectPattern("docked, no card", (where_t){ 1, 0, 0, 0, 1 }, COLOUR_RED, COLOUR_RED);
  expectPattern("docked, bad file", (where_t){ 1, 0, 1, 1, 1 }, COLOUR_RED, COLOUR_RED);

  /* On USB-C out of the dock: the same. The card belongs to the PC here too,
   * and the user needs to see the unit charging. This is the case that used
   * to flash the SD error. */
  expectPattern("on USB, no card", (where_t){ 0, 1, 0, 0, 1 }, COLOUR_RED, COLOUR_RED);
  expectPattern("on USB, bad file", (where_t){ 0, 1, 1, 1, 1 }, COLOUR_RED, COLOUR_RED);
  expectPattern("docked and on USB, no card", (where_t){ 1, 1, 0, 0, 1 },
      COLOUR_RED, COLOUR_RED);
}

static void test_button_and_override_outrank_sd_error(void)
{
  where_t outNoCard = { 0, 0, 0, 0, 1 };
  uint32_t got0, got1;

  testCase("test_button_and_override_outrank_sd_error");

  setUp(outNoCard);
  shimmerStatus.buttonPressed = 1;
  drawTwoTicks(&got0, &got1);
  expectX("button held, tick 0", got0, COLOUR_GREEN);
  expectX("button held, tick 1", got1, COLOUR_GREEN);

  setUp(outNoCard);
  shimmerStatus.toggleLedRedCmd = 1;
  drawTwoTicks(&got0, &got1);
  expectX("host override, tick 0", got0, COLOUR_RED);
  expectX("host override, tick 1", got1, COLOUR_RED);
}

int main(void)
{
  printf("shimmer_leds host tests (" PLATFORM_NAME ")\n\n");
  hostTestSilenceUnused();

  test_sd_error_out_of_dock();
  test_sd_error_hidden_docked_or_on_usb();
  test_button_and_override_outrank_sd_error();

  return hostTestReport("shimmer_leds (" PLATFORM_NAME ")");
}

#endif /* SHIMMER_HOST_TEST */
