/*
 * Host-test stub for the board HAL.
 *
 * Battery/shimmer_battery.c includes this on both platforms, for the LED colour
 * constants. Their VALUES are load-bearing here: the battery module writes them
 * into batteryStatus.battStatLed / .battStatLedCharging and the host tests
 * assert on exactly what it wrote, so a wrong value would make a green test
 * meaningless.
 *
 * Shimmer3's LED_LWR_* come from the real LEDs/shimmer_leds.h, included below.
 * Shimmer3R's LED_RGB_* live in the firmware's own hal_Board.h and are mirrored
 * here - keep them in step with
 * shimmer3r-firmware/LogAndStream_Shimmer3R/Shimmer_Driver/hal_Board.h.
 *
 * LEDs/shimmer_leds.c also takes the LED driver from here: Shimmer3 includes
 * this header directly, Shimmer3R through shimmer_include.h. The prototypes
 * mirror both firmwares' hal_Board.h. Nothing in host_stubs.c defines them; a
 * suite that links LEDs/shimmer_leds.c defines its own, to record what the
 * state machine drew (see test_leds.c).
 */
#ifndef HOST_TEST_STUB_HAL_BOARD_H
#define HOST_TEST_STUB_HAL_BOARD_H

#include <stdint.h>

#include "LEDs/shimmer_leds.h"

void Board_ledOn(uint8_t ledMask);
void Board_ledOff(uint8_t ledMask);
void Board_ledToggle(uint8_t ledMask);
uint8_t Board_isLedOnUprGreen(void);

#if defined(SHIMMER3R)
#define LED_PWM_ON  255
#define LED_PWM_OFF 0

void Board_ledLwrSetColour(uint32_t ledMask);
void Board_ledLwrSetColourRgb(int16_t red, int16_t green, int16_t blue);
void Board_ledUprSetColour(uint32_t ledMask);
void Board_ledUprSetColourRgb(int16_t red, int16_t green, int16_t blue);

/* Mirrors led_rgb_t in the Shimmer3R firmware's hal_Board.h. */
typedef enum
{
  LED_RGB_ALL_OFF = 0x000000,
  LED_RGB_RED = 0xFF0000,
  LED_RGB_GREEN = 0x00FF00,
  LED_RGB_BLUE = 0x0000FF,
  LED_RGB_YELLOW = 0xFFFF00,
  LED_RGB_PURPLE = 0x800080,
  LED_RGB_ALL_ON = 0xFFFFFF,
} led_rgb_t;
#endif /* SHIMMER3R */

#endif /* HOST_TEST_STUB_HAL_BOARD_H */
