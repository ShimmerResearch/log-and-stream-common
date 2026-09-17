/*
 * Host-side test for Battery/shimmer_battery.c.
 *
 * Built against the stub platform in Test/host/stubs - see stubs/README.md.
 * Run by .github/workflows/host-tests.yml.
 *
 * WHAT THIS IS FOR. Three pieces of state that a user reads off the device, and
 * one that can end a recording:
 *
 *   battStat            the three-level fuel gauge, WITH HYSTERESIS
 *   battChargingStatus  what the charger chip is doing
 *   battStatLed*        the colour, and whether it flashes
 *   battCritical        the low-battery auto-stop
 *
 * The hysteresis is the reason this is worth a test. The four thresholds
 * overlap by 50 ADC counts each so that a battery sitting on a boundary does
 * not make the LED flicker between two colours, which means the answer depends
 * on the PREVIOUS state as well as the current voltage - and that is not
 * something you can check by reading the function, or by watching a bench unit
 * for an afternoon. Every one of the nine state-by-threshold combinations is
 * covered below.
 *
 * The auto-stop matters for the opposite reason: it is the one path here that
 * can stop a recording on its own, and it takes three consecutive readings to
 * fire. Getting that wrong in either direction is bad - too eager ends
 * recordings early, too slow flattens the cell.
 *
 * This suite is built TWICE, once for each platform, because the LED constants
 * differ: Shimmer3 drives discrete LEDs (LED_LWR_*, 8-bit) and Shimmer3R an RGB
 * pair (LED_RGB_*, 24-bit). The colour assertions are #if'd accordingly; the
 * logic assertions are shared.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Without the
 * guard it collides with the firmware's own main() at link time, in both the
 * STM32 and the MSP430 build. Only the host-test build passes
 * -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hal_Board.h" /* LED colour constants, as shimmer_battery.c sees them */
#include "host_stubs.h"
#include "host_test.h"
#include "log_and_stream_includes.h"

/* The colour the platform under test uses for each meaning. */
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

/* A voltage comfortably inside the charger chip's believable window, so the
 * ranking tests are not accidentally answering the out-of-range question. */
#define MV_NOMINAL 3900

/* Put the gauge in a known state, then rank one ADC reading. Returns the level
 * the module settled on. */
static uint8_t rankFrom(uint8_t startingState, uint16_t adcVal)
{
  batteryStatus.battStat = startingState;
  batteryStatus.battStatusRaw.adcBattVal = adcVal;
  ShimBatt_rankBattUndockedVoltage();
  return batteryStatus.battStat;
}

/*
 * The hysteresis, exhaustively: each of the three states, against each of the
 * four thresholds and the points either side of them.
 *
 *   BATT_MID_MIN  2568   fall to LOW at or below this
 *   BATT_LOW_MAX  2618   rise out of LOW at or above this
 *   BATT_HIGH_MIN 2717   fall out of HIGH below this
 *   BATT_MID_MAX  2767   rise to HIGH at or above this
 *
 * The two 50-count overlaps (2568..2618 and 2717..2767) are the dead bands. A
 * reading inside one must leave the state alone - that is the whole point.
 */
static void test_undocked_voltage_hysteresis(void)
{
  testCase("test_undocked_voltage_hysteresis");
  hostStub_reset();

  /* From MID. */
  expectU("MID + 2567 -> LOW", rankFrom(BATT_MID, BATT_MID_MIN - 1), BATT_LOW);
  expectU("MID + 2568 stays MID", rankFrom(BATT_MID, BATT_MID_MIN), BATT_MID);
  expectU("MID + 2766 stays MID", rankFrom(BATT_MID, BATT_MID_MAX - 1), BATT_MID);
  expectU("MID + 2767 -> HIGH", rankFrom(BATT_MID, BATT_MID_MAX), BATT_HIGH);

  /* From LOW: it takes 2618, not 2568, to climb out. */
  expectU("LOW + 2567 stays LOW", rankFrom(BATT_LOW, BATT_MID_MIN - 1), BATT_LOW);
  expectU("LOW + 2568 stays LOW (dead band)", rankFrom(BATT_LOW, BATT_MID_MIN), BATT_LOW);
  expectU("LOW + 2617 stays LOW (dead band)", rankFrom(BATT_LOW, BATT_LOW_MAX - 1), BATT_LOW);
  expectU("LOW + 2618 -> MID", rankFrom(BATT_LOW, BATT_LOW_MAX), BATT_MID);
  expectU("LOW + 2767 -> HIGH, skipping MID", rankFrom(BATT_LOW, BATT_MID_MAX), BATT_HIGH);

  /* From HIGH: it takes a drop below 2717, not below 2767, to fall out. */
  expectU("HIGH + 2766 stays HIGH (dead band)",
      rankFrom(BATT_HIGH, BATT_MID_MAX - 1), BATT_HIGH);
  expectU("HIGH + 2717 stays HIGH (dead band)", rankFrom(BATT_HIGH, BATT_HIGH_MIN), BATT_HIGH);
  expectU("HIGH + 2716 -> MID", rankFrom(BATT_HIGH, BATT_HIGH_MIN - 1), BATT_MID);
  expectU("HIGH + 2568 -> MID", rankFrom(BATT_HIGH, BATT_MID_MIN), BATT_MID);
  expectU("HIGH + 2567 -> LOW, skipping MID", rankFrom(BATT_HIGH, BATT_MID_MIN - 1), BATT_LOW);

  /* The property the dead bands exist for: a battery resting on a boundary,
   * jittering by a count either way, must not change the LED colour. A gauge
   * without hysteresis fails this and flickers. */
  {
    uint8_t state = BATT_MID;
    uint8_t i;
    uint8_t changes = 0;
    for (i = 0; i < 20; i++)
    {
      uint8_t next = rankFrom(state, (uint16_t) (BATT_MID_MIN + (i & 1U)));
      if (next != state)
      {
        changes++;
      }
      state = next;
    }
    expectU("jitter across a threshold does not flicker", changes, 0);
  }
}

/* The gauge decides the colour when the device is running on its own battery. */
static void test_undocked_led_colour(void)
{
  testCase("test_undocked_led_colour");
  hostStub_reset();

  rankFrom(BATT_HIGH, BATT_MID_MIN - 1);
  expectX("LOW is red", batteryStatus.battStatLed, COLOUR_RED);
  rankFrom(BATT_LOW, BATT_LOW_MAX);
  expectX("MID is yellow", batteryStatus.battStatLed, COLOUR_YELLOW);
  rankFrom(BATT_MID, BATT_MID_MAX);
  expectX("HIGH is green", batteryStatus.battStatLed, COLOUR_GREEN);

  /* An unrecognised level must not leave the LED off - a dark LED reads as a
   * flat device. The default arm shows red. */
  batteryStatus.battStat = 0x77;
  ShimBatt_determineUndockedLedState();
  expectX("an unknown level falls back to red", batteryStatus.battStatLed, COLOUR_RED);

  /* Power-on: before any reading has been ranked, the gauge starts at MID
   * rather than at zero, so a freshly booted device shows yellow rather than
   * the fallback red. */
  ShimBatt_resetBatteryUndockedStatus();
  expectU("reset starts the gauge at MID", batteryStatus.battStat, BATT_MID);
  expectX("and shows yellow", batteryStatus.battStatLed, COLOUR_YELLOW);
}

/*
 * The charger chip reports through two open-drain pins, STAT1 and STAT2, which
 * the firmware reads as bits 6 and 7 of one byte. Both pins are active low.
 */
static void test_charging_status_ranking(void)
{
  testCase("test_charging_status_ranking");
  hostStub_reset();

  batteryStatus.battValMV = MV_NOMINAL;

  batteryStatus.battStatusRaw.rawBytes[2] = CHRG_CHIP_STATUS_SUSPENDED;
  ShimBatt_rankBattChargingStatus();
  expectU("STAT1+STAT2 off -> suspended", batteryStatus.battChargingStatus,
      CHARGING_STATUS_SUSPENDED);

  batteryStatus.battStatusRaw.rawBytes[2] = CHRG_CHIP_STATUS_FULLY_CHARGED;
  ShimBatt_rankBattChargingStatus();
  expectU("STAT2 on -> fully charged", batteryStatus.battChargingStatus,
      CHARGING_STATUS_FULLY_CHARGED);

  /* Preconditioning is reported to the user as plain charging - the distinction
   * is not one a user can act on. */
  batteryStatus.battStatusRaw.rawBytes[2] = CHRG_CHIP_STATUS_PRECONDITIONING;
  ShimBatt_rankBattChargingStatus();
  expectU("preconditioning is reported as charging",
      batteryStatus.battChargingStatus, CHARGING_STATUS_CHARGING);

  batteryStatus.battStatusRaw.rawBytes[2] = CHRG_CHIP_STATUS_BAD_BATTERY;
  ShimBatt_rankBattChargingStatus();
  expectU("both pins on -> bad battery", batteryStatus.battChargingStatus,
      CHARGING_STATUS_BAD_BATTERY);

  batteryStatus.battStatusRaw.rawBytes[2] = CHRG_CHIP_STATUS_UNKNOWN;
  ShimBatt_rankBattChargingStatus();
  expectU("0xFF -> unknown", batteryStatus.battChargingStatus, CHARGING_STATUS_UNKNOWN);

  /* Anything else is a chip that is not answering the way its datasheet says.
   * It must be distinguishable from "unknown", because one is a missing reading
   * and the other is a wrong one. */
  batteryStatus.battStatusRaw.rawBytes[2] = 0x20;
  ShimBatt_rankBattChargingStatus();
  expectU("an undefined code -> error", batteryStatus.battChargingStatus, CHARGING_STATUS_ERROR);

  /* An implausible voltage overrides everything: the reading is not to be
   * trusted, whatever the pins say. */
  batteryStatus.battValMV = BATTERY_ERROR_VOLTAGE_MAX + 1;
  batteryStatus.battStatusRaw.rawBytes[2] = CHRG_CHIP_STATUS_FULLY_CHARGED;
  ShimBatt_rankBattChargingStatus();
  expectU("an impossible voltage -> checking, whatever the pins say",
      batteryStatus.battChargingStatus, CHARGING_STATUS_CHECKING);

  batteryStatus.battValMV = BATTERY_ERROR_VOLTAGE_MAX;
  ShimBatt_rankBattChargingStatus();
  expectU("exactly at the limit is still believed",
      batteryStatus.battChargingStatus, CHARGING_STATUS_FULLY_CHARGED);
}

/*
 * The STAT1/STAT2 bitfield layout. batteryStatus.battStatusRaw is a union of
 * three raw bytes and a packed bitfield struct, and the firmware writes one
 * view and reads the other - so the bit positions are a compiler-dependent
 * detail that the charging status silently depends on.
 *
 * This is the assertion that catches a toolchain packing bitfields the other
 * way round, which would swap every charger state for its opposite.
 */
static void test_charger_bitfield_layout(void)
{
  testCase("test_charger_bitfield_layout");
  hostStub_reset();

  expectU("the raw view is 3 bytes", sizeof(batteryStatus.battStatusRaw), 3);

  batteryStatus.battStatusRaw.rawBytes[2] = 0;
  batteryStatus.battStatusRaw.STAT1 = 1;
  expectX("STAT1 is bit 6", batteryStatus.battStatusRaw.rawBytes[2], 0x40);

  batteryStatus.battStatusRaw.rawBytes[2] = 0;
  batteryStatus.battStatusRaw.STAT2 = 1;
  expectX("STAT2 is bit 7", batteryStatus.battStatusRaw.rawBytes[2], 0x80);

  batteryStatus.battStatusRaw.rawBytes[2] = 0;
  batteryStatus.battStatusRaw.STAT1 = 1;
  batteryStatus.battStatusRaw.STAT2 = 1;
  expectX("both set is the SUSPENDED code",
      batteryStatus.battStatusRaw.rawBytes[2], CHRG_CHIP_STATUS_SUSPENDED);

  /* The ADC value occupies the first two bytes of the same union, little-endian
   * - this is the layout the dock and Bluetooth links carry. */
  memset((void *) batteryStatus.battStatusRaw.rawBytes, 0, 3);
  batteryStatus.battStatusRaw.adcBattVal = 0x1234;
  expectX("ADC low byte first", batteryStatus.battStatusRaw.rawBytes[0], 0x34);
  expectX("ADC high byte second", batteryStatus.battStatusRaw.rawBytes[1], 0x12);
}

/* The charging colour is only driven while the device is docked or on USB. */
static void test_charging_led_colour(void)
{
  testCase("test_charging_led_colour");
  hostStub_reset();

  hostStub_setDockedOrUsbIn(1);

  batteryStatus.battChargingStatus = CHARGING_STATUS_CHARGING;
  ShimBatt_determineChargingLedState();
  expectX("charging is red", batteryStatus.battStatLedCharging, COLOUR_RED);
  expectU("and does not flash", batteryStatus.battStatLedFlash, 0);

  batteryStatus.battChargingStatus = CHARGING_STATUS_CHECKING;
  ShimBatt_determineChargingLedState();
  expectX("checking is red too", batteryStatus.battStatLedCharging, COLOUR_RED);

  batteryStatus.battChargingStatus = CHARGING_STATUS_FULLY_CHARGED;
  ShimBatt_determineChargingLedState();
  expectX("fully charged is green", batteryStatus.battStatLedCharging, COLOUR_GREEN);

  batteryStatus.battChargingStatus = CHARGING_STATUS_SUSPENDED;
  ShimBatt_determineChargingLedState();
  expectX("suspended is yellow", batteryStatus.battStatLedCharging, COLOUR_YELLOW);
  expectU("and does not flash", batteryStatus.battStatLedFlash, 0);

  /* The two fault states are the only ones that flash. That is the signal a
   * user is meant to act on, so it has to be distinguishable from charging -
   * same colour, different behaviour. */
  batteryStatus.battChargingStatus = CHARGING_STATUS_BAD_BATTERY;
  ShimBatt_determineChargingLedState();
  expectX("bad battery is red", batteryStatus.battStatLedCharging, COLOUR_RED);
  expectU("and flashes", batteryStatus.battStatLedFlash, 1);

  batteryStatus.battChargingStatus = CHARGING_STATUS_ERROR;
  ShimBatt_determineChargingLedState();
  expectX("error is red", batteryStatus.battStatLedCharging, COLOUR_RED);
  expectU("and flashes", batteryStatus.battStatLedFlash, 1);

  /* Unknown means the firmware has nothing to say, so it says nothing rather
   * than showing a colour that would be read as a claim. */
  batteryStatus.battChargingStatus = CHARGING_STATUS_UNKNOWN;
  ShimBatt_determineChargingLedState();
  expectX("unknown shows nothing", batteryStatus.battStatLedCharging, COLOUR_OFF);

  /* Undocked, the charging path is not evaluated at all - but the flash flag is
   * still cleared, so a fault flash cannot outlive the dock. */
  batteryStatus.battChargingStatus = CHARGING_STATUS_BAD_BATTERY;
  ShimBatt_determineChargingLedState();
  expectU("flash set while docked", batteryStatus.battStatLedFlash, 1);
  hostStub_setDockedOrUsbIn(0);
  ShimBatt_determineChargingLedState();
  expectU("undocking clears the fault flash", batteryStatus.battStatLedFlash, 0);
}

/*
 * ShimBatt_updateStatus() is the whole path, as the battery timer calls it.
 */
static void test_update_status_path(void)
{
  testCase("test_update_status_path");
  hostStub_reset();

  /* Undocked, the charger pins are meaningless - there is nothing driving them
   * - so they must be discarded rather than reported as a chip state. */
  hostStub_setDockedOrUsbIn(0);
  ShimBatt_updateStatus(2700, MV_NOMINAL, 1, 1);
  expectX("undocked discards the charger pins",
      batteryStatus.battStatusRaw.rawBytes[2], CHRG_CHIP_STATUS_UNKNOWN);
  expectU("and reports the charging status as unknown",
      batteryStatus.battChargingStatus, CHARGING_STATUS_UNKNOWN);
  expectU("the ADC reading is stored", batteryStatus.battStatusRaw.adcBattVal, 2700);
  expectU("the millivolt reading is stored", batteryStatus.battValMV, MV_NOMINAL);

  /* Docked, they are read. The argument order is (STAT1, STAT2) and both pins
   * are active low, so "fully charged" - 0x40, STAT2 asserted - is passed as
   * STAT1 high, STAT2 low. Getting this pair the wrong way round swaps
   * "charging" for "fully charged" on a real device, which is exactly the kind
   * of thing that is easier to catch here than on a bench. */
  hostStub_setDockedOrUsbIn(1);
  ShimBatt_updateStatus(2700, MV_NOMINAL, 1, 0);
  expectX("docked reads the charger pins",
      batteryStatus.battStatusRaw.rawBytes[2], CHRG_CHIP_STATUS_FULLY_CHARGED);
  expectU("and ranks them", batteryStatus.battChargingStatus, CHARGING_STATUS_FULLY_CHARGED);

  ShimBatt_updateStatus(2700, MV_NOMINAL, 0, 1);
  expectX("the other pin order is preconditioning",
      batteryStatus.battStatusRaw.rawBytes[2], CHRG_CHIP_STATUS_PRECONDITIONING);
  expectU("reported as charging", batteryStatus.battChargingStatus, CHARGING_STATUS_CHARGING);

  /* A charger reporting "suspended" on a cell that is actually flat is really
   * a bad battery, and is re-reported as one. This is the only place the
   * firmware second-guesses the chip. */
  ShimBatt_updateStatus(2400, BATTERY_ERROR_VOLTAGE_MIN, 1, 1);
  expectX("suspended at or below 3200 mV is reported as a bad battery",
      batteryStatus.battStatusRaw.rawBytes[2], CHRG_CHIP_STATUS_BAD_BATTERY);
  expectU("and ranks as one", batteryStatus.battChargingStatus, CHARGING_STATUS_BAD_BATTERY);

  /* One millivolt higher and the chip is taken at its word. */
  ShimBatt_updateStatus(2400, BATTERY_ERROR_VOLTAGE_MIN + 1, 1, 1);
  expectX("suspended above 3200 mV stays suspended",
      batteryStatus.battStatusRaw.rawBytes[2], CHRG_CHIP_STATUS_SUSPENDED);
}

/*
 * The low-battery auto-stop. Opt-in via config, and it takes three readings
 * below the cutoff before it acts - one dip must not end a recording.
 */
static void test_low_battery_auto_stop(void)
{
  testCase("test_low_battery_auto_stop");
  hostStub_reset();

  hostStub_setDockedOrUsbIn(0);
  shimmerStatus.sensing = 1;
  ShimConfig_getStoredConfig()->lowBatteryAutoStop = 1;

  /* Above the cutoff, nothing happens however long it runs. */
  {
    uint8_t i;
    for (i = 0; i < 10; i++)
    {
      ShimBatt_updateStatus(BATT_CUTOFF_3_65VOLTS, MV_NOMINAL, 0, 0);
    }
  }
  expectU("at the cutoff exactly, nothing is counted", batteryStatus.battCriticalCount, 0);
  expectU("and sensing is not stopped", hostStub_getStopSensingCount(), 0);

  /* Below it, the count rises - but the first two readings must not act. */
  ShimBatt_updateStatus(BATT_CUTOFF_3_65VOLTS - 1, MV_NOMINAL, 0, 0);
  expectU("first low reading counts", batteryStatus.battCriticalCount, 1);
  expectU("but does not stop sensing", hostStub_getStopSensingCount(), 0);

  ShimBatt_updateStatus(BATT_CUTOFF_3_65VOLTS - 1, MV_NOMINAL, 0, 0);
  expectU("second low reading counts", batteryStatus.battCriticalCount, 2);
  expectU("and still does not stop sensing", hostStub_getStopSensingCount(), 0);

  ShimBatt_updateStatus(BATT_CUTOFF_3_65VOLTS - 1, MV_NOMINAL, 0, 0);
  expectU("third low reading counts", batteryStatus.battCriticalCount, 3);
  expectU("and stops sensing", hostStub_getStopSensingCount(), 1);
  expectU("the critical flag latches", batteryStatus.battCritical, 1);

  /* Disabled in config, the cutoff is never reached however low the cell goes.
   * This is the setting a user turns off to run a device to flat deliberately. */
  hostStub_reset();
  hostStub_setDockedOrUsbIn(0);
  shimmerStatus.sensing = 1;
  ShimConfig_getStoredConfig()->lowBatteryAutoStop = 0;
  {
    uint8_t i;
    for (i = 0; i < 10; i++)
    {
      ShimBatt_updateStatus(0, MV_NOMINAL, 0, 0);
    }
  }
  expectU("auto-stop disabled: nothing counted", batteryStatus.battCriticalCount, 0);
  expectU("auto-stop disabled: sensing continues", hostStub_getStopSensingCount(), 0);

  /* Not sensing, the count still rises but there is nothing to stop. */
  hostStub_reset();
  hostStub_setDockedOrUsbIn(0);
  shimmerStatus.sensing = 0;
  ShimConfig_getStoredConfig()->lowBatteryAutoStop = 1;
  {
    uint8_t i;
    for (i = 0; i < 5; i++)
    {
      ShimBatt_updateStatus(0, MV_NOMINAL, 0, 0);
    }
  }
  expectU("idle: the count still rises", batteryStatus.battCriticalCount, 5);
  expectU("idle: nothing is stopped", hostStub_getStopSensingCount(), 0);

  /* PINNED, NOT ENDORSED. The count is cumulative across an undocked session,
   * not a run of consecutive readings: a good reading does not clear it, so
   * three low readings spread over hours will stop a recording just as three in
   * a row would. LogAndStream_setupDock() is the only thing that resets it,
   * deliberately, so that docking lets logging start again. Asserted here so
   * that anyone who expects "three consecutive" finds out from a test. */
  hostStub_reset();
  hostStub_setDockedOrUsbIn(0);
  shimmerStatus.sensing = 1;
  ShimConfig_getStoredConfig()->lowBatteryAutoStop = 1;
  ShimBatt_updateStatus(BATT_CUTOFF_3_65VOLTS - 1, MV_NOMINAL, 0, 0);
  ShimBatt_updateStatus(2700, MV_NOMINAL, 0, 0); /* a good reading between */
  expectU("a good reading does not clear the count", batteryStatus.battCriticalCount, 1);
  ShimBatt_updateStatus(BATT_CUTOFF_3_65VOLTS - 1, MV_NOMINAL, 0, 0);
  ShimBatt_updateStatus(BATT_CUTOFF_3_65VOLTS - 1, MV_NOMINAL, 0, 0);
  expectU("three non-consecutive low readings still stop sensing",
      hostStub_getStopSensingCount(), 1);
}

/* ShimBatt_init() has to leave the device showing something sensible before any
 * reading has been taken. */
static void test_init_state(void)
{
  testCase("test_init_state");
  hostStub_reset();

  /* Dirty the state first, so a missing reset shows up. */
  batteryStatus.battCritical = 1;
  batteryStatus.battCriticalCount = 9;
  batteryStatus.battStat = BATT_LOW;

  ShimBatt_init();

  expectU("critical flag cleared", batteryStatus.battCritical, 0);
  expectU("critical count cleared", batteryStatus.battCriticalCount, 0);
  expectU("gauge starts at MID, not flat", batteryStatus.battStat, BATT_MID);
  expectX("charger status starts unknown",
      batteryStatus.battStatusRaw.rawBytes[2], CHRG_CHIP_STATUS_UNKNOWN);
  expectU("charging status starts unknown", batteryStatus.battChargingStatus,
      CHARGING_STATUS_UNKNOWN);
}

/* The battery is polled every 2 s docked and every 60 s undocked. */
static void test_battery_interval(void)
{
  testCase("test_battery_interval");
  hostStub_reset();

  ShimBatt_setBatteryInterval(BATT_INTERVAL_SECS_DOCKED);
  expectU("docked interval reads back", ShimBatt_getBatteryInterval(), BATT_INTERVAL_SECS_DOCKED);
  ShimBatt_setBatteryInterval(BATT_INTERVAL_SECS_UNDOCKED);
  expectU("undocked interval reads back", ShimBatt_getBatteryInterval(),
      BATT_INTERVAL_SECS_UNDOCKED);

#if defined(SHIMMER3)
  /* Shimmer3 turns the interval into 32768 Hz timer ticks. The products are
   * 65536 and 1966080 - both beyond a 16-bit int, which is what MSP430 has, so
   * this is worth asserting rather than assuming. */
  ShimBatt_setBatteryInterval(BATT_INTERVAL_SECS_DOCKED);
  expectU("2 s is 65536 ticks", ShimBatt_getBatteryIntervalTicks(), 65536U);
  ShimBatt_setBatteryInterval(BATT_INTERVAL_SECS_UNDOCKED);
  expectU("60 s is 1966080 ticks", ShimBatt_getBatteryIntervalTicks(), 1966080U);
#endif
}

int main(void)
{
  printf("shimmer_battery host tests (" PLATFORM_NAME ")\n\n");
  hostTestSilenceUnused();

  test_undocked_voltage_hysteresis();
  test_undocked_led_colour();
  test_charging_status_ranking();
  test_charger_bitfield_layout();
  test_charging_led_colour();
  test_update_status_path();
  test_low_battery_auto_stop();
  test_init_state();
  test_battery_interval();

  return hostTestReport("shimmer_battery (" PLATFORM_NAME ")");
}

#endif /* SHIMMER_HOST_TEST */
