/*
 * Host-side test for the board revision gates in Boards/shimmer_boards.c.
 *
 * Built against the stub platform in Test/host/stubs - see stubs/README.md.
 * Run by .github/workflows/host-tests.yml, and cross-checked against the gate
 * table in docs/SHIMMER3_BOARD_REVISIONS.md by crosscheck_board_revisions.py.
 *
 * WHAT THIS IS FOR. These predicates decide which sensors the firmware believes
 * are fitted, from three bytes of EEPROM - board ID, major rev, minor rev. Get
 * one wrong and the firmware talks to a chip that is not there, or skips one
 * that is, on every unit of that revision. There is no runtime symptom short of
 * a sensor that reads zero.
 *
 * They also cannot be covered by bench testing in any useful sense. Asserting
 * ShimBrd_isBmp581PresentPerSrNumber() across its boundaries needs eight
 * different PCB revisions on the desk, several of which are dev builds that were
 * never made in quantity. Here each one is three bytes.
 *
 * AGENTS.md names these functions as the authority for what is fitted - above
 * the tables in docs/SHIMMER3_BOARD_REVISIONS.md, which are a conversion of a
 * hardware workbook that is not in this repository. That makes this file the
 * executable form of that authority, and crosscheck_board_revisions.py the
 * thing that reports when the document and the firmware have drifted apart.
 *
 * Built TWICE, once per platform: several gates are compiled out on the other
 * one, and a few (ShimBrd_areADS1292RClockLinesTied) answer differently on each.
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

#include "host_stubs.h"
#include "host_test.h"
#include "log_and_stream_includes.h"

#if defined(SHIMMER3)
#define PLATFORM_NAME "Shimmer3"
#else
#define PLATFORM_NAME "Shimmer3R"
#endif

/* Set the three EEPROM bytes that every gate below reads, and the host board
 * ID. This is the whole input space. */
static void setBoard(uint8_t hwId, uint8_t srId, uint8_t major, uint8_t minor)
{
  uint8_t page[3];

  page[0] = srId;
  page[1] = major;
  page[2] = minor;

  ShimBrd_setHwId(hwId);
  ShimBrd_resetDaughterCardId();
  ShimBrd_setDaugherCardIdMemory(0, page, sizeof(page));
}

/* "SR48-7-2" as the hardware team writes it. */
static const char *srName(uint8_t srId, uint8_t major, uint8_t minor)
{
  static char buf[24];
  snprintf(buf, sizeof(buf), "SR%u-%u-%u", srId, major, minor);
  return buf;
}

/*
 * The daughter-card ID is only believed when it has been programmed. 0x00 and
 * 0xFF both mean "not programmed" and must both be rejected - 0xFF because that
 * is what an erased EEPROM holds and what a failed read leaves behind, 0x00
 * because zeroes would otherwise look like a real board, SR0-0-0, to host
 * software (DEV-1019).
 */
static void test_daughter_card_id_set(void)
{
  testCase("test_daughter_card_id_set");
  hostStub_reset();

  ShimBrd_init();
  expectFalse("a reset card ID is not set", ShimBrd_isDaughterCardIdSet());
  expectX("and reads back as 0xFF, not 0x00", ShimBrd_getDaughtCardIdPtr()[0], 0xFF);

  setBoard(HW_ID_SHIMMER3R, 0x00, 0, 0);
  expectFalse("an all-zero ID is not set", ShimBrd_isDaughterCardIdSet());

  setBoard(HW_ID_SHIMMER3R, 0xFF, 0xFF, 0xFF);
  expectFalse("an erased ID is not set", ShimBrd_isDaughterCardIdSet());

  setBoard(HW_ID_SHIMMER3R, SHIMMER3_IMU, 11, 0);
  expectTrue("a programmed ID is set", ShimBrd_isDaughterCardIdSet());
}

/* The two comparison primitives every gate is built from. Their boundaries are
 * where the revision logic actually lives. */
static void test_sr_number_comparisons(void)
{
  testCase("test_sr_number_comparisons");
  hostStub_reset();

  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 7, 2);

  expectTrue("exact match", ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 7, 2));
  expectFalse("wrong minor", ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 7, 1));
  expectFalse("wrong major", ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 8, 2));
  expectFalse("wrong board", ShimBrd_isBoardSrNumber(EXP_BRD_EXG_UNIFIED, 7, 2));

  /* Gte compares major first, then minor - not a packed number, so 7.10 is
   * above 7.2 and 8.0 is above 7.99. */
  expectTrue("gte: equal", ShimBrd_isBoardSrNumberGte(EXP_BRD_GSR_UNIFIED, 7, 2));
  expectTrue("gte: lower minor", ShimBrd_isBoardSrNumberGte(EXP_BRD_GSR_UNIFIED, 7, 1));
  expectTrue("gte: lower major", ShimBrd_isBoardSrNumberGte(EXP_BRD_GSR_UNIFIED, 6, 9));
  expectFalse("gte: higher minor", ShimBrd_isBoardSrNumberGte(EXP_BRD_GSR_UNIFIED, 7, 3));
  expectFalse("gte: higher major", ShimBrd_isBoardSrNumberGte(EXP_BRD_GSR_UNIFIED, 8, 0));
  expectFalse("gte: a different board never matches",
      ShimBrd_isBoardSrNumberGte(EXP_BRD_EXG_UNIFIED, 1, 0));

  /* A major above the threshold wins regardless of the minor. */
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 8, 0);
  expectTrue("gte: 8.0 is above 7.2",
      ShimBrd_isBoardSrNumberGte(EXP_BRD_GSR_UNIFIED, 7, 2));

  /* An unprogrammed card matches nothing, whatever is asked. */
  setBoard(HW_ID_SHIMMER3R, 0xFF, 0xFF, 0xFF);
  expectFalse("unprogrammed matches no exact number",
      ShimBrd_isBoardSrNumber(0xFF, 0xFF, 0xFF));
  expectFalse("unprogrammed matches no gte", ShimBrd_isBoardSrNumberGte(0xFF, 0, 0));
}

/*
 * ShimBrd_isBmp581PresentPerSrNumber() - the gate with the interesting shape.
 *
 * The BMP581 replaces the BMP390 on up-rev'd Shimmer3R boards. Five production
 * lines take it from their ".2" minor, and SR48 has a sixth case: a dev build
 * at 7-2 that carries the part, while 8-0 and 8-1 - which are LATER boards - do
 * not. So the SR48 rule cannot be a plain ">= 7.2"; it is "7.2 or above, but
 * below 8.0" OR ">= 8.2", and SR48-8-0 and SR48-8-1 falling in the hole between
 * them is the entire point.
 *
 * Every board either side of every boundary is below. This is the case the
 * whole file is here for.
 */
typedef struct
{
  uint8_t srId;
  uint8_t major;
  uint8_t minor;
  uint8_t expected;
  const char *why;
} Bmp581Case;

static void test_bmp581_gate(void)
{
  static const Bmp581Case cases[] = {
    /* IMU, SR31: from 11.2 */
    { SHIMMER3_IMU, 11, 1, 0, "one minor below the line" },
    { SHIMMER3_IMU, 11, 2, 1, "the first board with it" },
    { SHIMMER3_IMU, 11, 3, 1, "a later minor keeps it" },
    { SHIMMER3_IMU, 12, 0, 1, "a later major keeps it" },
    { SHIMMER3_IMU, 10, 9, 0, "an earlier major never has it" },

    /* Proto3 Deluxe, SR38: from 4.2 */
    { EXP_BRD_PROTO3_DELUXE, 4, 1, 0, "one minor below the line" },
    { EXP_BRD_PROTO3_DELUXE, 4, 2, 1, "the first board with it" },
    { EXP_BRD_PROTO3_DELUXE, 5, 0, 1, "a later major keeps it" },

    /* ExG, SR47: from 8.2 - note 7.x never has it, unlike SR48 */
    { EXP_BRD_EXG_UNIFIED, 7, 2, 0, "SR47 has no 7-2 dev build" },
    { EXP_BRD_EXG_UNIFIED, 8, 1, 0, "one minor below the line" },
    { EXP_BRD_EXG_UNIFIED, 8, 2, 1, "the first board with it" },
    { EXP_BRD_EXG_UNIFIED, 9, 0, 1, "a later major keeps it" },

    /* Bridge Amplifier, SR49: from 4.2 */
    { EXP_BRD_BR_AMP_UNIFIED, 4, 1, 0, "one minor below the line" },
    { EXP_BRD_BR_AMP_UNIFIED, 4, 2, 1, "the first board with it" },

    /* GSR+, SR48: the two-window case. */
    { EXP_BRD_GSR_UNIFIED, 6, 0, 0, "the earliest proto" },
    { EXP_BRD_GSR_UNIFIED, 7, 0, 0, "below the dev build" },
    { EXP_BRD_GSR_UNIFIED, 7, 1, 0, "the BOOT0 ECO rev, still no BMP581" },
    { EXP_BRD_GSR_UNIFIED, 7, 2, 1, "the dev build that carries it" },
    { EXP_BRD_GSR_UNIFIED, 7, 3, 1, "above the dev build, same major" },
    { EXP_BRD_GSR_UNIFIED, 8, 0, 0, "LATER board, but back to the BMP390" },
    { EXP_BRD_GSR_UNIFIED, 8, 1, 0, "still the BMP390" },
    { EXP_BRD_GSR_UNIFIED, 8, 2, 1, "production line picks it up again" },
    { EXP_BRD_GSR_UNIFIED, 8, 3, 1, "and keeps it" },
    { EXP_BRD_GSR_UNIFIED, 9, 0, 1, "a later major keeps it" },

    /* A board ID with no BMP581 rule at all. */
    { EXP_BRD_PROTO3_MINI, 9, 9, 0, "no rule for this board ID" },
  };
  unsigned i;

  testCase("test_bmp581_gate");
  hostStub_reset();

  for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
  {
    char what[80];
    setBoard(HW_ID_SHIMMER3R, cases[i].srId, cases[i].major, cases[i].minor);
    snprintf(what, sizeof(what), "%s: %s",
        srName(cases[i].srId, cases[i].major, cases[i].minor), cases[i].why);
    expectU(what, ShimBrd_isBmp581PresentPerSrNumber() ? 1 : 0, cases[i].expected);
  }

  /* The gate is Shimmer3R-only: the same EEPROM bytes on a Shimmer3 host board
   * must not claim a BMP581. A daughter card can be moved between hosts. */
  setBoard(HW_ID_SHIMMER3, EXP_BRD_GSR_UNIFIED, 8, 2);
  expectFalse("a Shimmer3 host never reports a BMP581",
      ShimBrd_isBmp581PresentPerSrNumber());

  /* And an unprogrammed card must not either - the 0xFF,0xFF,0xFF page would
   * satisfy every ">=" test if isDaughterCardIdSet() were not checked first. */
  setBoard(HW_ID_SHIMMER3R, 0xFF, 0xFF, 0xFF);
  expectFalse("an unprogrammed card does not claim a BMP581",
      ShimBrd_isBmp581PresentPerSrNumber());
}

/*
 * The ".1" minors dropped the LIS3MDL alt-magnetometer and the ADXL371 high-g
 * accel - except on the IMU board, which kept the ADXL371. Both gates list
 * exact revisions rather than ranges, so each entry is its own case.
 */
static void test_lis3mdl_and_adxl371_gates(void)
{
  testCase("test_lis3mdl_and_adxl371_gates");
  hostStub_reset();

  /* LIS3MDL: named revisions only. */
  setBoard(HW_ID_SHIMMER3R, SHIMMER3_IMU, 11, 0);
  expectTrue("SR31-11-0 has the LIS3MDL", ShimBrd_isLis3mdlPresent());
  setBoard(HW_ID_SHIMMER3R, SHIMMER3_IMU, 11, 1);
  expectFalse("SR31-11-1 dropped it", ShimBrd_isLis3mdlPresent());
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 7, 1);
  expectTrue("SR48-7-1 has it", ShimBrd_isLis3mdlPresent());
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 8, 0);
  expectFalse("SR48-8-0 does not", ShimBrd_isLis3mdlPresent());
  setBoard(HW_ID_SHIMMER3, EXP_BRD_GSR_UNIFIED, 7, 1);
  expectFalse("and never on a Shimmer3 host", ShimBrd_isLis3mdlPresent());

  /* ADXL371: the IMU board is the exception that keeps it at .1. */
  setBoard(HW_ID_SHIMMER3R, SHIMMER3_IMU, 11, 0);
  expectTrue("SR31-11-0 has the ADXL371", ShimBrd_isAdxl371Present());
  setBoard(HW_ID_SHIMMER3R, SHIMMER3_IMU, 11, 1);
  expectTrue("SR31-11-1 keeps the ADXL371 - the IMU exception", ShimBrd_isAdxl371Present());
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 8, 0);
  expectFalse("SR48-8-0 dropped it", ShimBrd_isAdxl371Present());
}

/* SR48-6-0 is the earliest Shimmer3R proto and is special in three places. */
static void test_sr48_6_0_special_cases(void)
{
  testCase("test_sr48_6_0_special_cases");
  hostStub_reset();

  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 6, 0);
  expectTrue("SR48-6-0 is recognised", ShimBrd_isBoardSr48_6_0());
  expectFalse("and has no ADS7028", isAds7028Present());
  expectTrue("so it uses the MCU ADCs for sensing", ShimBrd_areMcuAdcsUsedForSensing());
  expectTrue("and keeps the default nBOOT0", ShimBrd_checkCorrectStateForBoot0());

  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 7, 0);
  expectFalse("SR48-7-0 is not SR48-6-0", ShimBrd_isBoardSr48_6_0());
  expectTrue("and does have an ADS7028", isAds7028Present());
  expectFalse("so it does not use the MCU ADCs", ShimBrd_areMcuAdcsUsedForSensing());
  expectTrue("but it does keep the default nBOOT0 - inverted circuitry",
      ShimBrd_checkCorrectStateForBoot0());

  /* The ECO rev that removed the inverting circuitry. */
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 7, 1);
  expectFalse("SR48-7-1 drives nBOOT0 low", ShimBrd_checkCorrectStateForBoot0());
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_EXG_UNIFIED, 7, 0);
  expectTrue("SR47-7-0 keeps the default nBOOT0", ShimBrd_checkCorrectStateForBoot0());
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_EXG_UNIFIED, 7, 1);
  expectFalse("SR47-7-1 drives it low", ShimBrd_checkCorrectStateForBoot0());

  /* Everything newer drives it low, matching Shimmer3 for dock compatibility. */
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 8, 0);
  expectFalse("SR48-8-0 drives it low", ShimBrd_checkCorrectStateForBoot0());
  setBoard(HW_ID_SHIMMER3, SHIMMER3_IMU, 10, 0);
  expectFalse("a Shimmer3 drives it low", ShimBrd_checkCorrectStateForBoot0());

  /* I2C4 is only wired on two of the boards. */
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 8, 0);
  expectTrue("GSR+ supports I2C4", ShimBrd_isI2c4Supported());
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_PROTO3_DELUXE, 4, 0);
  expectTrue("Proto3 Deluxe supports I2C4", ShimBrd_isI2c4Supported());
  setBoard(HW_ID_SHIMMER3R, SHIMMER3_IMU, 11, 0);
  expectFalse("the IMU board does not", ShimBrd_isI2c4Supported());

  /* On SR48-7-x the PPG I2C bus hangs off the ADC chip. */
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 7, 0);
  expectTrue("SR48-7-0 PPG I2C is ADC-controlled", ShimBrd_isI2cOnPPGControlledByAdcChip());
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 7, 1);
  expectTrue("SR48-7-1 too", ShimBrd_isI2cOnPPGControlledByAdcChip());
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 8, 0);
  expectFalse("SR48-8-0 is not", ShimBrd_isI2cOnPPGControlledByAdcChip());
}

/* The ExG front end is on several boards, and the two ADS1292R clocks are tied
 * together on some of them - which changes how the driver starts them. */
static void test_ads1292_gates(void)
{
  testCase("test_ads1292_gates");
  hostStub_reset();

  setBoard(HW_ID_SHIMMER3, EXP_BRD_EXG_UNIFIED, 4, 0);
  expectTrue("the unified ExG board carries an ADS1292", ShimBrd_isAds1292Present());
  setBoard(HW_ID_SHIMMER3, EXP_BRD_EXG, 1, 0);
  expectTrue("so does the original ExG board", ShimBrd_isAds1292Present());
  setBoard(HW_ID_SHIMMER3, SHIMMER_ECG_MD, 1, 0);
  expectTrue("and the ECG MD", ShimBrd_isAds1292Present());
  setBoard(HW_ID_SHIMMER3, EXP_BRD_GSR_UNIFIED, 4, 0);
  expectFalse("the GSR board does not", ShimBrd_isAds1292Present());

  /* Clock lines: on Shimmer3 from ExG rev 4, on Shimmer3R always. */
  setBoard(HW_ID_SHIMMER3, EXP_BRD_EXG_UNIFIED, 3, 0);
  expectFalse("S3 ExG rev 3: clocks separate", ShimBrd_areADS1292RClockLinesTied());
  setBoard(HW_ID_SHIMMER3, EXP_BRD_EXG_UNIFIED, 4, 0);
  expectTrue("S3 ExG rev 4: clocks tied", ShimBrd_areADS1292RClockLinesTied());
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_EXG_UNIFIED, 7, 0);
  expectTrue("S3R ExG: always tied", ShimBrd_areADS1292RClockLinesTied());
  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 8, 0);
  expectFalse("S3R GSR: no ADS1292 to tie", ShimBrd_areADS1292RClockLinesTied());
}

#if defined(SHIMMER3)
/*
 * Shimmer3-only gates. The RN4678 one is the interesting one: it has a
 * deliberate escape hatch so that a board whose SR number has not been
 * programmed yet still boots on the factory-test bench.
 */
static void test_shimmer3_only_gates(void)
{
  testCase("test_shimmer3_only_gates");
  hostStub_reset();

  hostStub_setEepromPresent(1);

  setBoard(HW_ID_SHIMMER3, SHIMMER3_IMU, 10, 0);
  expectTrue("SR31-10-0 has an RN4678", ShimBrd_isRn4678PresentAndCmdModeSupport());
  setBoard(HW_ID_SHIMMER3, SHIMMER3_IMU, 9, 0);
  expectFalse("SR31-9-0 does not", ShimBrd_isRn4678PresentAndCmdModeSupport());
  setBoard(HW_ID_SHIMMER3, EXP_BRD_GSR_UNIFIED, 5, 0);
  expectTrue("SR48-5-0 does", ShimBrd_isRn4678PresentAndCmdModeSupport());
  setBoard(HW_ID_SHIMMER3, EXP_BRD_GSR_UNIFIED, 4, 9);
  expectFalse("SR48-4-9 does not - major only, minor is ignored",
      ShimBrd_isRn4678PresentAndCmdModeSupport());

  /* The factory-test escape hatch: a completely unprogrammed board is assumed
   * to have the RN4678, so it can be brought up far enough to be programmed. */
  setBoard(HW_ID_SHIMMER3, 0xFF, 0xFF, 0xFF);
  expectTrue("an unprogrammed board is assumed to have one",
      ShimBrd_isRn4678PresentAndCmdModeSupport());

  /* No EEPROM fitted rules the whole thing out, which is what keeps older
   * sensors out of the escape hatch above. */
  hostStub_setEepromPresent(0);
  setBoard(HW_ID_SHIMMER3, SHIMMER3_IMU, 10, 0);
  expectFalse("no EEPROM: no RN4678 claim", ShimBrd_isRn4678PresentAndCmdModeSupport());
  hostStub_setEepromPresent(1);

  /* The WR accel substitution list - exact revisions, and only when the gyro in
   * use is the ICM-20948. */
  setBoard(HW_ID_SHIMMER3, SHIMMER3_IMU, 9, 1);
  ShimBrd_setGyroInUse(GYRO_ICM20948_IN_USE);
  expectTrue("SR31-9-1 with an ICM-20948 needs the substitution",
      ShimBrd_isSubstitutionNeededForWrAccel());
  ShimBrd_setGyroInUse(GYRO_MPU9X50_IN_USE);
  expectFalse("the same board with an MPU9x50 does not",
      ShimBrd_isSubstitutionNeededForWrAccel());
  ShimBrd_setGyroInUse(GYRO_ICM20948_IN_USE);
  setBoard(HW_ID_SHIMMER3, SHIMMER3_IMU, 9, 2);
  expectFalse("SR31-9-2 is not on the list", ShimBrd_isSubstitutionNeededForWrAccel());
  setBoard(HW_ID_SHIMMER3, EXP_BRD_GSR_UNIFIED, 4, 2);
  expectTrue("SR48-4-2 is - the GSR board has two entries",
      ShimBrd_isSubstitutionNeededForWrAccel());

  /* The reversed GSR control pins are one single revision. */
  setBoard(HW_ID_SHIMMER3, EXP_BRD_GSR_UNIFIED, 4, 1);
  expectTrue("SR48-4-1 has reversed GSR control pins", ShimBrd_areGsrControlsPinsReversed());
  setBoard(HW_ID_SHIMMER3, EXP_BRD_GSR_UNIFIED, 4, 2);
  expectFalse("SR48-4-2 does not", ShimBrd_areGsrControlsPinsReversed());
  setBoard(HW_ID_SHIMMER3, EXP_BRD_GSR_UNIFIED, 5, 1);
  expectFalse("SR48-5-1 does not", ShimBrd_areGsrControlsPinsReversed());

  /* Second-generation parts are inferred from the fitted combination rather
   * than from the SR number, because some boards predate the EEPROM. */
  setBoard(HW_ID_SHIMMER3, SHIMMER3_IMU, 8, 0);
  ShimBrd_setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_LSM303AHTR_IN_USE);
  hostStub_setBmp280InUse(1);
  expectTrue("LSM303AHTR + BMP280 means second-generation IMU parts",
      ShimBrd_are2ndGenImuSensorsPresent());
  expectFalse("on a known board ID, that is not an unknown board",
      ShimBrd_are2ndGenSensorsPresentAndUnknownBoard());

  setBoard(HW_ID_SHIMMER3, EXP_BRD_PROTO3_MINI, 3, 0);
  expectTrue("a Proto3 Mini with the same parts is an unknown board",
      ShimBrd_are2ndGenSensorsPresentAndUnknownBoard());

  hostStub_setBmp280InUse(0);
  setBoard(HW_ID_SHIMMER3, SHIMMER3_IMU, 8, 0);
  expectFalse("a BMP180 means first-generation", ShimBrd_are2ndGenImuSensorsPresent());

  /* The low-noise accel is inferred the same way. */
  hostStub_setBmp280InUse(1);
  ShimBrd_setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_ICM20948_IN_USE);
  expectTrue("ICM-20948 + BMP280 implies a KXTC9-2050",
      ShimBrd_isLnAccelKxtc9_2050Present());
  ShimBrd_setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_LSM303DLHC_IN_USE);
  expectFalse("an LSM303DLHC does not", ShimBrd_isLnAccelKxtc9_2050Present());
}
#endif /* SHIMMER3 */

/* The human-readable name shown on the dock and over Bluetooth. An unknown ID
 * has to produce something a support engineer can act on, not an empty string. */
static void test_daughter_card_name(void)
{
  testCase("test_daughter_card_name");
  hostStub_reset();

  setBoard(HW_ID_SHIMMER3R, SHIMMER3_IMU, 11, 0);
  ShimBrd_parseDaughterCardId();
  expectStr("SR31 names the IMU board", ShimBrd_getDaughtCardIdStrPtr(), "Shimmer3 IMU");

  setBoard(HW_ID_SHIMMER3R, EXP_BRD_GSR_UNIFIED, 8, 0);
  ShimBrd_parseDaughterCardId();
  expectStr("SR48 names the GSR board", ShimBrd_getDaughtCardIdStrPtr(), "Shimmer3 GSR+");

  setBoard(HW_ID_SHIMMER3R, EXP_BRD_EXG_UNIFIED, 8, 0);
  ShimBrd_parseDaughterCardId();
  expectStr("SR47 names the ExG board", ShimBrd_getDaughtCardIdStrPtr(), "Shimmer3 ExG");

  /* Unknown IDs fall back to "SR<n>", which is still enough to identify the
   * board from a support log. */
  setBoard(HW_ID_SHIMMER3R, 123, 1, 0);
  ShimBrd_parseDaughterCardId();
  expectStr("an unknown ID still reports its number",
      ShimBrd_getDaughtCardIdStrPtr(), "SR123");

  /* The name buffer is 26 bytes and the longest name is well inside it, but the
   * fallback is built with sprintf() from a uint8_t, so the widest possible
   * result is "SR255" - asserted so a wider ID field would fail here first. */
  setBoard(HW_ID_SHIMMER3R, 255, 0, 0);
  ShimBrd_parseDaughterCardId();
  expectStr("the widest fallback name", ShimBrd_getDaughtCardIdStrPtr(), "SR255");
}

/* --dump prints every gate's answer over the revision space, for
 * crosscheck_board_revisions.py to put against the documented table. */
static void dumpGateMatrix(void)
{
  static const uint8_t boards[] = { SHIMMER3_IMU, EXP_BRD_PROTO3_DELUXE,
    EXP_BRD_EXG_UNIFIED, EXP_BRD_GSR_UNIFIED, EXP_BRD_BR_AMP_UNIFIED };
  unsigned b;
  uint8_t major;
  uint8_t minor;

  printf(
      "# srId,major,minor,bmp581,lis3mdl,adxl371,ads7028,i2c4,boot0default\n");
  for (b = 0; b < sizeof(boards) / sizeof(boards[0]); b++)
  {
    for (major = 1; major <= 12; major++)
    {
      for (minor = 0; minor <= 4; minor++)
      {
        setBoard(HW_ID_SHIMMER3R, boards[b], major, minor);
        printf("%u,%u,%u,%u,%u,%u,%u,%u,%u\n", boards[b], major, minor,
            ShimBrd_isBmp581PresentPerSrNumber() ? 1 : 0,
            ShimBrd_isLis3mdlPresent() ? 1 : 0, ShimBrd_isAdxl371Present() ? 1 : 0,
            isAds7028Present() ? 1 : 0, ShimBrd_isI2c4Supported() ? 1 : 0,
            ShimBrd_checkCorrectStateForBoot0() ? 1 : 0);
      }
    }
  }
}

int main(int argc, char **argv)
{
  if (argc > 1 && strcmp(argv[1], "--dump") == 0)
  {
    hostStub_reset();
    dumpGateMatrix();
    return 0;
  }

  printf("shimmer_boards host tests (" PLATFORM_NAME ")\n\n");
  hostTestSilenceUnused();

  test_daughter_card_id_set();
  test_sr_number_comparisons();
  test_bmp581_gate();
  test_lis3mdl_and_adxl371_gates();
  test_sr48_6_0_special_cases();
  test_ads1292_gates();
#if defined(SHIMMER3)
  test_shimmer3_only_gates();
#endif
  test_daughter_card_name();

  return hostTestReport("shimmer_boards (" PLATFORM_NAME ")");
}

#endif /* SHIMMER_HOST_TEST */
