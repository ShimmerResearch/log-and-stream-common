/*
 * shimmer_factory_record.h
 *
 * Byte layouts of the records a production test reads back from a Shimmer3R
 * over the dock (docs/SHIMMER3_DOCK_PROTOCOL.md) and over Bluetooth
 * (docs/SHIMMER3_BT_COMMUNICATION_PROTOCOL.md): the boot report, the user
 * button state, and the power-profile record and its readback
 * (docs/SHIMMER3R_FACTORY_TEST_REPORT.md section 6).
 *
 * Everything here is pure: fixed little-endian layouts built byte by byte, so
 * the same code gives the same bytes on the MSP430 and the STM32 and can be
 * pinned by a host test (Test/host/test_factory_record.c). The dock and
 * Bluetooth handlers gather the values; these functions only lay them out.
 */

#ifndef TEST_SHIMMER_FACTORY_RECORD_H_
#define TEST_SHIMMER_FACTORY_RECORD_H_

#include <stdint.h>

/* --- Boot report ------------------------------------------------------------
 *
 * How the last boot went, latched as it happened (LogAndStream_setBootStage):
 *
 *   0  1  layout version (BOOT_REPORT_LAYOUT_VER)
 *   1 12  MCU unique ID, as GET_UNIQUE_SERIAL_COMMAND returns it
 *  13  1  flags (BOOT_REPORT_FLAG_*)
 *  14  2  I2C stage, ms
 *  16  2  Bluetooth stage, ms (including any failure stage)
 *  18  2  configuration stage, ms
 *  20  1  Bluetooth baud-rate attempts (0 = the first baud rate answered)
 *  21  1  LSE drive level (RCC LSEDRV bits; 0xFF = LSE not running)
 *
 * Stage times saturate at 65535 ms. */
#define BOOT_REPORT_LAYOUT_VER        1U
#define BOOT_REPORT_UID_LEN           12U
#define BOOT_REPORT_LEN               22U

#define BOOT_REPORT_FLAG_I2C_SLOW     0x01U /* I2C stage over 1 s */
#define BOOT_REPORT_FLAG_BT_FAILED    0x02U /* Bluetooth init gave up */
#define BOOT_REPORT_FLAG_CONFIG_SLOW  0x04U /* configuration stage over 2 s */
#define BOOT_REPORT_FLAG_LSE_FALLBACK 0x08U /* RTC running from the LSI */

#define BOOT_REPORT_LSE_DRIVE_NONE    0xFFU

typedef struct
{
  uint8_t uid[BOOT_REPORT_UID_LEN];
  uint8_t flags;
  uint16_t i2cMs;
  uint16_t btMs;
  uint16_t configMs;
  uint8_t btBaudAttempts;
  uint8_t lseDrive;
} boot_report_t;

/* Writes BOOT_REPORT_LEN bytes to out and returns that length. */
uint8_t ShimFactoryRecord_encodeBootReport(const boot_report_t *report, uint8_t *out);

/* Milliseconds as a u16 that stops at 65535 rather than wrapping. */
uint16_t ShimFactoryRecord_saturateMs(uint32_t ms);

/* --- User button state --------------------------------------------------------
 *
 *   0  1  level now (1 = pressed)
 *   1  2  presses since boot (wraps at 65536)
 *   3  2  length of the last completed press, ms (saturates at 65535)
 */
#define BUTTON_STATE_LEN 5U

uint8_t ShimFactoryRecord_encodeButtonState(uint8_t level,
    uint16_t pressCount,
    uint16_t lastPressMs,
    uint8_t *out);

/* --- Power-profile record -----------------------------------------------------
 *
 * A Shimmer3R can be armed, over the dock, to run a fixed power-profile
 * sequence on its next boot, so the current it draws can be captured with no
 * cable attached. The arm and the outcome live in one 16-byte EEPROM page
 * (docs/SHIMMER3_EEPROM_MEMORY_MAP.md):
 *
 *   0  2  magic (PP_RECORD_MAGIC)        2  1  layout version
 *   3  1  state (PP_STATE_*)             4  1  run_seq
 *   5  2  nonce                          7  4  step_ok
 *  11  2  parts_found                   13  1  schedule version
 *  14  2  CRC-16 over bytes 0-13 (ShimSwCrc_calc, low byte first)
 *
 * run_seq is the index of the step the run is on: written before each step,
 * so a run cut short shows where. It equals the step count once DONE.
 * step_ok has bit i set when step i completed. parts_found uses PP_PART_*. */
#define PP_RECORD_LEN        16U
#define PP_RECORD_MAGIC      0x5050U /* "PP" */
#define PP_RECORD_LAYOUT_VER 1U

typedef enum
{
  PP_STATE_DISARMED = 0, /* never armed since written, or disarmed */
  PP_STATE_ARMED = 1,    /* runs at the next boot, if undocked and no USB */
  PP_STATE_RUNNING = 2,  /* a run started; never re-run */
  PP_STATE_DONE = 3,     /* every step ran */
  PP_STATE_SKIPPED = 4,  /* the next boot was docked or on USB */
  PP_STATE_ABORTED = 5,  /* a boot found it RUNNING, or the run was stopped */
  /* Not stored: the page holds no valid record */
  PP_STATE_NONE = 0xFF,
} pp_state_t;

typedef struct
{
  uint8_t state;
  uint8_t runSeq;
  uint16_t nonce;
  uint32_t stepOk;
  uint16_t partsFound;
  uint8_t scheduleVer;
} pp_record_t;

/* Writes PP_RECORD_LEN bytes, CRC included */
void ShimFactoryRecord_encodePpRecord(const pp_record_t *record, uint8_t *out);
/* Returns 1 and fills record if the page holds a valid record (magic, layout
 * version and CRC), 0 otherwise */
uint8_t ShimFactoryRecord_decodePpRecord(const uint8_t *in, pp_record_t *record);

/* --- Power-profile schedule and readback ----------------------------------------
 *
 * The readback (dock GET COMP_TEST/0x04), at most 128 bytes:
 *
 *   0  1  layout version (PP_READBACK_LAYOUT_VER)
 *   1  1  state (PP_STATE_*, PP_STATE_NONE if no record)
 *   2  1  run_seq            3  2  nonce             5  4  step_ok
 *   9  2  parts_found       11  2  parts expected for the board
 *  13  2  return-to-baseline window after each step, ms
 *  15  1  marker id (PP_MARKER_*)
 *  16  1  n, the number of steps
 *  17 3n  per step: step id (PP_STEP_*) u8, duration ms u16
 *
 * With no record, every field from run_seq to parts_found is 0. */
#define PP_READBACK_LAYOUT_VER 1U
#define PP_READBACK_HEADER_LEN 17U
#define PP_READBACK_MAX_LEN    128U
#define PP_MAX_STEPS \
  ((PP_READBACK_MAX_LEN - PP_READBACK_HEADER_LEN) / 3U) /* 37 */

/* Marker 1: 500 ms with every LED channel full on, then the 16-bit nonce MSB
 * first, one 100 ms slot per bit, the LEDs on for the first 30 ms of a slot
 * for a 0 and 70 ms for a 1. Each later step starts with a 20 ms pulse. */
#define PP_MARKER_NONCE16     1U
#define PP_MARKER_SYNC_MS     500U
#define PP_MARKER_SLOT_MS     100U
#define PP_MARKER_BIT0_MS     30U
#define PP_MARKER_BIT1_MS     70U
#define PP_MARKER_PULSE_MS    20U
#define PP_MARKER_DURATION_MS (PP_MARKER_SYNC_MS + 16U * PP_MARKER_SLOT_MS)

/* Step ids. Values are protocol: never renumber, only add. */
typedef enum
{
  PP_STEP_MARKER = 1,
  PP_STEP_BASELINE = 2,     /* Bluetooth, SD and LEDs off */
  PP_STEP_SENSING_RAIL = 3, /* sensing rail on, every part idle */
  PP_STEP_LSM6DSV = 4,      /* each sensor at its highest current */
  PP_STEP_LIS3MDL = 5,
  PP_STEP_LIS2DW12 = 6,
  PP_STEP_LIS2MDL = 7,
  PP_STEP_ADXL371 = 8,
  PP_STEP_BMP390 = 9,
  PP_STEP_BMP581 = 10,
  PP_STEP_ADS1292R = 11,
  PP_STEP_ADS7028_GSR = 12,
  PP_STEP_EXP_POWER = 13, /* expansion-board power switch */
  PP_STEP_MICROPHONE = 14,
  PP_STEP_SD = 15,          /* SD power, init and mount */
  PP_STEP_LED_LWR_RED = 16, /* each LED channel alone, full on */
  PP_STEP_LED_LWR_GREEN = 17,
  PP_STEP_LED_LWR_BLUE = 18,
  PP_STEP_LED_UPR_RED = 19,
  PP_STEP_LED_UPR_GREEN = 20,
  PP_STEP_LED_UPR_BLUE = 21,
  PP_STEP_BLUETOOTH = 22, /* Bluetooth started as after boot */
} pp_step_id_t;

/* Parts the run probes for (parts_found) or the board should carry (parts
 * expected). The LSM6DSV, LIS2DW12 and LIS2MDL are on every Shimmer3R and
 * are not probed. */
#define PP_PART_LIS3MDL  0x0001U
#define PP_PART_ADXL371  0x0002U
#define PP_PART_ADS7028  0x0004U
#define PP_PART_BMP390   0x0008U
#define PP_PART_BMP581   0x0010U
#define PP_PART_ADS1292R 0x0020U

typedef struct
{
  uint8_t stepId;
  uint16_t durationMs;
} pp_step_t;

typedef struct
{
  uint8_t scheduleVer;
  uint8_t markerId;
  uint16_t baselineWindowMs;
  uint16_t partsExpected;
  uint8_t n;
  pp_step_t steps[PP_MAX_STEPS];
} pp_schedule_t;

/* Writes the readback for record (NULL if the page holds no valid record) and
 * schedule, and returns its length: 17 + 3n, n clamped to PP_MAX_STEPS */
uint8_t ShimFactoryRecord_encodePpReadback(const pp_record_t *record,
    const pp_schedule_t *schedule,
    uint8_t *out);

/* Appends a step; returns 0 if the schedule is full */
uint8_t ShimFactoryRecord_addPpStep(pp_schedule_t *schedule, uint8_t stepId, uint16_t durationMs);

/* Total time of a run in ms: every step, its window, and a pulse before every
 * step after the marker. Excludes the EEPROM writes at step boundaries. */
uint32_t ShimFactoryRecord_ppRunMs(const pp_schedule_t *schedule);

#endif /* TEST_SHIMMER_FACTORY_RECORD_H_ */
