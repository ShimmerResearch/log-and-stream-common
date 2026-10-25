/*
 * shimmer_factory_record.h
 *
 * Byte layouts of the records a production test reads back from a Shimmer3R
 * over the dock (docs/SHIMMER3_DOCK_PROTOCOL.md) and over Bluetooth
 * (docs/SHIMMER3_BT_COMMUNICATION_PROTOCOL.md): the boot report and the user
 * button state.
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

#endif /* TEST_SHIMMER_FACTORY_RECORD_H_ */
