/*
 * shimmer_test.h
 *
 *  Created on: Aug 14, 2025
 *      Author: MarkNolan
 */

#ifndef TEST_SHIMMER_TEST_H_
#define TEST_SHIMMER_TEST_H_

#include <stdint.h>

typedef enum
{
  PRINT_TO_DEBUGGER = 0,
  PRINT_TO_DOCK_UART,
  PRINT_TO_BT_UART
} factory_test_target_t;

/* The dock's COMP_TEST property and the 0xA8 Bluetooth argument. Values are
 * protocol (hosts restate them; Test/host/crosscheck_host_constants.py checks
 * the in-repo one), so each is spelled out: never renumber, only add.
 *
 * FACTORY_TEST_POWER_PROFILE is not a report suite: it arms the power-profile
 * record (Test/shimmer_factory_record.h), over the dock only and on Shimmer3R
 * only. Everything else answers it as it answers a suite past the last. */
typedef enum
{
  FACTORY_TEST_MAIN = 0,
  FACTORY_TEST_LEDS = 1,
  FACTORY_TEST_ICS = 2,
  FACTORY_TEST_LED_STATES = 3,
  FACTORY_TEST_POWER_PROFILE = 4,
  FACTORY_TEST_COUNT = 5
} factory_test_t;

#define MAX_TEST_REPORT_LENGTH          128
#define SELF_TEST_STR_PASS              "PASS"
#define SELF_TEST_STR_FAIL              "FAIL"
#define SELF_TEST_STR_EMPTY             ""
#define SELF_TEST_STR_CHIP_DETECTION    " - Chip not detected"
#define SELF_TEST_STR_SIGNAL_ISSUE      " - Signal issue"
#define SELF_TEST_STR_TEMPERATURE_ISSUE " - Temperature issue"
#define SELF_TEST_STR_DRDY_ISSUE        " - DRDY/INT issue"
#define SELF_TEST_STR_UNKNOWN           " - Unknown"

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

void ShimFactoryTest_setup(factory_test_target_t target, factory_test_t testToRun);

uint32_t ShimFactoryTest_run(void);

void ShimFactoryTest_sendReport(const char *str);
void ShimFactoryTest_sendReportImpl(const char *str, factory_test_target_t factoryTestTarget);
factory_test_t ShimFactoryTest_getTestToRun(void);
factory_test_target_t ShimFactoryTest_getTarget(void);

#endif /* TEST_SHIMMER_TEST_H_ */
