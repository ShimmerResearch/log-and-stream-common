/*
 * Host-test stub. log_and_stream_externs.h includes this on both platforms.
 * The factory-test harness itself is firmware-side and is not under test here.
 */
#ifndef HOST_TEST_STUB_HAL_FACTORYTEST_H
#define HOST_TEST_STUB_HAL_FACTORYTEST_H

#include <stdint.h>

typedef enum
{
  FACTORY_TEST_ALL = 0,
  FACTORY_TEST_ALL_BUT_SD_AND_BT
} factory_test_t;

void run_factory_test(factory_test_t factoryTest);

#endif /* HOST_TEST_STUB_HAL_FACTORYTEST_H */
