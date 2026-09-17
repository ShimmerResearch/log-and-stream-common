/*
 * Host-test stub. log_and_stream_externs.h includes this under SHIMMER3.
 * Declaration only - nothing under test calls into the gyro driver.
 */
#ifndef HOST_TEST_STUB_MPU9150_H
#define HOST_TEST_STUB_MPU9150_H

#include <stdint.h>

void MPU9150_init(void);

#endif /* HOST_TEST_STUB_MPU9150_H */
