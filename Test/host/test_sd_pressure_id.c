/*
 * Host-side test for SDCard/shimmer_sd_pressure_id.c (DEV-1123).
 *
 * The byte this encodes is the SD header's SDH_PRESSURE_SENSOR_ID: every host
 * parser (Java, C#, Python, the web SDK) reads it to name the pressure sensor a
 * recording came from, and a recording outlives the firmware that wrote it. So
 * the values pinned here are a file-format contract, not an implementation
 * detail - change one and every parser has to change with it, and SD cards
 * already in the field still carry the old meaning.
 *
 * shimmer_sd_pressure_id.c includes nothing but <stdint.h>, so this needs no
 * stubs. Run by .github/workflows/host-tests.yml.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include "host_test.h"

#include "SDCard/shimmer_sd_pressure_id.h"

/* The firmware's registry (Comms/shimmer_bt_uart.h) cannot be included here
 * without the platform headers it pulls in, so its values are restated. The
 * file format depends on them never moving. */
enum
{
  BMP180 = 0,
  BMP280 = 1,
  BMP390 = 2,
  BMP581 = 3
};

static void test_identified_by_chip_id(void)
{
  testCase("a sensor confirmed by its chip ID is stored as its bare ID");
  expectX("BMP180", ShimSdHead_encodePressureSensorId(BMP180, 1), 0x00);
  expectX("BMP280", ShimSdHead_encodePressureSensorId(BMP280, 1), 0x01);
  expectX("BMP390", ShimSdHead_encodePressureSensorId(BMP390, 1), 0x02);
  expectX("BMP581", ShimSdHead_encodePressureSensorId(BMP581, 1), 0x03);
  /* Any non-zero value counts as confirmed, as the firmware passes a flag. */
  expectX("identified flag is boolean", ShimSdHead_encodePressureSensorId(BMP581, 7), 0x03);
}

static void test_inferred_from_sr_number(void)
{
  /* Shimmer3R's PressureSensor_detect() falls back to the SR number when
   * neither chip answers or both do - the damaged-board case this flag is for. */
  testCase("a sensor inferred from the SR number sets bit 7");
  expectX("BMP390 inferred", ShimSdHead_encodePressureSensorId(BMP390, 0), 0x82);
  expectX("BMP581 inferred", ShimSdHead_encodePressureSensorId(BMP581, 0), 0x83);
  expectX("BMP180 inferred", ShimSdHead_encodePressureSensorId(BMP180, 0), 0x80);
}

static void test_none_fitted(void)
{
  testCase("no sensor fitted is 0xFE whether or not it was 'identified'");
  expectX("none, identified",
      ShimSdHead_encodePressureSensorId(SDH_PRESSURE_SENSOR_NONE, 1), 0xFE);
  expectX("none, inferred",
      ShimSdHead_encodePressureSensorId(SDH_PRESSURE_SENSOR_NONE, 0), 0xFE);
}

static void test_future_sensor_ids(void)
{
  testCase("IDs up to 0x7D are encodable for future sensors");
  expectX("next free ID", ShimSdHead_encodePressureSensorId(4, 1), 0x04);
  expectX("highest ID", ShimSdHead_encodePressureSensorId(0x7D, 1), 0x7D);
  expectX("highest ID inferred", ShimSdHead_encodePressureSensorId(0x7D, 0), 0xFD);
}

static void test_out_of_range_ids(void)
{
  /* 0x7E and 0x7F would collide with 0xFE/0xFF once bit 7 is set, and 0x80+
   * does not fit in 7 bits. None is allocatable, so the firmware writes
   * "not recorded" and every parser falls back to its own inference. */
  testCase("an ID no registry entry can back is written as not recorded");
  expectX("0x7E, identified", ShimSdHead_encodePressureSensorId(0x7E, 1), 0xFF);
  expectX("0x7E, inferred", ShimSdHead_encodePressureSensorId(0x7E, 0), 0xFF);
  expectX("0x7F, inferred", ShimSdHead_encodePressureSensorId(0x7F, 0), 0xFF);
  expectX("0x80", ShimSdHead_encodePressureSensorId(0x80, 1), 0xFF);
  expectX("0xFD", ShimSdHead_encodePressureSensorId(0xFD, 0), 0xFF);
  expectX("0xFF", ShimSdHead_encodePressureSensorId(0xFF, 1), 0xFF);
}

static void test_constants(void)
{
  testCase("the sentinel values are the documented ones");
  expectX("inferred bit", SDH_PRESSURE_SENSOR_INFERRED, 0x80);
  expectX("none", SDH_PRESSURE_SENSOR_NONE, 0xFE);
  /* Must equal the pre-fill of ShimSdHead_config2SdHead(): that is what makes
   * every pre-DEV-1123 header read as "not recorded" with no version check. */
  expectX("not recorded", SDH_PRESSURE_SENSOR_UNRECORDED, 0xFF);
  expectX("highest ID", SDH_PRESSURE_SENSOR_ID_MAX, 0x7D);
}

int main(void)
{
  printf("shimmer_sd_pressure_id host tests\n\n");
  hostTestSilenceUnused();

  test_identified_by_chip_id();
  test_inferred_from_sr_number();
  test_none_fitted();
  test_future_sensor_ids();
  test_out_of_range_ids();
  test_constants();

  return hostTestReport("shimmer_sd_pressure_id");
}

#endif /* SHIMMER_HOST_TEST */
