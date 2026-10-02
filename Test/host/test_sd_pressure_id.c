/*
 * Host-side test for SDCard/shimmer_sd_pressure_id.c (DEV-1123).
 *
 * The byte this encodes is the SD header's SDH_PRESSURE_SENSOR_ID. It exists so
 * that host parsers (Java, C#, Python, the web SDK) can name the pressure
 * sensor a recording came from instead of inferring it from the board's SR
 * number; parsers adopt it in their own releases, and until they do they ignore
 * it. A recording outlives the firmware that wrote it, so the values pinned
 * here are a file-format contract, not an implementation detail - change one
 * and every parser that reads the byte has to change with it, and SD cards
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

/* Second argument of ShimSdHead_encodePressureSensorId(). */
enum
{
  DETECTED = 0,
  INFERRED = 1
};

static void test_detected_on_hardware(void)
{
  testCase("a sensor detected on the hardware is stored as its bare ID");
  expectX("BMP180", ShimSdHead_encodePressureSensorId(BMP180, DETECTED), 0x00);
  expectX("BMP280", ShimSdHead_encodePressureSensorId(BMP280, DETECTED), 0x01);
  expectX("BMP390", ShimSdHead_encodePressureSensorId(BMP390, DETECTED), 0x02);
  expectX("BMP581", ShimSdHead_encodePressureSensorId(BMP581, DETECTED), 0x03);
}

static void test_inferred_from_sr_number(void)
{
  /* Shimmer3R's PressureSensor_detect() falls back to the SR number when
   * neither chip answers its ID check or both do - the damaged-board case this
   * flag is for. Shimmer3 never infers. */
  testCase("a sensor inferred from the SR number sets bit 7");
  expectX("BMP390 inferred", ShimSdHead_encodePressureSensorId(BMP390, INFERRED), 0x82);
  expectX("BMP581 inferred", ShimSdHead_encodePressureSensorId(BMP581, INFERRED), 0x83);
  expectX("BMP180 inferred", ShimSdHead_encodePressureSensorId(BMP180, INFERRED), 0x80);
  /* Any non-zero value counts, as the firmware passes a flag. */
  expectX("inferred flag is boolean", ShimSdHead_encodePressureSensorId(BMP581, 7), 0x83);
}

static void test_none_fitted(void)
{
  testCase("no sensor fitted is 0xFE whether or not it was inferred");
  expectX("none, detected",
      ShimSdHead_encodePressureSensorId(SDH_PRESSURE_SENSOR_NONE, DETECTED), 0xFE);
  expectX("none, inferred",
      ShimSdHead_encodePressureSensorId(SDH_PRESSURE_SENSOR_NONE, INFERRED), 0xFE);
}

static void test_future_sensor_ids(void)
{
  testCase("IDs up to 0x7D are encodable for future sensors");
  expectX("next free ID", ShimSdHead_encodePressureSensorId(4, DETECTED), 0x04);
  expectX("highest ID", ShimSdHead_encodePressureSensorId(0x7D, DETECTED), 0x7D);
  expectX("highest ID inferred", ShimSdHead_encodePressureSensorId(0x7D, INFERRED), 0xFD);
}

static void test_out_of_range_ids(void)
{
  /* 0x7E and 0x7F would collide with 0xFE/0xFF once bit 7 is set, and 0x80+
   * does not fit in 7 bits. None is allocatable, so the firmware writes
   * "not recorded" and every parser falls back to its own inference. */
  testCase("an ID no registry entry can back is written as not recorded");
  expectX("0x7E, detected", ShimSdHead_encodePressureSensorId(0x7E, DETECTED), 0xFF);
  expectX("0x7E, inferred", ShimSdHead_encodePressureSensorId(0x7E, INFERRED), 0xFF);
  expectX("0x7F, inferred", ShimSdHead_encodePressureSensorId(0x7F, INFERRED), 0xFF);
  expectX("0x80", ShimSdHead_encodePressureSensorId(0x80, DETECTED), 0xFF);
  expectX("0xFD", ShimSdHead_encodePressureSensorId(0xFD, INFERRED), 0xFF);
  expectX("0xFF", ShimSdHead_encodePressureSensorId(0xFF, DETECTED), 0xFF);
}

static void test_constants(void)
{
  testCase("the sentinel values are the documented ones");
  expectX("inferred bit", SDH_PRESSURE_SENSOR_INFERRED, 0x80);
  expectX("none", SDH_PRESSURE_SENSOR_NONE, 0xFE);
  /* Must equal the pre-fill of ShimSdHead_config2SdHead(): that is what makes
   * every header from firmware before this field read as "not recorded". */
  expectX("not recorded", SDH_PRESSURE_SENSOR_UNRECORDED, 0xFF);
  expectX("highest ID", SDH_PRESSURE_SENSOR_ID_MAX, 0x7D);
}

int main(void)
{
  printf("shimmer_sd_pressure_id host tests\n\n");
  hostTestSilenceUnused();

  test_detected_on_hardware();
  test_inferred_from_sr_number();
  test_none_fitted();
  test_future_sensor_ids();
  test_out_of_range_ids();
  test_constants();

  return hostTestReport("shimmer_sd_pressure_id");
}

#endif /* SHIMMER_HOST_TEST */
