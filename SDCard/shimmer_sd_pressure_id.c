/*
 * shimmer_sd_pressure_id.c
 *
 * See shimmer_sd_pressure_id.h for the byte layout (DEV-1123).
 */

#include <stdint.h>

#include "shimmer_sd_pressure_id.h"

uint8_t ShimSdHead_encodePressureSensorId(uint8_t sensorId, uint8_t inferredFromSrNumber)
{
  if (sensorId == SDH_PRESSURE_SENSOR_NONE)
  {
    return SDH_PRESSURE_SENSOR_NONE;
  }
  if (sensorId > SDH_PRESSURE_SENSOR_ID_MAX)
  {
    return SDH_PRESSURE_SENSOR_UNRECORDED;
  }
  return inferredFromSrNumber ? (uint8_t) (sensorId | SDH_PRESSURE_SENSOR_INFERRED) : sensorId;
}
