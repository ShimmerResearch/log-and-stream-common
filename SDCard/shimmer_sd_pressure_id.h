/*
 * shimmer_sd_pressure_id.h
 *
 * The pressure-sensor ID byte the SD header carries at SDH_PRESSURE_SENSOR_ID
 * (DEV-1123), so a parser can name the fitted sensor instead of inferring it
 * from the board's SR number. Kept free of any platform dependency so the
 * encoding can be host-tested (Test/host/test_sd_pressure_id.c).
 *
 * Bits 0-6 hold the sensor ID, drawn from the PRESSURE_SENSOR_* registry in
 * Comms/shimmer_bt_uart.h, which the 0xA7/0xA6 Bluetooth reply also uses.
 * Bit 7 is set when the firmware could not confirm the chip by its ID and fell
 * back to the SR number. See docs/SHIMMER3_SD_CARD_FORMAT.md.
 *
 *    0x00-0x7D  sensor ID, identified by chip ID (0x04 onwards: future sensors)
 *    0x80-0xFD  the same IDs, inferred from the SR number
 *    0xFE       no pressure sensor fitted
 *    0xFF       not recorded - every header from firmware before this field,
 *               because ShimSdHead_config2SdHead() pre-fills with 0xFF
 *
 * IDs 0x7E and 0x7F must never be allocated: with bit 7 set they would read as
 * "none" and "not recorded".
 */

#ifndef LOG_AND_STREAM_COMMON_SDCARD_SHIMMER_SD_PRESSURE_ID_H_
#define LOG_AND_STREAM_COMMON_SDCARD_SHIMMER_SD_PRESSURE_ID_H_

#include <stdint.h>

#define SDH_PRESSURE_SENSOR_ID_MAX     0x7DU
#define SDH_PRESSURE_SENSOR_INFERRED   0x80U
#define SDH_PRESSURE_SENSOR_NONE       0xFEU
#define SDH_PRESSURE_SENSOR_UNRECORDED 0xFFU

/* Returns the byte to write at SDH_PRESSURE_SENSOR_ID.
 *
 * sensorId is a PRESSURE_SENSOR_* value, or SDH_PRESSURE_SENSOR_NONE when no
 * sensor is fitted. identifiedByChipId is non-zero when the chip answered its
 * ID check unambiguously; zero when the firmware fell back to the SR number.
 *
 * An ID outside the allocatable range returns SDH_PRESSURE_SENSOR_UNRECORDED,
 * so a parser falls back to its own inference rather than trusting a value no
 * registry entry backs. */
uint8_t ShimSdHead_encodePressureSensorId(uint8_t sensorId, uint8_t identifiedByChipId);

#endif /* LOG_AND_STREAM_COMMON_SDCARD_SHIMMER_SD_PRESSURE_ID_H_ */
