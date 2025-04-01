#ifndef SHIMMER_CALIBRATION_H
#define SHIMMER_CALIBRATION_H

#include <stdint.h>

#if defined(SHIMMER3R)
#include "shimmer_definitions.h"
#include <shimmer_include.h>
#endif

#ifndef min
#define min(a, b) a > b ? b : a
#endif

#define SHIMMER_CALIB_DATA_MAX  22
#define SHIMMER_CALIB_COPY_SIZE 128
#if defined(SHIMMER3)
#define SHIMMER_CALIB_RAM_MAX 1024
#elif defined(SHIMMER3R)
#define SHIMMER_CALIB_RAM_MAX INFOMEM_CALIB_SIZE
#endif

typedef struct shimmer_calib_default_t
{
  uint16_t bias_x;
  uint16_t bias_y;
  uint16_t bias_z;
  uint16_t sens_x;
  uint16_t sens_y;
  uint16_t sens_z;
  int8_t align_xx;
  int8_t align_xy;
  int8_t align_xz;
  int8_t align_yx;
  int8_t align_yy;
  int8_t align_yz;
  int8_t align_zx;
  int8_t align_zy;
  int8_t align_zz;
} sc_default_t;

typedef union shimmer_calib_data_u
{
  uint8_t raw[SHIMMER_CALIB_DATA_MAX];

  struct
  {
    uint16_t bias_x;
    uint16_t bias_y;
    uint16_t bias_z;
    uint16_t sens_x;
    uint16_t sens_y;
    uint16_t sens_z;
    int8_t align_xx;
    int8_t align_xy;
    int8_t align_xz;
    int8_t align_yx;
    int8_t align_yy;
    int8_t align_yz;
    int8_t align_zx;
    int8_t align_zy;
    int8_t align_zz;
  } dd; //default data structure: 21byte as declared above in sc_default_t

  uint8_t fillerByte;
} sc_data_u;

typedef struct shimmer_calib_t
{
  uint16_t id;
  uint8_t range;
  uint8_t data_len;
  uint8_t ts[8];  //timestamp
  sc_data_u data; //[SHIMMER_CALIB_DATA_MAX];
} sc_t;

//SC_ short for SHIMMER_CALIBRATION_
#define SC_OFFSET_LENGTH_L         0
#define SC_OFFSET_LENGTH_H         1
#define SC_OFFSET_VER_HW_ID_L      2
#define SC_OFFSET_VER_HW_ID_H      3
#define SC_OFFSET_VER_FW_ID_L      4
#define SC_OFFSET_VER_FW_ID_H      5
#define SC_OFFSET_VER_FW_MAJOR_L   6
#define SC_OFFSET_VER_FW_MAJOR_H   7
#define SC_OFFSET_VER_FW_MINOR_L   8
#define SC_OFFSET_VER_FW_INTER_L   9
#define SC_OFFSET_FIRST_SENSOR     10

#define SC_OFFSET_SENSOR_ID_L      0
#define SC_OFFSET_SENSOR_ID_H      1
#define SC_OFFSET_SENSOR_RANGE     2
#define SC_OFFSET_SENSOR_LENGTH    3
#define SC_OFFSET_SENSOR_TIMESTAMP 4
#define SC_OFFSET_SENSOR_DATA      12

#define SC_TIMESTAMP_LENGTH        8

#if defined(SHIMMER3)
#define SC_SENSOR_ANALOG_ACCEL           2
#define SC_SENSOR_MPU9X50_ICM20948_GYRO  30
#define SC_SENSOR_LSM303_ACCEL           31
#define SC_SENSOR_LSM303_MAG             32
#define SC_SENSOR_MPU9X50_ICM20948_ACCEL 33
#define SC_SENSOR_MPU9X50_ICM20948_MAG   34
#define SC_SENSOR_BMP180_PRESSURE        36
#elif defined(SHIMMER3R)
#define SC_SENSOR_LSM6DSV_ACCEL   37
#define SC_SENSOR_LSM6DSV_GYRO    38
#define SC_SENSOR_LIS2DW12_ACCEL  39
#define SC_SENSOR_ADXL371_ACCEL   40
#define SC_SENSOR_LIS3MDL_MAG     41
#define SC_SENSOR_LIS2MDL_MAG     42
#define SC_SENSOR_BMP390_PRESSURE 43
#endif
#define SC_SENSOR_HOST_ECG        100
#define SC_SENSOR_ALL             0xFF

#define SC_DATA_LEN_STD_IMU_CALIB 21

#if defined(SHIMMER3)
//Analogue Accel Range
#define SC_SENSOR_RANGE_ANALOG_ACCEL                  0
#define SC_SENSOR_RANGE_MAX_ANALOG_ACCEL              1
#define SC_DATA_LEN_ANALOG_ACCEL                      SC_DATA_LEN_STD_IMU_CALIB
//MPU9x50 Gyro range
#define SC_SENSOR_RANGE_MPU9X50_ICM20948_GYRO_250DPS  0x00 //+/-250 dps
#define SC_SENSOR_RANGE_MPU9X50_ICM20948_GYRO_500DPS  0x01 //+/-500 dps
#define SC_SENSOR_RANGE_MPU9X50_ICM20948_GYRO_1000DPS 0x02 //+/-1000 dps
#define SC_SENSOR_RANGE_MPU9X50_ICM20948_GYRO_2000DPS 0x03 //+/-2000 dps
#define SC_SENSOR_RANGE_MAX_MPU9X50_ICM20948_GYRO     4
#define SC_DATA_LEN_MPU9X50_ICM20948_GYRO             SC_DATA_LEN_STD_IMU_CALIB
//LSM303DLHC Accel Range
#define SC_SENSOR_RANGE_LSM303_ACCEL_2G               0x00
#define SC_SENSOR_RANGE_LSM303_ACCEL_4G               0x01
#define SC_SENSOR_RANGE_LSM303_ACCEL_8G               0x02
#define SC_SENSOR_RANGE_LSM303_ACCEL_16G              0x03
#define SC_SENSOR_RANGE_MAX_LSM303_ACCEL              4
#define SC_DATA_LEN_LSM303_ACCEL                      SC_DATA_LEN_STD_IMU_CALIB
//LSM303DLHC Mag gain
#define SC_SENSOR_RANGE_LSM303_MAG_1_3G               0x01 //+/-1.3 Gauss
#define SC_SENSOR_RANGE_LSM303_MAG_1_9G               0x02 //+/-1.9 Gauss
#define SC_SENSOR_RANGE_LSM303_MAG_2_5G               0x03 //+/-2.5 Gauss
#define SC_SENSOR_RANGE_LSM303_MAG_4_0G               0x04 //+/-4.0 Gauss
#define SC_SENSOR_RANGE_LSM303_MAG_4_7G               0x05 //+/-4.7 Gauss
#define SC_SENSOR_RANGE_LSM303_MAG_5_6G               0x06 //+/-5.6 Gauss
#define SC_SENSOR_RANGE_LSM303_MAG_8_1G               0x07 //+/-8.1 Gauss
#define SC_SENSOR_RANGE_MAX_LSM303_MAG                7
#define SC_DATA_LEN_LSM303_MAG                        SC_DATA_LEN_STD_IMU_CALIB
//BMP180 Temperature/Pressure Range
#define SC_SENSOR_RANGE_BMP180                        0
#define SC_SENSOR_RANGE_MAX_BMP180                    1
#define SC_DATA_LEN_BMP180                            22
#else
#define SC_SENSOR_RANGE_LIS2MDL_RANGE 0
#define SC_SENSOR_RANGE_ADXL371_RANGE 0
#endif

void ShimCalib_init(void);

uint8_t *ShimCalib_getRam(void);

void ShimCalib_ram2File(void);

uint8_t ShimCalib_file2Ram(void);

//return 0: success, 1: fail
uint8_t ShimCalib_singleSensorWrite(const sc_t *sc1);

//return 0: success, 1: fail
uint8_t ShimCalib_singleSensorRead(sc_t *sc1);

void ShimCalib_checkRamLen(void);
void ShimCalib_ramTempInit(void);

uint8_t ShimCalib_ramWrite(const uint8_t *buf, uint8_t length, uint16_t offset);
uint8_t ShimCalib_ramRead(uint8_t *buf, uint8_t length, uint16_t offset);

void ShimCalib_defaultAll(void);
void ShimCalib_setDefaultForSensor(uint8_t sensor);
#if defined(SHIMMER3)
void ShimCalib_setDefaultKionixCalib(sc_t *sc1Ptr);
void ShimCalib_setDefaultMpu9X50Icm20948GyroCalib(sc_t *sc1Ptr);
void ShimCalib_setDefaultLsm303AccelCalib(sc_t *sc1Ptr);
void ShimCalib_setDefaultLsm303MagCalib(sc_t *sc1Ptr);
#elif defined(SHIMMER3R)
void ShimCalib_setDefaultLsm6dsvAccelCalib(sc_t *sc1Ptr);
void ShimCalib_setDefaultLsm6dsvGyroCalib(sc_t *sc1Ptr);
void ShimCalib_setDefaultAdxl371AccelCalib(sc_t *sc1Ptr);
void ShimCalib_setDefaultLis2dw12AccelCalib(sc_t *sc1Ptr);
void ShimCalib_setDefaultLis2mdlMagCalib(sc_t *sc1Ptr);
void ShimCalib_setDefaultLis3mdlMagCalib(sc_t *sc1Ptr);
#endif

void ShimCalib_initFromConfigBytesAll(void);
void ShimCalib_updateFromConfigBytesAll(void);
void ShimCalib_configBytes0To127ToCalibDumpBytes(uint8_t setCalibTsZero);
void ShimCalib_configBytes128To255ToCalibDumpBytes(uint8_t setCalibTsZero);
void ShimCalib_configBytesToCalibDump(uint8_t id, uint8_t setCalibTsZero);
void ShimCalib_singleSensorToCalibDump(uint16_t id,
    uint8_t range,
    uint8_t data_len,
    uint8_t *configBytePtr,
    uint8_t setCalibTsZero);

void ShimCalib_calibDumpToConfigBytesAndSdHeaderAll(void);
void ShimCalib_calibDumpToConfigBytesAndSdHeaderSingleSensor(uint8_t sensor);

#endif //SHIMMER_CALIBRATION_H
