/*
 * shimmer_calibration.c
 *
 *  Created on: Jul 11, 2016
 *      Author: WeiboP
 *
 *  Edited on: Jul 26, 2017
 *      Author: Sam OM
 */

#include "shimmer_calibration.h"

#include <stdint.h>
#include <string.h>

#include <Boards/shimmer_boards.h>
#include <Configuration/shimmer_config.h>
#include <SDCard/shimmer_sd_data_file.h>
#include <SDCard/shimmer_sd_header.h>
#include <log_and_stream_definitions.h>
#include <log_and_stream_externs.h>

#if defined(SHIMMER3)
#include "msp430.h"

#include "../../shimmer_btsd.h"
#include "../5xx_HAL/hal_Board.h"
#include "../5xx_HAL/hal_RTC.h"
#include "../BMPX80/bmpX80.h"
#include "../LSM303AHTR/lsm303ahtr.h"
#include "../LSM303DLHC/lsm303dlhc.h"
#include "../RN4X/RN4X.h"
#elif defined(SHIMMER3R)
#include "hal_Infomem.h"

#include "stm32u5xx.h"
#endif

#if USE_FATFS
#include "ff.h"
#endif

void ShimCalib_initVer(void);
uint8_t ShimCalib_findLength(sc_t *sc1);

uint8_t shimmerCalib_ram[SHIMMER_CALIB_RAM_MAX], shimmerCalib_macId[5],
    shimmerCalib_ramTemp[SHIMMER_CALIB_RAM_MAX];
uint16_t shimmerCalib_ramLen, shimmerCalib_ramTempLen, shimmerCalib_ramTempMax;

void ShimCalib_init(void)
{
  shimmerCalib_ramLen = 8;

  memcpy(&shimmerCalib_macId[0], ShimBt_macIdStrPtrGet() + 8, 4);
  shimmerCalib_macId[4] = 0;

  memset(shimmerCalib_ram, 0, SHIMMER_CALIB_RAM_MAX);
  shimmerCalib_ram[SC_OFFSET_LENGTH_L] = shimmerCalib_ramLen & 0xff;
  shimmerCalib_ram[SC_OFFSET_LENGTH_H] = (shimmerCalib_ramLen >> 8) & 0xff;
  ShimCalib_initVer();

  ShimCalib_defaultAll();
}

void ShimCalib_initVer(void)
{
  shimmerCalib_ram[SC_OFFSET_VER_HW_ID_L] = DEVICE_VER & 0xff;
  shimmerCalib_ram[SC_OFFSET_VER_HW_ID_H] = (DEVICE_VER >> 8) & 0xff;
  shimmerCalib_ram[SC_OFFSET_VER_FW_ID_L] = FW_IDENTIFIER & 0xff;
  shimmerCalib_ram[SC_OFFSET_VER_FW_ID_H] = (FW_IDENTIFIER >> 8) & 0xff;
  shimmerCalib_ram[SC_OFFSET_VER_FW_MAJOR_L] = FW_VER_MAJOR & 0xff;
  shimmerCalib_ram[SC_OFFSET_VER_FW_MAJOR_H] = (FW_VER_MAJOR >> 8) & 0xff;
  shimmerCalib_ram[SC_OFFSET_VER_FW_MINOR_L] = FW_VER_MINOR & 0xff;
  shimmerCalib_ram[SC_OFFSET_VER_FW_INTER_L] = FW_VER_REL & 0xff;
}

uint8_t ShimCalib_findLength(sc_t *sc1)
{
  switch (sc1->id)
  {
#if defined(SHIMMER3)
    case SC_SENSOR_ANALOG_ACCEL:
      return SC_DATA_LEN_ANALOG_ACCEL;
    case SC_SENSOR_MPU9X50_ICM20948_GYRO:
      return SC_DATA_LEN_MPU9X50_ICM20948_GYRO;
    case SC_SENSOR_LSM303_ACCEL:
      return SC_DATA_LEN_LSM303_ACCEL;
    case SC_SENSOR_LSM303_MAG:
      return SC_DATA_LEN_LSM303_MAG;
    case SC_SENSOR_BMP180_PRESSURE:
      return SC_DATA_LEN_BMP180;
#elif defined(SHIMMER3R)
    case SC_SENSOR_LSM6DSV_ACCEL:
    case SC_SENSOR_LSM6DSV_GYRO:
    case SC_SENSOR_LIS2DW12_ACCEL:
    case SC_SENSOR_ADXL371_ACCEL:
    case SC_SENSOR_LIS3MDL_MAG:
    case SC_SENSOR_LIS2MDL_MAG:
      return SC_DATA_LEN_STD_IMU_CALIB;
#endif
    default:
      return 0;
  }
}

uint8_t *ShimCalib_getBytesPtr(void)
{
  return &shimmerCalib_ram[0];
}

void ShimCalib_ram2File(void)
{
  char cal_file_name[48] = "";
#if _FATFS == FATFS_V_0_08B
  DIRS gdc;
#elif _FATFS == FATFS_V_0_12C
  DIR gdc;
#endif
  FIL gfc;
  //sc_t sc1;
  UINT bw;
  FRESULT res;
  uint8_t this_write_size;

  shimmerCalib_ramLen = min(*(uint16_t *) shimmerCalib_ram, SHIMMER_CALIB_RAM_MAX - 2);

  if (shimmerCalib_ramLen > 0)
  {
    strcpy((char *) cal_file_name, "/Calibration");
    res = f_opendir(&gdc, "/Calibration");
    if (res)
    {
      res = f_opendir(&gdc, "/calibration");
      if (res)
      {
        if (res == FR_NO_PATH)
        { //we'll have to make /Calibration first
          res = f_mkdir("/Calibration");
        }
      }
      else
      {
        strcpy((char *) cal_file_name, "/calibration");
      }
    }
    strcat(cal_file_name, "/calib_");
    strcat(cal_file_name, (char *) shimmerCalib_macId);

    res = f_open(&gfc, cal_file_name, (FA_WRITE | FA_CREATE_ALWAYS));
    if (res == FR_NO_FILE)
    {
      return;
    }

    while (gfc.fptr < shimmerCalib_ramLen + 2)
    {
      this_write_size = min(shimmerCalib_ramLen + 2 - gfc.fptr, SHIMMER_CALIB_COPY_SIZE);
      f_write(&gfc, shimmerCalib_ram + gfc.fptr, this_write_size, &bw);
    }

    f_close(&gfc);
#if defined(SHIMMER3)
    _delay_cycles(1200000); //50ms
#elif defined(SHIMMER3R)
    HAL_Delay(50); //50ms
    ShimSd_setFileTimestamp(cal_file_name);
#endif
  }
}

/*
 * if there wasn't a calibration/calibration file, new file won't be created.
 * ram buffer will use 0 as length, 00000 as content
 */
uint8_t ShimCalib_file2Ram(void)
{
#if USE_FATFS
  char cal_file_name[48] = "";
#if _FATFS == FATFS_V_0_08B
  DIRS gdc;
#elif _FATFS == FATFS_V_0_12C
  DIR gdc;
  FILINFO calibFileInfo;
#endif
  FIL gfc;
  UINT bw;
  FRESULT res;
  uint8_t this_read_size;
  uint16_t offset = 0;

  strcpy(cal_file_name, "/Calibration"); //"/Calibration/calibParams"
  if (f_opendir(&gdc, "/Calibration"))
  {
    if (f_opendir(&gdc, "/calibration"))
    {
      //CalibDefault(sensor);
      return 1;
    }
    else
    {
      strcpy(cal_file_name, "/calibration");
    }
  }
  strcat(cal_file_name, "/calib_");
  strcat(cal_file_name, (char *) shimmerCalib_macId);

  res = f_open(&gfc, cal_file_name, (FA_OPEN_EXISTING | FA_READ));
  //TODO investigate why these are different
#if defined(SHIMMER3)
  if (res != FR_OK)
  {
    return 1;
  }
  if (gfc.fptr == gfc.fsize)
  {
    return 1;
  }
#elif defined(SHIMMER3R)
  if (res == FR_NO_FILE)
  {
    return 1;
  }
#endif

  //if file not successfully open, don't wipe the previous dump RAM
  shimmerCalib_ramLen = 0;
  memset(shimmerCalib_ram, 0, SHIMMER_CALIB_RAM_MAX);

#if defined(SHIMMER3R)
  f_stat(cal_file_name, &calibFileInfo);
#endif

#if defined(SHIMMER3)
  while (gfc.fptr != gfc.fsize)
  {
    this_read_size = min(gfc.fsize - gfc.fptr, SHIMMER_CALIB_COPY_SIZE);
#elif defined(SHIMMER3R)
  while (gfc.fptr != calibFileInfo.fsize)
  {
    this_read_size = min(calibFileInfo.fsize - gfc.fptr, SHIMMER_CALIB_COPY_SIZE);
#endif
    res = f_read(&gfc, shimmerCalib_ram + offset, this_read_size, &bw);
    offset += this_read_size;
  }
  shimmerCalib_ramLen = min(*(uint16_t *) shimmerCalib_ram, SHIMMER_CALIB_RAM_MAX - 2);
  f_close(&gfc);
#if defined(SHIMMER3)
  _delay_cycles(1200000); //50ms
#elif defined(SHIMMER3R)
  HAL_Delay(50); //50ms
  ShimSd_setFileTimestamp(cal_file_name);
#endif

#endif
  return 0;
}

uint8_t ShimCalib_singleSensorWrite(const sc_t *sc1)
{
  sc_t curr_sc;
  uint16_t cnt = SC_OFFSET_FIRST_SENSOR;
  uint8_t sensor_found = 0;

  shimmerCalib_ramLen = min(*(uint16_t *) shimmerCalib_ram, SHIMMER_CALIB_RAM_MAX - 2);

  while (cnt + SC_OFFSET_SENSOR_DATA < shimmerCalib_ramLen + 2)
  {
    memcpy((uint8_t *) &curr_sc, shimmerCalib_ram + cnt, SC_OFFSET_SENSOR_TIMESTAMP);
    if ((curr_sc.id == sc1->id) && (curr_sc.range == sc1->range))
    {
      //setup RTC time as calib timestamp
      memcpy(shimmerCalib_ram + cnt + SC_OFFSET_SENSOR_TIMESTAMP, sc1->ts, SC_TIMESTAMP_LENGTH);
      memcpy(shimmerCalib_ram + cnt + SC_OFFSET_SENSOR_DATA, sc1->data.raw,
          curr_sc.data_len);
      sensor_found = 1;
      break;
    }
    cnt += SC_OFFSET_SENSOR_DATA + curr_sc.data_len;
  }

  if (!sensor_found)
  {
    memcpy(shimmerCalib_ram + shimmerCalib_ramLen + 2, (uint8_t *) sc1,
        SC_OFFSET_SENSOR_DATA + sc1->data_len);
    shimmerCalib_ramLen += SC_OFFSET_SENSOR_DATA + sc1->data_len;
    *(uint16_t *) shimmerCalib_ram = min(shimmerCalib_ramLen, SHIMMER_CALIB_RAM_MAX - 2);
  }
  return 0;
}

uint8_t ShimCalib_singleSensorRead(sc_t *sc1)
{
  sc_t curr_sc;
  uint16_t cnt = SC_OFFSET_FIRST_SENSOR;
  uint8_t sensor_found = 0;

  memset((uint8_t *) sc1->data.raw, 0, sc1->data_len);
  shimmerCalib_ramLen = min(*(uint16_t *) shimmerCalib_ram, SHIMMER_CALIB_RAM_MAX - 2);
  while (cnt < shimmerCalib_ramLen + 2)
  {
    memcpy((uint8_t *) &curr_sc, shimmerCalib_ram + cnt, SC_OFFSET_SENSOR_TIMESTAMP);
    if ((curr_sc.id == sc1->id) && (curr_sc.range == sc1->range))
    {
      memcpy((uint8_t *) sc1, shimmerCalib_ram + cnt,
          SC_OFFSET_SENSOR_DATA + curr_sc.data_len);
      sensor_found = 1;
      break;
    }
    cnt += SC_OFFSET_SENSOR_DATA + curr_sc.data_len;
  }
  if (!sensor_found)
  {
    return 1;
  }
  return 0;
}

void ShimCalib_checkRamLen(void)
{
  shimmerCalib_ramLen = min(*(uint16_t *) shimmerCalib_ram, SHIMMER_CALIB_RAM_MAX - 2);
  memset(shimmerCalib_ram + shimmerCalib_ramLen + 2, 0,
      SHIMMER_CALIB_RAM_MAX - shimmerCalib_ramLen - 2);
}

void ShimCalib_ramTempInit(void)
{
  shimmerCalib_ramTempMax = 0;
  shimmerCalib_ramTempLen = 0;
}

//return 0 : success on this write; 1: successfully finished all;  0xff: fail
//the calib dump write operation starts with offset = 0, ends with shimmerCalib_ramTempMax bytes received.
uint8_t ShimCalib_ramWrite(const uint8_t *buf, uint8_t length, uint16_t offset)
{
  if ((length <= 128) && (offset <= (SHIMMER_CALIB_RAM_MAX - 1))
      && (length + offset <= SHIMMER_CALIB_RAM_MAX))
  {
    memcpy(shimmerCalib_ramTemp + offset, buf, length);
  }
  else
  {
    return 0xff;
  }

  //to start a new calib_dump transmission, the sw must use offset = 0 to setup the correct length.
  //offset = 1 is not suggested, but will be considered.
  //starting with offset > 2 is not accepted.
  if (offset < 2)
  {
    //+ 2 as length bytes in header are not included in the overall array length
    shimmerCalib_ramTempMax = (*(uint16_t *) shimmerCalib_ramTemp) + 2;
    shimmerCalib_ramTempLen = length;
  }
  else
  {
    shimmerCalib_ramTempLen += length;
  }

  if (shimmerCalib_ramTempLen >= shimmerCalib_ramTempMax)
  {
    memcpy(shimmerCalib_ram, shimmerCalib_ramTemp, shimmerCalib_ramTempMax);

    ShimCalib_initVer();
    ShimCalib_ramTempInit();

    //Update config bytes with latest & current calib from calib dump
    ShimCalib_calibDumpToConfigBytesAndSdHeaderAll(1);
    return 1;
  }

  return 0;
}

//return 0 : success; 1: fail
uint8_t ShimCalib_ramRead(uint8_t *buf, uint8_t length, uint16_t offset)
{
  shimmerCalib_ramLen = min(*(uint16_t *) shimmerCalib_ram, SHIMMER_CALIB_RAM_MAX - 2);
  if ((length <= 128) && (offset <= (SHIMMER_CALIB_RAM_MAX - 1))
      && (length + offset <= SHIMMER_CALIB_RAM_MAX))
  {
    memcpy(buf, shimmerCalib_ram + offset, length);
    return 0;
  }
  else
  {
    memset(buf, 0, length);
    return 1;
  }
}

void ShimCalib_defaultAll(void)
{
#if defined(SHIMMER3)
  ShimCalib_setDefaultForSensor(SC_SENSOR_ANALOG_ACCEL);
  ShimCalib_setDefaultForSensor(SC_SENSOR_MPU9X50_ICM20948_GYRO);
  ShimCalib_setDefaultForSensor(SC_SENSOR_LSM303_ACCEL);
  ShimCalib_setDefaultForSensor(SC_SENSOR_LSM303_MAG);
#elif defined(SHIMMER3R)
  ShimCalib_setDefaultForSensor(SC_SENSOR_LSM6DSV_ACCEL);
  ShimCalib_setDefaultForSensor(SC_SENSOR_LSM6DSV_GYRO);
  ShimCalib_setDefaultForSensor(SC_SENSOR_LIS2DW12_ACCEL);
  ShimCalib_setDefaultForSensor(SC_SENSOR_ADXL371_ACCEL);
  ShimCalib_setDefaultForSensor(SC_SENSOR_LIS3MDL_MAG);
  ShimCalib_setDefaultForSensor(SC_SENSOR_LIS2MDL_MAG);
#endif
}

void ShimCalib_setDefaultForSensor(uint8_t sensor)
{
  sc_t sc1;

  //Set timestamp to 0 to symbolise default calibration in-use
  memset(sc1.ts, 0, sizeof(sc1.ts));

#if defined(SHIMMER3)
  if (sensor == SC_SENSOR_ANALOG_ACCEL)
  {
    ShimCalib_setDefaultKionixCalib(&sc1);
  }
  else if (sensor == SC_SENSOR_MPU9X50_ICM20948_GYRO)
  {
    ShimCalib_setDefaultMpu9X50Icm20948GyroCalib(&sc1);
  }
  else if (sensor == SC_SENSOR_LSM303_ACCEL)
  {
    ShimCalib_setDefaultLsm303AccelCalib(&sc1);
  }
  else if (sensor == SC_SENSOR_LSM303_MAG)
  {
    ShimCalib_setDefaultLsm303MagCalib(&sc1);
  }
#elif defined(SHIMMER3R)
  if (sensor == SC_SENSOR_LSM6DSV_ACCEL)
  {
    ShimCalib_setDefaultLsm6dsvAccelCalib(&sc1);
  }
  else if (sensor == SC_SENSOR_LSM6DSV_GYRO)
  {
    ShimCalib_setDefaultLsm6dsvGyroCalib(&sc1);
  }
  else if (sensor == SC_SENSOR_LIS2DW12_ACCEL)
  {
    ShimCalib_setDefaultLis2dw12AccelCalib(&sc1);
  }
  else if (sensor == SC_SENSOR_ADXL371_ACCEL)
  {
    ShimCalib_setDefaultAdxl371AccelCalib(&sc1);
  }
  else if (sensor == SC_SENSOR_LIS3MDL_MAG)
  {
    ShimCalib_setDefaultLis3mdlMagCalib(&sc1);
  }
  else if (sensor == SC_SENSOR_LIS2MDL_MAG)
  {
    ShimCalib_setDefaultLis2mdlMagCalib(&sc1);
  }
#endif
}

#if defined(SHIMMER3)
void ShimCalib_setDefaultKionixCalib(sc_t *sc1Ptr)
{
  uint16_t bias, sensitivity;
  sc1Ptr->id = SC_SENSOR_ANALOG_ACCEL;
  sc1Ptr->data_len = SC_DATA_LEN_ANALOG_ACCEL;
  sc1Ptr->range = SC_SENSOR_RANGE_ANALOG_ACCEL;

  bias = 2047;
  sensitivity = 83;

  sc1Ptr->data.dd.bias_x = bias;
  sc1Ptr->data.dd.bias_y = bias;
  sc1Ptr->data.dd.bias_z = bias;
  sc1Ptr->data.dd.sens_x = sensitivity;
  sc1Ptr->data.dd.sens_y = sensitivity;
  sc1Ptr->data.dd.sens_z = sensitivity;
  sc1Ptr->data.dd.align_xx = 0;
  sc1Ptr->data.dd.align_xy = -100;
  sc1Ptr->data.dd.align_xz = 0;
  sc1Ptr->data.dd.align_yx = -100;
  sc1Ptr->data.dd.align_yy = 0;
  sc1Ptr->data.dd.align_yz = 0;
  sc1Ptr->data.dd.align_zx = 0;
  sc1Ptr->data.dd.align_zy = 0;
  sc1Ptr->data.dd.align_zz = -100;

  ShimCalib_reverseBiasAndSensitivityByteOrder(sc1Ptr);
  ShimCalib_singleSensorWrite(sc1Ptr);
}

void ShimCalib_setDefaultMpu9X50Icm20948GyroCalib(sc_t *sc1Ptr)
{
  uint16_t bias, sensitivity;
  sc1Ptr->id = SC_SENSOR_MPU9X50_ICM20948_GYRO;
  sc1Ptr->data_len = SC_DATA_LEN_MPU9X50_ICM20948_GYRO;
  for (sc1Ptr->range = 0;
       sc1Ptr->range < SC_SENSOR_RANGE_MAX_MPU9X50_ICM20948_GYRO; sc1Ptr->range++)
  {
    bias = 0;
    if (sc1Ptr->range == SC_SENSOR_RANGE_MPU9X50_ICM20948_GYRO_250DPS)
    {
      sensitivity = 13100;
    }
    else if (sc1Ptr->range == SC_SENSOR_RANGE_MPU9X50_ICM20948_GYRO_500DPS)
    {
      sensitivity = 6550;
    }
    else if (sc1Ptr->range == SC_SENSOR_RANGE_MPU9X50_ICM20948_GYRO_1000DPS)
    {
      sensitivity = 3280;
    }
    else
    { //(sdHeadText[SDH_GYRO_RANGE] == SC_SENSOR_RANGE_MPU9250_GYRO_2000DPS)
      sensitivity = 1640;
    }
    sc1Ptr->data.dd.bias_x = bias;
    sc1Ptr->data.dd.bias_y = bias;
    sc1Ptr->data.dd.bias_z = bias;
    sc1Ptr->data.dd.sens_x = sensitivity;
    sc1Ptr->data.dd.sens_y = sensitivity;
    sc1Ptr->data.dd.sens_z = sensitivity;
    sc1Ptr->data.dd.align_xx = 0;
    sc1Ptr->data.dd.align_xy = -100;
    sc1Ptr->data.dd.align_xz = 0;
    sc1Ptr->data.dd.align_yx = -100;
    sc1Ptr->data.dd.align_yy = 0;
    sc1Ptr->data.dd.align_yz = 0;
    sc1Ptr->data.dd.align_zx = 0;
    sc1Ptr->data.dd.align_zy = 0;
    sc1Ptr->data.dd.align_zz = -100;

    ShimCalib_reverseBiasAndSensitivityByteOrder(sc1Ptr);
    ShimCalib_singleSensorWrite(sc1Ptr);
  }
}

void ShimCalib_setDefaultLsm303AccelCalib(sc_t *sc1Ptr)
{
  uint16_t bias, sensitivity;
  sc1Ptr->id = SC_SENSOR_LSM303_ACCEL;
  sc1Ptr->data_len = SC_DATA_LEN_LSM303_ACCEL;
  for (sc1Ptr->range = 0; sc1Ptr->range < SC_SENSOR_RANGE_MAX_LSM303_ACCEL;
       sc1Ptr->range++)
  {
    bias = 0;
    if (sc1Ptr->range == SC_SENSOR_RANGE_LSM303_ACCEL_2G)
    {
      sensitivity = 1631;
    }
    else if (sc1Ptr->range == SC_SENSOR_RANGE_LSM303_ACCEL_4G)
    {
      sensitivity = 815;
    }
    else if (sc1Ptr->range == SC_SENSOR_RANGE_LSM303_ACCEL_8G)
    {
      sensitivity = 408;
    }
    else
    { //(sc1Ptr->range == SC_SENSOR_RANGE_LSM303DLHC_ACCEL_16G)
      sensitivity = 135;
    }
    sc1Ptr->data.dd.bias_x = bias;
    sc1Ptr->data.dd.bias_y = bias;
    sc1Ptr->data.dd.bias_z = bias;
    sc1Ptr->data.dd.sens_x = sensitivity;
    sc1Ptr->data.dd.sens_y = sensitivity;
    sc1Ptr->data.dd.sens_z = sensitivity;
    sc1Ptr->data.dd.align_xx = -100;
    sc1Ptr->data.dd.align_xy = 0;
    sc1Ptr->data.dd.align_xz = 0;
    sc1Ptr->data.dd.align_yx = 0;
    sc1Ptr->data.dd.align_yy = 100;
    sc1Ptr->data.dd.align_yz = 0;
    sc1Ptr->data.dd.align_zx = 0;
    sc1Ptr->data.dd.align_zy = 0;
    sc1Ptr->data.dd.align_zz = -100;

    ShimCalib_reverseBiasAndSensitivityByteOrder(sc1Ptr);
    ShimCalib_singleSensorWrite(sc1Ptr);
  }
}

void ShimCalib_setDefaultLsm303MagCalib(sc_t *sc1Ptr)
{
  uint16_t bias;
  sc1Ptr->id = SC_SENSOR_LSM303_MAG;
  sc1Ptr->data_len = SC_DATA_LEN_LSM303_MAG;
  for (sc1Ptr->range = 0; sc1Ptr->range < SC_SENSOR_RANGE_MAX_LSM303_MAG; sc1Ptr->range++)
  {
    bias = 0;
    sc1Ptr->data.dd.bias_x = bias;
    sc1Ptr->data.dd.bias_y = bias;
    sc1Ptr->data.dd.bias_z = bias;
    if (sc1Ptr->range == SC_SENSOR_RANGE_LSM303_MAG_1_3G)
    {
      sc1Ptr->data.dd.sens_x = 1100;
      sc1Ptr->data.dd.sens_y = 1100;
      sc1Ptr->data.dd.sens_z = 980;
    }
    else if (sc1Ptr->range == SC_SENSOR_RANGE_LSM303_MAG_1_9G)
    {
      sc1Ptr->data.dd.sens_x = 855;
      sc1Ptr->data.dd.sens_y = 855;
      sc1Ptr->data.dd.sens_z = 760;
    }
    else if (sc1Ptr->range == SC_SENSOR_RANGE_LSM303_MAG_2_5G)
    {
      sc1Ptr->data.dd.sens_x = 670;
      sc1Ptr->data.dd.sens_y = 670;
      sc1Ptr->data.dd.sens_z = 600;
    }
    else if (sc1Ptr->range == SC_SENSOR_RANGE_LSM303_MAG_4_0G)
    {
      sc1Ptr->data.dd.sens_x = 450;
      sc1Ptr->data.dd.sens_y = 450;
      sc1Ptr->data.dd.sens_z = 400;
    }
    else if (sc1Ptr->range == SC_SENSOR_RANGE_LSM303_MAG_4_7G)
    {
      sc1Ptr->data.dd.sens_x = 400;
      sc1Ptr->data.dd.sens_y = 400;
      sc1Ptr->data.dd.sens_z = 355;
    }
    else if (sc1Ptr->range == SC_SENSOR_RANGE_LSM303_MAG_5_6G)
    {
      sc1Ptr->data.dd.sens_x = 330;
      sc1Ptr->data.dd.sens_y = 330;
      sc1Ptr->data.dd.sens_z = 295;
    }
    else
    { //sc1Ptr->range == SC_SENSOR_RANGE_LSM303DLHC_MAG_8_1G)
      sc1Ptr->data.dd.sens_x = 230;
      sc1Ptr->data.dd.sens_y = 230;
      sc1Ptr->data.dd.sens_z = 205;
    }
    sc1Ptr->data.dd.align_xx = -100;
    sc1Ptr->data.dd.align_xy = 0;
    sc1Ptr->data.dd.align_xz = 0;
    sc1Ptr->data.dd.align_yx = 0;
    sc1Ptr->data.dd.align_yy = 100;
    sc1Ptr->data.dd.align_yz = 0;
    sc1Ptr->data.dd.align_zx = 0;
    sc1Ptr->data.dd.align_zy = 0;
    sc1Ptr->data.dd.align_zz = -100;

    ShimCalib_reverseBiasAndSensitivityByteOrder(sc1Ptr);
    ShimCalib_singleSensorWrite(sc1Ptr);
  }
}

#elif defined(SHIMMER3R)
void ShimCalib_setDefaultLsm6dsvAccelCalib(sc_t *sc1Ptr)
{
  uint16_t bias, sensitivity;
  sc1Ptr->id = SC_SENSOR_LSM6DSV_ACCEL;
  sc1Ptr->data_len = SC_DATA_LEN_STD_IMU_CALIB;
  for (sc1Ptr->range = 0; sc1Ptr->range < 4; sc1Ptr->range++)
  {
    bias = 0;
    if (sc1Ptr->range == LSM6DSV_2g)
    {
      sensitivity = 1672;
    }
    else if (sc1Ptr->range == LSM6DSV_4g)
    {
      sensitivity = 836;
    }
    else if (sc1Ptr->range == LSM6DSV_8g)
    {
      sensitivity = 418;
    }
    else
    { //(sc1Ptr->range == LSM6DSV_16g)
      sensitivity = 209;
    }
    sc1Ptr->data.dd.bias_x = bias;
    sc1Ptr->data.dd.bias_y = bias;
    sc1Ptr->data.dd.bias_z = bias;
    sc1Ptr->data.dd.sens_x = sensitivity;
    sc1Ptr->data.dd.sens_y = sensitivity;
    sc1Ptr->data.dd.sens_z = sensitivity;
    sc1Ptr->data.dd.align_xx = -100;
    sc1Ptr->data.dd.align_xy = 0;
    sc1Ptr->data.dd.align_xz = 0;
    sc1Ptr->data.dd.align_yx = 0;
    sc1Ptr->data.dd.align_yy = 100;
    sc1Ptr->data.dd.align_yz = 0;
    sc1Ptr->data.dd.align_zx = 0;
    sc1Ptr->data.dd.align_zy = 0;
    sc1Ptr->data.dd.align_zz = -100;

    ShimCalib_reverseBiasAndSensitivityByteOrder(sc1Ptr);
    ShimCalib_singleSensorWrite(sc1Ptr);
  }
}

void ShimCalib_setDefaultLsm6dsvGyroCalib(sc_t *sc1Ptr)
{
  uint16_t bias, sensitivity;
  sc1Ptr->id = SC_SENSOR_LSM6DSV_GYRO;
  sc1Ptr->data_len = SC_DATA_LEN_STD_IMU_CALIB;
  for (sc1Ptr->range = 0; sc1Ptr->range < 6; sc1Ptr->range++)
  {
    bias = 0;
    if (sc1Ptr->range == LSM6DSV_125dps)
    {
      sensitivity = 229;
    }
    else if (sc1Ptr->range == LSM6DSV_250dps)
    {
      sensitivity = 114;
    }
    else if (sc1Ptr->range == LSM6DSV_500dps)
    {
      sensitivity = 57;
    }
    else if (sc1Ptr->range == LSM6DSV_1000dps)
    {
      sensitivity = 29;
    }
    else if (sc1Ptr->range == LSM6DSV_2000dps)
    {
      sensitivity = 14;
    }
    else
    { //(sc1Ptr->range == LSM6DSV_4000dps)
      sensitivity = 7;
    }

    sensitivity = sensitivity * 100; //Scale up

    sc1Ptr->data.dd.bias_x = bias;
    sc1Ptr->data.dd.bias_y = bias;
    sc1Ptr->data.dd.bias_z = bias;
    sc1Ptr->data.dd.sens_x = sensitivity;
    sc1Ptr->data.dd.sens_y = sensitivity;
    sc1Ptr->data.dd.sens_z = sensitivity;
    sc1Ptr->data.dd.align_xx = -100;
    sc1Ptr->data.dd.align_xy = 0;
    sc1Ptr->data.dd.align_xz = 0;
    sc1Ptr->data.dd.align_yx = 0;
    sc1Ptr->data.dd.align_yy = 100;
    sc1Ptr->data.dd.align_yz = 0;
    sc1Ptr->data.dd.align_zx = 0;
    sc1Ptr->data.dd.align_zy = 0;
    sc1Ptr->data.dd.align_zz = -100;

    ShimCalib_reverseBiasAndSensitivityByteOrder(sc1Ptr);
    ShimCalib_singleSensorWrite(sc1Ptr);
  }
}

void ShimCalib_setDefaultAdxl371AccelCalib(sc_t *sc1Ptr)
{
  uint16_t bias, sensitivity;
  sc1Ptr->id = SC_SENSOR_ADXL371_ACCEL;
  sc1Ptr->data_len = SC_DATA_LEN_STD_IMU_CALIB;

  bias = 10;       //+1g
  sensitivity = 1; //100mg/LSB which equates to 1.0197 LSB/m/s^2

  sc1Ptr->range = 0;
  sc1Ptr->data.dd.bias_x = bias;
  sc1Ptr->data.dd.bias_y = bias;
  sc1Ptr->data.dd.bias_z = bias;
  sc1Ptr->data.dd.sens_x = sensitivity;
  sc1Ptr->data.dd.sens_y = sensitivity;
  sc1Ptr->data.dd.sens_z = sensitivity;
  sc1Ptr->data.dd.align_xx = 0;
  sc1Ptr->data.dd.align_xy = 100;
  sc1Ptr->data.dd.align_xz = 0;
  sc1Ptr->data.dd.align_yx = 100;
  sc1Ptr->data.dd.align_yy = 0;
  sc1Ptr->data.dd.align_yz = 0;
  sc1Ptr->data.dd.align_zx = 0;
  sc1Ptr->data.dd.align_zy = 0;
  sc1Ptr->data.dd.align_zz = -100;

  ShimCalib_reverseBiasAndSensitivityByteOrder(sc1Ptr);
  ShimCalib_singleSensorWrite(sc1Ptr);
}

void ShimCalib_setDefaultLis2dw12AccelCalib(sc_t *sc1Ptr)
{
  uint16_t bias, sensitivity;
  sc1Ptr->id = SC_SENSOR_LIS2DW12_ACCEL;
  sc1Ptr->data_len = SC_DATA_LEN_STD_IMU_CALIB;
  for (sc1Ptr->range = 0; sc1Ptr->range < 4; sc1Ptr->range++)
  {
    bias = 0;
    if (sc1Ptr->range == LIS2DW12_2g)
    {
      sensitivity = 1671;
    }
    else if (sc1Ptr->range == LIS2DW12_4g)
    {
      sensitivity = 836;
    }
    else if (sc1Ptr->range == LIS2DW12_8g)
    {
      sensitivity = 418;
    }
    else
    { //(sc1Ptr->range == LIS2DW12_16g)
      sensitivity = 209;
    }
    sc1Ptr->data.dd.bias_x = bias;
    sc1Ptr->data.dd.bias_y = bias;
    sc1Ptr->data.dd.bias_z = bias;
    sc1Ptr->data.dd.sens_x = sensitivity;
    sc1Ptr->data.dd.sens_y = sensitivity;
    sc1Ptr->data.dd.sens_z = sensitivity;
    sc1Ptr->data.dd.align_xx = 0;
    sc1Ptr->data.dd.align_xy = -100;
    sc1Ptr->data.dd.align_xz = 0;
    sc1Ptr->data.dd.align_yx = -100;
    sc1Ptr->data.dd.align_yy = 0;
    sc1Ptr->data.dd.align_yz = 0;
    sc1Ptr->data.dd.align_zx = 0;
    sc1Ptr->data.dd.align_zy = 0;
    sc1Ptr->data.dd.align_zz = -100;

    ShimCalib_reverseBiasAndSensitivityByteOrder(sc1Ptr);
    ShimCalib_singleSensorWrite(sc1Ptr);
  }
}

void ShimCalib_setDefaultLis2mdlMagCalib(sc_t *sc1Ptr)
{
  uint16_t bias, sensitivity;
  sc1Ptr->id = SC_SENSOR_LIS2MDL_MAG;
  sc1Ptr->data_len = SC_DATA_LEN_STD_IMU_CALIB;

  bias = 0;
  sensitivity = 667;

  sc1Ptr->range = 0;
  sc1Ptr->data.dd.bias_x = bias;
  sc1Ptr->data.dd.bias_y = bias;
  sc1Ptr->data.dd.bias_z = bias;
  sc1Ptr->data.dd.sens_x = sensitivity;
  sc1Ptr->data.dd.sens_y = sensitivity;
  sc1Ptr->data.dd.sens_z = sensitivity;
  sc1Ptr->data.dd.align_xx = -100;
  sc1Ptr->data.dd.align_xy = 0;
  sc1Ptr->data.dd.align_xz = 0;
  sc1Ptr->data.dd.align_yx = 0;
  sc1Ptr->data.dd.align_yy = -100;
  sc1Ptr->data.dd.align_yz = 0;
  sc1Ptr->data.dd.align_zx = 0;
  sc1Ptr->data.dd.align_zy = 0;
  sc1Ptr->data.dd.align_zz = -100;

  ShimCalib_reverseBiasAndSensitivityByteOrder(sc1Ptr);
  ShimCalib_singleSensorWrite(sc1Ptr);
}

void ShimCalib_setDefaultLis3mdlMagCalib(sc_t *sc1Ptr)
{
  uint16_t bias, sensitivity;
  sc1Ptr->id = SC_SENSOR_LIS3MDL_MAG;
  sc1Ptr->data_len = SC_DATA_LEN_STD_IMU_CALIB;
  for (sc1Ptr->range = 0; sc1Ptr->range < 4; sc1Ptr->range++)
  {
    bias = 0;
    if (sc1Ptr->range == LIS3MDL_4_GAUSS)
    {
      sensitivity = 6842;
    }
    else if (sc1Ptr->range == LIS3MDL_8_GAUSS)
    {
      sensitivity = 3421;
    }
    else if (sc1Ptr->range == LIS3MDL_12_GAUSS)
    {
      sensitivity = 2281;
    }
    else
    { //(sc1Ptr->range == LIS3MDL_16_GAUSS)
      sensitivity = 1711;
    }
    sc1Ptr->data.dd.bias_x = bias;
    sc1Ptr->data.dd.bias_y = bias;
    sc1Ptr->data.dd.bias_z = bias;
    sc1Ptr->data.dd.sens_x = sensitivity;
    sc1Ptr->data.dd.sens_y = sensitivity;
    sc1Ptr->data.dd.sens_z = sensitivity;
    sc1Ptr->data.dd.align_xx = 100;
    sc1Ptr->data.dd.align_xy = 0;
    sc1Ptr->data.dd.align_xz = 0;
    sc1Ptr->data.dd.align_yx = 0;
    sc1Ptr->data.dd.align_yy = -100;
    sc1Ptr->data.dd.align_yz = 0;
    sc1Ptr->data.dd.align_zx = 0;
    sc1Ptr->data.dd.align_zy = 0;
    sc1Ptr->data.dd.align_zz = -100;

    ShimCalib_reverseBiasAndSensitivityByteOrder(sc1Ptr);
    ShimCalib_singleSensorWrite(sc1Ptr);
  }
}
#endif

void ShimCalib_reverseBiasAndSensitivityByteOrder(sc_t *sc1Ptr)
{
  uint16_t tmpBias, tmpSens;

  tmpBias = sc1Ptr->data.dd.bias_x;
  sc1Ptr->data.dd.bias_x = (((tmpBias & 0x00FF) << 8) | ((tmpBias & 0xFF00) >> 8));
  tmpBias = sc1Ptr->data.dd.bias_y;
  sc1Ptr->data.dd.bias_y = (((tmpBias & 0x00FF) << 8) | ((tmpBias & 0xFF00) >> 8));
  tmpBias = sc1Ptr->data.dd.bias_z;
  sc1Ptr->data.dd.bias_z = (((tmpBias & 0x00FF) << 8) | ((tmpBias & 0xFF00) >> 8));

  tmpSens = sc1Ptr->data.dd.sens_x;
  sc1Ptr->data.dd.sens_x = (((tmpSens & 0x00FF) << 8) | ((tmpSens & 0xFF00) >> 8));
  tmpSens = sc1Ptr->data.dd.sens_y;
  sc1Ptr->data.dd.sens_y = (((tmpSens & 0x00FF) << 8) | ((tmpSens & 0xFF00) >> 8));
  tmpSens = sc1Ptr->data.dd.sens_z;
  sc1Ptr->data.dd.sens_z = (((tmpSens & 0x00FF) << 8) | ((tmpSens & 0xFF00) >> 8));
}

void ShimCalib_initFromConfigBytesAll(void)
{
  ShimCalib_configBytes0To127ToCalibDumpBytes(1);
  ShimCalib_configBytes128To255ToCalibDumpBytes(1);
}

void ShimCalib_updateFromConfigBytesAll(void)
{
  ShimCalib_configBytes0To127ToCalibDumpBytes(0);
  ShimCalib_configBytes128To255ToCalibDumpBytes(0);
}

void ShimCalib_configBytes0To127ToCalibDumpBytes(uint8_t setCalibTsZero)
{
#if defined(SHIMMER3)
  ShimCalib_configBytesToCalibDump(SC_SENSOR_ANALOG_ACCEL, setCalibTsZero);
  ShimCalib_configBytesToCalibDump(SC_SENSOR_MPU9X50_ICM20948_GYRO, setCalibTsZero);
  ShimCalib_configBytesToCalibDump(SC_SENSOR_LSM303_ACCEL, setCalibTsZero);
  ShimCalib_configBytesToCalibDump(SC_SENSOR_LSM303_MAG, setCalibTsZero);
#elif defined(SHIMMER3R)
  ShimCalib_configBytesToCalibDump(SC_SENSOR_LSM6DSV_ACCEL, setCalibTsZero);
  ShimCalib_configBytesToCalibDump(SC_SENSOR_LSM6DSV_GYRO, setCalibTsZero);
  ShimCalib_configBytesToCalibDump(SC_SENSOR_LIS2DW12_ACCEL, setCalibTsZero);
  ShimCalib_configBytesToCalibDump(SC_SENSOR_LIS2MDL_MAG, setCalibTsZero);
#endif
}

void ShimCalib_configBytes128To255ToCalibDumpBytes(uint8_t setCalibTsZero)
{
#if defined(SHIMMER3R)
  ShimCalib_configBytesToCalibDump(SC_SENSOR_ADXL371_ACCEL, setCalibTsZero);
  ShimCalib_configBytesToCalibDump(SC_SENSOR_LIS3MDL_MAG, setCalibTsZero);
#endif
}

void ShimCalib_configBytesToCalibDump(uint8_t id, uint8_t setCalibTsZero)
{
  gConfigBytes *configBytes = ShimConfig_getStoredConfig();

#if defined(SHIMMER3)
  if (id == SC_SENSOR_ALL || id == SC_SENSOR_ANALOG_ACCEL)
  {
    ShimCalib_singleSensorToCalibDump(SC_SENSOR_ANALOG_ACCEL,
        SC_SENSOR_RANGE_ANALOG_ACCEL, SC_DATA_LEN_ANALOG_ACCEL,
        &configBytes->lnAccelCalib.rawBytes[0], setCalibTsZero);
  }
  if (id == SC_SENSOR_ALL || id == SC_SENSOR_MPU9X50_ICM20948_GYRO)
  {
    ShimCalib_singleSensorToCalibDump(SC_SENSOR_MPU9X50_ICM20948_GYRO,
        ShimConfig_gyroRangeGet(), SC_DATA_LEN_MPU9X50_ICM20948_GYRO,
        &configBytes->gyroCalib.rawBytes[0], setCalibTsZero);
  }
  if (id == SC_SENSOR_ALL || id == SC_SENSOR_LSM303_ACCEL)
  {
    ShimCalib_singleSensorToCalibDump(SC_SENSOR_LSM303_ACCEL,
        configBytes->wrAccelRange, SC_DATA_LEN_LSM303_ACCEL,
        &configBytes->wrAccelCalib.rawBytes[0], setCalibTsZero);
  }
  if (id == SC_SENSOR_ALL || id == SC_SENSOR_LSM303_MAG)
  {
    ShimCalib_singleSensorToCalibDump(SC_SENSOR_LSM303_MAG, configBytes->magRange,
        SC_DATA_LEN_LSM303_MAG, &configBytes->magCalib.rawBytes[0], setCalibTsZero);
  }
#elif defined(SHIMMER3R)
  if (id == SC_SENSOR_ALL || id == SC_SENSOR_LSM6DSV_ACCEL)
  {
    ShimCalib_singleSensorToCalibDump(SC_SENSOR_LSM6DSV_ACCEL,
        configBytes->lnAccelRange, SC_DATA_LEN_STD_IMU_CALIB,
        &configBytes->lnAccelCalib.rawBytes[0], setCalibTsZero);
  }
  if (id == SC_SENSOR_ALL || id == SC_SENSOR_LSM6DSV_GYRO)
  {
    ShimCalib_singleSensorToCalibDump(SC_SENSOR_LSM6DSV_GYRO,
        ShimConfig_gyroRangeGet(), SC_DATA_LEN_STD_IMU_CALIB,
        &configBytes->gyroCalib.rawBytes[0], setCalibTsZero);
  }
  if (id == SC_SENSOR_ALL || id == SC_SENSOR_LIS2DW12_ACCEL)
  {
    ShimCalib_singleSensorToCalibDump(SC_SENSOR_LIS2DW12_ACCEL,
        configBytes->wrAccelRange, SC_DATA_LEN_STD_IMU_CALIB,
        &configBytes->wrAccelCalib.rawBytes[0], setCalibTsZero);
  }
  if (id == SC_SENSOR_ALL || id == SC_SENSOR_ADXL371_ACCEL)
  {
    ShimCalib_singleSensorToCalibDump(SC_SENSOR_ADXL371_ACCEL, 0, SC_DATA_LEN_STD_IMU_CALIB,
        &configBytes->altAccelCalib.rawBytes[0], setCalibTsZero);
  }
  if (id == SC_SENSOR_ALL || id == SC_SENSOR_LIS3MDL_MAG)
  {
    ShimCalib_singleSensorToCalibDump(SC_SENSOR_LIS3MDL_MAG, configBytes->altMagRange,
        SC_DATA_LEN_STD_IMU_CALIB, &configBytes->altMagCalib.rawBytes[0], setCalibTsZero);
  }
  if (id == SC_SENSOR_ALL || id == SC_SENSOR_LIS2MDL_MAG)
  {
    ShimCalib_singleSensorToCalibDump(SC_SENSOR_LIS2MDL_MAG, SC_SENSOR_RANGE_LIS2MDL_RANGE,
        SC_DATA_LEN_STD_IMU_CALIB, &configBytes->magCalib.rawBytes[0], setCalibTsZero);
  }
#endif
}

void ShimCalib_singleSensorToCalibDump(uint16_t id,
    uint8_t range,
    uint8_t data_len,
    uint8_t *configBytePtr,
    uint8_t setCalibTsZero)
{
  uint8_t info_valid = 0;
  uint8_t byte_cnt = 0;
  sc_t sc1;

  /* Mark as valid if any calibration byte is not 0xFF (i.e., the default
   * config byte value) */
  for (byte_cnt = data_len; byte_cnt > 0; byte_cnt--)
  {
    if (configBytePtr[byte_cnt] != 0xFF)
    {
      info_valid = 1;
      break;
    }
  }

  /* Copy from config bytes to temporary object and then to calib dump in RAM */
  if (info_valid)
  {
    sc1.id = id;
    sc1.range = range;
    sc1.data_len = data_len;

    if (setCalibTsZero)
    {
      memset(sc1.ts, 0, sizeof(sc1.ts));
    }
    else
    {
      *(uint64_t *) (sc1.ts) = RTC_getRwcTime();
    }

    memcpy(sc1.data.raw, configBytePtr, sc1.data_len);
    ShimCalib_singleSensorWrite(&sc1);
  }
}

//
void ShimCalib_calibDumpToConfigBytesAndSdHeaderAll(uint8_t writeToFlash)
{
#if defined(SHIMMER3)
  ShimCalib_calibDumpToConfigBytesAndSdHeaderSingleSensor(SC_SENSOR_ANALOG_ACCEL, writeToFlash);
  ShimCalib_calibDumpToConfigBytesAndSdHeaderSingleSensor(
      SC_SENSOR_MPU9X50_ICM20948_GYRO, writeToFlash);
  ShimCalib_calibDumpToConfigBytesAndSdHeaderSingleSensor(SC_SENSOR_LSM303_MAG, writeToFlash);
  ShimCalib_calibDumpToConfigBytesAndSdHeaderSingleSensor(SC_SENSOR_LSM303_ACCEL, writeToFlash);
#elif defined(SHIMMER3R)
  ShimCalib_calibDumpToConfigBytesAndSdHeaderSingleSensor(SC_SENSOR_LSM6DSV_ACCEL, writeToFlash);
  ShimCalib_calibDumpToConfigBytesAndSdHeaderSingleSensor(SC_SENSOR_LSM6DSV_GYRO, writeToFlash);
  ShimCalib_calibDumpToConfigBytesAndSdHeaderSingleSensor(
      SC_SENSOR_LIS2DW12_ACCEL, writeToFlash);
  ShimCalib_calibDumpToConfigBytesAndSdHeaderSingleSensor(SC_SENSOR_ADXL371_ACCEL, writeToFlash);
  ShimCalib_calibDumpToConfigBytesAndSdHeaderSingleSensor(SC_SENSOR_LIS3MDL_MAG, writeToFlash);
  ShimCalib_calibDumpToConfigBytesAndSdHeaderSingleSensor(SC_SENSOR_LIS2MDL_MAG, writeToFlash);
#endif
}

void ShimCalib_calibDumpToConfigBytesAndSdHeaderSingleSensor(uint8_t sensor, uint8_t writeToFlash)
{
  sc_t sc1;
  uint16_t scs_infomem_offset, scs_sdhead_offset, scs_sdhead_ts;
  sc1.id = sensor;
  sc1.data_len = ShimCalib_findLength(&sc1);

  gConfigBytes *configBytes = ShimConfig_getStoredConfig();

  switch (sensor)
  {
#if defined(SHIMMER3)
    case SC_SENSOR_ANALOG_ACCEL:
      scs_infomem_offset = NV_LN_ACCEL_CALIBRATION;
      scs_sdhead_offset = SDH_LN_ACCEL_CALIBRATION;
      scs_sdhead_ts = SDH_LN_ACCEL_CALIB_TS;
      sc1.range = SC_SENSOR_RANGE_ANALOG_ACCEL;
      break;
    case SC_SENSOR_MPU9X50_ICM20948_GYRO:
      scs_infomem_offset = NV_GYRO_CALIBRATION;
      scs_sdhead_offset = SDH_GYRO_CALIBRATION;
      scs_sdhead_ts = SDH_GYRO_CALIB_TS;
      sc1.range = ShimConfig_gyroRangeGet();
      break;
    case SC_SENSOR_LSM303_MAG:
      scs_infomem_offset = NV_MAG_CALIBRATION;
      scs_sdhead_offset = SDH_MAG_CALIBRATION;
      scs_sdhead_ts = SDH_MAG_CALIB_TS;
      sc1.range = configBytes->magRange;
      break;
    case SC_SENSOR_LSM303_ACCEL:
      scs_infomem_offset = NV_WR_ACCEL_CALIBRATION;
      scs_sdhead_offset = SDH_WR_ACCEL_CALIBRATION;
      scs_sdhead_ts = SDH_WR_ACCEL_CALIB_TS;
      sc1.range = configBytes->wrAccelRange;
      break;
    default:
      scs_infomem_offset = NV_LN_ACCEL_CALIBRATION;
      scs_sdhead_offset = SDH_LN_ACCEL_CALIBRATION;
      scs_sdhead_ts = SDH_LN_ACCEL_CALIB_TS;
      sc1.range = SC_SENSOR_RANGE_ANALOG_ACCEL;
      break;
#elif defined(SHIMMER3R)
    case SC_SENSOR_LSM6DSV_ACCEL:
      scs_infomem_offset = NV_LN_ACCEL_CALIBRATION;
      scs_sdhead_offset = SDH_LN_ACCEL_CALIBRATION;
      scs_sdhead_ts = SDH_LN_ACCEL_CALIB_TS;
      sc1.range = configBytes->lnAccelRange;
      break;
    case SC_SENSOR_LSM6DSV_GYRO:
      scs_infomem_offset = NV_GYRO_CALIBRATION;
      scs_sdhead_offset = SDH_GYRO_CALIBRATION;
      scs_sdhead_ts = SDH_GYRO_CALIB_TS;
      sc1.range = ShimConfig_gyroRangeGet();
      break;
    case SC_SENSOR_LIS2DW12_ACCEL:
      scs_infomem_offset = NV_WR_ACCEL_CALIBRATION;
      scs_sdhead_offset = SDH_WR_ACCEL_CALIBRATION;
      scs_sdhead_ts = SDH_WR_ACCEL_CALIB_TS;
      sc1.range = configBytes->wrAccelRange;
      break;
    case SC_SENSOR_ADXL371_ACCEL:
      scs_infomem_offset = NV_ALT_ACCEL_CALIBRATION;
      scs_sdhead_offset = SDH_ALT_ACCEL_CALIBRATION;
      scs_sdhead_ts = SDH_ALT_ACCEL_CALIB_TS;
      sc1.range = SC_SENSOR_RANGE_ADXL371_RANGE;
      break;
    case SC_SENSOR_LIS2MDL_MAG:
      scs_infomem_offset = NV_MAG_CALIBRATION;
      scs_sdhead_offset = SDH_MAG_CALIBRATION;
      scs_sdhead_ts = SDH_MAG_CALIB_TS;
      sc1.range = SC_SENSOR_RANGE_LIS2MDL_RANGE;
      break;
    case SC_SENSOR_LIS3MDL_MAG:
      scs_infomem_offset = NV_ALT_MAG_CALIBRATION;
      scs_sdhead_offset = SDH_ALT_MAG_CALIBRATION;
      scs_sdhead_ts = SDH_ALT_MAG_CALIB_TS;
      sc1.range = configBytes->altMagRange;
      break;
    default:
      break;
#endif
  }

  ShimCalib_singleSensorRead(&sc1);

  memcpy(&configBytes->rawBytes[scs_infomem_offset], sc1.data.raw, sc1.data_len);
  if (writeToFlash)
  {
    InfoMem_write(scs_infomem_offset, sc1.data.raw, sc1.data_len);
  }

  memcpy(ShimSdHead_getSdHeadText() + scs_sdhead_offset, sc1.data.raw, sc1.data_len);
  memcpy(ShimSdHead_getSdHeadText() + scs_sdhead_ts, sc1.ts, 8);
}
