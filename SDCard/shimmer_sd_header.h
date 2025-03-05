/*
 * shimmer_sd_header.h
 *
 *  Created on: 27 Feb 2025
 *      Author: MarkNolan
 */

#ifndef LOG_AND_STREAM_COMMON_SDCARD_SHIMMER_SD_HEADER_H_
#define LOG_AND_STREAM_COMMON_SDCARD_SHIMMER_SD_HEADER_H_

#include <stdint.h>

//SD Log file header format
#if defined(SHIMMER3)
#define SD_HEAD_SIZE                256// 0-255
#elif defined(SHIMMER3R)
#define SD_HEAD_SIZE                    384
#endif

#define SDH_SAMPLE_RATE_0           0
#define SDH_SAMPLE_RATE_1           1
#define SDH_BUFFER_SIZE             2
#define SDH_SENSORS0                3
#define SDH_SENSORS1                4
#define SDH_SENSORS2                5
#define SDH_SENSORS3                6
#define SDH_SENSORS4                7
#define SDH_CONFIG_SETUP_BYTE0      8//sensors setting bytes
#define SDH_CONFIG_SETUP_BYTE1      9
#define SDH_CONFIG_SETUP_BYTE2      10
#define SDH_CONFIG_SETUP_BYTE3      11
#define SDH_CONFIG_SETUP_BYTE4      12
#define SDH_CONFIG_SETUP_BYTE5      13
#define SDH_CONFIG_SETUP_BYTE6      14
#define SDH_TRIAL_CONFIG0           16
#define SDH_TRIAL_CONFIG1           17
#define SDH_BROADCAST_INTERVAL      18
#define SDH_BT_COMMS_BAUD_RATE      19
#define SDH_EST_EXP_LEN_MSB         20
#define SDH_EST_EXP_LEN_LSB         21
#define SDH_MAX_EXP_LEN_MSB         22
#define SDH_MAX_EXP_LEN_LSB         23
#define SDH_MAC_ADDR                24
#define SDH_SHIMMERVERSION_BYTE_0   30
#define SDH_SHIMMERVERSION_BYTE_1   31
#define SDH_MYTRIAL_ID              32
#define SDH_NSHIMMER                33
#define SDH_FW_VERSION_TYPE_0       34
#define SDH_FW_VERSION_TYPE_1       35
#define SDH_FW_VERSION_MAJOR_0      36
#define SDH_FW_VERSION_MAJOR_1      37
#define SDH_FW_VERSION_MINOR        38
#define SDH_FW_VERSION_INTERNAL     39
#define SDH_DERIVED_CHANNELS_0      40
#define SDH_DERIVED_CHANNELS_1      41
#define SDH_DERIVED_CHANNELS_2      42
#define SDH_RTC_DIFF_0              44
#define SDH_RTC_DIFF_1              45
#define SDH_RTC_DIFF_2              46
#define SDH_RTC_DIFF_3              47
#define SDH_RTC_DIFF_4              48
#define SDH_RTC_DIFF_5              49
#define SDH_RTC_DIFF_6              50
#define SDH_RTC_DIFF_7              51
#define SDH_CONFIG_TIME_0           52
#define SDH_CONFIG_TIME_1           53
#define SDH_CONFIG_TIME_2           54
#define SDH_CONFIG_TIME_3           55
#define SDH_EXG_ADS1292R_1_CONFIG1        56
#define SDH_EXG_ADS1292R_1_CONFIG2        57
#define SDH_EXG_ADS1292R_1_LOFF           58
#define SDH_EXG_ADS1292R_1_CH1SET         59
#define SDH_EXG_ADS1292R_1_CH2SET         60
#define SDH_EXG_ADS1292R_1_RLD_SENS       61
#define SDH_EXG_ADS1292R_1_LOFF_SENS      62
#define SDH_EXG_ADS1292R_1_LOFF_STAT      63
#define SDH_EXG_ADS1292R_1_RESP1          64
#define SDH_EXG_ADS1292R_1_RESP2          65
#define SDH_EXG_ADS1292R_2_CONFIG1        66
#define SDH_EXG_ADS1292R_2_CONFIG2        67
#define SDH_EXG_ADS1292R_2_LOFF           68
#define SDH_EXG_ADS1292R_2_CH1SET         69
#define SDH_EXG_ADS1292R_2_CH2SET         70
#define SDH_EXG_ADS1292R_2_RLD_SENS       71
#define SDH_EXG_ADS1292R_2_LOFF_SENS      72
#define SDH_EXG_ADS1292R_2_LOFF_STAT      73
#define SDH_EXG_ADS1292R_2_RESP1          74
#define SDH_EXG_ADS1292R_2_RESP2          75
#define SDH_WR_ACCEL_CALIBRATION          76 //0x4c
#define SDH_GYRO_CALIBRATION              97 //0x61
#define SDH_MAG_CALIBRATION               118 //0x76
#define SDH_LN_ACCEL_CALIBRATION          139 //0x8b
#define SDH_TEMP_PRES_CALIBRATION         160
#define SDH_WR_ACCEL_CALIB_TS             182 //+8
#define SDH_GYRO_CALIB_TS                 190 //+8
#define SDH_MAG_CALIB_TS                  198 //+8
#define SDH_LN_ACCEL_CALIB_TS             206 //+8
#define SDH_DAUGHTER_CARD_ID_BYTE0        214 //+3
#define SDH_DERIVED_CHANNELS_3            217
#define SDH_DERIVED_CHANNELS_4            218
#define SDH_DERIVED_CHANNELS_5            219
#define SDH_DERIVED_CHANNELS_6            220
#define SDH_DERIVED_CHANNELS_7            221
#define SDH_TEMP_PRES_EXTRA_CALIB_BYTES   222
//#define SDH_MY_LOCALTIME_0TH             248
#define SDH_MY_LOCALTIME_5TH              251
#define SDH_MY_LOCALTIME                  252  //252-255
#define SDH_ALT_ACCEL_CALIBRATION       256 //+21
#define SDH_ALT_ACCEL_CALIB_TS          277 //+8
#define SDH_ALT_MAG_CALIBRATION         285 //+21
#define SDH_ALT_MAG_CALIB_TS            306 //+8

#if defined(SHIMMER3)
//SENSORS0
#define SDH_SENSOR_A_ACCEL             0x80
#define SDH_SENSOR_MPU9X50_GYRO        0x40
#define SDH_SENSOR_LSM303_MAG          0x20
#define SDH_SENSOR_EXG1_24BIT          0x10
#define SDH_SENSOR_EXG2_24BIT          0x08
#define SDH_SENSOR_GSR                 0x04
#define SDH_SENSOR_EXTCH7              0x02
#define SDH_SENSOR_EXTCH6              0x01
//SENSORS1
#define SDH_SENSOR_STRAIN              0x80
//#define SDH_SENSOR_HR                0x40
#define SDH_SENSOR_VBATT               0x20
#define SDH_SENSOR_LSM303DLHC_ACCEL    0x10
#define SDH_SENSOR_EXTCH15             0x08
#define SDH_SENSOR_INTCH1              0x04
#define SDH_SENSOR_INTCH12             0x02
#define SDH_SENSOR_INTCH13             0x01
//SENSORS2
#define SDH_SENSOR_INTCH14             0x80
#define SDH_SENSOR_MPU9X50_ACCEL       0x40
#define SDH_SENSOR_MPU9X50_MAG         0x20
#define SDH_SENSOR_EXG1_16BIT          0x10
#define SDH_SENSOR_EXG2_16BIT          0x08
#define SDH_SENSOR_BMPX80_PRES         0x04
//#define SDH_SENSOR_BMPX80_TEMP       0x02
//SENSORS3
#define SDH_SENSOR_MSP430_TEMP         0x01
#define SDH_SENSOR_TCXO                0x80

//SDH_TRIAL_CONFIG0
#define SDH_SDERROR_EN                 0x01
#endif
#define SDH_IAMMASTER                  0x02
#if defined(SHIMMER3)
#define SDH_TIME_SYNC                  0x04
//#define SDH_TIME_STAMP                 0x08// not used now, reserved as 1
#define SDH_BLUETOOTH_DISABLE          0x08
#define SDH_RWCERROR_EN                0x10// when 0, won't flash error. when 1, will flash error if RTC isn't set RTC_offset == 0
#define SDH_USER_BUTTON_ENABLE         0x20
#define SDH_SET_PMUX                   0x40// not used now, reserved as 0
#define SDH_RTC_SET_BY_BT              0x80

//SDH_TRIAL_CONFIG1
#define SDH_BATT_CRITICAL_CUTOFF       0x01
#endif
#define SDH_TCXO                       0x10
#if defined(SHIMMER3)
#define SDH_SINGLETOUCH                0x80
#endif

void ShimSdHead_reset(void);
uint8_t *S4Ram_getSdHeadText(void);

uint8_t S4Ram_sdHeadTextSet(const uint8_t *buf, uint16_t offset, uint16_t length);
uint8_t S4Ram_sdHeadTextGet(uint8_t *buf, uint16_t offset, uint16_t length);
uint8_t S4Ram_sdHeadTextGetByte(uint16_t offset);
uint8_t S4Ram_sdHeadTextSetByte(uint16_t offset, uint8_t val);

void ShimSdHead_config2SdHead(void);

void saveBmpCalibrationToSdHeader(void);

#endif /* LOG_AND_STREAM_COMMON_SDCARD_SHIMMER_SD_HEADER_H_ */
