/*
 * shimmer_sd_header.c
 *
 *  Created on: 27 Feb 2025
 *      Author: MarkNolan
 */

#include <Boards/shimmer_boards.h>
#include <Calibration/shimmer_calibration.h>
#include <Configuration/shimmer_config.h>
#include <SDCard/shimmer_sd_header.h>
#include <log_and_stream_externs.h>

#if defined(SHIMMER3)
#include "../../shimmer_btsd.h"
#include "../BMPX80/bmpX80.h"
#elif defined(SHIMMER3R)
#include "bmp3_defs.h"
#include "shimmer_definitions.h"
#endif

uint8_t sdHeadText[SD_HEAD_SIZE];

void ShimSdHead_reset(void)
{
  memset(sdHeadText, 0xff, SD_HEAD_SIZE);
}

uint8_t *ShimSdHead_getSdHeadText(void)
{
  return sdHeadText;
}

/*
 * sdHeadText: Set(), Get() and GetByte(), S4Ram_sdHeadTextSetByte()
 */
uint8_t ShimSdHead_sdHeadTextSet(const uint8_t *buf, uint16_t offset, uint16_t length)
{
  if ((offset > SD_HEAD_SIZE - 1) || (offset + length > SD_HEAD_SIZE) || (length == 0))
  {
    return 1;
  }
  memcpy(sdHeadText + offset, buf, length);
  return 0;
}

uint8_t ShimSdHead_sdHeadTextGet(uint8_t *buf, uint16_t offset, uint16_t length)
{
  if ((offset > SD_HEAD_SIZE - 1) || (offset + length > SD_HEAD_SIZE) || (length == 0))
  {
    return 1;
  }
  memcpy(buf, sdHeadText + offset, length);
  return 0;
}

uint8_t ShimSdHead_sdHeadTextGetByte(uint16_t offset)
{
  if (offset > SD_HEAD_SIZE - 1)
  {
    return 0;
  }
  return sdHeadText[offset];
}

uint8_t ShimSdHead_sdHeadTextSetByte(uint16_t offset, uint8_t val)
{
  if (offset > SD_HEAD_SIZE - 1)
  {
    return 1;
  }
  sdHeadText[offset] = val;
  return 0;
}

void ShimSdHead_config2SdHead(void)
{
  memset(sdHeadText, 0xff, SD_HEAD_SIZE);

  gConfigBytes *configBytes = ShimConfig_getStoredConfig();

  sdHeadText[SDH_SAMPLE_RATE_0] = configBytes->rawBytes[NV_SAMPLING_RATE];
  sdHeadText[SDH_SAMPLE_RATE_1] = configBytes->rawBytes[NV_SAMPLING_RATE + 1];
  sdHeadText[SDH_BUFFER_SIZE] = configBytes->rawBytes[NV_BUFFER_SIZE];
  sdHeadText[SDH_SENSORS0] = configBytes->rawBytes[NV_SENSORS0];
  sdHeadText[SDH_SENSORS1] = configBytes->rawBytes[NV_SENSORS1];
  sdHeadText[SDH_SENSORS2] = configBytes->rawBytes[NV_SENSORS2];
  sdHeadText[SDH_CONFIG_SETUP_BYTE0] = configBytes->rawBytes[NV_CONFIG_SETUP_BYTE0];
  sdHeadText[SDH_CONFIG_SETUP_BYTE1] = configBytes->rawBytes[NV_CONFIG_SETUP_BYTE1];
  sdHeadText[SDH_CONFIG_SETUP_BYTE2] = configBytes->rawBytes[NV_CONFIG_SETUP_BYTE2];
  sdHeadText[SDH_CONFIG_SETUP_BYTE3] = configBytes->rawBytes[NV_CONFIG_SETUP_BYTE3];
  sdHeadText[SDH_SENSORS3] = configBytes->rawBytes[NV_SENSORS3];
  sdHeadText[SDH_SENSORS4] = configBytes->rawBytes[NV_SENSORS4];
  sdHeadText[SDH_CONFIG_SETUP_BYTE4] = configBytes->rawBytes[NV_CONFIG_SETUP_BYTE4];
  sdHeadText[SDH_CONFIG_SETUP_BYTE5] = configBytes->rawBytes[NV_CONFIG_SETUP_BYTE5];
  sdHeadText[SDH_CONFIG_SETUP_BYTE6] = configBytes->rawBytes[NV_CONFIG_SETUP_BYTE6];
  /* little endian in fw, but they want big endian in sw trivial */
  sdHeadText[SDH_SHIMMERVERSION_BYTE_0] = DEVICE_VER >> 8;
  sdHeadText[SDH_SHIMMERVERSION_BYTE_1] = DEVICE_VER & 0xff;
  sdHeadText[SDH_FW_VERSION_TYPE_0] = FW_IDENTIFIER >> 8;
  sdHeadText[SDH_FW_VERSION_TYPE_1] = FW_IDENTIFIER & 0xff;
  sdHeadText[SDH_FW_VERSION_MAJOR_0] = FW_VER_MAJOR >> 8;
  sdHeadText[SDH_FW_VERSION_MAJOR_1] = FW_VER_MAJOR & 0xff;
  sdHeadText[SDH_FW_VERSION_MINOR] = FW_VER_MINOR;
  sdHeadText[SDH_FW_VERSION_INTERNAL] = FW_VER_REL;
  /* exg */
  sdHeadText[SDH_EXG_ADS1292R_1_CONFIG1] = configBytes->rawBytes[NV_EXG_ADS1292R_1_CONFIG1];
  sdHeadText[SDH_EXG_ADS1292R_1_CONFIG2] = configBytes->rawBytes[NV_EXG_ADS1292R_1_CONFIG2];
  sdHeadText[SDH_EXG_ADS1292R_1_LOFF] = configBytes->rawBytes[NV_EXG_ADS1292R_1_LOFF];
  sdHeadText[SDH_EXG_ADS1292R_1_CH1SET] = configBytes->rawBytes[NV_EXG_ADS1292R_1_CH1SET];
  sdHeadText[SDH_EXG_ADS1292R_1_CH2SET] = configBytes->rawBytes[NV_EXG_ADS1292R_1_CH2SET];
  sdHeadText[SDH_EXG_ADS1292R_1_RLD_SENS]
      = configBytes->rawBytes[NV_EXG_ADS1292R_1_RLD_SENS];
  sdHeadText[SDH_EXG_ADS1292R_1_LOFF_SENS]
      = configBytes->rawBytes[NV_EXG_ADS1292R_1_LOFF_SENS];
  sdHeadText[SDH_EXG_ADS1292R_1_LOFF_STAT]
      = configBytes->rawBytes[NV_EXG_ADS1292R_1_LOFF_STAT];
  sdHeadText[SDH_EXG_ADS1292R_1_RESP1] = configBytes->rawBytes[NV_EXG_ADS1292R_1_RESP1];
  sdHeadText[SDH_EXG_ADS1292R_1_RESP2] = configBytes->rawBytes[NV_EXG_ADS1292R_1_RESP2];
  sdHeadText[SDH_EXG_ADS1292R_2_CONFIG1] = configBytes->rawBytes[NV_EXG_ADS1292R_2_CONFIG1];
  sdHeadText[SDH_EXG_ADS1292R_2_CONFIG2] = configBytes->rawBytes[NV_EXG_ADS1292R_2_CONFIG2];
  sdHeadText[SDH_EXG_ADS1292R_2_LOFF] = configBytes->rawBytes[NV_EXG_ADS1292R_2_LOFF];
  sdHeadText[SDH_EXG_ADS1292R_2_CH1SET] = configBytes->rawBytes[NV_EXG_ADS1292R_2_CH1SET];
  sdHeadText[SDH_EXG_ADS1292R_2_CH2SET] = configBytes->rawBytes[NV_EXG_ADS1292R_2_CH2SET];
  sdHeadText[SDH_EXG_ADS1292R_2_RLD_SENS]
      = configBytes->rawBytes[NV_EXG_ADS1292R_2_RLD_SENS];
  sdHeadText[SDH_EXG_ADS1292R_2_LOFF_SENS]
      = configBytes->rawBytes[NV_EXG_ADS1292R_2_LOFF_SENS];
  sdHeadText[SDH_EXG_ADS1292R_2_LOFF_STAT]
      = configBytes->rawBytes[NV_EXG_ADS1292R_2_LOFF_STAT];
  sdHeadText[SDH_EXG_ADS1292R_2_RESP1] = configBytes->rawBytes[NV_EXG_ADS1292R_2_RESP1];
  sdHeadText[SDH_EXG_ADS1292R_2_RESP2] = configBytes->rawBytes[NV_EXG_ADS1292R_2_RESP2];

  sdHeadText[SDH_BT_COMMS_BAUD_RATE] = configBytes->rawBytes[NV_BT_COMMS_BAUD_RATE];
  sdHeadText[SDH_DERIVED_CHANNELS_0] = configBytes->rawBytes[NV_DERIVED_CHANNELS_0];
  sdHeadText[SDH_DERIVED_CHANNELS_1] = configBytes->rawBytes[NV_DERIVED_CHANNELS_1];
  sdHeadText[SDH_DERIVED_CHANNELS_2] = configBytes->rawBytes[NV_DERIVED_CHANNELS_2];
  sdHeadText[SDH_DERIVED_CHANNELS_3] = configBytes->rawBytes[NV_DERIVED_CHANNELS_3];
  sdHeadText[SDH_DERIVED_CHANNELS_4] = configBytes->rawBytes[NV_DERIVED_CHANNELS_4];
  sdHeadText[SDH_DERIVED_CHANNELS_5] = configBytes->rawBytes[NV_DERIVED_CHANNELS_5];
  sdHeadText[SDH_DERIVED_CHANNELS_6] = configBytes->rawBytes[NV_DERIVED_CHANNELS_6];
  sdHeadText[SDH_DERIVED_CHANNELS_7] = configBytes->rawBytes[NV_DERIVED_CHANNELS_7];

  /* sd config */
  sdHeadText[SDH_MYTRIAL_ID] = configBytes->rawBytes[NV_SD_MYTRIAL_ID];
  sdHeadText[SDH_NSHIMMER] = configBytes->rawBytes[NV_SD_NSHIMMER];
  sdHeadText[SDH_EST_EXP_LEN_MSB] = configBytes->rawBytes[NV_EST_EXP_LEN_MSB];
  sdHeadText[SDH_EST_EXP_LEN_LSB] = configBytes->rawBytes[NV_EST_EXP_LEN_LSB];
  sdHeadText[SDH_MAX_EXP_LEN_MSB] = configBytes->rawBytes[NV_MAX_EXP_LEN_MSB];
  sdHeadText[SDH_MAX_EXP_LEN_LSB] = configBytes->rawBytes[NV_MAX_EXP_LEN_LSB];
  sdHeadText[SDH_TRIAL_CONFIG0] = configBytes->rawBytes[NV_SD_TRIAL_CONFIG0];
  sdHeadText[SDH_TRIAL_CONFIG1] = configBytes->rawBytes[NV_SD_TRIAL_CONFIG1];
  sdHeadText[SDH_BROADCAST_INTERVAL] = configBytes->rawBytes[NV_SD_BT_INTERVAL];

#if defined(SHIMMER3)
  uint64_t *rwcTimeDiffPtr = getRwcTimeDiffPtr();
  sdHeadText[SDH_RTC_DIFF_7] = *((uint8_t *) rwcTimeDiffPtr);
  sdHeadText[SDH_RTC_DIFF_6] = *(((uint8_t *) rwcTimeDiffPtr) + 1);
  sdHeadText[SDH_RTC_DIFF_5] = *(((uint8_t *) rwcTimeDiffPtr) + 2);
  sdHeadText[SDH_RTC_DIFF_4] = *(((uint8_t *) rwcTimeDiffPtr) + 3);
  sdHeadText[SDH_RTC_DIFF_3] = *(((uint8_t *) rwcTimeDiffPtr) + 4);
  sdHeadText[SDH_RTC_DIFF_2] = *(((uint8_t *) rwcTimeDiffPtr) + 5);
  sdHeadText[SDH_RTC_DIFF_1] = *(((uint8_t *) rwcTimeDiffPtr) + 6);
  sdHeadText[SDH_RTC_DIFF_0] = *(((uint8_t *) rwcTimeDiffPtr) + 7);
#else
  sdHeadText[SDH_RTC_DIFF_7] = 0;
  sdHeadText[SDH_RTC_DIFF_6] = 0;
  sdHeadText[SDH_RTC_DIFF_5] = 0;
  sdHeadText[SDH_RTC_DIFF_4] = 0;
  sdHeadText[SDH_RTC_DIFF_3] = 0;
  sdHeadText[SDH_RTC_DIFF_2] = 0;
  sdHeadText[SDH_RTC_DIFF_1] = 0;
  sdHeadText[SDH_RTC_DIFF_0] = 0;
#endif

  memcpy(&sdHeadText[SDH_MAC_ADDR], &configBytes->rawBytes[NV_MAC_ADDRESS], 6);
  memcpy(&sdHeadText[SDH_CONFIG_TIME_0], &configBytes->rawBytes[NV_SD_CONFIG_TIME], 4);
  ShimSdHead_saveBmpCalibrationToSdHeader();

  ShimCalib_calibDumpToConfigBytesAndSdHeaderAll();
  memcpy(&sdHeadText[SDH_DAUGHTER_CARD_ID_BYTE0], ShimBrd_getDaughtCardIdPtr(), 3);
}

void ShimSdHead_saveBmpCalibrationToSdHeader(void)
{
  uint8_t *bmpCalibPtr = get_bmp_calib_data_bytes();
#if defined(SHIMMER3)
  /* BMP180 had 22 bytes stored in index SDH_TEMP_PRES_CALIBRATION. BMP280 had
   * 24 bytes spread across the 22 available bytes in SDH_TEMP_PRES_CALIBRATION
   * and a further 2 bytes in BMP280_XTRA_CALIB_BYTES. */
  memcpy(&sdHeadText[SDH_TEMP_PRES_CALIBRATION], bmpCalibPtr, BMP180_CALIB_DATA_SIZE);
  if (isBmp280InUse())
  {
    memcpy(&sdHeadText[SDH_TEMP_PRES_EXTRA_CALIB_BYTES],
        bmpCalibPtr + BMP180_CALIB_DATA_SIZE, BMP280_CALIB_XTRA_BYTES);
  }
#elif defined(SHIMMER3)
  /* BMP390 had 21 bytes stored in index SDH_TEMP_PRES_CALIBRATION */
  memcpy(&sdHeadText[SDH_TEMP_PRES_CALIBRATION], bmpCalibPtr, BMP3_LEN_CALIB_DATA);
#endif
}
