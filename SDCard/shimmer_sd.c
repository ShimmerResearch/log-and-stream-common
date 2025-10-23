/*
 * shimmer_sd.c
 *
 *  Created on: 23 Oct 2025
 *      Author: MarkNolan
 */

#include "shimmer_sd.h"

#include <stdint.h>
#include <string.h>

#include "log_and_stream_externs.h"

#if defined(SHIMMER3)
#include "ff.h"

#include "../5xx_HAL/hal_RTC.h"
#include "intrinsics.h"

#elif defined(SHIMMER3R)
#include "main.h"
#include "shimmer_definitions.h"
#include "shimmer_include.h"
#include "stm32u5xx_hal.h"

#if USE_FATFS
#include "fatfs.h"
#include "ff.h"
#else
#include "fx_api.h"
#endif
#endif

#define TEST_TEXT_LEN 40

FATFS fatfs; //File object

void ShimSd_init(void)
{
  memset(&fatfs, 0x00, sizeof(fatfs));
}

uint8_t ShimSd_test1(void)
{
#if USE_FATFS
  FIL test_file;
#endif
  char file_name[] = "test1.txt";
  char test_text1[TEST_TEXT_LEN] = "This is the 1st line of the test file.\n";
  char test_text2[TEST_TEXT_LEN] = "This is the 2nd line of the test file.\n";
  char test_text3[TEST_TEXT_LEN];
#if USE_FATFS
  UINT bw;
#endif

#if USE_FATFS
  shimmerStatus.sdBadFile += f_open(&test_file, file_name, FA_CREATE_ALWAYS | FA_WRITE);
  shimmerStatus.sdBadFile += f_write(&test_file, test_text1, TEST_TEXT_LEN - 1, &bw);
  shimmerStatus.sdBadFile += f_write(&test_file, test_text2, TEST_TEXT_LEN - 1, &bw);
  shimmerStatus.sdBadFile += f_close(&test_file);

  memset(test_text3, 0, 40);
  shimmerStatus.sdBadFile += f_open(&test_file, file_name, FA_OPEN_EXISTING | FA_READ);
  shimmerStatus.sdBadFile += f_read(&test_file, test_text3, TEST_TEXT_LEN - 1, &bw);
  shimmerStatus.sdBadFile += f_close(&test_file);
  f_unlink(file_name);
#endif

  shimmerStatus.sdBadFile += strcmp(test_text1, test_text3);

  return shimmerStatus.sdBadFile;
}

uint8_t ShimSd_test2(void)
{
  FRESULT res = FR_OK; /* FatFs function common result code */
#if USE_FATFS
  uint32_t byteswritten; //, bytesread; /* File write/read counts */
  uint8_t wtext[] = "FATFS works great!"; /* File write buffer */
#if _USE_MKFS
  uint8_t rtext[_MAX_SS]; /* File read buffer */
#endif
  FIL SDFile;

  if (ShimSd_mount(SD_MOUNT) != FR_OK)
  {
    shimmerStatus.sdBadFile = 1;
  }
  else
  {
#if _USE_MKFS
    if (f_mkfs((TCHAR const *) SDPath, FM_ANY, 0, rtext, sizeof(rtext)) != FR_OK)
    {
      Error_Handler();
    }
    else
    {
#endif
      //Open file for writing (Create)
      if (f_open(&SDFile, "test2.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
      {
        shimmerStatus.sdBadFile = 1;
      }
      else
      {

        //Write to the text file
        res = f_write(&SDFile, wtext, strlen((char *) wtext), (void *) &byteswritten);
        if ((byteswritten == 0) || (res != FR_OK))
        {
          shimmerStatus.sdBadFile = 1;
        }
        else
        {
          f_close(&SDFile);
        }
      }
#if _USE_MKFS
    }
#endif
  }
  ShimSd_mount(SD_UNMOUNT);
#endif
  return res;
}

FRESULT ShimSd_mount(sd_mount_state_t val)
{
  FRESULT result;
  if (val == SD_MOUNT)
  {
#if _FATFS == FATFS_V_0_08B
    result = f_mount(0, &fatfs);
#elif _FATFS == FATFS_V_0_12C
    result = f_mount(&fatfs, (TCHAR const *) SDPath, 0);
#endif
  }
  else
  {
#if _FATFS == FATFS_V_0_08B
    f_mount(0, NULL);
#elif _FATFS == FATFS_V_0_12C
    f_mount(&fatfs, (TCHAR const *) NULL, 0);
#endif
  }

#if _FATFS == FATFS_V_0_08B
  /*TODO not sure if Shimmer added this function */
  set_sd_detect(val);
#endif
  return result;
}

FRESULT ShimSd_setFileTimestamp(char *path)
{
#if defined(SHIMMER3R)
  FILINFO fno;
  RTC_TimeTypeDef RTC_TimeStructure;
  RTC_DateTypeDef RTC_DateStructure;

  //TODO figure out best system below and delete the unused section
  //HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_Format_BIN);
  //HAL_RTC_GetDate(&hrtc, &RTC_DateStructure, RTC_Format_BIN);
  //int hour = RTC_TimeStructure.RTC_Hours;
  //int min = RTC_TimeStructure.RTC_Minutes;
  //int sec = RTC_TimeStructure.RTC_Seconds;
  //int month = RTC_DateStructure.RTC_Month;
  //int mday = RTC_DateStructure.RTC_Date;
  //int year = RTC_DateStructure.RTC_Year;
  //year += 2000;
  //fno.fdate = (WORD) (((year - 1980) << 9) | month << 5 | mday);
  //fno.ftime = (WORD) (hour << 11 | min << 5 | sec / 2);

  SHIM_RTC_t data;
  RTC_getDateTime(&data);
  data.year += 2000;
  fno.fdate = (WORD) (((data.year - 1980) << 9) | data.month << 5 | data.date);
  fno.ftime = (WORD) (data.hours << 11 | data.minutes << 5 | data.seconds / 2);

  return f_utime(path, &fno);
#else
  return FR_OK;
#endif
}

void ShimSd_findError(uint8_t err, uint8_t *name)
{
  switch (err)
  {
    case 0:
      strcpy((char *) name, "OK");
      break;
    case 1:
      strcpy((char *) name, "DISK_ERR");
      break;
    case 2:
      strcpy((char *) name, "INT_ERR");
      break;
    case 3:
      strcpy((char *) name, "NOT_READY");
      break;
    case 4:
      strcpy((char *) name, "NO_FILE");
      break;
    case 5:
      strcpy((char *) name, "NO_PATH");
      break;
    case 6:
      strcpy((char *) name, "INVALID_NAME");
      break;
    case 7:
      strcpy((char *) name, "DENIED");
      break;
    case 8:
      strcpy((char *) name, "EXIST");
      break;
    case 9:
      strcpy((char *) name, "INVALID_OBJ");
      break;
    case 10:
      strcpy((char *) name, "WRITE_PROTEC");
      break;
    case 11:
      strcpy((char *) name, "INVALID_DRIV");
      break;
    case 12:
      strcpy((char *) name, "NOT_ENABLED");
      break;
    case 13:
      strcpy((char *) name, "NO_FILESYSTE");
      break;
    case 14:
      strcpy((char *) name, "MKFS_ABORTED");
      break;
    case 15:
      strcpy((char *) name, "TIMEOUT");
      break;
    case 16:
      strcpy((char *) name, "LOCKED");
      break;
    case 17:
      strcpy((char *) name, "NOT_ENOUGH_C");
      break;
    case 18:
      strcpy((char *) name, "TOO_MANY_OPE");
      break;
    default:
      strcpy((char *) name, "NO_REASON");
      break;
  } //FRESULT;
}
