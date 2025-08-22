/*
 * s4sd.cpp
 *
 *  Created on: Mar 21, 2024
 *      Author: MarkNolan
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <log_and_stream_externs.h>
#include <log_and_stream_includes.h>

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

#if defined(SHIMMER3)
#define assert_param(expr) ((void) 0U)
#endif

void ShimSd_openNewDataFile(void);
void ShimSd_writeSdHeaderToFile(void);
void ShimSd_closeDataFile(void);

/* Two-dimensional SD write buffer */
uint8_t sdWrBuf[NUM_SDWRBUF][SD_WRITE_BUF_SIZE];
/* Current sensing buffer index in-which new data is being saved into */
uint8_t sdBufSens = 0;
/* Current write buffer index that is being written to the SD card */
uint8_t sdBufWr = 0;
/* Length of data within each buffer */
uint16_t sdWrLen[NUM_SDWRBUF];

uint8_t fileName[64], dirName[64], expDirName[32], sdBufInQ;
uint16_t fileNum, dirCounter;
uint64_t sdFileCrTs, sdFileSyncTs;
#if USE_FATFS
FRESULT file_status;
FATFS fatfs; //File object
#if _FATFS == FATFS_V_0_08B
DIRS dir; //Directory object
#elif _FATFS == FATFS_V_0_12C
DIR dir; //Directory object
#endif
FIL dataFile;
FILINFO dataFileInfo;
char dataFileName[256];

#if _FATFS == FATFS_V_0_12C
extern uint8_t retSD;  /* Return value for SD */
extern char SDPath[4]; /* SD logical drive path */
extern FATFS SDFatFS;  /* File system object for SD logical drive */
extern FIL SDFile;     /* File object for SD */
#endif

#endif

void ShimSdDataFile_init(void)
{
  sensing.isFileCreated = 0;
  sensing.inSdWr = 0;
  sensing.inSdWrCnt = 0;

  memset(&fileName[0], 0x00, sizeof(fileName));
  memset(&dirName[0], 0x00, sizeof(dirName));
  memset(&expDirName[0], 0x00, sizeof(expDirName));
  memset(&sdWrBuf[0][0], 0x00, sizeof(sdWrBuf));
  sdBufInQ = 0;

  sdBufSens = 0;
  sdBufWr = 0;
  fileNum = 0;
  dirCounter = 0;
  memset(&sdWrLen[0], 0x00, sizeof(sdWrLen));
  sdFileCrTs = 0;
  sdFileSyncTs = 0;
  file_status = FR_OK;
  memset(&fatfs, 0x00, sizeof(fatfs));
  memset(&dir, 0x00, sizeof(dir));
  memset(&dataFile, 0x00, sizeof(dataFile));
  memset(&dataFileInfo, 0x00, sizeof(dataFileInfo));
  memset(&dataFileName[0], 0x00, sizeof(dataFileName));

  ShimSd_setDataFileNameIfEmpty();
}

#define TEST_TEXT_LEN 40

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

  if (ShimSd_mount(1) != FR_OK)
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
  ShimSd_mount(0);
#endif
  return res;
}

FRESULT ShimSd_mount(uint8_t val)
{
  FRESULT result;
  if (1 == val)
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

void ShimSd_setDataFileNameIfEmpty(void)
{
  if (strlen((char *) fileName) == 0)
  {
    strcpy((char *) fileName, "no_file   ");
  }
}

uint8_t ShimSdDataFile_setBasedir(void)
{
#if USE_FATFS
  FILINFO fno;
  //volatile uint8_t res;
  uint16_t tmp_counter = 0;
  char lfn[_MAX_LFN + 1] = { 0 }, *fname, *scout, *dash, dirnum[8];

#if _FATFS == FATFS_V_0_08B
  fno.lfname = lfn;
  fno.lfsize = sizeof(lfn);
#endif

  file_status = f_opendir(&dir, "/data");
  if (file_status)
  {
    if (file_status == FR_NO_PATH) //we'll have to make /data first
    {
      file_status = f_mkdir("/data");
    }
    if (file_status) //in every case, we're toast
    {
      return 0; //FAIL;
    }

    //try one more time
    file_status = f_opendir(&dir, "/data");
    if (file_status)
    {
      return 0; //FAIL;
    }
  }
#if _FATFS != FATFS_V_0_08B
  file_status = f_closedir(&dir);
  ShimSd_setFileTimestamp("/data");
#endif

  strcpy((char *) expDirName, "data/");
  strcat((char *) expDirName, ShimConfig_expIdParseToTxtAndPtrGet());
  strcat((char *) expDirName, "_");
  strcat((char *) expDirName, ShimConfig_configTimeParseToTxtAndPtrGet());

  file_status = f_opendir(&dir, (char *) expDirName);
  if (file_status)
  {
    if (file_status == FR_NO_PATH) //we'll have to make the experiment folder first
    {
      file_status = f_mkdir((char *) expDirName);
    }
    if (file_status) //in every case, we're toast
    {
      return 0; //FAIL;
    }

    //try one more time
    file_status = f_opendir(&dir, (char *) expDirName);
    if (file_status)
    {
      return 0; //FAIL;
    }
  }

  dirCounter = 0; //this might be the first log for this shimmer

  //file name format
  //shimmername    as defined in sdlog.cfg
  //-              separator
  //000
  //we want to create a new directory with a sequential run number each power-up/reset for each shimmer
  while (f_readdir(&dir, &fno) == FR_OK)
  {
    if (*fno.fname == 0)
    {
      break;
    }
    else if (fno.fattrib & AM_DIR)
    {
#if _FATFS == FATFS_V_0_08B
      fname = (*fno.lfname) ? fno.lfname : fno.fname;
#elif _FATFS == FATFS_V_0_12C
      fname = (*lfn) ? lfn : fno.fname;
#endif

      if (!strncmp(fname, ShimConfig_shimmerNameParseToTxtAndPtrGet(), strlen(fname) - 4))
      { //-4 because of the -000 etc.
        scout = strchr(fname, '-');
        if (scout)
        { //if not, something is seriously wrong!
          scout++;
          dash = strchr(scout, '-');
          while (dash)
          { //In case the shimmer name contains '-'
            scout = dash + 1;
            dash = strchr(scout, '-');
          }
          strcpy(dirnum, scout);
          tmp_counter = atoi(dirnum);
          if (tmp_counter >= dirCounter)
          {
            dirCounter = tmp_counter;
            dirCounter++; //start with next in numerical sequence
          }
        }
        else
        {
          return 0; //FAIL;
        }
      }
    }
  }
#if _FATFS != FATFS_V_0_08B
  file_status = f_closedir(&dir);
  ShimSd_setFileTimestamp((char *) expDirName);
#endif

  //at this point, we have the id string and the counter, so we can make a directory name
  return 1; //SUCCESS;
#endif
}

uint8_t ShimSdDataFile_makeBasedir(void)
{
  memset(dirName, 0, 64);

  char dir_counter_text[4];
  ShimUtil_ItoaWith0((uint64_t) dirCounter, (uint8_t *) dir_counter_text, 4);

  strcpy((char *) dirName, (char *) expDirName);
  strcat((char *) dirName, "/");
  strcat((char *) dirName, ShimConfig_shimmerNameParseToTxtAndPtrGet());
  strcat((char *) dirName, "-");
  strcat((char *) dirName, dir_counter_text);

#if USE_FATFS
  file_status = f_mkdir((char *) dirName);
  if (file_status)
  {
    ShimSd_findError(file_status, dirName);
    return 0; //FAIL;
  }

#endif

  memset(fileName, 0, 64);
  strcpy((char *) fileName, (char *) dirName);
  strcat((char *) fileName, "/000");
  fileNum = 0;
  //sprintf((char*)fileName, "/%03d", fileNum++);

  ShimSd_setFileTimestamp((char *) dirName);

  return 1; //SUCCESS;
}

void ShimSdDataFile_makeFileName(char *name_buf)
{
  //strcpy(file_name, "this_is_a_very_long_name_for_the_dataFile");
  uint8_t temp_str[7];
  sprintf((char *) temp_str, "/%03d", fileNum++);

  strcpy((char *) name_buf, (char *) dirName);
  strcat((char *) name_buf, (char *) temp_str);
}

void ShimSdDataFile_fileInit(void)
{
  if (!shimmerStatus.sdPowerOn)
  {
    Board_setSdPower(1);
  }

  sensing.isSdOperating = 1;
  ShimSdDataFile_setBasedir();
  ShimSdDataFile_makeBasedir();
  ShimSdHead_config2SdHead();

  ShimSd_openNewDataFile();

  sdFileSyncTs = sdFileCrTs = RTC_get64();

  ShimSd_writeSdHeaderToFile();

  sensing.isSdOperating = 0;
  sensing.isFileCreated = 1;
  memset(sdWrLen, 0, sizeof(sdWrLen));
  sdBufInQ = sdBufSens = sdBufWr = 0;
}

void ShimSdDataFile_close(void)
{
  ShimSd_closeDataFile();
  //Board_sdPower(0);
  shimmerStatus.sdBadFile = 0;
  sensing.isFileCreated = 0;
}

void ShimSdDataFile_writeToBuff(uint8_t *buf, uint16_t len)
{
  //uint8_t *sensing_buf;
  //uint16_t *sensing_buf_len;
  if ((NUM_SDWRBUF == sdBufInQ) || (sensing.isFileCreated == 0))
  {
    __NOP();
    return;
  }
  //sdWrBuf[NUM_SDWRBUF][SD_WRITE_BUF_SIZE], sdBufSens, sdBufWr, sdBufInQ;

  /* If enabled, write the sync offset to the start of the buffer */
  if (sdWrLen[sdBufSens] == 0 && shimmerStatus.sdSyncEnabled)
  {
    PrepareSDBuffHead();
  }

  memcpy(sdWrBuf[sdBufSens] + sdWrLen[sdBufSens], buf, len);
  sdWrLen[sdBufSens] += len;
  if (sdWrLen[sdBufSens] + len > SD_WRITE_BUF_SIZE)
  {
    ShimTask_set(TASK_SDWRITE);
    sdBufInQ++;
    sdBufSens++;
    if (sdBufSens >= NUM_SDWRBUF)
    {
      sdBufSens = 0;
    }
  }
}

void ShimSdDataFile_writeToCard(void)
{
#if USE_FATFS
  UINT bw;
#endif //USE_FATFS
  uint8_t *writing_buf;
  uint16_t *writing_buf_len;

  writing_buf = sdWrBuf[sdBufWr];
  writing_buf_len = sdWrLen + sdBufWr;

  __NOP();

  if ((0 == *writing_buf_len) || (0 == sdBufInQ))
  {
    return;
  }

  sensing.inSdWr = 1;
  sensing.isSdOperating = 1;

#if USE_FATFS
  //dataFileInfo.fsize was not incrementing the file size.
  file_status = f_lseek(&dataFile, f_size(&dataFile));
  assert_param(file_status == FR_OK);
  file_status = f_write(&dataFile, writing_buf, *writing_buf_len, &bw);
  assert_param(file_status == FR_OK);
#endif

  __NOP();
  __NOP();

  /* split file every hour upwards from 000 */
  if ((sensing.latestTs - sdFileCrTs) >= BIN_FILE_SPLIT_TIME_TICKS)
  {
    sdFileSyncTs = sdFileCrTs = RTC_get64();

    ShimSd_closeDataFile();
    ShimSd_openNewDataFile();
    ShimSd_writeSdHeaderToFile();
  }

  /* Sync file every minute */
  else if (sensing.latestTs - sdFileSyncTs >= BIN_FILE_SYNC_TIME_TICKS)
  {
#if USE_FATFS
    file_status = f_sync(&dataFile);
    assert_param(file_status == FR_OK);
#endif
    sdFileSyncTs = RTC_get64();
  }

  sensing.isSdOperating = 0;

  *writing_buf_len = 0;
  //sdBufWr++;
  if (++sdBufWr >= NUM_SDWRBUF)
  {
    sdBufWr = 0;
  }
  //sdBufInQ--;
  if (--sdBufInQ)
  {
    ShimTask_set(TASK_SDWRITE);
  }

#if USE_FATFS
  if (file_status != 0)
  {
    shimmerStatus.sdBadFile = 1;
  }
  else
  {
    shimmerStatus.sdBadFile = 0;
  }
#endif

  sensing.inSdWr = 0;
  sensing.inSdWrCnt = 0;
  //__enable_irq();
}

void ShimSd_openNewDataFile(void)
{
  ShimSdDataFile_makeFileName(dataFileName);
#if USE_FATFS //USE_FATFS
  file_status = f_open(&dataFile, dataFileName, FA_WRITE | FA_CREATE_NEW);
  assert_param(file_status == FR_OK);
  f_stat(dataFileName, &dataFileInfo);
#endif
}

void ShimSd_writeSdHeaderToFile(void)
{
#if USE_FATFS
  UINT bw;
#endif

  uint8_t temp_sdHeadText[SD_HEAD_SIZE];
  ShimSdHead_sdHeadTextGet(temp_sdHeadText, 0, SD_HEAD_SIZE);

  /* Because the S3 uses it's RTC in counter mode and it doesn't feature a way
   * of setting the current counter value, S3 code uses an 8-byte RTC diff and
   * a 5 byte clock counter which are both stored to the SD header. Because S3R
   * has an RTC in-which we can set the time, there's no need to have a separate
   * RTC diff variable. Since the header only has space for 5 bytes initial
   * timestamp counter, we need to put the upper 3 bytes of the RTC time within
   * the RTC diff bytes. */

#if defined(SHIMMER3R)
  /* Set lower 5 bytes of RTC diff to be 0*/
  temp_sdHeadText[SDH_RTC_DIFF_7] = 0;
  temp_sdHeadText[SDH_RTC_DIFF_6] = 0;
  temp_sdHeadText[SDH_RTC_DIFF_5] = 0;
  temp_sdHeadText[SDH_RTC_DIFF_4] = 0;
  temp_sdHeadText[SDH_RTC_DIFF_3] = 0;
#endif

  /* Save initial timestamp to header (5 bytes, LSB order) */
  temp_sdHeadText[SDH_INITIAL_TIMESTAMP_0] = (sdFileSyncTs >> 0) & 0xff;
  temp_sdHeadText[SDH_INITIAL_TIMESTAMP_1] = (sdFileSyncTs >> 8) & 0xff;
  temp_sdHeadText[SDH_INITIAL_TIMESTAMP_2] = (sdFileSyncTs >> 16) & 0xff;
  temp_sdHeadText[SDH_INITIAL_TIMESTAMP_3] = (sdFileSyncTs >> 24) & 0xff;
  temp_sdHeadText[SDH_INITIAL_TIMESTAMP_4] = (sdFileSyncTs >> 32) & 0xff;

#if defined(SHIMMER3R)
  /* Save upper 3 bytes to RTC timestamp diff bytes (MSB order) */
  temp_sdHeadText[SDH_RTC_DIFF_2] = (sdFileSyncTs >> 40) & 0xff;
  temp_sdHeadText[SDH_RTC_DIFF_1] = (sdFileSyncTs >> 48) & 0xff;
  temp_sdHeadText[SDH_RTC_DIFF_0] = (sdFileSyncTs >> 56) & 0xff;
#endif

#if USE_FATFS //USE_FATFS
  file_status = f_write(&dataFile, temp_sdHeadText, SD_HEAD_SIZE, &bw);
#endif
}

void ShimSd_closeDataFile(void)
{
#if USE_FATFS
  file_status = f_sync(&dataFile);
  assert_param(file_status == FR_OK);
  file_status = f_close(&dataFile);
  assert_param(file_status == FR_OK);
  ShimSd_setFileTimestamp(dataFileName);
#endif
}

uint8_t ShimSd_isFileStatusOk(void)
{
  return file_status == FR_OK;
}

uint8_t *ShimSd_fileNamePtrGet(void)
{
  return &fileName[0];
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

void PrepareSDBuffHead(void)
{
  memcpy(&sdWrBuf[sdBufSens][sdWrLen[sdBufSens]], ShimSdSync_myTimeDiffPtrGet(),
      SYNC_PACKET_PAYLOAD_SIZE);
  sdWrLen[sdBufSens] += SYNC_PACKET_PAYLOAD_SIZE;
  ShimSdSync_resetMyTimeDiff();
}
