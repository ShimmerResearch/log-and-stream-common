/*
 * s4sd.cpp
 *
 *  Created on: Mar 21, 2024
 *      Author: MarkNolan
 */

#include "shimmer_sd.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <Configuration/shimmer_config.h>
#include <SDCard/shimmer_sd_header.h>
#include <SDSync/shimmer_sd_sync.h>
#include <Sensing/shimmer_sensing.h>
#include <TaskList/shimmer_taskList.h>
#include <Util/shimmer_util.h>
#include <log_and_stream_definitions.h>
#include <log_and_stream_externs.h>

#if defined(SHIMMER3)
#include "ff.h"

#include "../5xx_HAL/hal_RTC.h"

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

uint8_t fileName[64], dirName[64], expDirName[32], dataBuf[100],
    sdWrBuf[NUM_SDWRBUF][SD_WRITE_BUF_SIZE], //btMacHex[6], btMacAscii[14],
    sdBufInQ, expIdName[MAX_CHARS], shimmerName[MAX_CHARS],
    configTimeText[UINT32_LEN], //sdBufInQMax,
    dirLen, sdBufSens = 0, sdBufWr = 0;
uint16_t fileNum, dirCounter, sdWrLen[NUM_SDWRBUF];
uint64_t sdFileCrTs, sdFileSyncTs;
#if USE_FATFS
FRESULT file_status;
//char file_name_current[128];
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

static uint8_t all0xff[7U] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

uint8_t sdInfoSyncDelayed = 0;

#endif

void SD_init(void)
{
  setSdInfoSyncDelayed(0);
  //fileNum = 0;

  //*configTimeText = '\0';
  //*fileName = '\0';

  sensing.isFileCreated = 0;
  sensing.inSdWr = 0;
  sensing.inSdWrCnt = 0;

  memset(&fileName[0], 0x00, sizeof(fileName));
  memset(&dirName[0], 0x00, sizeof(dirName));
  memset(&expDirName[0], 0x00, sizeof(expDirName));
  memset(&dataBuf[0], 0x00, sizeof(dataBuf));
  memset(&sdWrBuf[0][0], 0x00, sizeof(sdWrBuf));
  sdBufInQ = 0;
  memset(&expIdName[0], 0x00, sizeof(expIdName));
  memset(&shimmerName[0], 0x00, sizeof(shimmerName));
  memset(&configTimeText[0], 0x00, sizeof(configTimeText));

  dirLen = 0;
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
  sdInfoSyncDelayed = 0;
}

#define TEST_TEXT_LEN 40

uint8_t SD_test1(void)
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

uint8_t SD_test2(void)
{
  FRESULT res = FR_OK; /* FatFs function common result code */
#if USE_FATFS
  uint32_t byteswritten; //, bytesread; /* File write/read counts */
  uint8_t wtext[] = "FATFS works great!"; /* File write buffer */
#if _USE_MKFS
  uint8_t rtext[_MAX_SS]; /* File read buffer */
#endif
  FIL SDFile;

  if (SD_mount(1) != FR_OK)
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
  SD_mount(0);
#endif
  return res;
}

void SD_setShimmerName(void)
{
  uint8_t i;
  gConfigBytes *configBytes = ShimConfig_getStoredConfig();

  memset(&shimmerName[0], 0x00, sizeof(shimmerName));

  for (i = 0; (i < MAX_CHARS - 1) && isprint((uint8_t) configBytes->shimmerName[i]); i++)
    ;
  if (i == 0)
  {
    setDefaultShimmerName();
  }
  memcpy((char *) shimmerName, &(configBytes->shimmerName[0]),
      sizeof(configBytes->shimmerName));
}

void SD_setExpIdName(void)
{
  uint8_t i;
  gConfigBytes *configBytes = ShimConfig_getStoredConfig();

  memset(&expIdName[0], 0x00, sizeof(expIdName));

  for (i = 0; (i < MAX_CHARS - 1) && (isprint((uint8_t) configBytes->expIdName[i])); i++)
    ;
  if (i == 0)
  {
    setDefaultTrialId();
    i = 12;
  }
  memcpy((char *) expIdName, &(configBytes->expIdName[0]), i);
  //strcpy((char*)expIdName,"DefaultTrial");
}

void SD_setCfgTime(void)
{
  uint32_t cfg_time_temp = 0;
  uint8_t i;
  gConfigBytes *configBytes = ShimConfig_getStoredConfig();

  //MSB order
  for (i = 0; i < 4; i++)
  {
    cfg_time_temp <<= 8;
    cfg_time_temp |= configBytes->rawBytes[NV_SD_CONFIG_TIME + i];
  }
  if (cfg_time_temp)
  {
    ShimUtil_ItoaNo0((uint64_t) cfg_time_temp, configTimeText, UINT32_LEN);
  }
  else
  {
    strcpy((char *) configTimeText, "0");
  }
}

void SetName(void)
{
  if (strlen((char *) configTimeText) == 0)
  {
    strcpy((char *) configTimeText, "0");
  }
  if (strlen((char *) fileName) == 0)
  {
    strcpy((char *) fileName, "no_file   ");
  }
}

void SD_infomem2Names(void)
{
  SD_setShimmerName();
  SD_setExpIdName();
  SD_setCfgTime();
}

uint8_t SD_setBasedir(void)
{
#if USE_FATFS
  FILINFO fno;
  //volatile uint8_t res;
  uint16_t tmp_counter = 0;
  char lfn[_MAX_LFN + 1], *fname, *scout, *dash, dirnum[8];

  SD_infomem2Names();

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
    if (file_status = f_opendir(&dir, "/data"))
    {
      return 0; //FAIL;
    }
  }
#if _FATFS != FATFS_V_0_08B
  file_status = f_closedir(&dir);
  set_file_timestamp("/data");
#endif

  strcpy((char *) expDirName, "data/");
  strcat((char *) expDirName, (char *) expIdName);
  strcat((char *) expDirName, "_");
  strcat((char *) expDirName, (char *) configTimeText);

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

      if (!strncmp(fname, (char *) shimmerName, strlen(fname) - 4))
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
  set_file_timestamp((char *) expDirName);
#endif

  //at this point, we have the id string and the counter, so we can make a directory name
  return 1; //SUCCESS;
#endif
}

uint8_t SD_makeBasedir(void)
{
  memset(dirName, 0, 64);

  char dir_counter_text[4];
  ShimUtil_ItoaWith0((uint64_t) dirCounter, (uint8_t *) dir_counter_text, 4);

  strcpy((char *) dirName, (char *) expDirName);
  strcat((char *) dirName, "/");
  strcat((char *) dirName, (char *) shimmerName);
  strcat((char *) dirName, "-");
  strcat((char *) dirName, dir_counter_text);

#if USE_FATFS
  if (file_status = f_mkdir((char *) dirName))
  {
    FindError(file_status, dirName);
    return 0; //FAIL;
  }

#endif

  memset(fileName, 0, 64);
  strcpy((char *) fileName, (char *) dirName);
  dirLen = strlen((char *) dirName);
  strcat((char *) fileName, "/000");
  fileNum = 0;
  //sprintf((char*)fileName, "/%03d", fileNum++);

  set_file_timestamp((char *) dirName);

  return 1; //SUCCESS;
}

void SD_makeFileName(char *name_buf)
{
  //strcpy(file_name, "this_is_a_very_long_name_for_the_dataFile");
  uint8_t temp_str[7];
  sprintf((char *) temp_str, "/%03d", fileNum++);

  strcpy((char *) name_buf, (char *) dirName);
  strcat((char *) name_buf, (char *) temp_str);
}

void SD_fileInit(void)
{
#if USE_SD
#if USE_FATFS
  UINT bw;
#endif
  uint8_t temp_sdHeadText[SD_HEAD_SIZE];

  if (!shimmerStatus.sdPowerOn)
  {
    Board_setSdPower(1);
  }

  sensing.isSdOperating = 1;
  SD_setBasedir();
  SD_makeBasedir();
  ShimSdHead_config2SdHead();
  SD_makeFileName(dataFileName);

  S4Ram_sdHeadTextGet(temp_sdHeadText, 0, SD_HEAD_SIZE);

#if USE_FATFS
  file_status = f_open(&dataFile, dataFileName, FA_WRITE | FA_CREATE_NEW);
  f_stat(dataFileName, &dataFileInfo);
#endif
  sdFileSyncTs = sdFileCrTs = RTC_get64();
#if USE_8BYTES_INIT_TS
  *(uint64_t *) (temp_sdHeadText + SDH_MY_LOCALTIME_0TH) = sdFileSyncTs;
#else
  temp_sdHeadText[SDH_MY_LOCALTIME_5TH] = (sdFileSyncTs >> 32) & 0xff;
  *(uint32_t *) (temp_sdHeadText + SDH_MY_LOCALTIME)
      = (uint32_t) (sdFileSyncTs & 0xffffffff);
#endif
#if USE_FATFS
  file_status = f_write(&dataFile, temp_sdHeadText, SD_HEAD_SIZE, &bw);
#endif
  sensing.isSdOperating = 0;
  sensing.isFileCreated = 1;
  memset(sdWrLen, 0, NUM_SDWRBUF * sizeof(sdWrLen[0]));
  sdBufInQ = sdBufSens = sdBufWr = 0;
#endif //USE_SD
}

void SD_close(void)
{
#if USE_SD
#if USE_FATFS
  f_sync(&dataFile);
  f_close(&dataFile);
  set_file_timestamp(dataFileName);
  file_status = FR_OK;
#endif
  //Board_sdPower(0);
  shimmerStatus.sdBadFile = 0;
  sensing.isFileCreated = 0;
#endif //USE_SD
}

void SD_writeToBuff(uint8_t *buf, uint16_t len)
{
#if USE_SD
  //uint8_t *sensing_buf;
  //uint16_t *sensing_buf_len;
  if ((NUM_SDWRBUF == sdBufInQ) || (sensing.isFileCreated == 0))
  {
    __NOP();
    return;
  }
  //sdWrBuf[NUM_SDWRBUF][SD_WRITE_BUF_SIZE], sdBufSens, sdBufWr, sdBufInQ;

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
#endif //USE_SD
}

void SD_writeToCard(void)
{
  //__disable_irq();
#if USE_FATFS
  UINT bw;
#endif
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
  file_status = f_lseek(&dataFile,
      f_size(&dataFile)); //dataFileInfo.fsize was not incrementing the file size.
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
#if USE_FATFS
    file_status = f_sync(&dataFile);
    assert_param(file_status == FR_OK);
    file_status = f_close(&dataFile);
    assert_param(file_status == FR_OK);
    set_file_timestamp(dataFileName);
#endif
    SD_makeFileName(dataFileName);
#if USE_FATFS
    file_status = f_open(&dataFile, dataFileName, FA_WRITE | FA_CREATE_NEW);
    assert_param(file_status == FR_OK);
#endif
    uint8_t temp_sdHeadText[SD_HEAD_SIZE];
    S4Ram_sdHeadTextGet(temp_sdHeadText, 0, SD_HEAD_SIZE);
#if USE_8BYTES_INIT_TS
    *(uint64_t *) (temp_sdHeadText + SDH_MY_LOCALTIME_0TH) = sdFileSyncTs;
#else
    temp_sdHeadText[SDH_MY_LOCALTIME_5TH] = (sdFileSyncTs >> 32) & 0xff;
    *(uint32_t *) (temp_sdHeadText + SDH_MY_LOCALTIME)
        = (uint32_t) (sdFileSyncTs & 0xffffffff);
#endif

#if USE_FATFS
    file_status = f_write(&dataFile, temp_sdHeadText, SD_HEAD_SIZE, &bw);
    assert_param(file_status == FR_OK);
#endif
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
  //else if (test_cnt >= 15) {
  //   f_close(&dataFile);
  //}

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

FRESULT SD_mount(uint8_t val)
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

void UpdateSdConfig(void)
{
  FIL cfgFile;

  if (!shimmerStatus.docked && CheckSdInslot() && !shimmerStatus.sdBadFile)
  {
    uint8_t sd_power_state = shimmerStatus.sdPowerOn;
    if (!shimmerStatus.sdPowerOn)
    {
      Board_setSdPower(1);
    }

    char buffer[66], val_char[21];
    float val_num;
    uint16_t val_int, val_f;
    uint64_t temp64;

    uint8_t i;
    resetSyncVariablesBeforeParseConfig();

    UINT bw;

    memset(shimmerName, 0, sizeof(shimmerName));
    memset(expIdName, 0, sizeof(expIdName));
    memset(configTimeText, 0, sizeof(configTimeText));

    char cfgname[] = "sdlog.cfg";

    gConfigBytes *storedConfig = ShimConfig_getStoredConfig();

    if (memcmp(all0xff, ShimConfig_getStoredConfig(), 6))
    {
      file_status = f_open(&cfgFile, cfgname, FA_WRITE | FA_CREATE_ALWAYS);

      //sensor0
      sprintf(buffer, "accel=%d\r\n", storedConfig->chEnLnAccel);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "gyro=%d\r\n", storedConfig->chEnGyro);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "mag=%d\r\n", storedConfig->chEnMag);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "exg1_24bit=%d\r\n", storedConfig->chEnExg1_24Bit);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "exg2_24bit=%d\r\n", storedConfig->chEnExg2_24Bit);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "gsr=%d\r\n", storedConfig->chEnGsr);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#if defined(SHIMMER3)
      sprintf(buffer, "extch7=%d\r\n", storedConfig->chEnExtADC7);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "extch6=%d\r\n", storedConfig->chEnExtADC6);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#elif defined(SHIMMER3)
      sprintf(buffer, "extch0=%d\r\n", storedConfig->chEnExtADC0);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "extch1=%d\r\n", storedConfig->chEnExtADC1);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#endif
      //sensor1
      sprintf(buffer, "br_amp=%d\r\n", storedConfig->chEnBridgeAmp);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "vbat=%d\r\n", storedConfig->chEnVBattery);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "accel_d=%d\r\n", storedConfig->chEnWrAccel);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#if defined(SHIMMER3)
      sprintf(buffer, "extch15=%d\r\n", storedConfig->chEnExtADC15);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "intch1=%d\r\n", storedConfig->chEnIntADC1);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "intch12=%d\r\n", storedConfig->chEnIntADC12);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "intch13=%d\r\n", storedConfig->chEnIntADC13);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      //sensor2
      sprintf(buffer, "intch14=%d\r\n", storedConfig->chEnIntADC14);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#elif defined(SHIMMER3)
      sprintf(buffer, "extch2=%d\r\n", storedConfig->chEnExtADC2);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "intch3=%d\r\n", storedConfig->chEnIntADC3);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "intch0=%d\r\n", storedConfig->chEnIntADC0);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "intch1=%d\r\n", storedConfig->chEnIntADC1);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      //sensor2
      sprintf(buffer, "intch2=%d\r\n", storedConfig->chEnIntADC2);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#endif
      sprintf(buffer, "accel_alt=%d\r\n", storedConfig->chEnAltAccel);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "mag_alt=%d\r\n", storedConfig->chEnAltMag);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "exg1_16bit=%d\r\n", storedConfig->chEnExg1_16Bit);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "exg2_16bit=%d\r\n", storedConfig->chEnExg2_16Bit);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "pres=%d\r\n", storedConfig->chEnPressureAndTemperature);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      //sample_rate
      val_num = FreqDiv(storedConfig->samplingRateTicks);
      val_int = (uint16_t) floor(val_num); //the compiler can't handle sprintf %f here
      val_f = (uint16_t) round((val_num - floor(val_num)) * 100);
      if (val_f == 100)
      {
        val_f = 0;
        val_int++;
      }
      if (val_f)
      {
        sprintf(val_char, "%d.%d", val_int, val_f);
      }
      else
      {
        sprintf(val_char, "%d", val_int);
      }
      sprintf(buffer, "sample_rate=%s\r\n", val_char);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      //setup config
      sprintf(buffer, "mg_internal_rate=%d\r\n", ShimConfig_configByteMagRateGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "mg_range=%d\r\n", storedConfig->magRange);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "acc_internal_rate=%d\r\n", storedConfig->wrAccelRate);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#if defined(SHIMMER3)
      sprintf(buffer, "accel_alt_range=%d\r\n", storedConfig->altAccelRange);
#elif defined(SHIMMER3R)
      sprintf(buffer, "accel_ln_range=%d\r\n", storedConfig->lnAccelRange);
#endif
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#if defined(SHIMMER3)
      sprintf(buffer,
          (isBmp180InUse() ? ("pres_bmp180_prec=%d\r\n") : ("pres_bmp280_prec=%d\r\n")),
#elif defined(SHIMMER3R)
      sprintf(buffer, "pres_bmp390_prec=%d\r\n",
#endif
          ShimConfig_configBytePressureOversamplingRatioGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "gsr_range=%d\r\n", storedConfig->gsrRange);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "exp_power=%d\r\n", storedConfig->expansionBoardPower);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "gyro_range=%d\r\n", ShimConfig_gyroRangeGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "gyro_samplingrate=%d\r\n", storedConfig->gyroRate);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "acc_range=%d\r\n", storedConfig->wrAccelRange);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "acc_lpm=%d\r\n", ShimConfig_wrAccelLpModeGet());
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "acc_hrm=%d\r\n", storedConfig->wrAccelHrMode);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#if defined(SHIMMER3R)
      sprintf(buffer, "mag_alt_rate=%d\r\n", storedConfig->altMagRate);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "accel_alt_rate=%d\r\n", storedConfig->altAccelRate);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
#endif

      //trial config
      sprintf(buffer, "user_button_enable=%d\r\n", storedConfig->userButtonEnable);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "rtc_error_enable=%d\r\n", storedConfig->rtcErrorEnable);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "sd_error_enable=%d\r\n", storedConfig->sdErrorEnable);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "iammaster=%d\r\n", storedConfig->masterEnable);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "sync=%d\r\n", storedConfig->syncEnable);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "low_battery_autostop=%d\r\n", storedConfig->lowBatteryAutoStop);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "interval=%d\r\n", storedConfig->btIntervalSecs);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "bluetoothDisabled=%d\r\n", storedConfig->bluetoothDisable);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      sprintf(buffer, "max_exp_len=%d\r\n", storedConfig->experimentLengthMaxInMinutes);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      sprintf(buffer, "est_exp_len=%d\r\n", storedConfig->experimentLengthEstimatedInSec);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      parseSyncNodeNamesFromConfig(&storedConfig->rawBytes[0]);
      for (i = 0; i < getSyncNodeNum(); i++)
      {
        sprintf(buffer, "node=%s\r\n", (char *) getSyncNodeNamePtrForIndex(i));
        f_write(&cfgFile, buffer, strlen(buffer), &bw);
      }

      if (memcmp(all0xff, storedConfig + NV_CENTER, 6))
      {
        parseSyncCenterNameFromConfig(&storedConfig->rawBytes[0]);
        sprintf(buffer, "center=%s\r\n", (char *) getSyncCenterNamePtr());
        f_write(&cfgFile, buffer, strlen(buffer), &bw);
      }

      sprintf(buffer, "singletouch=%d\r\n", storedConfig->singleTouchStart);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      sprintf(buffer, "myid=%d\r\n", storedConfig->myTrialID);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "Nshimmer=%d\r\n", storedConfig->numberOfShimmers);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      SD_infomem2Names();
      sprintf(buffer, "shimmername=%s\r\n", shimmerName);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "experimentid=%s\r\n", expIdName);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "configtime=%s\r\n", configTimeText);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      temp64 = storedConfig->rawBytes[NV_DERIVED_CHANNELS_0]
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_1]) << 8)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_2]) << 16)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_3]) << 24)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_4]) << 32)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_5]) << 40)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_6]) << 48)
          + (((uint64_t) storedConfig->rawBytes[NV_DERIVED_CHANNELS_7]) << 56);
      ShimUtil_ItoaNo0(temp64, (uint8_t *) val_char, 21);
      sprintf(buffer, "derived_channels=%s\r\n", val_char); //todo: got value 0?
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      sprintf(buffer, "EXG_ADS1292R_1_CONFIG1=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_CONFIG1]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_CONFIG2=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_CONFIG2]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_LOFF=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_LOFF]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_CH1SET=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_CH1SET]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_CH2SET=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_CH2SET]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_RLD_SENS=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_RLD_SENS]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_LOFF_SENS=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_LOFF_SENS]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_LOFF_STAT=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_LOFF_STAT]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_RESP1=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_RESP1]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_1_RESP2=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_1_RESP2]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_CONFIG1=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_CONFIG1]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_CONFIG2=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_CONFIG2]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_LOFF=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_LOFF]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_CH1SET=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_CH1SET]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_CH2SET=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_CH2SET]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_RLD_SENS=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_RLD_SENS]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_LOFF_SENS=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_LOFF_SENS]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_LOFF_STAT=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_LOFF_STAT]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_RESP1=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_RESP1]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);
      sprintf(buffer, "EXG_ADS1292R_2_RESP2=%d\r\n",
          storedConfig->rawBytes[NV_EXG_ADS1292R_2_RESP2]);
      f_write(&cfgFile, buffer, strlen(buffer), &bw);

      file_status = f_close(&cfgFile);
      set_file_timestamp(cfgname);

#if defined(SHIMMER3)
      _delay_cycles(2400000); //100ms @ 24MHz
#elif defined(SHIMMER3R)
      HAL_Delay(100); //100ms @ 24MHz
#endif
    }
    else
    {
      file_status = FR_DISK_ERR;
    }
    if (!sd_power_state)
    {
      Board_setSdPower(0);
    }
  }
}

void ParseConfig(void)
{
  FIL cfgFile;

  char buffer[66], *equals;
  uint8_t string_length = 0;
  float sample_rate = 51.2;
  uint16_t sample_period = 0;
  uint64_t derived_channels_val = 0;
  uint8_t gsr_range = 0;

  uint8_t triggerSdCardUpdate = 0;

  CheckSdInslot();

  char cfgname[] = "sdlog.cfg";
  file_status = f_open(&cfgFile, cfgname, FA_READ | FA_OPEN_EXISTING);
  if (file_status == FR_NO_FILE)
  {
    ShimConfig_readRam();
    UpdateSdConfig();
    //fileBad = 0;
  }
  else if (file_status != FR_OK)
  {
    shimmerStatus.sdBadFile = 1;
    //fileBad = (initializing) ? 0 : 1;
    return;
  }
  else
  {
    gConfigBytes stored_config_temp;
    memset((uint8_t *) (stored_config_temp.rawBytes), 0,
        sizeof(stored_config_temp.rawBytes)); //0

    resetSyncVariablesBeforeParseConfig();
    resetSyncNodeArray();

    memset((uint8_t *) (stored_config_temp.rawBytes), 0, NV_LN_ACCEL_CALIBRATION); //0
    memset((uint8_t *) (stored_config_temp.rawBytes + NV_LN_ACCEL_CALIBRATION), 0xFF, 84);
    memset((uint8_t *) (stored_config_temp.rawBytes + NV_DERIVED_CHANNELS_3), 0, 5); //0
    memset((uint8_t *) (stored_config_temp.rawBytes + NV_SENSORS3), 0, 5); //0
    memset((uint8_t *) (stored_config_temp.rawBytes + NV_ALT_ACCEL_CALIBRATION), 0xFF, 82);
    memset((uint8_t *) (stored_config_temp.rawBytes + NV_SD_MYTRIAL_ID), 0, 9); //0
    InfoMem_read(NV_MAC_ADDRESS, stored_config_temp.rawBytes + NV_MAC_ADDRESS, 7);
    memset((uint8_t *) (stored_config_temp.rawBytes + NV_BT_SET_PIN + 1), 0xff, 24);
    memset((uint8_t *) (stored_config_temp.rawBytes + NV_CENTER), 0xff, 128);

#if defined(SHIMMER3)
    stored_config_temp.rawBytes[NV_SD_TRIAL_CONFIG0] &= ~SDH_SET_PMUX; //PMUX reserved as 0
//stored_config_temp.rawBytes[NV_SD_TRIAL_CONFIG0] |= SDH_TIME_STAMP; //TIME_STAMP always = 1
#endif
    stored_config_temp.gsrRange = GSR_AUTORANGE;
    stored_config_temp.bufferSize = 1;
#if defined(SHIMMER3)
    stored_config_temp.btCommsBaudRate = getDefaultBaudForBtVersion();
#elif defined(SHIMMER3R)
    stored_config_temp.btCommsBaudRate = 0xFF;
#endif
    stored_config_temp.bluetoothDisable = 0;
    stored_config_temp.btIntervalSecs = SYNC_INT_C;

    memset(shimmerName, 0, sizeof(shimmerName));
    memset(expIdName, 0, sizeof(expIdName));
    memset(configTimeText, 0, sizeof(configTimeText));
    *configTimeText = '\0';

    stored_config_temp.shimmerName[0] = '\0';
    stored_config_temp.expIdName[0] = '\0';
    stored_config_temp.configTime = 0;

    while (f_gets(buffer, 64, &cfgFile))
    {
      if (!(equals = strchr(buffer, '=')))
      {
        continue;
      }
      equals++; //this is the value
      if (strstr(buffer, "accel="))
      {
        stored_config_temp.chEnLnAccel = atoi(equals);
      }
      else if (strstr(buffer, "gyro="))
      {
        stored_config_temp.chEnGyro = atoi(equals);
      }
      else if (strstr(buffer, "mag="))
      {
        stored_config_temp.chEnMag = atoi(equals);
      }
      else if (strstr(buffer, "exg1_24bit="))
      {
        stored_config_temp.chEnExg1_24Bit = atoi(equals);
      }
      else if (strstr(buffer, "exg2_24bit="))
      {
        stored_config_temp.chEnExg2_24Bit = atoi(equals);
      }
      else if (strstr(buffer, "gsr="))
      {
        stored_config_temp.chEnGsr = atoi(equals);
      }
#if defined(SHIMMER3)
      else if (strstr(buffer, "extch7="))
      {
        stored_config_temp.chEnExtADC7 = atoi(equals);
      }
      else if (strstr(buffer, "extch6="))
      {
        stored_config_temp.chEnExtADC6 = atoi(equals);
      }
#elif defined(SHIMMER3R)
      else if (strstr(buffer, "extch0="))
      {
        stored_config_temp.chEnExtADC0 = atoi(equals);
      }
      else if (strstr(buffer, "extch1="))
      {
        stored_config_temp.chEnExtADC1 = atoi(equals);
      }
#endif
      else if (strstr(buffer, "str=") || strstr(buffer, "br_amp="))
      {
        stored_config_temp.chEnBridgeAmp = atoi(equals);
      }
      else if (strstr(buffer, "vbat="))
      {
        stored_config_temp.chEnVBattery = atoi(equals);
      }
      else if (strstr(buffer, "accel_d="))
      {
        stored_config_temp.chEnWrAccel = atoi(equals);
      }
#if defined(SHIMMER3)
      else if (strstr(buffer, "extch15="))
      {
        stored_config_temp.chEnExtADC15 = atoi(equals);
      }
      else if (strstr(buffer, "intch1="))
      {
        stored_config_temp.chEnIntADC1 = atoi(equals);
      }
      else if (strstr(buffer, "intch12="))
      {
        stored_config_temp.chEnIntADC12 = atoi(equals);
      }
      else if (strstr(buffer, "intch13="))
      {
        stored_config_temp.chEnIntADC13 = atoi(equals);
      }
      else if (strstr(buffer, "intch14="))
      {
        stored_config_temp.chEnIntADC14 = atoi(equals);
      }
      else if (strstr(buffer, "accel_mpu="))
      {
        stored_config_temp.chEnAltAccel = atoi(equals);
      }
      else if (strstr(buffer, "mag_mpu="))
      {
        stored_config_temp.chEnAltMag = atoi(equals);
      }
#elif defined(SHIMMER3R)
      else if (strstr(buffer, "extch2="))
      {
        stored_config_temp.chEnExtADC2 = atoi(equals);
      }
      else if (strstr(buffer, "intch3="))
      {
        stored_config_temp.chEnIntADC3 = atoi(equals);
      }
      else if (strstr(buffer, "intch0="))
      {
        stored_config_temp.chEnIntADC0 = atoi(equals);
      }
      else if (strstr(buffer, "intch1="))
      {
        stored_config_temp.chEnIntADC1 = atoi(equals);
      }
      else if (strstr(buffer, "intch2="))
      {
        stored_config_temp.chEnIntADC2 = atoi(equals);
      }
      else if (strstr(buffer, "accel_alt="))
      {
        stored_config_temp.chEnAltAccel = atoi(equals);
      }
      else if (strstr(buffer, "mag_alt="))
      {
        stored_config_temp.chEnAltMag = atoi(equals);
      }
#endif
      else if (strstr(buffer, "exg1_16bit="))
      {
        stored_config_temp.chEnExg1_16Bit = atoi(equals);
      }
      else if (strstr(buffer, "exg2_16bit="))
      {
        stored_config_temp.chEnExg2_16Bit = atoi(equals);
      }
      else if (strstr(buffer, "pres="))
      {
        stored_config_temp.chEnPressureAndTemperature = atoi(equals);
      }
      else if (strstr(buffer, "sample_rate="))
      {
        sample_rate = atof(equals);
      }
      else if (strstr(buffer, "mg_internal_rate="))
      {
        ShimConfig_configByteMagRateSet(&stored_config_temp, atoi(equals));
      }
      else if (strstr(buffer, "mg_range="))
      {
        stored_config_temp.magRange = atoi(equals);
      }
      else if (strstr(buffer, "acc_internal_rate="))
      {
        stored_config_temp.wrAccelRate = atoi(equals);
      }
#if defined(SHIMMER3)
      else if (strstr(buffer, "accel_alt_range="))
      {
        stored_config_temp.altAccelRange = atoi(equals);
      }
#elif defined(SHIMMER3R)
      else if (strstr(buffer, "accel_ln_range="))
      {
        stored_config_temp.lnAccelRange = atoi(equals);
      }
#endif
      else if (strstr(buffer, "acc_range="))
      {
        stored_config_temp.wrAccelRange = atoi(equals);
      }
      else if (strstr(buffer, "acc_lpm="))
      {
        ShimConfig_wrAccelLpModeSet(&stored_config_temp, atoi(equals));
      }
      else if (strstr(buffer, "acc_hrm="))
      {
        stored_config_temp.wrAccelHrMode = atoi(equals);
      }
      else if (strstr(buffer, "gsr_range="))
      { //or "gsr_range="?
        gsr_range = atoi(equals);
        if (gsr_range > 4)
        {
          gsr_range = 4;
        }

        stored_config_temp.gsrRange = gsr_range;
      }
      else if (strstr(buffer, "gyro_samplingrate="))
      {
        ShimConfig_gyroRateSet(&stored_config_temp, atoi(equals));
      }
      else if (strstr(buffer, "gyro_range="))
      {
        ShimConfig_gyroRangeSet(&stored_config_temp, atoi(equals));
      }
#if defined(SHIMMER3)
      else if (strstr(buffer, "pres_bmp180_prec=") || strstr(buffer, "pres_bmp280_prec="))
      {
        ShimConfig_configBytePressureOversamplingRatioSet(&stored_config_temp, atoi(equals));
      }
#elif defined(SHIMMER3R)
      else if (strstr(buffer, "pres_bmp390_prec="))
      {
        ShimConfig_configBytePressureOversamplingRatioSet(&stored_config_temp, atoi(equals));
      }
#endif

#if defined(SHIMMER3R)
      else if (strstr(buffer, "mag_alt_rate="))
      {
        stored_config_temp.altMagRate = atoi(equals);
      }
      else if (strstr(buffer, "accel_alt_rate="))
      {
        stored_config_temp.altAccelRate = atoi(equals);
      }
#endif
      else if (strstr(buffer, "rtc_error_enable="))
      {
        stored_config_temp.rtcErrorEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "sd_error_enable="))
      {
        stored_config_temp.sdErrorEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "user_button_enable="))
      {
        stored_config_temp.userButtonEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "iammaster="))
      { //0=slave=node
        stored_config_temp.masterEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "sync="))
      {
        stored_config_temp.syncEnable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "bluetoothDisabled="))
      {
        stored_config_temp.bluetoothDisable = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "low_battery_autostop="))
      {
        stored_config_temp.lowBatteryAutoStop = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "interval="))
      {
        stored_config_temp.btIntervalSecs = atoi(equals) > 255 ? 255 : atoi(equals);
      }
      else if (strstr(buffer, "exp_power="))
      {
        stored_config_temp.expansionBoardPower = (atoi(equals) == 0) ? 0 : 1;
      }
      else if (strstr(buffer, "center="))
      {
        parseSyncCenterNameFromCfgFile(&stored_config_temp.rawBytes[0], equals);
      }
      else if (strstr(buffer, "node"))
      {
        parseSyncNodeNameFromCfgFile(&stored_config_temp.rawBytes[0], equals);
      }
      else if (strstr(buffer, "est_exp_len="))
      {
        stored_config_temp.experimentLengthEstimatedInSec = atoi(equals);
      }
      else if (strstr(buffer, "max_exp_len="))
      {
        stored_config_temp.experimentLengthMaxInMinutes = atoi(equals);
      }
#if defined(SHIMMER3)
      else if (strstr(buffer, "singletouch="))
      {
        if (IS_SUPPORTED_SINGLE_TOUCH)
        {
          stored_config_temp.singleTouchStart = (atoi(equals) == 0) ? 0 : 1;
        }
        else
        {
          stored_config_temp.singleTouchStart = 0;
        }
      }
#endif
      else if (strstr(buffer, "myid="))
      {
        stored_config_temp.myTrialID = atoi(equals);
      }
      else if (strstr(buffer, "Nshimmer="))
      {
        stored_config_temp.numberOfShimmers = atoi(equals);
      }
      else if (strstr(buffer, "shimmername="))
      {
        string_length = strlen(equals);
        if (string_length > MAX_CHARS)
        {
          string_length = MAX_CHARS - 1;
        }
        else if (string_length >= 2)
        {
          string_length -= 2;
        }
        else
        {
          string_length = 0;
        }
        memcpy(&stored_config_temp.shimmerName[0], equals, string_length);
        if (!memcmp(&stored_config_temp.shimmerName[0], "ID", 2))
        {
          memcpy(&stored_config_temp.shimmerName[0], "id", 2);
        }
        memcpy((char *) shimmerName, &stored_config_temp.shimmerName[0], MAX_CHARS - 1);
        shimmerName[string_length] = 0;
      }
      else if (strstr(buffer, "experimentid="))
      {
        string_length = strlen(equals);
        if (string_length > MAX_CHARS)
        {
          string_length = MAX_CHARS - 1;
        }
        else if (string_length >= 2)
        {
          string_length -= 2;
        }
        else
        {
          string_length = 0;
        }
        memcpy(&stored_config_temp.expIdName[0], equals, string_length);
        memcpy((char *) expIdName, &stored_config_temp.expIdName[0], MAX_CHARS - 1);
        expIdName[string_length] = 0;
      }
      else if (strstr(buffer, "configtime="))
      {
        stored_config_temp.configTime = atol(equals);
        string_length = MAX_CHARS < strlen(equals) ? MAX_CHARS : strlen(equals) - 1;
        memcpy((char *) configTimeText, equals, string_length - 1);
        *(configTimeText + string_length - 1) = 0;
      }

      else if (strstr(buffer, "EXG_ADS1292R_1_CONFIG1="))
      {
        stored_config_temp.exgADS1292rRegsCh1.config1 = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_CONFIG2="))
      {
        stored_config_temp.exgADS1292rRegsCh1.config2 = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_LOFF="))
      {
        stored_config_temp.exgADS1292rRegsCh1.loff = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_CH1SET="))
      {
        stored_config_temp.exgADS1292rRegsCh1.ch1set = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_CH2SET="))
      {
        stored_config_temp.exgADS1292rRegsCh1.ch2set = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_RLD_SENS="))
      {
        stored_config_temp.exgADS1292rRegsCh1.rldSens = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_LOFF_SENS="))
      {
        stored_config_temp.exgADS1292rRegsCh1.loffSens = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_LOFF_STAT="))
      {
        stored_config_temp.exgADS1292rRegsCh1.loffStat = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_RESP1="))
      {
        stored_config_temp.exgADS1292rRegsCh1.resp1 = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_1_RESP2="))
      {
        stored_config_temp.exgADS1292rRegsCh1.resp2 = atoi(equals);
      }

      else if (strstr(buffer, "EXG_ADS1292R_2_CONFIG1="))
      {
        stored_config_temp.exgADS1292rRegsCh2.config1 = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_CONFIG2="))
      {
        stored_config_temp.exgADS1292rRegsCh2.config2 = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_LOFF="))
      {
        stored_config_temp.exgADS1292rRegsCh2.loff = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_CH1SET="))
      {
        stored_config_temp.exgADS1292rRegsCh2.ch1set = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_CH2SET="))
      {
        stored_config_temp.exgADS1292rRegsCh2.ch2set = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_RLD_SENS="))
      {
        stored_config_temp.exgADS1292rRegsCh2.rldSens = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_LOFF_SENS="))
      {
        stored_config_temp.exgADS1292rRegsCh2.loffSens = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_LOFF_STAT="))
      {
        stored_config_temp.exgADS1292rRegsCh2.loffStat = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_RESP1="))
      {
        stored_config_temp.exgADS1292rRegsCh2.resp1 = atoi(equals);
      }
      else if (strstr(buffer, "EXG_ADS1292R_2_RESP2="))
      {
        stored_config_temp.exgADS1292rRegsCh2.resp2 = atoi(equals);
      }

      else if (strstr(buffer, "derived_channels="))
      {
        derived_channels_val = atoll(equals);
        stored_config_temp.rawBytes[NV_DERIVED_CHANNELS_0] = derived_channels_val & 0xFF;
        stored_config_temp.rawBytes[NV_DERIVED_CHANNELS_1]
            = (derived_channels_val >> 8) & 0xFF;
        stored_config_temp.rawBytes[NV_DERIVED_CHANNELS_2]
            = (derived_channels_val >> 16) & 0xFF;
        stored_config_temp.rawBytes[NV_DERIVED_CHANNELS_3]
            = (derived_channels_val >> 24) & 0xFF;
        stored_config_temp.rawBytes[NV_DERIVED_CHANNELS_4]
            = (derived_channels_val >> 32) & 0xFF;
        stored_config_temp.rawBytes[NV_DERIVED_CHANNELS_5]
            = (derived_channels_val >> 40) & 0xFF;
        stored_config_temp.rawBytes[NV_DERIVED_CHANNELS_6]
            = (derived_channels_val >> 48) & 0xFF;
        stored_config_temp.rawBytes[NV_DERIVED_CHANNELS_7]
            = (derived_channels_val >> 56) & 0xFF;
      }
    }
    file_status = f_close(&cfgFile);

#if defined(SHIMMER3)
    _delay_cycles(1200000); //50ms
#elif defined(SHIMMER3R)
    HAL_Delay(50); //50ms
#endif

    sample_period = FreqDiv(sample_rate);
    stored_config_temp.samplingRateTicks = sample_period;

    ShimConfig_experimentLengthSecsMaxSet(stored_config_temp.experimentLengthMaxInMinutes);
    setSyncEstExpLen(stored_config_temp.experimentLengthEstimatedInSec);

    triggerSdCardUpdate |= ShimConfig_checkAndCorrectConfig(&stored_config_temp);

    /* Calibration bytes are not copied over from the temporary config bytes */
    /* Infomem D - Bytes 0-33 - General settings */
    gConfigBytes *storedConfig = ShimConfig_getStoredConfig();
    memcpy(&storedConfig->rawBytes[0], &stored_config_temp.rawBytes[0], NV_NUM_SETTINGS_BYTES);
    /* Infomem D - Bytes 118-122 - Derived channel settings */
    memcpy(&storedConfig->rawBytes[NV_DERIVED_CHANNELS_3],
        &stored_config_temp.rawBytes[NV_DERIVED_CHANNELS_3], 5);
    /* Infomem C - Bytes 128-132 - MPL related settings - no longer used/supported */
    memcpy(&storedConfig->rawBytes[NV_SENSORS3],
        &stored_config_temp.rawBytes[NV_SENSORS3], 5);
    /* Infomem C - Bytes 187-223 - Shimmer name, exp ID, config time, trial ID, num Shimmers, trial config, BT interval, est exp len, max exp len */
    memcpy(&storedConfig->rawBytes[NV_SD_SHIMMER_NAME],
        &stored_config_temp.rawBytes[NV_SD_SHIMMER_NAME], 37);
    /* Infomem B - Bytes 256-381 - Center and Node MAC addresses */
    memcpy(&storedConfig->rawBytes[NV_CENTER],
        &stored_config_temp.rawBytes[NV_CENTER], NV_NUM_BYTES_SYNC_CENTER_NODE_ADDRS);

    ShimSdHead_config2SdHead();
    SetName();

#if defined(SHIMMER3)
    InfoMem_write(0, &storedConfig->rawBytes[0], NV_NUM_SETTINGS_BYTES);
    InfoMem_write(NV_SENSORS3, &storedConfig->rawBytes[NV_SENSORS3], 5);
    InfoMem_write(NV_DERIVED_CHANNELS_3, &storedConfig->rawBytes[NV_DERIVED_CHANNELS_3], 5);
    InfoMem_write(NV_SD_SHIMMER_NAME,
        &storedConfig->rawBytes[NV_SD_SHIMMER_NAME], NV_NUM_SD_BYTES);
    InfoMem_write(NV_CENTER, &storedConfig->rawBytes[NV_CENTER],
        NV_NUM_BYTES_SYNC_CENTER_NODE_ADDRS);
#elif defined(SHIMMER3R)
    InfoMem_update();
#endif

    /* If the configuration needed to be corrected, update the config file */
    if (triggerSdCardUpdate)
    {
      UpdateSdConfig();
    }
  }
}

uint8_t isFileStatusOk(void)
{
  return file_status == FR_OK;
}

uint8_t isSdInfoSyncDelayed(void)
{
  return sdInfoSyncDelayed;
}

void setSdInfoSyncDelayed(uint8_t state)
{
  sdInfoSyncDelayed = state;
}

uint8_t *getConfigTimeTextPtr(void)
{
  return &configTimeText[0];
}

uint8_t *getFileNamePtr(void)
{
  return &fileName[0];
}

FRESULT set_file_timestamp(char *path)
{
#if defined(SHIMMER3R)
  FILINFO fno;
  RTC_TimeTypeDef RTC_TimeStructure;
  RTC_DateTypeDef RTC_DateStructure;
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

  S4_RTC_t data;
  S4_RTC_GetDateTime(&data);
  data.year += 2000;
  fno.fdate = (WORD) (((data.year - 1980) << 9) | data.month << 5 | data.date);
  fno.ftime = (WORD) (data.hours << 11 | data.minutes << 5 | data.seconds / 2);

  return f_utime(path, &fno);
#else
  return FR_OK;
#endif
}

void FindError(uint8_t err, uint8_t *name)
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

void SdInfoSync()
{
    setSdInfoSyncDelayed(0);
    if (GetSdCfgFlag())
    { // info > sdcard
        ShimConfig_readRam();
        UpdateSdConfig();
        SetSdCfgFlag(0);
    }
    else
    {
        ReadSdConfiguration();
    }

    if (GetRamCalibFlag())
    {
        ShimmerCalib_ram2File();
        SetRamCalibFlag(0);
    }
    else
    {
        if (ShimmerCalib_file2Ram())
        {
            ShimmerCalib_ram2File();
        }
        else
        {
            // only need to do this when file2Ram succeeds
            ShimmerCalibSyncFromDumpRamAll();
        }
    }

    ShimSens_configureChannels();
    CheckOnDefault();
}
