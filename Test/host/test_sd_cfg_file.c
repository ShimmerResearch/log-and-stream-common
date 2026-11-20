/*
 * Host-side test for SDCard/shimmer_sd_cfg_file.c: the alternative accel and
 * mag enables survive a round trip through sdlog.cfg (DEV-1326).
 *
 * Built against the stub platform in Test/host/stubs - see stubs/README.md.
 * Run by .github/workflows/host-tests.yml.
 *
 * WHAT THIS IS FOR. ShimSdCfgFile_generate() writes the two enables as
 * accel_alt= and mag_alt= on both platforms. Until DEV-1326 the Shimmer3
 * parser read them only as accel_mpu= and mag_mpu=, and the parse starts from
 * a blank configuration, so a Shimmer3 that read back its own file - on
 * undock, or at boot with the configuration-changed flag clear - turned both
 * sensors off. Seen on a unit on 20 Nov 2026 (DEV-1431 bench, Shimmer3 B8B6):
 * InfoMem byte 5 went 0x64 -> 0x04 while its card's sdlog.cfg said
 * accel_alt=1 and mag_alt=1. test_round_trip_keeps_the_alternative_sensors
 * fails on Shimmer3 with the old parser. A Shimmer3 now reads both names; a
 * Shimmer3R still reads only accel_alt and mag_alt.
 *
 * The card is one file in memory. The configuration helpers the parser calls
 * are stand-ins below: this test is about which keys reach which bits, not
 * about the range and rate conversions those helpers do.
 *
 * Built TWICE, once per platform: the parser's key table differs by platform.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "host_test.h"
#include "log_and_stream_includes.h"

#if defined(SHIMMER3)
#define PLATFORM_NAME "Shimmer3"
#else
#define PLATFORM_NAME "Shimmer3R"
#endif

/* ---- the card: sdlog.cfg alone ------------------------------------------- */

static struct
{
  char data[4096];
  size_t len;
  uint8_t present;
} card;

static void cardHolds(const char *text)
{
  card.len = strlen(text);
  memcpy(card.data, text, card.len);
  card.data[card.len] = 0;
  card.present = 1;
}

FRESULT f_open(FIL *fp, const TCHAR *path, BYTE mode)
{
  (void) path;
  memset(fp, 0, sizeof(*fp));
  if (mode & FA_WRITE)
  {
    card.present = 1; /* FA_OPEN_ALWAYS: written from the start, then truncated */
    return FR_OK;
  }
  return card.present ? FR_OK : FR_NO_FILE;
}

FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw)
{
  size_t at = (size_t) fp->fptr;
  if (at + btw >= sizeof(card.data))
  {
    *bw = 0;
    return FR_DENIED;
  }
  memcpy(&card.data[at], buff, btw);
  fp->fptr += btw;
  if ((size_t) fp->fptr > card.len)
  {
    card.len = (size_t) fp->fptr;
  }
  card.data[card.len] = 0;
  *bw = btw;
  return FR_OK;
}

FRESULT f_truncate(FIL *fp)
{
  card.len = (size_t) fp->fptr;
  card.data[card.len] = 0;
  return FR_OK;
}

TCHAR *f_gets(TCHAR *buff, int len, FIL *fp)
{
  int n = 0;
  while (n < len - 1 && (size_t) fp->fptr < card.len)
  {
    char c = card.data[fp->fptr++];
    buff[n++] = c;
    if (c == '\n')
    {
      break;
    }
  }
  buff[n] = 0;
  return n ? buff : 0;
}

FRESULT f_close(FIL *fp)
{
  (void) fp;
  return FR_OK;
}

/* ---- stand-ins for what the parser and the writer call -------------------- */

static char nameText[] = "Shimmer_TEST";
static char expIdText[] = "DefaultTrial";
static char configTimeText[] = "0";
static uint8_t syncName[12];

uint8_t LogAndStream_checkSdInSlot(void)
{
  return 1;
}

void LogAndStream_infomemUpdate(void)
{
}

void Board_setSdPower(uint8_t state)
{
  (void) state;
}

void ShimTask_clear(TaskId_t task_id)
{
  (void) task_id;
}

uint8_t getDefaultBaudForBtVersion(void)
{
  return 0;
}

SENSINGTypeDef sensing;
#if defined(SHIMMER3R)
uint8_t isBmp581InUse(void)
{
  return 1;
}
#endif

uint8_t ShimConfig_areConfigBytesValid(void)
{
  return 1;
}

uint8_t ShimConfig_checkAndCorrectConfig(void)
{
  return 0; /* nothing to correct, so parse() does not write the file back */
}

void ShimConfig_checkBtModeFromConfig(void)
{
}

uint8_t ShimConfig_configByteAltMagRateGet(void)
{
  return 0;
}

void ShimConfig_configByteAltMagRateSet(uint8_t value)
{
  (void) value;
}

uint8_t ShimConfig_configByteMagRateGet(void)
{
  return 0;
}

void ShimConfig_configByteMagRateSet(uint8_t value)
{
  (void) value;
}

uint8_t ShimConfig_configBytePressureOversamplingRatioGet(void)
{
  return 0;
}

void ShimConfig_configBytePressureOversamplingRatioSet(uint8_t value)
{
  (void) value;
}

char *ShimConfig_configTimeParseToTxtAndPtrGet(void)
{
  return configTimeText;
}

void ShimConfig_configTimeSet(uint32_t time)
{
  (void) time;
}

void ShimConfig_createBlankConfigBytes(void)
{
  memset(ShimConfig_getStoredConfig()->rawBytes, 0, STOREDCONFIG_SIZE);
}

char *ShimConfig_expIdParseToTxtAndPtrGet(void)
{
  return expIdText;
}

uint16_t ShimConfig_experimentLengthEstimatedInSecGet(void)
{
  return 0;
}

uint16_t ShimConfig_experimentLengthMaxInMinutesGet(void)
{
  return 0;
}

float ShimConfig_freqDiv(float samplingRate)
{
  return samplingRate > 0 ? 32768.0f / samplingRate : 0;
}

void ShimConfig_gyroRangeSet(uint8_t value)
{
  (void) value;
}

void ShimConfig_gyroRateSet(uint8_t value)
{
  (void) value;
}

void ShimConfig_readRam(void)
{
}

char *ShimConfig_shimmerNameParseToTxtAndPtrGet(void)
{
  return nameText;
}

uint8_t ShimConfig_wrAccelLpModeGet(void)
{
  return 0;
}

void ShimConfig_wrAccelLpModeSet(uint8_t value)
{
  (void) value;
}

void ShimSdSync_parseSyncCenterNameFromCfgFile(uint8_t *storedConfigPtr, char *equals)
{
  (void) storedConfigPtr;
  (void) equals;
}

void ShimSdSync_parseSyncCenterNameFromConfig(uint8_t *storedConfigPtr)
{
  (void) storedConfigPtr;
}

void ShimSdSync_parseSyncNodeNameFromCfgFile(uint8_t *storedConfigPtr, char *equals)
{
  (void) storedConfigPtr;
  (void) equals;
}

void ShimSdSync_parseSyncNodeNamesFromConfig(uint8_t *storedConfigPtr)
{
  (void) storedConfigPtr;
}

void ShimSdSync_resetSyncNodeArray(void)
{
}

void ShimSdSync_resetSyncVariablesBeforeParseConfig(void)
{
}

uint8_t *ShimSdSync_syncCenterNamePtrGet(void)
{
  return syncName;
}

uint8_t *ShimSdSync_syncNodeNamePtrForIndexGet(uint8_t index)
{
  (void) index;
  return syncName;
}

uint8_t ShimSdSync_syncNodeNumGet(void)
{
  return 0;
}

/* ---- the tests ------------------------------------------------------------ */

static gConfigBytes *cfg(void)
{
  return ShimConfig_getStoredConfig();
}

/* A configuration with the given alternative-sensor enables and a few others
 * on, at 51.2 Hz, as a host would leave it. */
static void configure(uint8_t altAccel, uint8_t altMag)
{
  memset(cfg()->rawBytes, 0, STOREDCONFIG_SIZE);
  cfg()->chEnLnAccel = 1;
  cfg()->chEnGyro = 1;
  cfg()->chEnPressureAndTemperature = 1;
  cfg()->chEnAltAccel = altAccel;
  cfg()->chEnAltMag = altMag;
  cfg()->samplingRateTicks = 640;
}

/* Generate the file from that configuration, blank the configuration (a
 * different one stands in for whatever InfoMem held), and read the file back. */
static void roundTrip(void)
{
  memset(&card, 0, sizeof(card));
  shimmerStatus.docked = 0;
  shimmerStatus.sdBadFile = 0;
  expectU("generate() wrote the file", ShimSdCfgFile_generate(), 1);
  memset(cfg()->rawBytes, 0, STOREDCONFIG_SIZE);
  ShimSdCfgFile_parse();
}

static void test_round_trip_keeps_the_alternative_sensors(void)
{
  testCase("test_round_trip_keeps_the_alternative_sensors");
  configure(1, 1);
  roundTrip();
  expectTrue("the file says accel_alt=1", strstr(card.data, "\naccel_alt=1\r\n") != 0);
  expectTrue("the file says mag_alt=1", strstr(card.data, "\nmag_alt=1\r\n") != 0);
  expectU("alt accel read back on", cfg()->chEnAltAccel, 1);
  expectU("alt mag read back on", cfg()->chEnAltMag, 1);
  /* controls: keys that always round-tripped */
  expectU("LN accel read back on", cfg()->chEnLnAccel, 1);
  expectU("gyro read back on", cfg()->chEnGyro, 1);
  expectU("pressure read back on", cfg()->chEnPressureAndTemperature, 1);
}

static void test_round_trip_keeps_them_off(void)
{
  testCase("test_round_trip_keeps_them_off");
  configure(0, 0);
  roundTrip();
  expectU("alt accel read back off", cfg()->chEnAltAccel, 0);
  expectU("alt mag read back off", cfg()->chEnAltMag, 0);
  configure(1, 0);
  roundTrip();
  expectU("alt accel alone: accel on", cfg()->chEnAltAccel, 1);
  expectU("alt accel alone: mag off", cfg()->chEnAltMag, 0);
  configure(0, 1);
  roundTrip();
  expectU("alt mag alone: accel off", cfg()->chEnAltAccel, 0);
  expectU("alt mag alone: mag on", cfg()->chEnAltMag, 1);
}

/* The older Shimmer3 names, as the C# API writes them and the Shimmer3 user
 * manual's example shows them. A Shimmer3 still reads them. A Shimmer3R
 * ignores them, as the manual says: a Shimmer3 file must not turn its
 * alternative sensors on. */
static void test_the_older_shimmer3_names(void)
{
  testCase("test_the_older_shimmer3_names");
  memset(cfg()->rawBytes, 0, STOREDCONFIG_SIZE);
  cardHolds("accel=1\r\naccel_mpu=1\r\nmag_mpu=1\r\nsample_rate=51.2\r\n");
  ShimSdCfgFile_parse();
#if defined(SHIMMER3)
  expectU("accel_mpu=1 turns alt accel on", cfg()->chEnAltAccel, 1);
  expectU("mag_mpu=1 turns alt mag on", cfg()->chEnAltMag, 1);
#else
  expectU("accel_mpu=1 is ignored", cfg()->chEnAltAccel, 0);
  expectU("mag_mpu=1 is ignored", cfg()->chEnAltMag, 0);
#endif
  expectU("accel=1 still read", cfg()->chEnLnAccel, 1);
}

/* Keys that only start with accel_alt / mag_alt are ranges and rates, not
 * enables: cfg_key_is() needs the '=' straight after the name. */
static void test_range_and_rate_keys_are_not_enables(void)
{
  testCase("test_range_and_rate_keys_are_not_enables");
  memset(cfg()->rawBytes, 0, STOREDCONFIG_SIZE);
  cardHolds("accel_alt_range=1\r\naccel_alt_rate=1\r\nmag_alt_range=1\r\n"
            "mag_alt_rate=1\r\nsample_rate=51.2\r\n");
  ShimSdCfgFile_parse();
  expectU("alt accel stays off", cfg()->chEnAltAccel, 0);
  expectU("alt mag stays off", cfg()->chEnAltMag, 0);
}

int main(void)
{
  printf("shimmer_sd_cfg_file host tests (" PLATFORM_NAME ")\n\n");
  hostTestSilenceUnused();

  test_round_trip_keeps_the_alternative_sensors();
  test_round_trip_keeps_them_off();
  test_the_older_shimmer3_names();
  test_range_and_rate_keys_are_not_enables();

  return hostTestReport("shimmer_sd_cfg_file (" PLATFORM_NAME ")");
}

#endif /* SHIMMER_HOST_TEST */
