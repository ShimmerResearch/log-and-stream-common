/*
 * Host-side test for Audio/shimmer_audio_file.c.
 *
 * The header of an audio file is a contract with every player that opens the
 * WAV and every importer that pairs it with its data file (DEV-99,
 * docs/SHIMMER3R_AUDIO.md), so its offsets are pinned here as numbers, and
 * the WAV parts are checked as a standard reader would read them.
 *
 * shimmer_audio_file.c includes nothing but the C library, so this needs no
 * stubs. Run by .github/workflows/host-tests.yml.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <string.h>

#include "host_test.h"

#include "Audio/shimmer_audio_file.h"

static uint32_t get16(const uint8_t *p)
{
  return (uint32_t) p[0] | ((uint32_t) p[1] << 8);
}

static uint32_t get32(const uint8_t *p)
{
  return get16(p) | (get16(p + 2) << 16);
}

static uint64_t get64(const uint8_t *p)
{
  return (uint64_t) get32(p) | ((uint64_t) get32(p + 4) << 32);
}

static ShimAudioFileInfo sample(void)
{
  ShimAudioFileInfo i;

  memset(&i, 0, sizeof(i));
  i.hwId = 10;
  i.mode = 3;
  i.openReason = 1;
  i.fileIndex = 0x0102;
  i.dataFileTs = 0x0123456789ABCDEFULL;
  i.configTime = 0x11223344UL;
  i.fwId = 3;
  i.fwMajor = 1;
  i.fwMinor = 1;
  i.fwInternal = 26;
  memcpy(i.mac, "\xCF\x9B\x9E\x36\x15\xBC", 6);
  i.firstSampleTicks = 0x00003569D5D30000ULL;
  i.decimation = 24;
  i.gain = -8;
  i.closed = 1;
  i.fault = 4;
  i.samples = 57600000UL; /* an hour */
  i.lastSampleTicks = 0x00003569E1D30000ULL;
  i.silenceSamples = 2048;
  i.escapeDrains = 3;
  i.writeErrors = 5;
  i.adfErrorEvents = 700;
  i.adfSaturation = 650;
  i.adfOverflow = 1;
  return i;
}

static void test_wav_parts(void)
{
  uint8_t h[AUDIO_FILE_HEADER_LEN];
  ShimAudioFileInfo i = sample();

  ShimAudioFile_packHeader(h, &i);

  testCase("a standard reader sees a 16 kHz mono 16-bit PCM WAV");
  expectU("RIFF", memcmp(h, "RIFF", 4), 0);
  expectU("RIFF size: the file less 8", get32(h + 4), 512U - 8U + 115200000UL);
  expectU("WAVE", memcmp(h + 8, "WAVE", 4), 0);
  expectU("fmt", memcmp(h + 12, "fmt ", 4), 0);
  expectU("fmt length", get32(h + 16), 16);
  expectU("PCM", get16(h + 20), 1);
  expectU("mono", get16(h + 22), 1);
  expectU("16000 Hz", get32(h + 24), 16000);
  expectU("32000 B/s", get32(h + 28), 32000);
  expectU("block align", get16(h + 32), 2);
  expectU("16 bits", get16(h + 34), 16);

  testCase("the shim chunk fills the gap so the samples start at 512");
  expectU("shim id", memcmp(h + 36, "shim", 4), 0);
  expectU("shim length reaches the data chunk", 36U + 8U + get32(h + 40), 504);
  expectU("data id at 504", memcmp(h + 504, "data", 4), 0);
  expectU("data length", get32(h + 508), 115200000UL);
  expectU("header length", AUDIO_FILE_HEADER_LEN, 512);
  expectU("samples start at", AUDIO_FILE_DATA_OFFSET, 512);
}

static void test_shim_fields(void)
{
  uint8_t h[AUDIO_FILE_HEADER_LEN];
  ShimAudioFileInfo i = sample();
  unsigned k;
  unsigned nonZero = 0;

  ShimAudioFile_packHeader(h, &i);

  testCase("the identity fields sit at their documented offsets");
  expectU("magic", memcmp(h + 44, "SAUD", 4), 0);
  expectU("format version", h[48], 1);
  expectU("hardware id", h[49], 10);
  expectU("mode", h[50], 3);
  expectU("open reason", h[51], 1);
  expectX("file index", get16(h + 52), 0x0102);
  expectX("data file timestamp", get64(h + 56), 0x0123456789ABCDEFULL);
  expectX("config time", get32(h + 64), 0x11223344UL);
  expectU("firmware id", get16(h + 68), 3);
  expectU("firmware major", get16(h + 70), 1);
  expectU("firmware minor", h[72], 1);
  expectU("firmware internal", h[73], 26);
  expectU("MAC", memcmp(h + 74, "\xCF\x9B\x9E\x36\x15\xBC", 6), 0);
  expectX("first sample ticks", get64(h + 80), 0x00003569D5D30000ULL);
  expectU("decimation", get32(h + 88), 24);
  expectX("gain, two's complement", get32(h + 92), 0xFFFFFFF8UL);

  testCase("the closing summary sits at its documented offsets");
  expectU("closed", h[96], 1);
  expectU("fault", h[97], 4);
  expectU("samples", get32(h + 100), 57600000UL);
  expectX("last sample ticks", get64(h + 104), 0x00003569E1D30000ULL);
  expectU("silence samples", get32(h + 112), 2048);
  expectU("escape drains", get16(h + 116), 3);
  expectU("write errors", get16(h + 118), 5);
  expectU("ADF error events", get32(h + 120), 700);
  expectU("ADF saturation", get32(h + 124), 650);
  expectU("ADF overflow", get32(h + 128), 1);

  testCase("everything else in the shim chunk is zero");
  expectU("reserved at 54", get16(h + 54), 0);
  expectU("reserved at 98", get16(h + 98), 0);
  for (k = 132; k < 504; k++)
  {
    nonZero += h[k] != 0;
  }
  expectU("bytes 132-503", nonZero, 0);
}

static void test_open_header(void)
{
  uint8_t h[AUDIO_FILE_HEADER_LEN];
  ShimAudioFileInfo i = sample();

  testCase("a file as opened: no samples, not closed");
  i.closed = 0;
  i.samples = 0;
  ShimAudioFile_packHeader(h, &i);
  expectU("not closed", h[96], 0);
  expectU("RIFF size of an empty WAV", get32(h + 4), 504);
  expectU("no sample bytes", get32(h + 508), 0);
}

int main(void)
{
  printf("shimmer_audio_file host tests\n\n");
  hostTestSilenceUnused();

  test_wav_parts();
  test_shim_fields();
  test_open_header();

  return hostTestReport("shimmer_audio_file");
}

#endif /* SHIMMER_HOST_TEST */
