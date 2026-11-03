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

static void test_fp_header(void)
{
  uint8_t h[AUDIO_FP_FILE_HEADER_LEN];
  uint8_t wav[AUDIO_FILE_HEADER_LEN];
  ShimAudioFileInfo i = sample();
  ShimAudioFpSummary fp;
  unsigned k;
  unsigned nonZero = 0;

  memset(&fp, 0, sizeof(fp));
  fp.chunks = 899;
  fp.lastChunkTicks = 0x00003569E1D20000ULL;
  fp.records = 1234567UL;
  fp.chunksSkipped = 7;
  i.mode = 1;
  ShimAudioFile_packFpHeader(h, &i, &fp);
  ShimAudioFile_packHeader(wav, &i);

  testCase("the fingerprint file's own fields");
  expectU("magic", memcmp(h, "SAFP", 4), 0);
  expectU("version", get16(h + 4), 1);
  expectU("file header bytes", get16(h + 6), 512);
  expectU("sample rate", get16(h + 8), 16000);
  expectU("frame hop samples", get16(h + 10), 256);
  expectU("chunk header bytes", get16(h + 12), 16);
  expectU("record bytes", get16(h + 14), 4);
  expectU("chunk samples", get32(h + 16), 80000);
  expectU("chunk hop samples", get32(h + 20), 64000);
  expectU("header length", AUDIO_FP_FILE_HEADER_LEN, 512);

  testCase("bytes 44-95 are the WAV's identity, byte for byte");
  expectU("identity matches the WAV header's", memcmp(h + 44, wav + 44, 52), 0);
  expectU("mode", h[50], 1);
  expectX("first chunk ticks at 80", get64(h + 80), 0x00003569D5D30000ULL);

  testCase("the closing summary sits at its documented offsets");
  expectU("closed", h[96], 1);
  expectU("fault", h[97], 4);
  expectU("chunks", get32(h + 100), 899);
  expectX("last chunk ticks", get64(h + 104), 0x00003569E1D20000ULL);
  expectU("records", get32(h + 112), 1234567UL);
  expectU("escape drains", get16(h + 116), 3);
  expectU("write errors", get16(h + 118), 5);
  expectU("ADF error events", get32(h + 120), 700);
  expectU("ADF saturation", get32(h + 124), 650);
  expectU("ADF overflow", get32(h + 128), 1);
  expectU("chunks skipped", get32(h + 132), 7);

  testCase("everything else is zero, the WAV's sample counts included");
  for (k = 24; k < 44; k++)
  {
    nonZero += h[k] != 0;
  }
  expectU("bytes 24-43", nonZero, 0);
  expectU("reserved at 54", get16(h + 54), 0);
  expectU("reserved at 98", get16(h + 98), 0);
  nonZero = 0;
  for (k = 136; k < AUDIO_FP_FILE_HEADER_LEN; k++)
  {
    nonZero += h[k] != 0;
  }
  expectU("bytes 136-511", nonZero, 0);
}

static void test_fp_chunk_header(void)
{
  uint8_t c[AUDIO_FP_CHUNK_HEADER_LEN];

  testCase("a chunk header");
  ShimAudioFile_packFpChunkHeader(c, 0x01020304UL, 0x00003569D5D30000ULL, 4695);
  expectX("mark, \"CH\" on the card", get16(c), 0x4843);
  expectU("bytes as stored", c[0] == 'C' && c[1] == 'H', 1);
  expectU("records", get16(c + 2), 4695);
  expectX("chunk", get32(c + 4), 0x01020304UL);
  expectX("start ticks", get64(c + 8), 0x00003569D5D30000ULL);
  expectU("length", AUDIO_FP_CHUNK_HEADER_LEN, 16);
}

/* audfprint's hash, as audio_fingerprint.h af_hash builds it */
static uint32_t audfprintHash(uint8_t f1, uint8_t f2, uint8_t dt)
{
  return ((uint32_t) (f1 & 0x7FU) << 25)
      | (((uint32_t) ((int) f2 - (int) f1) & 0x3FU) << 19) | dt;
}

static void test_fp_record(void)
{
  uint8_t r[AUDIO_FP_RECORD_LEN];
  uint32_t v;

  testCase("a record packs the landmark");
  expectU("packs", ShimAudioFile_packFpRecord(r, 312, 200, 180, 62), 1);
  v = get32(r);
  expectU("t1, bits 0-8", v & 0x1FFU, 312);
  expectU("dt, bits 9-14", (v >> 9) & 0x3FU, 62);
  expectU("df, bits 15-20", (v >> 15) & 0x3FU, (uint32_t) (180 - 200) & 0x3FU);
  expectU("f1 & 127, bits 21-27", (v >> 21) & 0x7FU, 200U & 0x7FU);
  expectU("bits 28-31 zero", v >> 28, 0);

  testCase("the record gives back audfprint's hash");
  expectX("hash", ((v >> 21) << 25) | (((v >> 15) & 0x3FU) << 19) | ((v >> 9) & 0x3FU),
      audfprintHash(200, 180, 62));
  expectU("a rising pair packs", ShimAudioFile_packFpRecord(r, 0, 3, 40, 2), 1);
  v = get32(r);
  expectX("its hash", ((v >> 21) << 25) | (((v >> 15) & 0x3FU) << 19) | ((v >> 9) & 0x3FU),
      audfprintHash(3, 40, 2));

  testCase("what does not fit is refused and nothing written");
  memset(r, 0xA5, sizeof(r));
  expectU("t1 512", ShimAudioFile_packFpRecord(r, 512, 1, 2, 3), 0);
  expectU("dt 64", ShimAudioFile_packFpRecord(r, 1, 1, 2, 64), 0);
  expectX("untouched", get32(r), 0xA5A5A5A5UL);
}

int main(void)
{
  printf("shimmer_audio_file host tests\n\n");
  hostTestSilenceUnused();

  test_wav_parts();
  test_shim_fields();
  test_open_header();
  test_fp_header();
  test_fp_chunk_header();
  test_fp_record();

  return hostTestReport("shimmer_audio_file");
}

#endif /* SHIMMER_HOST_TEST */
