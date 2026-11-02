/*
 * shimmer_audio_file.h
 *
 * The header of an audio file, audio/<experiment>/<session>/<NNN>.WAV beside
 * data/<experiment>/<session>/<NNN> (DEV-99). A standard 16 kHz, mono, 16-bit
 * PCM WAV that any player opens, whose first 512 bytes carry a "shim" chunk
 * identifying the recording, so audio and sensor data can be paired and put on
 * one clock after the files are copied off. 512 bytes so the samples start on
 * a card sector and every write is a whole number of sectors.
 *
 * Kept free of any platform dependency so the layout can be host-tested
 * (Test/host/test_audio_file.c). docs/SHIMMER3R_AUDIO.md has the table. All
 * fields are little-endian.
 *
 * The header is written when the file opens and again when it closes, with the
 * RIFF sizes and the closing summary. A file whose `closed` byte is 0 was cut
 * short, for example by a flat battery: its samples are good up to its length,
 * but its sizes and summary are not.
 */

#ifndef SHIMMER_AUDIO_FILE_H_
#define SHIMMER_AUDIO_FILE_H_

#include <stdint.h>

#define AUDIO_FILE_HEADER_LEN     512U
#define AUDIO_FILE_FORMAT_VERSION 1U
#define AUDIO_FILE_RATE_HZ        16000U
/* Where the samples start: AUDIO_FILE_HEADER_LEN */
#define AUDIO_FILE_DATA_OFFSET    AUDIO_FILE_HEADER_LEN

/* openReason */
#define AUDIO_FILE_OPEN_SESSION   0U /* the first file of a logging session */
#define AUDIO_FILE_OPEN_SPLIT     1U /* the data file split and this follows it */

typedef struct
{
  uint8_t hwId;        /* DEVICE_VER: 10 for Shimmer3R */
  uint8_t mode;        /* configuration byte 233's mode, as recorded */
  uint8_t openReason;  /* AUDIO_FILE_OPEN_* */
  uint16_t fileIndex;  /* NNN, as the data file's */
  uint64_t dataFileTs; /* the data file's initial timestamp: pairs the two */
  uint32_t configTime;
  uint16_t fwId;
  uint16_t fwMajor;
  uint8_t fwMinor;
  uint8_t fwInternal;
  uint8_t mac[6];
  uint64_t firstSampleTicks; /* Shimmer clock of this file's first sample */
  uint32_t decimation;       /* the ADF's sinc5 ratio */
  int32_t gain;              /* the ADF's SCALE */
  /* The closing summary: zero until the file closes */
  uint8_t closed;
  uint8_t fault; /* AUDIO_FAULT_* (Audio/shimmer_audio.h), the session's first */
  uint32_t samples;         /* samples in this file */
  uint64_t lastSampleTicks; /* Shimmer clock of this file's last sample */
  uint32_t silenceSamples;  /* samples written as silence after an overrun */
  uint16_t escapeDrains; /* writes forced ahead of other tasks to save the ring */
  uint16_t writeErrors;
  uint32_t adfErrorEvents;
  uint32_t adfSaturation;
  uint32_t adfOverflow;
} ShimAudioFileInfo;

/* Writes AUDIO_FILE_HEADER_LEN bytes of header for info into out */
void ShimAudioFile_packHeader(uint8_t *out, const ShimAudioFileInfo *info);

#endif /* SHIMMER_AUDIO_FILE_H_ */
