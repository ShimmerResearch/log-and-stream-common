/*
 * shimmer_audio_file.h
 *
 * The layouts of the audio files beside data/<experiment>/<session>/<NNN>
 * (DEV-99): the raw audio's NNN.WAV and the fingerprint's NNN.AFP, both in
 * audio/<experiment>/<session>/.
 *
 * The WAV is a standard 16 kHz, mono, 16-bit PCM WAV that any player opens,
 * whose first 512 bytes carry a "shim" chunk identifying the recording, so
 * audio and sensor data can be paired and put on one clock after the files are
 * copied off. 512 bytes so the samples start on a card sector and every write
 * is a whole number of sectors.
 *
 * Kept free of any platform dependency so the layout can be host-tested
 * (Test/host/test_audio_file.c). docs/SHIMMER3R_AUDIO.md has the tables. All
 * fields are little-endian.
 *
 * Each header is written when the file opens and again when it closes, with
 * the sizes and the closing summary. A file whose `closed` byte is 0 was cut
 * short, for example by a flat battery: its contents are good up to its
 * length, but its sizes and summary are not.
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

/*
 * The fingerprint file, audio/<experiment>/<session>/<NNN>.AFP, written in
 * modes 1 and 2. A 512-byte header, then one record per landmark, grouped in
 * chunks: 5 s of audio analysed every 4 s, each chunk preceded by its own
 * header. Bytes 44-95 of the file header are the WAV's identity fields, laid
 * out the same, so one routine reads both; offset 80 holds the Shimmer clock
 * of the file's first chunk. The rest of ShimAudioFileInfo's closing summary
 * applies too, except samples, lastSampleTicks and silenceSamples, which are
 * the WAV's.
 *
 * Bytes 4-15 give the version, the header's length and the chunk header and
 * record sizes, so a reader can find the chunks without knowing the rest of
 * the header: the first chunk starts at the header's length.
 *
 * A record is one landmark: bits 0-8 t1, the frame (16 ms) within its chunk;
 * 9-14 dt; 15-20 (f2 - f1) & 63; 21-27 f1 & 127; 28-31 zero. That is the
 * audfprint hash (f1 << 25 | df << 19 | dt) and when it occurred.
 */
/* The version of the chunk and record layout. Byte 48 is the identity's. */
#define AUDIO_FP_FORMAT_VERSION    1U
#define AUDIO_FP_FILE_HEADER_LEN   512U
#define AUDIO_FP_CHUNK_HEADER_LEN  16U
#define AUDIO_FP_RECORD_LEN        4U
#define AUDIO_FP_CHUNK_SAMPLES     80000U /* 5 s at 16 kHz */
#define AUDIO_FP_CHUNK_HOP_SAMPLES 64000U /* one chunk every 4 s */
#define AUDIO_FP_FRAME_HOP_SAMPLES 256U   /* a record's t1 counts these */
#define AUDIO_FP_CHUNK_MARK        0x4843U /* "CH": finds chunks in a cut-short file */

typedef struct
{
  uint32_t chunks;         /* chunks in this file */
  uint64_t lastChunkTicks; /* Shimmer clock of the last one's first sample */
  uint32_t records;        /* records in this file */
  uint32_t chunksSkipped;  /* since the session began: analysis fell behind */
} ShimAudioFpSummary;

/* Writes AUDIO_FP_FILE_HEADER_LEN bytes of fingerprint-file header */
void ShimAudioFile_packFpHeader(uint8_t *out,
    const ShimAudioFileInfo *info,
    const ShimAudioFpSummary *fp);
/* Writes AUDIO_FP_CHUNK_HEADER_LEN bytes. chunk counts from the session's
 * first: its first sample is chunk x AUDIO_FP_CHUNK_HOP_SAMPLES into the
 * session's audio. startTicks is that sample's Shimmer clock. */
void ShimAudioFile_packFpChunkHeader(uint8_t *out, uint32_t chunk, uint64_t startTicks, uint16_t records);
/* Writes one AUDIO_FP_RECORD_LEN record for the landmark (t1, f1, f2, dt).
 * Returns 0, and writes nothing, if t1 or dt does not fit. */
uint8_t ShimAudioFile_packFpRecord(uint8_t *out, uint16_t t1, uint8_t f1, uint8_t f2, uint8_t dt);

#endif /* SHIMMER_AUDIO_FILE_H_ */
