/*
 * shimmer_audio_mode.h
 *
 * Configuration byte 233 (NV_AUDIO_MODE): whether a Shimmer3R records audio
 * while it logs to SD, and what it keeps (DEV-99). Kept free of any platform
 * dependency so every one of its 256 values can be host-tested
 * (Test/host/test_audio_mode.c). docs/SHIMMER3_CONFIGURATION_INFOMEM.md
 * section 8.5 has the table and the reasons behind each rule.
 *
 *    bits 0-1  mode         AUDIO_MODE_*
 *    bits 2-3  gain preset  AUDIO_GAIN_*; only the calibrated default so far
 *    bits 4-6  unused, cleared
 *    bit 7     must be 0
 *
 * 0 is off. Hosts that do not know the byte write 0 there or leave it alone,
 * and erased flash (0xFF) has bit 7 set, so an older setup never records.
 * Anything this firmware cannot act on reads as 0 too: bit 7 set, a reserved
 * gain preset, or a mode it does not build yet (AUDIO_MODES_BUILT). So a host
 * that writes a mode and reads it back learns whether this unit records it.
 * ShimAudioMode_correct() is that rule; every query below applies it first.
 * Whether the hardware has a microphone at all is the caller's to add
 * (ShimBrd_isMicrophoneSupported()).
 */

#ifndef LOG_AND_STREAM_COMMON_CONFIGURATION_SHIMMER_AUDIO_MODE_H_
#define LOG_AND_STREAM_COMMON_CONFIGURATION_SHIMMER_AUDIO_MODE_H_

#include <stdint.h>

#define AUDIO_MODE_MASK            0x03U
#define AUDIO_MODE_GAIN_SHIFT      2U
#define AUDIO_MODE_GAIN_MASK       0x0CU
#define AUDIO_MODE_INVALID         0x80U

/* Bits 0-1: what is recorded while logging. FINGERPRINT keeps only the audio
 * fingerprint; FINGERPRINT_RAW keeps it and the raw 16 kHz audio; RAW keeps
 * only the raw audio. */
#define AUDIO_MODE_OFF             0U
#define AUDIO_MODE_FINGERPRINT     1U
#define AUDIO_MODE_FINGERPRINT_RAW 2U
#define AUDIO_MODE_RAW             3U

/* Bits 2-3: microphone gain. 1-3 are reserved for presets, should field use
 * call for them, and read as off until built. */
#define AUDIO_GAIN_DEFAULT         0U

/* The modes this firmware records, one bit per AUDIO_MODE_*. Raw audio from
 * DEV-99 step 4, the fingerprint modes from step 5. */
#define AUDIO_MODES_BUILT                                  \
  ((1U << AUDIO_MODE_OFF) | (1U << AUDIO_MODE_FINGERPRINT) \
      | (1U << AUDIO_MODE_FINGERPRINT_RAW) | (1U << AUDIO_MODE_RAW))

/* The byte this firmware would act on: raw itself when it is valid and its
 * mode is built, and 0 otherwise. Bits 4-6 are cleared. */
uint8_t ShimAudioMode_correct(uint8_t raw);

uint8_t ShimAudioMode_mode(uint8_t raw);
uint8_t ShimAudioMode_gainPreset(uint8_t raw);
uint8_t ShimAudioMode_recordsFingerprint(uint8_t raw);
uint8_t ShimAudioMode_recordsRawAudio(uint8_t raw);

/* For the sdlog.cfg key: raw with its mode replaced. A value outside the field
 * sets bit 7, so the byte reads as off rather than as whatever the low bits of
 * a typing error happen to be. */
uint8_t ShimAudioMode_withMode(uint8_t raw, int value);

#endif /* LOG_AND_STREAM_COMMON_CONFIGURATION_SHIMMER_AUDIO_MODE_H_ */
