/*
 * shimmer_audio.h
 *
 * Audio recording while logging to SD (DEV-99), the platform-neutral half.
 * Configuration byte 233 (Configuration/shimmer_audio_mode.h) says what is
 * recorded. The Shimmer3R platform records it (PlatAudio_*, in
 * shimmer3r-firmware's Shimmer_Driver/hal_AudioRecorder.c) into audio/ beside
 * each data file (SDCard/shimmer_sd_companion.h), in the layout of
 * Audio/shimmer_audio_file.h. docs/SHIMMER3R_AUDIO.md has the whole of it.
 * Shimmer3 has no microphone, so everything here compiles to nothing there.
 */

#ifndef SHIMMER_AUDIO_H_
#define SHIMMER_AUDIO_H_

#include <stdint.h>

/* Status byte 1 (Comms/shimmer_bt_uart.c ShimBt_assembleStatusBytes).
 * RECORDING: capturing for the current log. FAULT: this session's audio
 * failed or was degraded. */
#define AUDIO_STATUS_RECORDING 0x02U
#define AUDIO_STATUS_FAULT     0x04U

/* AUDIO_STATUS_RESPONSE's payload length, after the opcode */
#define AUDIO_STATUS_LEN       32U

/* AUDIO_STATUS_RESPONSE byte 2. OFF: not configured, or not logging.
 * STARTING: logging has started and the file and the mic are coming up.
 * STOPPED: the session's recording ended normally. FAULT: stopped for the
 * rest of the session. */
#define AUDIO_STATE_OFF        0U
#define AUDIO_STATE_STARTING   1U
#define AUDIO_STATE_RECORDING  2U
#define AUDIO_STATE_STOPPED    3U
#define AUDIO_STATE_FAULT      4U

/* AUDIO_STATUS_RESPONSE byte 3, the session's first fault. OPEN: a directory
 * or file could not be made. CAPTURE: the microphone would not start. WRITE:
 * a write to the card failed. OVERRUN: the card fell further behind than the
 * ring holds, and the gap was written as silence. FP_BEHIND: the fingerprint
 * fell further behind than the ring holds, and skipped chunks. OVERRUN and
 * FP_BEHIND are degradations: recording carries on. */
#define AUDIO_FAULT_NONE       0U
#define AUDIO_FAULT_OPEN       1U
#define AUDIO_FAULT_CAPTURE    2U
#define AUDIO_FAULT_WRITE      3U
#define AUDIO_FAULT_OVERRUN    4U
#define AUDIO_FAULT_FP_BEHIND  5U

#if defined(SHIMMER3R)
void ShimAudio_onSessionStart(uint64_t dataFileTs);
void ShimAudio_onFileSplit(uint64_t dataFileTs);
void ShimAudio_onSyncTick(void);
void ShimAudio_onSessionStop(void);
/* TASK_AUDIO */
void ShimAudio_process(void);
/* AUDIO_STATUS_RECORDING and AUDIO_STATUS_FAULT, for status byte 1 */
uint8_t ShimAudio_statusBits(void);
/* GET_AUDIO_STATUS_COMMAND: writes AUDIO_STATUS_LEN bytes, returns it */
uint8_t ShimAudio_buildStatus(uint8_t *buf);
/* The sound-level channel (Audio/shimmer_audio_level.h, DEV-1413): its capture
 * runs while the unit senses, streaming, logging or both, if configuration
 * byte 233 bit 4 is set. Called when sensing's peripherals start and stop. */
void ShimAudio_onSensingStart(void);
void ShimAudio_onSensingStop(void);
/* TASK_AUDIO_LEVEL */
void ShimAudio_levelProcess(void);
#else
static inline void ShimAudio_onSessionStart(uint64_t dataFileTs)
{
  (void) dataFileTs;
}

static inline void ShimAudio_onFileSplit(uint64_t dataFileTs)
{
  (void) dataFileTs;
}

static inline void ShimAudio_onSyncTick(void)
{
}

static inline void ShimAudio_onSessionStop(void)
{
}

static inline void ShimAudio_onSensingStart(void)
{
}

static inline void ShimAudio_onSensingStop(void)
{
}
#endif

#endif /* SHIMMER_AUDIO_H_ */
