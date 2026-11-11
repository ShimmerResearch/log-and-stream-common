/*
 * shimmer_audio_level.h
 *
 * The sound level (DEV-1413): the A-weighted level of each 128 ms block of
 * the Shimmer3R's 16 kHz microphone capture, sent in every data packet as the
 * SOUND_LEVEL channel while configuration byte 233 bit 4 is set
 * (Configuration/shimmer_audio_mode.h). The arithmetic only: the platform runs
 * it on each block as it completes (shimmer3r-firmware's
 * Shimmer_Driver/hal_AudioLevel.c). Kept free of any platform dependency so
 * it can be host-tested against a reference (Test/host/test_audio_level.c).
 * docs/SHIMMER3R_AUDIO.md has the channel and what its value means.
 *
 * The level is LAeq over the block: the mean square of the A-weighted samples,
 * in dB against a full-scale sine (AES17), plus the microphone's offset from
 * that to dB SPL. The offsets are worked out from the datasheets and the
 * capture's gain, not measured against a calibrated meter, so the level is
 * nominal: good for changes and comparisons on one unit, within a few dB in
 * absolute terms.
 *
 * The A-weighting is IEC 61672's analogue curve as three biquads at 16 kHz:
 * the low-frequency poles by the bilinear transform, the high-frequency
 * section fitted, since the curve's 12.2 kHz poles lie above the 8 kHz
 * Nyquist frequency. It is within 0.06 dB of the curve from 20 Hz to 7 kHz.
 */

#ifndef SHIMMER_AUDIO_LEVEL_H_
#define SHIMMER_AUDIO_LEVEL_H_

#include <stdint.h>

/* The channel's value when there is no level: capture is starting (its first
 * 256 ms), stopped or failed */
#define AUDIO_LEVEL_NONE              INT16_MIN /* -32768, 0x8000 */

/* The microphone's offset from dB against a full-scale sine to dB SPL, in
 * hundredths of a dB, at the capture's gain for that part
 * (docs/SHIMMER3R_AUDIO.md section 4: a 94 dB SPL tone at -14.1 and
 * -13.7 dBFS) */
#define AUDIO_LEVEL_OFFSET_MP23DB01HP 10810 /* ADF SCALE code 6 */
#define AUDIO_LEVEL_OFFSET_IM68D121   10770 /* ADF SCALE code 1 */

typedef struct
{
  float s[3][2]; /* the three biquads' states, transposed direct form II */
} ShimAudioLevel_Filter;

void ShimAudioLevel_reset(ShimAudioLevel_Filter *f);

/* Runs count samples through the A-weighting filter, carrying its state on
 * from the last call, and returns their mean square in (sample units)^2 */
float ShimAudioLevel_filter(ShimAudioLevel_Filter *f, const int16_t *x, uint32_t count);

/* A mean square from ShimAudioLevel_filter() as the channel's value: dB SPL in
 * hundredths, with offset from AUDIO_LEVEL_OFFSET_*. Never AUDIO_LEVEL_NONE:
 * clamped to 0 (digital silence) and to 32767. */
int16_t ShimAudioLevel_centiDb(float meanSquare, int16_t offset);

#endif /* SHIMMER_AUDIO_LEVEL_H_ */
