/*
 * shimmer_audio_mode.c
 *
 * See shimmer_audio_mode.h for the byte layout.
 */

#include <stdint.h>

#include "shimmer_audio_mode.h"

uint8_t ShimAudioMode_correct(uint8_t raw)
{
  uint8_t mode = raw & AUDIO_MODE_MASK;
  uint8_t gain = (uint8_t) ((raw & AUDIO_MODE_GAIN_MASK) >> AUDIO_MODE_GAIN_SHIFT);

  if ((raw & AUDIO_MODE_INVALID) || (gain != AUDIO_GAIN_DEFAULT)
      || !(AUDIO_MODES_BUILT & (1U << mode)))
  {
    return 0;
  }
  return raw & (AUDIO_MODE_MASK | AUDIO_MODE_GAIN_MASK | AUDIO_SOUND_LEVEL);
}

uint8_t ShimAudioMode_mode(uint8_t raw)
{
  return ShimAudioMode_correct(raw) & AUDIO_MODE_MASK;
}

uint8_t ShimAudioMode_gainPreset(uint8_t raw)
{
  return (uint8_t) ((ShimAudioMode_correct(raw) & AUDIO_MODE_GAIN_MASK) >> AUDIO_MODE_GAIN_SHIFT);
}

uint8_t ShimAudioMode_recordsFingerprint(uint8_t raw)
{
  uint8_t mode = ShimAudioMode_mode(raw);

  return (mode == AUDIO_MODE_FINGERPRINT) || (mode == AUDIO_MODE_FINGERPRINT_RAW);
}

uint8_t ShimAudioMode_recordsRawAudio(uint8_t raw)
{
  uint8_t mode = ShimAudioMode_mode(raw);

  return (mode == AUDIO_MODE_FINGERPRINT_RAW) || (mode == AUDIO_MODE_RAW);
}

uint8_t ShimAudioMode_soundLevel(uint8_t raw)
{
  return (ShimAudioMode_correct(raw) & AUDIO_SOUND_LEVEL) != 0U;
}

uint8_t ShimAudioMode_withMode(uint8_t raw, int value)
{
  if ((value < 0) || (value > (int) AUDIO_MODE_MASK))
  {
    return raw | AUDIO_MODE_INVALID;
  }
  return (uint8_t) ((raw & ~AUDIO_MODE_MASK) | (uint8_t) value);
}

uint8_t ShimAudioMode_withSoundLevel(uint8_t raw, int value)
{
  if ((value < 0) || (value > 1))
  {
    return raw | AUDIO_MODE_INVALID;
  }
  return value ? (uint8_t) (raw | AUDIO_SOUND_LEVEL) : (uint8_t) (raw & ~AUDIO_SOUND_LEVEL);
}
