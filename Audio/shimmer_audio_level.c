/*
 * shimmer_audio_level.c
 *
 * See shimmer_audio_level.h.
 *
 * The filter: three biquads in transposed direct form II, each with b0 = 1,
 * then one gain that makes it 0 dB at 1 kHz.
 *   section 0, 1: the 20.6 Hz pole pair and the 107.7/737.9 Hz poles, each
 *                 with a double zero at z = 1 (bilinear transform)
 *   section 2:    the 12.2 kHz pole pair, fitted to the analogue curve from
 *                 1.5 to 7.2 kHz (a double zero at z = -0.2053, a double pole
 *                 at z = -0.1328)
 * In single precision: the host test holds it within 0.01 dB of the same
 * filter in double, and within 0.1 dB of the curve from 20 Hz to 7 kHz
 * (Test/host/test_audio_level.c).
 */

#include "shimmer_audio_level.h"

#if defined(SHIMMER3R) || defined(SHIMMER_HOST_TEST)

#include <math.h>
#include <string.h>

/* 10 log10 of a full-scale sine's mean square, 32768^2 / 2 */
#define FULL_SCALE_SINE_DB 87.29866f

#define S0_A1              (-1.9838867573f)
#define S0_A2              (0.9839516664f)
#define S1_A1              (-1.7055096337f)
#define S1_A2              (0.7159875753f)
#define S2_B1              (0.4106722890f)
#define S2_B2              (0.0421629322f)
#define S2_A1              (0.2655271310f)
#define S2_A2              (0.0176261643f)
#define GAIN               (0.9382065491f)

void ShimAudioLevel_reset(ShimAudioLevel_Filter *f)
{
  memset(f, 0, sizeof(*f));
}

float ShimAudioLevel_filter(ShimAudioLevel_Filter *f, const int16_t *x, uint32_t count)
{
  float s01 = f->s[0][0], s02 = f->s[0][1];
  float s11 = f->s[1][0], s12 = f->s[1][1];
  float s21 = f->s[2][0], s22 = f->s[2][1];
  float sum = 0.0f;
  float v, y;
  uint32_t i;

  for (i = 0U; i < count; i++)
  {
    v = (float) x[i];
    /* b = 1, -2, 1 */
    y = v + s01;
    s01 = s02 - 2.0f * v - S0_A1 * y;
    s02 = v - S0_A2 * y;
    v = y;
    y = v + s11;
    s11 = s12 - 2.0f * v - S1_A1 * y;
    s12 = v - S1_A2 * y;
    v = y;
    y = v + s21;
    s21 = s22 + S2_B1 * v - S2_A1 * y;
    s22 = S2_B2 * v - S2_A2 * y;
    y *= GAIN;
    sum += y * y;
  }
  f->s[0][0] = s01;
  f->s[0][1] = s02;
  f->s[1][0] = s11;
  f->s[1][1] = s12;
  f->s[2][0] = s21;
  f->s[2][1] = s22;
  return count != 0U ? sum / (float) count : 0.0f;
}

int16_t ShimAudioLevel_centiDb(float meanSquare, int16_t offset)
{
  float c;

  if (!(meanSquare > 0.0f)) /* also NaN */
  {
    return 0;
  }
  c = (10.0f * log10f(meanSquare) - FULL_SCALE_SINE_DB) * 100.0f + (float) offset;
  if (c <= 0.0f)
  {
    return 0;
  }
  if (c >= 32767.0f)
  {
    return 32767;
  }
  return (int16_t) (c + 0.5f);
}

#endif /* SHIMMER3R || SHIMMER_HOST_TEST */
