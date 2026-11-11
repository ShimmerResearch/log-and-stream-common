/*
 * Host-side test for Audio/shimmer_audio_level.c, the Shimmer3R sound level
 * (DEV-1413).
 *
 * The channel's value is a contract with every host that shows it, so it is
 * pinned against references worked out here in double precision:
 * - the filter against IEC 61672's analogue A-weighting curve, by the level of
 *   long sines from 20 Hz to 7 kHz;
 * - the single-precision filter against the same biquads in double, on a mix;
 * - the conversion to hundredths of a dB SPL, including the microphones'
 *   offsets and the clamps;
 * - the state carried from one block to the next.
 *
 * shimmer_audio_level.c needs only the C library and libm. Run by
 * .github/workflows/host-tests.yml.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <math.h>
#include <string.h>

#include "host_test.h"

#include "Audio/shimmer_audio_level.h"

#define FS        16000.0
#define BLOCK     2048U
#define SECONDS_1 16000U
#define PI        3.14159265358979323846

static int16_t buf[4U * SECONDS_1];

static void expectNear(const char *what, double got, double want, double tol)
{
  if (fabs(got - want) > tol)
  {
    printf("    got %.4f, want %.4f +/- %.4f\n", got, want, tol);
  }
  expectTrue(what, fabs(got - want) <= tol);
}

/* IEC 61672-1 Annex E: A(f) = 20 log10(RA(f)) + 2.00 dB */
static double iecA(double f)
{
  const double f1 = 20.598997, f2 = 107.65265, f3 = 737.86223, f4 = 12194.217;
  double f2sq = f * f;
  double ra = (f4 * f4 * f2sq * f2sq)
      / ((f2sq + f1 * f1) * sqrt((f2sq + f2 * f2) * (f2sq + f3 * f3)) * (f2sq + f4 * f4));

  return 20.0 * log10(ra) + 2.00;
}

static void sine(int16_t *x, uint32_t n, double hz, double amplitude)
{
  uint32_t i;

  for (i = 0U; i < n; i++)
  {
    x[i] = (int16_t) lrint(amplitude * sin(2.0 * PI * hz * (double) i / FS));
  }
}

/* dB against a full-scale sine of the A-weighted level of x, after the filter
 * has settled on its first half */
static double levelDb(const int16_t *x, uint32_t n)
{
  ShimAudioLevel_Filter f;
  float ms;

  ShimAudioLevel_reset(&f);
  (void) ShimAudioLevel_filter(&f, x, n / 2U);
  ms = ShimAudioLevel_filter(&f, x + n / 2U, n - n / 2U);
  return 10.0 * log10((double) ms) - 10.0 * log10(32768.0 * 32768.0 / 2.0);
}

static void test_a_weighting_curve(void)
{
  /* Third-octave centres; 4 s each, so even 20 Hz settles */
  static const double freqs[] = { 20.0, 31.5, 63.0, 125.0, 250.0, 500.0, 1000.0,
    2000.0, 3150.0, 4000.0, 5000.0, 6300.0, 7000.0 };
  uint32_t i;
  double amp = 32768.0 * pow(10.0, -20.0 / 20.0); /* -20 dBFS */

  testCase("the filter follows IEC 61672's A-weighting from 20 Hz to 7 kHz");
  for (i = 0U; i < sizeof(freqs) / sizeof(freqs[0]); i++)
  {
    char what[64];

    sine(buf, 4U * SECONDS_1, freqs[i], amp);
    snprintf(what, sizeof(what), "A-weighting at %.1f Hz", freqs[i]);
    expectNear(what, levelDb(buf, 4U * SECONDS_1) + 20.0, iecA(freqs[i]), 0.10);
  }
  sine(buf, 4U * SECONDS_1, 1000.0, amp);
  expectNear("0 dB at 1 kHz", levelDb(buf, 4U * SECONDS_1) + 20.0, 0.0, 0.02);
}

/* The same three biquads, gain and order in double */
static double referenceMeanSquare(const int16_t *x, uint32_t n)
{
  static const double b[3][3]
      = { { 1, -2, 1 }, { 1, -2, 1 }, { 1, 0.4106722890, 0.0421629322 } };
  static const double a[3][2] = { { -1.9838867573, 0.9839516664 },
    { -1.7055096337, 0.7159875753 }, { 0.2655271310, 0.0176261643 } };
  double s[3][2] = { { 0 } };
  double sum = 0.0, v, y;
  uint32_t i, k;

  for (i = 0U; i < n; i++)
  {
    v = x[i];
    for (k = 0U; k < 3U; k++)
    {
      y = b[k][0] * v + s[k][0];
      s[k][0] = b[k][1] * v - a[k][0] * y + s[k][1];
      s[k][1] = b[k][2] * v - a[k][1] * y;
      v = y;
    }
    v *= 0.9382065491;
    sum += v * v;
  }
  return sum / n;
}

static void test_single_matches_double(void)
{
  ShimAudioLevel_Filter f;
  uint32_t i, n = 4U * SECONDS_1;
  uint32_t seed = 12345U;
  double ref;

  testCase("single precision is within 0.01 dB of double");
  /* A mix: speech-band tones, a low hum and pseudo-random noise */
  for (i = 0U; i < n; i++)
  {
    double t = (double) i / FS;
    seed = seed * 1103515245U + 12345U;
    buf[i] = (int16_t) lrint(3000.0 * sin(2 * PI * 220.0 * t)
        + 1500.0 * sin(2 * PI * 1870.0 * t) + 4000.0 * sin(2 * PI * 50.0 * t)
        + (double) ((int32_t) (seed >> 16) % 2000 - 1000));
  }
  ShimAudioLevel_reset(&f);
  ref = referenceMeanSquare(buf, n);
  expectNear("mix", 10.0 * log10((double) ShimAudioLevel_filter(&f, buf, n)),
      10.0 * log10(ref), 0.01);

  sine(buf, n, 25.0, 30000.0);
  ShimAudioLevel_reset(&f);
  ref = referenceMeanSquare(buf, n);
  expectNear("loud 25 Hz, the poles nearest z = 1",
      10.0 * log10((double) ShimAudioLevel_filter(&f, buf, n)), 10.0 * log10(ref), 0.01);
}

static void test_state_carries_over(void)
{
  ShimAudioLevel_Filter whole, parts;
  float a, b1, b2;

  testCase("one block filtered whole or in two calls gives the same level");
  sine(buf, BLOCK * 4U, 440.0, 8000.0);
  ShimAudioLevel_reset(&whole);
  ShimAudioLevel_reset(&parts);
  (void) ShimAudioLevel_filter(&whole, buf, BLOCK);
  (void) ShimAudioLevel_filter(&parts, buf, BLOCK);
  a = ShimAudioLevel_filter(&whole, buf + BLOCK, BLOCK);
  b1 = ShimAudioLevel_filter(&parts, buf + BLOCK, BLOCK / 2U);
  b2 = ShimAudioLevel_filter(&parts, buf + BLOCK + BLOCK / 2U, BLOCK / 2U);
  expectNear("mean of the halves", 10.0 * log10((b1 + b2) / 2.0f), 10.0 * log10(a), 0.001);
  expectTrue("same state after", memcmp(&whole, &parts, sizeof(whole)) == 0);

  testCase("reset clears the state");
  ShimAudioLevel_reset(&parts);
  memset(&whole, 0, sizeof(whole));
  expectTrue("all zero", memcmp(&whole, &parts, sizeof(whole)) == 0);
  expectTrue("no samples, no level", ShimAudioLevel_filter(&parts, buf, 0U) == 0.0f);
}

static void test_centi_db(void)
{
  double ms;

  testCase("hundredths of a dB SPL, with each microphone's offset");
  /* A 94 dB SPL tone puts the MP23DB01HP at -14.1 dBFS, the IM68D121 at
   * -13.7 (docs/SHIMMER3R_AUDIO.md section 4) */
  ms = pow(32768.0 * pow(10.0, -14.1 / 20.0), 2.0) / 2.0;
  expectNear("MP23DB01HP, 94 dB SPL",
      ShimAudioLevel_centiDb((float) ms, AUDIO_LEVEL_OFFSET_MP23DB01HP), 9400.0, 1.0);
  ms = pow(32768.0 * pow(10.0, -13.7 / 20.0), 2.0) / 2.0;
  expectNear("IM68D121, 94 dB SPL",
      ShimAudioLevel_centiDb((float) ms, AUDIO_LEVEL_OFFSET_IM68D121), 9400.0, 1.0);
  expectX("offset values", AUDIO_LEVEL_OFFSET_MP23DB01HP, 10810);
  expectX("offset values", AUDIO_LEVEL_OFFSET_IM68D121, 10770);

  testCase("a whole 1 kHz sine at -20 dBFS through the filter");
  sine(buf, 4U * SECONDS_1, 1000.0, 32768.0 * 0.1);
  {
    ShimAudioLevel_Filter f;

    ShimAudioLevel_reset(&f);
    (void) ShimAudioLevel_filter(&f, buf, 2U * SECONDS_1);
    expectNear("88.10 dB SPL on an MP23DB01HP",
        ShimAudioLevel_centiDb(ShimAudioLevel_filter(&f, buf + 2U * SECONDS_1, 2U * SECONDS_1),
            AUDIO_LEVEL_OFFSET_MP23DB01HP),
        8810.0, 2.0);
  }

  testCase("the clamps, and never the no-level value");
  expectU("silence",
      (uint16_t) ShimAudioLevel_centiDb(0.0f, AUDIO_LEVEL_OFFSET_MP23DB01HP), 0);
  expectU("NaN", (uint16_t) ShimAudioLevel_centiDb(NAN, AUDIO_LEVEL_OFFSET_MP23DB01HP), 0);
  expectU("negative",
      (uint16_t) ShimAudioLevel_centiDb(-1.0f, AUDIO_LEVEL_OFFSET_MP23DB01HP), 0);
  expectU("below 0 dB SPL",
      (uint16_t) ShimAudioLevel_centiDb(1e-6f, AUDIO_LEVEL_OFFSET_MP23DB01HP), 0);
  expectU("far too loud",
      (uint16_t) ShimAudioLevel_centiDb(1e36f, AUDIO_LEVEL_OFFSET_MP23DB01HP), 32767);
  expectX("no level is -32768", (uint16_t) AUDIO_LEVEL_NONE, 0x8000);
  /* The loudest an int16 capture can be: a full-scale square wave, +3 dB */
  ms = 32768.0 * 32768.0;
  expectNear("full-scale square wave",
      ShimAudioLevel_centiDb((float) ms, AUDIO_LEVEL_OFFSET_MP23DB01HP),
      10810.0 + 301.0, 1.0);
}

int main(void)
{
  printf("shimmer_audio_level host tests\n\n");
  hostTestSilenceUnused();

  test_a_weighting_curve();
  test_single_matches_double();
  test_state_carries_over();
  test_centi_db();

  return hostTestReport("shimmer_audio_level");
}

#endif /* SHIMMER_HOST_TEST */
