/*
 * shimmer_audio.c
 *
 * See shimmer_audio.h. The recording itself is the platform's (PlatAudio_*,
 * log_and_stream_externs.h); this decides whether there is one.
 */

#include "shimmer_audio.h"

#if defined(SHIMMER3R)

#include "log_and_stream_externs.h"
#include "log_and_stream_includes.h"

void ShimAudio_onSessionStart(uint64_t dataFileTs)
{
  uint8_t mode = ShimConfig_audioMode();

  if (mode != AUDIO_MODE_OFF)
  {
    PlatAudio_sessionStart(mode, dataFileTs);
  }
}

void ShimAudio_onFileSplit(uint64_t dataFileTs)
{
  PlatAudio_fileSplit(dataFileTs);
}

void ShimAudio_onSyncTick(void)
{
  PlatAudio_syncTick();
}

void ShimAudio_onSessionStop(void)
{
  PlatAudio_sessionStop();
}

void ShimAudio_process(void)
{
  PlatAudio_process();
}

uint8_t ShimAudio_statusBits(void)
{
  return PlatAudio_statusBits();
}

uint8_t ShimAudio_buildStatus(uint8_t *buf)
{
  return PlatAudio_buildStatus(buf);
}

void ShimAudio_onSensingStart(void)
{
  if (ShimConfig_isSoundLevelEnabled())
  {
    PlatAudio_levelStart();
  }
}

void ShimAudio_onSensingStop(void)
{
  PlatAudio_levelStop();
}

void ShimAudio_levelProcess(void)
{
  PlatAudio_levelProcess();
}

#endif /* SHIMMER3R */
