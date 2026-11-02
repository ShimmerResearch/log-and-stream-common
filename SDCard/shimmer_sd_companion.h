/*
 * shimmer_sd_companion.h
 *
 * Files recorded alongside each SD data file, in trees that mirror data/:
 * events/ (the event channel, Events/shimmer_events.h) and, on Shimmer3R,
 * audio/ (Audio/shimmer_audio.h, DEV-99). The data file reports its lifecycle
 * to all of them here, in one place, so a new kind of companion file is one
 * line in each function below.
 *
 * As for the event file: session start, file split and the one-minute sync
 * only queue work for the companion's own task, so the data file is never
 * delayed. Session stop runs at once, so every companion is closed before the
 * card can be handed to a host.
 */

#ifndef SHIMMER_SD_COMPANION_H_
#define SHIMMER_SD_COMPANION_H_

#include <stdint.h>

#include "Audio/shimmer_audio.h"
#include "Events/shimmer_events.h"

/* The data file has just opened: its initial timestamp */
static inline void ShimSdCompanion_onSessionStart(uint64_t dataFileTs)
{
  ShimEvt_onSessionStart(dataFileTs);
  ShimAudio_onSessionStart(dataFileTs);
}

/* The data file has split into its next NNN: the new file's initial timestamp */
static inline void ShimSdCompanion_onFileSplit(uint64_t dataFileTs)
{
  ShimEvt_onFileSplit(dataFileTs);
  ShimAudio_onFileSplit(dataFileTs);
}

/* The data file's one-minute sync */
static inline void ShimSdCompanion_onSyncTick(void)
{
  ShimEvt_onSyncTick();
  ShimAudio_onSyncTick();
}

/* Logging is stopping: close everything now */
static inline void ShimSdCompanion_onSessionStop(void)
{
  ShimEvt_onSessionStop();
  ShimAudio_onSessionStop();
}

#if defined(SHIMMER3R)
/* <root>/<experiment>/<session>/<NNN><ext>, beside the current data file
 * data/<experiment>/<session>/<NNN>. Returns 0 if that does not fit in len.
 * The event file still builds its own path the same way
 * (Events/shimmer_events.c); it can move to this once DEV-971 has merged. */
uint8_t ShimSdCompanion_path(const char *root, const char *ext, char *out, uint16_t len);
/* Creates each directory along path, not its last element. 0 on a failure. */
uint8_t ShimSdCompanion_makeDirs(char *path);
#endif

#endif /* SHIMMER_SD_COMPANION_H_ */
