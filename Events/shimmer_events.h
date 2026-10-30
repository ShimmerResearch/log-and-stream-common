/*
 * shimmer_events.h
 *
 * The event channel: input edges recorded with their own timestamps,
 * independent of the sampling rate, in an event file beside each data file
 * (docs/SHIMMER3_EVENT_CHANNEL.md). The input today is the user button, whose
 * edges are recorded while buttonEventMarkerEnable is set.
 *
 * Interrupt handlers only queue an edge with its time. Everything else runs in
 * the main loop, in TASK_EVENT_CHANNEL, the lowest-priority task. The event
 * file's open, split and sync are deferred to that task so they never delay
 * the data file; only the close at the end of a session happens at once.
 *
 * The event file borrows sdlog.cfg's file object for the length of a logging
 * session (ShimSdCfgFile_lendFil): Shimmer3 has no RAM for another, and
 * sdlog.cfg is never read or written while logging.
 */

#ifndef SHIMMER_EVENTS_H_
#define SHIMMER_EVENTS_H_

#include <stdint.h>

/* Edges an interrupt can queue before the main loop takes them. Divides 256,
 * so the free-running ring indexes stay correct across their wrap. */
#define EVT_RING_LEN        16U
/* Records kept while no event file is open: the gesture that started logging,
 * and anything between the data file opening and the event file opening */
#define EVT_PREROLL_LEN     8U
/* How long before the data file opened a kept record may be and still go into
 * the event file: long enough for the double press that started logging */
#define EVT_PREROLL_TICKS   65536UL /* 2 s */
/* At the data file's one-minute sync, a heartbeat is written if nothing else
 * has been for this long, so a file that stops growing has stopped recording */
#define EVT_HEARTBEAT_TICKS 1966080UL /* 60 s */
/* "events/" in place of the data file's "data/": two bytes longer than the
 * data file's name buffer */
#define EVT_PATH_LEN        72U

void ShimEvt_init(void);

/* Interrupt context. Queue a user button edge, timestamped now. Returns 1 if
 * the MCU needs waking to process it. */
uint8_t ShimEvt_isrPushButtonEdge(uint8_t isPress);
/* Interrupt context, from the 100 ms blink tick. Returns 1 if the MCU needs
 * waking for a debounce recheck. */
uint8_t ShimEvt_tick(void);

/* TASK_EVENT_CHANNEL */
void ShimEvt_process(void);

/* From the button module, for each debounced edge while markers are on */
void ShimEvt_recordButtonEdge(uint64_t ts, uint8_t level, uint8_t flags);

/* From the data file's lifecycle (SDCard/shimmer_sd_data_file.c) */
void ShimEvt_onSessionStart(uint64_t fileOpenTs);
void ShimEvt_onFileSplit(uint64_t fileOpenTs);
void ShimEvt_onSyncTick(void);
void ShimEvt_onSessionStop(void);

/* 1 once the event file has failed this session. The data file is never
 * affected. */
uint8_t ShimEvt_isFileError(void);

#endif /* SHIMMER_EVENTS_H_ */
