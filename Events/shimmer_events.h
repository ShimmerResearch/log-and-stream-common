/*
 * shimmer_events.h
 *
 * The event channel: input edges recorded with their own timestamps,
 * independent of the sampling rate, in an event file beside each data file
 * (docs/SHIMMER3_EVENT_CHANNEL.md). Two inputs: the user button, recorded
 * while buttonEventMarkerEnable is set, and a TTL input, recorded while
 * ttlInputEnable is set. The TTL input's edges are latched by a timer capture,
 * which is armed only for a logging session (Board_evtTtlArm).
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
#define EVT_RING_LEN               16U
/* Records kept while no event file is open: the gesture that started logging,
 * and anything between the data file opening and the event file opening */
#define EVT_PREROLL_LEN            8U
/* How long before the data file opened a kept record may be and still go into
 * the event file: long enough for the double press that started logging */
#define EVT_PREROLL_TICKS          65536UL /* 2 s */
/* At the data file's one-minute sync, a heartbeat is written if nothing else
 * has been for this long, so a file that stops growing has stopped recording */
#define EVT_HEARTBEAT_TICKS        1966080UL /* 60 s */
/* "events/" in place of the data file's "data/": two bytes longer than the
 * data file's name buffer */
#define EVT_PATH_LEN               72U
/* TTL flood control: more edges than this in one 100 ms tick - a chattering or
 * mis-wired input - and the capture interrupt is switched off for
 * EVT_TTL_THROTTLE_TICKS, so it cannot starve everything else */
#define EVT_TTL_MAX_EDGES_PER_TICK 32U
#define EVT_TTL_THROTTLE_TICKS     10U /* 1 s */
/* Passed in with a TTL edge, never written to the file: the timer overwrote at
 * least one edge before this one */
#define EVT_RAW_EDGES_LOST         0x40U
/* Records waiting for room in the Bluetooth TX ring, and frames sent per pass
 * of the task. A record that finds the queue full is lost to the link only -
 * the event file still has it - and the gap shows in the frames' sequence
 * numbers. Shimmer3's TX ring is 256 bytes and shared with the data stream, so
 * one frame at a time there, and RAM keeps its queue to a double press. */
#if defined(SHIMMER3)
#define EVT_BT_QUEUE_LEN       4U
#define EVT_BT_FRAMES_PER_PASS 1U
#else
#define EVT_BT_QUEUE_LEN       8U
#define EVT_BT_FRAMES_PER_PASS 8U
#endif
/* GET_EVENT_CHANNEL_STATUS_COMMAND's response payload, after the opcode */
#define EVT_STATUS_LEN 9U

void ShimEvt_init(void);

/* Interrupt context. Queue a user button edge, timestamped now. Returns 1 if
 * the MCU needs waking to process it. */
uint8_t ShimEvt_isrPushButtonEdge(uint8_t isPress);
/* Interrupt context, from the platform's timer capture. Queue a TTL input edge
 * the platform has already timestamped from its capture register. flags:
 * EVT_FLAG_HW_CAPTURE, EVT_FLAG_LATE, EVT_RAW_EDGES_LOST. Returns 1 if the MCU
 * needs waking to process it. */
uint8_t ShimEvt_isrPushTtlEdge(uint8_t level, uint64_t ts, uint8_t flags);
/* Interrupt context, from the 100 ms blink tick. Returns 1 if the MCU needs
 * waking for a debounce recheck or to re-arm a throttled TTL input. */
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

/* SET_FEATURE / FEATURE_EVENT_STREAM, and its reset on connect, disconnect and
 * FEATURE_NONE. Any context: the task applies it. While on, every record is
 * also sent to the host, and the inputs that are enabled are armed even when
 * nothing is being logged. */
void ShimEvt_setBtStream(uint8_t enable);
/* Interrupt context, from ShimBt_TxCpltCallback: room may have been made for
 * a waiting frame */
void ShimEvt_btTxSpaceAvailable(void);
/* GET_EVENT_CHANNEL_STATUS_COMMAND: writes EVT_STATUS_LEN bytes, returns it */
uint8_t ShimEvt_buildStatus(uint8_t *out);

#endif /* SHIMMER_EVENTS_H_ */
