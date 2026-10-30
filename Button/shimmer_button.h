/*
 * shimmer_button.h
 *
 *  Created on: 30 Jun 2025
 *      Author: MarkNolan
 *
 * The user button. Its interrupt only queues each edge with its time
 * (ShimEvt_isrPushButtonEdge); this runs in the main loop on those edges:
 * debounce and gestures (Button/shimmer_button_gesture.h), the button state
 * the LEDs show, the event marker records, and starting or stopping logging.
 */

#ifndef LOG_AND_STREAM_COMMON_BUTTON_SHIMMER_BUTTON_H_
#define LOG_AND_STREAM_COMMON_BUTTON_SHIMMER_BUTTON_H_

#include "stdint.h"

void ShimBtn_init(void);
/* One edge from the interrupt: isPress is 1 for a press, 0 for a release */
void ShimBtn_processEdge(uint64_t ts, uint8_t isPress);
/* 1 while an edge ignored as bounce still has to be checked against the pin */
uint8_t ShimBtn_isRecheckPending(void);
void ShimBtn_recheckIfDue(uint64_t now);
/* The debounced level: 1 = pressed */
uint8_t ShimBtn_isPressed(void);

#endif /* LOG_AND_STREAM_COMMON_BUTTON_SHIMMER_BUTTON_H_ */
