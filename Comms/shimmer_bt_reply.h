/*
 * shimmer_bt_reply.h
 *
 * Command replies on the Bluetooth link. Data packets and replies share one TX
 * ring, and once a stream outruns the link the data fills it. A data packet
 * that does not fit is a gap the host can see in the timestamps; a reply that
 * does not fit used to vanish without trace, so the host timed out on a
 * command the sensor had run. This module keeps room in the ring for replies,
 * and holds back a reply that still does not fit until it does.
 *
 * Self-contained apart from the two ring functions below, so the host test
 * suite can build it against a fake ring.
 */

#ifndef SHIMMER_BT_REPLY_H_
#define SHIMMER_BT_REPLY_H_

#include <stdint.h>

#include "log_and_stream_definitions.h"

/* TX-ring space a data packet may not use, so a reply arriving while a stream
 * outruns the link still fits.
 *
 * Shimmer3R: matches SD_FT_TX_RESERVE, the headroom the SD file transfer
 * already keeps. It covers every reply except a long SD directory listing,
 * which is held back instead.
 *
 * Shimmer3: the ring is 256 bytes, so only the short replies are covered - an
 * ACK, a NACK, a status push or a single-value GET, with CRC. A longer reply
 * is held back rather than costing the stream a large part of the ring. */
#if defined(SHIMMER3R)
#define BT_TX_REPLY_RESERVE 256U
#else
#define BT_TX_REPLY_RESERVE 16U
#endif

/* A held reply waits here. Room for the largest reply with its CRC, plus a
 * status push queued behind it. */
#define BT_PENDING_REPLY_SIZE (RESPONSE_PACKET_SIZE + 2U + 16U)

/* Queue a command reply: an ACK or NACK, a response, a status push. One that
 * does not fit in the ring now is held, and goes out ahead of any further data
 * packet. Returns 0 if queued or held, 1 if dropped - only when the hold
 * buffer is full too. Main-loop context only. */
uint8_t ShimBtReply_send(uint8_t *buf, uint16_t len);

/* Move a held reply into the ring if it now fits. Runs as TASK_BT_REPLY_RETRY,
 * which the TX-complete path sets while a reply is held. */
void ShimBtReply_retry(void);

uint8_t ShimBtReply_isPending(void);

/* Gate for data packets and SD file-transfer data frames: refused while a reply
 * is held, so the ring drains to make room for it, and when the packet would
 * eat into BT_TX_REPLY_RESERVE. */
uint8_t ShimBtReply_isRoomForSensorData(uint16_t len);

/* Forget a held reply: the TX ring has been cleared or the link is gone. */
void ShimBtReply_reset(void);

/* Replies dropped because the hold buffer was full. Expected to stay 0, since a
 * host waits for one reply before sending its next command; readable over SWD. */
uint16_t ShimBtReply_getDropCount(void);

/* Provided by Comms/shimmer_bt_uart.c (the fake ring in the host tests). */
uint8_t ShimBt_writeReplyToTxBuf(uint8_t *buf, uint16_t len);
uint16_t ShimBt_getSpaceInBtTxBuf(void);

#endif /* SHIMMER_BT_REPLY_H_ */
