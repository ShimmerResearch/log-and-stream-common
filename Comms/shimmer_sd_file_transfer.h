/*
 * shimmer_sd_file_transfer.h
 *
 * SD-card file transfer over the Bluetooth command channel.
 *
 * Host-driven, stateless protocol: the host walks the card with
 * SD_LIST_DIR/SD_FILE_STAT, then pulls file content in windows with
 * SD_FILE_READ. Each accepted read streams the requested window as
 * self-CRC'd SD_FILE_DATA_RESPONSE frames from a low-priority task
 * (TASK_SD_FILE_TRANSFER) that only ever copies what fits in the BT TX
 * ring per pass — pacing is inherited from the UART TX-complete interrupt
 * and the module's RTS/CTS flow control. Resume after any interruption is
 * a fresh SD_FILE_READ from the last byte offset the host holds.
 *
 * v1 policy: transfers are only served while the device is idle (not
 * sensing/logging/streaming) and the MCU owns the SD card (not docked,
 * no USB-C). All one-shot responses report denial in-band via a status
 * byte so the host never has to time out on a gate.
 *
 *  Created on: 18 Aug 2026
 *      Author: MarkNolan
 */

#ifndef SHIMMER3_COMMON_SOURCE_COMMS_SHIMMER_SD_FILE_TRANSFER_H_
#define SHIMMER3_COMMON_SOURCE_COMMS_SHIMMER_SD_FILE_TRANSFER_H_

#include <stdint.h>

/* Status byte shared by the one-shot responses (SD_LIST_DIR_RESPONSE,
 * SD_FILE_STAT_RESPONSE, SD_FREE_SPACE_RESPONSE, SD_DELETE_RESPONSE).
 * Values 0x01-0x13 are raw FatFs FRESULT codes passed through. */
#define SD_FT_STATUS_OK             0x00
#define SD_FT_STATUS_SD_UNAVAILABLE 0xF0 /* docked / USB-C / no card / bad card */
#define SD_FT_STATUS_BUSY           0xF1 /* sensing / logging / streaming */
#define SD_FT_STATUS_BAD_ARGS       0xF2

/* Transfer status codes carried in SD_FILE_STATUS_RESPONSE (0xC6) frames */
#define SD_FT_XFER_WINDOW_COMPLETE 0
#define SD_FT_XFER_EOF             1
#define SD_FT_XFER_HOST_ABORT      2
#define SD_FT_XFER_SD_LOST         3
#define SD_FT_XFER_FS_ERROR        4
#define SD_FT_XFER_SUPERSEDED      5
#define SD_FT_XFER_DENIED          6
#define SD_FT_XFER_NOT_FOUND       7

/* Directory entry attribute bits in SD_LIST_DIR_RESPONSE */
#define SD_FT_ATTR_DIR            (1 << 0)
#define SD_FT_ATTR_NAME_TRUNCATED (1 << 1)

#define SD_FT_MAX_PATH_LEN          96
#define SD_FT_LIST_MAX_ENTRIES      16
#define SD_FT_LIST_NAME_MAX         64
#define SD_FT_BLOCK_PAYLOAD_MIN     64
#define SD_FT_BLOCK_PAYLOAD_MAX     1024
#define SD_FT_BLOCK_PAYLOAD_DEFAULT 512

/* Data frame: [0x8A][0xC5][sessionId][seq u16][len u16][payload][crc u16]  */
#define SD_FT_DATA_FRAME_HEADER_LEN 7
#define SD_FT_FRAME_CRC_LEN         2
/* Status frame: [0x8A][0xC6][sessionId][status][nextOffset u32][crc u16]  */
#define SD_FT_STATUS_FRAME_LEN      (8 + SD_FT_FRAME_CRC_LEN)

/* TX-ring headroom kept free for command responses while streaming */
#define SD_FT_TX_RESERVE            256
/* Blocks pushed per task invocation before yielding to lower-priority tasks */
#define SD_FT_BLOCKS_PER_PASS       4

void ShimSdFileTransfer_init(void);

/* Silent teardown: close the cached file, drop any active window and pending
 * status frames without emitting anything (BT link gone or power policy). */
void ShimSdFileTransfer_reset(void);

/* Teardown + queue an SD_FILE_STATUS_RESPONSE frame carrying the given
 * SD_FT_XFER_* code (link still up: host abort, dock/USB preemption). Safe
 * to call in any state; a no-op when no window is active. */
void ShimSdFileTransfer_abort(uint8_t xferStatus);

/* TASK_SD_FILE_TRANSFER handler */
void ShimSdFileTransfer_run(void);

/* Called from the BT TX-complete path when ring space has been freed */
void ShimSdFileTransfer_txSpaceAvailableEvent(void);

/* Command staging, called from ShimBt_processCmd(). Arguments are copied
 * out of the volatile BT args buffer; validation failures surface in the
 * response status byte, never as silence. */
void ShimSdFileTransfer_stageListDir(uint8_t *argsPtr);
void ShimSdFileTransfer_stagePath(uint8_t *argsPtr); /* STAT + DELETE */
void ShimSdFileTransfer_startRead(uint8_t *argsPtr);

/* Response builders, called from ShimBt_sendRsp(). Write the response
 * (opcode + payload) to dst and return the number of bytes written. Each
 * one-shot response is kept short enough to fit an un-widened response
 * path (<= ~250 bytes including ACK + CRC) so the same wire format can
 * later serve SHIMMER3's 133-byte response budget. */
uint16_t ShimSdFileTransfer_buildListDirRsp(uint8_t *dst);
uint16_t ShimSdFileTransfer_buildStatRsp(uint8_t *dst);
uint16_t ShimSdFileTransfer_buildFreeSpaceRsp(uint8_t *dst);
uint16_t ShimSdFileTransfer_buildDeleteRsp(uint8_t *dst);

#endif /* SHIMMER3_COMMON_SOURCE_COMMS_SHIMMER_SD_FILE_TRANSFER_H_ */
