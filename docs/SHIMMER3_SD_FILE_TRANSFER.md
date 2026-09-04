# Shimmer3R SD File Transfer

Reading logged data files off the SD card over the Bluetooth link, without
docking the device or removing the card. Also directory listing, file stat,
free space and delete.

> **Shimmer3R only.** The Shimmer3 build compiles the same file with empty
> function bodies. A Shimmer3 silently ignores the opcodes, so hosts must gate
> on the firmware version rather than probing.

> **Verified against** — the revisions these claims were read from. A pinned
> commit is a citation, not a claim of currency.
>
> - **Firmware:** `log-and-stream-common` @ `f3cf73e` —
>   `Comms/shimmer_sd_file_transfer.{h,c}`, `Comms/shimmer_bt_uart.h`
>   (opcodes), `TaskList/shimmer_taskList.h` (`TASK_SD_FILE_TRANSFER`).
> - **Platform firmware:** `shimmer3r-firmware` @ `a8f105e5`.

**Source references:**

| Layer | File |
|---|---|
| State machine, framing, access control | `Comms/shimmer_sd_file_transfer.c` |
| Constants and status codes | `Comms/shimmer_sd_file_transfer.h` |
| Opcodes | `Comms/shimmer_bt_uart.h` |
| What the transferred files contain | [SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md) |
| Command framing in general | [SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md) |

---

## 1. Opcodes

| Constant | Value | Direction |
|---|---:|---|
| `SD_LIST_DIR_COMMAND` | `0xCC` | Host → device |
| `SD_LIST_DIR_RESPONSE` | `0xC1` | Device → host |
| `SD_FILE_STAT_COMMAND` | `0xC2` | Host → device |
| `SD_FILE_STAT_RESPONSE` | `0xC3` | Device → host |
| `SD_FILE_READ_COMMAND` | `0xC4` | Host → device |
| `SD_FILE_DATA_RESPONSE` | `0xC5` | Device → host |
| `SD_FILE_STATUS_RESPONSE` | `0xC6` | Device → host |
| `SD_TRANSFER_ABORT_COMMAND` | `0xC7` | Host → device |
| `SD_FREE_SPACE_COMMAND` | `0xC8` | Host → device |
| `SD_FREE_SPACE_RESPONSE` | `0xC9` | Device → host |
| `SD_DELETE_COMMAND` | `0xCA` | Host → device |
| `SD_DELETE_RESPONSE` | `0xCB` | Device → host |

> **`SD_LIST_DIR_COMMAND` is `0xCC`, breaking the otherwise contiguous block,
> and the reason is a hardware constraint.** Command opcodes must avoid `0x80`,
> `0xC0` and `0xD0` because the CYW20820 UART receive demux treats those as
> EZ-Serial binary start-of-frame bytes and routes them to the EZ-Serial parser
> instead of the Shimmer command parser. `0xC0` was the natural choice and is
> unusable. Anyone adding an opcode here must observe the same rule.

## 2. Access control

`sdFtAccessCheck` runs before any operation:

| Condition | Result |
|---|---|
| Docked, USB plugged in, card not owned by the MCU, no card, or bad card | `SD_FT_STATUS_SD_UNAVAILABLE` (`0xF0`) |
| Sensing, logging or streaming | `SD_FT_STATUS_BUSY` (`0xF1`) |
| Bad arguments | `SD_FT_STATUS_BAD_ARGS` (`0xF2`) |
| Otherwise | `SD_FT_STATUS_OK` (`0x00`) |

> **Transfer is idle-only.** Any sensing activity blocks it, and the check is
> `sensing || sdLogging || btStreaming` — so a device streaming without logging
> also refuses. A host must stop the trial before retrieving files.

> **Docking blocks transfer too**, because the card is then the host's over
> mass storage, not the MCU's. The two retrieval routes are mutually exclusive
> by design.

### 2.1 Card power bring-up

If the card was powered down when the last Bluetooth connection dropped, the
access check performs a **full bring-up**, not just a rail restore and remount.
The source is explicit about why: re-registering the FatFs volume alone leaves
the SDMMC peripheral holding state from before the card lost power —
`sdPeripheralInit` stays 1 and the handle stays `READY` — so the peripheral
and the card disagree about what has happened.

> **This is the failure that made a Shimmer3R unresponsive until a power cycle
> after a second connection using these commands.** The fix was to
> re-initialise the peripheral rather than assume it was still valid. Anything
> touching this path needs testing across repeated connect/disconnect cycles
> with a transfer in each.

## 3. Reading a file

### 3.1 The request

`SD_FILE_READ_COMMAND` arguments:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Start offset, LE |
| 4 | 4 | Window length, LE |
| 8 | 2 | Block payload length, LE |
| 10 | 1 | Path length |
| 11 | *n* | Path, up to `SD_FT_MAX_PATH_LEN` = 96 |

Block payload length is bounded by `SD_FT_BLOCK_PAYLOAD_MIN` = 64 and
`SD_FT_BLOCK_PAYLOAD_MAX` = 1024, defaulting to
`SD_FT_BLOCK_PAYLOAD_DEFAULT` = 512, and is **masked to a word boundary**
(`&= ~3`) — see §5.

### 3.2 Windows and sessions

A read requests a **window** — a byte range — rather than a whole file. The
device streams that range as a sequence of data frames and then sends a status
frame saying how it ended. The host then requests the next window.

Each window gets a `sessionId` from an incrementing counter. Every frame
carries it, so a host can tell frames of the current window from stragglers of
a previous one.

> **Starting a read while one is in flight supersedes it.** The old window is
> terminated with `SD_FT_XFER_SUPERSEDED` and a *new* session id is issued.
> Frames already queued for the old session may still arrive after the status
> frame — match on session id, do not assume ordering.

### 3.3 Data frame

`SD_FT_DATA_FRAME_HEADER_LEN` = 7:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | `INSTREAM_CMD_RESPONSE` |
| 1 | 1 | `SD_FILE_DATA_RESPONSE` (`0xC5`) |
| 2 | 1 | Session id |
| 3-4 | 2 | Block sequence number, LE |
| 5-6 | 2 | Payload byte count, LE |
| 7 | *n* | Payload |
| 7+*n* | 2 | CRC-16 over the header and payload, LE |

Data frames are sent as `SENSOR_DATA`, so they share the stream path with
sensor packets rather than the command path.

### 3.4 Status frame

`SD_FT_STATUS_FRAME_LEN` = 10:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | `INSTREAM_CMD_RESPONSE` |
| 1 | 1 | `SD_FILE_STATUS_RESPONSE` (`0xC6`) |
| 2 | 1 | Session id |
| 3 | 1 | Transfer status |
| 4-7 | 4 | Next offset, LE |
| 8-9 | 2 | CRC-16 over the first 8 bytes, LE |

Status frames go on the `SHIMMER_CMD` path.

| Status | Value | Meaning |
|---|---:|---|
| `SD_FT_XFER_WINDOW_COMPLETE` | 0 | Window delivered; request the next from *next offset* |
| `SD_FT_XFER_EOF` | 1 | End of file reached |
| `SD_FT_XFER_HOST_ABORT` | 2 | Host sent `SD_TRANSFER_ABORT_COMMAND` |
| `SD_FT_XFER_SD_LOST` | 3 | Card became unavailable mid-transfer |
| `SD_FT_XFER_FS_ERROR` | 4 | FatFs error |
| `SD_FT_XFER_SUPERSEDED` | 5 | A newer read replaced this window |
| `SD_FT_XFER_DENIED` | 6 | Access check failed, or a bad path |
| `SD_FT_XFER_NOT_FOUND` | 7 | Path does not exist |

**Every window ends with exactly one status frame**, whatever the outcome. A
host that has not seen one has not finished.

> **`nextOffset` is where to resume**, not how much was sent. On an error it is
> the offset the failure occurred at, so a host can retry from there rather
> than restarting the file.

## 4. Flow control

The transfer runs as `TASK_SD_FILE_TRANSFER`, deliberately placed at bit 23 —
below `TASK_SDWRITE` and `TASK_DOCK_OR_USB_STATE_CHANGE` — so a bulk transfer
can never starve logging or a dock event
([SHIMMER3_ARCHITECTURE_OVERVIEW.md](SHIMMER3_ARCHITECTURE_OVERVIEW.md) §4.1).

Two mechanisms keep the link responsive:

**A transmit reserve.** `SD_FT_TX_RESERVE` = 256 bytes of the Bluetooth
transmit buffer are kept free at all times:

```c
if (ShimBt_getSpaceInBtTxBuf() < frameLen + SD_FT_TX_RESERVE) {
    waitingForTxSpace = 1;
    return;
}
```

so command responses and status frames always have room even with a window in
flight.

**A per-pass block cap.** `SD_FT_BLOCKS_PER_PASS` = 4 bounds how many blocks
one task execution emits, so the cooperative scheduler is not held for the
whole window.

When the buffer fills, `waitingForTxSpace` is set and
`ShimSdFileTransfer_txSpaceAvailableEvent` re-arms the task from the
transmit-complete path.

> **A frame that fails to send is re-read, not lost.** `readOffset` advances
> only after a successful push, and the file pointer is re-synchronised with
> `f_tell()` / `f_lseek()` on the next pass. The cost is a repeated `f_read`;
> the benefit is that partial sends cannot corrupt the stream.

## 5. The alignment problem

The transmit buffer is a union with a deliberate one-byte offset:

```c
static union {
    uint32_t align;
    uint8_t  bytes[1 + SD_FT_DATA_FRAME_HEADER_LEN + SD_FT_BLOCK_PAYLOAD_MAX + SD_FT_FRAME_CRC_LEN];
} xferBufStorage;

static uint8_t *const xferBuf = &xferBufStorage.bytes[1];
```

Because the header is 7 bytes, payload written at `xferBuf[7]` would land at
absolute offset 7 — not word-aligned. Offsetting the buffer by 1 inside a
word-aligned union puts the payload at absolute offset **8**.

Why it matters: an unaligned destination made the SD DMA path shift the block
content and leave three stale bytes at the tail. The frame CRC is computed
*afterwards*, so it covered the corruption faithfully and **the host could not
detect it**.

Four `_Static_assert`s lock the invariant in place:

```c
_Static_assert((1 + SD_FT_DATA_FRAME_HEADER_LEN) % 4 == 0, ...);
_Static_assert(SD_FT_BLOCK_PAYLOAD_MIN % 4 == 0, ...);
_Static_assert(SD_FT_BLOCK_PAYLOAD_DEFAULT % 4 == 0, ...);
_Static_assert(SD_FT_BLOCK_PAYLOAD_MAX % 4 == 0, ...);
```

> **Changing `SD_FT_DATA_FRAME_HEADER_LEN` breaks the build, by design.** If
> you add a field to the data frame header you must also change the `1 +`
> offset so the sum stays a multiple of 4. The assertion is the only thing
> standing between a header change and silent, CRC-blessed data corruption.

> **The `&= ~3` clamp on the requested block length depends on
> `SD_FT_BLOCK_PAYLOAD_MIN` being word-granular** — masking must not drop a
> minimum-length request below the minimum. That is what the second assertion
> protects.

The diskio scratch-buffer slow path remains as a backstop for destinations that
still end up misaligned, such as a resume from a non-word-aligned byte offset.

## 6. Directory listing, stat, free space, delete

| Constant | Value |
|---|---:|
| `SD_FT_LIST_MAX_ENTRIES` | 16 |
| `SD_FT_LIST_NAME_MAX` | 64 |
| `SD_FT_MAX_PATH_LEN` | 96 |

Entry attribute flags:

| Flag | Value | Meaning |
|---|---:|---|
| `SD_FT_ATTR_DIR` | `1 << 0` | Entry is a directory |
| `SD_FT_ATTR_NAME_TRUNCATED` | `1 << 1` | Name exceeded 64 characters and was cut |

> **A listing returns at most 16 entries per call.** A session directory with
> more files than that needs several calls. `SD_FT_ATTR_NAME_TRUNCATED` marks
> a name the host cannot use verbatim to open the file — it will not match.

`ShimSdFileTransfer_stagePath` serves both stat and delete, so both share the
96-byte path limit.

## 7. Host retrieval flow

1. Check the firmware version supports the commands.
2. Stop sensing; ensure the device is undocked and not on USB.
3. `SD_FREE_SPACE_COMMAND` for an overall picture.
4. `SD_LIST_DIR_COMMAND` down from `/data`, repeating for directories with more
   than 16 entries.
5. `SD_FILE_STAT_COMMAND` for each file's size.
6. `SD_FILE_READ_COMMAND` in windows, following `nextOffset` from each status
   frame until `SD_FT_XFER_EOF`.
7. Verify each data frame's CRC; on failure re-request that window from the
   status frame's `nextOffset`.
8. Optionally `SD_DELETE_COMMAND`.

> **Mirror the on-card directory structure host-side, and name the backup
> folder by the device's MAC address rather than its Shimmer name.** Names are
> user-editable and not unique; the MAC is neither.

The retrieved bytes are exactly the on-card file — header plus records as
described in [SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md).

## Still unverified / not found in code

- **Payload layouts of the list, stat, free-space and delete responses.** The
  builders `ShimSdFileTransfer_buildListDirRsp`, `...buildStatRsp`,
  `...buildFreeSpaceRsp` and `...buildDeleteRsp` exist and their bounds are
  documented above, but the byte-level response layouts were not enumerated for
  this document.
- **The pending-status queue depth.** `pendingStatus[]` is a fixed array and
  `sdFtQueueStatus` overwrites the **last** slot when full rather than
  dropping the new entry or growing. The array size was not read, so how many
  statuses can be outstanding is unknown.
- **Whether transfer during logging will be permitted.** The idle-only
  restriction is a deliberate current constraint, not an inherent one; a
  concurrent-transfer change would depend on the SD write path tolerating an
  interleaved reader.
- **The CRC used by `sdFtCalcCrc`.** Whether it shares `CRC_INIT = 0xB0CA` with
  the rest of the comms code was not confirmed.
- **`SD_FT_STATUS_BAD_ARGS` (`0xF2`).** Defined, but no path was found that
  returns it — argument faults observed in the read path produce
  `SD_FT_XFER_DENIED` instead.
