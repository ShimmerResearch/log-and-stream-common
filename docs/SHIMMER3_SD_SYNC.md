# Shimmer3 / Shimmer3R SD Sync

Multi-device time synchronisation for SD logging. One device acts as the
**centre** and periodically connects to each **node** over Bluetooth to
exchange clock readings; each node records its own offset from the centre so
that separately logged files can be aligned afterwards.

The offset is **recorded, never applied**. No device adjusts its clock. Sync
produces a per-node correction that host software applies when combining files.

> **Verified against** — the revisions these claims were read from. A pinned
> commit is a citation, not a claim of currency.
>
> - **Firmware:** `log-and-stream-common` @ `f3cf73e` —
>   `SDSync/shimmer_sd_sync.{h,c}` in full, plus `log_and_stream_definitions.h`
>   (`MAX_NODES`, `MAX_CHARS`).
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4`;
>   `shimmer3r-firmware` @ `a8f105e5`.

> **How to read this document.** **S3** = Shimmer3 (MSP430); **S3R** =
> Shimmer3R (STM32U5). LogAndStream only. All times are in 32768 Hz ticks
> unless stated.

**Source references:**

| Layer | File |
|---|---|
| Protocol, scheduling, offset selection | `SDSync/shimmer_sd_sync.c` |
| Constants and packet layout | `SDSync/shimmer_sd_sync.h` |
| Node table in configuration | [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §9 |
| `sdlog.cfg` keys | [SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md) §5.3 |
| LED indication | [SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) §3.4 |

---

## 1. Roles and configuration

| Role | Set by | Behaviour |
|---|---|---|
| **Centre** | `masterEnable`, configuration byte 217 bit 1 | Initiates connections to each node in turn |
| **Node** | `masterEnable` clear | Waits to be connected to |

Both require `syncEnable` (byte 217 bit 2). Sync is mutually exclusive with
normal log-and-stream operation: `ShimConfig_checkBtModeFromConfig` sets
`sdSyncEnabled` only when Bluetooth is enabled *and* `syncEnable` is set, and
the radio is then owned by the sync state machine rather than by a host.

> **A device in SD-sync mode is not available for host connections in the usual
> way.** The upper LED reflects this with a different set of patterns
> ([SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) §3.4).

### 1.1 The node table

Up to `MAX_NODES` = **20** nodes, each a 6-byte Bluetooth address plus a name
of up to `MAX_CHARS` = 13 characters.

Addresses live in configuration bytes 256-381
([SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §9),
which has room for 21 entries — the centre address plus 20 nodes.

> **The configuration table holds 21 addresses but `MAX_NODES` is 20.** The
> first entry is the centre; the remaining 20 match the node array. A host that
> writes a 21st node has nowhere for the firmware to put it.

`ShimSdSync_parseSyncNodeNamesFromConfig` and
`ShimSdSync_parseSyncCenterNameFromConfig` populate the runtime arrays from
configuration; `ShimSdSync_parseSyncNodeNameFromCfgFile` and
`...CenterNameFromCfgFile` do the same from `sdlog.cfg`'s repeated `node=`
lines and single `center=` line.

`ShimSdSync_checkSyncCenterName` runs as part of
`ShimConfig_checkAndCorrectConfig`.

## 2. Estimated experiment length drives everything

`ShimSdSync_setSyncEstExpLen(est_exp_len)` takes a value in **minutes** from
configuration bytes 220-221 and derives the whole schedule:

```c
if (est_exp_len < 10) { shortExpFlag = 1; est_exp_len = 0xffff; }
else                  { shortExpFlag = 0; if (est_exp_len > 180) est_exp_len = 180; }
estLen  = est_exp_len * 60;    // seconds
estLen3 = estLen / 3;
```

The source documents the intended values as **5** for a short trial (under an
hour), **45** for medium (under three hours) and **180** for long (over three
hours).

> **Anything below 10 minutes is not "10 minutes" — it becomes 0xFFFF.** The
> short-trial branch replaces the value with 65535, giving
> `estLen = 3,932,100` seconds and `estLen3 = 1,310,700`. That is not a length
> estimate at all; it is a sentinel that makes the sync window effectively
> unbounded, combined with the `shortExpFlag` behaviour in §4.2. A configuration
> of "9 minutes" and one of "1 minute" behave identically.

> **Above 180 minutes the value is clamped**, so a 10-hour trial is scheduled as
> if it were three hours.

Two derived limits are then clamped at start:

```c
rcWindowC    = min(SYNC_WINDOW_C, estLen3 - SYNC_BOOT)      // 800
rcNodeReboot = min(SYNC_NODE_REBOOT, estLen3 / SYNC_WINDOW_N)  // 17
```

## 3. Constants

| Constant | Value | Meaning |
|---|---:|---|
| `SYNC_PERIOD` | 32768 | Sync timer period — one tick per second |
| `SYNC_FACTOR` | 1 | `32768 / SYNC_PERIOD` |
| `SYNC_INT_C` | 54 | Minimum broadcast interval, seconds |
| `SYNC_BOOT` | 3 | Settling period before the centre starts |
| `SYNC_CD` | 2 | Cool-down between connection attempts |
| `SYNC_EXTEND` | 4 | Window extension on activity |
| `SYNC_T_EACHNODE_C` | 12 | Seconds the centre allows per node |
| `SYNC_WINDOW_C` | 800 | Centre sync window |
| `SYNC_WINDOW_N` | 50 | Node sync window |
| `SYNC_NODE_REBOOT` | 17 | Node reboot threshold |
| `SYNC_TRANS_IN_ONE_COMM` | 50 | Clock exchanges per connection |
| `SYNC_NEXT2MATCH` | 2 | |
| `BT_SD_SYNC_CRC_MODE` | `CRC_1BYTE_ENABLED` | Sync packets always carry a 1-byte CRC |

`btIntervalSecs` (configuration byte 219) is clamped to at least `SYNC_INT_C`
when `syncEnable` is set — see
[SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §10.3.

> **The source requires `SYNC_BOOT > SYNC_CD`** (3 > 2), noted in a comment
> beside the definitions. Changing either without preserving that relation
> breaks the scheduler.

## 4. The exchange

### 4.1 Packets

**Centre to node** — 10 bytes plus CRC, built by `ShimSdSync_centerT10`:

| Offset | Constant | Size | Content |
|---:|---|---:|---|
| 0 | `SYNC_PACKET_CMD_IDX` | 1 | `SET_SD_SYNC_COMMAND` |
| 1 | `SYNC_PACKET_FLG_IDX` | 1 | The centre's `sensing` flag |
| 2-9 | `SYNC_PACKET_TIME_IDX` | 8 | Centre's real-world clock, ticks, **LSB order** |
| 10 | — | 1 | CRC |

**Node to centre** — 3 bytes, built by `ShimSdSync_nodeT1`:

| Offset | Content |
|---:|---|
| 0 | `ACK_COMMAND_PROCESSED` |
| 1 | `SD_SYNC_RESPONSE` |
| 2 | `SYNC_PACKET_RESEND` (`0x01`) or `SYNC_FINISHED` (`0xFF`) |

> **The node response carries no CRC**, despite `BT_SD_SYNC_CRC_MODE` being
> enabled. Only the centre-to-node direction is protected. A corrupted node
> response is indistinguishable from a real one, and `0x01` versus `0xFF`
> differ in every bit, so a heavily corrupted response is more likely to be read
> as "resend" than as "finished" — which fails safe.

### 4.2 One connection is 50 exchanges

For each node, the centre opens a Bluetooth connection and then repeats:

1. Centre sends a 10-byte sync packet with its current clock.
2. Node validates the CRC. On failure it replies `SYNC_PACKET_RESEND` without
   recording anything.
3. On success the node computes the offset against the local time it saved when
   the packet arrived, and stores it in `myTimeDiffArr[]` with a sign flag in
   `myTimeDiffFlagArr[]`.
4. If fewer than `SYNC_TRANS_IN_ONE_COMM` = 50 samples have been collected, the
   node replies `SYNC_PACKET_RESEND` and the centre sends another.
5. On the 50th, the node runs the outlier rejection of §5, replies
   `SYNC_FINISHED`, and the centre moves on.

The sign flag exists because the offset is stored as an unsigned 64-bit
magnitude:

```c
if (myLocalTimeLong > myCenterTimeLong) { flag = 0; diff = local - centre; }
else                                    { flag = 1; diff = centre - local; }
```

**Flag 0 means the node is ahead of the centre; flag 1 means behind.**

> **The centre's `sensing` flag in byte 1 is what makes single-touch start
> work.** A node with `singleTouchStart` set, not currently sensing, that
> receives a packet whose flag byte is set, calls
> `ShimTask_setStartLoggingIfNotAlready`. So starting the centre starts every
> node it reaches. This is inside `#if IS_SUPPORTED_SINGLE_TOUCH`.

### 4.3 The first result is discarded

`firstOutlier` is initialised at sync start:

| Role | Initial value |
|---|---|
| Centre | `nodeSuccFull` — every node's bit set |
| Node | 1 |

A node only publishes its offset when `firstOutlier` is zero:

```c
if (!firstOutlier) { rcFirstOffsetRxed = 1; rcFindSmallest(); ... }
```

and the centre clears a node's bit on that node's first success rather than
counting it:

```c
if (firstOutlier & nodeShift(syncNodeCnt)) firstOutlier &= ~nodeShift(...);
else                                       nodeSucc     |= nodeShift(...);
```

> **The first complete sync with each node is deliberately thrown away.** It is
> treated as a warm-up whose timing is unrepresentative — radio connection
> setup dominates. A trial that only ever achieves one sync round therefore has
> **no** recorded offset at all, and the node's blue LED stays in the
> "waiting for first offset" pattern for the whole trial.

## 5. Outlier rejection and offset selection

`ShimSdSync_rcFindSmallest` reduces the 50 samples to one. Two passes.

**Pass 1 — blacklist.** For each sample `i`, compare it against the next five
samples cyclically. Count how many differ by more than **3277 ticks** (0.1 s).
If four or more of the five are that far away, blacklist `i`. Up to 20 samples
can be blacklisted.

**Pass 2 — pick the extreme.** Walk the non-blacklisted samples and keep the
one that is "smallest" in a sign-aware sense:

| Stored sign | New sample sign | Rule |
|---|---|---|
| positive | positive | Keep the smaller magnitude |
| positive | negative | **Replace** — a negative always beats a positive |
| negative | positive | Ignore |
| negative | negative | Keep the **larger** magnitude |

Read as a signed number, all four rules reduce to "keep the algebraically
smallest offset" — the sample where the node's clock read least ahead of the
centre's.

> **Taking the minimum rather than the mean is deliberate.** Each measurement is
> the true offset plus a non-negative transport delay, so the minimum is the
> sample with the least delay and therefore the best estimate. Averaging would
> bias the result by the mean latency.

> **The blacklist threshold is 0.1 s, which is enormous** relative to the
> sub-millisecond synchronisation being attempted. It rejects gross outliers —
> a retransmitted packet, a radio stall — not measurement noise. The minimum
> selection in pass 2 is what actually finds the good sample.

> **`black` is not reset when the inner loop breaks on a match.** It is set to 1
> inside the loop and cleared at the top of the `if (black)` branch on the next
> iteration, which happens to work, but a reader tracing the flag will find its
> lifetime confusing.

The winner is published as 9 bytes in `myTimeDiff[]`:

| Offset | Content |
|---:|---|
| 0 | Sign flag — 0 = node ahead, 1 = node behind |
| 1-8 | Magnitude, 64-bit, LSB order |

`ShimSdSync_myTimeDiffPtrGet()` exposes it. This is what a host must apply to
align that node's file against the centre's.

## 6. Scheduling

`ShimSdSync_handleSyncTimerTrigger` runs once per second and dispatches to the
centre or node handler.

Every `estLen3` seconds the counter resets and a new sync round begins, with
`syncThis` counting rounds. The comment "*can stop syncing after certain #*"
guards `syncThis > 3` — after four rounds the behaviour changes.

### 6.1 Centre

Active only while `sensing` and with at least one node configured.

| `syncCnt` | Action |
|---|---|
| 1 | Start |
| `> SYNC_BOOT` and `< SYNC_WINDOW_C` | Attempt each node in turn |

Within the window, while `nodeSucc != nodeSuccFull` and there are nodes left,
the centre:

1. Skips nodes whose success bit is already set.
2. Starts Bluetooth and attempts a connection, allowing
   `SYNC_T_EACHNODE_C` = 12 seconds.
3. On success or timeout, **stops Bluetooth** (`btStopCb(0)`) before moving to
   the next node.
4. Waits `SYNC_CD` = 2 seconds before the next attempt.

> **Bluetooth is stopped and restarted for every node.** That is what makes a
> full round take so long — 12 seconds of allowance plus a radio restart per
> node, so twenty nodes is on the order of five minutes even when every one
> succeeds first time.

> **`shortExpFlag` makes the centre declare success without one.** In the short
> branch, `nodeSucc |= 1 << syncNodeCnt` is set on *timeout* as well as on
> success. A short trial therefore reports every node as synced whether or not
> any packet was exchanged. Combined with §2's 0xFFFF substitution, "short
> trial" means "do not block on sync", not "sync quickly".

### 6.2 Node

The node is passive: it waits for a connection, responds, and maintains its own
window. `syncNodeWinExpire` extends by `SYNC_EXTEND` on each response while
below the limit, so a node in active conversation keeps its window open.

`rcNodeReboot` bounds how long a node waits before restarting its radio.

## 7. What a host must do

1. Read each node's 9-byte offset (§5) — over Bluetooth, or from the
   `SDH_RTC_DIFF` field in the file header on Shimmer3.
2. Convert: `offsetTicks = magnitude`, negated when the sign flag is 0
   (node ahead).
3. Add the offset to that node's timestamps to place them on the centre's
   timeline.
4. **Check `rcFirstOffsetRxed`.** A node that never got past its first sync has
   no valid offset, and its data cannot be aligned.

Absolute time still comes from the real-world clock; sync only relates the
devices to each other. See
[SHIMMER3_TIMEKEEPING.md](SHIMMER3_TIMEKEEPING.md).

## 8. Diagnostics

| Accessor | Reports |
|---|---|
| `ShimSdSync_isBtSdSyncRunning()` | Sync state machine active |
| `ShimSdSync_syncSuccCenterGet()` | `syncSuccC` |
| `ShimSdSync_syncSuccNodeGet()` | `syncSuccN` |
| `ShimSdSync_syncCntGet()` | Rounds completed |
| `ShimSdSync_rcFirstOffsetRxedGet()` | Whether this node has a usable offset |
| `ShimSdSync_syncNodeNumGet()` | Configured node count |

The blue upper LED reflects `rcFirstOffsetRxed` while sensing — see
[SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) §3.4.

## Still unverified / not found in code

- **The achieved synchronisation accuracy.** Nothing in the firmware states a
  target or a measured figure. The 0.1 s blacklist threshold and the
  minimum-selection strategy bound the *algorithm*, not the result.
- **What happens after `syncThis > 3`.** The branch is empty apart from the
  comment "*can stop syncing after certain #*"; the `else` increments
  `syncThis`. The intended behaviour is not implemented or not obvious.
- **`SYNC_NEXT2MATCH` (2).** Defined, never referenced.
- **The commented-out original parameters** at the top of the header
  (`RC_AHD`, `RC_WINDOW_N`, `RC_WINDOW_C`, `RC_INT_N`, `RC_CLK_N`, `RC_CLK_C`,
  `RC_FACTOR_N`, `RC_FACTOR_C`) and the note "*all node time must \*2 in use,
  all center time must \*4 in use*". Whether those multipliers still apply to
  the current constants was not established, and no code applies them.
- **How `syncSuccC` and `syncSuccN` are consumed.** Both are maintained and
  exposed, but the LED code uses `rcFirstOffsetRxed` instead, and the original
  blink logic that used them is commented out in `shimmer_leds.c`.
- **Whether a node ever reboots its radio in practice.** `rcNodeReboot` and
  `nReboot` / `cReboot` are maintained, but the reboot path was not traced end
  to end.
- **Recovery when the centre itself has no valid real-world clock.** The centre
  transmits whatever `RTC_get64()` returns; nothing checks that the clock was
  ever set.
