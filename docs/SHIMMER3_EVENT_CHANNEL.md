# Shimmer3 / Shimmer3R event channel

The event channel records input edges with their own timestamps, independent of
the sampling rate, in an event file beside each data file. The input today is
the user button: with event markers on, every press and release is recorded,
and a double press starts and stops logging. A timer-captured TTL input and
streaming the events over Bluetooth are planned; the file format already has
room for both (§3.2).

Code: `Events/` (`shimmer_events.c`, `shimmer_event_record.c`) and `Button/`
(`shimmer_button.c`, `shimmer_button_gesture.c`).

## 1. Turning it on

| Where | Name |
|---|---|
| Configuration byte 218, bit 1 | `buttonEventMarkerEnable` ([CONFIGURATION_INFOMEM](SHIMMER3_CONFIGURATION_INFOMEM.md) §8.2) |
| `sdlog.cfg` | `button_event_marker_enable` |
| SD header byte 17 | `SDH_BUTTON_EVENT_MARKER` (0x02), a copy of byte 218 |

It is off by default. With it off, nothing is written and no `events/`
directory is created. The button still starts and stops logging with a single
press, as it always has (§2.3).

`userButtonEnable` (byte 217 bit 5) still decides whether the button may start
and stop logging at all. With markers on and `userButtonEnable` off, presses are
recorded and the button leaves logging alone.

> **`userButtonEnable` off also starts logging on undock.** With it off (and
> `singleTouchStart` off), the unit starts logging whenever it leaves the dock
> with a card in the slot (`ShimSens_startLoggingIfUndockStartEnabled()`). On a
> Shimmer3R, unplugging USB-C counts as leaving the dock. So turning the button
> off does not stop a unit logging, and nothing in the event channel changes
> that.

On a Shimmer3R SR48-6-0 board the bit is cleared at validation: that board's
button is wired to BOOT0, whose interrupt is used for dock detection, so there
are no button edges to record.

> **A host that rewrites byte 218 must keep bit 1.** Read the byte, change the
> bits you mean to, and write it back.

## 2. The user button

### 2.1 From interrupt to main loop

The button interrupt only takes the edge's direction and the time
(`RTC_get64()`), and queues them in a 16-entry ring for `TASK_EVENT_CHANNEL`,
the lowest-priority task. Everything else runs in the main loop.

| | Edge direction from | Press is |
|---|---|---|
| Shimmer3 | `P1IES`: the edge the interrupt was armed for. The opposite edge is armed before the edge is queued | falling (active low) |
| Shimmer3R | which EXTI callback ran, rising or falling | rising (active high) |

Neither reads the pin to decide which edge it was: by the time the interrupt
runs the pin may already have moved on, and reading it used to turn a quick tap
into a phantom release.

`shimmerStatus.buttonPressed`, which the lower green LED follows, is now the
debounced level, written only from the main loop.

### 2.2 Debounce

An edge less than 25 ms after the last accepted one is ignored as bounce. The
last ignored edge may still have been real - a tap shorter than 25 ms, or a
bounce that ends at the other level - so once the line has been quiet for 25 ms
the pin is checked. The 100 ms LED blink tick queues the check
(`ShimEvt_tick()` from `LogAndStream_blinkTimerCommon()`). If the pin disagrees,
the ignored edge becomes an accepted one at its own time, marked
`EVT_FLAG_SYNTHESIZED`.

An edge in the same direction as the last accepted one means the edge between
them was missed. It is put back, marked synthesized, at the time of the last
ignored edge if that went the missing way, and otherwise at the new edge's time.

### 2.3 Starting and stopping logging

| Markers | Gesture | Rule |
|---|---|---|
| off | single press | A release more than 0.5 s after the previous release. A press of 5 s or more does nothing |
| on | double press | Two presses of at most 300 ms each, released at most 400 ms apart. A third quick press starts a new pair. Single presses are markers only, so a marker cannot stop a recording |

Either way, nothing happens while the device is being configured, or while SD
sync is in the middle of an exchange over Bluetooth.

> **Single press after the debounce.** A bounce on a press used to be able to
> start and stop logging in one press; it no longer can. A tap shorter than
> 25 ms still counts, but its action waits for the recheck, up to about
> 0.1 s.

## 3. The event file

### 3.1 Where

```
/
├── data/<experimentId>_<configTime>/<shimmerName>-<NNN>/<file>
└── events/<experimentId>_<configTime>/<shimmerName>-<NNN>/<file>
```

`events/X` belongs with `data/X`: the same experiment directory, session
directory and file number, with no extension. The event file splits when the
data file does. Its header also carries the data file's initial timestamp
(§3.2), so a copied or renamed file can still be paired.

Every tool that reads a card today looks only under `data/`, so the event files
are invisible to them. Consensys imports a card unchanged. On Shimmer3R the SD
file transfer can list, read and delete them
([SD_FILE_TRANSFER](SHIMMER3R_SD_FILE_TRANSFER.md) §6).

The longest event file path is 53 characters, within Shimmer3's 64-character
long-name limit ([SD_CARD_FORMAT](SHIMMER3_SD_CARD_FORMAT.md) §1.4).

### 3.2 Layout

A 64-byte header, then 16-byte records, all little-endian. Both are multiples of
16, so no record straddles a 512-byte sector.

Header:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Magic, `SEVT` |
| 4 | 1 | Format version, 1 |
| 5 | 1 | Header length, 64 |
| 6 | 1 | Record length, 16 |
| 7 | 1 | Hardware ID: 3 Shimmer3, 10 Shimmer3R |
| 8 | 8 | File-open time: the data file's initial timestamp, all 64 bits |
| 16 | 8 | Shimmer3: the offset from boot time to real-world time, as SD header bytes 44-51. Shimmer3R: 0 |
| 24 | 4 | Config time |
| 28 | 2 | Firmware ID |
| 30 | 2 | Firmware major version |
| 32 | 1 | Firmware minor version |
| 33 | 1 | Firmware internal version |
| 34 | 2 | Sequence number of the first record |
| 36 | 1 | Why the file was opened: 1 session start, 2 split |
| 37 | 1 | Inputs recorded: bit 0 button, bit 1 TTL |
| 38 | 26 | Reserved, zero |

Readers should take the record length from byte 6 and skip any header bytes
beyond those they know, using byte 5.

Record:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Type: 0x01 edge, 0x02 initial state, 0x03 heartbeat, 0x04 overflow (reserved). 0x10 and up are reserved |
| 1 | 1 | Flags: bit 0 time latched by a timer, bit 1 synthesized, bit 2 late, bit 3 state stale, bit 7 final |
| 2 | 2 | Sequence number. Continues across a split |
| 4 | 8 | Time, in RTC ticks at 32768 Hz |
| 12 | 1 | Channel: 0 button, 1 TTL, 0xFF none |
| 13 | 1 | State of every input after this record: bit 0 button, bit 1 TTL |
| 14 | 2 | Edges lost so far this session, stopping at 0xFFFF |

### 3.3 What is written when

The event file follows the data file but never delays it. Its open, split and
one-minute sync wait for `TASK_EVENT_CHANNEL`, the lowest-priority task; only
the close at the end of a session is done at once.

- **Session start.** When the data file opens (`ShimSdDataFile_fileInit()`),
  the event file is only queued. The task then writes:
  - the header;
  - kept records from at most 2 s before the data file opened - the gesture
    that started logging;
  - an initial-state record at the data file's open time;
  - anything recorded between the data file opening and now.
- **During the session.** Each debounced edge is written as it is processed.
- **One-minute sync.** At the data file's one-minute sync, a heartbeat is
  written if nothing else has been for 60 s, then the file is synced. A file
  that stops growing has stopped recording.
- **Split.** The old file ends with a final record, a heartbeat with the final
  flag, at the split time. The new file opens with reason 2, carries on the
  sequence numbers, and starts with an initial-state record. An edge from before
  the split that is still queued when the split happens goes in the old file.
- **Stop.** A final record, then the file is closed. A file with no final
  record was cut short, for example by a flat battery.
- **Errors.** A failed open or write stops the event file for the rest of the
  session. The data file is never affected.

Edges are lost only when the 16-entry ring is full when an interrupt comes, or
when more than 8 records arrive before the event file opens. Both are counted in
every later record's dropped field.

### 3.4 Timing

Times are RTC ticks, 1/32768 s (about 30.5 µs), the same timeline as the data
file's samples: pair a record with the data file by subtracting the header's
file-open time, or compare it directly with an unwrapped sample time
([STREAMING_DATA_FORMAT](SHIMMER3_STREAMING_DATA_FORMAT.md) §2.1).

A button edge's time is the RTC reading in its interrupt, so it includes that
interrupt's latency: microseconds normally, longer if another interrupt or a
masked section holds it off. That is far below anything a press can resolve.
Button records leave the timer-latched flag clear; it is for the TTL input.

### 3.5 Memory

The event file uses `sdlog.cfg`'s file object (`ShimSdCfgFile_lendFil()`) for
the length of a session: Shimmer3 has no RAM for another, and `sdlog.cfg` is
never read or written while logging. While it is lent, `ShimSdCfgFile_parse()`
and `ShimSdCfgFile_generate()` leave the card alone. The rest costs 449 bytes
on Shimmer3, from the link map: the ring 160, the 8 kept records 128, the file
state and path 112, the button's gesture state 44, and the indexes 5.

## 4. Worked example

A Shimmer3R on v1.01.025 logging for 2.5 minutes, started and stopped by a
double press, with one marker press. The two 10 ms TTL pulses show how the TTL
input's records will look; the firmware records only the button so far. 336
bytes: the header and 17 records.

```
 # | type  | flags | ch  | state | t from open (s) |
 0 | edge  | 0x00  | btn | b01   |       -0.619995 | press  - the double press
 1 | edge  | 0x00  | btn | b00   |       -0.519989 | release  that started
 2 | edge  | 0x00  | btn | b01   |       -0.359985 | press    logging
 3 | edge  | 0x00  | btn | b00   |       -0.239990 | release
 4 | init  | 0x00  | -   | b00   |       +0.000000 | state at the data file's open
 5 | edge  | 0x00  | btn | b01   |      +12.399994 | marker press
 6 | edge  | 0x00  | btn | b00   |      +12.579987 | marker release
 7 | edge  | 0x01  | ttl | b10   |      +30.000122 | TTL rising (timer-latched)
 8 | edge  | 0x01  | ttl | b00   |      +30.010132 | TTL falling
 9 | edge  | 0x01  | ttl | b10   |      +45.000488 | TTL rising
10 | edge  | 0x01  | ttl | b00   |      +45.010498 | TTL falling
11 | hbeat | 0x00  | -   | b00   |     +120.000000 | nothing for 60 s
12 | edge  | 0x00  | btn | b01   |     +150.000000 | press  - the double press
13 | edge  | 0x00  | btn | b00   |     +150.109985 | release  that stopped
14 | edge  | 0x00  | btn | b01   |     +150.299988 | press    logging
15 | edge  | 0x00  | btn | b00   |     +150.410004 | release
16 | hbeat | 0x80  | -   | b00   |     +150.809998 | final, just before the close
```

```
header
  0000  53 45 56 54 01 40 10 0A 00 20 12 37 72 35 00 00
  0010  00 00 00 00 00 00 00 00 90 5C E4 6A 03 00 01 00
  0020  01 19 00 00 01 03 00 00 00 00 00 00 00 00 00 00
  0030  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
records:  type flags seq    time (u64)                ch st dropped
  0040  01 00   00 00  A4 D0 11 37 72 35 00 00   00 01 00 00
  0050  01 00   01 00  71 DD 11 37 72 35 00 00   00 00 00 00
  0060  01 00   02 00  EC F1 11 37 72 35 00 00   00 01 00 00
  0070  01 00   03 00  48 01 12 37 72 35 00 00   00 00 00 00
  0080  02 00   04 00  00 20 12 37 72 35 00 00   FF 00 00 00
  0090  01 00   05 00  33 53 18 37 72 35 00 00   00 01 00 00
  00A0  01 00   06 00  3D 6A 18 37 72 35 00 00   00 00 00 00
  00B0  01 01   07 00  04 20 21 37 72 35 00 00   01 02 00 00
  00C0  01 01   08 00  4C 21 21 37 72 35 00 00   01 00 00 00
  00D0  01 01   09 00  10 A0 28 37 72 35 00 00   01 02 00 00
  00E0  01 01   0A 00  58 A1 28 37 72 35 00 00   01 00 00 00
  00F0  03 00   0B 00  00 20 4E 37 72 35 00 00   FF 00 00 00
  0100  01 00   0C 00  00 20 5D 37 72 35 00 00   00 01 00 00
  0110  01 00   0D 00  14 2E 5D 37 72 35 00 00   00 00 00 00
  0120  01 00   0E 00  66 46 5D 37 72 35 00 00   00 01 00 00
  0130  01 00   0F 00  7B 54 5D 37 72 35 00 00   00 00 00 00
  0140  03 80   10 00  AE 87 5D 37 72 35 00 00   FF 00 00 00
```

- **Header.** Line 0000: `SEVT`, version 1, header length 64, record length 16,
  hardware ID 10, then the file-open time 0x357237122000 ticks, which is
  10:15:00.250 UTC on 30 Oct 2026. Line 0010: a zero RWC offset, config time
  `6AE45C90` (09:00:00 UTC), firmware ID 3, major version 1. Line 0020: minor 1,
  internal 0x19 (25), first sequence number 0, opened at session start, button
  and TTL recorded.
- **Records 0-3** come before the file-open time: the double press that started
  logging, kept and written first.

`Test/host/test_event_record.c` pins the encoder to these bytes.
`Extras/python_scripts/Shimmer_common/shimmer_event_file.py` prints a file like
this and checks it: whole records, sequence numbers without gaps, times that
never go backwards, a final record, and with `--data` that it pairs with the
data file it was recorded beside.

## 5. Not yet

- **TTL input.** Timer input capture: P1.5 on TA0.CCI4A on Shimmer3, PE9 on
  TIM1_CH1 on Shimmer3R. Its records will set the timer-latched flag.
- **Streaming the events over Bluetooth.** An opt-in in-stream frame; hosts do
  not handle an unknown in-stream response today, so it cannot be sent unasked.
- **Host parsers.** Consensys, the APIs and the web SDK do not read event files
  yet.
