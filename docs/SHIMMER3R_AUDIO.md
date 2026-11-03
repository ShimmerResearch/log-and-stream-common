# Shimmer3R audio recording

A Shimmer3R can record from its PDM microphone while it logs to SD (DEV-99).
Configuration byte 233 says whether it does and what it keeps
([SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §8.5).
Shimmer3 has no microphone, and none of this exists there.

| Mode (byte 233 bits 0-1) | Records | Built |
|---|---|---|
| 0 | Nothing | yes |
| 1 | The audio fingerprint only | not yet (DEV-99 step 5) |
| 2 | The fingerprint and the raw 16 kHz audio | not yet (step 5) |
| 3 | The raw 16 kHz audio only | yes |

A mode that is not built reads back as 0, so a host learns what a unit records
by writing the byte and reading it back.

> **Raw audio records speech.** A host should ask for confirmation before
> setting mode 2 or 3, and everyone near the device needs to know it is
> recording. The fingerprint (mode 1) does not store audio.

| Code | Where |
|---|---|
| Configuration byte | `Configuration/shimmer_audio_mode.{c,h}` |
| Lifecycle, status | `Audio/shimmer_audio.{c,h}`, `SDCard/shimmer_sd_companion.h` |
| File header | `Audio/shimmer_audio_file.{c,h}` |
| The recorder | shimmer3r-firmware `Shimmer_Driver/hal_AudioRecorder.c` |
| Capture: ADF1, GPDMA1 channel 11 | shimmer3r-firmware `Shimmer_Driver/hal_Audio.c` |

## 1. When it records

Audio follows SD logging. When a logging session's data file opens, the
recorder is queued. `TASK_AUDIO` then:
1. makes the directories;
2. opens the first audio file;
3. powers the microphone and starts capture.

The first 256 ms of capture are dropped while the microphone starts and the
filters settle. In two bench runs the first audio sample came 0.36 and 0.48 s
after the data file's initial timestamp, which is 0.73 and 0.85 s after its first
sample: the data file's packet ring holds samples taken before the file opened.
Each file's header says exactly when.

Logging stopping, by any route, stops capture at once. Everything captured is
written and the file closed, before a dock can hand the card to a host. A
failure in audio (a file that will not open, a microphone that will not start,
a failed write) stops audio for the rest of the session, with the reason in the
status (§5). It never stops logging.

The audio file splits when the data file does, at its hourly split, into the
next file of the same number. Its one-minute sync syncs the audio file too.
Nothing is lost or repeated at a split: the new file's first sample is one
sample period after the old file's last. The two files do not change over at the
same instant, though. On the bench the audio changed over about 0.11 s of
capture earlier than the data file, so match samples to records by their
Shimmer clock times, not by which file they are in.

## 2. Files

```
data/<experimentId>_<configTime>/<shimmerName>-<NNN>/<NNN>
audio/<experimentId>_<configTime>/<shimmerName>-<NNN>/<NNN>.WAV
```

`audio/X/NNN.WAV` belongs with `data/X/NNN`: the same experiment directory,
session directory and file number. This is the companion-file rule the event
channel's `events/` tree uses
([SHIMMER3_EVENT_CHANNEL.md](SHIMMER3_EVENT_CHANNEL.md) §4.1). The header also
carries the data file's initial timestamp, so a copied or renamed file can
still be paired.

- **What reads it.** Every tool that reads a card looks only under `data/`, so
  audio files are invisible to Consensys's import and to Clear SD. The SD file
  transfer can list, read and delete them
  ([SHIMMER3R_SD_FILE_TRANSFER.md](SHIMMER3R_SD_FILE_TRANSFER.md) §6).
- **Size and card.** Raw audio is 32,000 bytes a second, about 115 MB an hour,
  so a FAT32 file stays well under 4 GB between hourly splits. A 16 GB card holds
  about five days of raw audio and 128 Hz sensor data.

## 3. The file

A standard 16 kHz, mono, 16-bit PCM WAV, which any player opens. Its first 512
bytes are the header, so the samples start on a card sector. All fields are
little-endian.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 12 | `RIFF`, u32 file size − 8, `WAVE` |
| 12 | 24 | `fmt ` chunk: PCM, 1 channel, 16000 Hz, 32000 B/s, align 2, 16 bits |
| 36 | 8 | `shim` chunk id, length 460 |
| 44 | 4 | `SAUD` |
| 48 | 1 | Format version, 1 |
| 49 | 1 | Hardware ID, 10 |
| 50 | 1 | Mode, as recorded |
| 51 | 1 | Why it opened: 0 the session's first file, 1 after a split |
| 52 | 2 | File number `NNN` |
| 56 | 8 | The data file's initial timestamp |
| 64 | 4 | Config time |
| 68 | 6 | Firmware ID, major, minor, internal (u16, u16, u8, u8) |
| 74 | 6 | Bluetooth MAC |
| 80 | 8 | **Shimmer clock of the first sample** |
| 88 | 8 | ADF decimation (u32), gain (i32) |
| 96 | 1 | **Closed**: 1 when the file closed normally |
| 97 | 1 | The session's first fault (§5), 0 if none |
| 100 | 4 | Samples in the file |
| 104 | 8 | **Shimmer clock of the last sample** |
| 112 | 4 | Samples written as silence after an overrun |
| 116 | 4 | Escape-hatch writes (u16), write errors (u16), since the session began |
| 120 | 12 | ADF error interrupts, of them saturation, of them overflow (u32 each), since the session began |
| 132 | 372 | Zero |
| 504 | 8 | `data` chunk id, u32 sample bytes |
| 512 | — | The samples |

- **The header is written twice.** It is written when the file opens, then
  rewritten with the first sample's clock as soon as that sample is on the
  card. It is written a final time, with the RIFF sizes and the closing summary
  from offset 96, when the file closes.
- **A file whose closed byte is 0 was cut short,** by a flat battery or a
  removed card. Its samples are good up to the file's length, but its RIFF sizes
  and summary are not. Take the sample count from the file size: (size − 512) / 2.
- **Timing.** The Shimmer clock is the same 32768 Hz count the data file's
  timestamps use. The audio clock runs from the 16 MHz crystal, so the two drift
  apart by however far that unit's 32 kHz crystal is off: −23 ppm on one bench
  unit, +56 ppm on another whose factory test puts its 32 kHz crystal 55.5 ppm
  slow. That is up to about 200 ms an hour. So place sample *n* of an *N*-sample
  file at
  `first + n × (last − first) / (N − 1)`. Do not use n / 16000 s from `first`.
- **Gaps.** Sample *n* is always 1/16000 s of capture after sample *n − 1*. If
  the card ever fell further behind than the 8 s capture ring, the samples it
  missed are written as silence and counted at offset 112, so the count never
  slips.

## 4. Capture and timing

**Capture settings:**
- The ADF filters the 1.536 MHz PDM stream with a sinc5 at decimation 24 and a
  reshape filter at 4, giving exactly 16 kHz, then a high-pass filter.
- Gain is SCALE +6 for now. The calibrated default is still to be measured
  against a sound level.

**How samples reach the card:**
- GPDMA1 channel 11 writes the samples straight into an 8 s ring, through a
  circular linked list of 64 nodes, each a 128 ms block.
- Each block's interrupt only stamps it with the Shimmer clock and queues
  `TASK_AUDIO`. Nothing is copied at interrupt level.
- The interrupt is at priority 2 and the ADF's at 7, both below the RTC
  sampling interrupt at 1, so recording never delays a sensor sample.
- On the bench, 70 minutes of 1024 Hz logging with raw audio recording lost no
  samples. One sample stamp in 4.3 million was more than one tick late, by 4
  ticks (122 µs).

**Sharing the main loop:**
- `TASK_AUDIO` is the lowest-priority task. Each pass writes 2 KB at a time,
  for at most 2 ms, and gives way as soon as any other task is pending.
- An SD card can stall a write for a few hundred milliseconds, and the ring
  holds 8 s. If the ring ever passes half full, the main loop writes one block
  ahead of the task list. That is the escape hatch, and each use is counted.

## 5. Status

**Status byte 1** is Shimmer3R only, and comes in `GET_STATUS_COMMAND`'s reply
and every unsolicited status push
([SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md) §5.3):

| Bit | Meaning |
|---:|---|
| 1 | Recording: audio is being captured for the current log |
| 2 | Fault: this session's audio failed or was degraded; cleared when the next session starts |

The firmware pushes a status frame whenever either bit changes. A connected host
sees recording start within a second of logging starting, and sees it stop and
any fault, without polling. Firmware before DEV-99 sends zero in these bits, so gate
them on the firmware version.

**`GET_AUDIO_STATUS_COMMAND`** (`0xBC`) is answered with `AUDIO_STATUS_RESPONSE`
(`0xBD`) and 24 bytes:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Payload version, 1 |
| 1 | 1 | Mode configured (byte 233, corrected) |
| 2 | 1 | State: 0 off, 1 starting, 2 recording, 3 stopped, 4 fault |
| 3 | 1 | Fault: 0 none, 1 open (a directory or file), 2 capture (the mic would not start), 3 write, 4 overrun (silence was written) |
| 4 | 4 | Seconds recorded this session |
| 8 | 4 | Samples written as silence this session |
| 12 | 2 | Escape-hatch writes |
| 14 | 2 | Write errors |
| 16 | 4 | ADF error interrupts |
| 20 | 4 | Of them, saturation |

Overrun is a degradation, not a stop: recording carries on, and the fault bit
says the file has silence in it.

## 6. Power and sleep

**What runs while recording:**
- The microphone, the ADF and its PLL3 kernel clock, and GPDMA.
- An interrupt every 128 ms.
- The card writes, 32 KB a second.

**Sleep:** all of this runs through the Sleep mode the firmware idles in
([SHIMMER3_POWER_MANAGEMENT.md](SHIMMER3_POWER_MANAGEMENT.md) §1). Stop modes
would halt capture, and the firmware does not use them.

**Not measured yet:** the current cost.

## 7. The factory microphone test

The factory test S3R_TEST_0026 captures through the same driver, at its own
settings (decimation 64, gain −8). It runs one capture at a time. A factory test
while a recording owns the microphone reports `FAIL: Microphone busy recording`
rather than restarting the ADF under the recording
([SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md) §5.4).

## Not yet

**Firmware:**
- The fingerprint modes 1 and 2, and their `.AFP` file (DEV-99 step 5).
- Gain presets. Byte 233 bits 2-3 are reserved for them.
- An LED cue while raw audio records.
- Whether to mask the ADF's saturation flag. In two bench recordings it fired
  263 and 278 times in the first 11 s and never after, so it looks like a
  start-up transient rather than clipping.

**Hosts:**
- Byte 233 in the host APIs' configuration models.
- The audio status bits.
- An importer that pairs `audio/` with `data/` and places the samples on the
  Shimmer clock.
