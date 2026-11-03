# Shimmer3R audio recording

A Shimmer3R can record from its PDM microphone while it logs to SD (DEV-99).
Configuration byte 233 says whether it does and what it keeps
([SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §8.5).
Shimmer3 has no microphone, and none of this exists there.

| Mode (byte 233 bits 0-1) | Records | Built |
|---|---|---|
| 0 | Nothing | yes |
| 1 | The audio fingerprint only | yes |
| 2 | The fingerprint and the raw 16 kHz audio | yes |
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
| File layouts | `Audio/shimmer_audio_file.{c,h}` |
| The recorder | shimmer3r-firmware `Shimmer_Driver/hal_AudioRecorder.c` |
| The fingerprint: a port of audfprint, with CMSIS-DSP's FFT | shimmer3r-firmware `Middlewares/Third_Party/AudioFingerprint`, `Middlewares/Third_Party/CMSIS-DSP` |
| Capture: ADF1, GPDMA1 channel 11 | shimmer3r-firmware `Shimmer_Driver/hal_Audio.c` |

## 1. When it records

Audio follows SD logging. When a logging session's data file opens, the
recorder is queued. `TASK_AUDIO` then:
1. makes the directories;
2. opens the first audio files: the WAV in modes 2 and 3, the fingerprint file
   in modes 1 and 2;
3. powers the microphone and starts capture.

The first 256 ms of capture are dropped while the microphone starts and the
filters settle. In two bench runs the first audio sample came 0.36 and 0.48 s
after the data file's initial timestamp, which is 0.73 and 0.85 s after its first
sample: the data file's packet ring holds samples taken before the file opened.
Each file's header says exactly when.

Logging stopping, by any route, stops capture at once. Everything captured is
written to the WAV, a fingerprint chunk already analysed is written, and the
files are closed, before a dock can hand the card to a host. The fingerprint
chunk still filling or being analysed at that moment is dropped, so the last 4
to 9 s of a session have no fingerprint. A failure in audio (a file that will
not open, a microphone that will not start, a failed write) stops audio for the
rest of the session, with the reason in the status (§5). It never stops logging.

The audio files split when the data file does, at its hourly split, into the
next files of the same number. Its one-minute sync syncs the audio files too.
Nothing is lost or repeated at a split: the new WAV's first sample is one
sample period after the old file's last. The two files do not change over at the
same instant, though. On the bench the audio changed over about 0.11 s of
capture earlier than the data file, so match samples to records by their
Shimmer clock times, not by which file they are in. The fingerprint's chunks
run on across a split (§3.2), and a chunk whose records are being written
finishes in the old file first.

## 2. Files

```
data/<experimentId>_<configTime>/<shimmerName>-<NNN>/<NNN>
audio/<experimentId>_<configTime>/<shimmerName>-<NNN>/<NNN>.WAV   modes 2 and 3
audio/<experimentId>_<configTime>/<shimmerName>-<NNN>/<NNN>.AFP   modes 1 and 2
```

`audio/X/NNN.WAV` and `audio/X/NNN.AFP` belong with `data/X/NNN`: the same experiment directory,
session directory and file number. This is the companion-file rule the event
channel's `events/` tree uses
([SHIMMER3_EVENT_CHANNEL.md](SHIMMER3_EVENT_CHANNEL.md) §4.1). Each header also
carries the data file's initial timestamp, so a copied or renamed file can
still be paired.

- **What reads it.** Every tool that reads a card looks only under `data/`, so
  audio files are invisible to Consensys's import and to Clear SD. The SD file
  transfer can list, read and delete them
  ([SHIMMER3R_SD_FILE_TRANSFER.md](SHIMMER3R_SD_FILE_TRANSFER.md) §6).
- **Size and card.** Raw audio is 32,000 bytes a second, about 115 MB an hour,
  so a FAT32 file stays well under 4 GB between hourly splits. A 16 GB card holds
  about five days of raw audio and 128 Hz sensor data. The fingerprint is at
  most 4,695 records of 4 bytes per 4 s chunk, under 17 MB an hour, and
  usually far less.

## 3. The files

### 3.1 The WAV

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

### 3.2 The fingerprint file

The landmark hashes of audfprint (Dan Ellis, MIT), at the settings Shimmer's
phone app uses, so the device's hashes can be matched against hashes the app or
a PC makes from the same sound. The audio is analysed in chunks of 5 s, one
every 4 s, each on its own, as the app does. Chunk *k* is the session's audio
samples *k* × 64000 to *k* × 64000 + 79999, counted from the session's first
sample: in mode 2, the same samples as the WAVs, joined in order. The device
stores the hashes, not the spectral peaks they are made from, so a host matches
them as they are.

A 512-byte header, then the chunks, each a 16-byte chunk header followed by its
records. All fields are little-endian. Bytes 4-15 say where the chunks start
and how big their headers and records are, so a reader that only wants the
hashes can skip the rest of the header.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | `SAFP` |
| 4 | 4 | Version of the chunk and record layout (u16), 1; header bytes (u16), 512 |
| 8 | 4 | Sample rate (u16), 16000; frame hop (u16), 256 samples |
| 12 | 4 | Chunk header bytes (u16), 16; record bytes (u16), 4 |
| 16 | 8 | Chunk samples (u32), 80000; chunk hop (u32), 64000 |
| 24 | 20 | Zero |
| 44 | 52 | `SAUD` and the identity fields, laid out as in the WAV's header (§3.1), except: |
| 80 | 8 | **Shimmer clock of the file's first chunk** |
| 96 | 1 | **Closed**: 1 when the file closed normally |
| 97 | 1 | The session's first fault (§5), 0 if none |
| 100 | 4 | Chunks in the file |
| 104 | 8 | **Shimmer clock of the file's last chunk** |
| 112 | 4 | Records in the file |
| 116 | 4 | Escape-hatch writes (u16), write errors (u16), since the session began |
| 120 | 12 | ADF error interrupts, of them saturation, of them overflow (u32 each), since the session began |
| 132 | 4 | Chunks skipped since the session began (§4) |
| 136 | 376 | Zero |
| 512 | — | The chunks |

A chunk header:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | `0x4843`, `CH` on the card: finds the chunks again in a file cut short |
| 2 | 2 | Records that follow |
| 4 | 4 | The chunk's number *k*, counted from the session's first chunk |
| 8 | 8 | Shimmer clock of the chunk's first sample, measured |

A record is one landmark, a u32:

| Bits | Field |
|---:|---|
| 0-8 | *t1*: the frame within the chunk, 0-312, in 256-sample (16 ms) frames |
| 9-14 | *dt*: frames from the first peak to the second |
| 15-20 | (*f2* − *f1*) & 63 |
| 21-27 | *f1* & 127 |
| 28-31 | Zero |

- **The hash** is audfprint's, `f1 << 25 | df << 19 | dt`, from bits 21-27,
  15-20 and 9-14.
- **Its time** is the chunk's Shimmer clock plus *t1* × 256 samples of audio
  clock. Over one chunk the two clocks' drift is under a millisecond.
- **A chunk's clock can be a few ticks late.** It comes from the stamp of the
  128 ms block its first sample is in, and that block's interrupt waits for the
  sensors' (§4): on the bench, at 1024 Hz with every sensor on, a median
  0.7 ticks and at most 4 ticks (0.12 ms) off a line through all the chunks.
  For alignment finer than that, fit a line through the file's chunk clocks.
- **Recomputing the hashes on a PC** from the same samples gives the same
  hashes, bit for bit, only with the firmware's single-precision `logf`
  (newlib's). Another maths library moves the odd borderline peak: on the bench,
  1 chunk in 1,048 differed, by a few hashes.
- **The headers are written as the WAV's are:** when the file opens, again when
  its first chunk is written, and with the closing summary when it closes. A
  file whose closed byte is 0 was cut short: walk its chunk headers, and ignore a
  last chunk with fewer records than its header says.
- **A skipped chunk** leaves a gap in the chunk numbers. The status and the
  header count them.

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
- On the bench, 70 minutes of 1024 Hz logging with every sensor on lost no
  samples, recording raw audio (mode 3) or raw audio and the fingerprint
  (mode 2). With the fingerprint, 11 sample stamps in 4.3 million were more
  than one tick late, the worst by 10 ticks (0.31 ms): the official firmware's
  own level without audio.

**Sharing the main loop:**
- `TASK_AUDIO` is the lowest-priority task. Each pass works in short jobs (a
  2 KB write to the WAV, one block copied into the fingerprint's chunk buffer,
  one unit of analysis, or 2 KB of records) for at most 2 ms, and gives way as
  soon as any other task is pending.
- An SD card can stall a write for a few hundred milliseconds, and the ring
  holds 8 s. If the ring ever passes half full, the main loop writes one block
  of the WAV ahead of the task list. That is the escape hatch, and each use is
  counted.

**The fingerprint:**
- A full 5 s chunk is analysed one `af_step()` unit at a time: an STFT frame, a
  spectrogram row or column, or one frame's peaks or landmarks. There are about
  2,080 units in a chunk, about 0.9 s of CPU at 48 MHz in all, and none takes
  more than a few milliseconds.
- The chunk buffer holds one chunk. While it is analysed, the next chunk's
  audio waits in the ring; its first second is carried over from this one.
- If analysis ever falls so far behind that the ring would overwrite audio a
  chunk still needs, that chunk is skipped and the next starts half a ring
  behind the capture. Skipped chunks are counted, and the first sets fault 5.
- The tables the analysis needs take about 70 ms to build, so the firmware
  builds them at boot, before anything samples.
- The analysis keeps a spectrogram of the chunk in RAM, which could be turned
  back into speech. It is never logged, stored or sent; only the hashes leave.

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
(`0xBD`) and 32 bytes:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Payload version, 1 |
| 1 | 1 | Mode configured (byte 233, corrected) |
| 2 | 1 | State: 0 off, 1 starting, 2 recording, 3 stopped, 4 fault |
| 3 | 1 | Fault: 0 none, 1 open (a directory or file), 2 capture (the mic would not start), 3 write, 4 overrun (silence was written), 5 the fingerprint fell behind (chunks were skipped) |
| 4 | 4 | Seconds recorded this session: of WAV in modes 2 and 3, of audio fingerprinted in mode 1 |
| 8 | 4 | Samples written as silence this session |
| 12 | 2 | Escape-hatch writes |
| 14 | 2 | Write errors |
| 16 | 4 | ADF error interrupts |
| 20 | 4 | Of them, saturation |
| 24 | 4 | Fingerprint chunks written this session |
| 28 | 4 | Fingerprint chunks skipped this session |

Faults 4 and 5 are degradations, not stops: recording carries on, and the fault
bit says the WAV has silence in it or the fingerprint has gaps.

## 6. Power and sleep

**What runs while recording:**
- The microphone, the ADF and its PLL3 kernel clock, and GPDMA.
- An interrupt every 128 ms.
- The card writes: 32 KB a second of raw audio, and a few KB every 4 s of
  fingerprint.
- In modes 1 and 2, the analysis: about 0.9 s of CPU every 4 s at 48 MHz,
  during which the main loop does not idle.

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
- Whether the fingerprint file stores hashes, as now, or spectral peaks (§3.2).
- Gain presets. Byte 233 bits 2-3 are reserved for them.
- An LED cue while raw audio records.
- Whether to mask the ADF's saturation flag. In two bench recordings it fired
  263 and 278 times in the first 11 s and never after, so it looks like a
  start-up transient rather than clipping.

**Hosts:**
- Byte 233 in the host APIs' configuration models.
- The audio status bits.
- An importer that pairs `audio/` with `data/` and places the samples and the
  fingerprint's chunks on the Shimmer clock.
