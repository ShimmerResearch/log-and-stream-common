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

Bit 4 of the same byte turns on the **sound level** (DEV-1413): every data
packet carries the A-weighted level of the microphone's latest 128 ms, while
the unit senses, whether it streams, logs or both (§8). It stores no audio.

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
| The sound level: the arithmetic | `Audio/shimmer_audio_level.{c,h}` |
| The sound level: its capture and task | shimmer3r-firmware `Shimmer_Driver/hal_AudioLevel.c` |

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
files are closed, before a dock can hand the card to a host. The WAV gets at
most 1 s for this, about twice what a full ring takes on a healthy card. A card
that has slowed down can owe far more, and the rest is then dropped: the file
ends at its last sample written, and fault 4 is set. Its header gives that
sample's clock only if the file ended within about 8 s of the capture, which is
as far back as the capture keeps block times; a file that ended further behind
has 0 there (§3.1, byte 104).
The fingerprint chunk still filling or being analysed at that moment is
dropped, so the last 4 to 9 s of a session have no fingerprint. A failure in
audio (a file that will
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

**The capture is shared with the sound level** (§8), which holds it for as
long as the unit senses with the level on:
- **A log that starts meanwhile joins it.** Its first audio sample is the start
  of the 128 ms block filling at that moment. There is no start-up drop, since
  the microphone has already settled, and the session's ADF counts start there.
- **A log that stops meanwhile leaves the capture running.** It writes out the
  WAV only as far as the capture had got at the stop. The capture stops when
  sensing does.

## 2. Files

```
data/<experimentId>_<configTime>/<shimmerName>-<NNN>/<NNN>
audio/<experimentId>_<configTime>/<shimmerName>-<NNN>/<NNN>.WAV   modes 2 and 3
audio/<experimentId>_<configTime>/<shimmerName>-<NNN>/<NNN>.AFP   modes 1 and 2
```

`audio/X/NNN.WAV` and `audio/X/NNN.AFP` belong with `data/X/NNN`: the same experiment directory,
session directory and file number. Like the data tree, each audio file and each
directory the recorder makes is dated with the unit's RTC: a file when it closes,
a directory when it is made. This is the companion-file rule the event
channel's `events/` tree uses
([SHIMMER3_EVENT_CHANNEL.md](SHIMMER3_EVENT_CHANNEL.md) §4.1). Each header also
carries the data file's initial timestamp, so a copied or renamed file can
still be paired.

- **What reads it.** Every tool that reads a card looks only under `data/`, so
  audio files are invisible to Consensys's import and to Clear SD. The SD file
  transfer can list, read and delete them
  ([SHIMMER3R_SD_FILE_TRANSFER.md](SHIMMER3R_SD_FILE_TRANSFER.md) §6).
- **Session numbers.** Clear SD empties `data/` and leaves `audio/`, so the
  session number is counted under `audio/` as well as `data/` and `events/`
  (`sdCompanionRoots`), and a number still in use there is not given to a new
  session.
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
| 56 | 8 | The data file's initial timestamp, all 64 bits. The data file's own header keeps the low 40 ([SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md), bytes 251-255), so pair on those |
| 64 | 4 | Config time |
| 68 | 6 | Firmware ID, major, minor, internal (u16, u16, u8, u8) |
| 74 | 6 | Bluetooth MAC |
| 80 | 8 | **Shimmer clock of the first sample** |
| 88 | 8 | ADF decimation (u32), gain (i32) |
| 96 | 1 | **Closed**: 1 when the file closed normally |
| 97 | 1 | The session's first fault (§5), 0 if none |
| 100 | 4 | Samples in the file |
| 104 | 8 | **Shimmer clock of the last sample**, 0 if not known: a file cut short at stop more than about 8 s behind the capture (§1) |
| 112 | 4 | Samples written as silence after an overrun |
| 116 | 4 | Escape-hatch writes (u16), write errors (u16), since the session began; each stops at 65535 |
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
  slips. On the bench, a card made to take nothing for 12 s left 65,536 samples
  (4.1 s) of silence at exactly the samples the ring lost. Every sample after
  them kept its index, and the recording then caught up in about 140 writes.
  That was measured when the escape hatch (§4) also paid silence; it no longer
  does, so `TASK_AUDIO` alone now pays it.

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
| 116 | 4 | Escape-hatch writes (u16), write errors (u16), since the session began; each stops at 65535 |
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
- The gain (SCALE) depends on the microphone fitted, since the two parts are
  15 dB apart in sensitivity: code 6 (+18.1 dB) for the MP23DB01HP, and code 1
  (+3.5 dB) for the IM68D121JV01 on the ".3" boards
  (`ShimBrd_isIm68d121PresentPerSrNumber()`,
  [SHIMMER3_BOARD_REVISIONS.md](SHIMMER3_BOARD_REVISIONS.md)). The WAV header
  records the code used (§3.1, offset 92).

**What the gain gives.** The figures below are worked out from the datasheets
and RM0456, not measured against a sound level:
- **The chain:** a full-scale PDM stream gives 24^5 out of the sinc5. SCALE
  follows, then the reshape filter (about +9.3 dB), then the 24-bit output, of
  which the WAV keeps the top 16 bits. At code 6 the WAV runs 26.9 dB above the
  microphone's own dBFS.
- **What clips first:** the reshape filter's input, limited to 22 bits. It
  saturates at about −2.7 dBFS in the WAV, so loud sound flattens a little below
  full scale.

| | MP23DB01HP, code 6 | IM68D121JV01, code 1 |
|---|---|---|
| Sensitivity (94 dB SPL, 1 kHz) | −41 dBFS (performance mode) | −26 dBFS |
| A 94 dB SPL tone in the WAV | −14.1 dBFS peak | −13.7 dBFS peak |
| Clips at | 105 dB SPL | 105 dB SPL |
| Microphone self-noise in the WAV | −82 dBFS(A), 19 dB above 16-bit quantisation | −85 dBFS(A), 16 dB above |

- **The bench recordings fit this.** On MP23DB01HP boards a quiet room came out
  at −60 to −67 dBFS RMS, which this puts at 44-51 dB SPL. A soundtrack played
  loud at 30 cm came out at 67-77 dB SPL, with peaks at 92.
- **The PDM clock must stay at 1.536 MHz or above.** The MP23DB01HP's
  performance mode starts at 1.5 MHz; below it, its low-power mode is 17 dB
  more sensitive.
- **At code 6 the IM68D121JV01 would clip at 90 dB SPL,** which a raised voice
  near the unit can reach. That is why the gain follows the board.
- **An unprogrammed daughter card reads as the MP23DB01HP.**

**How samples reach the card:**
- GPDMA1 channel 11 writes the samples straight into an 8 s ring, through a
  circular linked list of 64 nodes, each a 128 ms block.
- Each block's interrupt only stamps it with the Shimmer clock and queues
  `TASK_AUDIO`, and `TASK_AUDIO_LEVEL` while the sound level is on (§8).
  Nothing is copied at interrupt level.
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
- The hatch stops once the ring has overrun. What is owed first then is
  silence, which frees nothing in the ring, so the hatch would only add a card
  write to every pass of the main loop. On the bench, with a card that had
  slowed to about 0.2 s a write, that made every pass long enough for the
  sampling interrupts to queue `TASK_SAVEDATA` again before it ended. The
  data file's writes and the dock then waited for hours (DEV-99). `TASK_AUDIO`
  pays the silence as the task list allows.

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
- The tables the analysis need take 68.9 ms to build (3,308,553 cycles at
  48 MHz, measured on the bench), so the firmware builds them at boot, before
  anything samples.
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
any fault, without polling. Releases before `LogAndStream_Shimmer3R_v1.01.026`,
the first with audio, send zero in these bits, so gate them on the firmware
version: v1.01.026 or later. The development builds with audio from before that
release report v1.01.025, the release they were cut from. On that version, ask
with `GET_AUDIO_STATUS_COMMAND`, which firmware without audio leaves unanswered.

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
| 12 | 2 | Escape-hatch writes; stops at 65535 |
| 14 | 2 | Write errors; stops at 65535 |
| 16 | 4 | ADF error interrupts |
| 20 | 4 | Of them, saturation |
| 24 | 4 | Fingerprint chunks written this session |
| 28 | 4 | Fingerprint chunks skipped this session |

Faults 4 and 5 are degradations, not stops: recording carries on, and the fault
bit says the WAV has silence in it or the fingerprint has gaps. The other faults
stop the recording: the files are closed with the fault in their headers, the
recording bit clears and the fault bit sets, and logging carries on. A file
closed by a failed write ends at its last whole write, and its last-sample clock
is that sample's. The fault bit stays set after the log stops, until the next
session starts.

Each fault has been forced on the bench, with logging at 1024 Hz losing nothing:

| Fault | How it was forced | What followed |
|---|---|---|
| 4 | The WAV took nothing for 12 s | Fault bit set when the ring overran; recording on, seconds paused, 65,536 silence samples |
| 5 | The analysis did nothing for 12 s | Fault bit set at the next chunk; 3 chunks skipped, numbering and hashes of the rest still right |
| 3 | A WAV write failed at 20 s | State 4, recording bit clear and fault bit set within a second; both files closed at 20 s |

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

**What it costs.** Measured on a Shimmer3R (an ExG board, 47-7-1) powered at 3.7 V
on its battery connector by a power profiler. It was logging at 128 Hz (LN accel,
gyro, mag, VBatt), undocked, with Bluetooth on but not connected. Each figure is
the mean of 2 minutes:

| Logging with | Current | Added by audio |
|---|---:|---:|
| No audio (mode 0) | 22.1 mA | |
| The fingerprint (mode 1) | 24.0 mA | +1.8 mA (+8%) |
| Raw audio (mode 3) | 26.4 mA | +4.3 mA (+19%) |
| Both (mode 2) | 26.8 mA | +4.7 mA (+21%) |

From the differences:
- the capture (the microphone, the ADF and its clock, GPDMA and the 128 ms
  interrupts) costs about 1.4 mA;
- the fingerprint's analysis costs about 0.4 mA;
- raw audio's card writes cost about 2.9 mA.

The write cost depends on the card. The fingerprint's 4 s cycle shows in the
current, and the 4 s averages varied by about 0.2 mA within each run.

**The sound level** (§8) keeps the capture running whenever the unit senses
with it on, streaming included. That costs the capture's share above, about
1.4 mA, plus its filter: about 1.5 ms of CPU every 128 ms, estimated, not
measured. Its current while streaming has not been measured yet. For
comparison, the same unit undocked and not logging drew 20.0 mA. A raw-audio
build wrote zeros in place of the room, the same number of bytes, so the card
saw the same writes.

## 7. The factory microphone test

The factory test S3R_TEST_0026 captures through the same driver, at its own
settings (decimation 64, gain −8). It runs one capture at a time. A factory test
while a recording or the sound level (§8) holds the microphone would report
`FAIL: Microphone busy recording`, whichever it is,
rather than restart the ADF under the recording
([SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md) §5.6).

In practice neither route can start one while the unit senses, so the check is
a safeguard:
- **Bluetooth** refuses `SET_FACTORY_TEST` while sensing
  (`ShimBt_isCmdBlockedWhileSensing`).
- **A docked Shimmer3R's USB is down while it senses,** so its dock UART is not
  there either.

On the bench, with the sound level on and the unit streaming docked, both were
tried: no test ran until the stream stopped.

## 8. The sound level

Configuration byte 233 bit 4
([SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §8.5)
adds the `SOUND_LEVEL` channel, ID `0x38`
([SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §3), to
every data packet: the A-weighted level at the microphone, in nominal dB SPL.
- **It is a sensor channel like any other:** listed in the inquiry response and
  the SD header, at the sampling rate, logged and streamed.
- **It needs no log, unlike the recording (§1).** The capture runs whenever the
  unit senses with the bit set, whether it streams, logs or both, and stops when
  sensing stops.
- **It stores no audio.**

| | |
|---|---|
| Position | After the ExG blocks and before the ADS7028 block, so `VBATT` stays last (STREAMING_DATA_FORMAT §4.2) |
| Encoding | int16, little-endian, hundredths of a dB(A) SPL: 4512 is 45.12 dB(A) |
| No level | `-32768` (`0x8000`): before the first level after the capture starts, and while it is not running |
| Each value | LAeq over one 128 ms capture block: the mean square of the A-weighted samples |

**What it measures:**
- **The A-weighting** is IEC 61672's curve as three biquads at 16 kHz. It is
  within 0.1 dB of the curve from 20 Hz to 7 kHz (`Test/host/test_audio_level.c`).
  The microphone's and the ADF's own frequency response are not corrected.
- **The step to dB SPL** is the microphone's offset at the capture's gain.
  - **From §4's figures:** a 94 dB SPL tone comes out at −14.1 dB against a
    full-scale sine on the MP23DB01HP at code 6, and at −13.7 dB on the
    IM68D121JV01 at code 1. So the offsets are 108.1 and 107.7 dB.
  - **Worked out, not measured:** the offsets come from the datasheets, so the
    level is **nominal**. It is good for changes and comparisons on one unit,
    and a few dB out in absolute terms.
  - **The plastics move it too:** on the bench, +2.3 dB at 1 kHz and −3.8 dB at
    4 kHz.
- **The range:** values are clamped to 0-327.67 dB, and 0 means digital silence.
  The loudest an int16 capture can show is a full-scale square wave, 111.1 dB
  on the MP23DB01HP; the capture clips near 105 dB SPL (§4).

**When a value applies:**
- **The first value** comes with the third block, 256 to 384 ms after sensing
  starts: 318-320 ms in every streamed run on the bench. The first two blocks are
  the microphone's start-up (§1), and until then packets carry `-32768`.
  - **A log that starts from rest is slower.** Its start holds the main loop while
    it opens its files, and the level waits behind it: on the bench the first
    value came 0.84-0.93 s in.
  - **After a configuration write** the first log's file opening can take longer
    still, on cards with small clusters (DEV-1429).
- **A value describes the last block worked.** That is the 128 ms that ended up
  to one block before the packet, plus the wait for `TASK_AUDIO_LEVEL`.
  - **The wait is short.** On the bench, a block's value was in the packets a
    median of 3.6 ms after the block ended, and 99% within 7.4 ms.
  - **So the middle of the block** a value describes is 64 to 192 ms before the
    packet's timestamp, plus that wait.
  - For a level that matters over seconds this makes no difference.
  - To place a sharp sound in time, use the WAV (§3.1).
- **While the main loop is held,** the value holds too.
  - **What holds it:** a log opening its files mid-stream did, for about a second
    on the bench.
  - **When the loop is free again:** the task finds more than 4 blocks waiting,
    restarts its filter on the newest, and carries on.
  - **What is lost:** the blocks in between get no value of their own. On the
    bench that happened once in a 120 s run, at the log's start: six blocks, after
    which the next value was exact again.
- **Each value repeats** until the next block: at 1024 Hz, about 131 packets in
  a row carry the same value.

**How it is worked out:**
- **`TASK_AUDIO_LEVEL`** runs each completed block through the filter as it
  arrives, at an estimated 1.5 ms of CPU a block. It sits above the data path,
  unlike `TASK_AUDIO`, so the level stays current while the card is slow.
  - The filter's state carries from block to block.
  - If the task falls more than 4 blocks behind, it restarts the filter on the
    block before the newest rather than work through old audio.
- **The sampling interrupt** copies the latest value into each packet. The
  channel adds no bus to wait for: with no other SPI channel, it completes the
  packet's SPI stage itself.
- **The capture** is the recording's, shared (§1).

**For hosts:**
- **Detecting support:** `LogAndStream_Shimmer3R_v1.01.026` is the first
  release with the channel. Firmware without it clears bit 4, so a host learns
  whether a unit has it by writing the bit and reading it back, which also
  answers for the development builds before that release.
- **Older hosts:** one that does not know `0x38` cannot know its width, so it
  should refuse the stream rather than guess (STREAMING_DATA_FORMAT §5.6).

## Not yet

**Firmware:**
- The gain checked against a sound level (§4 is worked out, not measured), and
  any recording at all on an IM68D121JV01 board.
- Gain presets. Byte 233 bits 2-3 are reserved for them.
- An LED cue while raw audio records.
- Whether to mask the ADF's saturation flag. In bench recordings it fired 260
  to 280 times in the first 11 s and never after, a start-up transient. It
  counts real overload too: nine taps on a bare board's SD card cage, which
  drove the WAV to full scale, took one recording's count to 2,106. So it is
  worth keeping, if only as a count.

**The sound level (§8):**
- Its level checked against a calibrated sound level meter, on each microphone
  part, bare and in plastics.
- Its current while the unit only streams, and its CPU time per block.

**Benched** (Shimmer3R, MP23DB01HP, bench-only image that writes a known signal
into each capture block in the DMA interrupt):
- **The arithmetic:** in a 120 s stream at 512 Hz, all 928 values matched the
  same filter worked on a PC to within 0.01 dB.
- **A log joined mid-stream and stopped while the stream went on:**
  - the WAV matched the signal sample for sample (942,080 samples, no silence,
    closed, no fault);
  - the log's 30,719 records were all in the stream, byte for byte.
- **Streaming at 1024 Hz with every sensor on:** classic (CRC off) and BLE (CRC
  1), 60 s each, lost nothing.
- **The release matrix's four `level_*` rows** passed.

**Hosts:**
- Byte 233 in the host APIs' configuration models.
- The audio status bits.
- An importer that pairs `audio/` with `data/` and places the samples and the
  fingerprint's chunks on the Shimmer clock.
