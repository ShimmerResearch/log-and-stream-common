"""Check the audio a Shimmer3R records while it logs (docs/SHIMMER3R_AUDIO.md).

audio/X/NNN.WAV (modes 2 and 3) and audio/X/NNN.AFP (modes 1 and 2) belong
with data/X/NNN (§2). A release row's log is short, so there is one of each,
NNN = 000. check() reads their headers, walks the fingerprint's chunks, and
compares both with the data file and with the GET_AUDIO_STATUS_COMMAND reply
taken after the log stopped.

What is judged is structure and timing: the files are there and paired, closed
cleanly with no fault, cover the log, keep the Shimmer clock, and agree with
the status. What the microphone heard is the bench room's business. The WAV's
level is reported, and judged only far enough to show sound arrived. Nothing
here recomputes the fingerprint: that needs the algorithm on the PC.

A WAV records whoever is near the bench. It is read where it lies on the card,
nothing of its sound is kept, and remove_raw() deletes it once it is checked.
"""

from __future__ import annotations

import math
import struct
from array import array
from dataclasses import dataclass, replace
from pathlib import Path

RATE = 16000
HEADER = 512
CHUNK_SAMPLES, CHUNK_HOP = 80000, 64000
FRAME_HOP = 256
MAX_T1 = 312                                     # the last frame of a chunk (§3.2)
MAX_RECORDS = 4695                               # per chunk (§2)
CHUNK_MARK = 0x4843
TICKS = 32768
TS_WRAP = 1 << 24
# The data file's header keeps the low 40 bits of its initial timestamp
# (SD_CARD_FORMAT, bytes 251-255); the audio headers keep all 64.
HEADER_TS_BITS = 40
# The audio clock runs from the 16 MHz crystal, the Shimmer clock from the
# 32 kHz one: -23 and +56 ppm apart on two bench units (§3.1).
MAX_PPM = 300
# A chunk's measured clock off the line through all of them: at most 4 ticks
# on the bench at 1024 Hz with every sensor on (§3.2).
CHUNK_CLOCK_TICKS = 8
# Where the audio sits against the data file (§1): the first sample 0.36-0.48 s
# after its initial timestamp, and capture stops when logging does.
FIRST_SAMPLE_S = (0.1, 1.5)
# With the sound level on, the capture is already running when the log starts
# and the session joins it at the block then filling (§1), up to 128 ms before
# the recorder's task opens the files
FIRST_SAMPLE_JOINED_S = (-0.2, 1.5)
LAST_SAMPLE_S = (-1.5, 0.5)
# Sound arrived: a quiet room is about -67 dBFS RMS; a dead microphone or a
# stuck PDM line leaves nothing after the high-pass filter.
MIN_RMS_LSB = 1.0

WAV_MODES, AFP_MODES = {2, 3}, {1, 2}
STATES = {0: "off", 1: "starting", 2: "recording", 3: "stopped", 4: "fault"}
FAULTS = {0: "none", 1: "open", 2: "capture", 3: "write", 4: "overrun", 5: "fingerprint fell behind"}


@dataclass
class Identity:
    """Bytes 44-95 of either file, and the summary from 96 (§3.1, §3.2)."""
    magic: bytes
    version: int
    hw: int
    mode: int
    reason: int
    file_no: int
    data_ts: int
    config_time: int
    fw: tuple
    mac: str
    first_ticks: int
    closed: int
    fault: int
    count: int              # samples in a WAV, chunks in an .AFP
    last_ticks: int
    count2: int             # silence samples in a WAV, records in an .AFP
    escapes: int
    write_errors: int
    adf_errors: int
    adf_saturation: int
    adf_overflow: int


def identity(raw: bytes) -> Identity:
    return Identity(
        magic=raw[44:48], version=raw[48], hw=raw[49], mode=raw[50], reason=raw[51],
        file_no=struct.unpack_from("<H", raw, 52)[0],
        data_ts=struct.unpack_from("<Q", raw, 56)[0],
        config_time=struct.unpack_from("<I", raw, 64)[0],
        fw=struct.unpack_from("<HHBB", raw, 68),
        mac=raw[74:80].hex().upper(),
        first_ticks=struct.unpack_from("<Q", raw, 80)[0],
        closed=raw[96], fault=raw[97],
        count=struct.unpack_from("<I", raw, 100)[0],
        last_ticks=struct.unpack_from("<Q", raw, 104)[0],
        count2=struct.unpack_from("<I", raw, 112)[0],
        escapes=struct.unpack_from("<H", raw, 116)[0],
        write_errors=struct.unpack_from("<H", raw, 118)[0],
        adf_errors=struct.unpack_from("<I", raw, 120)[0],
        adf_saturation=struct.unpack_from("<I", raw, 124)[0],
        adf_overflow=struct.unpack_from("<I", raw, 128)[0],
    )


@dataclass
class AudioStatus:
    """GET_AUDIO_STATUS_COMMAND's reply (§5)."""
    version: int
    mode: int
    state: int
    fault: int
    seconds: int
    silence: int
    escapes: int
    write_errors: int
    adf_errors: int
    adf_saturation: int
    chunks: int
    chunks_skipped: int


def audio_status(payload: bytes) -> AudioStatus:
    if len(payload) != 32:
        raise ValueError(f"audio status is {len(payload)} bytes, expected 32")
    return AudioStatus(payload[0], payload[1], payload[2], payload[3],
                       *struct.unpack_from("<IIHHIIII", payload, 4))


@dataclass
class DataFile:
    """What the audio is held against: the data file's header and its records' span."""
    initial_ts: int         # the RTC when the file opened: header bytes 251-255, its low 40 bits, until lifted
    config_time: int        # header bytes 52-55, MSB first
    mac: str
    fw: tuple
    audio_mode: int         # header byte 226, SDH_AUDIO_MODE
    first_ts24: int         # the first record's 24-bit stamp
    span_ticks: int         # first record to last

    @classmethod
    def from_file(cls, head: bytes, first_ts24: int, span_ticks: int) -> "DataFile":
        initial = int.from_bytes(head[252:256], "little") | (head[251] << 32)
        return cls(initial_ts=initial, config_time=int.from_bytes(head[52:56], "big"),
                   mac=head[24:30].hex().upper(),
                   fw=(int.from_bytes(head[34:36], "big"), int.from_bytes(head[36:38], "big"), head[38], head[39]),
                   audio_mode=head[226], first_ts24=first_ts24, span_ticks=span_ticks)

    def lifted(self, full_ts: int) -> "DataFile":
        """This file with its initial timestamp's high bits taken from an audio
        header's 64-bit copy, if that copy's low 40 bits are this one."""
        if full_ts % (1 << HEADER_TS_BITS) != self.initial_ts:
            return self
        return replace(self, initial_ts=full_ts)

    @property
    def first_ticks(self) -> int:
        """The first record on the full Shimmer clock. The packet ring holds
        samples taken before the file opened, so it precedes initial_ts."""
        return self.initial_ts - ((self.initial_ts - self.first_ts24) % TS_WRAP)

    @property
    def last_ticks(self) -> int:
        return self.first_ticks + self.span_ticks


@dataclass
class Chunk:
    index: int
    ticks: int
    records: int


def read_chunks(raw: bytes, problems: list) -> list:
    """Walk an .AFP's chunks from byte 512, checking each record's fields."""
    out, pos = [], struct.unpack_from("<H", raw, 6)[0]
    while pos + 16 <= len(raw):
        mark, n, k, ticks = struct.unpack_from("<HHIQ", raw, pos)
        if mark != CHUNK_MARK:
            problems.append(f"AFP: chunk header at byte {pos} has mark {mark:#06x}")
            break
        end = pos + 16 + 4 * n
        if end > len(raw):
            problems.append(f"AFP: chunk {k} says {n} records, the file ends first")
            break
        bad = [r for (r,) in struct.iter_unpack("<I", raw[pos + 16:end]) if r >> 28 or (r & 0x1FF) > MAX_T1]
        if bad:
            problems.append(f"AFP: chunk {k} has {len(bad)} records with bits 28-31 set or t1 past {MAX_T1}")
        if n > MAX_RECORDS:
            problems.append(f"AFP: chunk {k} has {n} records, more than {MAX_RECORDS}")
        out.append(Chunk(k, ticks, n))
        pos = end
    if pos != len(raw) and not problems:
        problems.append(f"AFP: {len(raw) - pos} bytes after the last whole chunk")
    return out


def _fit(xs, ys):
    """Least-squares line y = a + b x; returns (a, b)."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return my - b * mx, b


def _identity_checks(name: str, ident: Identity, mode: int, data: DataFile) -> list:
    p = []
    if ident.magic != b"SAUD" or ident.version != 1 or ident.hw != 10:
        p.append(f"{name}: identity {ident.magic!r} version {ident.version} hardware {ident.hw}")
    if ident.mode != mode:
        p.append(f"{name}: records mode {ident.mode}, configured {mode}")
    if ident.reason != 0 or ident.file_no != 0:
        p.append(f"{name}: opened for reason {ident.reason} as file {ident.file_no}, expected the session's first, 000")
    if ident.data_ts != data.initial_ts:
        p.append(f"{name}: data file timestamp {ident.data_ts} (low 40 bits "
                 f"{ident.data_ts % (1 << HEADER_TS_BITS)}), the data file's header says {data.initial_ts}")
    if ident.config_time != data.config_time:
        p.append(f"{name}: config time {ident.config_time}, the data file's {data.config_time}")
    if ident.mac != data.mac or ident.fw != data.fw:
        p.append(f"{name}: MAC {ident.mac} firmware {ident.fw}, the data file's {data.mac} {data.fw}")
    if ident.closed != 1:
        p.append(f"{name}: not closed (byte 96 = {ident.closed})")
    if ident.fault:
        p.append(f"{name}: fault {ident.fault} ({FAULTS.get(ident.fault, '?')})")
    if ident.escapes or ident.write_errors or ident.adf_overflow:
        p.append(f"{name}: {ident.escapes} escape-hatch writes, {ident.write_errors} write errors, "
                 f"{ident.adf_overflow} ADF overflows")
    return p


@dataclass
class AudioCheck:
    problems: list
    note: str


def audio_dir(data_path: Path) -> Path:
    """.../data/X/S/000 -> .../audio/X/S, the companion directory (§2)."""
    parts = Path(data_path).parts
    i = len(parts) - 1 - parts[::-1].index("data")
    return Path(*parts[:i], "audio", *parts[i + 1:-1])


def check(data_path: Path, data: DataFile, mode: int, status: AudioStatus | None,
          first_window: tuple[float, float] = FIRST_SAMPLE_S) -> AudioCheck:
    """The session's audio files beside data_path (.../data/X/S/000), for a log
    recorded in `mode`, against its data file and the status read after the stop."""
    data_path = Path(data_path)
    adir = audio_dir(data_path)
    wav_path, afp_path = adir / f"{data_path.name}.WAV", adir / f"{data_path.name}.AFP"
    p, notes = [], []
    if mode == 0:
        # Nothing else is judged, so a candidate from before DEV-99 passes too.
        if adir.exists():
            p.append(f"audio off, but {adir} exists")
        return AudioCheck(p, "")
    # Byte 226 is the whole corrected byte 233: bit 4 is the sound level (§8)
    if data.audio_mode & 0x03 != mode:
        p.append(f"data file header byte 226 says audio mode {data.audio_mode & 0x03} "
                 f"({data.audio_mode:#04x}), configured {mode}")

    present = {e.name for e in adir.iterdir()} if adir.is_dir() else set()
    want = ({wav_path.name} if mode in WAV_MODES else set()) | ({afp_path.name} if mode in AFP_MODES else set())
    if present != want:
        p.append(f"audio files {sorted(present) or 'none'}, expected {sorted(want)} in mode {mode}")
    if status is not None:
        if (status.version, status.mode, status.state, status.fault) != (1, mode, 3, 0):
            p.append(f"audio status after the stop: version {status.version}, mode {status.mode}, "
                     f"state {STATES.get(status.state, status.state)}, fault "
                     f"{FAULTS.get(status.fault, status.fault)}; expected 1, {mode}, stopped, none")
        if status.silence or status.escapes or status.write_errors or status.chunks_skipped:
            p.append(f"audio status: {status.silence} silence samples, {status.escapes} escape-hatch writes, "
                     f"{status.write_errors} write errors, {status.chunks_skipped} chunks skipped")

    wav = afp = None
    samples = None
    if wav_path.name in present:
        raw = wav_path.read_bytes()
        wav = identity(raw)
        data = data.lifted(wav.data_ts)
        p += _identity_checks("WAV", wav, mode, data)
        fmt = struct.unpack_from("<4sI4s4sIHHIIHH4sI", raw, 0)
        if fmt != (b"RIFF", len(raw) - 8, b"WAVE", b"fmt ", 16, 1, 1, RATE, 2 * RATE, 2, 16, b"shim", 460):
            p.append(f"WAV: RIFF and fmt header {fmt}")
        samples = (len(raw) - HEADER) // 2
        if raw[504:508] != b"data" or struct.unpack_from("<I", raw, 508)[0] != 2 * samples or wav.count != samples:
            p.append(f"WAV: data chunk {raw[504:508]!r} of {struct.unpack_from('<I', raw, 508)[0]} bytes, "
                     f"header says {wav.count} samples, the file holds {samples}")
        if wav.count2:
            p.append(f"WAV: {wav.count2} samples written as silence after an overrun")
        p += _clock_checks("WAV", wav.first_ticks, wav.last_ticks, samples, data, first_window)
        x = array("h", raw[HEADER:HEADER + 2 * samples])
        if x:
            mean = sum(x) / len(x)
            rms = (sum((v - mean) ** 2 for v in x) / len(x)) ** 0.5
            peak = max(max(x), -min(x))
            if rms < MIN_RMS_LSB:
                p.append(f"WAV: no sound (RMS {rms:.2f} LSB): microphone or capture dead")
            level = 20 * math.log10(max(rms, 1e-3) / 32768)
            notes.append(f"WAV {samples / RATE:.1f} s, RMS {level:.0f} dBFS, peak {peak}"
                         + (f", {sum(1 for v in x if v in (32767, -32768))} at full scale" if peak >= 32767 else ""))
            del x
        if status is not None and status.seconds != samples // RATE:
            p.append(f"audio status: {status.seconds} s recorded, the WAV holds {samples / RATE:.2f} s")

    if afp_path.name in present:
        raw = afp_path.read_bytes()
        afp = identity(raw)
        data = data.lifted(afp.data_ts)
        p += _identity_checks("AFP", afp, mode, data)
        geom = struct.unpack_from("<4sHHHHHHII20s", raw, 0)
        if geom != (b"SAFP", 1, HEADER, RATE, FRAME_HOP, 16, 4, CHUNK_SAMPLES, CHUNK_HOP, bytes(20)):
            p.append(f"AFP: header bytes 0-43 {raw[:44].hex()}")
        skipped = struct.unpack_from("<I", raw, 132)[0]
        chunks = read_chunks(raw, p)
        records = sum(c.records for c in chunks)
        ks = [c.index for c in chunks]
        if ks != list(range(len(chunks))):
            p.append(f"AFP: chunk numbers {ks[:8]}{'...' if len(ks) > 8 else ''}, expected 0 up without a gap")
        if (afp.count, afp.count2) != (len(chunks), records) or skipped:
            p.append(f"AFP: header says {afp.count} chunks, {afp.count2} records, {skipped} skipped; "
                     f"the file holds {len(chunks)} chunks, {records} records")
        if chunks and (afp.first_ticks, afp.last_ticks) != (chunks[0].ticks, chunks[-1].ticks):
            p.append(f"AFP: header's first and last chunk clocks {afp.first_ticks}, {afp.last_ticks}; "
                     f"the chunks' {chunks[0].ticks}, {chunks[-1].ticks}")
        if status is not None and (status.chunks, status.chunks_skipped) != (len(chunks), skipped):
            p.append(f"audio status: {status.chunks} chunks written, {status.chunks_skipped} skipped; "
                     f"the file holds {len(chunks)}, skipped {skipped}")
        # Chunk k starts at the session's sample k x 64000 (§3.2): on the WAV's
        # clock in mode 2, otherwise from where the audio began to the log's end.
        if samples is not None:
            heard = samples
        else:
            heard = int((data.last_ticks - (chunks[0].ticks if chunks else data.initial_ts)) * RATE / TICKS)
        expect = (heard - CHUNK_SAMPLES) // CHUNK_HOP + 1 if heard >= CHUNK_SAMPLES else 0
        # The chunk still filling or being analysed at the stop is dropped (§1).
        lo, hi = expect - 2, expect + (0 if samples is not None else 1)
        if not lo <= len(chunks) <= hi:
            p.append(f"AFP: {len(chunks)} chunks for {heard / RATE:.1f} s of audio, expected {max(lo, 0)}-{hi}")
        if len(chunks) >= 3:
            a, b = _fit(ks, [c.ticks for c in chunks])
            ppm = ((CHUNK_HOP * TICKS / RATE) / b - 1) * 1e6     # + when the audio clock runs fast
            worst = max(abs(c.ticks - (a + b * c.index)) for c in chunks)
            if abs(ppm) > MAX_PPM or worst > CHUNK_CLOCK_TICKS:
                p.append(f"AFP: chunk clocks run {ppm:+.0f} ppm, the worst {worst:.1f} ticks off their line")
            notes.append(f"AFP {len(chunks)} chunks, {records} hashes, audio clock {ppm:+.0f} ppm")
        else:
            notes.append(f"AFP {len(chunks)} chunks, {records} hashes")
        p += _clock_checks("AFP", chunks[0].ticks if chunks else afp.first_ticks, None, None, data,
                           first_window)
        if wav is not None and chunks and samples and samples > 1:
            per = (wav.last_ticks - wav.first_ticks) / (samples - 1)
            off = [c.ticks - (wav.first_ticks + c.index * CHUNK_HOP * per) for c in chunks
                   if c.index * CHUNK_HOP < samples]
            if off and max(abs(o) for o in off) > CHUNK_CLOCK_TICKS:
                p.append(f"AFP: chunk clocks {min(off):+.1f} to {max(off):+.1f} ticks off the WAV's clock")

    for name, ident in (("WAV", wav), ("AFP", afp)):
        if ident is not None and status is not None and \
                (ident.adf_errors, ident.adf_saturation) != (status.adf_errors, status.adf_saturation):
            p.append(f"{name}: ADF errors {ident.adf_errors}, of them saturation {ident.adf_saturation}; "
                     f"the status says {status.adf_errors}, {status.adf_saturation}")
    ref = wav or afp
    if ref is not None:
        notes.append(f"ADF saturation {ref.adf_saturation}")
    return AudioCheck(p, "; ".join(notes))


def _clock_checks(name, first, last, samples, data: DataFile,
                  first_window: tuple[float, float] = FIRST_SAMPLE_S) -> list:
    """The audio's first sample just after the data file opened, its last at the
    log's end, and the audio clock within MAX_PPM of the Shimmer clock."""
    p = []
    d0 = (first - data.initial_ts) / TICKS
    if not first_window[0] <= d0 <= first_window[1]:
        p.append(f"{name}: first sample {d0:+.2f} s from the data file's initial timestamp, "
                 f"expected {first_window[0]} to {first_window[1]} s")
    if last == 0:
        # A WAV cut short at stop far behind the capture: its last sample's time is not known
        p.append(f"{name}: no last-sample time (the file was cut short at stop)")
    elif last is not None:
        d1 = (last - data.last_ticks) / TICKS
        if not LAST_SAMPLE_S[0] <= d1 <= LAST_SAMPLE_S[1]:
            p.append(f"{name}: last sample {d1:+.2f} s from the data file's last record, "
                     f"expected {LAST_SAMPLE_S[0]} to {LAST_SAMPLE_S[1]} s")
        if samples and samples > 1 and last > first:
            ppm = ((samples - 1) * TICKS / (last - first) / RATE - 1) * 1e6
            if abs(ppm) > MAX_PPM:
                p.append(f"{name}: {samples} samples over {(last - first) / TICKS:.3f} s of Shimmer clock, "
                         f"{ppm:+.0f} ppm")
    return p


def remove_raw(data_path: Path) -> list:
    """Delete the session's WAVs from the card, as they hold whatever the bench
    room said. Returns the names removed."""
    adir = audio_dir(data_path)
    gone = sorted(adir.glob("*.WAV")) if adir.is_dir() else []
    for w in gone:
        w.unlink()
    return [w.name for w in gone]
