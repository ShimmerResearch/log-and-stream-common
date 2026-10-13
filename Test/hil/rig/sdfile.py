"""Read a Shimmer3R SD data file well enough to check it (docs/SHIMMER3_SD_CARD_FORMAT.md).

A Shimmer3R file is self-describing: the 384-byte header records the channel
list (§3.5), so records can be sliced without knowing the configuration.
Shimmer3 files are not, and are out of scope here.

Blocks: records accumulate into a 512-byte buffer, so the body is a run of
blocks of floor((512 - syncHead) / recordLen) records, each preceded by a
9-byte sync offset when SD sync is on (§2.1). The last block is shorter.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from pathlib import Path

from .bt import HW_SHIMMER3R, TS_WRAP, channel_width, classify_step

HEAD_S3R = 384
SDH_TRIAL_CONFIG0 = 16
SDH_TIME_SYNC = 0x04
SDH_NUM_ENABLED_CHANNELS = 314
SDH_CHANNEL_ID_BYTE_0 = 315
BLOCK = 512
SYNC_HEAD = 9


class SdFileError(Exception):
    pass


@dataclass
class Header:
    divider: int
    mac: str
    hw: int
    fw: tuple[int, int, int, int]
    channels: list[int]
    sync: bool

    @property
    def record_len(self) -> int:
        return 3 + sum(channel_width(c, self.hw) for c in self.channels)


@dataclass
class FileCheck:
    header: Header
    records: int = 0
    zero_ts: int = 0           # records stamped 00 00 00: invalid (§2.1)
    gaps: int = 0
    missing: int = 0
    max_gap_ticks: int = 0
    dupes: int = 0
    jitter: int = 0
    max_jitter_ticks: int = 0
    span_ticks: int = 0
    trailing_bytes: int = 0    # bytes that do not form a whole record
    deltas: dict = field(default_factory=dict)
    ts: list = field(default_factory=list)       # kept with check(keep=True)
    rows: list = field(default_factory=list)


def read_header(raw: bytes) -> Header:
    if len(raw) < HEAD_S3R:
        raise SdFileError(f"file is {len(raw)} bytes, shorter than the {HEAD_S3R}-byte header")
    hw = struct.unpack(">H", raw[30:32])[0]
    if hw != HW_SHIMMER3R:
        raise SdFileError(f"header says hardware {hw}; only Shimmer3R ({HW_SHIMMER3R}) files are read here")
    n = raw[SDH_NUM_ENABLED_CHANNELS]
    return Header(
        divider=struct.unpack("<H", raw[0:2])[0],
        mac=raw[24:30].hex().upper(),
        hw=hw,
        fw=(struct.unpack(">H", raw[34:36])[0], struct.unpack(">H", raw[36:38])[0], raw[38], raw[39]),
        channels=list(raw[SDH_CHANNEL_ID_BYTE_0:SDH_CHANNEL_ID_BYTE_0 + n]),
        sync=bool(raw[SDH_TRIAL_CONFIG0] & SDH_TIME_SYNC),
    )


def check(path: Path, keep: bool = False) -> FileCheck:
    raw = Path(path).read_bytes()
    h = read_header(raw)
    rl = h.record_len
    head = SYNC_HEAD if h.sync else 0
    per_block = (BLOCK - head) // rl
    if per_block < 1:
        raise SdFileError(f"record length {rl} does not fit a block")
    fc = FileCheck(h)
    pos, end = HEAD_S3R, len(raw)
    last = None
    while pos < end:
        pos += head
        for _ in range(per_block):
            if pos + rl > end:
                break
            ts = raw[pos] | (raw[pos + 1] << 8) | (raw[pos + 2] << 16)
            if keep:
                fc.ts.append(ts)
                fc.rows.append(raw[pos + 3:pos + rl])
            pos += rl
            fc.records += 1
            if ts == 0:
                fc.zero_ts += 1
                continue
            if last is not None:
                d = (ts - last) % TS_WRAP
                fc.deltas[d] = fc.deltas.get(d, 0) + 1
                fc.span_ticks += d
                classify_step(fc, d, h.divider)
            last = ts
        else:
            continue
        break
    fc.trailing_bytes = max(0, end - pos)
    return fc


def session_dirs(root: Path) -> set[Path]:
    """Every <experiment>/<name>-NNNN session directory under /data."""
    data = Path(root) / "data"
    if not data.is_dir():
        return set()
    return {s for e in data.iterdir() if e.is_dir() for s in e.iterdir() if s.is_dir()}


def iter_records(path: Path):
    """(timestamp, record bytes) for every whole record of one file, in order."""
    raw = Path(path).read_bytes()
    h = read_header(raw)
    rl = h.record_len
    head = SYNC_HEAD if h.sync else 0
    per_block = (BLOCK - head) // rl
    pos, end = HEAD_S3R, len(raw)
    while pos < end:
        pos += head
        for _ in range(per_block):
            if pos + rl > end:
                return
            yield raw[pos] | (raw[pos + 1] << 8) | (raw[pos + 2] << 16), raw[pos + 3:pos + rl]
            pos += rl


@dataclass
class SessionCheck:
    files: list[str]
    header: Header | None = None
    records: int = 0
    zero_ts: int = 0
    gaps: int = 0
    missing: int = 0
    max_gap_ticks: int = 0
    dupes: int = 0
    jitter: int = 0
    max_jitter_ticks: int = 0
    span_ticks: int = 0
    joins: list = field(default_factory=list)          # (file, step in ticks) where one file meets the next
    header_mismatches: list = field(default_factory=list)
    gap_times_s: list = field(default_factory=list)    # seconds into the session of each gap, first 200

    @property
    def hours(self) -> float:
        return self.span_ticks / 32768 / 3600

    def loss_fraction(self) -> float:
        total = self.records + self.missing
        return self.missing / total if total else 0.0


def check_session(session_dir: Path) -> SessionCheck:
    """Every data file of one session (000, 001, ...) judged as one recording.

    Files roll over hourly (SD_CARD_FORMAT §4.1). The first record of a file
    must follow the last of the one before by one sample period; a step there
    is a loss at the split, and is listed in `joins` as well as counted.
    """
    files = sorted((p for p in Path(session_dir).iterdir() if p.is_file() and p.name.isdigit()),
                   key=lambda p: int(p.name))
    sc = SessionCheck(files=[p.name for p in files])
    last = None
    for f in files:
        raw_head = f.read_bytes()[:HEAD_S3R]
        h = read_header(raw_head)
        if sc.header is None:
            sc.header = h
        elif (h.divider, h.channels, h.mac) != (sc.header.divider, sc.header.channels, sc.header.mac):
            sc.header_mismatches.append(f.name)
        first_in_file = True
        for ts, _ in iter_records(f):
            sc.records += 1
            if ts == 0:
                sc.zero_ts += 1
                continue
            if last is not None:
                d = (ts - last) % TS_WRAP
                if first_in_file:
                    sc.joins.append((f.name, d))
                gaps_before = sc.gaps
                sc.span_ticks += d
                classify_step(sc, d, sc.header.divider)
                if sc.gaps != gaps_before and len(sc.gap_times_s) < 200:
                    sc.gap_times_s.append(round(sc.span_ticks / 32768, 1))
            last = ts
            first_in_file = False
    return sc
