"""The event channel (DEV-971) as the release tier checks it: configuration
byte 232, the event file beside a data file, and the Bluetooth record stream.

Written against docs/SHIMMER3_CONFIGURATION_INFOMEM.md §8.3,
docs/SHIMMER3_EVENT_CHANNEL.md and SHIMMER3_BT_COMMUNICATION_PROTOCOL.md
§7.15. An event file is read with Extras' shimmer_event_file.py, the reader a
person at the bench uses, so the rig and the person judge a file the same way.

The bench has no hand on the button and nothing on the TTL input, so an event
file or a stream here should hold state records only: the initial state, the
heartbeats and the final record. An edge is a finding.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

_EXTRAS = Path(__file__).resolve().parents[3] / "Extras" / "python_scripts" / "Shimmer_common"
sys.path.insert(0, str(_EXTRAS))
import shimmer_crc  # noqa: E402
import shimmer_event_file as evf  # noqa: E402

# --- configuration byte 232 (INFOMEM §8.3) ------------------------------------

BYTE = 232
SD_HEADER_BYTE = 225                            # SDH_BUTTON_LOG_MODE: byte 232 as kept
UBE_BYTE, UBE_BIT = 217, 0x20                   # userButtonEnable
SINGLE_TOUCH_BYTE, SINGLE_TOUCH_BIT = 218, 0x80

MODE_MASK, TRIG_MASK, TRIG_SHIFT, TTL, INVALID = 0x07, 0x18, 3, 0x20, 0x80
MODE_LEGACY, MODE_DISABLED, MODE_SINGLE, MODE_DOUBLE, MODE_DOUBLE_RECORDED, MODE_RECORDED = range(6)
TRIG_HOST, TRIG_HOST_UNDOCK, TRIG_NONE = range(3)

ENABLE_BUTTON, ENABLE_TTL = 0x01, 0x02          # the file header's and the status's enables


def correct(raw: int, *, sr48_6_0: bool = False) -> int:
    """Byte 232 as the firmware keeps it: ShimBtnLogMode_correct, then the
    board rules in ShimConfig_checkAndCorrectConfig. SR48-6-0's button is on
    BOOT0, so it has no presses to record (mode 4 becomes 3, 5 becomes 1), and
    its TTL input is not supported."""
    mode, trig = raw & MODE_MASK, (raw & TRIG_MASK) >> TRIG_SHIFT
    if raw & INVALID:
        v = 0
    elif not MODE_DISABLED <= mode <= MODE_RECORDED or trig > TRIG_NONE:
        v = raw & TTL               # legacy lands here too: its triggers are not read
    else:
        v = raw & (MODE_MASK | TRIG_MASK | TTL)
    if sr48_6_0:
        if v & MODE_MASK == MODE_DOUBLE_RECORDED:
            v = (v & ~MODE_MASK) | MODE_DOUBLE
        elif v & MODE_MASK == MODE_RECORDED:
            v = (v & ~MODE_MASK) | MODE_DISABLED
        v &= ~TTL
    return v


def ube_held(kept: int) -> int | None:
    """userButtonEnable as the correction holds it for a kept byte 232, or None
    in legacy mode, where it is the host's. It is 0 only where a press cannot
    start logging and leaving the dock does (0x09, 0x0D)."""
    mode, trig = kept & MODE_MASK, (kept & TRIG_MASK) >> TRIG_SHIFT
    if mode == MODE_LEGACY:
        return None
    return 0 if mode in (MODE_DISABLED, MODE_RECORDED) and trig == TRIG_HOST_UNDOCK else 1


def enables(kept: int) -> int:
    """The inputs a kept byte records: button modes 4 and 5, and bit 5."""
    return ((ENABLE_BUTTON if kept & MODE_MASK in (MODE_DOUBLE_RECORDED, MODE_RECORDED) else 0)
            | (ENABLE_TTL if kept & TTL else 0))


def undock_starts(kept: int, ube: int) -> bool:
    """Whether leaving the dock starts logging (no single touch: no release has it)."""
    if kept & MODE_MASK == MODE_LEGACY:
        return not ube
    return (kept & TRIG_MASK) >> TRIG_SHIFT == TRIG_HOST_UNDOCK


def logging_off(kept: int) -> bool:
    return kept & MODE_MASK != MODE_LEGACY and (kept & TRIG_MASK) >> TRIG_SHIFT == TRIG_NONE


# --- the event file (EVENT_CHANNEL §4) ----------------------------------------

TYPE_EDGE, TYPE_INIT, TYPE_HEARTBEAT, TYPE_OVERFLOW = 1, 2, 3, 4
STATE_BUTTON, STATE_TTL = 0x01, 0x02            # a record's state byte: pressed, high
OPEN_SESSION_START = 1
TICKS_PER_S = evf.TICKS_PER_S
# A heartbeat is written at the data file's one-minute sync once 60 s have
# passed with nothing written (§4.3). The first sync can come just short of
# that, so a quiet file may go two syncs between records.
MAX_QUIET_S = 125.0
HEARTBEAT_BY_S = 130.0          # a log this long has had a heartbeat
SPAN_SLACK_S = 2.0              # the final record against the data file's last sample
# A data file's first samples can come from before it opened: sampling starts at
# the start command, and the file opens once its session folder is numbered and
# made, which takes seconds in a trial folder of hundreds of sessions (about 4 s
# for 531). Those samples are buffered and written first. The event file starts
# at the open (§4.3), so its span is measured against the data file's from there.
DATA_LEAD_MAX_S = 30.0
TS24 = 1 << 24                  # a data sample's stamp: the RTC's low 24 bits


def companion_path(data_path: Path, root: Path) -> Path:
    """events/X/NNN for data/X/NNN: the same path under the events root (§4.1)."""
    rel = Path(data_path).relative_to(root)
    return Path(root).joinpath("events", *rel.parts[1:])


@dataclass
class FileCheck:
    problems: list[str] = field(default_factory=list)
    note: str = ""


def check_file(path: Path, data_blob: bytes, *, want_enables: int, hw_id: int,
               data_first_ts: int, data_span_s: float) -> FileCheck:
    """An event file from a session nobody touched: the reference reader's
    checks (sequence, order, final record, paired with this data file), then
    that it records the inputs byte 232 enables, opened with the session, holds
    only state records, has a heartbeat at least every MAX_QUIET_S and closes
    with the data file.

    data_first_ts is the 24-bit stamp of the data file's first sample and
    data_span_s the time from it to the last. The data file's open is the
    header's initial timestamp, which the reader checks is the event file's."""
    fc = FileCheck()
    path = Path(path)
    if not path.is_file():
        fc.problems.append(f"no event file at {path.as_posix()}")
        return fc
    try:
        h, recs = evf.parse(path.read_bytes())
    except ValueError as e:
        fc.problems.append(str(e))
        return fc
    p = fc.problems
    p += evf.check(h, recs, data_blob)
    if h["hw_id"] != hw_id:
        p.append(f"header hardware {h['hw_id']}, expected {hw_id}")
    if h["enables"] != want_enables:
        p.append(f"header enables {h['enables']:#04x}, byte 232 enables {want_enables:#04x}")
    if h["open_reason"] != OPEN_SESSION_START:
        p.append(f"opened for reason {h['open_reason']}, expected the session's start")
    if h["first_seq"] != 0:
        p.append(f"first sequence number {h['first_seq']}, expected 0")
    if not recs:
        return fc
    first = recs[0]
    if first["type"] != TYPE_INIT:
        p.append(f"first record is type {first['type']:#04x}, not the initial state")
    else:
        if first["ts"] != h["file_open_ts"]:
            p.append(f"initial state at {first['ts']}, the file opened at {h['file_open_ts']}")
        if first["state"] & STATE_BUTTON:
            p.append("initial state has the button pressed")
        if h["enables"] & ENABLE_TTL and first["state"] & STATE_TTL:
            p.append("initial state has the TTL input high with nothing connected")
    edges = sum(r["type"] == TYPE_EDGE for r in recs)
    overflows = sum(r["type"] == TYPE_OVERFLOW for r in recs)
    if edges or overflows:
        p.append(f"{edges} edge and {overflows} overflow records with nothing connected")
    if any(r["dropped"] for r in recs):
        p.append(f"dropped count reaches {max(r['dropped'] for r in recs)}")
    gaps = [(b["ts"] - a["ts"]) / TICKS_PER_S for a, b in zip(recs, recs[1:])]
    if gaps and max(gaps) > MAX_QUIET_S:
        p.append(f"{max(gaps):.1f} s between records: a heartbeat is due every 60 s")
    beats = sum(r["type"] == TYPE_HEARTBEAT and not r["flags"] & evf.FLAG_FINAL for r in recs)
    span = (recs[-1]["ts"] - first["ts"]) / TICKS_PER_S
    if span >= HEARTBEAT_BY_S and not beats:
        p.append(f"no heartbeat in {span:.0f} s")
    lead = ((h["file_open_ts"] - data_first_ts) % TS24) / TICKS_PER_S
    if lead > DATA_LEAD_MAX_S:
        p.append(f"the data file's first sample is {lead:.1f} s before its open, or after it")
    elif abs(span - (data_span_s - lead)) > SPAN_SLACK_S:
        p.append(f"spans {span:.1f} s, the data file {data_span_s - lead:.1f} s from its open")
    fc.note = f"{len(recs)} records, {beats} heartbeats, {span:.1f} s"
    if lead >= 1.0:
        fc.note += f"; the data file opened {lead:.1f} s after its first sample"
    return fc


# --- the Bluetooth stream (BT_COMMUNICATION_PROTOCOL §7.15) -------------------

SET_FEATURE, FEATURE_EVENT_STREAM = 0xB7, 0x03
GET_EVENT_CHANNEL_STATUS, EVENT_CHANNEL_STATUS_RSP, STATUS_LEN = 0xB9, 0xBA, 9
FRAME_HEAD = bytes([0x8A, 0xBB])
RECORD_LEN = 16
FRAME_LEN = 3 + RECORD_LEN + 2

STATUS_STREAM_ON, STATUS_FILE_OPEN, STATUS_FILE_ERROR = 0x01, 0x02, 0x04
STATUS_TTL_ARMED, STATUS_TTL_THROTTLED = 0x08, 0x10


@dataclass
class Status:
    """GET_EVENT_CHANNEL_STATUS_COMMAND's reply (ShimEvt_buildStatus)."""
    enables: int
    flags: int
    state: int
    next_seq: int
    dropped: int
    frames_lost: int

    @classmethod
    def parse(cls, b: bytes) -> "Status":
        if len(b) != STATUS_LEN:
            raise ValueError(f"status is {len(b)} bytes, expected {STATUS_LEN}")
        return cls(b[0], b[1], b[2], b[3] | (b[4] << 8), b[5] | (b[6] << 8), b[7] | (b[8] << 8))

    def __str__(self):
        return (f"enables {self.enables:#04x} flags {self.flags:#04x} state {self.state:#04x} "
                f"next seq {self.next_seq} dropped {self.dropped} frames lost {self.frames_lost}")


def scan(buf: bytes) -> tuple[list[dict], int, bytes]:
    """(records, frames whose CRC failed, every other byte) in what the link
    carried. A frame is 0x8A 0xBB, length 16, the record and a CRC-16 over the
    rest; a candidate whose length or CRC is wrong is passed over a byte at a
    time, since other traffic can hold 0x8A 0xBB by chance."""
    recs, bad, other = [], 0, bytearray()
    i = 0
    while i < len(buf):
        if buf[i:i + 2] == FRAME_HEAD and i + FRAME_LEN <= len(buf) and buf[i + 2] == RECORD_LEN:
            frame = buf[i:i + FRAME_LEN]
            if shimmer_crc.calc_crc(FRAME_LEN - 2, frame) == frame[-2] | (frame[-1] << 8):
                recs.append(evf.decode_record(bytes(frame), 3))
                i += FRAME_LEN
                continue
            bad += 1
        other.append(buf[i])
        i += 1
    return recs, bad, bytes(other)


def set_stream(link, on: bool) -> None:
    """SET_FEATURE FEATURE_EVENT_STREAM: BtError on a NACK, firmware without it."""
    link.command(SET_FEATURE, bytes([FEATURE_EVENT_STREAM, 1 if on else 0]))


def status(link) -> Status:
    link.command(GET_EVENT_CHANNEL_STATUS, rsp=EVENT_CHANNEL_STATUS_RSP)
    return Status.parse(link._read(STATUS_LEN))


def listen(link, seconds: float) -> tuple[list[dict], int, bytes]:
    """Whatever arrives on an idle link for `seconds`, scanned for frames."""
    buf = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        buf += link.ser.read(max(1, link.ser.in_waiting))
    return scan(bytes(buf))


def write_bytes(link, values: dict[int, int]) -> None:
    """One-byte SET_INFOMEM writes, the way §8.3 tells a host to set byte 232."""
    for offset, v in values.items():
        link.write_infomem(bytes([v]), offset)


def read_bytes(link) -> tuple[int, int, int]:
    """(byte 217, byte 218, byte 232) over Bluetooth."""
    b = link.read_infomem(UBE_BYTE, BYTE - UBE_BYTE + 1)
    return b[0], b[SINGLE_TOUCH_BYTE - UBE_BYTE], b[-1]
