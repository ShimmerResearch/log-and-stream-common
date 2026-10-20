"""SD logging started and stopped while a Bluetooth stream runs, and what that
does to the stream's timestamps.

A host that logs while it streams sends the log start mid-stream. The
reference host sets the real-world clock first, every time: SET_RWC_COMMAND,
then START_LOGGING_COMMAND. The protocol allows both while sensing
(SHIMMER3_BT_COMMUNICATION_PROTOCOL.md, SET_RWC_COMMAND). Neither may
disturb the stream. Its stamps must stay increasing, one period apart and on
the same sampling grid.

The worst disturbance is a stamp behind the one before it. A host that
unwraps the 24-bit counter as specified (SHIMMER3_STREAMING_DATA_FORMAT.md
§2.1) takes a step back beyond its reorder window for a wrap, so every later
sample comes out 512 s late.

schedule() builds the commands for BtLink.stream(actions=...). analyse()
judges the stamps, and by_cycle() says which log start or stop each problem
followed.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable

from . import bt

SET_RWC = 0x8F
TICKS_PER_S = bt.TICKS_PER_S

# A stamp more than this many ticks off the grid is off it; one tick is the
# quantisation every stream shows (classify_step uses the same line).
GRID_TOL = 1
# Stamps that must agree on a new offset before it counts as a moved grid
# rather than one stamp off it.
SETTLE = 3
# Packets that must arrive, as a fraction of the stream's length at its rate,
# before its stamps are judged at all (the release rows use the same floor).
MIN_PACKET_FRACTION = 0.8


def rwc_payload(now_s: float | None = None) -> bytes:
    """SET_RWC_COMMAND for the host's clock now: 64-bit little-endian 32768 Hz
    ticks of the Unix epoch, as the reference host sends it."""
    ticks = int((time.time() if now_s is None else now_s) * TICKS_PER_S)
    return bytes([SET_RWC]) + ticks.to_bytes(8, "little")


@dataclass
class Action:
    """One command sent mid-stream, t seconds after the stream started."""
    t: float
    kind: str                       # set_rwc, start, status or stop
    cycle: int                      # 1-based log cycle it belongs to
    payload: bytes | Callable[[], bytes]

    @property
    def label(self) -> str:
        return f"{self.kind} {self.cycle}"

    def bytes(self) -> bytes:
        return self.payload() if callable(self.payload) else self.payload

    @property
    def acked(self) -> bool:
        """Answered with a bare ACK. GET_STATUS's answer is an in-stream
        response, which the stream counts as a status frame instead."""
        return self.kind != "status"


@dataclass
class Plan:
    actions: list[Action]
    seconds: float                  # how long to stream
    rwc_cycles: list[int]           # cycles whose start sets the clock first


def schedule(cycles: int, lead_s: float = 3.0, on_s: float = 2.0, off_s: float = 2.0,
             rwc_gap_s: float = 0.2, status_after_s: float = 1.0, tail_s: float = 2.0) -> Plan:
    """Log cycles inside one stream: odd cycles start logging plainly, even
    cycles set the clock first, as the reference host does. Each start is
    followed by a status query, which shows whether logging really began."""
    actions = []
    t = lead_s
    rwc = []
    for c in range(1, cycles + 1):
        start = t
        if c % 2 == 0:
            actions.append(Action(t, "set_rwc", c, rwc_payload))
            rwc.append(c)
            start = t + rwc_gap_s
        actions.append(Action(start, "start", c, bytes([bt.START_LOGGING])))
        actions.append(Action(start + status_after_s, "status", c, bytes([bt.GET_STATUS])))
        actions.append(Action(start + on_s, "stop", c, bytes([bt.STOP_LOGGING])))
        t = start + on_s + off_s
    return Plan(actions, t - off_s + tail_s, rwc)


@dataclass
class Grid:
    packets: int = 0
    stats: bt.StreamStats = field(default_factory=bt.StreamStats)   # judged on the moved grid
    backward: list = field(default_factory=list)    # (index, step): stamps at or behind the one before
    shifts: list = field(default_factory=list)      # (index, ticks): where the grid moved, and by how much
    issues: list = field(default_factory=list)      # (index, text), in stream order

    @property
    def clean(self) -> bool:
        return not self.issues


def _offset(ticks: int, period: int) -> int:
    """Where a stamp sits against its slot, as classify_step places it: from a
    quarter period early to three quarters late."""
    return (ticks + period // 4) % period - period // 4


def analyse(ts: list[int], period: int, max_jitter: int | None = None) -> Grid:
    """Judge a stream's stamps: steps that go backwards, moves of the sampling
    grid, then lost samples, duplicates and jitter against the moved grid.

    A grid move is a new offset that the next SETTLE stamps keep; one stamp off
    the grid and then back is jitter. Lost samples and duplicates are counted by
    classify_step on the moved grid, so a single move is not also reported as
    jitter on every stamp after it. Jitter is an issue only past max_jitter
    ticks (default a quarter period, the smoke rule).
    """
    g = Grid(packets=len(ts))
    if len(ts) < 2:
        return g
    max_jitter = period // 4 if max_jitter is None else max_jitter
    cum = [0]
    for a, b in zip(ts, ts[1:]):
        d = bt.ts_step(b, a)
        if d <= 0:
            g.backward.append((len(cum), d))
        cum.append(cum[-1] + d)

    anchor = [0] * len(cum)
    a = 0
    for i in range(1, len(cum)):
        o = _offset(cum[i] - a, period)
        if abs(o) > GRID_TOL:
            nxt = cum[i + 1:i + 1 + SETTLE]
            if nxt and all(abs(_offset(c - a - o, period)) <= GRID_TOL for c in nxt):
                g.shifts.append((i, o))
                a += o
        anchor[i] = a

    back = dict(g.backward)
    shift = dict(g.shifts)
    st = g.stats
    for i in range(1, len(cum)):
        d = (cum[i] - anchor[i]) - (cum[i - 1] - anchor[i - 1])
        before = (st.gaps, st.missing, st.dupes, st.jitter, st.max_jitter_ticks)
        bt.classify_step(st, d, period)
        texts = []
        if i in back:
            texts.append(f"stamp {-back[i]} ticks behind the one before")
        if i in shift:
            texts.append(f"grid moved {shift[i]:+d} ticks")
        if st.missing > before[1]:
            texts.append(f"{st.missing - before[1]} samples lost (step {cum[i] - cum[i - 1]} ticks)")
        if st.dupes > before[2] and i not in back:
            texts.append(f"stamp in a slot already taken (step {cum[i] - cum[i - 1]} ticks)")
        if st.jitter > before[3]:
            off = abs(_offset(cum[i] - anchor[i], period))
            if off > max_jitter:
                texts.append(f"stamp {off} ticks off its slot")
        if texts:
            g.issues.append((i, ", ".join(texts)))
    return g


def by_cycle(grid: Grid, events: list) -> dict[str, list[str]]:
    """Group the grid's issues by the command they followed.

    events are (label, packet index, seconds) as BtLink.stream records them:
    the packet index is how many packets had arrived when the command went out,
    so packets from that index on came after it. An issue is put under the
    cycle's "start" (its clock set, log start or status query) or its "stop";
    one before any command is under "stream".
    """
    out: dict[str, list[str]] = {}
    marks = sorted(((idx, label) for label, idx, _ in events), key=lambda m: m[0])
    for i, text in grid.issues:
        key = "stream"
        for idx, label in marks:
            if idx > i:
                break
            kind, cycle = label.rsplit(" ", 1)
            key = f"cycle {cycle} {'stop' if kind == 'stop' else 'start'}"
        out.setdefault(key, []).append(text)
    return out


def logging_seen(st) -> int:
    """In-stream status frames that showed logging: the replies to the status
    queries, and any status the firmware pushed on a change of state."""
    return sum(1 for s in st.pushed if s & bt.STATUS_LOGGING)


def judge(st, plan: Plan, period: int) -> tuple[list[str], str]:
    """The stream's problems, each naming the cycle it followed, and a one-line
    summary that says whether clock sets and plain starts differ."""
    g = analyse(st.ts, period)
    groups = by_cycle(g, st.events)
    problems = []
    expected = plan.seconds * TICKS_PER_S / period
    if g.packets < MIN_PACKET_FRACTION * expected:
        # Too few stamps to judge: an empty or dead stream must not pass as clean.
        problems.append(f"{g.packets} packets in a {plan.seconds:.0f} s stream, expected ~{expected:.0f}")
    if st.resyncs:
        problems.append(f"{st.resyncs} bytes did not frame as packets (first: {st.misframes[:1]})")
    if st.crc_errors:
        problems.append(f"{st.crc_errors} link CRC errors")
    if len(st.events) < len(plan.actions):
        problems.append(f"only {len(st.events)} of {len(plan.actions)} commands went out before the stop")
    want = sum(a.acked for a in plan.actions)
    if st.acks != want:
        problems.append(f"{st.acks} ACKs for the {want} commands sent mid-stream")
    for key, texts in groups.items():
        more = f" and {len(texts) - 3} more" if len(texts) > 3 else ""
        problems.append(f"{key}: {'; '.join(texts[:3])}{more}")
    cycles = sorted({a.cycle for a in plan.actions})
    rwc = [c for c in cycles if c in plan.rwc_cycles]
    plain = [c for c in cycles if c not in plan.rwc_cycles]

    def hit(cs):
        return sum(1 for c in cs if f"cycle {c} start" in groups)
    summary = (f"{g.packets} packets, {len(cycles)} log starts; the stream was disturbed by "
               f"{hit(rwc)} of {len(rwc)} starts after a clock set and {hit(plain)} of {len(plain)} without")
    return problems, summary
