"""Bluetooth throughput: what the link carries, and how much of a heavy stream arrives.

Two measurements, made on one connection in this order (TEST_PROCEDURE §5.5):

- **Link rate.** The firmware's own SET_DATA_RATE_TEST (0xA4) floods the link
  with 5-byte [0xA5][u32 counter] packets (BT_COMMUNICATION_PROTOCOL §7.12).
  Bytes per second through the radio, with no sampling involved. The firmware
  queues a packet only when there is room for it, so a gap in the counter is a
  packet lost, never one skipped.
- **Sample delivery.** A stream at 1024 Hz with LN accel, gyro, mag, WR accel
  and VBatt, the DEV-1150 load, which outruns every shipped Shimmer3 radio.
  The figure is samples delivered per second of device time: the rate times
  the fraction of timestamp slots that arrived. It does not depend on how long
  the start and the stop took.

They catch different things. The data-rate test bypasses the sample path; the
stream goes through it, and on the RN42 v6.15 how much a saturated stream
delivers moves with the timing of that path. A sample-timer ISR change that was
functionally correct cost 9.8% of delivered samples (DEV-1222), and a
reply-path change once cost a fifth (DEV-1150).

On a Shimmer3R both also run over BLE (smoke.ble_throughput): BtLink takes
rig/ble.py's link, which reads and writes like the SPP port, so nothing here
knows which link it is on.

The link test runs first because on Shimmer3R classic throughput stays low for
about a minute after a stream stops, a bench effect seen with the data-rate
test itself.

Each unit is judged against figures for the same bench (rig.toml, see
Rig.throughput_baseline): its radio build's row in [throughput_by_radio], then
its own `throughput`, which wins key by key. A Shimmer3's figures are what the
last release gave on that unit; a Shimmer3R's link rate follows its IF820
module build, which a unit can be re-flashed to change. The same build repeats
within 0.3% when run interleaved, so TOLERANCE leaves room for bench drift and
still catches changes of the size DEV-1150 and DEV-1222 found. A row whose
figures move more from day to day sets its own `tolerance`.
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass
from pathlib import Path

from . import bt
from .config import THROUGHPUT_KEYS

SET_DATA_RATE_TEST, DATA_RATE_TEST_RESPONSE = 0xA4, 0xA5
PACKET = 5                                   # [0xA5][u32 counter LE]
SET_SAMPLING_RATE, SET_SENSORS = 0x05, 0x08
# Sensor bytes 0-2 (InfoMem 3-5): LN accel, gyro, mag | VBatt, WR accel. The
# same bits on both platforms (CONFIGURATION_INFOMEM §2).
LOAD_SENSORS = bytes.fromhex("e03000")
LOAD_DIVIDER = 32                            # 1024 Hz
# The channels that load streams, the same ids on both platforms
# (STREAMING_DATA_FORMAT §3): a figure from any other set is a different
# packet size, and not comparable with the baseline.
LOAD_CHANNELS = frozenset({0x00, 0x01, 0x02,     # LN accel
                           0x03,                 # VBatt
                           0x04, 0x05, 0x06,     # WR accel
                           0x07, 0x08, 0x09,     # mag
                           0x0A, 0x0B, 0x0C})    # gyro
TOLERANCE = 0.05
BACKUP_DIR = Path(__file__).resolve().parent.parent / "out" / "infomem"   # git-ignored


def save_infomem(unit_name: str, image: bytes) -> Path:
    """Keep a unit's InfoMem on disk before the load changes it.

    The load is stored configuration and the caller writes InfoMem back
    afterwards; if that fails, this file is the copy to write back by hand.
    """
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    p = BACKUP_DIR / f"{unit_name}_{time.strftime('%Y%m%d_%H%M%S')}.bin"
    p.write_bytes(image)
    return p


@dataclass
class DataRate:
    bytes: int = 0              # everything received between the ACK and the stop
    seconds: float = 0.0
    packets: int = 0
    gaps: int = 0               # counter jumps
    lost: int = 0               # packets those jumps skipped
    resyncs: int = 0            # bytes that did not frame as test packets
    stopped: bool = False       # the flood ended after the stop was sent
    stop_ack: bool = False      # and the last byte before the silence was an ACK
    last: int = -1              # the last counter framed; -1 before the first

    @property
    def bytes_per_s(self) -> float:
        return self.bytes / self.seconds if self.seconds else 0.0


def scan_data_rate(data: bytes, st: DataRate | None = None) -> DataRate:
    """Count test packets, counter gaps and stray bytes in what the flood delivered.

    Packets are contiguous, so after the first each must start where the last
    ended. A byte that is not 0xA5 there is stray: it is counted and skipped,
    and framing resumes at the next 0xA5 whose counter moves forward by less
    than 2^16 (a 0xA5 inside a counter would rarely do so). The counter starts
    at 0 when the test starts, so a first counter above 0 is packets lost. It
    is a uint32, so after the first packet steps are taken modulo 2^32 and a
    wrap is a step of one. st.last carries the position, so a scan can resume.
    """
    st = st or DataRate()
    i, n = 0, len(data)
    while i + PACKET <= n:
        if data[i] != DATA_RATE_TEST_RESPONSE:
            st.resyncs += 1
            i += 1
            continue
        c = int.from_bytes(data[i + 1:i + PACKET], "little")
        step = c + 1 if st.last < 0 else (c - st.last) % (1 << 32)
        if not 1 <= step < 1 << 16:
            st.resyncs += 1
            i += 1
            continue
        if step > 1:
            st.gaps += 1
            st.lost += step - 1
        st.packets += 1
        st.last = c
        i += PACKET
    return st


def data_rate_test(link: bt.BtLink, seconds: float) -> DataRate:
    """Run SET_DATA_RATE_TEST for `seconds` and stop it.

    The start takes effect after its ACK is queued, so the ACK is read first;
    the stop flushes the transmit ring before its ACK, so after the stop the
    link must go quiet, ending on that ACK, within a few seconds.
    """
    link.command(SET_DATA_RATE_TEST, b"\x01")
    buf = bytearray()
    old = link.ser.timeout
    link.ser.timeout = 0.2
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            buf += link.ser.read(max(1, link.ser.in_waiting))
    finally:
        st = DataRate(bytes=len(buf), seconds=time.monotonic() - t0)
        link.ser.write(bytes([SET_DATA_RATE_TEST, 0]))
        tail = b""
        end = time.monotonic() + 5.0
        link.ser.timeout = 0.3
        try:
            while time.monotonic() < end:
                chunk = link.ser.read(4096)
                if not chunk:
                    st.stopped = True
                    break
                tail = (tail + chunk)[-8:]
        finally:
            link.ser.timeout = old
        st.stop_ack = st.stopped and tail.endswith(bytes([bt.ACK]))
    return scan_data_rate(bytes(buf), st)


def load_stream(link: bt.BtLink, seconds: float) -> tuple[bt.StreamStats, bt.Inquiry]:
    """Stream the DEV-1150 load for `seconds`. The sensors and rate are stored
    configuration: the caller restores InfoMem afterwards."""
    link.command(SET_SENSORS, LOAD_SENSORS)
    link.command(SET_SAMPLING_RATE, LOAD_DIVIDER.to_bytes(2, "little"))
    inq = link.inquiry()
    if inq.divider != LOAD_DIVIDER:
        raise bt.BtError(f"inquiry reports divider {inq.divider} after setting {LOAD_DIVIDER}")
    if set(inq.channels) != LOAD_CHANNELS or len(inq.channels) != len(LOAD_CHANNELS):
        raise bt.BtError(f"inquiry reports channels {[f'{c:#04x}' for c in inq.channels]} after setting the "
                         f"load's sensors; expected {sorted(f'{c:#04x}' for c in LOAD_CHANNELS)}")
    st = link.stream(seconds, inq)
    # A stop the firmware did not act on leaves the unit sensing; name that
    # here rather than as a dock timeout in the caller's restore.
    link.wait_status(bt.STATUS_SENSING, False, timeout_s=5)
    return st, inq


def samples_per_s(st: bt.StreamStats, inq: bt.Inquiry) -> float:
    """Samples delivered per second of device time: rate x slots that arrived."""
    slots = st.packets + st.missing
    return inq.rate_hz * st.packets / slots if slots else 0.0


def figures(rate: DataRate, st: bt.StreamStats, inq: bt.Inquiry) -> dict[str, float]:
    return {"link_bytes_per_s": round(rate.bytes_per_s), "stream_samples_per_s": round(samples_per_s(st, inq), 1)}


STREAM_COVERAGE = 0.9


def defects(rate: DataRate, st: bt.StreamStats, inq: bt.Inquiry) -> list[str]:
    """What fails whatever the baseline: lost test packets, misframing, a stop
    not obeyed, a stream whose stamps do not span it.

    Samples lost between packets are not here: the load outruns the link by
    design, and how much arrives is the figure judged against the baseline.
    But a stream that stopped delivering, or delivered one packet, shows no
    gap at all, and samples_per_s would read it as everything delivered. So
    its stamps must cover STREAM_COVERAGE of the stream time; the bench's
    runs cover 99% or more (19.87 of 20.0 s on the RN42 v6.15 at its worst).
    """
    out = []
    if rate.packets == 0:
        out.append("the data-rate test delivered no packets")
    if rate.gaps:
        out.append(f"data-rate test: {rate.lost} packets lost in {rate.gaps} counter gaps")
    if rate.resyncs:
        out.append(f"data-rate test: {rate.resyncs} bytes did not frame as test packets")
    if not rate.stopped:
        out.append("data-rate test: the link was still busy 5 s after the stop")
    elif not rate.stop_ack:
        out.append("data-rate test: the stop was not ACKed")
    covered_s = (st.packets + st.missing) * inq.divider / bt.TICKS_PER_S
    if st.packets == 0:
        out.append("the stream delivered no packets")
    elif st.packets < 2 or covered_s < STREAM_COVERAGE * st.seconds:
        out.append(f"stream: {st.packets} packets whose stamps cover {covered_s:.2f} s of a {st.seconds:.1f} s "
                   "stream - it started late, or its end is missing")
    if st.resyncs:
        # The bytes say what went wrong: a packet a byte short is a byte lost
        # on the way out (TEST_PROCEDURE §5.5), anything else is not.
        first = f" (first from {st.misframes[0]})" if st.misframes else ""
        out.append(f"stream: {st.resyncs} bytes did not frame as packets{first}")
    if st.dupes:
        out.append(f"stream: {st.dupes} duplicate or backwards stamps")
    return out


def as_toml(measured: dict[str, float], key: str = "throughput") -> str:
    """The figures as a rig.toml line, to record them as a unit's baseline
    (`throughput`, or `ble_throughput` for the BLE figures)."""
    return f"{key} = {{ " + ", ".join(f"{k} = {measured[k]:g}" for k in THROUGHPUT_KEYS) + " }"


def compare(measured: dict[str, float], baseline: dict[str, float]) -> tuple[list[str], list[str]]:
    """(shortfalls, gains) against the baseline: figures more than its tolerance
    (TOLERANCE unless the baseline sets one) below it, or above it."""
    tolerance = baseline.get("tolerance", TOLERANCE)
    short, gain = [], []
    for k in THROUGHPUT_KEYS:
        if k not in baseline:
            continue
        ratio = measured[k] / baseline[k]
        if ratio < 1 - tolerance:
            short.append(f"{k} {measured[k]:g} is {1 - ratio:.1%} below the baseline {baseline[k]:g}")
        elif ratio > 1 + tolerance:
            gain.append(f"{k} {measured[k]:g} is {ratio - 1:.1%} above the baseline {baseline[k]:g}")
    return short, gain


def radio_mismatch(named: str | None, reported: str) -> str | None:
    """Why the radio build rig.toml names cannot be the one the unit reports, or None.

    Only the version numbers can be compared: a module's test builds report
    their release's version, so which build of a version is fitted is for
    rig.toml to say, and a wrong one shows as a figure far off its baseline.
    """
    def version(text):
        # As numbers: the Shimmer3R dock reports "app=v01.04.18.18" for v1.4.18.18.
        m = re.search(r"\d+(?:\.\d+){1,3}", text or "")
        return tuple(int(x) for x in m.group().split(".")) if m else None

    want, got = version(named), version(reported)
    if want and got and want != got:
        return (f"rig.toml names radio {named!r} but the unit reports {reported!r}: "
                "update its radio, which picks its throughput baseline")
    return None


def judge(measured: dict[str, float], baseline: dict[str, float], unit_name: str) -> tuple[list[str], str]:
    """(failures, note) for a unit: no baseline is not a pass, it is not_covered (the caller skips)."""
    short, gain = compare(measured, baseline)
    note = ", ".join(f"{k} {measured[k]:g}" + (f" (baseline {baseline[k]:g})" if k in baseline else "")
                     for k in THROUGHPUT_KEYS)
    if gain:
        note += "; " + "; ".join(gain) + f" - if this build is released, record the new figures for {unit_name}"
    if short:
        short = [s + " - confirm with an interleaved A/B against the release before believing it "
                 "(TEST_PROCEDURE §5.5)" for s in short]
    return short, note
