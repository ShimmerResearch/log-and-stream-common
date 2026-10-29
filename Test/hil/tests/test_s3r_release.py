"""Shimmer3R release tier: the configuration matrix in matrix/s3r_release.toml.

Each row is one report item, release.<id>. For every row:

1. Configure over USB: rate, sensor set and ranges written as raw InfoMem.
   Read back, the image must hold exactly that - and every enabled sensor's
   ODR must cover the packet rate, because the firmware is meant to raise it
   (INFOMEM §10.6). A host that wrote only the bitmap and rate, as many do,
   depends on that correction.
2. Run the row's mode: Bluetooth stream (docked), SD log (undocked by the hub
   port, started and stopped over Bluetooth), or both at once.
3. Check timing (lost / duplicate samples, jitter, device-clock rate), framing
   and link CRCs, that inquiry and SD header agree with the stored
   configuration, and the signals of a unit lying still (rig/signals.py).
   With both, every streamed packet must be in the log, byte for byte.

Two observations are reported as their own items rather than failing every
row they touch (see _Aside): loss in the first second of an SD log
(release.sd_log_start), and a stream stop that goes unACKed or leaves the
link silent (release.stop_ack).

The unit's own configuration is restored when the module finishes.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import pytest

from rig import bt, decode, dock, s3r_boards, s3r_usb, sdfile, signals
from rig.s3r_config import S3rConfig, divider_for

try:
    import tomllib
except ModuleNotFoundError:     # pragma: no cover
    import tomli as tomllib

pytestmark = pytest.mark.release

# HIL_MATRIX points at another matrix file - an experiment's rows, say - unless
# the file says it is for Shimmer3 (platform = "shimmer3"), which is
# test_s3_release's. A run bound to a release candidate must use the default
# (run_rc.py does).
MATRIX = Path(__file__).resolve().parents[1] / "matrix" / "s3r_release.toml"
if os.environ.get("HIL_MATRIX"):
    with open(os.environ["HIL_MATRIX"], "rb") as f:
        if tomllib.load(f).get("platform") != "shimmer3":
            MATRIX = Path(os.environ["HIL_MATRIX"])
with open(MATRIX, "rb") as f:
    _M = tomllib.load(f)
SETS = {k: set(v) for k, v in _M["sets"].items()}
ROWS = _M["row"]

RATE_TOLERANCE = 1e-3
MAX_JITTER_FRACTION = 0.25
LOG_START_S = 1.0               # the window release.sd_log_start owns


@pytest.fixture(scope="module")
def original_config(unit):
    """The unit's configuration before the matrix, put back afterwards."""
    port = s3r_usb.wait_cdc(unit.usb_serial, 30)
    with dock.open_when_ready(port) as d:
        image = d.read_infomem()
    yield image
    port = s3r_usb.wait_cdc(unit.usb_serial, 60)
    with dock.open_when_ready(port) as d:
        d.write_infomem(image)


def _configure(unit, row) -> tuple[S3rConfig, set[str]]:
    """Write the row's configuration; return the read-back and the sensors left out as unfitted."""
    port = s3r_usb.wait_cdc(unit.usb_serial, 60)
    with dock.open_when_ready(port) as d:
        card = d.get(dock.COMP_DAUGHTER_CARD, 0x02, bytes([3, 0]))[:3]
        unfitted = s3r_boards.OPTIONAL - s3r_boards.fitted_optional(card)
        want = SETS[row["sensors"]] - unfitted
        cfg = S3rConfig(d.read_infomem())
        cfg.divider = divider_for(row["rate_hz"])
        cfg.set_sensors(want)
        cfg.ln_accel_range = row["ln_accel_range"]
        cfg.gyro_range = row["gyro_range"]
        cfg.wr_accel_range = row["wr_accel_range"]
        d.write_infomem(cfg.bytes())
        back = S3rConfig(d.read_infomem())
    problems = []
    if back.divider != cfg.divider:
        problems.append(f"divider read back {back.divider}, wrote {cfg.divider}")
    if back.enabled() != want:
        problems.append(f"sensors read back {sorted(back.enabled())}, wrote {sorted(want)}")
    for f in ("ln_accel_range", "gyro_range", "wr_accel_range"):
        if getattr(back, f) != row[f]:
            problems.append(f"{f} read back {getattr(back, f)}, wrote {row[f]}")
    problems += [f"ODR not corrected: {p}" for p in back.odr_shortfalls()]
    assert not problems, "configuration: " + "; ".join(problems)
    return back, SETS[row["sensors"]] & unfitted


def _timing(st, period, skip_first_s: float = 0.0) -> list[str]:
    """Timing problems; with skip_first_s, steps inside the opening window are not judged here."""
    out = []
    if skip_first_s and st.ts:
        # Re-judge from the kept timestamps, ignoring the opening window.
        skip = int(skip_first_s * 32768 / period)
        fresh = type(st)(header=st.header) if hasattr(st, "header") else bt.StreamStats()
        for a, b in zip(st.ts[skip:], st.ts[skip + 1:]):
            bt.classify_step(fresh, (b - a) % bt.TS_WRAP, period)
        st = fresh
    if st.gaps:
        out.append(f"{st.gaps} gaps, {st.missing} samples lost, largest step {st.max_gap_ticks} ticks")
    if st.dupes:
        out.append(f"{st.dupes} duplicate or backwards stamps")
    if st.max_jitter_ticks > MAX_JITTER_FRACTION * period:
        out.append(f"stamp jitter up to {st.max_jitter_ticks} ticks (period {period})")
    return out


def _start_loss(ts, period) -> int:
    """Samples lost within the first LOG_START_S of a log."""
    n = int(LOG_START_S * 32768 / period) + 1
    lost = 0
    for a, b in zip(ts[:n], ts[1:n + 1]):
        k = round(((b - a) % bt.TS_WRAP) / period)
        lost += max(0, k - 1)
    return lost


def _new_session(root: Path, before: set) -> Path:
    new = sorted(sdfile.session_dirs(root) - before)      # relative: the drive letter may have changed
    assert len(new) == 1, f"expected one new SD session, found {len(new)}: {new}"
    files = sorted(p for p in (root / new[0]).iterdir() if p.is_file())
    assert files and files[0].name == "000", f"{new[0]}: no data file 000"
    assert len(files) == 1, f"{new[0]}: {len(files)} data files for a short log"
    return files[0]


def _check_log(path: Path, cfg: S3rConfig, inq, seconds, unit, aside):
    """Calibration comes from the file's own header, as an SD parser would take it."""
    fc = sdfile.check(path, keep=True)
    calibs = decode.calibs_from_sd_header(path.read_bytes()[:sdfile.HEAD_S3R])
    h = fc.header
    probs = []
    if h.mac != unit.mac:
        probs.append(f"header MAC {h.mac}")
    if h.divider != cfg.divider:
        probs.append(f"header divider {h.divider}, config {cfg.divider}")
    if inq and h.channels != inq.channels:
        probs.append(f"header channels {[hex(c) for c in h.channels]}, inquiry {[hex(c) for c in inq.channels]}")
    expected = seconds * cfg.rate_hz
    if fc.records < 0.8 * expected:
        probs.append(f"{fc.records} records for {seconds} s at {cfg.rate_hz:.1f} Hz")
    if fc.zero_ts:
        probs.append(f"{fc.zero_ts} records with a zero timestamp")
    if fc.trailing_bytes:
        probs.append(f"{fc.trailing_bytes} bytes after the last whole record")
    probs += _timing(fc, h.divider, skip_first_s=LOG_START_S)
    aside.log_start(_start_loss(fc.ts, h.divider))
    chans = decode.split_channels(fc.rows, h.channels, bt.HW_SHIMMER3R)
    sig, note = signals.check_rest(chans, calibs, cfg.rate_hz)
    return fc, probs + sig, note


class _Aside:
    """Records observations as their own report items, beside the row's.

    Two known kinds of failure would otherwise fail every row they touch and
    hide whatever else a row finds: sample loss in the first second of an SD
    log, and a stream stop that goes unACKed or leaves the link silent. Each
    row records them here as release.sd_log_start and release.stop_ack, which
    the report aggregates worst-first like any item.
    """

    def __init__(self, request, row_id):
        self.col = request.config._hil_collector
        self.row = row_id

    def log_start(self, lost: int):
        self.col.record("release.sd_log_start", "fail" if lost else "pass",
                        f"{self.row}: {lost} samples lost in the first {LOG_START_S:.0f} s" if lost else None)

    def stop(self, st, hung: bool):
        bad = []
        if not st.stop_ack:
            bad.append("no ACK")
        if hung:
            bad.append("link silent afterwards, reconnected")
        self.col.record("release.stop_ack", "fail" if bad else "pass",
                        f"{self.row}: {', '.join(bad)} ({st.after_stop} packets after the stop)" if bad else None)


@pytest.mark.parametrize("row", ROWS, ids=[r["id"] for r in ROWS])
@pytest.mark.hil("release.{row}", needs=["shimmer3r", "usb", "bt"])
def test_row(row, unit, rig, power, original_config, request, hil_note):
    aside = _Aside(request, row["id"])
    if row["mode"] != "stream" and "hub_port" not in rig.capabilities(unit):
        pytest.skip("logging rows need the unit's USB on a switchable hub port")

    cfg, unfitted = _configure(unit, row)
    port = bt.spp_port(unit.mac)
    assert port, f"no paired SPP port for {unit.mac}"
    probs: list[str] = []
    notes: list[str] = []

    if row["mode"] == "stream":
        with bt.BtLink(port) as link:
            inq = link.inquiry()
            # Calibration as a streaming host gets it: for the current ranges.
            calibs = decode.calibs_from_blob(link.all_calibration())
            link.set_crc(row.get("crc", 0))
            st = link.stream(row["seconds"], inq, keep=True)
            hung = not _answers_after_stop(link)
            if not hung:
                link.set_crc(0)
        # A silent link is closed: disconnecting also resets the CRC mode (§3.3).
        aside.stop(st, hung)
        probs += _stream_checks(st, inq, cfg, row)
        chans = decode.split_channels(st.rows, inq.channels, bt.HW_SHIMMER3R)
        sig, note = signals.check_rest(chans, calibs, cfg.rate_hz)
        probs += sig
        notes.append(f"{st.packets} pkts, {note}")
    else:
        root = s3r_usb.wait_drive(unit.usb_serial, 60)
        before = sdfile.session_dirs(root)
        power.off(unit.hub_port, unit.name)
        try:
            s3r_usb.wait_cdc_gone(unit.usb_serial, 15)
            with bt.BtLink(port) as link:
                inq = link.inquiry()
                if row["mode"] == "log":
                    link.simple(bt.START_LOGGING)
                    link.wait_status(bt.STATUS_LOGGING, True, 10)
                    time.sleep(row["seconds"])
                    link.simple(bt.STOP_LOGGING)
                    link.wait_status(bt.STATUS_LOGGING, False, 15)
                    st = None
                else:
                    link.set_crc(row.get("crc", 0))
                    st = link.stream(row["seconds"], inq, keep=True,
                                     start_op=bt.START_SDBT, stop_op=bt.STOP_SDBT)
                    hung = not _answers_after_stop(link)
                    if not hung:
                        link.wait_status(bt.STATUS_SENSING, False, 15)
                        link.set_crc(0)
                    aside.stop(st, hung)
            if st is not None and hung:
                # The first link has gone silent and is now closed; confirm on a
                # fresh one that sensing really stopped before re-docking.
                with bt.BtLink(port) as again:
                    again.wait_status(bt.STATUS_SENSING, False, 15)
        finally:
            power.on(unit.hub_port, unit.name)
        root = s3r_usb.wait_drive(unit.usb_serial, 90)
        path = _new_session(root, before)
        fc, lp, note = _check_log(path, cfg, inq, row["seconds"], unit, aside)
        probs += [f"log: {p}" for p in lp]
        notes.append(f"{fc.records} records, {note}")
        if st is not None:
            # Same opening window as the log: start-of-log loss belongs to
            # release.sd_log_start, not to every row that also streams.
            probs += [f"stream: {p}" for p in _stream_checks(st, inq, cfg, row, skip_first_s=LOG_START_S)]
            logged = set(zip(fc.ts, fc.rows))
            missing = sum(1 for t, r in zip(st.ts, st.rows) if (t, r) not in logged)
            if missing:
                probs.append(f"{missing} of {st.packets} streamed packets are not in the SD log byte for byte")
            notes.append(f"{st.packets} streamed, all in the log" if not missing else "")
    if unfitted:
        notes.append(f"not fitted on this board, left out: {', '.join(sorted(unfitted))}")
    hil_note("; ".join(n for n in notes if n))
    assert not probs, "; ".join(probs)


def _answers_after_stop(link) -> bool:
    """True if the device answers a command after a stream stop.

    Seen on Shimmer3R v1.01.021: about one stop in twelve at 51.2 Hz goes
    unACKed, more packets than usual follow it, and the device then answers
    nothing more on that link - a reconnect recovers it. The row's own data is
    unaffected, so the hang is recorded for release.stop_ack and the row goes on.
    """
    try:
        link.status()                             # also flushes a stale packet
        return True
    except bt.BtError:
        return False


def _stream_checks(st, inq, cfg: S3rConfig, row, skip_first_s: float = 0.0) -> list[str]:
    out = []
    if inq.divider != cfg.divider:
        out.append(f"inquiry divider {inq.divider}, config {cfg.divider}")
    if st.resyncs:
        out.append(f"{st.resyncs} framing resyncs (first: {st.misframes[:1]})")
    if st.crc_errors:
        out.append(f"{st.crc_errors} CRC errors (mode {row.get('crc', 0)})")
    expected = row["seconds"] * cfg.rate_hz
    if st.packets < 0.8 * expected:
        out.append(f"{st.packets} packets in {st.seconds:.1f} s, expected ~{expected:.0f}")
    out += _timing(st, inq.divider, skip_first_s=skip_first_s)
    span = sum(d * n for d, n in st.deltas.items())
    if span and st.packets > 1:
        rate = (st.packets - 1) * 32768 / span
        if abs(rate / cfg.rate_hz - 1) > RATE_TOLERANCE and not st.gaps:
            out.append(f"device-clock rate {rate:.3f} Hz, configured {cfg.rate_hz:.3f} Hz")
    return out

