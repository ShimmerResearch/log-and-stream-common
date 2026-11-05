"""Shimmer3 release tier: the configuration matrix in matrix/s3_release.toml.

Each row is one report item, release.<id>, run on every Shimmer3 in the Base6:
one unit per radio firmware that shipped (TEST_PROCEDURE §5.1). For every row
and unit:

1. Configure over Bluetooth, the way a customer's host does: the whole InfoMem
   (configure = "infomem", as Consensys writes it) or the individual commands
   (configure = "commands", as older hosts and pyshimmer do). Rate, sensors,
   ranges, and the ODRs a host derives for the rate: Shimmer3 firmware does
   not raise them itself (rig/s3_config.py).
2. Read InfoMem back over the Base6 dock and compare it with what was meant,
   byte for byte. That is its own item, release.bt_config: a row whose
   configuration did not take is not streamed, and is reported not_covered.
3. Stream over Bluetooth, docked, and check timing (lost / duplicate samples,
   jitter, device-clock rate), framing and link CRCs, that the inquiry agrees
   with the stored configuration, and the signals of a unit lying still
   (rig/signals.py).

A stream stop that goes unACKed or leaves the link silent is recorded as
release.stop_ack, as on Shimmer3R, rather than failing the row.

No row logs. A docked Shimmer3 gives its SD card to the dock and will not
start a log, and the Base6 cannot undock one slot, so logging stays a manual
sign-off item.

The dock only verifies and restores. Before the first row, each unit's InfoMem
(over the dock) and calibration dump (over Bluetooth) are saved to disk. After
the last, the dump is written back if it changed, because every InfoMem write
at offset 0 re-stamps the dump's records for the ranges it set. Then InfoMem is
written back over the dock.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from pathlib import Path

import pytest

from rig import bt, decode, dock, signals, throughput
from rig.s3_config import (MAG_FLOOR_HZ, S3Config, calib_records, divider_for, expected_channels, odr_codes,
                           packet_bytes)
from rig.s3_flash import _masked, reset_slot, restore_infomem

try:
    import tomllib
except ModuleNotFoundError:     # pragma: no cover
    import tomli as tomllib

pytestmark = pytest.mark.release

PLATFORM = "shimmer3"
DEFAULT_MATRIX = Path(__file__).resolve().parents[1] / "matrix" / "s3_release.toml"


def _matrix_path() -> Path:
    """HIL_MATRIX names an experiment's rows. This module takes them only from a
    file that says it is for Shimmer3 (platform = "shimmer3"), and the Shimmer3R
    module only from one that does not. A run bound to a release candidate uses
    the default (run_rc.py clears HIL_MATRIX)."""
    env = os.environ.get("HIL_MATRIX")
    if env:
        with open(env, "rb") as f:
            if tomllib.load(f).get("platform") == PLATFORM:
                return Path(env)
    return DEFAULT_MATRIX


with open(_matrix_path(), "rb") as f:
    _M = tomllib.load(f)
SETS = {k: set(v) for k, v in _M["sets"].items()}
ROWS = _M["row"]

RANGES = ("gyro_range", "wr_accel_range", "alt_accel_range")
RATE_TOLERANCE = 1e-3
MAX_JITTER_FRACTION = 0.25
# A row is judged for loss on a unit only if it needs at most this share of
# what the unit's radio delivered of a saturated stream (rig.toml's
# stream_samples_per_s, in 30-byte load packets).
LINK_FIT = 0.75
LOAD_PACKET = 4 + sum(bt.channel_width(c, bt.HW_SHIMMER3) for c in throughput.LOAD_CHANNELS)
RESET_SETTLE_S = 4.0            # radio bring-up after a slot reset, as rig/btmode.py waits


@dataclass
class Saved:
    infomem: bytes              # 512 bytes, over the dock
    dump: bytes                 # the calibration dump, over Bluetooth
    records: dict               # its records: which blocks are a calibration and which a default
    backup: Path


def _open_bt(unit, base6=None, attempts: int = 2, wait_s: float = 10.0) -> bt.BtLink:
    """Connect, retrying as fleet's open_bt does: on this bench a classic link
    right after another unit's stream can be slow to page, and Windows can hold
    a COM port for a moment after pairing activity. Neither is the unit's.

    Given the Base6, a unit that takes the link but answers nothing is reset
    through its slot before the failure is raised: silent over classic BT while
    the dock works (known_issues DEV-1136), which a reset clears. Seen 29 Oct
    2026 on RN4678 V1.23 after ten rows. Without the reset every later row on
    the unit fails the same way, and its calibration dump cannot be put back."""
    port = bt.spp_port(unit.mac)
    if not port:
        pytest.skip(f"no paired SPP port for {unit.mac} - pair its classic side once in Windows")
    last, held = None, False
    for _ in range(attempts + 1):
        try:
            return bt.BtLink(port, connect_timeout_s=20)
        except Exception as e:      # serial.SerialException or BtError
            last = e
            held = "Access is denied" in str(e) or isinstance(getattr(e, "__context__", None), PermissionError)
            time.sleep(wait_s * (2 if held else 1))
    if held:
        pytest.skip(f"{port} is held open by another process ({last}) - not tested")
    if base6 is not None and isinstance(last, bt.BtError):
        reset_slot(base6, unit.base6_slot)
        time.sleep(RESET_SETTLE_S)
        pytest.fail(f"{port}: the unit took the link but answered nothing after {attempts + 1} attempts ({last}); "
                    f"slot {unit.base6_slot} reset for what follows (DEV-1136)")
    pytest.fail(f"{port}: no Bluetooth connection after {attempts + 1} attempts ({last})")


@pytest.fixture(scope="module")
def saved(unit, base6):
    """The unit's InfoMem and calibration dump before the matrix, put back afterwards."""
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        infomem = bytes(d.read_infomem())
    backup = throughput.save_infomem(unit.name, infomem)
    with _open_bt(unit) as link:
        dump = link.read_calib_dump()
    dump_backup = backup.with_name(backup.stem + "_calib_dump.bin")
    dump_backup.write_bytes(dump)
    yield Saved(infomem, dump, calib_records(dump), backup)

    # The dump first: writing it pushes its blocks into InfoMem
    # (CALIBRATION §8.3), which the dock write after it then puts back.
    # Twice: a unit silent over Bluetooth is reset by the first try (_open_bt).
    for _ in range(2):
        problem = None
        try:
            with _open_bt(unit, base6) as link:
                if link.read_calib_dump() != dump:
                    link.write_calib_dump(dump)
                    if link.read_calib_dump() != dump:
                        problem = "read back differs"
            break
        except (Exception, pytest.skip.Exception, pytest.fail.Exception) as e:
            problem = repr(e)                     # InfoMem still goes back
    base6.select(unit.base6_slot)
    restore_infomem(base6, infomem, None, backup)
    if problem:
        pytest.fail(f"{unit.name}: calibration dump not restored ({problem}); the original is in {dump_backup}. "
                    "A reset while docked also rebuilds it from the SD card's copy and InfoMem.")


class _ConfigFailed(Exception):
    pass


def _intended(base: S3Config, row) -> S3Config:
    """The base image with the row's rate, sensors, ranges and derived ODRs."""
    want = S3Config(base.bytes())
    want.divider = divider_for(row["rate_hz"])
    want.set_sensors(SETS[row["sensors"]])
    for f in RANGES:
        want.put(f, row[f])
    for f, v in odr_codes(row["rate_hz"], via_commands=row["configure"] == "commands").items():
        want.put(f, v)
    return want


def _by_commands(link: bt.BtLink, want: S3Config) -> list[str]:
    """Send the configuration as the individual commands; return what their GETs read back wrong."""
    link.set_sensors(want.sensor_bytes())
    link.set_sampling_divider(want.divider)
    fields = [*RANGES, "wr_accel_rate", "wr_accel_lp_mode", "gyro_rate", "mag_rate"]
    for f in fields:
        link.set_setting(f, want.get(f))
    problems = []
    if (got := link.get_sampling_divider()) != want.divider:
        problems.append(f"GET_SAMPLING_RATE {got}, set {want.divider}")
    for f in fields:
        if (got := link.get_setting(f)) != want.get(f):
            problems.append(f"GET {f} {got}, set {want.get(f)}")
    return problems


def _differences(want: S3Config, back: S3Config) -> list[str]:
    """Each named field that differs, then any other byte."""
    out = []
    if back.divider != want.divider:
        out.append(f"divider {back.divider}, meant {want.divider}")
    if back.enabled() != want.enabled():
        out.append(f"sensors {sorted(back.enabled())}, meant {sorted(want.enabled())}")
    named = {0, 1, 6, 7, 8}                         # every bit of these is a field above
    for f in S3Config.FIELDS:
        if back.get(f) != want.get(f):
            out.append(f"{f} {back.get(f)}, meant {want.get(f)}")
    a, b = _masked(want.bytes()[:bt.INFOMEM_CONFIG_BYTES]), _masked(back.bytes()[:bt.INFOMEM_CONFIG_BYTES])
    other = [i for i, (x, y) in enumerate(zip(a, b)) if x != y and i not in named]
    if other:
        out.append(f"other bytes differ at offsets {other[:20]}")
    return out


def _configure(unit, base6, row, saved: Saved) -> tuple[S3Config, str]:
    """Write the row's configuration over Bluetooth, and read InfoMem back over the dock.

    Returns the stored image and a note. Raises _ConfigFailed, naming what
    differs, if the stored image is not what was meant.
    """
    note = row["configure"]
    try:
        with _open_bt(unit, base6) as link:
            base = S3Config(link.read_infomem())       # as a host reads it first
            want = _intended(base, row)
            if row["configure"] == "commands":
                problems = _by_commands(link, want)
            else:
                missing = want.with_calibration(saved.records)
                if missing:
                    note += f", no stored calibration block for {', '.join(missing)} at these ranges"
                link.write_infomem(want.bytes())
                problems = []
    except bt.BtError as e:
        raise _ConfigFailed(f"over Bluetooth: {e}") from e
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        back = S3Config(d.read_infomem(0, bt.INFOMEM_CONFIG_BYTES))
    problems += [f"read back over the dock: {p}" for p in _differences(want, back)]
    if problems:
        raise _ConfigFailed("; ".join(problems))
    return back, note


def _link_budget(rig, unit, row) -> str:
    """How much of this unit's radio the row needs; skips the row if more than LINK_FIT."""
    need = row["rate_hz"] * packet_bytes(SETS[row["sensors"]], row["crc"])
    sps = rig.throughput_baseline(unit).get("stream_samples_per_s")
    if not sps:
        return f"{need:.0f} B/s (no stream baseline for {unit.name}, so the fit is not known)"
    carries = sps * LOAD_PACKET
    if need > LINK_FIT * carries:
        pytest.skip(f"needs {need:.0f} B/s, and {unit.name}'s radio delivered {carries:.0f} B/s of a saturated "
                    f"stream (rig.toml): over {LINK_FIT:.0%} of it, loss would be the link's")
    return f"{need:.0f} B/s, {need / carries:.0%} of what this radio carries"


class _Aside:
    """Records observations as their own report items, beside the row's.

    release.bt_config: the configuration written over Bluetooth, read back over
    the dock. A failure there means the row's stream would test something else,
    so the row is not streamed.
    release.stop_ack: a stream stop that goes unACKed or leaves the link
    silent, as on Shimmer3R (test_s3r_release._Aside).
    """

    def __init__(self, request, row_id, unit_name):
        self.col = request.config._hil_collector
        self.who = f"{unit_name}: {row_id}"

    def config(self, problem: str | None):
        self.col.record("release.bt_config", "fail" if problem else "pass",
                        f"{self.who}: {problem}" if problem else None)

    def stop(self, st, hung: bool):
        bad = []
        if not st.stop_ack:
            bad.append("no ACK")
        if hung:
            bad.append("link silent afterwards, reconnected")
        self.col.record("release.stop_ack", "fail" if bad else "pass",
                        f"{self.who}: {', '.join(bad)} ({st.after_stop} packets after the stop)" if bad else None)


@pytest.mark.parametrize("row", ROWS, ids=[r["id"] for r in ROWS])
@pytest.mark.hil("release.{row}", needs=["shimmer3", "bt", "dock"])
def test_row(row, unit, rig, base6, saved, request, hil_note):
    aside = _Aside(request, row["id"], unit.name)
    budget = _link_budget(rig, unit, row)
    try:
        cfg, how = _configure(unit, base6, row, saved)
    except _ConfigFailed as e:
        aside.config(str(e))
        pytest.skip(f"not streamed: its configuration over Bluetooth did not take (release.bt_config): {e}")
    aside.config(None)

    with _open_bt(unit, base6) as link:
        inq = link.inquiry()
        # Calibration as a streaming host gets it: for the current ranges.
        calibs = decode.calibs_from_blob(link.all_calibration())
        link.set_crc(row["crc"])
        st = link.stream(row["seconds"], inq, keep=True)
        hung = not _answers_after_stop(link)
        if not hung:
            # A sensing Shimmer3 turns its dock UART off: name a stop not acted
            # on here, not as a dock timeout in the next row's read-back.
            link.wait_status(bt.STATUS_SENSING, False, timeout_s=5)
            link.set_crc(0)
    aside.stop(st, hung)
    if hung:
        # The silent link is closed (which also resets the CRC mode, §3.3):
        # confirm on a fresh one that sensing really stopped.
        with _open_bt(unit, base6) as again:
            again.wait_status(bt.STATUS_SENSING, False, timeout_s=15)

    sensors = SETS[row["sensors"]]
    probs = _inquiry_checks(inq, cfg, sensors) + _stream_checks(st, inq, cfg, row)
    chans = decode.split_channels(st.rows, inq.channels, bt.HW_SHIMMER3)
    sig, note = signals.check_rest(chans, calibs, cfg.rate_hz, mag_ceiling_hz=MAG_FLOOR_HZ, hw=bt.HW_SHIMMER3,
                                   defaults=cfg.default_calibrations(saved.records))
    probs += sig
    dlhc = " (mag X, Z, Y: LSM303DLHC)" if "mag" in sensors and list(inq.channels) == expected_channels(
        sensors, True) else ""
    hil_note(f"{how}; {st.packets} pkts, {len(inq.channels)} channels{dlhc}, {budget}; {note}")
    assert not probs, "; ".join(probs)


def _answers_after_stop(link) -> bool:
    """True if the device answers a command after a stream stop (test_s3r_release._answers_after_stop)."""
    try:
        link.status()                             # also flushes a stale packet
        return True
    except bt.BtError:
        return False


def _inquiry_checks(inq, cfg: S3Config, sensors: set[str]) -> list[str]:
    """The inquiry must describe the stored configuration: its divider, setup bytes
    6-9 and the channels in the firmware's order (STREAMING_DATA_FORMAT §4.1)."""
    out = []
    if inq.divider != cfg.divider:
        out.append(f"inquiry divider {inq.divider}, stored {cfg.divider}")
    if bytes(inq.config) != cfg.setup_bytes:
        out.append(f"inquiry setup bytes {bytes(inq.config).hex()}, stored {cfg.setup_bytes.hex()}")
    # The mag's order depends on the board's part, which only the firmware knows.
    allowed = [expected_channels(sensors, dlhc) for dlhc in (False, True)]
    if list(inq.channels) not in allowed:
        out.append(f"inquiry channels {[hex(c) for c in inq.channels]}, expected "
                   f"{[hex(c) for c in allowed[0]]} for {sorted(sensors)}")
    return out


def _stream_checks(st, inq, cfg: S3Config, row) -> list[str]:
    out = []
    if st.resyncs:
        out.append(f"{st.resyncs} framing resyncs (first: {st.misframes[:1]})")
    if st.crc_errors:
        out.append(f"{st.crc_errors} CRC errors (mode {row['crc']})")
    expected = row["seconds"] * cfg.rate_hz
    if st.packets < 0.8 * expected:
        out.append(f"{st.packets} packets in {st.seconds:.1f} s, expected ~{expected:.0f}")
    if st.gaps:
        out.append(f"{st.gaps} gaps, {st.missing} samples lost, largest step {st.max_gap_ticks} ticks")
    if st.dupes:
        out.append(f"{st.dupes} duplicate or backwards stamps")
    if st.max_jitter_ticks > MAX_JITTER_FRACTION * inq.divider:
        out.append(f"stamp jitter up to {st.max_jitter_ticks} ticks (period {inq.divider})")
    span = sum(d * n for d, n in st.deltas.items())
    if span and st.packets > 1:
        rate = (st.packets - 1) * 32768 / span
        if abs(rate / cfg.rate_hz - 1) > RATE_TOLERANCE and not st.gaps:
            out.append(f"device-clock rate {rate:.3f} Hz, configured {cfg.rate_hz:.3f} Hz")
    return out
