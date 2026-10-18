"""Soak tier: what only shows up over time or repetition.

Advisory, not required for a release (TEST_PROCEDURE §10.4): the point is a
measured rate per candidate - lost samples per hour, failures per hundred
reconnects - to compare across releases. Every item still fails on any
fault, so the report says plainly that something happened; its note carries
the rate.

Durations scale with HIL_SOAK_SCALE (default 1, about three hours on one
Shimmer3R and four Shimmer3s). 0.05 proves the tier in a few minutes; 3 is an
overnight run. A long log needs scale >= 0.8 to cross an hourly file rollover.

    set HIL_SOAK_SCALE=3
    python Test/hil/run_rc.py <tag> --no-flash --tier soak
"""

from __future__ import annotations

import os
import statistics
import time
from contextlib import contextmanager

import pytest

from rig import bt, dock, s3r_usb, sdfile
from rig.s3r_config import S3rConfig, divider_for

pytestmark = pytest.mark.soak

SCALE = float(os.environ.get("HIL_SOAK_SCALE", "1"))
LONG_LOG_MIN = 75 * SCALE          # 75 min crosses one hourly rollover at scale 1
LONG_STREAM_MIN = 30 * SCALE
DOCK_CYCLES = max(2, round(50 * SCALE))
RECONNECTS = max(2, round(50 * SCALE))
SOAK_RATE_HZ = 256                 # long log and long stream
RECONNECT_RATE_HZ = 51.2           # the rate every shipped radio sustains
RESET_RECOVERY_S = 60              # how long a reset unit gets to answer Bluetooth again
IMU = {"ln_accel", "gyro", "mag", "vbatt"}


# --- configuration, restored afterwards ---------------------------------------

@contextmanager
def s3r_config(unit, rate_hz, sensors=IMU):
    port = s3r_usb.wait_cdc(unit.usb_serial, 60)
    with dock.open_when_ready(port) as d:
        original = d.read_infomem()
        cfg = S3rConfig(original)
        cfg.divider = divider_for(rate_hz)
        cfg.set_sensors(sensors)
        d.write_infomem(cfg.bytes())
    try:
        yield
    finally:
        port = s3r_usb.wait_cdc(unit.usb_serial, 120)
        with dock.open_when_ready(port) as d:
            d.write_infomem(original)


@contextmanager
def s3_rate(unit, base6, rate_hz):
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        original = d.read_infomem(0, 2)
        d.write_infomem(divider_for(rate_hz).to_bytes(2, "little"), offset=0)
    try:
        yield
    finally:
        base6.select(unit.base6_slot)
        with dock.open_when_ready(base6.uart_port) as d:
            d.write_infomem(original, offset=0)


@pytest.fixture(scope="session")
def base6_if_any(rig):
    if not rig.base6_serial:
        yield None
        return
    from rig.base6 import Base6
    with Base6(rig.base6_serial) as b:
        yield b


# --- long log -------------------------------------------------------------------

@pytest.mark.hil("soak.long_log", needs=["shimmer3r", "usb", "bt", "hub_port"])
def test_long_log(unit, power, hil_note):
    """Log for LONG_LOG_MIN undocked; judge every file of the session as one recording."""
    port = bt.spp_port(unit.mac)
    assert port, f"no paired SPP port for {unit.mac}"
    with s3r_config(unit, SOAK_RATE_HZ):
        root = s3r_usb.wait_drive(unit.usb_serial, 60)
        before = sdfile.session_dirs(root)
        power.off(unit.hub_port, unit.name)
        try:
            s3r_usb.wait_cdc_gone(unit.usb_serial, 15)
            with bt.BtLink(port) as link:
                link.simple(bt.START_LOGGING)
                link.wait_status(bt.STATUS_LOGGING, True, 10)
            # No link held open for an hour: reconnect to stop, as a user would.
            time.sleep(LONG_LOG_MIN * 60)
            with bt.BtLink(port) as link:
                link.simple(bt.STOP_LOGGING)
                link.wait_status(bt.STATUS_LOGGING, False, 30)
        finally:
            power.on(unit.hub_port, unit.name)
        root = s3r_usb.wait_drive(unit.usb_serial, 120)
        new = sorted(sdfile.session_dirs(root) - before)
    assert len(new) == 1, f"expected one new session, found {len(new)}"
    sc = sdfile.check_session(root / new[0])     # relative: the drive letter may have changed
    per_hour = sc.missing / sc.hours if sc.hours else 0
    hil_note(f"{SOAK_RATE_HZ} Hz, {sc.hours:.2f} h in {len(sc.files)} files: {sc.records} records, "
             f"{sc.missing} lost in {sc.gaps} gaps ({sc.loss_fraction():.3%}, {per_hour:.0f}/h), "
             f"joins {[d for _, d in sc.joins]}")
    problems = []
    if sc.header_mismatches:
        problems.append(f"files {sc.header_mismatches} disagree with 000's header")
    bad_joins = [(f, d) for f, d in sc.joins if abs(d - sc.header.divider) > 1]
    if bad_joins:
        problems.append(f"file joins off one period: {bad_joins}")
    if sc.zero_ts:
        problems.append(f"{sc.zero_ts} zero timestamps")
    if sc.gaps or sc.dupes:
        problems.append(f"{sc.missing} samples lost in {sc.gaps} gaps ({sc.loss_fraction():.3%}, "
                        f"{sc.missing / sc.hours if sc.hours else 0:.0f}/h over {sc.hours:.2f} h), "
                        f"largest stall {sc.max_gap_ticks} ticks ({sc.max_gap_ticks / 32.768:.0f} ms), "
                        f"{sc.dupes} duplicates; first gaps at {sc.gap_times_s[:10]} s")
    assert not problems, "; ".join(problems)


# --- dock / undock --------------------------------------------------------------

@pytest.mark.hil("soak.dock_cycles", needs=["shimmer3r", "usb", "hub_port"])
def test_dock_cycles(unit, power, hil_note):
    """Undock and re-dock DOCK_CYCLES times; USB, the dock protocol and the drive must return."""
    enum_s, failures = [], []
    for i in range(DOCK_CYCLES):
        try:
            power.off(unit.hub_port, unit.name)
            s3r_usb.wait_cdc_gone(unit.usb_serial, 15)
            time.sleep(2)
            power.on(unit.hub_port, unit.name)
            t0 = time.monotonic()
            port = s3r_usb.wait_cdc(unit.usb_serial, 60)
            enum_s.append(time.monotonic() - t0)
            with dock.open_when_ready(port) as d:
                d.version()
            s3r_usb.wait_drive(unit.usb_serial, 60)
        except Exception as e:                      # record and carry on: a rate, not a stop
            failures.append(f"cycle {i + 1}: {type(e).__name__}: {str(e)[:120]}")
            power.on(unit.hub_port, unit.name)
            time.sleep(10)
    if enum_s:
        hil_note(f"{DOCK_CYCLES} cycles, {len(failures)} failed; CDC back in median "
                 f"{statistics.median(enum_s):.1f} s, max {max(enum_s):.1f} s")
    assert not failures, f"{len(failures)} of {DOCK_CYCLES} cycles failed: {failures[:5]}"


# --- Bluetooth reconnect loop -----------------------------------------------------

def _reconnect_loop(unit, n, reset=None) -> tuple[dict, list]:
    """N connect -> status -> 3 s stream -> stop -> status -> close cycles.

    With `reset` (Shimmer3: a reset through the Base6 RST line), two failed
    connects in a row trigger a reset and a retry, and the count says whether
    the reset brought Bluetooth back. Seen once on both RN42 units at
    v1.01.006: classic Bluetooth dead while the dock answered, cleared only by
    a reset - the "until reset" signature of a firmware-side hang.
    """
    port = bt.spp_port(unit.mac)
    if not port:
        pytest.skip(f"no paired SPP port for {unit.mac}")
    counts = {"connect_failed": 0, "no_status": 0, "stream_loss": 0, "stop_unacked": 0,
              "silent_after_stop": 0, "recovered_by_reset": 0, "dead_after_reset": 0}
    first = []
    fails_in_row = 0
    for i in range(n):
        try:
            link = bt.BtLink(port, connect_timeout_s=20)
            fails_in_row = 0
        except Exception as e:
            counts["connect_failed"] += 1
            fails_in_row += 1
            first.append(f"{i + 1}: connect {str(e)[:80]}")
            if reset and fails_in_row >= 2:
                reset()
                # Retry for up to RESET_RECOVERY_S rather than once: the unit
                # must boot and bring its radio up before a page can succeed,
                # and a single early try would call a slow boot "dead".
                t0, err = time.monotonic(), None
                while time.monotonic() - t0 < RESET_RECOVERY_S:
                    time.sleep(3)
                    try:
                        bt.BtLink(port, connect_timeout_s=20).close()
                        err = None
                        break
                    except Exception as e:
                        err = e
                if err is None:
                    counts["recovered_by_reset"] += 1
                    first.append(f"{i + 1}: reset brought Bluetooth back after "
                                 f"{time.monotonic() - t0:.0f} s")
                else:
                    counts["dead_after_reset"] += 1
                    first.append(f"{i + 1}: still no Bluetooth {RESET_RECOVERY_S} s after reset: {str(err)[:80]}")
                fails_in_row = 0
            else:
                time.sleep(10)
            continue
        try:
            try:
                link.status()
            except bt.BtError:
                counts["no_status"] += 1
                first.append(f"{i + 1}: no status reply on a fresh link")
                continue
            st = link.stream(3, link.inquiry())
            if st.gaps or st.dupes or st.resyncs:
                counts["stream_loss"] += 1
                first.append(f"{i + 1}: stream {st.missing} lost in {st.gaps} gaps, {st.dupes} dupes, "
                             f"{st.resyncs} resyncs, largest step {st.max_gap_ticks} ticks")
            if not st.stop_ack:
                counts["stop_unacked"] += 1
            try:
                link.status()
            except bt.BtError:
                counts["silent_after_stop"] += 1
                first.append(f"{i + 1}: silent after stop ({st.after_stop} packets after it)")
        finally:
            link.close()
        time.sleep(1)
    return counts, first


@pytest.mark.hil("soak.bt_reconnect", needs=["shimmer3r", "bt", "usb"])
def test_bt_reconnect_s3r(unit, hil_note):
    with s3r_config(unit, RECONNECT_RATE_HZ):
        counts, first = _reconnect_loop(unit, RECONNECTS)
    bad = {k: v for k, v in counts.items() if v}
    hil_note(f"{RECONNECTS} cycles at {RECONNECT_RATE_HZ} Hz: " + (", ".join(f"{k} {v}" for k, v in bad.items()) or "all clean"))
    assert not bad, f"{bad}; first: {first[:4]}"


@pytest.mark.hil("soak.bt_reconnect", needs=["shimmer3", "bt", "dock"])
def test_bt_reconnect_s3(unit, base6_if_any, hil_note):
    from rig.s3_flash import reset_slot
    with s3_rate(unit, base6_if_any, RECONNECT_RATE_HZ):
        counts, first = _reconnect_loop(unit, RECONNECTS,
                                        reset=lambda: reset_slot(base6_if_any, unit.base6_slot))
    bad = {k: v for k, v in counts.items() if v}
    hil_note(f"{unit.radio}: {RECONNECTS} cycles: " + (", ".join(f"{k} {v}" for k, v in bad.items()) or "all clean"))
    assert not bad, f"{bad}; first: {first[:4]}"


# --- long stream ------------------------------------------------------------------

@pytest.mark.hil("soak.long_stream", needs=["shimmer3r", "bt", "usb"])
def test_long_stream(unit, hil_note):
    port = bt.spp_port(unit.mac)
    assert port, f"no paired SPP port for {unit.mac}"
    with s3r_config(unit, SOAK_RATE_HZ):
        with bt.BtLink(port) as link:
            inq = link.inquiry()
            st = link.stream(LONG_STREAM_MIN * 60, inq)
            try:
                link.status()
                silent = False
            except bt.BtError:
                silent = True        # the stop left the link silent; closing it recovers
    expected = LONG_STREAM_MIN * 60 * inq.rate_hz
    hil_note(f"{LONG_STREAM_MIN:.0f} min at {inq.rate_hz:.0f} Hz: {st.packets} packets of ~{expected:.0f}, "
             f"{st.missing} lost in {st.gaps} gaps, {st.dupes} dupes, {st.resyncs} resyncs, "
             f"jitter max {st.max_jitter_ticks} ticks")
    problems = []
    if st.gaps or st.dupes:
        problems.append(f"{st.missing} samples lost in {st.gaps} gaps, {st.dupes} duplicates")
    if st.resyncs or st.crc_errors:
        problems.append(f"{st.resyncs} resyncs, {st.crc_errors} CRC errors (first: {st.misframes[:1]})")
    if st.max_jitter_ticks > inq.divider // 4:
        problems.append(f"jitter up to {st.max_jitter_ticks} ticks (period {inq.divider})")
    if not st.stop_ack or silent:
        problems.append(f"stop {'unACKed' if not st.stop_ack else 'ACKed'}"
                        f"{', link silent afterwards until reconnect' if silent else ''}")
    assert not problems, "; ".join(problems)
