"""Shimmer3R smoke tier: does this build boot, talk, stream, log and dock?

Each test is one report item (see conftest.py). Thresholds are deliberately
strict where the firmware is deterministic - a timestamp step is either one
sample period or a dropped sample - and stated where they are judgement.

Order matters only in that test_sd_log leaves the unit docked, which is the
state every other test starts from.
"""

from __future__ import annotations

import time
from pathlib import Path

import pytest

from rig import bt, dock, s3r_usb, sdfile, throughput

pytestmark = pytest.mark.smoke

STREAM_SECONDS = 10.0
LINK_SECONDS = 10.0
LOAD_SECONDS = 20.0
LOG_SECONDS = 15.0
# Device-clock rate must match the configured divider exactly; this allows for
# one tick of quantisation over the run, nothing more.
RATE_TOLERANCE = 1e-3
# A sample may be stamped a little late (interrupt latency) without being lost.
# Judgement, not specification: past a quarter of a period, a host that infers
# sample time from the stamp is misled enough to matter.
MAX_JITTER_FRACTION = 0.25
# A missing tail leaves no gap between the samples that did arrive, so the
# samples' own timestamps must span the time asked for. A stream's stamps fall
# short of the host's stream time by the start latency and the samples in
# flight when it stops: 0.25 to 0.29 s on the bench (497 and 499 of 512 at
# 51.2 Hz). A log's records cover at least the time between the host seeing
# the logging bit and sending STOP (771 of 768 at 51.2 Hz). Past these the end
# of the data is missing.
STREAM_TAIL_S = 0.5
LOG_TAIL_S = 0.25

# InfoMem bits the firmware changes by itself when a configuration is written,
# by design: (offset, mask). flagWriteCfgToSd is set on every write so that
# sdlog.cfg is regenerated (docs/SHIMMER3_CONFIGURATION_INFOMEM.md, byte 230).
SELF_SET_BITS = [(230, 0x01)]


def _masked(im: bytes) -> bytes:
    b = bytearray(im)
    for off, mask in SELF_SET_BITS:
        b[off] &= ~mask & 0xFF
    return bytes(b)


def _timing_problems(st, period: int) -> list[str]:
    out = []
    if st.gaps:
        out.append(f"{st.gaps} gaps, {st.missing} samples lost, largest step {st.max_gap_ticks} ticks")
    if st.dupes:
        out.append(f"{st.dupes} steps under half a period (duplicate or backwards stamps)")
    if st.max_jitter_ticks > MAX_JITTER_FRACTION * period:
        out.append(f"stamp jitter up to {st.max_jitter_ticks} ticks (period {period})")
    return out


@pytest.fixture(scope="module")
def cdc(unit):
    return s3r_usb.wait_cdc(unit.usb_serial, 30)


@pytest.fixture
def docklink(cdc):
    with dock.open_when_ready(cdc) as d:
        yield d


@pytest.fixture
def btlink(unit):
    port = bt.spp_port(unit.mac)
    if not port:
        pytest.skip(f"no paired SPP port for {unit.mac} - pair it once in Windows")
    with bt.BtLink(port) as link:
        yield link


def _expected_version(candidate):
    return candidate["firmware_version"].lstrip("vV") if candidate else None


@pytest.mark.hil("smoke.boot_identity", needs=["shimmer3r", "usb"])
def test_boot_identity(unit, docklink, candidate, hil_note):
    v = docklink.version()
    assert v.hw == 10, f"hardware version {v.hw}, expected 10 (Shimmer3R)"
    assert v.fw_id == 3, f"firmware identifier {v.fw_id}, expected 3 (LogAndStream)"
    want = _expected_version(candidate)
    if want:
        assert v.text == want, f"unit runs v{v.text}, candidate is v{want} - was it flashed?"
    if unit.mac:
        assert docklink.mac() == unit.mac, "MAC differs from rig.toml - wrong unit on this port?"
    hil_note(f"v{v.text}, radio {docklink.bt_version()}")


@pytest.mark.hil("smoke.config_roundtrip", needs=["shimmer3r", "usb"])
def test_config_roundtrip(docklink, hil_note):
    """Write the configuration back unchanged, then change the rate and restore it.

    The firmware validates and corrects what it is given (checkAndCorrectConfig),
    so a value it accepted must read back exactly - anything else is a change
    the host did not ask for.
    """
    original = docklink.read_infomem()
    assert len(original) == dock.INFOMEM_SIZE

    docklink.write_infomem(original)
    back = docklink.read_infomem()
    diff = [i for i, (a, b) in enumerate(zip(_masked(original), _masked(back))) if a != b]
    assert not diff, f"unchanged config read back different at offsets {diff}"

    div = int.from_bytes(original[0:2], "little")
    new_div = 655 if div != 655 else 328          # ~50 Hz / ~100 Hz: both always valid
    try:
        docklink.write_infomem(new_div.to_bytes(2, "little"), offset=0)
        got = int.from_bytes(docklink.read_infomem(0, 2), "little")
        assert got == new_div, f"wrote rate divider {new_div}, read back {got}"
    finally:
        docklink.write_infomem(original)
    assert _masked(docklink.read_infomem()) == _masked(original), "config not restored"
    hil_note(f"divider {div} -> {new_div} -> {div}")


@pytest.mark.hil("smoke.bt_identity", needs=["shimmer3r", "bt"])
def test_bt_identity(btlink, candidate, hil_note):
    assert btlink.hw == 10
    fw_id, major, minor, patch = btlink.fw_version()
    assert fw_id == 3, f"firmware identifier {fw_id} over Bluetooth, expected 3 (LogAndStream)"
    text = f"{major}.{minor:02d}.{patch:03d}"
    want = _expected_version(candidate)
    if want:
        assert text == want, f"over Bluetooth the unit reports v{text}, candidate is v{want}"
    s = btlink.status()
    assert s & bt.STATUS_SD_INSERTED, f"status {s:#04x}: no SD card"
    assert not s & bt.STATUS_SD_BAD_FILE, f"status {s:#04x}: SD card or file unusable"
    hil_note(f"status {s:#04x}")


@pytest.mark.hil("smoke.bt_stream", needs=["shimmer3r", "bt"])
def test_bt_stream(btlink, hil_note):
    inq = btlink.inquiry()
    st = btlink.stream(STREAM_SECONDS, inq)
    assert st.packets > 1, f"{st.packets} packets in {st.seconds:.1f} s"
    assert st.resyncs == 0, f"{st.resyncs} bytes did not frame as packets"
    problems = _timing_problems(st, inq.divider)
    assert not problems, "; ".join(problems)
    span = sum(d * n for d, n in st.deltas.items())
    covered_s = (span + inq.divider) / bt.TICKS_PER_S
    assert st.seconds - covered_s <= STREAM_TAIL_S, \
        f"the stamps cover {covered_s:.2f} s of a {st.seconds:.2f} s stream: the end is missing, or it started late"
    device_rate = (st.packets - 1) * bt.TICKS_PER_S / span
    assert abs(device_rate / inq.rate_hz - 1) < RATE_TOLERANCE, \
        f"device-clock rate {device_rate:.3f} Hz, configured {inq.rate_hz:.3f} Hz"
    hil_note(f"{st.packets} packets at {inq.rate_hz:.1f} Hz over {covered_s:.2f} of {st.seconds:.2f} s, "
             f"{len(inq.channels)} channels, no loss, {st.jitter} late stamps (max {st.max_jitter_ticks} ticks)")


@pytest.mark.hil("smoke.bt_throughput", needs=["shimmer3r", "bt", "usb"])
def test_bt_throughput(rig, unit, docklink, btlink, hil_note):
    """What the link carries, and how much of a 1024 Hz stream arrives (TEST_PROCEDURE §5.5).

    rig/throughput.py has the two measurements, judged against this bench's
    figures for the unit's IF820 module build ([throughput_by_radio] in
    rig.toml) and any of the unit's own; without any the item is not_covered,
    with the figures to record. The module is checked to be the version
    rig.toml names first, because that picks the baseline. If the IF820
    carries the whole load, the stream figure sits near 1024 and a fall is
    samples lost. The load is stored configuration, so InfoMem is put back.
    """
    mismatch = throughput.radio_mismatch(unit.radio, docklink.bt_version())
    assert not mismatch, mismatch
    baseline = rig.throughput_baseline(unit)
    original = docklink.read_infomem()
    try:
        rate = throughput.data_rate_test(btlink, LINK_SECONDS)
        st, inq = throughput.load_stream(btlink, LOAD_SECONDS)
    finally:
        docklink.write_infomem(original)
        restored = _masked(docklink.read_infomem()) == _masked(original)
    assert restored, "configuration not restored after the throughput load"
    problems = throughput.defects(rate, st)
    assert not problems, "; ".join(problems)
    measured = throughput.figures(rate, st, inq)
    if not baseline:
        pytest.skip(f"no throughput baseline for {unit.name} ({unit.radio}) in rig.toml; "
                    f"measured {throughput.as_toml(measured)}")
    short, note = throughput.judge(measured, baseline, unit.name)
    hil_note(note)
    assert not short, "; ".join(short)


@pytest.mark.hil("smoke.usb_replug", needs=["shimmer3r", "usb", "hub_port"])
def test_usb_replug(unit, power):
    """Undock and re-dock: the CDC port, the dock protocol and the SD card must all come back."""
    power.off(unit.hub_port, unit.name)
    try:
        s3r_usb.wait_cdc_gone(unit.usb_serial, 15)
        time.sleep(2)
    finally:
        # Whatever happened while undocked, the tests after this one start docked.
        power.on(unit.hub_port, unit.name)
    port = s3r_usb.wait_cdc(unit.usb_serial, 30)
    with dock.open_when_ready(port) as d:
        d.version()
    s3r_usb.wait_drive(unit.usb_serial, 45)


@pytest.mark.hil("smoke.sd_log", needs=["shimmer3r", "usb", "bt", "hub_port"])
def test_sd_log(unit, power, hil_note):
    """Undock, log over a Bluetooth command, re-dock, and read the new session over USB.

    Logging cannot start while USB power is present, and docking stops it
    (LogAndStream_setupDock), so the hub port is the dock.
    """
    root = s3r_usb.wait_drive(unit.usb_serial, 45)
    before = sdfile.session_dirs(root)

    port = bt.spp_port(unit.mac)
    if not port:
        pytest.skip(f"no paired SPP port for {unit.mac}")

    power.off(unit.hub_port, unit.name)
    try:
        s3r_usb.wait_cdc_gone(unit.usb_serial, 15)
        with bt.BtLink(port) as link:
            inq = link.inquiry()
            link.simple(bt.START_LOGGING)
            link.wait_status(bt.STATUS_LOGGING, True, 10)
            time.sleep(LOG_SECONDS)
            link.simple(bt.STOP_LOGGING)
            link.wait_status(bt.STATUS_LOGGING, False, 10)
    finally:
        power.on(unit.hub_port, unit.name)

    root = s3r_usb.wait_drive(unit.usb_serial, 60)
    new = sorted(sdfile.session_dirs(root) - before)      # relative: the drive letter may have changed
    assert len(new) == 1, f"expected one new session directory, found {len(new)}: {new}"
    files = sorted(p for p in (root / new[0]).iterdir() if p.is_file())
    assert files and files[0].name == "000", f"{new[0]}: no data file 000 ({[f.name for f in files]})"

    fc = sdfile.check(files[0])
    h = fc.header
    assert h.mac == unit.mac, f"header MAC {h.mac}"
    assert h.divider == inq.divider, f"header divider {h.divider}, inquiry said {inq.divider}"
    assert h.channels == inq.channels, f"header channels {h.channels}, inquiry said {inq.channels}"
    assert fc.records > 1, f"{fc.records} records"
    assert fc.zero_ts == 0, f"{fc.zero_ts} records with a zero timestamp"
    problems = _timing_problems(fc, h.divider)
    assert not problems, "; ".join(problems)
    assert fc.trailing_bytes == 0, f"{fc.trailing_bytes} bytes after the last whole record"
    covered_s = (fc.span_ticks + h.divider) / bt.TICKS_PER_S
    assert covered_s >= LOG_SECONDS - LOG_TAIL_S, \
        f"the records cover {covered_s:.2f} s of a {LOG_SECONDS:.0f} s log: the end is missing"
    hil_note(f"{new[0].parent.name}/{new[0].name}: {fc.records} records over {covered_s:.2f} s, no loss, "
             f"{fc.jitter} late stamps (max {fc.max_jitter_ticks} ticks)")
