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

from rig import ble, bt, dock, dock_protocol, flash, midstream, s3r_boards, s3r_usb, sdfile, throughput, usb_throughput
from rig.s3r_config import S3rConfig, divider_for

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


@pytest.mark.hil("smoke.dock_protocol", needs=["shimmer3r", "usb"])
def test_dock_protocol(unit, rig, docklink, candidate, hil_note):
    """Every dock command over USB CDC, each error reply and the framing rules
    (rig/dock_protocol.py). What it writes it puts back; the clock is set to the PC's."""

    def recover() -> str:
        # The reboot into ST's bootloader went ahead: start the application again.
        if not s3r_usb.dfu_present():
            return "no DFU device appeared either"
        if not rig.stm32_programmer:
            return "the unit is in DFU, and rig.toml names no stm32_programmer to start it"
        try:
            flash.start_from_dfu_and_wait(rig.stm32_programmer, unit.usb_serial,
                                          rig.stlink_for(unit), 30)
        except flash.FlashError as e:
            return str(e)                       # DEV-1316: the bootloader missed the jump
        return "the application was started again from DFU"

    note, failures = dock_protocol.walk(docklink, dock_protocol.S3R, "USB CDC", mac=unit.mac,
                                        version=_expected_version(candidate), recover=recover)
    hil_note(note)
    assert not failures, "; ".join(failures)


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


RESTORE_ATTEMPTS = 3


def _restore_infomem(unit, original: bytes, backup: Path, load_error: BaseException | None) -> None:
    """Write InfoMem back on a dock link opened now, retrying while the USB re-enumerates.

    A link opened before the load can be dead by the end of it: on the bench
    (19 Oct 2026) the first write after a docked throughput run failed with
    "The device does not recognize the command" on a handle held across it.
    Runs in a finally block, so a failure names the load's own error and the
    backup to restore from by hand, and is chained from the load's error.
    """
    seen = []
    for _ in range(RESTORE_ATTEMPTS):
        try:
            port = s3r_usb.wait_cdc(unit.usb_serial, 30)
            with dock.open_when_ready(port) as d:
                d.write_infomem(original)
                if _masked(d.read_infomem()) != _masked(original):
                    raise dock.DockError("read back differs")
            return
        except (dock.DockError, OSError) as e:   # serial.SerialException and TimeoutError are OSErrors
            seen.append(str(e))
            time.sleep(2.0)
    msg = f"InfoMem not restored after the throughput load ({'; '.join(seen)}); the original is in {backup}"
    if load_error is not None:
        raise dock.DockError(f"{msg} - after the load itself failed with {load_error!r}") from load_error
    raise dock.DockError(msg)


@pytest.mark.hil("smoke.bt_throughput", needs=["shimmer3r", "bt", "usb"])
def test_bt_throughput(rig, unit, cdc, btlink, hil_note):
    """What the link carries, and how much of a 1024 Hz stream arrives (TEST_PROCEDURE §5.5).

    rig/throughput.py has the two measurements, judged against this bench's
    figures for the unit's IF820 module build ([throughput_by_radio] in
    rig.toml) and any of the unit's own; without any the item is not_covered,
    with the figures to record. The module is checked to be the version
    rig.toml names first, because that picks the baseline. If the IF820
    carries the whole load, the stream figure sits near 1024 and a fall is
    samples lost. The load is stored configuration, so InfoMem is saved to
    disk and put back.
    """
    with dock.open_when_ready(cdc) as d:
        reported = d.bt_version()
        original = bytes(d.read_infomem())
    mismatch = throughput.radio_mismatch(unit.radio, reported)
    assert not mismatch, mismatch
    baseline = rig.throughput_baseline(unit)
    backup = throughput.save_infomem(unit.name, original)
    error = None
    try:
        rate = throughput.data_rate_test(btlink, LINK_SECONDS)
        st, inq = throughput.load_stream(btlink, LOAD_SECONDS)
    except BaseException as e:
        error = e
        raise
    finally:
        _restore_infomem(unit, original, backup, error)
    problems = throughput.defects(rate, st, inq)
    assert not problems, "; ".join(problems)
    measured = throughput.figures(rate, st, inq)
    if not baseline:
        pytest.skip(f"no throughput baseline for {unit.name} ({unit.radio}) in rig.toml; "
                    f"measured {throughput.as_toml(measured)}")
    short, note = throughput.judge(measured, baseline, unit.name)
    hil_note(note)
    assert not short, "; ".join(short)


@pytest.mark.hil("smoke.ble_throughput", needs=["shimmer3r", "bt", "ble", "usb"])
def test_ble_throughput(rig, unit, cdc, hil_note):
    """smoke.bt_throughput's two measurements over BLE instead of SPP (TEST_PROCEDURE §5.5).

    The link is the IF820's CYSPP service with data as notifications, as the
    web SDK receives it (rig/ble.py). The figures are judged against this
    bench's BLE figures for the unit's module build ([ble_throughput_by_radio]
    in rig.toml) and any of the unit's own (ble_throughput); without any the
    item is not_covered, with the figures to record. "bt" is among its needs so
    that the run turns the radio on (conftest._bt_on). A unit whose EEPROM has
    BLE switched off (bleEnabled, EEPROM_MEMORY_MAP §4.2) is not_covered too:
    the bench cannot reach it over BLE, and the run does not change the EEPROM.
    The load is stored configuration, so InfoMem is saved to disk and put back.
    """
    with dock.open_when_ready(cdc) as d:
        # A dock that cannot read the page fails the item: that is a broken
        # dock or device, not a unit without BLE.
        radio = d.radio_settings()
        if not dock.ble_enabled(radio):
            pytest.skip(f"{unit.name}: BLE is off in its EEPROM (radio settings byte 2 = {radio[2]:#04x}, "
                        "bleEnabled clear)")
        reported = d.bt_version()
        original = bytes(d.read_infomem())
    mismatch = throughput.radio_mismatch(unit.radio, reported)
    assert not mismatch, mismatch
    baseline = rig.ble_throughput_baseline(unit)
    backup = throughput.save_infomem(unit.name, original)
    error = None
    try:
        with bt.BtLink(ser=ble.BleSerial(unit.mac)) as link:
            hil_note(link.ser.describe())
            rate = throughput.data_rate_test(link, LINK_SECONDS)
            st, inq = throughput.load_stream(link, LOAD_SECONDS)
    except BaseException as e:
        error = e
        raise
    finally:
        _restore_infomem(unit, original, backup, error)
    problems = throughput.defects(rate, st, inq)
    assert not problems, "; ".join(problems)
    measured = throughput.figures(rate, st, inq)
    if not baseline:
        pytest.skip(f"no BLE throughput baseline for {unit.name} ({unit.radio}) in rig.toml; "
                    f"measured {throughput.as_toml(measured, 'ble_throughput')}")
    short, note = throughput.judge(measured, baseline, unit.name)
    hil_note(note)
    # A PC's Bluetooth stack can stick in a state that holds BLE at about 60%
    # until Bluetooth is switched off and on (TEST_PROCEDURE §5.5).
    assert not short, "; ".join(short) + (" - switch the PC's Bluetooth off and on and repeat before "
                                         "believing a BLE shortfall")


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


@pytest.mark.hil("smoke.usb_throughput", needs=["shimmer3r", "usb"])
def test_usb_throughput(unit, hil_note):
    """How fast the SD card reads and writes over USB mass storage, as a customer offloads a recording.

    rig/usb_throughput.py writes a 32 MiB file to the unit's drive around the
    PC's cache, reads it back and compares it, then removes it. Both rates are
    judged against the unit's own `usb_throughput` in rig.toml; without one the
    item is not_covered, with the figures to record. The card sets the figure,
    so it is per unit. A read within USB Full Speed is reported as such: it
    points at the link, not the card.
    """
    if not usb_throughput.supported():
        pytest.skip("the unbuffered transfers this item times are only implemented on Windows")
    root = s3r_usb.wait_drive(unit.usb_serial, 45)
    result = usb_throughput.measure(root)
    measured = result.figures()
    slow_link = usb_throughput.speed_note(measured)
    if not unit.usb_throughput:
        pytest.skip(f"no USB throughput baseline for {unit.name} in rig.toml; measured "
                    f"{usb_throughput.as_toml(measured)}" + (f"; {slow_link}" if slow_link else ""))
    short, note = throughput.judge(measured, unit.usb_throughput, unit.name, usb_throughput.USB_THROUGHPUT_KEYS)
    hil_note(note + (f"; {slow_link}" if slow_link else ""))
    assert not short, "; ".join(short + ([slow_link] if slow_link else []))


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


MIDSTREAM_CYCLES = 6
MIDSTREAM_DIVIDER = 128         # 256 Hz
MIDSTREAM_CRC = 2


@pytest.mark.hil("smoke.sd_start_midstream", needs=["shimmer3r", "usb", "bt", "hub_port"])
def test_sd_start_midstream(unit, cdc, power, hil_note):
    """Start and stop SD logging while streaming: the stream's stamps must not notice.

    Six 2 s logs inside one 256 Hz stream (rig/midstream.py). Odd cycles send
    START_LOGGING alone. Even ones set the clock first, as the reference host
    does before every log start. The clock is set once before the stream too,
    so every mid-stream set is a repeat, not the first set after power-up.

    The stream runs with link CRC on, so every frame is confirmed by its CRC.
    Without one a frame is confirmed by a plausible timestamp step, and an
    off-grid stamp would be dropped as a misframe instead of judged.

    Fails on a stamp at or behind the one before, a moved sampling grid, lost
    or duplicate samples, or jitter past a quarter period, naming the cycle each
    followed. Each start must also have begun a log: a status query shows
    logging, and once re-docked the card holds one new session per cycle.

    Logging cannot start docked, so the hub port is the dock, as in
    smoke.sd_log. The rate is stored configuration, so InfoMem is saved to disk
    and put back.
    """
    port = bt.spp_port(unit.mac)
    if not port:
        pytest.skip(f"no paired SPP port for {unit.mac}")
    with dock.open_when_ready(cdc) as d:
        original = bytes(d.read_infomem())
        d.write_infomem(MIDSTREAM_DIVIDER.to_bytes(2, "little"), offset=0)
    backup = throughput.save_infomem(unit.name, original)
    plan = midstream.schedule(MIDSTREAM_CYCLES)
    error = None
    try:
        root = s3r_usb.wait_drive(unit.usb_serial, 45)
        before = sdfile.session_dirs(root)
        power.off(unit.hub_port, unit.name)
        try:
            s3r_usb.wait_cdc_gone(unit.usb_serial, 15)
            with bt.BtLink(port) as link:
                link.command(midstream.SET_RWC, midstream.rwc_payload()[1:])
                inq = link.inquiry()
                assert inq.divider == MIDSTREAM_DIVIDER, f"inquiry reports divider {inq.divider} after setting it"
                link.status_len()            # asked with CRC off, so its reply has no CRC bytes
                link.set_crc(MIDSTREAM_CRC)
                st = link.stream(plan.seconds, inq, keep=True, actions=plan.actions)
                link.wait_status(bt.STATUS_SENSING, False, 15)
                link.set_crc(0)
        finally:
            power.on(unit.hub_port, unit.name)
    except BaseException as e:
        error = e
        raise
    finally:
        _restore_infomem(unit, original, backup, error)

    root = s3r_usb.wait_drive(unit.usb_serial, 90)
    new = sdfile.session_dirs(root) - before
    problems, summary = midstream.judge(st, plan, inq.divider)
    if not midstream.logging_seen(st):
        problems.append("no status during the stream showed logging")
    if len(new) != MIDSTREAM_CYCLES:
        problems.append(f"{len(new)} new SD sessions for {MIDSTREAM_CYCLES} log starts")
    assert not problems, f"{summary}: " + "; ".join(problems)
    hil_note(f"{summary}; {len(new)} sessions logged")


INQUIRY_SECONDS = 60.0
INQUIRY_INTERVAL_S = 0.25
INQUIRY_RATE_HZ = 1024
INQUIRY_SENSORS = {"ln_accel", "gyro", "mag", "vbatt", "wr_accel", "alt_accel", "alt_mag", "pressure", "gsr"}
# v1.01.024 lost samples on about one inquiry in 17 (11 in 187 on the bench).
# A clean log after fewer inquiries than this could be luck: 120 miss the race
# about once in 1,500 runs, the ~190 a run sends about once in 80,000.
MIN_INQUIRIES = 120
TIMEOUT_100_MS = 3277           # log_and_stream_definitions.h: the gather fail-safe's limit


def _failsafe_loss(divider: int) -> int:
    """Samples lost when the packet ring's fail-safe releases a gather that never
    completed (stallLimitPeriods, ShimSens_startSensing): 104 at 1024 Hz."""
    return max(2, TIMEOUT_100_MS // divider) + 2


@pytest.mark.hil("smoke.sd_inquiry_midlog", needs=["shimmer3r", "usb", "bt", "hub_port"])
def test_sd_inquiry_midlog(unit, cdc, power, hil_note):
    """INQUIRY every 0.25 s through a 60 s SD log at 1024 Hz: the log must lose nothing.

    A regression case (TEST_PROCEDURE §7, DEV-1363). Up to v1.01.024 the
    inquiry reply rebuilt the channel layout even while sensing, and the
    rebuild raced the gather in the RTC interrupt. A gather that lost the race
    never completed, and the packet ring's fail-safe released it 100 ms later:
    104 samples at 1024 Hz, on about one inquiry in 17. A host that connects
    to a unit already logging sends exactly this inquiry; the Java driver does
    on connect. smoke.sd_log cannot see it: its one inquiry comes before
    START_LOGGING.

    1024 Hz with every sensor the board has fitted gives the race the most
    gathers to hit. Fails on lost or duplicate samples, jitter past a quarter
    period, a missing tail, or an inquiry that reports a layout other than the
    log's. A gap of the fail-safe's length is named as such. A clean log after
    fewer than MIN_INQUIRIES inquiries is not_covered: it could be luck.

    Logging cannot start docked, so the hub port is the dock, as in
    smoke.sd_log. The rate and sensors are stored configuration, so InfoMem is
    saved to disk and put back.
    """
    port = bt.spp_port(unit.mac)
    if not port:
        pytest.skip(f"no paired SPP port for {unit.mac}")
    with dock.open_when_ready(cdc) as d:
        original = bytes(d.read_infomem())
        card = d.get(dock.COMP_DAUGHTER_CARD, 0x02, bytes([3, 0]))[:3]
    want = INQUIRY_SENSORS - (s3r_boards.OPTIONAL - s3r_boards.fitted_optional(card))
    cfg = S3rConfig(original)
    cfg.divider = divider_for(INQUIRY_RATE_HZ)
    cfg.set_sensors(want)
    backup = throughput.save_infomem(unit.name, original)
    sent = 0
    moved = []                  # inquiries whose layout differs from the one the log started with
    error = None
    try:
        with dock.open_when_ready(cdc) as d:
            d.write_infomem(cfg.bytes())
            back = S3rConfig(d.read_infomem())
        assert (back.divider, back.enabled()) == (cfg.divider, want), \
            f"configuration read back divider {back.divider}, sensors {sorted(back.enabled())}; " \
            f"wrote {cfg.divider}, {sorted(want)}"
        root = s3r_usb.wait_drive(unit.usb_serial, 45)
        before = sdfile.session_dirs(root)
        power.off(unit.hub_port, unit.name)
        try:
            s3r_usb.wait_cdc_gone(unit.usb_serial, 15)
            with bt.BtLink(port) as link:
                inq = link.inquiry()
                link.simple(bt.START_LOGGING)
                link.wait_status(bt.STATUS_LOGGING, True, 10)
                end = time.monotonic() + INQUIRY_SECONDS
                while time.monotonic() < end:
                    again = link.inquiry()
                    sent += 1
                    if (again.divider, again.channels) != (inq.divider, inq.channels):
                        moved.append(sent)
                    time.sleep(INQUIRY_INTERVAL_S)
                link.simple(bt.STOP_LOGGING)
                link.wait_status(bt.STATUS_LOGGING, False, 15)
        finally:
            power.on(unit.hub_port, unit.name)
    except BaseException as e:
        error = e
        raise
    finally:
        _restore_infomem(unit, original, backup, error)

    root = s3r_usb.wait_drive(unit.usb_serial, 90)
    new = sorted(sdfile.session_dirs(root) - before)      # relative: the drive letter may have changed
    assert len(new) == 1, f"expected one new session directory, found {len(new)}: {new}"
    files = sorted(p for p in (root / new[0]).iterdir() if p.is_file())
    assert files and files[0].name == "000", f"{new[0]}: no data file 000 ({[f.name for f in files]})"

    fc = sdfile.check(files[0])
    h = fc.header
    covered_s = (fc.span_ticks + h.divider) / bt.TICKS_PER_S
    summary = f"{sent} inquiries during {covered_s:.1f} s of logging at {cfg.rate_hz:.0f} Hz, {len(h.channels)} channels"
    problems = []
    if h.divider != cfg.divider:
        problems.append(f"header divider {h.divider}, configured {cfg.divider}")
    if h.channels != inq.channels:
        problems.append(f"header channels {h.channels}, inquiry said {inq.channels}")
    problems += _timing_problems(fc, h.divider)
    steps = {d: n for d, n in sorted(fc.deltas.items()) if d > 1.5 * h.divider}
    if steps:
        failsafe = (_failsafe_loss(h.divider) + 1) * h.divider
        problems.append("steps of " + ", ".join(f"{d} ticks x {n}" for d, n in steps.items()) +
                        (f"; {failsafe} ticks is the packet ring's 100 ms fail-safe releasing a gather the "
                         f"inquiry reply's rebuild broke (DEV-1363)" if failsafe in steps else ""))
    if moved:
        problems.append(f"{len(moved)} of {sent} inquiries reported a layout other than the log's "
                        f"(the first was number {moved[0]})")
    if covered_s < INQUIRY_SECONDS - LOG_TAIL_S:
        problems.append(f"the records cover {covered_s:.2f} s of a log over {INQUIRY_SECONDS:.0f} s long: "
                        "the end is missing")
    assert not problems, f"{summary}: " + "; ".join(problems)
    if sent < MIN_INQUIRIES:
        pytest.skip(f"{summary}: no loss, but fewer than {MIN_INQUIRIES} inquiries cannot show the race is gone")
    hil_note(f"{summary}: {fc.records} records, no loss, {fc.jitter} late stamps (max {fc.max_jitter_ticks} ticks)")
