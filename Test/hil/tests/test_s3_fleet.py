"""Shimmer3 fleet tier: one unit per radio firmware that shipped to customers.

The radio firmware is the variable that matters on Shimmer3 (TEST_PROCEDURE
§5.1): the firmware detects the module from its banner, picks a baud rate and
gates features on the version, and a bring-up failure halts the unit outright.
So every candidate is run on one unit of each shipped radio, all docked in a
Base6 and reached one slot at a time.

The shipped set is RN42 v4.77, RN42 v6.15, RN4678 v1.00.5 and RN4678 v1.23.
A unit whose banner is not in it fails boot_identity: either the bench has
drifted from what customers have, or the firmware misreads the module.
"""

from __future__ import annotations

import re
import time

import pytest

from rig import bt, dock, throughput
from rig.base6 import Base6

pytestmark = pytest.mark.fleet

SHIPPED_RADIOS = {("RN42", "4.77"), ("RN42", "6.15"), ("RN4678", "1.00.5"), ("RN4678", "1.23")}
STREAM_SECONDS = 10.0
SELF_SET_BITS = [(230, 0x01)]   # flagWriteCfgToSd, set on every config write by design


def masked(im: bytes) -> bytes:
    """InfoMem (or its first bytes) with the bits the firmware sets itself cleared."""
    b = bytearray(im)
    for off, mask in SELF_SET_BITS:
        if off < len(b):
            b[off] &= ~mask & 0xFF
    return bytes(b)


def parse_radio(banner: str) -> tuple[str, str] | None:
    """('RN42', '6.15') etc. from a GET_BT_VERSION banner, or None if unrecognised.

    RN4678: 'RN4678 V1.23 06/30/2021 ...'. RN41/RN42: 'Ver 4.77 RN-42 ...' or
    'Ver 6.15 04/26/2013 ...' - v6.x names no module, but only the RN42 ran it.
    """
    m = re.match(r"RN4678 V(\d+\.\d+(?:\.\d+)?)", banner)
    if m:
        return "RN4678", m.group(1)
    m = re.match(r"Ver (\d+\.\d+)", banner)
    if m:
        module = "RN41" if "RN-41" in banner else "RN42"
        return module, m.group(1)
    return None


@pytest.fixture(scope="session")
def base6(rig):
    with Base6(rig.base6_serial) as b:
        yield b


@pytest.fixture
def docklink(base6, unit):
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        yield d


def open_bt(unit, attempts: int = 2, wait_s: float = 10.0) -> bt.BtLink:
    """Connect, retrying once: a classic link right after another unit's
    stream can be slow to page on this kind of bench, and that is a bench
    effect, not the unit's."""
    port = bt.spp_port(unit.mac)
    if not port:
        pytest.skip(f"no paired SPP port for {unit.mac} - pair its classic side once in Windows")
    last = None
    held = False
    for i in range(attempts + 1):
        try:
            return bt.BtLink(port, connect_timeout_s=20)
        except Exception as e:      # serial.SerialException or BtError
            last = e
            # "Access is denied" means another handle has the COM port open -
            # seen transiently right after Windows pairing activity. That is a
            # fact about the PC, not the unit, so it is not a firmware failure.
            held = "Access is denied" in str(e) or isinstance(getattr(e, "__context__", None), PermissionError)
            time.sleep(wait_s * (2 if held else 1))
    if held:
        pytest.skip(f"{port} is held open by another process ({last}) - not tested")
    pytest.fail(f"{port}: no Bluetooth connection after {attempts + 1} attempts ({last})")


@pytest.mark.hil("fleet.boot_identity", needs=["shimmer3", "dock"])
def test_boot_identity(unit, docklink, candidate, hil_note):
    v = docklink.version()
    assert v.hw == 3, f"hardware version {v.hw}, expected 3 (Shimmer3)"
    assert v.fw_id == 3, f"firmware identifier {v.fw_id}, expected 3 (LogAndStream)"
    if candidate:
        want = candidate["firmware_version"].lstrip("vV")
        assert v.text == want, f"unit runs v{v.text}, candidate is v{want} - was it flashed?"
    if unit.mac:
        assert docklink.mac() == unit.mac, "MAC differs from rig.toml - wrong unit in this slot?"
    banner = docklink.bt_version()
    radio = parse_radio(banner)
    assert radio, f"radio banner not recognised: {banner!r}"
    assert radio in SHIPPED_RADIOS, f"{radio[0]} v{radio[1]} never shipped to customers ({banner!r})"
    hil_note(f"v{v.text}, {radio[0]} v{radio[1]}")


@pytest.mark.hil("fleet.config_roundtrip", needs=["shimmer3", "dock"])
def test_config_roundtrip(docklink):
    original = docklink.read_infomem()
    docklink.write_infomem(original)
    back = docklink.read_infomem()
    diff = [i for i, (a, b) in enumerate(zip(masked(original), masked(back))) if a != b]
    assert not diff, f"unchanged config read back different at offsets {diff}"


STREAM_DIVIDER = 640    # 51.2 Hz: well inside every shipped radio's capacity
RESTORE_ATTEMPTS = 3


def restore_infomem(base6, original: bytes, stream_error: BaseException | None,
                    backup=None) -> list[str]:
    """Write the unit's own InfoMem (from offset 0) back; return what each failed attempt saw.

    Runs in a finally block, so a failure here must not replace the stream's
    own exception: when both fail, the DockError names the stream's error and
    is chained from it.
    """
    seen = []
    for _ in range(RESTORE_ATTEMPTS):
        try:
            with dock.open_when_ready(base6.uart_port) as d:
                d.write_infomem(original, offset=0)
                if masked(d.read_infomem(0, len(original))) != masked(original):
                    raise dock.DockError("read back differs")
            return seen
        except (dock.DockError, OSError) as e:   # serial.SerialException is an OSError
            seen.append(str(e))
            time.sleep(1.0)
    msg = f"could not restore the configuration over the dock: {seen}"
    if backup is not None:
        msg += f"; the original is in {backup}"
    if stream_error is not None:
        raise dock.DockError(f"{msg} - after the stream itself failed with {stream_error!r}") from stream_error
    raise dock.DockError(msg)


@pytest.mark.hil("fleet.bt_stream", needs=["shimmer3", "bt", "dock"])
def test_bt_stream(unit, base6, hil_note):
    """Stream at a rate the test chooses, with the unit's own channel set.

    The unit's stored rate is whatever its last user left - two bench units
    came configured for 1024 Hz with ten or eleven channels and lost about
    half their samples over classic Bluetooth - so the rate is set over the
    dock first and restored afterwards. What each radio sustains at its
    ceiling is a separate, later item.
    """
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        original = d.read_infomem(0, 2)
        d.write_infomem(STREAM_DIVIDER.to_bytes(2, "little"), offset=0)
    stream_error = None
    try:
        with open_bt(unit) as link:
            assert link.hw == 3
            inq = link.inquiry()
            assert inq.divider == STREAM_DIVIDER, f"inquiry reports divider {inq.divider} after setting it"
            st = link.stream(STREAM_SECONDS, inq)
            # A stop the firmware did not act on leaves the unit sensing, and
            # a sensing Shimmer3 turns its dock UART off: name that here
            # rather than as a dock timeout in the restore below.
            link.wait_status(bt.STATUS_SENSING, False, timeout_s=5)
    except BaseException as e:
        stream_error = e
        raise
    finally:
        retries = restore_infomem(base6, original, stream_error)
        if retries:
            hil_note(f"divider restore needed {len(retries)} retries: {'; '.join(retries)}")
    expected = STREAM_SECONDS * inq.rate_hz
    assert st.packets > 0.8 * expected, f"{st.packets} packets in {st.seconds:.1f} s, expected ~{expected:.0f}"
    assert st.resyncs == 0, f"{st.resyncs} bytes did not frame as packets"
    assert st.gaps == 0, f"{st.gaps} gaps, {st.missing} samples lost, largest {st.max_gap_ticks} ticks"
    assert st.dupes == 0, f"{st.dupes} duplicate or backwards stamps"
    assert st.max_jitter_ticks <= 0.25 * inq.divider, f"stamp jitter up to {st.max_jitter_ticks} ticks"
    hil_note(f"{st.packets} packets at {inq.rate_hz:.1f} Hz, {len(inq.channels)} channels, no loss")


LINK_SECONDS = 10.0
LOAD_SECONDS = 20.0


@pytest.mark.hil("fleet.bt_throughput", needs=["shimmer3", "bt", "dock"])
def test_bt_throughput(rig, unit, base6, hil_note):
    """What the radio carries, and how much of a stream that outruns it arrives (§5.5).

    rig/throughput.py has the two measurements. Both are judged against the
    figures the last release gave on this unit and bench, from rig.toml; a
    unit without them is reported not_covered with the figures to record.
    The load is stored configuration, so the whole InfoMem is put back.
    """
    baseline = rig.throughput_baseline(unit)
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        original = bytes(d.read_infomem())
    backup = throughput.save_infomem(unit.name, original)
    error = None
    try:
        with open_bt(unit) as link:
            rate = throughput.data_rate_test(link, LINK_SECONDS)
            st, inq = throughput.load_stream(link, LOAD_SECONDS)
    except BaseException as e:
        error = e
        raise
    finally:
        retries = restore_infomem(base6, original, error, backup)
        if retries:
            hil_note(f"configuration restore needed {len(retries)} retries: {'; '.join(retries)}")
    problems = throughput.defects(rate, st)
    assert not problems, "; ".join(problems)
    measured = throughput.figures(rate, st, inq)
    if not baseline:
        pytest.skip(f"no throughput baseline for {unit.name} in rig.toml; measured {throughput.as_toml(measured)}")
    short, note = throughput.judge(measured, baseline, unit.name)
    hil_note(note)
    assert not short, "; ".join(short)


REBOOTS = 5


@pytest.mark.hil("fleet.reboot", needs=["shimmer3", "dock"])
def test_reboot(unit, base6, hil_note):
    """Reset the unit repeatedly; every boot must bring the radio up and answer.

    On Shimmer3 a radio that does not answer at any rung of the baud ladder
    halts the boot for good (TEST_PROCEDURE §5.1), so an intermittent
    bring-up shows here as a unit that stops answering the dock.
    """
    from rig.s3_flash import reset_slot
    banners = set()
    for i in range(REBOOTS):
        reset_slot(base6, unit.base6_slot)
        t0 = time.monotonic()
        try:
            with dock.open_when_ready(base6.uart_port, attempts=30, delay_s=0.5) as d:
                d.version()
                banners.add(d.bt_version())
        except dock.DockError as e:
            pytest.fail(f"boot {i + 1} of {REBOOTS}: no answer on the dock ({e})")
    assert len(banners) == 1, f"radio banner changed between boots: {banners}"
    hil_note(f"{REBOOTS} resets, each answered; last boot answered in {time.monotonic() - t0:.1f} s")
