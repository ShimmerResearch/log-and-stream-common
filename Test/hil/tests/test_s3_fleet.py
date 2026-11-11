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
import struct
import time

import pytest

from rig import ble, bt, dock, dock_protocol, midstream, throughput
from rig.s3_config import MAG_DLHC, S3Config, calib_records
from rig.s3_flash import restore_infomem

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


@pytest.mark.hil("fleet.dock_protocol", needs=["shimmer3", "dock"])
def test_dock_protocol(unit, docklink, candidate, hil_note):
    """Every dock command over the Base6's UART, each error reply and the framing
    rules (rig/dock_protocol.py). What it writes it puts back; the clock is set to the PC's."""
    version = candidate["firmware_version"].lstrip("vV") if candidate else None
    note, failures = dock_protocol.walk(docklink, dock_protocol.S3, "Base6 UART", mac=unit.mac, version=version)
    hil_note(note)
    assert not failures, "; ".join(failures)


ALT_BLOCKS = {"alt_accel": 133, "alt_mag": 154}     # NV_ALT_ACCEL/MAG_CALIBRATION: InfoMem offsets


def alt_test_block(current: bytes, n: int) -> bytes:
    """A plausible 21-byte block (CALIBRATION §3: big-endian bias and
    sensitivity, then an int8 alignment matrix) that differs from `current`."""
    block = struct.pack(">3h3h9b", 11 + n, -22 - n, 33 + n, 1234, 1235 + n, 1236,
                        100, 0, 0, 0, 100, 0, 0, 0, 100)
    return block if block != current else struct.pack(">3h", 44, 55, 66) + block[6:]


@pytest.mark.hil("fleet.alt_calibration", needs=["shimmer3", "bt", "dock"])
def test_alt_calibration(unit, base6, hil_note):
    """SET_ALT_ACCEL_CALIBRATION / SET_ALT_MAG_CALIBRATION, then the matching GET,
    return the block just written.

    The GET answers from the calibration dump, under the alt accel's range and
    range 0 for the alt mag. Until DEV-996 the SET reached InfoMem and the SD
    header but never the dump, so the GET answered 21 zeros. InfoMem is read
    over the dock as well, to say which side is wrong.

    What it writes it puts back: the calibration dump over Bluetooth, then the
    whole InfoMem over the dock, from copies taken first and kept on disk.
    """
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        original = bytes(d.read_infomem())
    backup = throughput.save_infomem(unit.name, original)
    written, got, error = {}, {}, None
    try:
        with open_bt(unit) as link:
            dump = link.read_calib_dump()
            backup.with_name(backup.stem + "_calib_dump.bin").write_bytes(dump)
            for n, name in enumerate(ALT_BLOCKS):
                written[name] = alt_test_block(link.get_calibration(name), n)
                link.set_calibration(name, written[name])
                got[name] = link.get_calibration(name)
            # Put the dump back first: writing it pushes its blocks into InfoMem
            # (CALIBRATION §8.3), which the InfoMem restore below then undoes.
            if link.read_calib_dump() != dump:
                link.write_calib_dump(dump)
                assert link.read_calib_dump() == dump, "calibration dump not restored: read back differs"
        base6.select(unit.base6_slot)
        with dock.open_when_ready(base6.uart_port) as d:
            im = d.read_infomem()
    except BaseException as e:
        error = e
        raise
    finally:
        retries = restore_infomem(base6, original, error, backup)
        if retries:
            hil_note(f"InfoMem restore needed {len(retries)} retries: {'; '.join(retries)}")
    problems, notes = [], []
    for name, off in ALT_BLOCKS.items():
        in_infomem = im[off:off + bt.CALIB_BLOCK] == written[name]
        notes.append(f"{name}: GET {'matches' if got[name] == written[name] else 'differs'}, "
                     f"InfoMem {'holds it' if in_infomem else 'does not'}")
        if got[name] != written[name]:
            what = "21 zero bytes" if not any(got[name]) else got[name].hex()
            problems.append(f"{name}: wrote {written[name].hex()}, GET returned {what}"
                            f" (InfoMem {'holds the block' if in_infomem else 'does not hold it either'})")
        elif not in_infomem:
            problems.append(f"{name}: the GET answers the block, but InfoMem does not hold it")
    hil_note("; ".join(notes))
    assert not problems, "; ".join(problems)


# The LSM303's default alignment, x100, shared by its accel and mag
# (CALIBRATION §6.1). The LSM303AH's X and Y are the LSM303DLHC's swapped and
# negated: a mirror image of it, determinant -1.
_DLHC_ALIGN = (-100, 0, 0, 0, 100, 0, 0, 0, -100)
_AH_ALIGN = (0, -100, 0, 100, 0, 0, 0, 0, -100)

# The LSM303 mag's default calibration (CALIBRATION §4.1): range 0 is the
# LSM303AH, 1 to 7 the LSM303DLHC's gain codes. (X/Y, Z) sensitivity in
# LSB/gauss and the alignment; the bias is 0.
LSM303_MAG_ID = 32                          # SC_SENSOR_LSM303_MAG
LSM303_MAG_SEEDS = {
    0: ((667, 667), _AH_ALIGN),             # LSM303AH: 1.5 mgauss/LSB
    1: ((1100, 980), _DLHC_ALIGN),          # ±1.3 gauss
    2: ((855, 760), _DLHC_ALIGN),
    3: ((670, 600), _DLHC_ALIGN),
    4: ((450, 400), _DLHC_ALIGN),
    5: ((400, 355), _DLHC_ALIGN),
    6: ((330, 295), _DLHC_ALIGN),
    7: ((230, 205), _DLHC_ALIGN),           # ±8.1 gauss
}
MAG_CHANNELS = {0x07, 0x08, 0x09}           # mag X, Y, Z in an inquiry's channel list
BOOT_SETTLE_S = 4.0     # radio bring-up after a slot reset, as rig/btmode.py waits


def mag_seed_block(rng: int) -> bytes:
    (sxy, sz), align = LSM303_MAG_SEEDS[rng]
    return struct.pack(">3h3h9b", 0, 0, 0, sxy, sxy, sz, *align)


# What firmware up to v1.01.009 seeded at mag range 0: range 7's values.
OLD_MAG_RANGE0 = mag_seed_block(7)


def describe_mag_block(block: bytes) -> str:
    sens = struct.unpack(">3h", block[6:12])
    align = struct.unpack("9b", block[12:21])
    return f"sens {sens} align {align}"


@pytest.fixture
def booted_docked(unit, base6):
    """Reset the unit through its slot first, so that its calibration dump is
    the one a docked boot builds: from the seeds and InfoMem, after the
    firmware's repairs. A unit that last booted undocked holds the dump it
    read from its SD card instead, and docking does not rebuild it."""
    from rig.s3_flash import reset_slot
    reset_slot(base6, unit.base6_slot)
    with dock.open_when_ready(base6.uart_port, attempts=30, delay_s=0.5) as d:
        d.version()
    time.sleep(BOOT_SETTLE_S)


@pytest.mark.hil("fleet.mag_seeds", needs=["shimmer3", "bt", "dock"])
def test_mag_seeds(unit, booted_docked, hil_note):
    """The calibration dump holds a default LSM303 mag record for every range,
    0 to 7, and each one that is still a default holds its part's values.

    Up to v1.01.009 the seed loop ran 0-6 against gain codes 1-7 (DEV-985,
    DEV-995): range 7 (±8.1 gauss) had no record, so GET_MAG_CALIBRATION
    there read 21 zeros, and range 0, the LSM303AH's, carried ±8.1 gauss's
    230/230/205 and the LSM303DLHC's alignment, so an LSM303AH read about 3x
    high.

    Writes nothing, after a reset through the slot (booted_docked). A docked
    Shimmer3 builds its dump at boot from the seeds and InfoMem, so a record
    under a zero timestamp is a seed, except the range in use, where InfoMem's
    own block replaces it. That one, and any record with a timestamp, is
    someone's calibration and is not compared. One thing InfoMem's block may
    not be is the old range-0 default, which the boot replaces whatever its
    timestamp: on an LSM303AH board, where range 0 is always in use, that is
    the check on the repair.
    """
    with open_bt(unit) as link:
        cfg = S3Config(link.read_infomem())
        records = calib_records(link.read_calib_dump())
    in_use = cfg.get("mag_range")
    have = sorted(r for (sid, r) in records if sid == LSM303_MAG_ID)
    missing = [r for r in LSM303_MAG_SEEDS if r not in have]
    wrong, compared = [], []
    for rng in LSM303_MAG_SEEDS:
        rec = records.get((LSM303_MAG_ID, rng))
        if rec is None:
            continue
        block = bytes(rec.data[:21])
        if rng == in_use == 0:
            compared.append("0 (in use: not the old default)")
            if block == OLD_MAG_RANGE0:
                wrong.append(f"range 0, in use, still holds the default up to v1.01.009 gave it "
                             f"({describe_mag_block(block)}): the boot did not replace it")
            continue
        if rng == in_use or any(rec.ts):
            continue
        compared.append(str(rng))
        want = mag_seed_block(rng)
        if block != want:
            wrong.append(f"range {rng}: {describe_mag_block(block)}, "
                         f"expected {describe_mag_block(want)}")
    hil_note(f"LSM303 mag records for ranges {have}; range {in_use} in use; "
             f"defaults compared for ranges {compared}")
    problems = []
    if missing:
        problems.append(f"no LSM303 mag record for range(s) {missing}: a unit set to one reads an "
                        f"all-zero calibration")
    problems += wrong
    assert not problems, "; ".join(problems)


LSM303_ACCEL_ID = 31                        # SC_SENSOR_LSM303_ACCEL
# The wide-range accel's range code is the part's own full-scale field value,
# so the two parts number their full scales differently (CALIBRATION §6.1).
LSM303_ACCEL_SEEDS = {
    "LSM303DLHC": ({0: 1631, 1: 815, 2: 408, 3: 135}, _DLHC_ALIGN),  # ±2/4/8/16 g
    "LSM303AH": ({0: 1671, 1: 209, 2: 836, 3: 418}, _AH_ALIGN),     # ±2/16/4/8 g
}


def accel_seed_block(part: str, rng: int) -> bytes:
    sens, align = LSM303_ACCEL_SEEDS[part]
    return struct.pack(">3h3h9b", 0, 0, 0, sens[rng], sens[rng], sens[rng], *align)


def lsm303_part(cfg: S3Config, channels: list[int]) -> tuple[str, str, bool]:
    """Which LSM303 the firmware found at boot: (part, how that was told,
    whether that is proof). "LSM303AH" includes an ICM-20948 standing in for
    one, which the seed treats the same way.

    A host cannot ask for the part (STREAMING_DATA_FORMAT §5.2), but two
    things show it. With the mag enabled, the inquiry lists its channels X,
    Z, Y on an LSM303DLHC board and X, Y, Z on any other (§4.1). And a mag
    range other than 0 means an LSM303DLHC, because the firmware moves it to
    0 on any other board. Range 0 alone proves nothing: it is no LSM303DLHC
    gain, but nothing stops a host writing it to one.
    """
    mag = tuple(c for c in channels if c in MAG_CHANNELS)
    if mag:
        return ("LSM303DLHC" if mag == MAG_DLHC else "LSM303AH"), "the inquiry's mag channel order", True
    mag_range = cfg.get("mag_range")
    if mag_range != 0:
        return "LSM303DLHC", f"mag range {mag_range}", True
    return "LSM303AH", "mag range 0, with the mag off", False


@pytest.mark.hil("fleet.wr_accel_seeds", needs=["shimmer3", "bt", "dock"])
def test_wr_accel_seeds(unit, booted_docked, hil_note):
    """The calibration dump holds a default LSM303 wide-range accel record for
    every range, 0 to 3, and each one that is still a default holds the fitted
    part's values.

    Up to v1.01.009 every board got the LSM303DLHC's records. On an LSM303AH
    board, whose codes run ±2/16/4/8 g, that read 2.4% high at ±2 g and 2 to 4
    times out at the others, with X and Y swapped.

    The part is the one the firmware found at boot, as lsm303_part tells it.
    Writes nothing, after a reset through the slot (booted_docked). As in
    fleet.mag_seeds, the range in use and any record with a timestamp may be
    someone's calibration and are not compared.

    The reset matters here. Docked, the firmware builds the dump from the seed
    and InfoMem without reading the SD card, so this checks the seed even on a
    unit updated from v1.01.009, whose old records stay in InfoMem (range in
    use) and on the card: nothing repairs them (CALIBRATION §6.1). A unit that
    last booted undocked would hold the card's records instead.
    """
    with open_bt(unit) as link:
        cfg = S3Config(link.read_infomem())
        channels = list(link.inquiry().channels)
        records = calib_records(link.read_calib_dump())
    part, told_by, proven = lsm303_part(cfg, channels)
    in_use = cfg.get("wr_accel_range")
    have = sorted(r for (sid, r) in records if sid == LSM303_ACCEL_ID)
    missing = [r for r in range(4) if r not in have]
    wrong, compared = [], []
    for rng in range(4):
        rec = records.get((LSM303_ACCEL_ID, rng))
        if rec is None or rng == in_use or any(rec.ts):
            continue
        compared.append(rng)
        want = accel_seed_block(part, rng)
        if bytes(rec.data[:21]) != want:
            wrong.append(f"range {rng}: {describe_mag_block(bytes(rec.data))}, "
                         f"expected the {part}'s {describe_mag_block(want)}")
    hil_note(f"{part} board, from {told_by}; LSM303 accel records for ranges "
             f"{have}; range {in_use} in use; defaults compared for ranges {compared}")
    problems = []
    if missing:
        problems.append(f"no LSM303 accel record for range(s) {missing}")
    problems += wrong
    if wrong and not proven:
        problems.append(f"the part was inferred from {told_by}: if this is an LSM303DLHC board set to "
                        f"mag range 0, enable the mag and repeat, and the inquiry will settle it")
    assert not problems, "; ".join(problems)


STREAM_DIVIDER = 640    # 51.2 Hz: well inside every shipped radio's capacity


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
    problems = throughput.defects(rate, st, inq)
    assert not problems, "; ".join(problems)
    measured = throughput.figures(rate, st, inq)
    if not baseline:
        pytest.skip(f"no throughput baseline for {unit.name} in rig.toml; measured {throughput.as_toml(measured)}")
    short, note = throughput.judge(measured, baseline, unit.name)
    hil_note(note)
    assert not short, "; ".join(short)


BLE_SETTLE_S = 6.0


@pytest.mark.hil("fleet.ble_throughput", needs=["shimmer3", "bt", "ble", "dock"])
def test_ble_throughput(rig, unit, base6, hil_note):
    """fleet.bt_throughput's two measurements over BLE, on a Shimmer3 with an RN4678 (§5.5).

    The link is the RN4678's transparent UART service, data as notifications
    (rig/ble.py). An RN41 or RN42 has no BLE side, so its units do not have the
    "ble" capability and are reported not_covered. The figures are judged
    against [ble_throughput_by_radio] and the unit's ble_throughput; without
    any the item is not_covered, with the figures to record. A unit whose
    EEPROM has BLE off (bleEnabled) is not_covered too, and a dock that cannot
    read that page fails the item. The load is stored configuration, so the
    whole InfoMem is put back.

    The measurement waits BLE_SETTLE_S after connecting. Windows renegotiates
    the connection for its first few seconds (60 ms, 15 ms, then 60 ms about
    2.6 s in), and a flood started inside that window ran at 15 ms. That
    triples an RN4678's rate, and it is also where the modules failed on the
    bench: a v1.00.5 went silent until reset (supervision timeout) twice in
    two tries, and a v1.23 stopped taking connections. Waiting gives every run
    the same interval, the one a host settles on (TEST_PROCEDURE §5.5).
    """
    profile = ble.profile_for(unit.radio)
    baseline = rig.ble_throughput_baseline(unit)
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        radio = d.radio_settings()
        if not dock.ble_enabled(radio):
            pytest.skip(f"{unit.name}: BLE is off in its EEPROM (radio settings byte 2 = {radio[2]:#04x}, "
                        "bleEnabled clear)")
        original = bytes(d.read_infomem())
    backup = throughput.save_infomem(unit.name, original)
    error = None
    try:
        ser = ble.BleSerial(unit.mac, profile=profile)
        try:
            time.sleep(BLE_SETTLE_S)
        except BaseException:
            ser.close()
            raise
        with bt.BtLink(ser=ser) as link:
            hil_note(link.ser.describe())
            rate = throughput.data_rate_test(link, LINK_SECONDS)
            st, inq = throughput.load_stream(link, LOAD_SECONDS)
    except BaseException as e:
        error = e
        raise
    finally:
        retries = restore_infomem(base6, original, error, backup)
        if retries:
            hil_note(f"configuration restore needed {len(retries)} retries: {'; '.join(retries)}")
    problems = throughput.defects(rate, st, inq)
    assert not problems, "; ".join(problems)
    measured = throughput.figures(rate, st, inq)
    if not baseline:
        pytest.skip(f"no BLE throughput baseline for {unit.name} in rig.toml; "
                    f"measured {throughput.as_toml(measured, 'ble_throughput')}")
    short, note = throughput.judge(measured, baseline, unit.name)
    hil_note(note)
    # A PC's Bluetooth stack can stick in a state that holds BLE at about 60%
    # until Bluetooth is switched off and on (TEST_PROCEDURE §5.5).
    assert not short, "; ".join(short) + (" - switch the PC's Bluetooth off and on and repeat before "
                                         "believing a BLE shortfall")


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


MIDSTREAM_CYCLES = 6
MIDSTREAM_CRC = 2


@pytest.mark.hil("fleet.sd_start_midstream", needs=["shimmer3", "bt", "dock"])
def test_sd_start_midstream(unit, base6, hil_note):
    """smoke.sd_start_midstream's commands, sent to a docked Shimmer3.

    A Shimmer3 starts logging only undocked (ShimSens_checkStartLoggingConditions),
    and a Base6 cannot undock one slot. Docked, the log starts are ACKed and
    refused, but the clock sets still apply mid-stream, so the stream is still
    judged as smoke.sd_start_midstream judges it:
    - a disturbed stream fails the item;
    - a clean one is not_covered, because no log started;
    - if a status shows logging after all, the item is judged in full.
    The stream runs at 51.2 Hz, as fleet.bt_stream does, and the rate is put back.
    """
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        original = d.read_infomem(0, 2)
        d.write_infomem(STREAM_DIVIDER.to_bytes(2, "little"), offset=0)
    plan = midstream.schedule(MIDSTREAM_CYCLES)
    error = None
    try:
        with open_bt(unit) as link:
            link.command(midstream.SET_RWC, midstream.rwc_payload()[1:])
            inq = link.inquiry()
            assert inq.divider == STREAM_DIVIDER, f"inquiry reports divider {inq.divider} after setting it"
            link.status_len()            # asked with CRC off, so its reply has no CRC bytes
            link.set_crc(MIDSTREAM_CRC)
            st = link.stream(plan.seconds, inq, keep=True, actions=plan.actions)
            # A sensing Shimmer3 turns its dock UART off: name a stop not acted
            # on here, not as a dock timeout in the restore.
            link.wait_status(bt.STATUS_SENSING, False, timeout_s=5)
            link.set_crc(0)
    except BaseException as e:
        error = e
        raise
    finally:
        retries = restore_infomem(base6, original, error)
        if retries:
            hil_note(f"divider restore needed {len(retries)} retries: {'; '.join(retries)}")
    problems, summary = midstream.judge(st, plan, inq.divider)
    assert not problems, f"{summary}: " + "; ".join(problems)
    if not midstream.logging_seen(st):
        pytest.skip(f"docked in a Base6, which cannot undock one slot, so no log started: the unit "
                    f"refuses a log start while docked. Clean across the commands: {summary}")
    hil_note(summary)
