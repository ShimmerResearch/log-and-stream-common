"""Checks for open firmware defects: one report item per Jira ticket.

Each test reproduces one filed defect and fails while the defect is there.
They are marked `defect`, which no release tier selects (run_rc.py runs smoke,
fleet, release and soak), so a known failure here never blocks a release.
Once a fix lands the check passes on the fixed build. That is the time to move
it into its platform's tier, and into required/ if it should gate releases, in
the same PR as the fix.

    python -m pytest Test/hil/tests/test_defects.py -m defect -rA
    python -m pytest Test/hil/tests/test_defects.py -m defect -k dev1297 --unit s3-rn42-615

What a check changes on a unit it puts back before the next one starts: the
whole InfoMem over the dock, and the calibration dump over Bluetooth, from
copies taken first and also kept on disk (the `restored` fixture). The
read-only checks touch neither.

The Shimmer3s sit in the Base6, so nothing here logs to SD: a docked Shimmer3
refuses a log start, and a Base6 cannot undock one slot.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from pathlib import Path

import pytest

from rig import bt, dock, dock_protocol, midstream, s3r_usb, throughput
from rig.s3_config import S3Config, calib_records
from rig.s3_flash import reset_slot, restore_infomem

pytestmark = pytest.mark.defect

RESET_SETTLE_S = 4.0            # radio bring-up after a slot reset, as rig/btmode.py waits


# --- shared ------------------------------------------------------------------

def _open_bt(unit, base6=None, attempts: int = 2, wait_s: float = 10.0) -> bt.BtLink:
    """Connect as test_s3_release._open_bt does: retry what is the PC's, and
    reset through the Base6 a unit that takes the link but answers nothing
    (DEV-1136), so the checks after it still have a unit to talk to."""
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


@dataclass
class Saved:
    infomem: bytes              # 512 bytes, over the dock
    dump: bytes                 # the calibration dump, over Bluetooth
    backup: Path


@pytest.fixture
def restored(unit, base6):
    """A Shimmer3's InfoMem and calibration dump, put back after the test.

    The dump first, as test_s3_release.saved does: writing it pushes its
    blocks into InfoMem (CALIBRATION §8.3), which the dock write after it then
    puts back. Twice, because a unit silent over Bluetooth is reset by the
    first try."""
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        infomem = bytes(d.read_infomem())
    backup = throughput.save_infomem(unit.name, infomem)
    with _open_bt(unit, base6) as link:
        dump = link.read_calib_dump()
    dump_backup = backup.with_name(backup.stem + "_calib_dump.bin")
    dump_backup.write_bytes(dump)
    yield Saved(infomem, dump, backup)

    problem = None
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
        pytest.fail(f"{unit.name}: calibration dump not restored ({problem}); the original is in {dump_backup}")


def _configure_over_dock(unit, base6, change) -> S3Config:
    """Apply change(cfg) to the unit's configuration bytes 0-9 (rate, sensors,
    setup bytes), write just those over the dock and return what the unit
    stored. A dock write leaves the calibration dump alone; a Bluetooth
    InfoMem write would re-stamp it."""
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        cfg = S3Config(d.read_infomem(0, bt.INFOMEM_CONFIG_BYTES))
        change(cfg)
        d.write_infomem(cfg.bytes()[:10], offset=0)
        return S3Config(d.read_infomem(0, bt.INFOMEM_CONFIG_BYTES))


def _stop_and_check(link: bt.BtLink) -> str | None:
    """After a stream: None if the unit answers and has stopped sensing, else
    what went wrong (the silent link is closed by the caller, and a fresh one
    checked). A sensing Shimmer3 turns its dock UART off, so a stop not acted
    on is named here rather than as a dock timeout in the restore."""
    try:
        link.status()                             # also flushes a stale packet
    except bt.BtError:
        return "the unit was silent after the stop"
    link.wait_status(bt.STATUS_SENSING, False, timeout_s=5)
    return None


# --- DEV-1279: dock requests whose LEN does not match their arguments ----------

@pytest.mark.hil("defect.dev1279_dock_arg_lengths", needs=["shimmer3", "dock"])
def test_dev1279_dock_arg_lengths_s3(unit, base6, hil_note):
    """GET INFOMEM, CARD_ID and CARD_MEM with no arguments, one byte short and
    one byte over must each get BAD_ARG (rig/dock_protocol.arg_lengths).
    Reads only."""
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        w = dock_protocol.Walk(d, dock_protocol.S3)
        dock_protocol.arg_lengths(w)
    hil_note(f"Base6 UART: {w.counts['error']} requests")
    assert not w.failures, "; ".join(w.failures)


@pytest.mark.hil("defect.dev1279_dock_arg_lengths", needs=["shimmer3r", "usb"])
def test_dev1279_dock_arg_lengths_s3r(unit, hil_note):
    """The same requests over a Shimmer3R's USB CDC port."""
    cdc = s3r_usb.wait_usb_settled(unit.usb_serial, 60)
    with dock.open_when_ready(cdc) as d:
        w = dock_protocol.Walk(d, dock_protocol.S3R)
        dock_protocol.arg_lengths(w)
    hil_note(f"USB CDC: {w.counts['error']} requests")
    assert not w.failures, "; ".join(w.failures)


# --- DEV-985: the LSM303 magnetometer's default calibration seeds --------------

LSM303_MAG_ID = 32                          # SC_SENSOR_LSM303_MAG (CALIBRATION §4.1)
LSM303_MAG_RANGES = range(1, 8)             # gain codes ±1.3 to ±8.1 Ga; 0 is not one


@pytest.mark.hil("defect.dev985_mag_seed_ranges", needs=["shimmer3", "bt", "dock"])
def test_dev985_mag_seed_ranges(unit, base6, hil_note):
    """The calibration dump must hold an LSM303 mag record for every gain code,
    1 to 7, so a range with no calibration of its own still has a default.

    The seed loop runs 0-6 (ShimCalib_setDefaultLsm303MagCalib), so ±8.1 Ga
    (code 7) has none, and GET_MAG_CALIBRATION at that range reads 21 zeros.
    Reads only: a docked Shimmer3 builds its dump at boot from those seeds and
    InfoMem, so what is read is what the seeds made.
    """
    with _open_bt(unit, base6) as link:
        cfg = S3Config(link.read_infomem())
        records = calib_records(link.read_calib_dump())
    have = sorted(r for (sid, r) in records if sid == LSM303_MAG_ID)
    missing = [r for r in LSM303_MAG_RANGES if r not in have]
    in_use = cfg.get("mag_range")
    hil_note(f"LSM303 mag records for ranges {have}; range in use {in_use}")
    note = (f" Range {in_use} is in use, so its record may be InfoMem's own block rather than a seed."
            if in_use in LSM303_MAG_RANGES else "")
    assert not missing, (f"no LSM303 mag record for gain code(s) {missing} (records: {have}); "
                         f"a unit set to that range reads an all-zero calibration.{note}")


# --- DEV-1327: the mag rate set by command and by InfoMem ----------------------

MAG_RATE_220HZ = 7                          # LSM303DLHC_MAG_220HZ: 220 Hz on an LSM303DLHC


@pytest.mark.hil("defect.dev1327_mag_rate_220hz", needs=["shimmer3", "bt", "dock"])
def test_dev1327_mag_rate_command_matches_infomem(unit, base6, restored, hil_note):
    """Mag rate code 7 stored the same way whichever route sets it.

    SET_MAG_SAMPLING_RATE clamps 7 to 6 (75 Hz) in ShimConfig_configByteMagRateSet,
    while an InfoMem write stores 7, which Consensys writes and the release
    matrix relies on above 75 Hz (rig/s3_config.py MAG_FAST). The ticket asks
    whether 220 Hz is allowed. Either answer passes this check, if both routes
    give it: 7 from both, or 6 from both.
    """
    with _open_bt(unit, base6) as link:
        link.set_setting("mag_rate", MAG_RATE_220HZ)
        by_command = link.get_setting("mag_rate")
    stored = _configure_over_dock(unit, base6, lambda c: c.put("mag_rate", MAG_RATE_220HZ))
    with _open_bt(unit, base6) as link:
        by_infomem = link.get_setting("mag_rate")
    hil_note(f"mag rate 7 by command reads {by_command}; by InfoMem reads {by_infomem} "
             f"(stored {stored.get('mag_rate')})")
    assert by_command == by_infomem, (f"mag rate code 7: SET_MAG_SAMPLING_RATE stores {by_command}, "
                                      f"an InfoMem write stores {by_infomem}")


# --- DEV-996: the alt accel and alt mag calibration, written and read back ------

ALT_BLOCKS = {"alt_accel": 133, "alt_mag": 154}     # NV_ALT_ACCEL/MAG_CALIBRATION: InfoMem offsets


def _test_block(current: bytes, n: int) -> bytes:
    """A plausible 21-byte block (CALIBRATION §3: big-endian bias and
    sensitivity, then an int8 alignment matrix) that differs from `current`."""
    block = struct.pack(">3h3h9b", 11 + n, -22 - n, 33 + n, 1234, 1235 + n, 1236,
                        100, 0, 0, 0, 100, 0, 0, 0, 100)
    return block if block != current else struct.pack(">3h", 44, 55, 66) + block[6:]


@pytest.mark.hil("defect.dev996_alt_calibration", needs=["shimmer3", "bt", "dock"])
def test_dev996_alt_calibration_reads_back(unit, base6, restored, hil_note):
    """SET_ALT_ACCEL_CALIBRATION / SET_ALT_MAG_CALIBRATION, then the matching GET,
    must return the block just written.

    On Shimmer3 the SET reaches InfoMem and the SD header but not the
    calibration dump (ShimCalib_configBytesToCalibDump has no case for ids 33
    and 34), and the GET answers from the dump, so it returns 21 zeros or an
    older record. InfoMem is read over the dock as well, to say which side is
    wrong.
    """
    written, got = {}, {}
    with _open_bt(unit, base6) as link:
        for n, name in enumerate(ALT_BLOCKS):
            written[name] = _test_block(link.get_calibration(name), n)
            link.set_calibration(name, written[name])
            got[name] = link.get_calibration(name)
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        im = d.read_infomem()
    problems, notes = [], []
    for name, off in ALT_BLOCKS.items():
        in_infomem = im[off:off + bt.CALIB_BLOCK] == written[name]
        notes.append(f"{name}: GET {'matches' if got[name] == written[name] else 'differs'}, "
                     f"InfoMem {'holds it' if in_infomem else 'does not'}")
        if got[name] != written[name]:
            what = "21 zero bytes" if not any(got[name]) else got[name].hex()
            problems.append(f"{name}: wrote {written[name].hex()}, GET returned {what}"
                            f" (InfoMem {'holds the block' if in_infomem else 'does not hold it either'})")
    hil_note("; ".join(notes))
    assert not problems, "; ".join(problems)


# --- DEV-1297: a battery or status read answered mid-stream --------------------

READS_DIVIDER = 128                         # 256 Hz, LN accel only, 1-byte CRC: the Java HIL's setup
READS_CRC = 1
READS_STREAM_S = 60.0
READS_LEAD_S = 2.0
READ_EVERY_S = 0.5                          # a battery read, then a status read a quarter period later


def reads_plan(seconds: float) -> list[midstream.Action]:
    out, t, i = [], READS_LEAD_S, 1
    while t + READ_EVERY_S < seconds:
        out.append(midstream.Action(t, "vbatt", i, bytes([bt.GET_VBATT])))
        out.append(midstream.Action(t + READ_EVERY_S / 2, "status", i, bytes([bt.GET_STATUS])))
        t += READ_EVERY_S
        i += 1
    return out


def losses_by_reply(gaps: list[tuple[int, int]], instream: list[tuple[int, int]]) -> dict:
    """Samples lost next to each kind of in-stream reply, and elsewhere.

    A reply framed after packet n-1 sits where sample n would be; the defect
    is that sample never being sent, so the gap shows at packet n. One packet
    either side is allowed for the order the radio delivers them in.
    """
    out = {bt.VBATT_RSP: 0, bt.STATUS_RSP: 0, None: 0}
    for idx, lost in gaps:
        op = next((op for op, n in instream if n - 1 <= idx <= n + 1), None)
        out[op if op in out else None] += lost
    return out


@pytest.mark.hil("defect.dev1297_reads_midstream", needs=["shimmer3", "bt", "dock"])
def test_dev1297_reads_midstream(unit, base6, restored, hil_note):
    """Battery and status reads answered while streaming must cost no samples.

    A minute at 256 Hz with a battery read every half second and a status read
    between them: about 115 of each. The Java HIL found a Shimmer3 skipping the
    sample at the reply for about 1 battery read in 12 and 1 status read in 20.
    Every gap is attributed to the reply next to it, if any.
    """
    def ln_only(c: S3Config):
        c.divider = READS_DIVIDER
        c.set_sensors({"ln_accel"})
    _configure_over_dock(unit, base6, ln_only)
    plan = reads_plan(READS_STREAM_S)
    with _open_bt(unit, base6) as link:
        inq = link.inquiry()
        assert inq.divider == READS_DIVIDER, f"inquiry reports divider {inq.divider} after setting it"
        link.status_len()                        # asked with CRC off, so its reply has no CRC bytes
        link.set_crc(READS_CRC)
        st = link.stream(READS_STREAM_S, inq, keep=True, actions=plan)
        hung = _stop_and_check(link)
        if not hung:
            link.set_crc(0)
    if hung:
        with _open_bt(unit, base6) as again:
            again.wait_status(bt.STATUS_SENSING, False, timeout_s=15)

    sent = {k: sum(a.kind == k for a in plan) for k in ("vbatt", "status")}
    answered = {k: sum(op == rsp for op, _ in st.instream) for k, rsp in
                (("vbatt", bt.VBATT_RSP), ("status", bt.STATUS_RSP))}
    lost = losses_by_reply(bt.gap_positions(st.ts, inq.divider), st.instream)
    summary = (f"{st.packets} packets at {inq.rate_hz:.0f} Hz; battery reads {answered['vbatt']}/{sent['vbatt']} "
               f"answered, {lost[bt.VBATT_RSP]} samples lost next to them; status reads "
               f"{answered['status']}/{sent['status']}, {lost[bt.STATUS_RSP]} lost; {lost[None]} lost elsewhere")
    hil_note(summary)
    problems = []
    if st.resyncs or st.crc_errors:
        problems.append(f"{st.resyncs} framing resyncs, {st.crc_errors} CRC errors (first: {st.misframes[:1]})")
    if st.gaps:
        problems.append(f"{st.gaps} gaps, {st.missing} samples lost")
    # Every battery read has one reply. Status frames can outnumber the reads:
    # the firmware also pushes one of its own when its state changes.
    if answered["vbatt"] != sent["vbatt"] or answered["status"] < sent["status"]:
        problems.append(f"replies {answered}, reads sent {sent}")
    if hung:
        problems.append(hung)
    assert not problems, f"{summary}: " + "; ".join(problems)
