"""Release tier: the event channel and configuration byte 232 (DEV-971).

The bench has no hand on the button and nothing on the TTL input, so these
check everything around them; the presses and edges themselves are a person's
(docs/SHIMMER3_TEST_PROCEDURE.md §5.12, the required item gate4.event_channel).
The event file a log writes is checked by the release matrix's logging rows
(test_s3r_release.py). Each item is its own report item:

release.evt_config         Byte 232 read back as INFOMEM §8.3 keeps it, for
                           every kind of value: kept, reserved, invalid, with
                           userButtonEnable held at the mode's equivalent and
                           singleTouchStart cleared. Shimmer3R over the USB dock,
                           writing the whole image; Shimmer3 over Bluetooth, one
                           byte at a time, as §8.3 tells a host to.
release.evt_bt_stream      The Bluetooth stream (BT protocol §7.15), with byte
                           232 set to 0x24 by a one-byte SET_INFOMEM: the
                           opt-out probe is ACKed and sends nothing; the opt-in
                           sends the initial state as one CRC-checked frame,
                           sequence 0; the status agrees and shows the TTL input
                           armed; nothing more arrives while nothing is
                           connected; the opt-out disarms; a second opt-in starts
                           at sequence 0 again; a new connection starts opted out.
release.evt_log_triggers   Shimmer3R, undocked by the hub port: 0x11 (logging
                           off) ACKs START_LOGGING and logs nothing; 0x09 starts a
                           log on the undock; 0x01 does not, but a host command
                           still does.
release.evt_session_number Shimmer3R: a session whose data folder alone is
                           deleted, as Consensys' Clear SD leaves it, keeps its
                           number: the next log is numbered past its event file.

A Shimmer3 logs only undocked and the Base6 cannot undock one, so the last two
are Shimmer3R only. Each unit's configuration is put back at the end.
"""

from __future__ import annotations

import shutil
import time
from pathlib import Path

import pytest

from rig import bt, dock, events, s3r_boards, s3r_usb, sdfile
from rig.s3_flash import restore_infomem

pytestmark = pytest.mark.release

# Values for release.evt_config: (byte 232 written, userButtonEnable written).
# Every kept value class from §8.3's table, legacy with either userButtonEnable,
# and each way a value is not kept. Expectations come from rig/events.py.
CONFIG_CASES = [
    (0x00, 1), (0x00, 0), (0x20, 0),
    (0x01, 0), (0x09, 1), (0x0D, 1), (0x11, 0), (0x02, 0), (0x0B, 0), (0x24, 0), (0x2C, 0), (0x15, 0),
    (0x80, 1), (0xFF, 1), (0x07, 1), (0x26, 1), (0x19, 1), (0x08, 0), (0x41, 1),
]
STREAM_MODE = 0x24              # both inputs recorded, logging by host command
OPT_IN_S = 1.5                  # the initial state follows the opt-in's ACK
QUIET_S = 10.0                  # nothing connected: nothing more may arrive
UNDOCK_START_S = 15.0           # BT up after the undock, then the log seen started
LOG_S = 5.0


# --- Shimmer3R ----------------------------------------------------------------

@pytest.fixture(scope="module")
def s3r_saved(unit):
    """The unit's InfoMem, put back afterwards, and its board id."""
    port = s3r_usb.wait_usb_settled(unit.usb_serial, 60)
    with s3r_usb.stall_named(unit.usb_serial), dock.open_when_ready(port) as d:
        image = d.read_infomem()
        card = d.get(dock.COMP_DAUGHTER_CARD, 0x02, bytes([3, 0]))[:3]
    yield image, s3r_boards.is_sr48_6_0(card)
    port = s3r_usb.wait_usb_settled(unit.usb_serial, 90)
    with s3r_usb.stall_named(unit.usb_serial), dock.open_when_ready(port) as d:
        d.write_infomem(image)


def _s3r_write(unit, base: bytes, value: int, ube: int, single_touch: int = 0) -> bytes:
    """Byte 232, userButtonEnable and singleTouchStart written over the dock into
    the saved image; the image read back."""
    im = bytearray(base)
    im[events.BYTE] = value
    im[events.UBE_BYTE] = (im[events.UBE_BYTE] & ~events.UBE_BIT & 0xFF) | (events.UBE_BIT if ube else 0)
    im[events.SINGLE_TOUCH_BYTE] = ((im[events.SINGLE_TOUCH_BYTE] & ~events.SINGLE_TOUCH_BIT & 0xFF)
                                   | (events.SINGLE_TOUCH_BIT if single_touch else 0))
    port = s3r_usb.wait_usb_settled(unit.usb_serial, 90)
    with s3r_usb.stall_named(unit.usb_serial), dock.open_when_ready(port) as d:
        d.write_infomem(bytes(im))
        return bytes(d.read_infomem())


def _judge(value: int, ube: int, back: tuple[int, int, int], sr48_6_0: bool) -> list[str]:
    """back = (byte 217, byte 218, byte 232) as read."""
    b217, b218, b232 = back
    kept = events.correct(value, sr48_6_0=sr48_6_0)
    held = events.ube_held(kept)
    want_ube = ube if held is None else held
    out = []
    if b232 != kept:
        out.append(f"{value:#04x}: read back {b232:#04x}, expected {kept:#04x}")
    if bool(b217 & events.UBE_BIT) != bool(want_ube):
        out.append(f"{value:#04x} with userButtonEnable {ube}: it reads {int(bool(b217 & events.UBE_BIT))}, "
                   f"expected {want_ube}")
    if b218 & events.SINGLE_TOUCH_BIT:
        out.append(f"{value:#04x}: singleTouchStart kept; no release builds single touch")
    return out


@pytest.mark.hil("release.evt_config", needs=["shimmer3r", "usb"])
def test_s3r_config(unit, s3r_saved, hil_note):
    base, sr48 = s3r_saved
    probs = []
    for value, ube in CONFIG_CASES:
        back = _s3r_write(unit, base, value, ube, single_touch=1)
        probs += _judge(value, ube, (back[events.UBE_BYTE], back[events.SINGLE_TOUCH_BYTE], back[events.BYTE]),
                        sr48)
    hil_note(f"{len(CONFIG_CASES)} values" + (", SR48-6-0 rules" if sr48 else ""))
    assert not probs, "; ".join(probs)


def _bt(unit) -> bt.BtLink:
    port = bt.spp_port(unit.mac)
    if not port:
        pytest.skip(f"no paired SPP port for {unit.mac} - pair its classic side once in Windows")
    return bt.BtLink(port, connect_timeout_s=20)


@pytest.mark.hil("release.evt_bt_stream", needs=["shimmer3r", "usb", "bt"])
def test_s3r_bt_stream(unit, s3r_saved, hil_note):
    _, sr48 = s3r_saved
    probs, note = _bt_stream(unit, events.correct(STREAM_MODE, sr48_6_0=sr48), ttl_must_be_low=True)
    hil_note(note)
    assert not probs, "; ".join(probs)


def _undocked(unit, power, port: str, act) -> str:
    """Undock by the hub port, run act(link) over Bluetooth, re-dock. Returns a
    note of what the link needed: a second connection, or a wait while the unit
    was silent opening a log (bt.BtLink.wait_status)."""
    notes = []
    power.off(unit.hub_port, unit.name)
    try:
        s3r_usb.wait_cdc_gone(unit.usb_serial, 15)
        try:
            link = bt.BtLink(port, connect_timeout_s=20)
        except bt.BtError as e:
            # Seen once in six undocked connects (2026-11-08, byte 232 0x11):
            # the link opened and the first exchange went unanswered.
            notes.append(f"connected a second time ({e})")
            time.sleep(3)
            link = bt.BtLink(port, connect_timeout_s=20)
        with link:
            act(link)
            if link.silent_s >= 1:
                notes.append(f"silent {link.silent_s:.1f} s while starting a log")
            # Never re-dock mid-log: the dock would take the card from a log
            # this test did not mean to leave running.
            if link.status() & bt.STATUS_LOGGING:
                link.simple(bt.STOP_LOGGING)
                link.wait_status(bt.STATUS_LOGGING, False, 15)
    finally:
        power.on(unit.hub_port, unit.name)
    return "; ".join(notes)


def _logging_within(link, seconds: float) -> bool:
    try:
        link.wait_status(bt.STATUS_LOGGING, True, seconds)
        return True
    except bt.BtError:
        return False


@pytest.mark.hil("release.evt_log_triggers", needs=["shimmer3r", "usb", "bt", "hub_port"])
def test_s3r_log_triggers(unit, power, s3r_saved, hil_note):
    base, _ = s3r_saved
    port = bt.spp_port(unit.mac)
    assert port, f"no paired SPP port for {unit.mac}"
    probs, notes = [], []

    def case(value: int, ube: int, check) -> int | None:
        """New sessions on the card after the undocked check, or None if the value did not take."""
        back = _s3r_write(unit, base, value, ube)
        if back[events.BYTE] != value:
            probs.append(f"{value:#04x}: read back {back[events.BYTE]:#04x}, not run")
            return None
        root = s3r_usb.wait_drive(unit.usb_serial, 60)
        before = sdfile.session_dirs(root)
        link_note = _undocked(unit, power, port, check)
        if link_note:
            notes.append(f"{value:#04x}: {link_note}")
        root = s3r_usb.wait_drive(unit.usb_serial, 90)
        return len(sdfile.session_dirs(root) - before)

    def logging_off(link):
        if _logging_within(link, 3):
            probs.append("0x11: logging started on the undock")
            return
        link.simple(bt.START_LOGGING)                  # ACKed: command() raises on a NACK
        if _logging_within(link, LOG_S):
            probs.append("0x11: START_LOGGING started a log with logging off")

    def undock_start(link):
        if not _logging_within(link, UNDOCK_START_S):
            probs.append(f"0x09: no log within {UNDOCK_START_S:.0f} s of the undock")
        else:
            time.sleep(LOG_S)

    def host_only(link):
        if _logging_within(link, 3):
            probs.append("0x01: logging started on the undock")
            return
        link.simple(bt.START_LOGGING)
        if not _logging_within(link, 10):
            probs.append("0x01: START_LOGGING did not start a log")
        else:
            time.sleep(LOG_S)

    for value, ube, check, want in ((0x11, 0, logging_off, 0), (0x09, 1, undock_start, 1),
                                    (0x01, 0, host_only, 1)):
        n = case(value, ube, check)
        if n is None:
            continue
        if n != want:
            probs.append(f"{value:#04x}: {n} new sessions on the card, expected {want}")
        notes.append(f"{value:#04x}: {n} new sessions")
    hil_note(", ".join(notes))
    assert not probs, "; ".join(probs)


@pytest.mark.hil("release.evt_session_number", needs=["shimmer3r", "usb", "bt", "hub_port"])
def test_s3r_session_number(unit, power, s3r_saved, hil_note):
    base, sr48 = s3r_saved
    if sr48:
        pytest.skip("SR48-6-0 records neither input, so it writes no event file to keep a number")
    mode = 0x05                                         # presses recorded: an event file every log
    back = _s3r_write(unit, base, mode, 1)
    assert back[events.BYTE] == mode, f"byte 232 read back {back[events.BYTE]:#04x}, wrote {mode:#04x}"
    port = bt.spp_port(unit.mac)
    assert port, f"no paired SPP port for {unit.mac}"
    notes = []

    def one_log() -> Path:
        root = s3r_usb.wait_drive(unit.usb_serial, 60)
        before = sdfile.session_dirs(root)

        def log(link):
            link.simple(bt.START_LOGGING)
            link.wait_status(bt.STATUS_LOGGING, True, 10)
            time.sleep(LOG_S)
        link_note = _undocked(unit, power, port, log)
        if link_note:
            notes.append(link_note)
        root = s3r_usb.wait_drive(unit.usb_serial, 90)
        new = sorted(sdfile.session_dirs(root) - before)
        assert len(new) == 1, f"expected one new SD session, found {new}"
        return root / new[0]

    first = one_log()
    root = first.parents[2]
    evdir = events.companion_path(first / "000", root).parent
    assert evdir.is_dir(), f"no event folder {evdir.relative_to(root).as_posix()} for the first log"
    shutil.rmtree(first)                                # data/ only, as Clear SD does
    time.sleep(2)
    second = one_log()
    n1, n2 = _session_number(first), _session_number(second)
    hil_note("; ".join([f"{first.name} then {second.name}"] + notes))
    assert second.parent.name == first.parent.name, f"the second log went to {second.parent.name}"
    assert n2 > n1, (f"the second log reused session {n2} although {evdir.relative_to(root).as_posix()} "
                     f"still holds number {n1}")


def _session_number(session: Path) -> int:
    return int(session.name.rsplit("-", 1)[1])


# --- Shimmer3 -----------------------------------------------------------------

@pytest.fixture(scope="module")
def s3_saved(unit, base6):
    """The unit's InfoMem, read and put back over the Base6 dock."""
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        infomem = bytes(d.read_infomem())
    yield infomem
    base6.select(unit.base6_slot)
    restore_infomem(base6, infomem, None)


@pytest.mark.hil("release.evt_config", needs=["shimmer3", "bt", "dock"])
def test_s3_config(unit, s3_saved, hil_note):
    probs = []
    with _bt(unit) as link:
        for value, ube in CONFIG_CASES:
            b217, b218, _ = events.read_bytes(link)
            # Byte 232 first: every write is corrected against the bytes
            # already stored, so userButtonEnable written before it would be
            # held to the previous value's mode.
            events.write_bytes(link, {
                events.BYTE: value,
                events.UBE_BYTE: (b217 & ~events.UBE_BIT & 0xFF) | (events.UBE_BIT if ube else 0),
                events.SINGLE_TOUCH_BYTE: b218 | events.SINGLE_TOUCH_BIT,
            })
            probs += _judge(value, ube, events.read_bytes(link), sr48_6_0=False)
    hil_note(f"{len(CONFIG_CASES)} values")
    assert not probs, "; ".join(probs)


@pytest.mark.hil("release.evt_bt_stream", needs=["shimmer3", "bt", "dock"])
def test_s3_bt_stream(unit, s3_saved, hil_note):
    # In a Base6 the input is on the dock's pin 10, whatever the Base6 does with
    # it (EVENT_CHANNEL §3.1), so its level is noted rather than judged.
    probs, note = _bt_stream(unit, STREAM_MODE, ttl_must_be_low=False)
    hil_note(note)
    assert not probs, "; ".join(probs)


# --- both ---------------------------------------------------------------------

def _bt_stream(unit, kept: int, ttl_must_be_low: bool) -> tuple[list[str], str]:
    want = events.enables(kept)
    ttl = bool(want & events.ENABLE_TTL)
    probs, notes = [], []
    with _bt(unit) as link:
        b217, _, _ = events.read_bytes(link)
        events.write_bytes(link, {events.BYTE: STREAM_MODE, events.UBE_BYTE: b217 | events.UBE_BIT})
        got = events.read_bytes(link)[2]
        if got != kept:
            return [f"byte 232 read back {got:#04x} after a one-byte SET_INFOMEM of {STREAM_MODE:#04x}, "
                    f"expected {kept:#04x}"], ""

        s = events.status(link)
        if s.enables != want:
            probs.append(f"status before: enables {s.enables:#04x}, expected {want:#04x}")
        if s.flags & (events.STATUS_STREAM_ON | events.STATUS_TTL_ARMED):
            probs.append(f"status before opting in: {s}")

        events.set_stream(link, False)                  # the probe: ACKed, changes nothing
        recs, bad, _ = events.listen(link, OPT_IN_S)
        if recs or bad:
            probs.append(f"the opt-out probe brought {len(recs)} frames and {bad} bad ones")

        for attempt in ("opt-in", "second opt-in"):
            events.set_stream(link, True)
            recs, bad, _ = events.listen(link, OPT_IN_S)
            probs += _initial_state(attempt, recs, bad, ttl, ttl_must_be_low)
            if recs and attempt == "opt-in":
                notes.append(f"initial state {recs[0]['state']:#04x}")
            s = events.status(link)
            if not s.flags & events.STATUS_STREAM_ON or s.next_seq != 1 or s.frames_lost or s.dropped:
                probs.append(f"status after the {attempt}: {s}")
            if ttl and not s.flags & events.STATUS_TTL_ARMED:
                probs.append(f"{attempt}: the TTL input is not armed for an opted-in host")
            if attempt == "opt-in":
                recs, bad, _ = events.listen(link, QUIET_S)
                if recs or bad:
                    probs.append(f"{len(recs)} frames ({bad} with a bad CRC) in {QUIET_S:.0f} s "
                                 "with nothing connected"
                                 + ("" if ttl_must_be_low else ": does the dock drive pin 10?"))
            events.set_stream(link, False)
            recs, bad, _ = events.listen(link, OPT_IN_S)
            s = events.status(link)
            if recs or bad or s.flags & (events.STATUS_STREAM_ON | events.STATUS_TTL_ARMED):
                probs.append(f"after the {attempt}'s opt-out: {len(recs)} frames, {s}")
        # Left opted in when the link drops: the next connection starts opted out.
        events.set_stream(link, True)
        events.listen(link, OPT_IN_S)
    time.sleep(2)
    with _bt(unit) as link:
        recs, bad, _ = events.listen(link, OPT_IN_S)
        s = events.status(link)
        if recs or bad or s.flags & (events.STATUS_STREAM_ON | events.STATUS_TTL_ARMED):
            probs.append(f"a new connection is not opted out: {len(recs)} frames, {s}")
    return probs, "; ".join(notes)


def _initial_state(what: str, recs: list[dict], bad: int, ttl: bool, ttl_must_be_low: bool) -> list[str]:
    if bad or len(recs) != 1:
        return [f"{what}: {len(recs)} frames and {bad} with a bad CRC, expected the initial state alone"]
    r = recs[0]
    out = []
    if r["type"] != events.TYPE_INIT or r["seq"] != 0 or r["dropped"]:
        out.append(f"{what}: first record type {r['type']:#04x} seq {r['seq']} dropped {r['dropped']}, "
                   "expected the initial state at sequence 0")
    if r["state"] & events.STATE_BUTTON:
        out.append(f"{what}: the button reads pressed")
    if ttl and ttl_must_be_low and r["state"] & events.STATE_TTL:
        out.append(f"{what}: the TTL input reads high with nothing connected")
    return out
