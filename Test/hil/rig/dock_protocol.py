"""The dock protocol checked command by command: the walk behind smoke.dock_protocol
(Shimmer3R, over USB CDC) and fleet.dock_protocol (Shimmer3, over the Base6's UART).

Every GET and SET in docs/SHIMMER3_DOCK_PROTOCOL.md Appendix A, the error replies
the parser and the dispatcher give (§2.1, Appendix A), and the framing rules of
§2.1. After every error reply a GET of the version must be answered, so the parser
has to come back from each one.

Whatever is written is put back. The clock is set to the PC's time, which is what
any host does when a unit docks. Everything else is rewritten with the unit's own
bytes, and a write that ought to be refused is given the unit's own bytes too, so
that a firmware which wrongly accepts one changes nothing.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from typing import Callable

import serial

from rig import dock
from rig.dock import (BAD_ARG, BAD_CMD, BAD_CRC, CMD_GET, CMD_SET, COMP_BAT, COMP_BLUETOOTH,
                      COMP_D_ACCEL, COMP_DAUGHTER_CARD, COMP_GSR, COMP_SHIMMER, COMP_TEST, PROP_BAT_VALUE,
                      PROP_BT_VER, PROP_CARD_ID, PROP_CARD_MEM, PROP_CURR_TIME, PROP_ENTER_BOOTLOADER,
                      PROP_INFOMEM, PROP_LED0_STATE, PROP_MAC, PROP_RWC_CFG_TIME, PROP_VER, TICKS_PER_S, frame)

S3R, S3 = "shimmer3r", "shimmer3"
HW_VERSION = {S3R: 10, S3: 3}
FW_ID_LOGANDSTREAM = 3

# InfoMem bits the firmware changes by itself: flagWriteCfgToSd on every config
# write (byte 230 bit 0), and rtcSetByBt (byte 217 bit 7), which setting the clock
# over the dock clears by design (SHIMMER3_DOCK_PROTOCOL.md Appendix A).
FLAG_WRITE_CFG_TO_SD = (230, 0x01)
RTC_SET_BY_BT = (217, 0x80)

CLOCK_TOLERANCE_S = 0.1     # host timing of a request is good to a few ms either way
LONG_GAP_S = 0.25           # well past the parser's 100 ms inter-byte timeout
SHORT_GAP_S = 0.02
BOOTLOADER_ARM_S = 5        # armed, then cancelled; the wait outlasts it


def _masked(im: bytes, bits) -> bytes:
    b = bytearray(im)
    for off, mask in bits:
        if off < len(b):
            b[off] &= ~mask & 0xFF
    return bytes(b)


def _diff(a: bytes, b: bytes) -> list[int]:
    return [i for i, (x, y) in enumerate(zip(a, b)) if x != y] + list(range(min(len(a), len(b)), max(len(a), len(b))))


def _card_name(page: bytes) -> str:
    return "unprogrammed" if page[0] == 0xFF else f"SR{page[0]}-{page[1]}-{page[2]}"


@dataclass
class Walk:
    """One unit's walk: what failed, what was counted, and what it found out."""
    link: dock.DockLink
    platform: str
    failures: list[str] = field(default_factory=list)
    got: set = field(default_factory=set)       # GET properties read and checked
    wrote: set = field(default_factory=set)     # SETs written and read back
    counts: dict = field(default_factory=lambda: {"error": 0, "framing": 0})
    facts: list[str] = field(default_factory=list)

    def fail(self, msg: str) -> None:
        self.failures.append(msg)

    def check(self, ok: bool, msg: str) -> bool:
        if not ok:
            self.fail(msg)
        return ok

    def alive(self) -> bool:
        """The parser came back: a GET of the version is answered."""
        try:
            r = self.link.request(CMD_GET, COMP_SHIMMER, PROP_VER)
        except (dock.DockError, serial.SerialException) as e:
            self.fail(f"no answer after that ({e})")
            return False
        return self.check(r.code == dock.CMD_RSP and r.prop == PROP_VER, f"VER after that gave {r.name}")

    def expect(self, label: str, raw: bytes, code: int) -> None:
        """Send `raw`; it must get exactly `code`, and the parser must come back."""
        self.counts["error"] += 1
        try:
            r = self.link.exchange(raw)
        except dock.DockError as e:
            self.fail(f"{label}: {e}")
        else:
            self.check(r.code == code, f"{label}: {r.name}, expected {dock.CODE_NAMES[code]}")
        if not self.alive():
            self.fail(f"(after {label})")


# --- the sections -------------------------------------------------------------

def gets(w: Walk, mac: str | None, version: str | None) -> dict:
    """Every GET: lengths and shapes against the doc, values against rig.toml and
    the candidate. Returns what the later sections need."""
    d = w.link
    seen = {}

    w.got.add("VER")
    raw = d.get(COMP_SHIMMER, PROP_VER)
    prodtest = False
    if w.check(len(raw) == 7, f"VER gave {len(raw)} bytes, expected 7"):
        hw, fw_id, major, minor, patch = struct.unpack("<BHHBB", raw)
        text = f"{major}.{minor:02d}.{patch:03d}"
        w.check(hw == HW_VERSION[w.platform], f"VER hardware {hw}, expected {HW_VERSION[w.platform]}")
        w.check(fw_id == FW_ID_LOGANDSTREAM, f"VER firmware identifier {fw_id}, expected {FW_ID_LOGANDSTREAM}")
        if version:
            w.check(text == version, f"VER says v{text}, candidate is v{version}")
        prodtest = has_production_test_cmds(w.platform, (major, minor, patch))
    seen["prodtest"] = prodtest

    w.got.add("MAC")
    raw = d.get(COMP_SHIMMER, PROP_MAC)
    if w.check(len(raw) == 6, f"MAC gave {len(raw)} bytes, expected 6") and mac:
        w.check(raw.hex().upper() == mac.upper(), f"MAC {raw.hex().upper()}, rig.toml says {mac}")

    w.got.add("RWC_CFG_TIME")
    raw = d.get(COMP_SHIMMER, PROP_RWC_CFG_TIME)
    w.check(len(raw) == 8, f"RWC_CFG_TIME gave {len(raw)} bytes, expected 8")

    # The clock must run at its own rate: two reads half a second apart.
    w.got.add("CURR_LOCAL_TIME")
    h0 = time.monotonic()
    t0 = d.current_ticks()
    time.sleep(0.5)
    t1 = d.current_ticks()
    host = time.monotonic() - h0
    unit = (t1 - t0) / TICKS_PER_S
    w.check(abs(unit - host) < CLOCK_TOLERANCE_S,
            f"CURR_LOCAL_TIME advanced {unit:.3f} s while {host:.3f} s passed")
    seen["unit_ticks"] = t1

    w.got.add("BAT")
    raw = d.get(COMP_BAT, PROP_BAT_VALUE)
    if w.check(len(raw) == 3, f"BAT gave {len(raw)} bytes, expected 3"):
        adc = raw[0] | (raw[1] << 8)
        w.check(adc <= 0x0FFF, f"BAT ADC value {adc} is beyond 12 bits")

    w.got.add("BT VER")
    raw = d.get(COMP_BLUETOOTH, PROP_BT_VER)
    # The module's own banner: ASCII, and on an RN42 more than one line.
    text = raw.rstrip(b"\x00\r\n ")
    w.check(bool(text) and all(32 <= b < 127 or b in b"\r\n\t" for b in text), f"BT VER gave {raw!r}")

    # The expansion-board ID page, whole and in pieces: offset and length are honoured.
    w.got.add("CARD_ID")
    page = d.card_id()
    if w.check(len(page) == dock.CARD_ID_SIZE, f"CARD_ID gave {len(page)} bytes, expected 16"):
        w.check(d.card_id(3, 0) == page[:3], "CARD_ID (3 at 0) differs from the page")
        w.check(d.card_id(1, 15) == page[15:], "CARD_ID (1 at 15) differs from the page")
        w.check(d.card_id(0, 0) == b"", "CARD_ID of length 0 returned data")
        w.facts.append(f"card {_card_name(page)}")
    seen["card_id"] = page

    # All of the EEPROM CARD_MEM reaches, and the radio page within it.
    w.got.add("CARD_MEM")
    eeprom = d.read_card_mem(0, dock.CARD_MEM_SIZE)
    if w.check(len(eeprom) == dock.CARD_MEM_SIZE, f"CARD_MEM read {len(eeprom)} of {dock.CARD_MEM_SIZE} bytes"):
        radio = dock.EEPROM_RADIO_PAGE - dock.CARD_MEM_BASE
        w.check(d.radio_settings() == eeprom[radio:radio + dock.RADIO_PAGE_SIZE],
                "CARD_MEM's radio page read alone differs from the whole read")
        w.check(d.read_card_mem(dock.CARD_MEM_SIZE - 1, 1) == eeprom[-1:], "CARD_MEM's last byte differs")
    seen["eeprom"] = eeprom

    # InfoMem whole, then again in chunks that straddle the 128-byte ones.
    w.got.add("INFOMEM")
    im = d.read_infomem()
    again = d.read_infomem(chunk=100)
    if w.check(len(im) == dock.INFOMEM_SIZE, f"INFOMEM read {len(im)} of {dock.INFOMEM_SIZE} bytes"):
        w.check(again == im, f"INFOMEM read in 100-byte chunks differs at {_diff(im, again)[:8]}")
        w.check(d.read_infomem(dock.INFOMEM_SIZE - 1, 1) == im[-1:], "INFOMEM's last byte differs")
        w.check(d.get(COMP_SHIMMER, PROP_INFOMEM, struct.pack("<BH", 0, 0)) == b"",
                "INFOMEM of length 0 returned data")
    seen["infomem"] = im

    if prodtest:
        production_test_gets(w)
    return seen


def has_production_test_cmds(platform: str, version: tuple) -> bool:
    """Whether the firmware has the production-test commands: boot report, button
    state, LED_SET and the power-profile record (dock.PRODUCTION_TEST_CMDS_SINCE)."""
    return platform == S3R and tuple(version) >= dock.PRODUCTION_TEST_CMDS_SINCE


def production_test_gets(w: Walk) -> None:
    """The production-test GETs. The power-profile record is only read: a generic
    walk never arms it (an armed unit runs the sequence at its next boot), and it
    must find it not armed."""
    d = w.link

    w.got.add("BOOT_REPORT")
    raw = d.boot_report()
    if w.check(len(raw) == dock.BOOT_REPORT_LEN, f"BOOT_REPORT gave {len(raw)} bytes, expected {dock.BOOT_REPORT_LEN}"):
        w.check(raw[0] == 1, f"BOOT_REPORT layout version {raw[0]}, expected 1")

    w.got.add("BUTTON")
    raw = d.get(COMP_SHIMMER, dock.PROP_BUTTON)
    if w.check(len(raw) == dock.BUTTON_STATE_LEN, f"BUTTON gave {len(raw)} bytes, expected {dock.BUTTON_STATE_LEN}"):
        w.check(raw[0] in (0, 1), f"BUTTON level {raw[0]}, expected 0 or 1")

    w.got.add("POWER_PROFILE")
    raw = d.power_profile()
    if w.check(dock.PP_READBACK_HEADER_LEN <= len(raw) <= dock.PP_READBACK_MAX_LEN,
               f"POWER_PROFILE gave {len(raw)} bytes, expected 17-128"):
        n = raw[16]
        w.check(raw[0] == 1, f"POWER_PROFILE layout version {raw[0]}, expected 1")
        w.check(len(raw) == dock.PP_READBACK_HEADER_LEN + 3 * n,
                f"POWER_PROFILE gave {len(raw)} bytes for {n} steps")
        w.check(raw[1] not in (dock.PP_STATE_ARMED, dock.PP_STATE_RUNNING),
                f"the power-profile record is {'armed' if raw[1] == dock.PP_STATE_ARMED else 'running'}"
                " - it would run at the next boot")


def errors(w: Walk, seen: dict) -> None:
    """Each error reply, from the parser (sent as a bare prefix: it answers at
    once and does not wait for the rest) and from the dispatcher."""
    G, S = CMD_GET, CMD_SET
    im, page, eeprom = seen["infomem"], seen["card_id"], seen["eeprom"]
    prodtest = seen.get("prodtest", False)
    past_last = dock.FACTORY_TEST_COUNT if prodtest else dock.FACTORY_TEST_COUNT_BEFORE_POWER_PROFILE

    def bad_crc(f: bytes) -> bytes:
        return f[:-2] + bytes([f[-2] ^ 0xFF, f[-1]])

    # The parser: an unknown command byte, or a LEN of 0 or past the buffer.
    w.expect("command byte 0x05", b"$\x05", BAD_CMD)
    w.expect("command byte 0x02 (a response code)", b"$\x02", BAD_CMD)
    w.expect("LEN 0", b"$\x03\x00", BAD_ARG)
    w.expect("LEN 134", b"$\x03\x86", BAD_ARG)

    # A damaged frame, a GET and a SET (the SET carries InfoMem's own first byte).
    w.expect("GET VER, bad CRC", bad_crc(frame(G, COMP_SHIMMER, PROP_VER)), BAD_CRC)
    w.expect("SET INFOMEM, bad CRC",
             bad_crc(frame(S, COMP_SHIMMER, PROP_INFOMEM, struct.pack("<BH", 1, 0) + im[:1])), BAD_CRC)

    # Components and properties that are not handled.
    unknown = [
        (G, COMP_D_ACCEL, 0x02, "GET D_ACCEL DATA_RATE (declared, not handled)"),
        (G, COMP_GSR, 0x00, "GET GSR (no live properties)"),
        (G, 0x7F, 0x00, "GET component 0x7F"),
        (G, COMP_SHIMMER, 0x00, "GET SHIMMER ENABLE"),
        (G, COMP_SHIMMER, PROP_LED0_STATE, "GET SHIMMER LED0_STATE (ShimmerGQ only)"),
        (G, COMP_SHIMMER, PROP_ENTER_BOOTLOADER, "GET ENTER_BOOTLOADER"),
        (G, COMP_BAT, 0x03, "GET BAT 0x03"),
        (G, COMP_DAUGHTER_CARD, 0x04, "GET DAUGHTER_CARD 0x04"),
        (G, COMP_BLUETOOTH, 0x02, "GET BT 0x02"),
        (G, COMP_TEST, 0x00, "GET TEST"),
        (S, COMP_SHIMMER, PROP_MAC, "SET MAC"),
        (S, COMP_SHIMMER, PROP_VER, "SET VER"),
        (S, COMP_SHIMMER, PROP_CURR_TIME, "SET CURR_LOCAL_TIME"),
        (S, COMP_BAT, PROP_BAT_VALUE, "SET BAT VALUE"),
        (S, COMP_BLUETOOTH, PROP_BT_VER, "SET BT VER"),
        (S, COMP_TEST, past_last, f"SET TEST suite {past_last} (past the last)"),
        (S, COMP_TEST, 0x7F, "SET TEST suite 0x7F"),
        (S, 0x7F, 0x00, "SET component 0x7F"),
    ]
    if not prodtest:
        # Shimmer3 and older Shimmer3R firmware have none of the production-test
        # commands. SET TEST 4 is the suite past their last, above.
        unknown += [
            (G, COMP_SHIMMER, dock.PROP_BOOT_REPORT, "GET BOOT_REPORT (newer Shimmer3R only)"),
            (G, COMP_SHIMMER, dock.PROP_BUTTON, "GET BUTTON (newer Shimmer3R only)"),
            (S, COMP_SHIMMER, dock.PROP_LED_SET, "SET LED_SET (newer Shimmer3R only)"),
            (G, COMP_TEST, dock.FACTORY_TEST_POWER_PROFILE, "GET TEST 4 (newer Shimmer3R only)"),
        ]
    for cmd, comp, prop, label in unknown:
        w.expect(label, frame(cmd, comp, prop), BAD_CMD)
    if prodtest:
        for prop, label in [(dock.PROP_BOOT_REPORT, "BOOT_REPORT"), (dock.PROP_BUTTON, "BUTTON")]:
            w.expect(f"GET {label} with a byte too many", frame(G, COMP_SHIMMER, prop, b"\x00"), BAD_ARG)
        w.expect("GET TEST 4 with a byte too many",
                 frame(G, COMP_TEST, dock.FACTORY_TEST_POWER_PROFILE, b"\x00"), BAD_ARG)
        w.expect("SET LED_SET with one byte", frame(S, COMP_SHIMMER, dock.PROP_LED_SET, b"\x01"), BAD_ARG)
        w.expect("SET LED_SET with a mask past blue",
                 frame(S, COMP_SHIMMER, dock.PROP_LED_SET, b"\x08\x00"), BAD_ARG)
    if w.platform == S3:
        # Shimmer3R only. 0 seconds would cancel, so a Shimmer3 that wrongly took it still does nothing.
        w.expect("SET ENTER_BOOTLOADER on a Shimmer3", frame(S, COMP_SHIMMER, PROP_ENTER_BOOTLOADER, b"\x00"),
                 BAD_CMD)

    # Properties with a fixed LEN, given one byte too many (and the clock, one too few).
    for comp, prop, label in [(COMP_SHIMMER, PROP_MAC, "MAC"), (COMP_SHIMMER, PROP_VER, "VER"),
                              (COMP_SHIMMER, PROP_RWC_CFG_TIME, "RWC_CFG_TIME"),
                              (COMP_SHIMMER, PROP_CURR_TIME, "CURR_LOCAL_TIME"),
                              (COMP_BAT, PROP_BAT_VALUE, "BAT"), (COMP_BLUETOOTH, PROP_BT_VER, "BT VER")]:
        w.expect(f"GET {label} with a byte too many", frame(G, comp, prop, b"\x00"), BAD_ARG)
    w.expect("SET RWC_CFG_TIME with 7 bytes", frame(S, COMP_SHIMMER, PROP_RWC_CFG_TIME, bytes(7)), BAD_ARG)
    w.expect("SET RWC_CFG_TIME with 9 bytes", frame(S, COMP_SHIMMER, PROP_RWC_CFG_TIME, bytes(9)), BAD_ARG)

    # Lengths and offsets out of range: reads, then writes carrying the unit's own bytes.
    def rng(length, offset, wide=True):
        return struct.pack("<BH", length, offset) if wide else bytes([length, offset])
    for length, offset in [(129, 0), (1, 0x200), (2, 0x1FF)]:
        w.expect(f"GET INFOMEM {length} at {offset}", frame(G, COMP_SHIMMER, PROP_INFOMEM, rng(length, offset)),
                 BAD_ARG)
    for length, offset in [(17, 0), (1, 16), (2, 15)]:
        w.expect(f"GET CARD_ID {length} at {offset}",
                 frame(G, COMP_DAUGHTER_CARD, PROP_CARD_ID, rng(length, offset, wide=False)), BAD_ARG)
    for length, offset in [(129, 0), (1, 2032), (2, 2031)]:
        w.expect(f"GET CARD_MEM {length} at {offset}",
                 frame(G, COMP_DAUGHTER_CARD, PROP_CARD_MEM, rng(length, offset)), BAD_ARG)
    w.expect("SET INFOMEM 1 at 512", frame(S, COMP_SHIMMER, PROP_INFOMEM, rng(1, 512) + im[-1:]), BAD_ARG)
    w.expect("SET INFOMEM 2 at 511", frame(S, COMP_SHIMMER, PROP_INFOMEM, rng(2, 511) + im[-1:] + b"\xFF"), BAD_ARG)
    w.expect("SET CARD_ID 2 at 15",
             frame(S, COMP_DAUGHTER_CARD, PROP_CARD_ID, rng(2, 15, wide=False) + page[15:] + eeprom[:1]), BAD_ARG)
    w.expect("SET CARD_MEM of length 0", frame(S, COMP_DAUGHTER_CARD, PROP_CARD_MEM, rng(0, 0)), BAD_ARG)
    w.expect("SET CARD_MEM 2 at 2031",
             frame(S, COMP_DAUGHTER_CARD, PROP_CARD_MEM, rng(2, 2031) + eeprom[-1:] + b"\xFF"), BAD_ARG)


def framing(w: Walk) -> None:
    """§2.1: junk before a frame is skipped, a frame may arrive in pieces, and a
    gap of more than 100 ms abandons the frame it falls in."""
    d = w.link
    ver = frame(CMD_GET, COMP_SHIMMER, PROP_VER)

    def answered(label: str) -> None:
        r = d.read_reply()
        w.check(r is not None and r.code == dock.CMD_RSP and r.prop == PROP_VER, f"{label}: {r.name if r else 'no reply'}")

    w.counts["framing"] += 1
    d.ser.reset_input_buffer()
    d.ser.write(b"\x00\x13\xA5\x7E" + ver)
    answered("junk before the '$'")

    w.counts["framing"] += 1
    d.ser.reset_input_buffer()
    d.ser.write(ver[:3])
    time.sleep(SHORT_GAP_S)
    d.ser.write(ver[3:])
    answered(f"a frame in two writes {SHORT_GAP_S * 1000:.0f} ms apart")

    # The rest of a frame cut off by a long gap is dropped byte by byte, and only
    # a '$' would start a new one, so pick a request whose tail has none.
    w.counts["framing"] += 1
    probe = next(f for f in (ver, frame(CMD_GET, COMP_SHIMMER, PROP_MAC), frame(CMD_GET, COMP_BAT, PROP_BAT_VALUE))
                 if dock.START not in f[3:])
    d.ser.reset_input_buffer()
    d.ser.write(probe[:3])
    time.sleep(LONG_GAP_S)
    d.ser.write(probe[3:])
    late = d.read_reply(timeout_s=0.5)
    w.check(late is None, f"a frame broken by a {LONG_GAP_S * 1000:.0f} ms gap was answered ({late.name if late else ''})")
    w.alive()

    # Two requests in one write. The protocol is one request at a time, so this
    # is recorded, not judged; what is judged is that each reply is well formed
    # and the parser comes back.
    w.counts["framing"] += 1
    d.ser.reset_input_buffer()
    d.ser.write(ver + frame(CMD_GET, COMP_SHIMMER, PROP_MAC))
    got = []
    while len(got) < 3:
        r = d.read_reply(timeout_s=0.5)
        if r is None:
            break
        got.append({PROP_VER: "VER", PROP_MAC: "MAC"}.get(r.prop, "?") if r.code == dock.CMD_RSP else r.name)
    w.facts.append(f"pipelined: {'+'.join(got) or 'nothing'}")
    w.alive()


def sets(w: Walk, seen: dict) -> None:
    """The writes, each read back: the clock, the board ID page and a page of
    EEPROM. InfoMem writes are smoke/fleet.config_roundtrip's."""
    d = w.link
    im = seen["infomem"]

    # The clock, set to the PC's time as a host does on docking. Setting it over
    # the dock clears rtcSetByBt and must leave the rest of InfoMem alone.
    w.wrote.add("RWC_CFG_TIME")
    before = d.current_ticks()
    now = int(time.time() * TICKS_PER_S)
    d.set_rwc(now)
    set_at = time.monotonic()
    w.facts.append(f"clock {(before - now) / TICKS_PER_S:+.1f} s")
    w.check(d.rwc_config_ticks() == now, "RWC_CFG_TIME does not read back the value just set")
    cur = d.current_ticks()
    drift = (cur - now) / TICKS_PER_S - (time.monotonic() - set_at)
    w.check(abs(drift) < CLOCK_TOLERANCE_S, f"after setting the clock it reads {drift:+.3f} s from the PC")
    after = d.read_infomem()
    w.check(not after[RTC_SET_BY_BT[0]] & RTC_SET_BY_BT[1], "setting the clock over the dock left rtcSetByBt set")
    ignore = [FLAG_WRITE_CFG_TO_SD, RTC_SET_BY_BT]
    changed = _diff(_masked(im, ignore), _masked(after, ignore))
    w.check(not changed, f"setting the clock changed InfoMem at {changed[:8]}")
    if _masked(after, [FLAG_WRITE_CFG_TO_SD]) != _masked(im, [FLAG_WRITE_CFG_TO_SD]):
        d.write_infomem(im)
        w.check(_masked(d.read_infomem(), [FLAG_WRITE_CFG_TO_SD]) == _masked(im, [FLAG_WRITE_CFG_TO_SD]),
                "InfoMem not restored after the clock set")

    # The board ID page, with its own bytes. An unprogrammed page (0xFF, also what
    # a failed read at boot leaves) is not written: it may not be what the chip holds.
    page = seen["card_id"]
    if page[0] != 0xFF:
        w.wrote.add("CARD_ID")
        d.write_card_id(page)
        w.check(d.card_id() == page, "CARD_ID does not read back after rewriting it")

    # Newer Shimmer3R: hold the LEDs, then hand them back.
    if seen.get("prodtest"):
        w.wrote.add("LED_SET")
        d.led_set(0x01, 0x04)
        d.led_set(0xFF, 0xFF)

    # A page of the general expansion-board storage (EEPROM 16-31), with its own bytes.
    w.wrote.add("CARD_MEM")
    region = seen["eeprom"][:16]
    d.write_card_mem(region, 0)
    w.check(d.read_card_mem(0, 16) == region, "CARD_MEM 0-15 does not read back after rewriting it")


def bootloader_cancel(w: Walk, recover: Callable[[], str] | None) -> None:
    """Shimmer3R: arm the reboot into ST's bootloader, cancel it with 0, and
    outwait it on the same port. If it fires anyway, `recover` restarts the
    application and says how."""
    d = w.link
    w.wrote.add("ENTER_BOOTLOADER")
    d.enter_bootloader(BOOTLOADER_ARM_S)
    time.sleep(0.3)
    refused = None
    for _ in range(2):              # armed now, so one retry before it fires
        try:
            d.enter_bootloader(0)
            refused = None
            break
        except (dock.DockError, serial.SerialException) as e:
            refused = e
    if refused:
        w.fail(f"ENTER_BOOTLOADER 0 was refused ({refused})")
    time.sleep(BOOTLOADER_ARM_S + 1.5)
    try:
        d.version()
    except (dock.DockError, serial.SerialException) as e:
        how = recover() if recover else "not recovered"
        w.fail(f"the reboot into the bootloader was not cancelled: the port went quiet ({e}); {how}")


def walk(link: dock.DockLink, platform: str, transport: str, mac: str | None = None,
         version: str | None = None, recover: Callable[[], str] | None = None) -> tuple[str, list[str]]:
    """Run every section; return the note for the report and the failures.

    A section that raises is recorded and the walk goes on, unless the unit
    stops answering, when there is nothing left to learn from it."""
    w = Walk(link, platform)
    seen: dict = {}
    steps = [("GETs", lambda: seen.update(gets(w, mac, version))),
             ("errors", lambda: errors(w, seen)),
             ("framing", lambda: framing(w)),
             ("SETs", lambda: sets(w, seen))]
    if platform == S3R:
        steps.append(("bootloader cancel", lambda: bootloader_cancel(w, recover)))
    for name, step in steps:
        if name != "GETs" and not seen:
            w.fail(f"{name}: not run, the GETs did not finish")
            continue
        try:
            step()
        except (dock.DockError, serial.SerialException, struct.error) as e:
            w.fail(f"{name}: {e}")
            time.sleep(0.15)                # let the parser time out whatever was half sent
            if not w.alive():
                w.fail("the unit stopped answering; the rest was not run")
                break
    c = w.counts
    # Short: a fleet item carries four units' notes in the report's 500 characters.
    note = (f"{transport}: {len(w.got)} GETs, {len(w.wrote)} SETs, {c['error']} errors, "
            f"{c['framing']} framing; " + "; ".join(w.facts))
    return note, w.failures
