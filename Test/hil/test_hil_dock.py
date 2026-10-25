#!/usr/bin/env python3
"""Unit tests for rig.dock_protocol, the walk behind smoke.dock_protocol and
fleet.dock_protocol. Run: python Test/hil/test_hil_dock.py (no hardware).

The walk runs against FirmwareModel, a Python model of the receive state machine
and dispatcher in Comms/shimmer_dock_usart.c (docs/SHIMMER3_DOCK_PROTOCOL.md):
a faithful model must pass, and each deliberately broken one must fail with the
defect named. The model keeps its own clock, so the 100 ms inter-byte timeout,
the real-world clock and the reboot into the bootloader run without waiting.

No pyserial needed: where it is not installed (CI) a stand-in is put in its
place, as test_hil_timing.py does.
"""

import struct
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    import serial  # noqa: F401
except ImportError:
    _serial = types.ModuleType("serial")

    class SerialException(IOError):
        pass

    _serial.SerialException = SerialException
    _serial.Serial = object
    sys.modules["serial"] = _serial

from rig import dock, dock_protocol  # noqa: E402
from rig.dock import ACK, BAD_ARG, BAD_CMD, BAD_CRC, START  # noqa: E402

TICKS = dock.TICKS_PER_S


class FakeTime:
    """time.monotonic, time.time and time.sleep on one clock that only moves when told."""

    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        return self.t

    def time(self):
        return 1_790_000_000.0 + self.t

    def sleep(self, s):
        self.t += s


class FirmwareModel:
    """The dock side of a unit, as Comms/shimmer_dock_usart.c behaves, behind the
    serial calls DockLink makes. The flags below break it in one way each."""

    gap_timeout = True              # the 100 ms inter-byte reset (§2.1)
    unknown_code = BAD_CMD          # what an unhandled component or property gets
    clears_rtc_set_by_bt = True     # SET RWC_CFG_TIME clears InfoMem byte 217 bit 7
    honours_cancel = True           # ENTER_BOOTLOADER 0 cancels a pending reboot

    def __init__(self, clock: FakeTime, platform: str = "shimmer3r", card=(48, 6, 0)):
        self.clock = clock
        self.platform = platform
        self.timeout = 1.0
        self.out = bytearray()
        self.steps, self.buf, self.need, self.last = "idle", bytearray(), 0, None
        self.mac = bytes.fromhex("CF9B9E3615BC")
        self.version = (10 if platform == "shimmer3r" else 3, 3, 1, 1, 22)
        self.infomem = bytearray((i * 7) & 0xFF for i in range(512))
        self.infomem[217] |= 0x80                       # rtcSetByBt, as a Bluetooth host left it
        self.infomem[230] &= 0xFE
        self.eeprom = bytearray((i * 13) & 0xFF for i in range(2048))
        self.eeprom[0:16] = bytes(card) + b"\xFF" * 13
        self.card_page = bytes(self.eeprom[0:16])       # read at boot
        self.offset_s = -3.25                           # the unit's clock, against the PC's
        self.cfg_ticks = 0
        self.reboot_at = None
        self.gone = False
        self.silent_after = None                        # stop answering after this many frames
        self.frames = []                                # (cmd, comp, prop) of each one processed
        self.pp_state = 0                               # power-profile record: DISARMED
        self.leds = None                                # LED_SET masks while held

    @property
    def prodtest(self):
        """The production-test commands: Shimmer3R from dock.PRODUCTION_TEST_CMDS_SINCE."""
        return self.platform == "shimmer3r" and tuple(self.version[2:]) >= dock.PRODUCTION_TEST_CMDS_SINCE

    # --- the serial side -----------------------------------------------------

    def reset_input_buffer(self):
        self.out.clear()

    def _check_alive(self):
        if self.reboot_at is not None and self.clock.t >= self.reboot_at:
            self.gone = True
        if self.gone:
            raise dock_protocol.serial.SerialException("device disconnected")

    def write(self, b):
        self._check_alive()
        for x in b:
            self._rx(x)

    def read(self, n):
        self._check_alive()
        if not self.out:
            self.clock.sleep(self.timeout)
            return b""
        got = bytes(self.out[:n])
        del self.out[:n]
        return got

    # --- the firmware --------------------------------------------------------

    def ticks(self):
        return int((self.clock.time() + self.offset_s) * TICKS)

    def _reply(self, code, comp=None, prop=None, data=b""):
        if self.silent_after is not None and len(self.frames) > self.silent_after:
            return
        body = bytes([START, code]) if comp is None else bytes([START, code, 2 + len(data), comp, prop]) + data
        self.out += body + struct.pack("<H", dock.crc16(body))

    def _rx(self, x):
        now = self.clock.t
        if self.gap_timeout and self.last is not None and now - self.last > 0.1:
            self.steps = "idle"
        self.last = now
        if self.steps == "idle":
            if x == START:
                self.buf, self.steps = bytearray([x]), "cmd"
        elif self.steps == "cmd":
            if x in (dock.CMD_SET, dock.CMD_GET):
                self.buf.append(x)
                self.steps = "len"
            else:
                self.steps = "idle"
                self._reply(BAD_CMD)
        elif self.steps == "len":
            if x == 0 or x > 133:
                self.steps = "idle"
                self._reply(BAD_ARG)
            else:
                self.buf.append(x)
                self.need, self.steps = x, "data"
        elif self.steps == "data":
            self.buf.append(x)
            self.need -= 1
            if not self.need:
                self.need, self.steps = 2, "crc"
        elif self.steps == "crc":
            self.buf.append(x)
            self.need -= 1
            if not self.need:
                self.steps, self.last = "idle", None
                self._process(bytes(self.buf))

    def _process(self, f):
        n = f[2]
        if struct.unpack("<H", f[n + 3:n + 5])[0] != dock.crc16(f[:n + 3]):
            self.frames.append(None)
            return self._reply(BAD_CRC)
        cmd, comp, prop, data = f[1], f[3], f[4], f[5:n + 3]
        self.frames.append((cmd, comp, prop))
        handler = getattr(self, f"_{'get' if cmd == dock.CMD_GET else 'set'}_{comp:02x}_{prop:02x}", None)
        if handler is None:
            return self._reply(self.unknown_code)
        handler(comp, prop, n, data)

    def _fixed(self, n, want, comp, prop, value):
        if n != want:
            return self._reply(BAD_ARG)
        self._reply(dock.CMD_RSP, comp, prop, value)

    def _get_01_02(self, c, p, n, d):
        self._fixed(n, 2, c, p, self.mac)

    def _get_01_03(self, c, p, n, d):
        self._fixed(n, 2, c, p, struct.pack("<BHHBB", *self.version))

    def _get_01_04(self, c, p, n, d):
        self._fixed(n, 2, c, p, struct.pack("<Q", self.cfg_ticks))

    def _get_01_05(self, c, p, n, d):
        self._fixed(n, 2, c, p, struct.pack("<Q", self.ticks()))

    def _get_01_06(self, c, p, n, d):
        length, offset = d[0], d[1] | d[2] << 8
        if length <= 0x80 and offset <= 0x1FF and length + offset <= 0x200:
            return self._reply(dock.CMD_RSP, c, p, bytes(self.infomem[offset:offset + length]))
        self._reply(BAD_ARG)

    def _get_02_02(self, c, p, n, d):
        self._fixed(n, 2, c, p, bytes([0xA2, 0x08, 0x01]))

    def _get_03_02(self, c, p, n, d):
        length, offset = d[0], d[1]
        if length <= 16 and offset <= 15 and length + offset <= 16:
            return self._reply(dock.CMD_RSP, c, p, self.card_page[offset:offset + length])
        self._reply(BAD_ARG)

    def _get_03_03(self, c, p, n, d):
        length, offset = d[0], d[1] | d[2] << 8
        if length <= 128 and offset <= 2031 and length + offset <= 2032:
            return self._reply(dock.CMD_RSP, c, p, bytes(self.eeprom[offset + 16:offset + 16 + length]))
        self._reply(BAD_ARG)

    def _get_0a_03(self, c, p, n, d):
        # An RN42's banner is two lines.
        self._fixed(n, 2, c, p, b"CYW20820 app=v01.04.18.18" if self.platform == "shimmer3r"
                    else b"Ver 6.15 04/26/2013\n\r(c) Roving Networks\r\n")

    def _set_01_04(self, c, p, n, d):
        if n != 10:
            return self._reply(BAD_ARG)
        ticks = struct.unpack("<Q", d)[0]
        self.offset_s = ticks / TICKS - self.clock.time()
        self.cfg_ticks = ticks
        if self.clears_rtc_set_by_bt:
            self.infomem[217] &= 0x7F
        self._reply(ACK)

    def _set_01_06(self, c, p, n, d):
        length, offset = d[0], d[1] | d[2] << 8
        if length <= 128 and offset <= 511 and length + offset <= 512:
            self.infomem[offset:offset + length] = d[3:3 + length]
            self.infomem[230] |= 0x01                   # flagWriteCfgToSd
            return self._reply(ACK)
        self._reply(BAD_ARG)

    def _set_01_09(self, c, p, n, d):
        if self.platform != "shimmer3r":
            return self._reply(BAD_CMD)
        if d[0]:
            self.reboot_at = self.clock.t + d[0]
        elif self.honours_cancel:
            self.reboot_at = None
        self._reply(ACK)

    # --- production-test commands (newer Shimmer3R) ---------------------------

    def _get_01_0a(self, c, p, n, d):
        if not self.prodtest:
            return self._reply(BAD_CMD)
        self._fixed(n, 2, c, p, bytes([1]) + bytes(range(12)) + bytes(9))

    def _get_01_0b(self, c, p, n, d):
        if not self.prodtest:
            return self._reply(BAD_CMD)
        self._fixed(n, 2, c, p, struct.pack("<BHH", 0, 3, 250))

    def _set_01_0c(self, c, p, n, d):
        if not self.prodtest:
            return self._reply(BAD_CMD)
        if n != 4:
            return self._reply(BAD_ARG)
        if d[0] == 0xFF and d[1] == 0xFF:
            self.leds = None
        elif d[0] > 7 or d[1] > 7:
            return self._reply(BAD_ARG)
        else:
            self.leds = (d[0], d[1])
        self._reply(ACK)

    def _get_0b_04(self, c, p, n, d):
        if not self.prodtest:
            return self._reply(BAD_CMD)
        steps = [(1, 2100), (2, 5000)]
        body = struct.pack("<BBBHIHHHBB", 1, self.pp_state, 0, 0, 0, 0, 0x0B, 1000, 1, len(steps))
        body += b"".join(struct.pack("<BH", i, ms) for i, ms in steps)
        self._fixed(n, 2, c, p, body)

    def _set_0b_04(self, c, p, n, d):
        if not self.prodtest:
            return self._reply(BAD_CMD)
        if n == 5 and d[0] == 1:
            self.pp_state = dock.PP_STATE_ARMED
        elif n == 3 and d[0] == 0:
            self.pp_state = 0
        else:
            return self._reply(BAD_ARG)
        self._reply(ACK)

    def _set_03_02(self, c, p, n, d):
        length, offset = d[0], d[1]
        if length <= 16 and offset <= 15 and length + offset <= 16:
            self.eeprom[offset:offset + length] = d[2:2 + length]
            page = bytearray(self.card_page)
            page[offset:offset + length] = d[2:2 + length]
            self.card_page = bytes(page)
            return self._reply(ACK)
        self._reply(BAD_ARG)

    def _set_03_03(self, c, p, n, d):
        length, offset = d[0], d[1] | d[2] << 8
        if 0 < length <= 128 and offset + length - 1 < 2032:
            self.eeprom[offset + 16:offset + 16 + length] = d[3:3 + length]
            return self._reply(ACK)
        self._reply(BAD_ARG)


def link_to(model) -> dock.DockLink:
    d = dock.DockLink.__new__(dock.DockLink)
    d.ser = model
    return d


class Walks(unittest.TestCase):
    def run_walk(self, model, **kw):
        with mock.patch.object(dock_protocol, "time", model.clock):
            return dock_protocol.walk(link_to(model), model.platform, "USB CDC", mac="CF9B9E3615BC",
                                      version="1.01.022", **kw)

    def model(self, platform="shimmer3r", **flags):
        m = FirmwareModel(FakeTime(), platform)
        for k, v in flags.items():
            setattr(m, k, v)
        return m

    def test_a_faithful_shimmer3r_passes_and_is_left_as_found(self):
        m = self.model()
        infomem, eeprom = bytes(m.infomem), bytes(m.eeprom)
        note, failures = self.run_walk(m)
        self.assertEqual(failures, [])
        self.assertIn("USB CDC: 9 GETs, 4 SETs, 50 errors, 4 framing", note)
        self.assertIn("card SR48-6-0", note)
        self.assertIn("clock -3.2 s", note)
        self.assertIn("pipelined: VER+MAC", note)
        self.assertEqual(bytes(m.eeprom), eeprom, "the EEPROM was changed")
        self.assertEqual(bytes(m.infomem[:230]) + bytes(m.infomem[231:]), infomem[:230] + infomem[231:],
                         "InfoMem was not put back")
        self.assertAlmostEqual(m.offset_s, 0, delta=0.01, msg="the clock was not set to the PC's time")
        self.assertIsNone(m.reboot_at)

    def test_a_shimmer3_refuses_the_bootloader_and_is_not_asked_to_enter_it(self):
        m = self.model("shimmer3")
        m.version = (3, 3, 1, 1, 22)
        note, failures = self.run_walk(m)
        self.assertEqual(failures, [])
        self.assertIn("9 GETs, 3 SETs, 51 errors", note)
        sets = [f for f in m.frames if f and f[0] == dock.CMD_SET and f[2] == dock.PROP_ENTER_BOOTLOADER]
        self.assertEqual(len(sets), 1, "a Shimmer3 should be sent ENTER_BOOTLOADER once, to be refused")

    def test_the_doc_s_old_claim_bad_arg_for_an_unknown_property_is_caught(self):
        _, failures = self.run_walk(self.model(unknown_code=BAD_ARG))
        self.assertTrue(any(f.startswith("GET SHIMMER ENABLE: BAD_ARG, expected BAD_CMD") for f in failures),
                        failures)

    def test_a_parser_with_no_gap_timeout_is_caught(self):
        _, failures = self.run_walk(self.model(gap_timeout=False))
        self.assertTrue(any("broken by a 250 ms gap was answered" in f for f in failures), failures)

    def test_a_clock_set_that_leaves_rtc_set_by_bt_is_caught(self):
        _, failures = self.run_walk(self.model(clears_rtc_set_by_bt=False))
        self.assertIn("setting the clock over the dock left rtcSetByBt set", failures)

    def test_a_cancel_that_does_not_hold_is_caught_and_recovered(self):
        recovered = []
        _, failures = self.run_walk(self.model(honours_cancel=False),
                                    recover=lambda: recovered.append(1) or "restarted from DFU")
        self.assertEqual(recovered, [1])
        self.assertTrue(any("not cancelled" in f and "restarted from DFU" in f for f in failures), failures)

    def prodtest_model(self, **flags):
        m = self.model(**flags)
        m.version = (10, 3) + dock.PRODUCTION_TEST_CMDS_SINCE
        return m

    def run_prodtest_walk(self, m):
        with mock.patch.object(dock_protocol, "time", m.clock):
            return dock_protocol.walk(link_to(m), m.platform, "USB CDC", mac="CF9B9E3615BC", version="1.01.024")

    def test_a_newer_shimmer3r_answers_the_production_test_commands(self):
        m = self.prodtest_model()
        note, failures = self.run_prodtest_walk(m)
        self.assertEqual(failures, [])
        self.assertIn("USB CDC: 12 GETs, 5 SETs, 51 errors, 4 framing", note)
        self.assertIsNone(m.leds, "LED_SET was not released")

    def test_the_generic_walk_never_arms_the_power_profile(self):
        m = self.prodtest_model()
        self.run_prodtest_walk(m)
        armed = [f for f in m.frames if f == (dock.CMD_SET, dock.COMP_TEST, dock.FACTORY_TEST_POWER_PROFILE)]
        self.assertEqual(armed, [], "the walk sent SET TEST 4 to a unit that would run it at its next boot")
        past = [f for f in m.frames if f and f[0] == dock.CMD_SET and f[1] == dock.COMP_TEST]
        self.assertEqual({f[2] for f in past}, {dock.FACTORY_TEST_COUNT, 0x7F})

    def test_an_armed_power_profile_is_caught(self):
        m = self.prodtest_model()
        m.pp_state = dock.PP_STATE_ARMED
        _, failures = self.run_prodtest_walk(m)
        self.assertTrue(any("power-profile record is armed" in f for f in failures), failures)

    def test_older_firmware_is_not_asked_for_the_production_test_commands(self):
        m = self.model()
        self.run_walk(m)
        # Each is sent once, bare, to be refused; none is read as a value.
        for frame in [(dock.CMD_GET, dock.COMP_SHIMMER, dock.PROP_BOOT_REPORT),
                      (dock.CMD_GET, dock.COMP_SHIMMER, dock.PROP_BUTTON),
                      (dock.CMD_SET, dock.COMP_SHIMMER, dock.PROP_LED_SET),
                      (dock.CMD_GET, dock.COMP_TEST, dock.FACTORY_TEST_POWER_PROFILE),
                      (dock.CMD_SET, dock.COMP_TEST, dock.FACTORY_TEST_POWER_PROFILE)]:
            self.assertEqual(m.frames.count(frame), 1, frame)

    def test_an_unprogrammed_card_id_is_not_written(self):
        m = FirmwareModel(FakeTime(), card=(0xFF, 0xFF, 0xFF))
        note, failures = self.run_walk(m)
        self.assertEqual(failures, [])
        self.assertIn("card unprogrammed", note)
        written =[f for f in m.frames if f and f[0] == dock.CMD_SET and f[1:] == (dock.COMP_DAUGHTER_CARD,
                                                                                   dock.PROP_CARD_ID)]
        # Only the out-of-range one, which must be refused.
        self.assertEqual(len(written), 1)

    def test_a_unit_that_stops_answering_ends_the_walk_without_raising(self):
        m = self.model()
        m.silent_after = 30
        _, failures = self.run_walk(m)
        self.assertTrue(failures)
        self.assertTrue(any("stopped answering" in f or "no answer" in f for f in failures), failures)

    def test_the_wrong_unit_is_named(self):
        m = self.model()
        m.mac = bytes.fromhex("000000000001")
        _, failures = self.run_walk(m)
        self.assertIn("MAC 000000000001, rig.toml says CF9B9E3615BC", failures)


    def test_four_units_notes_fit_the_report(self):
        notes = []
        for name in ("s3-rn4678-123", "s3-rn42-477", "s3-rn42-615", "s3-rn4678-1005"):
            m = FirmwareModel(FakeTime(), "shimmer3", card=(0xFF, 0xFF, 0xFF))
            m.version = (3, 3, 1, 1, 22)
            with mock.patch.object(dock_protocol, "time", m.clock):
                note, _ = dock_protocol.walk(link_to(m), m.platform, "Base6 UART")
            notes.append(f"{name}: {note.replace('pipelined: VER+MAC', 'pipelined: BAD_CMD+MAC')}")
        self.assertLessEqual(len("; ".join(notes)), 500, "; ".join(notes))


class Replies(unittest.TestCase):
    def test_error_codes_come_back_as_replies_not_exceptions(self):
        m = FirmwareModel(FakeTime())
        d = link_to(m)
        self.assertEqual(d.exchange(b"$\x05").code, BAD_CMD)
        self.assertEqual(d.exchange(b"$\x03\x00").code, BAD_ARG)
        f = dock.frame(dock.CMD_GET, dock.COMP_SHIMMER, dock.PROP_VER)
        self.assertEqual(d.exchange(f[:-1] + bytes([f[-1] ^ 1])).code, BAD_CRC)
        r = d.request(dock.CMD_GET, dock.COMP_SHIMMER, dock.PROP_MAC)
        self.assertEqual((r.code, r.comp, r.prop, r.data.hex()), (dock.CMD_RSP, 1, 2, "cf9b9e3615bc"))

    def test_get_still_raises_on_an_error_code(self):
        d = link_to(FirmwareModel(FakeTime()))
        with self.assertRaisesRegex(dock.DockError, "bad command"):
            d.get(dock.COMP_SHIMMER, 0x00)

    def test_nothing_arriving_is_none_not_an_error(self):
        d = link_to(FirmwareModel(FakeTime()))
        self.assertIsNone(d.read_reply(timeout_s=0.1))

    def test_typed_helpers_round_trip(self):
        m = FirmwareModel(FakeTime())
        d = link_to(m)
        d.set_rwc(123456789)
        self.assertEqual(d.rwc_config_ticks(), 123456789)
        self.assertEqual(d.card_id(3), bytes([48, 6, 0]))
        self.assertEqual(len(d.read_card_mem(0, dock.CARD_MEM_SIZE)), 2032)
        d.write_card_mem(b"\x01\x02", 100)
        self.assertEqual(d.read_card_mem(100, 2), b"\x01\x02")


if __name__ == "__main__":
    unittest.main()
