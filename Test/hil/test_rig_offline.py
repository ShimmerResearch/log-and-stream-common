#!/usr/bin/env python3
"""Offline tests for the bench rig: the parts that decide which unit is touched
and how a reply is read. Run: python Test/hil/test_rig_offline.py

No hardware, no gh, no pyserial needed: where a module imports pyserial and it
is not installed (CI), a stand-in is put in its place, and nothing here opens a
port.
"""

import sys
import types
import unittest
from contextlib import contextmanager
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

try:
    import serial  # noqa: F401
    from serial.tools import list_ports  # noqa: F401
except ImportError:
    _serial = types.ModuleType("serial")
    _tools = types.ModuleType("serial.tools")
    _lp = types.ModuleType("serial.tools.list_ports")
    _lp.comports = lambda: []
    _tools.list_ports = _lp
    _serial.tools = _tools
    _serial.Serial = object
    _serial.SerialException = type("SerialException", (OSError,), {})
    sys.modules.update({"serial": _serial, "serial.tools": _tools, "serial.tools.list_ports": _lp})

from rig import bt, config, dock, flash, power, sdfile  # noqa: E402
import run_rc  # noqa: E402


def load_rig(text):
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "rig.toml"
        p.write_text('bench = "b"\n' + text, encoding="utf-8")
        return config.load(p)


S3R_UNIT = '[[unit]]\nname = "{n}"\nplatform = "shimmer3r"\nusb_serial = "{n}"\n'


class StLinkPerUnit(unittest.TestCase):
    def test_a_bench_wide_st_link_serves_a_bench_with_one_unit(self):
        rig = load_rig('[tools]\nstlink_serial = "ST1"\n' + S3R_UNIT.format(n="a"))
        self.assertEqual(rig.stlink_for(rig.units[0]), "ST1")

    def test_a_bench_wide_st_link_is_refused_for_two_units(self):
        with self.assertRaisesRegex(config.RigError, "its own stlink_serial"):
            load_rig('[tools]\nstlink_serial = "ST1"\n' + S3R_UNIT.format(n="a") + S3R_UNIT.format(n="b"))

    def test_each_unit_uses_its_own(self):
        rig = load_rig(S3R_UNIT.format(n="a") + 'stlink_serial = "ST1"\n'
                       + S3R_UNIT.format(n="b") + 'stlink_serial = "ST2"\n' + S3R_UNIT.format(n="c"))
        self.assertEqual([rig.stlink_for(u) for u in rig.units], ["ST1", "ST2", None])

    def test_two_units_cannot_name_one_st_link(self):
        with self.assertRaisesRegex(config.RigError, "same ST-Link"):
            load_rig('[tools]\nstlink_serial = "ST1"\n' + S3R_UNIT.format(n="a") + 'stlink_serial = "ST1"\n'
                     + S3R_UNIT.format(n="b"))


class RigIdentities(unittest.TestCase):
    def test_two_units_cannot_share_a_device_or_a_port(self):
        unit = '[[unit]]\nname = "{n}"\nplatform = "shimmer3r"\n{extra}\n'
        for extra in ('mac = "AABBCCDDEEFF"', 'usb_serial = "0123456789AB"', "hub_port = 2"):
            with self.subTest(extra=extra):
                with self.assertRaisesRegex(config.RigError, "two units share"):
                    load_rig(unit.format(n="a", extra=extra) + unit.format(n="b", extra=extra))

    def test_usb_serials_differing_only_in_case_are_one_device(self):
        unit = '[[unit]]\nname = "{n}"\nplatform = "shimmer3r"\nusb_serial = "{s}"\n'
        with self.assertRaisesRegex(config.RigError, "two units share usb_serial"):
            load_rig(unit.format(n="a", s="abc123") + unit.format(n="b", s="ABC123"))

    def test_a_hub_port_must_be_a_port_number(self):
        for bad in ("0", "-1", '"1"', "true"):
            with self.subTest(hub_port=bad):
                with self.assertRaisesRegex(config.RigError, "hub_port"):
                    load_rig(S3R_UNIT.format(n="a") + f"hub_port = {bad}\n")

    def test_different_identities_load(self):
        rig = load_rig(S3R_UNIT.format(n="a") + "hub_port = 1\n" + S3R_UNIT.format(n="b") + "hub_port = 2\n")
        self.assertEqual([u.hub_port for u in rig.units], [1, 2])


class FlashEveryUnitOrNone(unittest.TestCase):
    def test_a_unit_that_cannot_be_flashed_stops_the_run_before_any_flash(self):
        import tempfile
        rig = config.Rig(path=Path("rig.toml"), bench="b", stm32_programmer=Path("cli"),
                         units=[config.Unit("a", "shimmer3r", usb_serial="A"), config.Unit("b", "shimmer3r")])
        flashed = []
        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(run_rc, "HERE", Path(d)), \
                mock.patch.object(run_rc.rig_config, "load", lambda p: rig), \
                mock.patch.object(run_rc, "fetch", lambda tag, out: ({"commit": "0" * 40}, Path("img.hex"))), \
                mock.patch.object(run_rc.rig_flash, "flash", lambda *a, **k: flashed.append(a)):
            with self.assertRaisesRegex(SystemExit, "b: no usb_serial"):
                run_rc.main(["LogAndStream_Shimmer3R_v1.01.022"])
        self.assertEqual(flashed, [], "a unit was flashed before the run was refused")

    def test_an_undocked_unit_stops_the_run_before_any_flash(self):
        import tempfile
        rig = config.Rig(path=Path("rig.toml"), bench="b", stm32_programmer=Path("cli"),
                         units=[config.Unit("a", "shimmer3r", usb_serial="A"),
                                config.Unit("b", "shimmer3r", usb_serial="B", hub_port=3)])
        flashed = []
        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(run_rc, "HERE", Path(d)), \
                mock.patch.object(run_rc.rig_config, "load", lambda p: rig), \
                mock.patch.object(run_rc, "fetch", lambda tag, out: ({"commit": "0" * 40}, Path("img.hex"))), \
                mock.patch.object(run_rc.s3r_usb, "cdc_port", lambda s: "COM9" if s == "A" else None), \
                mock.patch.object(run_rc.rig_flash, "flash", lambda *a, **k: flashed.append(a)):
            with self.assertRaisesRegex(SystemExit, "b: switch hub port 3 on"):
                run_rc.main(["LogAndStream_Shimmer3R_v1.01.022"])
        self.assertEqual(flashed, [], "a unit was flashed before the run was refused")

    def test_a_shimmer3_without_a_base6_slot_stops_the_run_before_any_flash(self):
        import tempfile
        rig = config.Rig(path=Path("rig.toml"), bench="b", base6_serial="FT1",
                         units=[config.Unit("a", "shimmer3", base6_slot=1), config.Unit("b", "shimmer3")])
        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(run_rc, "HERE", Path(d)), \
                mock.patch.object(run_rc.rig_config, "load", lambda p: rig), \
                mock.patch.object(run_rc, "fetch", lambda tag, out: ({"commit": "0" * 40}, Path("img.txt"))):
            with self.assertRaisesRegex(SystemExit, "b: no base6_slot"):
                run_rc.main(["LogAndStream_Shimmer3_v1.01.006"])

    def test_a_unit_with_an_st_link_is_flashed_though_not_on_usb(self):
        # SWD is the recovery route for firmware too broken to come up on USB.
        import tempfile
        rig = config.Rig(path=Path("rig.toml"), bench="b", stm32_programmer=Path("cli"),
                         units=[config.Unit("a", "shimmer3r", usb_serial="A", stlink_serial="ST1")])
        flashed = []

        def flash(*a, **k):
            flashed.append(a)
            raise SystemExit("stop after flashing")

        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(run_rc, "HERE", Path(d)), \
                mock.patch.object(run_rc.rig_config, "load", lambda p: rig), \
                mock.patch.object(run_rc, "fetch", lambda tag, out: ({"commit": "0" * 40}, Path("img.hex"))), \
                mock.patch.object(run_rc.s3r_usb, "cdc_port", lambda s: None), \
                mock.patch.object(run_rc.rig_flash, "flash", flash):
            with self.assertRaisesRegex(SystemExit, "stop after flashing"):
                run_rc.main(["LogAndStream_Shimmer3R_v1.01.022"])
        self.assertEqual(len(flashed), 1)
        self.assertEqual((flashed[0][3], flashed[0][4]), ("swd", "ST1"))

    def test_run_options_reach_pytest(self):
        import tempfile
        rig = config.Rig(path=Path("rig.toml"), bench="b", units=[config.Unit("a", "shimmer3r", usb_serial="A")])
        cmds = []

        class Proc:
            stdout = iter(())

            def __init__(self, cmd, **kw):
                cmds.append(cmd)

            def wait(self):
                return 0

        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(run_rc, "HERE", Path(d)), \
                mock.patch.object(run_rc.rig_config, "load", lambda p: rig), \
                mock.patch.object(run_rc, "fetch", lambda tag, out: ({"commit": "0" * 40}, Path("img.hex"))), \
                mock.patch.object(run_rc.subprocess, "Popen", Proc):
            run_rc.main(["LogAndStream_Shimmer3R_v1.01.022", "--no-flash", "--manual-power",
                         "--waive", "smoke.sd_log=ABC-1"])
        self.assertIn("--manual-power", cmds[0])
        self.assertIn("--waive=smoke.sd_log=ABC-1", cmds[0])

    def test_a_report_from_a_run_that_stopped_early_is_not_uploaded(self):
        import json
        import tempfile
        rig = config.Rig(path=Path("rig.toml"), bench="b", units=[config.Unit("a", "shimmer3r", usb_serial="A")])
        for pytest_rc, uploads in ((1, True), (2, False)):     # tests failed: evidence; interrupted: maybe partial
            uploaded = []

            class Proc:
                stdout = iter(())

                def __init__(self, cmd, **kw):
                    report = Path(cmd[cmd.index("--hil-report") + 1])
                    report.write_text(json.dumps({"results": [{"id": "smoke.x", "status": "pass"}]}),
                                      encoding="utf-8")

                def wait(self):
                    return pytest_rc

            gate = types.SimpleNamespace(returncode=1, stdout="", stderr="")
            with self.subTest(pytest_rc=pytest_rc), tempfile.TemporaryDirectory() as d, \
                    mock.patch.object(run_rc, "HERE", Path(d)), \
                    mock.patch.object(run_rc.rig_config, "load", lambda p: rig), \
                    mock.patch.object(run_rc, "fetch", lambda tag, out: ({"commit": "0" * 40}, Path("img.hex"))), \
                    mock.patch.object(run_rc.subprocess, "Popen", Proc), \
                    mock.patch.object(run_rc.subprocess, "run", lambda *a, **k: gate), \
                    mock.patch.object(run_rc, "gh", lambda *a, **k: uploaded.append(a) or ""):
                rc = run_rc.main(["LogAndStream_Shimmer3R_v1.01.022", "--no-flash", "--upload"])
                self.assertEqual(bool(uploaded), uploads)
                self.assertEqual(rc, pytest_rc)


class SessionDirs(unittest.TestCase):
    def test_sessions_compare_across_a_drive_letter_change(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d1, tempfile.TemporaryDirectory() as d2:
            for root in (d1, d2):
                (Path(root) / "data" / "exp" / "default-0001").mkdir(parents=True)
            (Path(d2) / "data" / "exp" / "default-0002").mkdir()
            new = sdfile.session_dirs(Path(d2)) - sdfile.session_dirs(Path(d1))
        self.assertEqual(new, {Path("data/exp/default-0002")})


class FakeHub:
    def __init__(self, on_ports):
        self.mask = sum(1 << (p - 1) for p in on_ports)
        self.calls = []
        self.closed = False

    def states(self):
        return self.mask

    def switch(self, ports, on):
        self.calls.append((tuple(ports), on))
        for p in ports:
            self.mask = self.mask | (1 << (p - 1)) if on else self.mask & ~(1 << (p - 1))

    def close(self):
        self.closed = True


def hub_power(on_ports):
    hp = power.HubPower.__new__(power.HubPower)
    hp.hub = FakeHub(on_ports)
    hp._initial = {}
    return hp


class HubPowerRestores(unittest.TestCase):
    def test_a_port_the_run_switched_off_is_switched_back_on(self):
        hp = hub_power([1])
        hp.off(1, "u1")
        hp.close()
        self.assertTrue(hp.hub.states() & 1)
        self.assertTrue(hp.hub.closed)

    def test_a_port_already_off_is_left_off(self):
        hp = hub_power([])
        hp.off(2, "u2")
        hp.close()
        self.assertFalse(hp.hub.states() & 2)
        self.assertNotIn(((2,), True), hp.hub.calls)

    def test_a_port_the_run_switched_on_goes_back_off(self):
        hp = hub_power([])
        hp.on(3, "u3")
        hp.off(3, "u3")
        hp.on(3, "u3")
        hp.close()
        self.assertFalse(hp.hub.states() & 4)

    def test_the_first_state_seen_is_the_one_restored(self):
        hp = hub_power([4])
        hp.off(4, "u4")
        hp.on(4, "u4")
        hp.off(4, "u4")
        hp.close()
        self.assertTrue(hp.hub.states() & 8)


class FakeDock:
    def __init__(self):
        self.entered = False

    def enter_bootloader(self, _):
        self.entered = True


@contextmanager
def dfu_bench(sequence):
    """flash.flash_dfu with the USB side faked: dfu_devices() returns each item
    of `sequence` in turn (the last one repeats), and the programmer is not run."""
    seq = list(sequence)
    dockobj = FakeDock()
    runs = []

    def dfu_devices():
        return seq.pop(0) if len(seq) > 1 else seq[0]

    @contextmanager
    def open_when_ready(port, **kw):
        yield dockobj

    with mock.patch.object(flash.s3r_usb, "dfu_devices", dfu_devices), \
            mock.patch.object(flash.s3r_usb, "wait_cdc", lambda s, t=0: "COM9"), \
            mock.patch.object(flash.s3r_usb, "wait_cdc_gone", lambda s, t=0: None), \
            mock.patch.object(flash.dock, "open_when_ready", open_when_ready), \
            mock.patch.object(flash, "_run", lambda cli, args, timeout_s=300: runs.append(args) or "ok"), \
            mock.patch.object(flash.time, "sleep", lambda s: None):
        yield dockobj, runs


class DfuTargetsOnlyThisUnit(unittest.TestCase):
    def test_refuses_when_another_unit_is_already_in_dfu(self):
        with dfu_bench([["USB\\VID_0483&PID_DF11\\OTHER"]]) as (d, runs):
            with self.assertRaisesRegex(flash.FlashError, "already present"):
                flash.flash_dfu(Path("cli"), Path("img.hex"), "SER1")
        self.assertFalse(d.entered, "the unit was sent to its bootloader before the check")
        self.assertEqual(runs, [])

    def test_refuses_when_two_appear(self):
        with dfu_bench([[], ["A", "B"]]) as (d, runs):
            with self.assertRaisesRegex(flash.FlashError, "appeared at once"):
                flash.flash_dfu(Path("cli"), Path("img.hex"), "SER1")
        self.assertEqual(runs, [])

    def test_flashes_the_one_that_appeared(self):
        with dfu_bench([[], [], ["A"]]) as (d, runs):
            flash.flash_dfu(Path("cli"), Path("img.hex"), "SER1")
        self.assertTrue(d.entered)
        self.assertEqual(len(runs), 1)
        self.assertIn("port=USB1", runs[0])


class FakeSpp:
    """The serial side of a BtLink: answers GET_STATUS with one status frame
    carrying `nstatus` status bytes, the first of them 0x20."""

    def __init__(self, nstatus):
        self.nstatus, self.buf, self.timeout = nstatus, bytearray(), 1.0

    @property
    def in_waiting(self):
        return len(self.buf)

    def reset_input_buffer(self):
        self.buf.clear()

    def write(self, data):
        if data == bytes([bt.GET_STATUS]):
            self.buf += bytes([bt.ACK, bt.INSTREAM, bt.STATUS_RSP, 0x20]) + bytes(self.nstatus - 1)

    def read(self, n=1):
        out = bytes(self.buf[:n])
        del self.buf[:n]
        return out


def bt_link(hw, fw, sent):
    """A BtLink on a FakeSpp whose firmware sends `sent` status bytes."""
    link = bt.BtLink.__new__(bt.BtLink)
    link.hw, link._nstatus, link.crc_mode = hw, None, 0
    link.ser = FakeSpp(sent)
    link.asked = 0

    def fw_version():
        link.asked += 1
        return fw
    link.fw_version = fw_version
    return link


class BtPortOpen(unittest.TestCase):
    def test_the_port_opens_with_dtr_and_rts_low(self):
        class FakeSerial:
            opened_with = None

            def __init__(self, *a, **k):
                self.dtr = self.rts = True              # pyserial's default
                self.port = None
                self.replies = bytearray([bt.ACK, bt.DEVICE_VERSION_RSP, bt.HW_SHIMMER3R])

            def open(self):
                FakeSerial.opened_with = (self.port, self.dtr, self.rts)

            def write(self, b):
                pass

            def reset_input_buffer(self):
                pass

            def read(self, n):
                out, self.replies = bytes(self.replies[:n]), self.replies[n:]
                return out

        with mock.patch.object(bt.serial, "Serial", FakeSerial), mock.patch.object(bt.time, "sleep", lambda s: None):
            link = bt.BtLink("COM9")
        self.assertEqual(FakeSerial.opened_with, ("COM9", False, False))
        self.assertEqual(link.hw, bt.HW_SHIMMER3R)

    def test_a_link_that_fails_its_first_exchange_closes_its_port(self):
        made = []

        class SilentSerial:
            def __init__(self, *a, **k):
                self.closed = False
                made.append(self)

            def open(self):
                pass

            def write(self, b):
                pass

            def reset_input_buffer(self):
                pass

            def read(self, n):
                return b""                              # nothing answers

            def close(self):
                self.closed = True

        with mock.patch.object(bt.serial, "Serial", SilentSerial), mock.patch.object(bt.time, "sleep", lambda s: None):
            with self.assertRaises(bt.BtError):
                bt.BtLink("COM9")
        self.assertTrue(made[0].closed, "the port was left open")


class PushedStatusFrame(unittest.TestCase):
    """A status frame the firmware pushes as a query goes out lands where the
    query's reply should be: ACK, then 0x8A instead of the response opcode."""

    class Spp(FakeSpp):
        def __init__(self, pushes):
            super().__init__(1)
            self.pushes, self.writes = pushes, []

        def write(self, data):
            self.writes.append(bytes(data))
            if self.pushes:
                self.pushes -= 1
                self.buf += bytes([bt.ACK, bt.INSTREAM, bt.STATUS_RSP, 0x20])
            self.buf += bytes([bt.ACK, bt.DEVICE_VERSION_RSP, bt.HW_SHIMMER3R])

    def link(self, pushes):
        link = bt.BtLink.__new__(bt.BtLink)
        link.hw, link._nstatus, link.crc_mode = bt.HW_SHIMMER3R, 1, 0
        link.ser = self.Spp(pushes)
        link.quiesce = lambda quiet_s=0.3, max_s=5.0: link.ser.buf.clear()
        return link

    def test_a_query_behind_a_pushed_status_frame_is_asked_again(self):
        link = self.link(pushes=1)
        self.assertEqual(link.device_version(), bt.HW_SHIMMER3R)
        self.assertEqual(len(link.ser.writes), 2)

    def test_it_gives_up_after_three_attempts(self):
        link = self.link(pushes=3)
        with self.assertRaises(bt.BtError):
            link.device_version()
        self.assertEqual(len(link.ser.writes), 3)

    def test_a_start_or_stop_is_never_sent_twice(self):
        link = self.link(pushes=1)
        link.command(bt.START_LOGGING)
        self.assertEqual(link.ser.writes, [bytes([bt.START_LOGGING])])


class StatusLength(unittest.TestCase):
    def test_shimmer3r_before_1_00_024_sends_one_status_byte(self):
        link = bt_link(bt.HW_SHIMMER3R, (3, 1, 0, 23), sent=1)
        self.assertEqual(link.status_len(), 1)
        self.assertEqual(link.status(timeout_s=0.2), 0x20)

    def test_shimmer3r_from_1_00_024_sends_two(self):
        link = bt_link(bt.HW_SHIMMER3R, (3, 1, 1, 0), sent=2)
        self.assertEqual(link.status(timeout_s=0.2), 0x20)
        self.assertEqual(link.status(timeout_s=0.2), 0x20)
        self.assertEqual(link.status_len(), 2)
        self.assertEqual(link.asked, 1, "the version is asked once, not per status")

    def test_a_two_byte_read_of_a_one_byte_reply_times_out(self):
        # What the fixed length used to do to pre-v1.00.024 Shimmer3R firmware.
        link = bt_link(bt.HW_SHIMMER3R, (3, 1, 1, 0), sent=1)
        with self.assertRaises(bt.BtError):
            link.status(timeout_s=0.2)

    def test_shimmer3_always_sends_one_and_is_not_asked_its_version(self):
        link = bt_link(bt.HW_SHIMMER3, None, sent=1)
        self.assertEqual(link.status(timeout_s=0.2), 0x20)
        self.assertEqual(link.asked, 0)


class TagVersion(unittest.TestCase):
    def test_the_tag_suffix_and_build_info_compare_as_one_version(self):
        platform, _, _, version = run_rc.parse_tag("LogAndStream_Shimmer3R_v1.01.022")
        self.assertEqual((platform, version), ("shimmer3r", "1.01.022"))
        for written in ("1.01.022", "v1.01.022", "V1.01.022"):
            self.assertEqual(run_rc.normalise_version(written), run_rc.normalise_version(version))


class TimestampSteps(unittest.TestCase):
    PERIOD = 128

    def classify(self, last, ts):
        st = bt.StreamStats()
        bt.classify_step(st, bt.ts_step(ts, last), self.PERIOD)
        return st

    def test_a_step_back_is_a_duplicate_not_a_huge_gap(self):
        st = self.classify(1000, 999)
        self.assertEqual((st.dupes, st.gaps, st.missing), (1, 0, 0))

    def test_a_wrap_is_a_normal_step(self):
        st = self.classify(bt.TS_WRAP - 28, 100)
        self.assertEqual((st.dupes, st.gaps, st.missing, st.jitter), (0, 0, 0, 0))

    def test_a_lost_sample_is_still_a_gap(self):
        st = self.classify(1000, 1000 + 2 * self.PERIOD)
        self.assertEqual((st.gaps, st.missing), (1, 1))

    def steps(self, *ds):
        st = bt.StreamStats()
        for d in ds:
            bt.classify_step(st, d, self.PERIOD)
        return st

    def test_a_stamp_more_than_half_a_period_late_is_jitter_not_loss(self):
        p = self.PERIOD
        late = p // 2 + 13                                   # over half a period late
        st = self.steps(p, p + late, p - late, p)
        self.assertEqual((st.gaps, st.missing, st.dupes), (0, 0, 0))
        self.assertEqual(st.max_jitter_ticks, late)

    def test_a_late_stamp_after_a_lost_sample_keeps_the_loss(self):
        p = self.PERIOD
        late = p // 2 + 13
        st = self.steps(2 * p + late, p - late)              # one lost, then one late
        self.assertEqual((st.gaps, st.missing, st.dupes), (1, 1, 0))

    def test_a_lost_sample_with_the_next_stamp_early_is_still_loss(self):
        p = self.PERIOD
        st = self.steps(2 * p - 1, p + 1)
        self.assertEqual((st.gaps, st.missing, st.dupes), (1, 1, 0))

    def test_a_lost_sample_then_a_duplicate_is_both(self):
        p = self.PERIOD
        st = self.steps(2 * p, 10)                           # a whole period over: not a late stamp
        self.assertEqual((st.gaps, st.missing, st.dupes), (1, 1, 1))

    def test_the_boundaries_are_three_quarters_of_a_period_either_side(self):
        # A stamp less than three quarters of a period late keeps its slot,
        # and exactly three quarters late is the next slot (rig/bt.py
        # classify_step), so from the first stamp:
        p = self.PERIOD
        cases = {               # step: (dupes, gaps, missing)
            3 * p // 4 - 1: (1, 0, 0),      # still the first slot: a duplicate
            3 * p // 4: (0, 0, 0),          # the next slot, a quarter of a period early
            p + p // 2: (0, 0, 0),          # half a period late: jitter, as seen on the bench
            p + 3 * p // 4 - 1: (0, 0, 0),
            p + 3 * p // 4: (0, 1, 1),      # a slot skipped: a sample lost
            2 * p - 1: (0, 1, 1),           # lost, and the next stamp a tick early
        }
        for d, want in cases.items():
            st = bt.StreamStats()
            bt.classify_step(st, d, p)
            with self.subTest(step=d):
                self.assertEqual((st.dupes, st.gaps, st.missing), want)


def sd_file(tmp, n_records, tail=b""):
    """A Shimmer3R SD file with SD sync on, timestamp-only records, `tail` appended."""
    head = bytearray(sdfile.HEAD_S3R)
    head[0:2] = (128).to_bytes(2, "little")
    head[30:32] = sdfile.HW_SHIMMER3R.to_bytes(2, "big")
    head[sdfile.SDH_TRIAL_CONFIG0] = sdfile.SDH_TIME_SYNC
    head[sdfile.SDH_NUM_ENABLED_CHANNELS] = 0
    body = bytes(sdfile.SYNC_HEAD) + b"".join((1 + 128 * i).to_bytes(3, "little") for i in range(n_records))
    p = Path(tmp) / "000"
    p.write_bytes(bytes(head) + body + tail)
    return p


class SdFileTrailing(unittest.TestCase):
    def test_a_whole_file_has_no_trailing_bytes(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            fc = sdfile.check(sd_file(d, 5))
        self.assertEqual((fc.records, fc.missing, fc.trailing_bytes), (5, 0, 0))

    def test_part_of_a_sync_header_at_the_end_is_trailing(self):
        import tempfile
        per_block = (sdfile.BLOCK - sdfile.SYNC_HEAD) // 3
        with tempfile.TemporaryDirectory() as d:
            fc = sdfile.check(sd_file(d, per_block, tail=bytes(5)))
        self.assertEqual(fc.records, per_block)
        self.assertEqual(fc.trailing_bytes, 5)

    def test_a_sync_header_with_no_record_after_it_is_trailing(self):
        import tempfile
        per_block = (sdfile.BLOCK - sdfile.SYNC_HEAD) // 3
        for tail, want in ((bytes(sdfile.SYNC_HEAD), sdfile.SYNC_HEAD),            # the header alone
                           (bytes(sdfile.SYNC_HEAD + 2), sdfile.SYNC_HEAD + 2)):   # and part of a record
            with self.subTest(tail=len(tail)), tempfile.TemporaryDirectory() as d:
                fc = sdfile.check(sd_file(d, per_block, tail=tail))
                self.assertEqual((fc.records, fc.trailing_bytes), (per_block, want))

    def test_a_session_split_across_files_reads_as_one_recording(self):
        # check_session (the soak tier's) joins 000, 001, ... and judges the
        # step across each split like any other.
        import shutil
        import tempfile
        with tempfile.TemporaryDirectory() as d, tempfile.TemporaryDirectory() as d2:
            first = sd_file(d, 5)                            # stamps 1, 129, ... 513
            shutil.copyfile(first, Path(d2) / "000")
            raw = bytearray(first.read_bytes())              # the same, carried on by five periods
            for i in range(5):
                pos = sdfile.HEAD_S3R + sdfile.SYNC_HEAD + 3 * i
                raw[pos:pos + 3] = (1 + 128 * (i + 5)).to_bytes(3, "little")
            (Path(d2) / "001").write_bytes(bytes(raw))
            sc = sdfile.check_session(Path(d2))
        self.assertEqual(sc.files, ["000", "001"])
        self.assertEqual((sc.records, sc.missing, sc.dupes), (10, 0, 0))
        self.assertEqual(sc.joins, [("001", 128)])

    def test_a_header_claiming_more_channels_than_a_shimmer3r_has_is_refused(self):
        raw = bytearray(sdfile.HEAD_S3R)
        raw[30:32] = sdfile.HW_SHIMMER3R.to_bytes(2, "big")
        raw[sdfile.SDH_NUM_ENABLED_CHANNELS] = sdfile.S3R_MAX_CHANNELS
        sdfile.read_header(bytes(raw))                       # 50 is the most it records
        raw[sdfile.SDH_NUM_ENABLED_CHANNELS] = sdfile.S3R_MAX_CHANNELS + 1
        with self.assertRaisesRegex(sdfile.SdFileError, "at most"):
            sdfile.read_header(bytes(raw))


class BuildInfoBinding(unittest.TestCase):
    TAG = "LogAndStream_Shimmer3R_v1.01.022"
    SHA = "0123456789abcdef0123456789abcdef01234567"

    def fetch_with(self, info_commit, has_build_info=True, download_fails=False):
        import json
        import subprocess
        import tempfile

        def gh(*args, **kw):
            if args[:2] == ("release", "view"):
                return "LogAndStream_Shimmer3R.hex" + ("\nbuild-info.json" if has_build_info else "")
            out = Path(args[args.index("-D") + 1])
            if "*.hex" in args:
                (out / "LogAndStream_Shimmer3R.hex").write_text(":00000001FF\n", encoding="utf-8")
            elif download_fails:
                raise subprocess.CalledProcessError(1, "gh", stderr="HTTP 502")
            else:
                (out / "build-info.json").write_text(json.dumps(
                    {"platform": "shimmer3r", "firmware_version": "1.01.022", "commit": info_commit}), encoding="utf-8")
            return ""

        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(run_rc, "gh", gh), \
                mock.patch.object(run_rc, "tag_commit", lambda repo, tag: self.SHA):
            return run_rc.fetch(self.TAG, Path(d))

    def test_build_info_for_the_tags_commit_is_accepted_in_lowercase(self):
        # The report needs a lowercase SHA, whatever case build-info.json had.
        info, _ = self.fetch_with(self.SHA.upper())
        self.assertEqual(info["commit"], self.SHA)

    def test_build_info_for_another_commit_is_refused(self):
        with self.assertRaisesRegex(SystemExit, "points at"):
            self.fetch_with("f" * 40)

    def test_a_release_without_build_info_gets_one_from_the_tag(self):
        info, _ = self.fetch_with(None, has_build_info=False)
        self.assertTrue(info["synthesised"])
        self.assertEqual(info["commit"], self.SHA)

    def test_a_failed_download_stops_the_run_instead_of_synthesising(self):
        import subprocess
        with self.assertRaises(subprocess.CalledProcessError):
            self.fetch_with(self.SHA, download_fails=True)


class RunnerExitStatus(unittest.TestCase):
    def results(self, *rs):
        return {"results": [dict(zip(("id", "status", "waiver"), r)) for r in rs]}

    def test_an_unwaived_failure_fails_the_run_even_if_pytest_passed(self):
        self.assertEqual(run_rc.exit_status(0, 1, self.results(("a", "pass"), ("b", "fail"))), 1)

    def test_a_waived_failure_and_uncovered_required_items_do_not(self):
        # One bench report cannot cover the items a person signs off; the gate
        # judges all the candidate's reports together.
        self.assertEqual(run_rc.exit_status(0, 1, self.results(("a", "pass"), ("b", "fail", "ABC-1"))), 0)

    def test_an_unusable_report_and_a_failed_test_fail_the_run(self):
        self.assertEqual(run_rc.exit_status(0, 2, self.results(("a", "pass"))), 2)
        self.assertEqual(run_rc.exit_status(1, 0, self.results(("a", "pass"))), 1)


class DockOpenRetries(unittest.TestCase):
    def test_a_port_dropped_during_the_first_read_is_retried(self):
        made = []

        class FakeLink:
            def __init__(self, port):
                self.closed = False
                made.append(self)

            def version(self):
                if len(made) == 1:
                    raise dock.serial.SerialException("device disconnected")
                return "1.01.022"

            def close(self):
                self.closed = True

        with mock.patch.object(dock, "DockLink", FakeLink), mock.patch.object(dock.time, "sleep", lambda s: None):
            link = dock.open_when_ready("COM9", attempts=3)
        self.assertEqual(len(made), 2)
        self.assertTrue(made[0].closed, "the first link was left open")
        self.assertIs(link, made[1])


if __name__ == "__main__":
    unittest.main(verbosity=2)
