#!/usr/bin/env python3
"""Offline tests for the bench rig: the parts that decide which unit is touched
and how a reply is read. Run: python Test/hil/test_rig_offline.py

No hardware, no gh, no pyserial needed: where a module imports pyserial and it
is not installed (CI), a stand-in is put in its place, and nothing here opens a
port.
"""

import asyncio
import json
import subprocess
import sys
import threading
import time
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

from rig import ble, bt, config, dock, flash, power, s3r_usb, sdfile, throughput  # noqa: E402
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


class DriveLookup(unittest.TestCase):
    """Get-Disk first, Win32_DiskDrive when Get-Disk does not list the unit.
    On the bench laptop Get-Disk has listed no disks at all for minutes, and
    listed a unit back from a long log minutes late, while Win32 listed both."""

    UNIT = {"number": 7, "name": "Shimmer Shimmer 10E0 USB Device", "serial": "205036654236", "letters": ["I"]}
    OTHER = {"number": 5, "name": "Shimmer Shimmer 15BC", "serial": "203C33795842", "letters": ["H"]}

    @contextmanager
    def powershell(self, get_disk, win32):
        """subprocess.run answering each query with what PowerShell prints:
        nothing for no disks, an object for one, a list for more."""
        asked = []

        def run(cmd, **kw):
            which = {s3r_usb.GET_DISK: "get_disk", s3r_usb.WIN32_DISKS: "win32"}[cmd[-1]]
            asked.append(which)
            disks = get_disk if which == "get_disk" else win32
            out = json.dumps(disks[0] if len(disks) == 1 else disks) if disks else ""
            return subprocess.CompletedProcess(cmd, 0, stdout=out + "\r\n", stderr="")
        with mock.patch.object(s3r_usb.subprocess, "run", run):
            yield asked

    def test_get_disk_empty_gives_the_drive_from_win32(self):
        with self.powershell(get_disk=[], win32=[self.UNIT]):
            self.assertEqual(s3r_usb.drive("205036654236"), Path("I:/"))
            self.assertEqual(s3r_usb.usb_disks(), [self.UNIT])

    def test_get_disk_missing_only_this_unit_asks_win32(self):
        with self.powershell(get_disk=[self.OTHER], win32=[self.OTHER, self.UNIT]) as asked:
            self.assertEqual(s3r_usb.drive("205036654236"), Path("I:/"))
        self.assertEqual(asked, ["get_disk", "win32"])

    def test_get_disk_listing_the_unit_without_a_letter_asks_win32(self):
        with self.powershell(get_disk=[dict(self.UNIT, letters=[])], win32=[self.UNIT]):
            self.assertEqual(s3r_usb.drive("205036654236"), Path("I:/"))

    def test_win32_is_not_asked_when_get_disk_has_the_unit(self):
        with self.powershell(get_disk=[self.OTHER, self.UNIT], win32=[]) as asked:
            self.assertEqual(s3r_usb.drive("205036654236"), Path("I:/"))
            self.assertEqual(s3r_usb.usb_disks(), [self.OTHER, self.UNIT])
        self.assertEqual(asked, ["get_disk", "get_disk"])

    def test_no_drive_when_neither_lists_the_unit(self):
        with self.powershell(get_disk=[self.OTHER], win32=[self.OTHER]):
            self.assertIsNone(s3r_usb.drive("205036654236"))


class UsbSettled(unittest.TestCase):
    """wait_usb_settled: a unit's CDC port and drive both back before a dock
    exchange. Sensing with USB-C in takes the whole USB device away and the stop
    re-enumerates it; the port alone can be up part-way through (DEV-1368)."""

    @contextmanager
    def unit(self, port_at=None, drive_at=None, port_gone=None):
        """A unit whose port is up from port_at s, except within the (from, to)
        window port_gone, and whose drive is mounted from drive_at, on a clock
        that only sleep advances. None: never."""
        clock = {"t": 0.0}

        def cdc_port(serial):
            t = clock["t"]
            up = port_at is not None and t >= port_at and not (port_gone and port_gone[0] <= t < port_gone[1])
            return "COM9" if up else None

        def drive(serial):
            return HERE if drive_at is not None and clock["t"] >= drive_at else None

        def sleep(s):
            clock["t"] += s
        fake_time = types.SimpleNamespace(monotonic=lambda: clock["t"], sleep=sleep)
        with mock.patch.object(s3r_usb, "cdc_port", cdc_port), \
                mock.patch.object(s3r_usb, "drive", drive), \
                mock.patch.object(s3r_usb, "time", fake_time):
            yield clock

    def test_a_port_without_its_drive_is_not_enough(self):
        with self.unit(port_at=1, drive_at=6) as clock:
            self.assertEqual(s3r_usb.wait_usb_settled("A", 60), "COM9")
            self.assertGreaterEqual(clock["t"], 6)

    def test_returns_at_once_when_both_are_back(self):
        with self.unit(port_at=0, drive_at=0) as clock:
            self.assertEqual(s3r_usb.wait_usb_settled("A", 60), "COM9")
            self.assertEqual(clock["t"], 0)

    def test_no_drive_times_out_on_one_deadline_naming_the_port(self):
        with self.unit(port_at=15) as clock:
            with self.assertRaises(TimeoutError) as cm:
                s3r_usb.wait_usb_settled("A", 20)
            self.assertLessEqual(clock["t"], 21)
        self.assertIn("did not mount", str(cm.exception))
        self.assertIn("COM9", str(cm.exception))

    def test_no_port_times_out(self):
        with self.unit(drive_at=0):
            with self.assertRaises(TimeoutError) as cm:
                s3r_usb.wait_usb_settled("A", 10)
        self.assertIn("CDC port", str(cm.exception))

    def test_a_port_that_goes_again_is_raised_not_waited_out(self):
        # Gone while the drive mounts and back later: a second enumeration, a
        # fault to see rather than absorb.
        with self.unit(port_at=0, drive_at=5, port_gone=(3, 8)) as clock:
            with self.assertRaises(OSError) as cm:
                s3r_usb.wait_usb_settled("A", 60)
            self.assertLess(clock["t"], 8)
        self.assertNotIsInstance(cm.exception, TimeoutError)
        self.assertIn("went away again", str(cm.exception))


class HostStall(unittest.TestCase):
    """stall_named: a dock exchange that fails soon after the unit's USB arrival
    names any host service stall it fell inside (System event 7011), and is
    still a failure (DEV-1368)."""

    ARRIVED = 1_000_000.0

    @contextmanager
    def host(self, arrived=ARRIVED, timeouts=(), now=None):
        """A unit that arrived at `arrived` (None: no record), a System log with
        these (time, service) 7011s, each readable once logged, and a clock that
        only sleep advances, from `now` (default 3 s after the arrival)."""
        clock = {"t": now if now is not None else (arrived or 0) + 3, "reads": 0}

        def sleep(s):
            clock["t"] += s

        def service_timeouts(since):
            clock["reads"] += 1
            clock["read_at"] = clock["t"]
            return sorted((t, s) for t, s in timeouts if since <= t <= clock["t"])
        fake_time = types.SimpleNamespace(time=lambda: clock["t"], sleep=sleep)
        with mock.patch.object(s3r_usb, "last_arrival", lambda serial: arrived), \
                mock.patch.object(s3r_usb, "service_timeouts", service_timeouts), \
                mock.patch.object(s3r_usb, "time", fake_time):
            yield clock

    def fail_with(self, error):
        with s3r_usb.stall_named("A"):
            raise error

    def test_a_failure_inside_a_stall_names_it_and_still_fails(self):
        a = self.ARRIVED
        cause = dock.serial.SerialException("Cannot configure port, something went wrong (995)")
        with self.host(timeouts=[(a + 30.2, "WSearch"), (a + 30.2, "Sense")]) as clock:
            with self.assertRaises(s3r_usb.HostServiceStall) as cm:
                self.fail_with(cause)
            # The stall shows only when the SCM gives up on it, 30 s after it began.
            self.assertGreaterEqual(clock["read_at"], a + 3 + s3r_usb.SCM_TIMEOUT_S)
        e = cm.exception
        self.assertIs(e.__cause__, cause)
        self.assertIsInstance(e, OSError)       # still caught where a SerialException is
        self.assertIn("Cannot configure port", str(e))
        self.assertIn("System event 7011 for Sense, WSearch at", str(e))
        self.assertIn("known_issues", str(e))

    def test_a_dock_error_inside_a_stall_is_named_too(self):
        with self.host(timeouts=[(self.ARRIVED + 30, "Sense")]):
            with self.assertRaises(s3r_usb.HostServiceStall) as cm:
                self.fail_with(dock.DockError("no reply to 2403"))
        self.assertIn("DockError: no reply", str(cm.exception))

    def test_with_no_stall_logged_the_failure_goes_on_as_it_was(self):
        cause = dock.DockError("no reply to 2403")
        with self.host() as clock:
            with self.assertRaises(dock.DockError) as cm:
                self.fail_with(cause)
            self.assertEqual(clock["reads"], 1)
        self.assertIs(cm.exception, cause)

    def test_a_stall_logged_before_the_arrival_is_not_this_one(self):
        cause = dock.DockError("no reply to 2403")
        with self.host(timeouts=[(self.ARRIVED - 5, "Sense")]):
            with self.assertRaises(dock.DockError) as cm:
                self.fail_with(cause)
        self.assertIs(cm.exception, cause)

    def test_a_failure_long_after_the_arrival_is_raised_at_once(self):
        cause = dock.DockError("no reply to 2403")
        with self.host(now=self.ARRIVED + 60) as clock:
            with self.assertRaises(dock.DockError) as cm:
                self.fail_with(cause)
            self.assertEqual((clock["t"], clock["reads"]), (self.ARRIVED + 60, 0))
        self.assertIs(cm.exception, cause)

    def test_no_arrival_on_record_is_raised_at_once(self):
        with self.host(arrived=None) as clock:
            with self.assertRaises(dock.DockError):
                self.fail_with(dock.DockError("no reply"))
            self.assertEqual(clock["reads"], 0)

    def test_other_errors_are_not_looked_into(self):
        with self.host(timeouts=[(self.ARRIVED + 30, "Sense")]) as clock:
            for error in (ValueError("bad row"), AssertionError("configuration: divider")):
                with self.subTest(error=error), self.assertRaises(type(error)):
                    self.fail_with(error)
            self.assertEqual(clock["reads"], 0)

    def test_a_named_stall_is_not_named_again(self):
        inner = s3r_usb.HostServiceStall("already named")
        with self.host(timeouts=[(self.ARRIVED + 30, "Sense")]) as clock:
            with self.assertRaises(s3r_usb.HostServiceStall) as cm:
                self.fail_with(inner)
            self.assertEqual(clock["reads"], 0)
        self.assertIs(cm.exception, inner)

    def test_a_failed_lookup_leaves_the_failure_itself(self):
        def broken(since):
            raise subprocess.TimeoutExpired("powershell", 60)
        cause = dock.DockError("no reply")
        with self.host(), mock.patch.object(s3r_usb, "service_timeouts", broken):
            with self.assertRaises(dock.DockError) as cm:
                self.fail_with(cause)
        self.assertIs(cm.exception, cause)

    def test_last_arrival_reads_the_composite_device(self):
        with mock.patch.object(s3r_usb, "_powershell", return_value="1793262028007") as ps:
            self.assertEqual(s3r_usb.last_arrival("204e36594236"), 1793262028.007)
        self.assertIn(r"'USB\VID_0483&PID_5710\204E36594236'", ps.call_args[0][0])
        with mock.patch.object(s3r_usb, "_powershell", return_value=""):
            self.assertIsNone(s3r_usb.last_arrival("204E36594236"))

    def test_service_timeouts_are_parsed_filtered_and_ordered(self):
        one = '{"ms":1793262058007,"service":"Sense"}'
        many = ('[{"ms":1793262058007,"service":"WSearch"},{"ms":1793262058007,"service":"Sense"},'
                '{"ms":1793224246170,"service":"Sense"}]')
        with mock.patch.object(s3r_usb, "_powershell", return_value=one):
            self.assertEqual(s3r_usb.service_timeouts(1793262000), [(1793262058.007, "Sense")])
        with mock.patch.object(s3r_usb, "_powershell", return_value=many):
            self.assertEqual(s3r_usb.service_timeouts(1793262000),
                             [(1793262058.007, "Sense"), (1793262058.007, "WSearch")])
        with mock.patch.object(s3r_usb, "_powershell", return_value=""):
            self.assertEqual(s3r_usb.service_timeouts(1793262000), [])


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



class DfuLeaveMissed(unittest.TestCase):
    """DEV-1316: ST's bootloader sometimes drops the -s jump and the unit never comes back."""

    def _flash(self, stlink):
        cdc = iter([TimeoutError("no CDC"), "COM9"])

        def wait_cdc(serial, timeout_s=0):
            r = next(cdc)
            if isinstance(r, Exception):
                raise r
            return r

        resets = []

        @contextmanager
        def open_when_ready(port, **kw):
            yield types.SimpleNamespace(version=lambda: types.SimpleNamespace(text="1.01.022"))

        with mock.patch.object(flash, "flash_dfu", lambda *a, **k: "ok"), \
                mock.patch.object(flash, "reset_swd", lambda cli, sn=None: resets.append(sn)), \
                mock.patch.object(flash.s3r_usb, "wait_cdc", wait_cdc), \
                mock.patch.object(flash.dock, "open_when_ready", open_when_ready), \
                mock.patch.object(flash.Path, "exists", lambda self: True):
            v = flash.flash(Path("cli"), Path("img.hex"), "SER1", "dfu", stlink_serial=stlink)
        return v, resets

    def test_resets_through_the_stlink_and_carries_on(self):
        v, resets = self._flash("STLINK1")
        self.assertEqual(resets, ["STLINK1"])
        self.assertEqual(v.text, "1.01.022")

    def test_without_an_stlink_says_a_hand_reset_starts_it(self):
        with self.assertRaisesRegex(flash.FlashError, "Reset the unit by hand"):
            self._flash(None)

    def _restart(self, stlink, cdc_back):
        """flash.restart() with the programmer and USB faked; cdc_back = whether the CDC
        port comes back after the first start."""
        calls = []

        def wait_cdc(serial, timeout_s=0):
            calls.append("wait_cdc")
            if not cdc_back:
                raise TimeoutError("no CDC")
            return "COM9"

        with mock.patch.object(flash, "_enter_dfu", lambda s, t: calls.append("enter_dfu")), \
                mock.patch.object(flash, "start_from_dfu", lambda cli: calls.append("-s")), \
                mock.patch.object(flash, "reset_swd", lambda cli, sn=None: calls.append(f"swd {sn}")), \
                mock.patch.object(flash.s3r_usb, "wait_cdc", wait_cdc):
            return flash.restart(Path("cli"), "SER1", stlink), calls

    def test_restart_with_an_stlink_never_goes_near_the_bootloader(self):
        port, calls = self._restart("STLINK1", True)
        self.assertEqual((port, calls), ("COM9", ["swd STLINK1", "wait_cdc"]))

    def test_restart_through_the_bootloader(self):
        port, calls = self._restart(None, True)
        self.assertEqual((port, calls), ("COM9", ["enter_dfu", "-s", "wait_cdc"]))

    def test_restart_whose_jump_is_missed_says_so(self):
        # The 7A62 smoke run (v1.01.023): a bare TimeoutError there surfaced later as an
        # InfoMem PermissionError and ended the run with no word of the cause.
        with self.assertRaisesRegex(flash.FlashError, r"SER1: restarted .*\(DEV-1316\)"):
            self._restart(None, False)


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

    def flaky_open(self, errors):
        """A port whose first opens fail with these messages, in order."""
        opens = []

        class FlakySerial:
            def __init__(self, *a, **k):
                self.port = None

            def open(self):
                opens.append(1)
                if len(opens) <= len(errors):
                    raise bt.serial.SerialException(errors[len(opens) - 1])

        with mock.patch.object(bt.serial, "Serial", FlakySerial), \
                mock.patch.object(bt.serial, "SerialException", Exception), \
                mock.patch.object(bt.time, "sleep", lambda s: None):
            try:
                bt._open_spp("COM18", 1.0)
                return len(opens), None
            except Exception as e:
                return len(opens), e

    DROP = "could not open port 'COM18': OSError(22, 'The device is not connected.', None, 1167)"

    def test_a_dropped_link_is_opened_again(self):
        opens, err = self.flaky_open([self.DROP])
        self.assertIsNone(err)
        self.assertEqual(opens, 2)

    def test_a_second_drop_is_reported(self):
        opens, err = self.flaky_open([self.DROP, self.DROP])
        self.assertIsNotNone(err)
        self.assertEqual(opens, 2)


class BtMode(unittest.TestCase):
    """rig/btmode.py: SD sync and bluetoothDisable cleared for a run, put back after."""

    class Unit:
        def __init__(self, name):
            self.name = name

    class Dock:
        def __init__(self, im):
            self.im = bytearray(im)

        def read_infomem(self, offset=0, length=None):
            return bytes(self.im[offset:offset + (length or len(self.im))])

        def write_infomem(self, data, offset=0):
            self.im[offset:offset + len(data)] = data

    def bench(self, byte217, refused=False):
        from contextlib import contextmanager

        from rig import btmode
        im = bytearray(range(256)) * 2
        im[btmode.SYNC_CFG_OFFSET] = byte217
        d, resets = self.Dock(im), []

        @contextmanager
        def access(unit):
            yield d

        def reset(unit):
            resets.append(unit.name)
            if refused:                                  # the firmware sets sync again on boot
                d.im[btmode.SYNC_CFG_OFFSET] |= btmode.SYNC_ENABLE
        return btmode.BtMode(access, reset, log=lambda m: None), d, bytes(im), resets

    def test_sync_on_is_cleared_and_put_back(self):
        mode, d, original, resets = self.bench(0x15)
        mode.enable(self.Unit("s3"))
        self.assertEqual(d.im[217], 0x11)
        self.assertEqual(bytes(d.im[:217]) + bytes(d.im[218:]), original[:217] + original[218:])
        self.assertEqual(mode.restore(), [])
        self.assertEqual(bytes(d.im), original)
        self.assertEqual(resets, ["s3", "s3"])

    def test_bluetooth_disabled_is_cleared_too(self):
        mode, d, _, _ = self.bench(0x0B)
        mode.enable(self.Unit("s3"))
        self.assertEqual(d.im[217], 0x03)

    def test_a_unit_already_on_is_left_alone(self):
        mode, d, original, resets = self.bench(0x11)
        mode.enable(self.Unit("s3r"))
        self.assertEqual(mode.restore(), [])
        self.assertEqual(bytes(d.im), original)
        self.assertEqual(resets, [])

    def test_a_unit_is_handled_once_per_run(self):
        mode, _, _, resets = self.bench(0x15)
        mode.enable(self.Unit("s3"))
        mode.enable(self.Unit("s3"))
        self.assertEqual(resets, ["s3"])

    def test_firmware_that_sets_it_again_stops_the_run_and_is_still_restored(self):
        from rig import btmode
        mode, d, original, _ = self.bench(0x15, refused=True)
        with self.assertRaises(btmode.BtModeError):
            mode.enable(self.Unit("s3"))
        self.assertEqual(mode.restore(), [])
        self.assertEqual(d.im[217], original[217])

    def test_a_write_that_fails_part_way_is_still_restored(self):
        mode, d, original, _ = self.bench(0x15)
        real = d.write_infomem
        state = {"failed": False}

        def write_then_fail(data, offset=0):
            real(data, offset)
            if not state["failed"]:
                state["failed"] = True
                raise IOError("dock link dropped after the write")
        d.write_infomem = write_then_fail
        with self.assertRaises(IOError):
            mode.enable(self.Unit("s3"))
        self.assertEqual(mode.restore(), [])
        self.assertEqual(bytes(d.im), original)

    def test_a_short_read_changes_nothing(self):
        from rig import btmode
        mode, d, original, resets = self.bench(0x15)
        d.read_infomem = lambda offset=0, length=None: bytes(d.im[:300])
        with self.assertRaises(btmode.BtModeError):
            mode.enable(self.Unit("s3"))
        self.assertEqual(bytes(d.im), original)
        self.assertEqual(resets, [])
        self.assertEqual(mode.restore(), [])

    def test_restore_checks_every_byte(self):
        mode, d, _, _ = self.bench(0x15)
        mode.enable(self.Unit("s3"))
        real = d.write_infomem

        def write_corrupting(data, offset=0):
            real(data, offset)
            d.im[40] ^= 0xFF                              # far from byte 217
        d.write_infomem = write_corrupting
        problems = mode.restore()
        self.assertEqual(len(problems), 1)
        self.assertIn("offsets [40]", problems[0])

    def test_the_self_set_config_flag_is_not_a_difference(self):
        mode, d, _, _ = self.bench(0x15)
        mode.enable(self.Unit("s3"))
        real = d.write_infomem

        def write_setting_flag(data, offset=0):
            real(data, offset)
            d.im[230] |= 0x01                             # flagWriteCfgToSd, set by the firmware
        d.write_infomem = write_setting_flag
        self.assertEqual(mode.restore(), [])


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


def sd_file(tmp, n_records, tail=b"", stamps=None, name="000"):
    """A Shimmer3R SD file with SD sync on, timestamp-only records, `tail` appended.
    The records are stamped one period (128) apart from 1, or with `stamps`."""
    head = bytearray(sdfile.HEAD_S3R)
    head[0:2] = (128).to_bytes(2, "little")
    head[30:32] = sdfile.HW_SHIMMER3R.to_bytes(2, "big")
    head[sdfile.SDH_TRIAL_CONFIG0] = sdfile.SDH_TIME_SYNC
    head[sdfile.SDH_NUM_ENABLED_CHANNELS] = 0
    if stamps is None:
        stamps = [1 + 128 * i for i in range(n_records)]
    body = bytes(sdfile.SYNC_HEAD) + b"".join(s.to_bytes(3, "little") for s in stamps)
    p = Path(tmp) / name
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


class SdFileZeroStamp(unittest.TestCase):
    # The stamp is the RTC's low 24 bits, so a log whose stamps are multiples of
    # the divider stamps the sample at each 512 s wrap 00 00 00 (DEV-1361).
    WRAP = bt.TS_WRAP

    def check(self, stamps):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            return sdfile.check(sd_file(d, 0, stamps=stamps))

    def test_a_zero_at_the_wrap_is_a_sample(self):
        fc = self.check([self.WRAP - 256, self.WRAP - 128, 0, 128, 256])
        self.assertEqual((fc.records, fc.zero_ts, fc.missing, fc.dupes, fc.jitter), (5, 0, 0, 0, 0))
        self.assertEqual(fc.span_ticks, 4 * 128)

    def test_a_zero_mid_range_is_an_unstamped_record(self):
        # DEV-1023's: an extra record, never stamped, between two real ones.
        fc = self.check([1000, 1128, 0, 1256, 1384])
        self.assertEqual((fc.records, fc.zero_ts, fc.missing, fc.dupes), (5, 1, 0, 0))

    def test_a_zero_more_than_a_second_before_the_wrap_is_unstamped(self):
        fc = self.check([self.WRAP - 32768 - 128, 0, self.WRAP - 32768])
        self.assertEqual(fc.zero_ts, 1)

    def test_a_first_record_stamped_zero_is_a_sample(self):
        fc = self.check([0, 128, 256])
        self.assertEqual((fc.zero_ts, fc.missing, fc.span_ticks), (0, 0, 256))

    def test_a_zero_at_the_wrap_across_a_file_split_is_a_sample(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            sd_file(d, 0, stamps=[self.WRAP - 256, self.WRAP - 128], name="000")
            sd_file(d, 0, stamps=[0, 128], name="001")
            sc = sdfile.check_session(Path(d))
        self.assertEqual((sc.records, sc.zero_ts, sc.missing, sc.dupes), (4, 0, 0, 0))
        self.assertEqual(sc.joins, [("001", 128)])


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


class DockReplyTimeout(unittest.TestCase):
    """read_reply reconfigures the port only for a timeout it is asked to change.
    pyserial reconfigures at every assignment, which on the Shimmer3R's CDC port
    sends the unit control requests, and that call was the one aborted with
    WinError 995 inside a host service stall (DEV-1368)."""

    class Port:
        def __init__(self, reply=b""):
            self._timeout, self.assigned, self.reply = 1.0, [], bytearray(reply)

        @property
        def timeout(self):
            return self._timeout

        @timeout.setter
        def timeout(self, t):
            self.assigned.append(t)
            self._timeout = t

        def read(self, n):
            out = bytes(self.reply[:n])
            del self.reply[:n]
            return out

    def link(self, reply=b""):
        d = dock.DockLink.__new__(dock.DockLink)
        d.ser = self.Port(reply)
        return d

    def test_the_port_keeps_the_timeout_it_has(self):
        for t in (None, 1.0):
            with self.subTest(timeout_s=t):
                d = self.link()
                self.assertIsNone(d.read_reply(t))
                self.assertEqual(d.ser.assigned, [])

    def test_a_different_timeout_is_set_and_put_back(self):
        d = self.link()
        self.assertIsNone(d.read_reply(5.0))
        self.assertEqual(d.ser.assigned, [5.0, 1.0])

    def test_a_reply_is_read_without_touching_the_port(self):
        ack = bytes([dock.START, dock.ACK])
        d = self.link(ack + dock.crc16(ack).to_bytes(2, "little"))
        self.assertEqual(d.read_reply().code, dock.ACK)
        self.assertEqual(d.ser.assigned, [])


def rate_packets(counters) -> bytes:
    return b"".join(bytes([throughput.DATA_RATE_TEST_RESPONSE]) + c.to_bytes(4, "little") for c in counters)


class DataRateScan(unittest.TestCase):
    def test_contiguous_packets_have_no_gaps(self):
        st = throughput.scan_data_rate(rate_packets(range(100)))
        self.assertEqual((st.packets, st.gaps, st.lost, st.resyncs), (100, 0, 0, 0))

    def test_a_counter_jump_is_packets_lost(self):
        st = throughput.scan_data_rate(rate_packets([*range(10), *range(12, 20)]))
        self.assertEqual((st.packets, st.gaps, st.lost), (18, 1, 2))

    def test_a_first_counter_above_zero_is_packets_lost(self):
        st = throughput.scan_data_rate(rate_packets(range(3, 10)))
        self.assertEqual((st.gaps, st.lost), (1, 3))

    def test_a_stray_byte_is_counted_and_framing_resumes(self):
        data = rate_packets(range(5)) + b"\x00" + rate_packets(range(5, 10))
        st = throughput.scan_data_rate(data)
        self.assertEqual((st.packets, st.gaps, st.resyncs), (10, 0, 1))

    def test_a_dropped_byte_loses_that_packet_and_no_more(self):
        data = bytearray(rate_packets(range(10)))
        del data[5 * 5 + 2]                     # one counter byte of packet 5
        st = throughput.scan_data_rate(bytes(data))
        self.assertEqual((st.packets, st.gaps, st.lost, st.resyncs), (9, 1, 1, 4))

    def test_a_partial_packet_at_the_end_is_not_counted(self):
        st = throughput.scan_data_rate(rate_packets(range(4)) + b"\xa5\x04\x00")
        self.assertEqual((st.packets, st.resyncs), (4, 0))

    def test_the_uint32_counter_wraps_to_zero_as_one_step(self):
        st = throughput.scan_data_rate(rate_packets([0xFFFFFFFF, 0, 1]),
                                       throughput.DataRate(last=0xFFFFFFFE))
        self.assertEqual((st.packets, st.gaps, st.resyncs, st.last), (3, 0, 0, 1))

    def test_a_gap_across_the_wrap_is_counted(self):
        st = throughput.scan_data_rate(rate_packets([1]), throughput.DataRate(last=0xFFFFFFFE))
        self.assertEqual((st.packets, st.gaps, st.lost), (1, 1, 2))


# A stream whose stamps span its 20 s: 640 slots of 1024 ticks.
SPANNING = bt.StreamStats(packets=600, missing=40, seconds=20.0)
INQ_32HZ = bt.Inquiry(1024, [], 0, b"")


class StreamCoverage(unittest.TestCase):
    """Lost samples between packets show as gaps; a stream that stopped
    delivering shows none, so its stamps must span the stream."""

    CLEAN = throughput.DataRate(bytes=5, seconds=1.0, packets=1, stopped=True, stop_ack=True)

    def test_a_stream_whose_stamps_span_it_is_clean(self):
        self.assertEqual(throughput.defects(self.CLEAN, SPANNING, INQ_32HZ), [])

    def test_one_packet_is_not_a_full_delivery(self):
        st = bt.StreamStats(packets=1, seconds=20.0)
        self.assertEqual(throughput.samples_per_s(st, INQ_32HZ), 32.0)   # what it would claim
        self.assertIn("cover", "; ".join(throughput.defects(self.CLEAN, st, INQ_32HZ)))

    def test_a_missing_end_is_a_defect(self):
        st = bt.StreamStats(packets=100, seconds=20.0)    # 3.1 s of stamps, no gaps
        self.assertIn("end is missing", "; ".join(throughput.defects(self.CLEAN, st, INQ_32HZ)))


class StreamShortPacket(unittest.TestCase):
    """A packet that arrives a byte short - the second byte lost, as on the
    RN42 when two starts of the TX chain race (TEST_PROCEDURE §5.5) - is
    plen - 1 bytes that do not frame, and the defect names them."""

    CHANNELS = list(range(0x0D))          # the load's 13 two-byte channels: 30-byte packets

    class Spp(FakeSpp):
        def __init__(self, stream):
            super().__init__(1)
            self.stream = stream

        def write(self, data):
            if data == bytes([bt.START_STREAMING]):
                self.buf += bytes([bt.ACK]) + self.stream
            elif data == bytes([bt.STOP_STREAMING]):
                self.buf += bytes([bt.ACK])

    @staticmethod
    def packet(ts):
        return bytes([0]) + ts.to_bytes(3, "little") + bytes(range(0x10, 0x10 + 26))

    def stream(self, drop=None):
        pkts = [self.packet(1000 + 32 * i) for i in range(10)]
        if drop is not None:
            p = pkts[5]
            pkts[5] = p[:drop] + p[drop + 1:]
        link = bt.BtLink.__new__(bt.BtLink)
        link.hw, link._nstatus, link.crc_mode = bt.HW_SHIMMER3, 1, 0
        link.ser = self.Spp(b"".join(pkts))
        link.ser.timeout = 0.0
        return link.stream(0.0, bt.Inquiry(32, self.CHANNELS, 0, b""))

    def test_a_whole_stream_frames(self):
        st = self.stream()
        self.assertEqual((st.packets, st.resyncs, st.stop_ack), (10, 0, True))

    def test_a_packet_a_byte_short_is_29_stray_bytes(self):
        st = self.stream(drop=1)
        self.assertEqual((st.packets, st.resyncs), (9, 29))
        short = self.packet(1000 + 32 * 5)
        self.assertTrue(st.misframes[0].startswith((short[:1] + short[2:]).hex()))

    def test_the_defect_carries_the_bytes(self):
        st = self.stream(drop=1)
        msg = "; ".join(throughput.defects(StreamCoverage.CLEAN, st, bt.Inquiry(32, self.CHANNELS, 0, b"")))
        self.assertIn("29 bytes did not frame as packets (first from 00", msg)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        self.t += 0.01
        return self.t


class DataRateSpp(FakeSpp):
    """SET_DATA_RATE_TEST: ACK, then packets on every read until the stop,
    which (if `obeys`) flushes them and is ACKed."""

    def __init__(self, obeys=True):
        super().__init__(1)
        self.obeys, self.flooding, self.counter = obeys, False, 0

    def write(self, data):
        if data == bytes([throughput.SET_DATA_RATE_TEST, 1]):
            self.buf += bytes([bt.ACK])
            self.flooding = True
        elif data == bytes([throughput.SET_DATA_RATE_TEST, 0]) and self.obeys:
            self.flooding = False
            self.buf[:] = bytes([bt.ACK])

    def read(self, n=1):
        if not self.buf and self.flooding:
            self.buf += rate_packets(range(self.counter, self.counter + 4))
            self.counter += 4
        return super().read(n)


class DataRateTest(unittest.TestCase):
    def run_test(self, spp):
        link = bt.BtLink.__new__(bt.BtLink)
        link.hw, link._nstatus, link.crc_mode, link.ser = bt.HW_SHIMMER3, 1, 0, spp
        with mock.patch.object(throughput, "time", FakeClock()):
            return throughput.data_rate_test(link, 0.5)

    def test_a_flood_that_stops_on_command(self):
        st = self.run_test(DataRateSpp())
        self.assertTrue(st.stopped and st.stop_ack)
        self.assertGreater(st.packets, 0)
        self.assertEqual((st.gaps, st.resyncs), (0, 0))
        # Every byte counts towards the rate; only a partial packet at the cut is not a packet.
        self.assertTrue(0 <= st.bytes - st.packets * throughput.PACKET < throughput.PACKET)
        self.assertEqual(throughput.defects(st, SPANNING, INQ_32HZ), [])

    def test_a_flood_that_ignores_the_stop_is_a_defect(self):
        st = self.run_test(DataRateSpp(obeys=False))
        self.assertFalse(st.stopped)
        self.assertIn("still busy", "; ".join(throughput.defects(st, SPANNING, INQ_32HZ)))


class LoadStreamChecksTheLoad(unittest.TestCase):
    """A figure from another packet size is not comparable with the baseline,
    so the load's divider and channel set are both checked before streaming."""

    class Link:
        def __init__(self, divider, channels):
            self.inq = bt.Inquiry(divider, list(channels), 0, b"")
            self.streamed = False

        def command(self, op, args=b""):
            pass

        def inquiry(self):
            return self.inq

        def stream(self, seconds, inq):
            self.streamed = True
            return bt.StreamStats(packets=1)

        def wait_status(self, mask, want, timeout_s=10.0):
            return 0

    def test_the_load_streams(self):
        link = self.Link(throughput.LOAD_DIVIDER, sorted(throughput.LOAD_CHANNELS))
        st, inq = throughput.load_stream(link, 1.0)
        self.assertTrue(link.streamed)
        self.assertEqual(len(inq.channels), 13)

    def test_a_missing_sensor_stops_it_before_streaming(self):
        link = self.Link(throughput.LOAD_DIVIDER, sorted(throughput.LOAD_CHANNELS - {0x04, 0x05, 0x06}))
        with self.assertRaisesRegex(bt.BtError, "channels"):
            throughput.load_stream(link, 1.0)
        self.assertFalse(link.streamed)

    def test_an_extra_sensor_stops_it_too(self):
        link = self.Link(throughput.LOAD_DIVIDER, [*sorted(throughput.LOAD_CHANNELS), 0x1C])
        with self.assertRaisesRegex(bt.BtError, "channels"):
            throughput.load_stream(link, 1.0)

    def test_a_wrong_rate_stops_it(self):
        with self.assertRaisesRegex(bt.BtError, "divider"):
            throughput.load_stream(self.Link(64, sorted(throughput.LOAD_CHANNELS)), 1.0)


class ThroughputJudgement(unittest.TestCase):
    BASE = {"link_bytes_per_s": 28900.0, "stream_samples_per_s": 842.0}

    def test_samples_per_s_is_the_rate_times_the_slots_that_arrived(self):
        st = bt.StreamStats(packets=750, missing=250)
        self.assertAlmostEqual(throughput.samples_per_s(st, bt.Inquiry(32, [], 0, b"")), 768.0)

    def test_within_tolerance_passes(self):
        short, gain = throughput.compare({"link_bytes_per_s": 28000, "stream_samples_per_s": 830}, self.BASE)
        self.assertEqual((short, gain), ([], []))

    def test_more_than_five_percent_below_fails(self):
        # The DEV-1222 #119 figure against main's.
        short, _ = throughput.compare({"link_bytes_per_s": 28900, "stream_samples_per_s": 759}, self.BASE)
        self.assertEqual(len(short), 1)
        self.assertIn("stream_samples_per_s 759 is 9.9% below", short[0])

    def test_above_is_noted_not_failed(self):
        short, note = throughput.judge({"link_bytes_per_s": 33000, "stream_samples_per_s": 842}, self.BASE, "u")
        self.assertEqual(short, [])
        self.assertIn("above the baseline", note)

    def test_a_figure_without_a_baseline_is_not_judged(self):
        short, _ = throughput.compare({"link_bytes_per_s": 1, "stream_samples_per_s": 842},
                                      {"stream_samples_per_s": 842.0})
        self.assertEqual(short, [])

    def test_a_shortfall_says_to_confirm_it_interleaved(self):
        short, _ = throughput.judge({"link_bytes_per_s": 20000, "stream_samples_per_s": 842}, self.BASE, "u")
        self.assertIn("interleaved A/B", short[0])

    def test_the_figures_print_as_a_rig_toml_line(self):
        line = throughput.as_toml({"link_bytes_per_s": 28900, "stream_samples_per_s": 842.4})
        self.assertEqual(line, "throughput = { link_bytes_per_s = 28900, stream_samples_per_s = 842.4 }")
        unit = load_rig('[[unit]]\nname = "a"\nplatform = "shimmer3"\n' + line + "\n").units[0]
        self.assertEqual(unit.throughput, {"link_bytes_per_s": 28900.0, "stream_samples_per_s": 842.4})


class ThroughputBaselineConfig(unittest.TestCase):
    UNIT = '[[unit]]\nname = "a"\nplatform = "shimmer3"\n'

    def test_no_baseline_is_empty(self):
        self.assertEqual(load_rig(self.UNIT).units[0].throughput, {})

    def test_an_unknown_key_is_refused(self):
        with self.assertRaisesRegex(config.RigError, "throughput.link_kbps is not one of"):
            load_rig(self.UNIT + "throughput = { link_kbps = 28.9 }\n")

    def test_a_figure_must_be_a_positive_number(self):
        for bad in ("0", "-5", "true", '"842"'):
            with self.subTest(bad=bad), self.assertRaisesRegex(config.RigError, "positive number"):
                load_rig(self.UNIT + f"throughput = {{ stream_samples_per_s = {bad} }}\n")

    def test_it_must_be_a_table(self):
        with self.assertRaisesRegex(config.RigError, "must be a table"):
            load_rig(self.UNIT + "throughput = 842\n")

    def test_a_tolerance_is_a_fraction_below_one(self):
        self.assertEqual(load_rig(self.UNIT + "throughput = { link_bytes_per_s = 1, tolerance = 0.1 }\n")
                         .units[0].throughput, {"link_bytes_per_s": 1.0, "tolerance": 0.1})
        with self.assertRaisesRegex(config.RigError, "fraction below 1"):
            load_rig(self.UNIT + "throughput = { link_bytes_per_s = 1, tolerance = 1.5 }\n")

    def test_a_tolerance_with_no_figure_to_judge_is_refused(self):
        # It would let the item pass without comparing anything.
        with self.assertRaisesRegex(config.RigError, "has no figure"):
            load_rig(self.UNIT + "throughput = { tolerance = 0.1 }\n")


class ThroughputByRadio(unittest.TestCase):
    """A Shimmer3R's link rate follows its IF820 module build, so a bench keeps
    baselines per build and each unit's `radio` picks its row."""

    RIG = ('[throughput_by_radio]\n'
           '"IF820 v1.4.16.16" = { link_bytes_per_s = 156672, tolerance = 0.10 }\n'
           '"IF820 v1.4.18.18" = { link_bytes_per_s = 77824, tolerance = 0.10 }\n'
           '[[unit]]\nname = "a"\nplatform = "shimmer3r"\nradio = "IF820 v1.4.16.16"\n'
           '[[unit]]\nname = "b"\nplatform = "shimmer3r"\nradio = "IF820 v1.4.18.18"\n'
           'throughput = { stream_samples_per_s = 1020, tolerance = 0.05 }\n'
           '[[unit]]\nname = "c"\nplatform = "shimmer3r"\nradio = "IF820 v1.4.17.17"\n')

    def test_a_unit_takes_its_radio_builds_row(self):
        rig = load_rig(self.RIG)
        self.assertEqual(rig.throughput_baseline(rig.units[0]), {"link_bytes_per_s": 156672, "tolerance": 0.10})

    def test_the_units_own_figures_win_key_by_key(self):
        rig = load_rig(self.RIG)
        self.assertEqual(rig.throughput_baseline(rig.units[1]),
                         {"link_bytes_per_s": 77824, "stream_samples_per_s": 1020, "tolerance": 0.05})

    def test_a_build_with_no_row_has_no_baseline(self):
        rig = load_rig(self.RIG)
        self.assertEqual(rig.throughput_baseline(rig.units[2]), {})

    def test_a_row_is_checked_like_a_units_baseline(self):
        with self.assertRaisesRegex(config.RigError, "positive number"):
            load_rig('[throughput_by_radio]\n"IF820 v1.4.16.16" = { link_bytes_per_s = -1 }\n')

    def test_a_row_with_only_a_tolerance_is_refused(self):
        with self.assertRaisesRegex(config.RigError, "has no figure"):
            load_rig('[throughput_by_radio]\n"IF820 v1.4.16.16" = { tolerance = 0.1 }\n')

    def test_a_units_own_tolerance_may_ride_on_its_radio_rows_figures(self):
        rig = load_rig(self.RIG.replace('throughput = { stream_samples_per_s = 1020, tolerance = 0.05 }',
                                        'throughput = { tolerance = 0.05 }'))
        self.assertEqual(rig.throughput_baseline(rig.units[1]), {"link_bytes_per_s": 77824, "tolerance": 0.05})

    def test_a_baseline_with_no_figure_is_no_baseline(self):
        # However it came about, nothing to compare is not_covered, never a pass.
        unit = config.Unit(name="u", platform="shimmer3", throughput={"tolerance": 0.1})
        rig = config.Rig(path=Path("rig.toml"), bench="b", units=[unit])
        self.assertEqual(rig.throughput_baseline(unit), {})

    def test_a_rows_own_tolerance_replaces_the_default(self):
        # DEV-573: one test build read 168.8-170.3 KB/s one day and 152-161 two days later.
        base = {"link_bytes_per_s": 168.8 * 1024, "tolerance": 0.10}
        self.assertEqual(throughput.compare({"link_bytes_per_s": 152 * 1024, "stream_samples_per_s": 1}, base),
                         ([], []))
        short, _ = throughput.compare({"link_bytes_per_s": 150 * 1024, "stream_samples_per_s": 1}, base)
        self.assertEqual(len(short), 1)

    def test_the_module_version_must_match_the_named_build(self):
        self.assertIsNone(throughput.radio_mismatch("IF820 v1.4.18.18 test build", "1.4.18.18"))
        # What the Shimmer3R dock reports, zero-padded.
        reported = "CYW20820 app=v01.04.18.18, stack=0x00000000, protocol=0x0104, hardware=0xf1"
        self.assertIsNone(throughput.radio_mismatch("IF820 v1.4.18.18", reported))
        self.assertIn("update its radio", throughput.radio_mismatch("IF820 v1.4.16.16", reported))
        self.assertIn("update its radio", throughput.radio_mismatch("IF820 v1.4.16.16", "v1.4.18.18"))
        self.assertIsNone(throughput.radio_mismatch(None, "1.4.18.18"))
        self.assertIsNone(throughput.radio_mismatch("IF820 v1.4.16.16", "unreadable"))


class BleBaselineConfig(unittest.TestCase):
    RIG = (
        '[throughput_by_radio]\n"IF820 v1.4.18.18 test_254" = { link_bytes_per_s = 172851, tolerance = 0.10 }\n'
        '[ble_throughput_by_radio]\n'
        '"IF820 v1.4.18.18 test_254" = { link_bytes_per_s = 112640, stream_samples_per_s = 1024, tolerance = 0.10 }\n'
        + S3R_UNIT.format(n="a") + 'mac = "CF9B9E3615BC"\nradio = "IF820 v1.4.18.18 test_254"\n'
        + S3R_UNIT.format(n="b") + 'mac = "CF9B9E3615BD"\nradio = "IF820 v1.4.18.18 test_254"\n'
        'ble_throughput = { stream_samples_per_s = 1000 }\n')

    def test_classic_and_ble_baselines_are_kept_apart(self):
        rig = load_rig(self.RIG)
        a, b = rig.units
        self.assertEqual(rig.throughput_baseline(a), {"link_bytes_per_s": 172851, "tolerance": 0.10})
        self.assertEqual(rig.ble_throughput_baseline(a),
                         {"link_bytes_per_s": 112640, "stream_samples_per_s": 1024, "tolerance": 0.10})
        # A unit's own BLE figure overrides its radio row's, key by key, as for Classic.
        self.assertEqual(rig.ble_throughput_baseline(b)["stream_samples_per_s"], 1000)
        self.assertEqual(rig.ble_throughput_baseline(b)["link_bytes_per_s"], 112640)
        self.assertNotIn("ble_throughput_by_radio", rig.extra)

    def test_a_ble_row_is_checked_like_a_classic_one(self):
        with self.assertRaisesRegex(config.RigError, "ble_throughput_by_radio.*has no figure"):
            load_rig('[ble_throughput_by_radio]\n"IF820 v1.4.16.16" = { tolerance = 0.1 }\n')
        with self.assertRaisesRegex(config.RigError, "ble_throughput has no figure"):
            load_rig(S3R_UNIT.format(n="a") + 'radio = "IF820 v1.4.16.16"\nble_throughput = { tolerance = 0.1 }\n')

    def test_ble_is_a_capability_only_with_bleak_and_only_on_a_shimmer3r(self):
        s3r = config.Unit(name="r", platform="shimmer3r", mac="CF9B9E3615BC")
        s3 = config.Unit(name="s", platform="shimmer3", mac="000666BAD1E8")
        for have in (True, False):
            with self.subTest(bleak=have), mock.patch.object(config, "ble_available", lambda: have):
                self.assertEqual("ble" in s3r.capabilities(), have)
                self.assertNotIn("ble", s3.capabilities())

    def test_an_rn4678_shimmer3_has_ble_and_an_rn42_does_not(self):
        rn4678 = config.Unit(name="a", platform="shimmer3", mac="44B7D02FA615", radio="RN4678 V1.23")
        rn42 = config.Unit(name="b", platform="shimmer3", mac="000666BAD1E8", radio="RN42 Ver 6.15")
        with mock.patch.object(config, "ble_available", lambda: True):
            self.assertIn("ble", rn4678.capabilities())
            self.assertNotIn("ble", rn42.capabilities())

    def test_each_module_family_has_its_own_ble_service(self):
        self.assertIs(ble.profile_for("IF820 v1.4.18.18 test_254"), ble.CYSPP)
        self.assertIs(ble.profile_for("RN4678 V1.00.5"), ble.MCHP_TRANSPARENT)
        self.assertIsNone(ble.profile_for("RN42 Ver 4.77"))
        self.assertIsNone(ble.profile_for(None))
        self.assertNotEqual(ble.MCHP_TRANSPARENT.notify, ble.MCHP_TRANSPARENT.write)

    def test_the_figures_to_record_name_the_ble_table(self):
        line = throughput.as_toml({"link_bytes_per_s": 115000, "stream_samples_per_s": 1024.0}, "ble_throughput")
        self.assertEqual(line, "ble_throughput = { link_bytes_per_s = 115000, stream_samples_per_s = 1024 }")


class BleEnableBit(unittest.TestCase):
    def test_the_radio_page_is_read_from_its_eeprom_address(self):
        d = dock.DockLink.__new__(dock.DockLink)
        sent = []
        d._transact = lambda cmd, comp, prop, data=b"": sent.append((cmd, comp, prop, data)) or bytes(range(3, 19))
        self.assertEqual(d.radio_settings(), bytes(range(3, 19)))
        # CARD_MEM reads EEPROM from offset + 16, so the page at 2032 is offset 2016, 16 bytes.
        self.assertEqual(sent, [(dock.CMD_GET, dock.COMP_DAUGHTER_CARD, dock.PROP_CARD_MEM, bytes([16, 0xE0, 0x07]))])

    def test_a_radio_page_of_the_wrong_length_is_an_error(self):
        d = dock.DockLink.__new__(dock.DockLink)
        for n in (3, 15, 17):
            with self.subTest(n=n):
                d._transact = lambda cmd, comp, prop, data=b"", n=n: bytes([3, 12, 0xFF]) + bytes(n - 3)
                with self.assertRaisesRegex(dock.DockError, f"gave {n} bytes"):
                    d.radio_settings()

    def test_ble_enabled_reads_byte_2_bit_0(self):
        self.assertTrue(dock.ble_enabled(bytes([3, 12, 0xFF]) + bytes(13)))   # 15BC: unprogrammed, both on
        self.assertTrue(dock.ble_enabled(bytes([3, 12, 0x01]) + bytes(13)))
        self.assertFalse(dock.ble_enabled(bytes([3, 12, 0x02]) + bytes(13)))  # classic only
        self.assertFalse(dock.ble_enabled(b""))

def _ble_serial(timeout=0.2):
    """A BleSerial with no radio behind it: notifications are fed by hand."""
    s = ble.BleSerial.__new__(ble.BleSerial)
    s.timeout, s.address, s.mtu, s.flow_indications = timeout, "CF:9B:9E:36:15:BC", 247, 0
    s.write_size = s.mtu - ble.ATT_HEADER
    s.profile = ble.CYSPP
    s._buf, s._cond, s._lost, s._client = bytearray(), threading.Condition(), False, object()
    return s


class BleSerialDescribe(unittest.TestCase):
    """The connection interval sets an RN4678's BLE rate, so the item's note carries it."""

    def test_the_interval_windows_reports_is_named(self):
        from types import SimpleNamespace
        s = _ble_serial()
        params = SimpleNamespace(connection_interval=48)          # 1.25 ms units
        s._client = SimpleNamespace(_backend=SimpleNamespace(
            _requester=SimpleNamespace(get_connection_parameters=lambda: params)))
        self.assertIn("connection interval 60 ms", s.describe())

    def test_no_interval_where_the_platform_gives_none(self):
        self.assertNotIn("interval", _ble_serial().describe())


class BleSerialReads(unittest.TestCase):
    def test_a_read_waits_for_the_bytes_asked_for(self):
        s = _ble_serial(timeout=2.0)
        threading.Timer(0.05, s._on_data, (None, bytearray(b"\xff\x25"))).start()
        threading.Timer(0.10, s._on_data, (None, bytearray(b"\x0a"))).start()
        self.assertEqual(s.read(3), b"\xff\x25\x0a")

    def test_a_read_returns_what_arrived_when_the_timeout_runs_out(self):
        s = _ble_serial(timeout=0.1)
        s._on_data(None, bytearray(b"\xa5\x00"))
        self.assertEqual(s.in_waiting, 2)
        self.assertEqual(s.read(5), b"\xa5\x00")
        self.assertEqual(s.read(5), b"")

    def test_what_arrived_before_a_drop_can_still_be_read(self):
        s = _ble_serial()
        s._on_data(None, bytearray(b"\xff"))
        s._on_disconnect(None)
        self.assertFalse(s.is_open)
        self.assertEqual(s.read(4), b"\xff")
        with self.assertRaises(ble.BleError):
            s.read(1)
        with self.assertRaises(ble.BleError):
            s.write(b"\x2e")

    def test_close_wakes_a_reader_waiting_with_no_timeout(self):
        s = _ble_serial(timeout=None)
        s._loop = asyncio.new_event_loop()
        s._thread = threading.Thread(target=lambda: None)
        s._thread.start()
        raised = []

        def reader():
            try:
                s.read(1)
            except ble.BleError as e:
                raised.append(e)

        t = threading.Thread(target=reader, daemon=True)
        t.start()
        time.sleep(0.1)
        s.close()
        t.join(2.0)
        self.assertFalse(t.is_alive(), "read() still blocked after close()")
        self.assertEqual(len(raised), 1)

    def _running_loop(self, s):
        s._loop = asyncio.new_event_loop()
        s._thread = threading.Thread(target=s._loop.run_forever, daemon=True)
        s._thread.start()

    def test_a_timed_out_operation_is_cancelled(self):
        s = _ble_serial()
        self._running_loop(s)
        cancelled = threading.Event()

        async def slow():
            try:
                await asyncio.sleep(10)
            except asyncio.CancelledError:
                cancelled.set()
                raise

        with self.assertRaises(ble.BleError):
            s._call(slow(), 0.1)
        self.assertTrue(cancelled.wait(2.0), "the timed-out operation is still running")
        s.close()
        self.assertTrue(s._loop.is_closed())

    def test_a_bleak_failure_comes_back_as_ble_error(self):
        s = _ble_serial()
        self._running_loop(s)

        async def fails():
            raise OSError("adapter gone")

        with self.assertRaisesRegex(ble.BleError, "OSError: adapter gone"):
            s._call(fails(), 1.0)
        s.close()

    def test_close_cancels_what_is_still_in_flight(self):
        s = _ble_serial()
        self._running_loop(s)
        cancelled = threading.Event()

        async def slow():
            try:
                await asyncio.sleep(10)
            except asyncio.CancelledError:
                cancelled.set()
                raise

        asyncio.run_coroutine_threadsafe(slow(), s._loop)
        time.sleep(0.05)
        s.close()
        self.assertTrue(cancelled.is_set())
        self.assertTrue(s._loop.is_closed())

    def test_reset_input_buffer_drops_what_is_queued(self):
        s = _ble_serial()
        s._on_data(None, bytearray(b"\x01\x02"))
        s.reset_input_buffer()
        self.assertEqual(s.in_waiting, 0)

    def test_a_write_goes_to_the_profiles_write_characteristic(self):
        s = _ble_serial()
        s.profile = ble.MCHP_TRANSPARENT
        sent = []

        class Client:
            def write_gatt_char(self, char, data, response):
                return (char, data, response)

        s._client = Client()
        s._call = lambda call, _timeout: sent.append(call)
        s.write(b"\x2e")
        self.assertEqual(sent, [(ble.MCHP_TRANSPARENT.write, b"\x2e", False)])

    def test_a_write_is_split_to_fit_the_att_mtu(self):
        s = _ble_serial()
        s.write_size = 20
        sent = []

        class Client:
            def write_gatt_char(self, char, data, response):
                return (char, data, response)

        s._client = Client()
        s._call = lambda call, _timeout: sent.append(call)
        self.assertEqual(s.write(bytes(range(45))), 45)
        self.assertEqual([len(d) for _, d, _ in sent], [20, 20, 5])
        self.assertTrue(all(c == ble.UNACKED_DATA and r is False for c, _, r in sent))

    def test_btlink_runs_over_any_link_with_the_serial_interface(self):
        # BtLink's first exchange (SET_CRC 0, then GET_DEVICE_VERSION) over the BLE adapter.
        s = _ble_serial(timeout=1.0)
        written = []

        def write(data):
            written.append(bytes(data))
            if data == bytes([bt.GET_DEVICE_VERSION]):
                s._on_data(None, bytearray([bt.ACK, bt.DEVICE_VERSION_RSP, bt.HW_SHIMMER3R]))
            return len(data)

        s.write = write
        with mock.patch.object(bt.time, "sleep", lambda _s: None):
            link = bt.BtLink(ser=s)
        self.assertEqual(link.hw, bt.HW_SHIMMER3R)
        self.assertEqual(written[0], bytes([bt.SET_CRC, 0]))
        self.assertIs(link.ser, s)


if __name__ == "__main__":
    unittest.main(verbosity=2)
