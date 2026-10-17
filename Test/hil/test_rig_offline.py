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
    sys.modules.update({"serial": _serial, "serial.tools": _tools, "serial.tools.list_ports": _lp})

from rig import bt, flash, power  # noqa: E402
import run_rc  # noqa: E402


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


if __name__ == "__main__":
    unittest.main(verbosity=2)
