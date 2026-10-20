#!/usr/bin/env python3
"""Unit tests for rig.usb_throughput and its baseline: what smoke.usb_throughput
measures and how it is judged. Run: python Test/hil/test_hil_usb_throughput.py
(no hardware).

The transfers go through an ordinary file here (usb_throughput._Buffered), so
the figures are meaningless, but everything around them is not: the data must
come back as it went in, the file must be gone afterwards whatever happened,
and only the time spent transferring counts.

No hardware and no pyserial needed: rig.config imports rig.ble, which is fine
without bleak, and nothing here imports pyserial.
"""

import sys
import tempfile
import textwrap
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rig import config, throughput, usb_throughput  # noqa: E402

MIB = 1024 * 1024
KEYS = usb_throughput.USB_THROUGHPUT_KEYS


class Clock:
    """perf_counter stand-in: each call is `step` seconds after the last."""

    def __init__(self, step):
        self.t, self.step = 0.0, step

    def __call__(self):
        self.t += self.step
        return self.t


def corrupting(opener):
    """An opener whose reads come back with one byte flipped."""
    class Corrupt(opener):
        def read(self, n):
            data = bytearray(super().read(n))
            if data:
                data[0] ^= 0xFF
            return bytes(data)
    return Corrupt


def truncating(opener):
    """An opener whose reads stop after the first chunk."""
    class Short(opener):
        calls = 0

        def read(self, n):
            Short.calls += 1
            return super().read(n) if Short.calls == 1 else b""
    return Short


class Measure(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.root = Path(self.dir.name)

    def tearDown(self):
        self.dir.cleanup()

    def test_round_trip_reports_both_rates_and_removes_the_file(self):
        r = usb_throughput.measure(self.root, size=4 * MIB, chunk=MIB, opener=usb_throughput._Buffered)
        self.assertEqual(sorted(r.figures()), sorted(KEYS))
        self.assertEqual((r.write.nbytes, r.read.nbytes), (4 * MIB, 4 * MIB))
        self.assertFalse((self.root / usb_throughput.FILE_NAME).exists())

    def test_only_transfer_time_counts(self):
        # Each clock reading is 0.25 s after the last, and each timed call takes
        # two readings: 4 chunk writes and a flush, then 4 chunk reads.
        r = usb_throughput.measure(self.root, size=4 * MIB, chunk=MIB, opener=usb_throughput._Buffered,
                                   clock=Clock(0.25))
        self.assertAlmostEqual(r.write.seconds, 5 * 0.25)
        self.assertAlmostEqual(r.read.seconds, 4 * 0.25)
        self.assertEqual(r.figures()["usb_read_bytes_per_s"], round(4 * MIB / 1.0))

    def test_data_that_comes_back_different_fails_and_leaves_nothing(self):
        with self.assertRaisesRegex(usb_throughput.UsbThroughputError, "differ"):
            usb_throughput.measure(self.root, size=2 * MIB, chunk=MIB,
                                   opener=corrupting(usb_throughput._Buffered))
        self.assertFalse((self.root / usb_throughput.FILE_NAME).exists())

    def test_a_short_read_fails(self):
        with self.assertRaisesRegex(usb_throughput.UsbThroughputError, "read back 1048576 of 3145728"):
            usb_throughput.measure(self.root, size=3 * MIB, chunk=MIB,
                                   opener=truncating(usb_throughput._Buffered))
        self.assertFalse((self.root / usb_throughput.FILE_NAME).exists())

    def test_a_full_drive_is_refused_before_writing(self):
        usage = mock.Mock(free=3 * MIB)
        with mock.patch.object(usb_throughput.shutil, "disk_usage", return_value=usage):
            with self.assertRaisesRegex(usb_throughput.UsbThroughputError, "bytes free"):
                usb_throughput.measure(self.root, size=2 * MIB, chunk=MIB, opener=usb_throughput._Buffered)
        self.assertFalse((self.root / usb_throughput.FILE_NAME).exists())

    def test_size_must_be_whole_chunks(self):
        with self.assertRaises(ValueError):
            usb_throughput.measure(self.root, size=MIB + 1, chunk=MIB, opener=usb_throughput._Buffered)


class Judge(unittest.TestCase):
    MEASURED = {"usb_write_bytes_per_s": 9.0e6, "usb_read_bytes_per_s": 20.0e6}

    def test_a_shortfall_past_the_tolerance_fails(self):
        base = {"usb_write_bytes_per_s": 10.0e6, "usb_read_bytes_per_s": 20.0e6, "tolerance": 0.05}
        short, note = throughput.judge(self.MEASURED, base, "s3r-a", KEYS)
        self.assertEqual(len(short), 1)
        self.assertIn("usb_write_bytes_per_s 9e+06 is 10.0% below the baseline 1e+07", short[0])
        self.assertIn("usb_read_bytes_per_s 2e+07 (baseline 2e+07)", note)

    def test_within_tolerance_passes_and_a_gain_is_noted(self):
        base = {"usb_write_bytes_per_s": 9.2e6, "usb_read_bytes_per_s": 15.0e6, "tolerance": 0.1}
        short, note = throughput.judge(self.MEASURED, base, "s3r-a", KEYS)
        self.assertEqual(short, [])
        self.assertIn("record the new figures for s3r-a", note)

    def test_the_bluetooth_keys_are_still_the_default(self):
        short, _ = throughput.judge({"link_bytes_per_s": 50.0, "stream_samples_per_s": 900.0},
                                    {"link_bytes_per_s": 100.0}, "u")
        self.assertEqual(len(short), 1)

    def test_baseline_line_and_full_speed_note(self):
        self.assertEqual(usb_throughput.as_toml(self.MEASURED),
                         "usb_throughput = { usb_write_bytes_per_s = 9e+06, usb_read_bytes_per_s = 2e+07 }")
        self.assertIsNone(usb_throughput.speed_note(self.MEASURED))
        self.assertIn("Full Speed", usb_throughput.speed_note({"usb_write_bytes_per_s": 0.9e6,
                                                               "usb_read_bytes_per_s": 1.0e6}))


class RigBaseline(unittest.TestCase):
    def load(self, unit_lines):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "rig.toml"
            p.write_text('bench = "b"\n\n[[unit]]\n' + textwrap.dedent(unit_lines), encoding="utf-8")
            return config.load(p)

    def test_a_shimmer3r_takes_its_own_usb_baseline(self):
        rig = self.load('''\
            name = "s3r-a"
            platform = "shimmer3r"
            usb_serial = "abc"
            usb_throughput = { usb_write_bytes_per_s = 8e6, usb_read_bytes_per_s = 2e7, tolerance = 0.15 }
            ''')
        self.assertEqual(rig.units[0].usb_throughput,
                         {"usb_write_bytes_per_s": 8e6, "usb_read_bytes_per_s": 2e7, "tolerance": 0.15})

    def test_a_bluetooth_key_in_the_usb_baseline_is_refused(self):
        with self.assertRaisesRegex(config.RigError, "usb_throughput.link_bytes_per_s is not one of"):
            self.load('''\
                name = "s3r-a"
                platform = "shimmer3r"
                usb_throughput = { link_bytes_per_s = 1000 }
                ''')

    def test_a_shimmer3_cannot_have_one(self):
        with self.assertRaisesRegex(config.RigError, "no USB-C"):
            self.load('''\
                name = "s3-a"
                platform = "shimmer3"
                usb_throughput = { usb_read_bytes_per_s = 1e6 }
                ''')


if __name__ == "__main__":
    unittest.main()
