#!/usr/bin/env python3
"""Unit tests for rig.bt.classify_step: how timestamp steps become lost samples,
duplicates and jitter. Run: python Test/hil/test_hil_timing.py (no hardware).

A miscount here fails release rows for the wrong reason, or passes them, so the
cases include the one that did: a stamp exactly half a period late, which an
earlier step-by-step rounding counted as a lost sample plus jitter.

No hardware and no pyserial needed: rig.bt imports pyserial, so where it is not
installed (CI) a stand-in is put in its place, as test_rig_offline.py does.
"""

import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

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

from rig.bt import StreamStats, classify_step  # noqa: E402


def run(steps, period):
    st = StreamStats()
    for d in steps:
        classify_step(st, d, period)
    return st


class Steps(unittest.TestCase):
    def test_clean(self):
        st = run([32] * 100, 32)
        self.assertEqual((st.gaps, st.missing, st.dupes, st.jitter), (0, 0, 0, 0))

    def test_half_period_late_is_jitter_not_loss(self):
        # 1024 Hz, period 32: one stamp 16 ticks late -> steps 48 then 16.
        st = run([32, 48, 16, 32], 32)
        self.assertEqual((st.gaps, st.missing, st.dupes), (0, 0, 0))
        self.assertEqual(st.max_jitter_ticks, 16)

    def test_late_up_to_three_quarters(self):
        st = run([100, 170, 30, 100], 100)
        self.assertEqual((st.gaps, st.missing, st.dupes), (0, 0, 0))
        self.assertEqual(st.max_jitter_ticks, 70)

    def test_lost_samples(self):
        st = run([32, 32 * 6, 32], 32)
        self.assertEqual((st.gaps, st.missing, st.dupes), (1, 5, 0))
        self.assertEqual(st.max_gap_ticks, 192)

    def test_late_stamp_after_a_gap(self):
        st = run([64, 64 * 3 + 20, 64 - 20, 64], 64)
        self.assertEqual((st.gaps, st.missing, st.dupes), (1, 2, 0))
        self.assertEqual(st.max_jitter_ticks, 20)

    def test_duplicate_stamp(self):
        st = run([32, 0, 32], 32)
        self.assertEqual(st.dupes, 1)
        self.assertEqual(st.missing, 0)

    def test_one_tick_quantisation_is_not_jitter(self):
        st = run([640, 641, 639, 640], 640)
        self.assertEqual(st.jitter, 0)


class LateByThreeQuartersOrMore(unittest.TestCase):
    """A stamp three quarters of a period late or more is first counted in the
    next slot, the slot before it lost. When the stamp after it lands in that
    same slot, it was a late stamp: one jitter, no loss and no duplicate."""

    def test_at_4096hz_the_dev1464_alarm(self):
        # s3r-c623 on v1.01.026, 12 Nov 2026: the sample due 7 ticks past the
        # second stamped at 13, the next on time at 15. Steps 14 then 2.
        st = run([8, 14, 2, 8], 8)
        self.assertEqual((st.gaps, st.missing, st.dupes, st.max_gap_ticks), (0, 0, 0, 0))
        self.assertEqual((st.jitter, st.max_jitter_ticks), (1, 6))

    def test_at_1024hz_almost_a_period_late(self):
        st = run([32, 32 + 31, 1, 32], 32)
        self.assertEqual((st.gaps, st.missing, st.dupes), (0, 0, 0))
        self.assertEqual((st.jitter, st.max_jitter_ticks), (1, 31))

    def test_late_after_a_lost_sample_keeps_the_loss(self):
        # Slot 2 lost, slot 3 stamped 6 late, slot 4 on time.
        st = run([8, 8 * 2 + 6, 2, 8], 8)
        self.assertEqual((st.gaps, st.missing, st.dupes, st.max_gap_ticks), (1, 1, 0, 22))
        self.assertEqual((st.jitter, st.max_jitter_ticks), (1, 6))

    def test_the_revision_is_returned(self):
        st = StreamStats()
        self.assertEqual([classify_step(st, d, 8) for d in (8, 14, 2, 8)], [None, None, 6, None])

    def test_without_a_stamp_in_the_same_slot_it_stays_a_loss(self):
        # The next stamp a slot on: a sample lost and one stamped a little early.
        st = run([8, 14, 10, 8], 8)
        self.assertEqual((st.gaps, st.missing, st.dupes, st.max_jitter_ticks), (1, 1, 0, 2))

    def test_an_on_time_stamp_after_a_loss_is_not_revised(self):
        # On its slot, not early for it: the stamp in the same slot after it
        # is a duplicate.
        st = run([8, 16, 1, 7], 8)
        self.assertEqual((st.gaps, st.missing, st.dupes), (1, 1, 1))


if __name__ == "__main__":
    unittest.main(verbosity=2)
