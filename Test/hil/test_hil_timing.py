#!/usr/bin/env python3
"""Unit tests for rig.bt.classify_step: how timestamp steps become lost samples,
duplicates and jitter. Run: python Test/hil/test_hil_timing.py (no hardware).

A miscount here fails release rows for the wrong reason, or passes them, so the
cases include the one that did: a stamp exactly half a period late, which an
earlier step-by-step rounding counted as a lost sample plus a duplicate.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
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


if __name__ == "__main__":
    unittest.main(verbosity=2)
