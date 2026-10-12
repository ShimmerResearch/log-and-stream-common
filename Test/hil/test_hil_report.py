#!/usr/bin/env python3
"""Unit tests for hil_report.py: how test outcomes become report items.
Run: python Test/hil/test_hil_report.py  (no hardware)
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hil_gate  # noqa: E402
import hil_report  # noqa: E402

SHA = "0123456789abcdef0123456789abcdef01234567"


def make(col):
    return col.report(platform="shimmer3r", commit=SHA, firmware_version="1.01.022",
                      bench="b", operator="o")


class Aggregation(unittest.TestCase):
    def status(self, *outcomes):
        c = hil_report.Collector()
        for o in outcomes:
            c.record("x", o)
        return make(c)["results"][0]["status"]

    def test_all_pass(self):
        self.assertEqual(self.status("pass", "pass"), "pass")

    def test_one_failure_wins(self):
        self.assertEqual(self.status("pass", "fail", "not_covered"), "fail")

    def test_partial_coverage_is_not_a_pass(self):
        # One unit ran, another could not: the item is not fully covered.
        self.assertEqual(self.status("pass", "not_covered"), "not_covered")

    def test_order_does_not_matter(self):
        self.assertEqual(self.status("fail", "pass"), "fail")

    def test_unknown_status_rejected(self):
        with self.assertRaises(ValueError):
            hil_report.Collector().record("x", "ok")


class Report(unittest.TestCase):
    def test_report_is_what_the_gate_accepts(self):
        c = hil_report.Collector()
        c.record("smoke.a", "pass", "u1: fine")
        c.record("smoke.b", "not_covered", "u1: bench lacks: hub")
        rep = make(c)
        self.assertEqual(hil_gate.validate_report("r", rep), [])
        self.assertEqual(rep["kind"], "automated")

    def test_notes_are_bounded(self):
        c = hil_report.Collector()
        for i in range(10):
            c.record("x", "pass", f"unit{i}: " + "y" * 200)
        note = make(c)["results"][0]["note"]
        self.assertLessEqual(len(note), 500)

    def test_bad_commit_refused_at_write_time(self):
        c = hil_report.Collector()
        c.record("x", "pass")
        with self.assertRaises(ValueError):
            c.report(platform="shimmer3r", commit="abc", firmware_version="1", bench="b", operator="o")


if __name__ == "__main__":
    unittest.main(verbosity=2)
