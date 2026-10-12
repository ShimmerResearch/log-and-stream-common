#!/usr/bin/env python3
"""Unit tests for hil_gate.py. Run: python Test/hil/test_hil_gate.py

The gate is the only thing standing between a release candidate and the public,
so each rule it enforces has a case here that would pass a bad candidate if the
rule were removed.
"""

import copy
import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hil_gate  # noqa: E402

SHA = "0123456789abcdef0123456789abcdef01234567"
OTHER_SHA = "fedcba9876543210fedcba9876543210fedcba98"
REQUIRED = [{"id": "a.one", "title": "One"}, {"id": "a.two", "title": "Two"}]


def report(results, **over):
    r = {
        "schema": hil_gate.REPORT_SCHEMA,
        "kind": "manual",
        "platform": "shimmer3r",
        "commit": SHA,
        "firmware_version": "v1.01.022",
        "bench": "bench-1",
        "operator": "tester",
        "created": "2026-10-12T10:00:00Z",
        "results": results,
    }
    r.update(over)
    return r


def ok(rid):
    return {"id": rid, "status": "pass"}


def gate(reports, required=REQUIRED, version="1.01.022", commit=SHA):
    return hil_gate.evaluate("shimmer3r", commit, version, reports, required)


class Rules(unittest.TestCase):
    def test_all_required_passed(self):
        v = gate({"r.json": report([ok("a.one"), ok("a.two")])})
        self.assertTrue(v.ok, v.problems)

    def test_no_reports_blocks(self):
        v = gate({})
        self.assertFalse(v.ok)

    def test_missing_required_item_blocks(self):
        v = gate({"r.json": report([ok("a.one")])})
        self.assertFalse(v.ok)
        self.assertTrue(any("a.two" in p for p in v.problems))

    def test_coverage_may_come_from_several_reports(self):
        v = gate({"r1.json": report([ok("a.one")], bench="b1"),
                  "r2.json": report([ok("a.two")], bench="b2")})
        self.assertTrue(v.ok, v.problems)

    def test_not_covered_is_not_a_pass(self):
        v = gate({"r.json": report([ok("a.one"), {"id": "a.two", "status": "not_covered"}])})
        self.assertFalse(v.ok)

    def test_failure_blocks_even_if_another_bench_passed(self):
        v = gate({"r1.json": report([ok("a.one"), ok("a.two")]),
                  "r2.json": report([{"id": "a.two", "status": "fail", "note": "drops"}])})
        self.assertFalse(v.ok)
        self.assertTrue(any("FAILED" in p and "drops" in p for p in v.problems))

    def test_failure_in_an_unrequired_item_blocks(self):
        v = gate({"r.json": report([ok("a.one"), ok("a.two"),
                                    {"id": "extra.x", "status": "fail"}])})
        self.assertFalse(v.ok)

    def test_waived_failure_passes_and_covers(self):
        v = gate({"r.json": report([ok("a.one"),
                                    {"id": "a.two", "status": "fail", "waiver": "DEV-1"}])})
        self.assertTrue(v.ok, v.problems)
        self.assertIn("DEV-1", v.covered["a.two"])

    def test_waived_not_covered_covers(self):
        v = gate({"r.json": report([ok("a.one"),
                                    {"id": "a.two", "status": "not_covered", "waiver": "DEV-2"}])})
        self.assertTrue(v.ok, v.problems)

    def test_waiver_must_be_an_issue_key(self):
        v = gate({"r.json": report([ok("a.one"),
                                    {"id": "a.two", "status": "fail", "waiver": "it's fine"}])})
        self.assertFalse(v.ok)


class Binding(unittest.TestCase):
    """A report is evidence about one build only."""

    def test_wrong_commit_blocks(self):
        v = gate({"r.json": report([ok("a.one"), ok("a.two")], commit=OTHER_SHA)})
        self.assertFalse(v.ok)

    def test_one_stale_report_rejects_the_set(self):
        v = gate({"good.json": report([ok("a.one"), ok("a.two")]),
                  "stale.json": report([ok("a.one")], commit=OTHER_SHA)})
        self.assertFalse(v.ok)

    def test_short_sha_rejected(self):
        v = gate({"r.json": report([ok("a.one"), ok("a.two")], commit=SHA[:12])})
        self.assertFalse(v.ok)

    def test_wrong_version_blocks(self):
        v = gate({"r.json": report([ok("a.one"), ok("a.two")], firmware_version="1.01.021")})
        self.assertFalse(v.ok)

    def test_version_v_prefix_ignored(self):
        v = gate({"r.json": report([ok("a.one"), ok("a.two")])}, version="v1.01.022")
        self.assertTrue(v.ok, v.problems)

    def test_commit_case_ignored_on_candidate_side(self):
        v = gate({"r.json": report([ok("a.one"), ok("a.two")])}, commit=SHA.upper())
        self.assertTrue(v.ok, v.problems)

    def test_wrong_platform_blocks(self):
        v = gate({"r.json": report([ok("a.one"), ok("a.two")], platform="shimmer3")})
        self.assertFalse(v.ok)


class Structure(unittest.TestCase):
    def test_bad_schema_blocks(self):
        v = gate({"r.json": report([ok("a.one"), ok("a.two")], schema="something/2")})
        self.assertFalse(v.ok)

    def test_bad_status_blocks(self):
        v = gate({"r.json": report([ok("a.one"), {"id": "a.two", "status": "ok"}])})
        self.assertFalse(v.ok)

    def test_duplicate_id_blocks(self):
        v = gate({"r.json": report([ok("a.one"), ok("a.two"), ok("a.two")])})
        self.assertFalse(v.ok)

    def test_missing_operator_blocks(self):
        r = report([ok("a.one"), ok("a.two")])
        del r["operator"]
        self.assertFalse(gate({"r.json": r}).ok)


class Files(unittest.TestCase):
    def test_shipped_required_lists_load(self):
        for p in hil_gate.PLATFORMS:
            items = hil_gate.load_required(p)
            self.assertTrue(items)

    def test_cli_exit_codes_and_summary(self):
        with tempfile.TemporaryDirectory() as d:
            good = Path(d, "hil-report-a.json")
            reqs = hil_gate.load_required("shimmer3r")
            rep = report([ok(i["id"]) for i in reqs])
            good.write_text(json.dumps(rep), encoding="utf-8")
            args = ["--platform", "shimmer3r", "--commit", SHA, "--version", "1.01.022"]
            buf = io.StringIO()
            with redirect_stdout(buf):
                self.assertEqual(hil_gate.main(args + [str(good)]), 0)
            self.assertIn("PASSED", buf.getvalue())

            bad = copy.deepcopy(rep)
            bad["results"] = bad["results"][1:]
            Path(d, "hil-report-b.json").write_text(json.dumps(bad), encoding="utf-8")
            with redirect_stdout(io.StringIO()):
                self.assertEqual(hil_gate.main(args + [str(Path(d, "hil-report-b.json"))]), 1)
                self.assertEqual(hil_gate.main(args), 1)

            Path(d, "broken.json").write_text("{", encoding="utf-8")
            with redirect_stdout(io.StringIO()):
                self.assertEqual(hil_gate.main(args + [str(Path(d, "broken.json"))]), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
