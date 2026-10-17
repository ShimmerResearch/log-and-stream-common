#!/usr/bin/env python3
"""Unit tests for hil_gate.py. Run: python Test/hil/test_hil_gate.py

The gate is the only thing standing between a release candidate and the public,
so each rule it enforces has a case here that would pass a bad candidate if the
rule were removed.
"""

import contextlib
import copy
import io
import json
import re
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hil_gate  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
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

    def test_a_gap_on_one_bench_is_covered_by_a_pass_on_another(self):
        # Rule 4: coverage is split across reports. A bench that cannot run an
        # item says so, and that is not a failure.
        v = gate({"r1.json": report([ok("a.one"), {"id": "a.two", "status": "not_covered"}], bench="b1"),
                  "r2.json": report([ok("a.two")], bench="b2")})
        self.assertTrue(v.ok, v.problems)

    def test_a_gap_in_an_unrequired_item_is_listed_not_blocking(self):
        v = gate({"r.json": report([ok("a.one"), ok("a.two"), {"id": "extra.x", "status": "not_covered"}])})
        self.assertTrue(v.ok, v.problems)
        self.assertEqual(v.extra, {"extra.x": "not_covered"})

    def test_waiver_must_be_an_issue_key(self):
        v = gate({"r.json": report([ok("a.one"),
                                    {"id": "a.two", "status": "fail", "waiver": "it's fine"}])})
        self.assertFalse(v.ok)

    def test_waiver_must_be_only_an_issue_key(self):
        for w in ("DEV-1\n", "DEV-1 ok", " DEV-1"):
            with self.subTest(waiver=w):
                v = gate({"r.json": report([ok("a.one"), {"id": "a.two", "status": "fail", "waiver": w}])})
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

    def test_sha_with_a_trailing_newline_rejected(self):
        v = gate({"r.json": report([ok("a.one"), ok("a.two")], commit=SHA + "\n")})
        self.assertFalse(v.ok)
        self.assertTrue(any("40-character" in p for p in v.problems), v.problems)

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


class Summary(unittest.TestCase):
    def test_report_fields_cannot_break_out_of_the_summary(self):
        # The summary is pasted into the release notes. A report is data, so a
        # pipe, a newline or a tag in it must stay inside its cell or line.
        reports = {"r|.json": report([ok("a.one"), {"id": "a.two", "status": "fail", "note": "x\n## injected"}],
                                     bench="b|1", operator="<img src=x>")}
        v = gate(reports)
        out = hil_gate.summary_markdown("shimmer3r", SHA, "1.01.022", reports, REQUIRED, v)
        self.assertNotIn("\n## injected", out)
        self.assertNotRegex(out, r"(?<!\\)<img")    # "\<img" renders as text
        row = next(ln for ln in out.splitlines() if "b\\|1" in ln)
        self.assertEqual(len(re.findall(r"(?<!\\)\|", row)), 6, row)
        self.assertTrue(any(ln.startswith("- ") and "## injected" in ln for ln in out.splitlines()))

    def test_escaping_keeps_ordinary_text(self):
        self.assertEqual(hil_gate.md("pass (hil-report-a.json)"), "pass (hil-report-a.json)")
        self.assertEqual(hil_gate.md("a|b\r\nc"), "a\\|b c")


class Files(unittest.TestCase):
    def test_shipped_required_lists_load(self):
        for p in hil_gate.PLATFORMS:
            items = hil_gate.load_required(p)
            self.assertTrue(items)

    def test_shipped_required_lists_cover_the_checklist_items_the_gate_checks(self):
        # §8 says which of its items the gate checks: from "On-device suites
        # pass" to "§7 regression cases". Each of those lines cites a section
        # first; every platform's list must require that section or a part of it.
        doc = (REPO / "docs" / "SHIMMER3_TEST_PROCEDURE.md").read_text(encoding="utf-8")
        sec = doc.split("\n## 8. Release sign-off", 1)[1].split("\n## ", 1)[0]
        lines = [ln for ln in sec.splitlines() if ln.startswith("- [ ]")]
        start = next(i for i, ln in enumerate(lines) if "On-device suites pass" in ln)
        end = next(i for i, ln in enumerate(lines) if "§7 regression cases" in ln)
        cited = [re.search(r"§\d+(?:\.\d+)*", ln).group() for ln in lines[start:end + 1]]
        self.assertGreaterEqual(len(cited), 10)
        # And the two gate 2 checks read off the candidate's build. The
        # warnings line is a Shimmer3 item.
        size = next(ln for ln in lines if "Flash and RAM" in ln)
        warnings = next(ln for ln in lines if "build warnings read" in ln)
        self.assertIn("§3.4", size)
        self.assertTrue(warnings.startswith("- [ ] Shimmer3 build") and "§3.2" in warnings)
        for p in hil_gate.PLATFORMS:
            refs = re.findall(r"§\d+(?:\.\d+)*", " ".join(i.get("ref", "") for i in hil_gate.load_required(p)))
            for c in cited + ["§3.4"] + (["§3.2"] if p == "shimmer3" else []):
                self.assertTrue(any(r == c or r.startswith(c + ".") for r in refs),
                                f"{p}: §8 has a gate item for {c}, but nothing in required/{p}.json cites it")

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

    def test_signoff_refuses_a_report_name_the_gate_would_not_read(self):
        import hil_signoff
        with redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as e:
                hil_signoff.main(["--tag", "LogAndStream_Shimmer3R_v1.01.022", "--out", "evidence.json", "--upload"])
        self.assertEqual(e.exception.code, 2)

    def test_unusable_inputs_are_exit_2_not_a_traceback(self):
        with tempfile.TemporaryDirectory() as d:
            args = ["--platform", "shimmer3r", "--commit", SHA, "--version", "1.01.022"]
            rep = json.dumps(report([ok(i["id"]) for i in hil_gate.load_required("shimmer3r")]))
            Path(d, "a").mkdir()
            Path(d, "b").mkdir()
            Path(d, "a", "hil-report.json").write_text(rep, encoding="utf-8")
            Path(d, "b", "hil-report.json").write_text(rep, encoding="utf-8")
            Path(d, "latin1.json").write_bytes(b'{"operator": "Se\xe1n"}')
            req = Path(d, "req.json")
            for items in (["a.one"], [{"id": ["a"]}], [{"id": "a.one"}, {"id": "a.one"}]):
                req.write_text(json.dumps({"schema": hil_gate.REQUIRED_SCHEMA, "platform": "shimmer3r",
                                           "items": items}), encoding="utf-8")
                with self.subTest(required=items), redirect_stdout(io.StringIO()):
                    self.assertEqual(hil_gate.main(args + ["--required", str(req)]), 2)
            for paths in ([Path(d, "a", "hil-report.json"), Path(d, "b", "hil-report.json")],
                          [Path(d, "latin1.json")]):
                with self.subTest(reports=[p.name for p in paths]), redirect_stdout(io.StringIO()):
                    self.assertEqual(hil_gate.main(args + [str(p) for p in paths]), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
