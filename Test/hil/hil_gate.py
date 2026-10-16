#!/usr/bin/env python3
"""Release gate: decide whether a firmware release candidate may be made public.

A release candidate is a GitHub pre-release. Whoever tests it - a person
walking the sign-off with hil_signoff.py, or the hardware-in-the-loop rig -
attaches one or more test reports to it (hil-report*.json). This script reads
those reports together with the list of required items for the platform and
answers one question: is every required item covered, on exactly this commit
and version, with nothing failing?

    python hil_gate.py --platform shimmer3r --commit <sha> --version 1.01.022 \
        report1.json [report2.json ...]

Exit status 0 means the candidate may be promoted, 1 means it may not, 2 means
the inputs themselves are unusable. A Markdown summary is written to stdout
(or --summary FILE), suitable for a job summary and for the release notes.

The rules, which docs/SHIMMER3_TEST_PROCEDURE.md §10 states for humans:

  1. At least one report.
  2. Every report names this platform, this exact commit and this version.
     A report for another build is not evidence about this one, so a mismatch
     rejects the whole set rather than being skipped.
  3. A failed result blocks, wherever it appears, unless it carries a waiver -
     an issue key recording who accepted the failure and why. A pass on one
     bench does not cancel a failure on another.
  4. Every required item is covered: passed in some report, or waived.
     Reports may contain any number of other items; they are listed, not
     required. An unwaived not_covered is a gap, not a failure: it covers
     nothing, so it blocks only a required item no other report passes.

Standard library only, so it runs in CI and on a Windows desk alike.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

REPORT_SCHEMA = "shimmer-hil-report/1"
REQUIRED_SCHEMA = "shimmer-hil-required/1"
PLATFORMS = ("shimmer3", "shimmer3r")
STATUSES = ("pass", "fail", "not_covered")
KINDS = ("manual", "automated")
WAIVER_RE = re.compile(r"^[A-Z][A-Z0-9]+-[0-9]+$")
SHA_RE = re.compile(r"^[0-9a-f]{40}$")

REQUIRED_DIR = Path(__file__).resolve().parent / "required"


class InputError(Exception):
    """A report or requirement file that cannot be read as one."""


@dataclass
class Verdict:
    ok: bool
    problems: list[str] = field(default_factory=list)
    covered: dict[str, str] = field(default_factory=dict)   # id -> how
    extra: dict[str, str] = field(default_factory=dict)     # id -> status


def normalise_version(v: str) -> str:
    """'v1.01.022' and '1.01.022' are the same version."""
    return v.strip().lstrip("vV")


def load_json(path: Path) -> dict:
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as e:
        raise InputError(f"{path}: {e}") from e
    if not isinstance(data, dict):
        raise InputError(f"{path}: top level is not an object")
    return data


def load_required(platform: str, path: Path | None = None) -> list[dict]:
    path = path or REQUIRED_DIR / f"{platform}.json"
    data = load_json(path)
    if data.get("schema") != REQUIRED_SCHEMA:
        raise InputError(f"{path}: schema is not {REQUIRED_SCHEMA}")
    if data.get("platform") != platform:
        raise InputError(f"{path}: platform is {data.get('platform')!r}, not {platform!r}")
    items = data.get("items")
    if not isinstance(items, list) or not items:
        raise InputError(f"{path}: no items")
    if not all(isinstance(i, dict) for i in items):
        raise InputError(f"{path}: every item must be an object")
    ids = [i.get("id") for i in items]
    if not all(isinstance(i, str) and i for i in ids) or len(set(ids)) != len(ids):
        raise InputError(f"{path}: item ids must be unique non-empty strings")
    return items


def validate_report(name: str, rep: dict) -> list[str]:
    """Structural problems with one report. Empty list = well formed."""
    p = []
    if rep.get("schema") != REPORT_SCHEMA:
        p.append(f"{name}: schema is {rep.get('schema')!r}, expected {REPORT_SCHEMA!r}")
    if rep.get("kind") not in KINDS:
        p.append(f"{name}: kind must be one of {', '.join(KINDS)}")
    for key in ("platform", "commit", "firmware_version", "bench", "operator", "created"):
        if not isinstance(rep.get(key), str) or not rep.get(key):
            p.append(f"{name}: missing {key}")
    if isinstance(rep.get("commit"), str) and not SHA_RE.match(rep["commit"]):
        p.append(f"{name}: commit must be a full 40-character lowercase SHA")
    results = rep.get("results")
    if not isinstance(results, list):
        p.append(f"{name}: results must be a list")
        return p
    seen = set()
    for i, r in enumerate(results):
        where = f"{name}: results[{i}]"
        if not isinstance(r, dict):
            p.append(f"{where} is not an object")
            continue
        rid = r.get("id")
        if not isinstance(rid, str) or not rid:
            p.append(f"{where}: missing id")
            continue
        if rid in seen:
            p.append(f"{where}: duplicate id {rid}")
        seen.add(rid)
        if r.get("status") not in STATUSES:
            p.append(f"{where} ({rid}): status must be one of {', '.join(STATUSES)}")
        w = r.get("waiver")
        if w is not None and not (isinstance(w, str) and WAIVER_RE.match(w)):
            p.append(f"{where} ({rid}): waiver must be an issue key such as ABC-123")
    return p


def evaluate(platform: str, commit: str, version: str,
             reports: dict[str, dict], required: list[dict]) -> Verdict:
    v = Verdict(ok=False)
    commit = commit.strip().lower()
    version = normalise_version(version)

    if not reports:
        v.problems.append("No test reports are attached to this release candidate.")
        return v

    for name, rep in reports.items():
        v.problems += validate_report(name, rep)
    if v.problems:
        return v

    for name, rep in reports.items():
        if rep["platform"] != platform:
            v.problems.append(f"{name}: tests {rep['platform']}, not {platform}")
        if rep["commit"] != commit:
            v.problems.append(f"{name}: tests commit {rep['commit'][:12]}, "
                              f"but the candidate is {commit[:12]}")
        if normalise_version(rep["firmware_version"]) != version:
            v.problems.append(f"{name}: tests version {rep['firmware_version']}, "
                              f"but the candidate is {version}")
    if v.problems:
        return v

    passes: dict[str, list[str]] = {}
    waivers: dict[str, list[str]] = {}
    for name, rep in reports.items():
        for r in rep["results"]:
            rid, status, waiver = r["id"], r["status"], r.get("waiver")
            if status == "fail" and not waiver:
                note = f" ({r['note']})" if r.get("note") else ""
                v.problems.append(f"{rid}: FAILED in {name}{note}")
            if status == "pass":
                passes.setdefault(rid, []).append(name)
            elif waiver:
                waivers.setdefault(rid, []).append(f"{status}, waived {waiver} in {name}")
            v.extra.setdefault(rid, status)

    for item in required:
        rid = item["id"]
        if rid in passes:
            v.covered[rid] = "pass (" + ", ".join(passes[rid]) + ")"
        elif rid in waivers:
            v.covered[rid] = "; ".join(waivers[rid])
        else:
            v.problems.append(f"{rid}: required but not covered - {item.get('title', '')}")

    for rid in list(v.extra):
        if rid in v.covered:
            del v.extra[rid]

    v.ok = not v.problems
    return v


def summary_markdown(platform: str, commit: str, version: str, reports: dict[str, dict],
                     required: list[dict], v: Verdict) -> str:
    out = [f"### Release gate: {'PASSED' if v.ok else 'BLOCKED'}", "",
           f"`{platform}` v{normalise_version(version)} at `{commit[:12]}`", ""]
    if reports:
        out += ["| Report | Kind | Bench | Operator | Created |", "|---|---|---|---|---|"]
        for name, rep in reports.items():
            out.append(f"| {name} | {rep.get('kind', '?')} | {rep.get('bench', '?')} "
                       f"| {rep.get('operator', '?')} | {rep.get('created', '?')} |")
        out.append("")
    out += ["| Required item | Ref | Result |", "|---|---|---|"]
    for item in required:
        rid = item["id"]
        out.append(f"| {item.get('title', rid)} (`{rid}`) | {item.get('ref', '')} "
                   f"| {v.covered.get(rid, '**not covered**')} |")
    if v.extra:
        n_pass = sum(1 for s in v.extra.values() if s == "pass")
        out += ["", f"Additional results: {len(v.extra)} ({n_pass} passed)."]
    if v.problems:
        out += ["", "**Why it is blocked:**", ""] + [f"- {p}" for p in v.problems]
    return "\n".join(out) + "\n"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--platform", required=True, choices=PLATFORMS)
    ap.add_argument("--commit", required=True, help="full SHA the release tag points at")
    ap.add_argument("--version", required=True, help="firmware version, e.g. 1.01.022")
    ap.add_argument("--required", type=Path, help="override the required-items file")
    ap.add_argument("--summary", type=Path, help="write the Markdown summary here too")
    ap.add_argument("reports", nargs="*", type=Path)
    a = ap.parse_args(argv)
    # The required items cite sections with '§'; a Windows pipe defaults to cp1252.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    try:
        required = load_required(a.platform, a.required)
        # Reports are named by file name, so two with one name would leave one
        # of them unread.
        reports = {}
        for p in a.reports:
            if p.name in reports:
                raise InputError(f"{p}: another report is also named {p.name}")
            reports[p.name] = load_json(p)
    except InputError as e:
        print(f"hil_gate: {e}", file=sys.stderr)
        return 2

    v = evaluate(a.platform, a.commit, a.version, reports, required)
    md = summary_markdown(a.platform, a.commit, a.version, reports, required, v)
    sys.stdout.write(md)
    if a.summary:
        a.summary.write_text(md, encoding="utf-8")
    return 0 if v.ok else 1


if __name__ == "__main__":
    sys.exit(main())
