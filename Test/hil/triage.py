#!/usr/bin/env python3
"""Ask Claude to triage a failed hardware-in-the-loop run.

    python Test/hil/triage.py <run dir>                 # writes <run dir>/triage.md
    python Test/hil/triage.py <run dir> --dry-run       # writes the prompt only, spends nothing

A run directory is what run_rc.py leaves: build-info.json, pytest.log and
hil-report-<bench>.json. Triage runs only if the report has a failed or
not-covered item - a clean run costs nothing.

What the model is given, and nothing else:
  - the report's failed and not-covered items, and the pytest failure text;
  - the commits between the previous public release and this candidate, in
    the platform firmware and in log-and-stream-common;
  - the bench's known-issues file and bench facts (rig.toml [triage]);
  - read-only access (Read, Grep, Glob) to the firmware checkouts named there.
No network, no MCP servers, no shell, and a hard spending cap.

What it returns is advice: for each failing item a classification - firmware
defect, known issue, bench artifact, harness defect or unclear - with the
evidence, the commits most likely involved, and the next experiment worth
running. It does not decide anything. The release gate stays the
deterministic one in hil_gate.py; a waiver is something a person writes.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from rig import config as rig_config  # noqa: E402

REPOS = {"shimmer3r": "ShimmerResearch/shimmer3r-firmware", "shimmer3": "ShimmerResearch/shimmer3-firmware"}
PREFIX = {"shimmer3r": "LogAndStream_Shimmer3R_v", "shimmer3": "LogAndStream_Shimmer3_v"}
LSC_REPO = "ShimmerResearch/log-and-stream-common"
MAX_LOG_CHARS = 60_000          # pytest failure text handed over, at most
DEFAULT_BUDGET_USD = 2.0

CLASSES = ["firmware_defect", "known_issue", "bench_artifact", "harness_defect", "unclear"]
SCHEMA = {
    "type": "object",
    "required": ["summary", "items"],
    "properties": {
        "summary": {"type": "string", "description": "Three sentences at most: what this run says about the candidate."},
        "items": {
            "type": "array",
            "items": {
                "type": "object",
                "required": ["id", "classification", "confidence", "evidence", "next_step"],
                "properties": {
                    "id": {"type": "string"},
                    "classification": {"type": "string", "enum": CLASSES},
                    "confidence": {"type": "string", "enum": ["high", "medium", "low"]},
                    "known_issue": {"type": "string", "description": "The known issue's key, only if classification is known_issue"},
                    "evidence": {"type": "string", "description": "What in the report, log, commits or source supports the classification. Quote numbers."},
                    "suspect_commits": {"type": "array", "items": {"type": "string"}},
                    "source_refs": {"type": "array", "items": {"type": "string"}, "description": "file:line references actually read"},
                    "next_step": {"type": "string", "description": "The single most useful next experiment or check."},
                },
            },
        },
        "new_issues": {
            "type": "array",
            "description": "Defects that look new - drafts for a person to review, never filed automatically.",
            "items": {"type": "object", "required": ["title", "body"],
                      "properties": {"title": {"type": "string"}, "body": {"type": "string"}}},
        },
    },
}

SYSTEM = """You are triaging a failed hardware-in-the-loop test run of Shimmer LogAndStream firmware.
A deterministic release gate has already decided pass/fail; your job is to explain the failures to an engineer.

Rules:
- Classify every failed or not-covered item: firmware_defect, known_issue, bench_artifact, harness_defect or unclear.
- known_issue only when an entry in the known-issues list matches the symptom; give its key.
- bench_artifact only with a named cause from the bench facts or the run's interference list.
- harness_defect when the evidence points at the test code (Test/hil) rather than the device.
- unclear is a good answer when the evidence does not decide it. Never guess a cause you cannot support.
- Quote the numbers you rely on. Cite source as file:line only for lines you have actually read with your tools.
- Use the commit list to name suspects only where a commit plausibly touches the failing subsystem.
- You may read the firmware checkouts (Read, Grep, Glob). Do not ask for anything else.
- Draft new_issues only for defects that match no known issue. Drafts are reviewed by a person; write them plainly."""


def find_claude(rig) -> str:
    configured = rig.extra.get("triage", {}).get("claude")
    if configured:
        return configured
    on_path = shutil.which("claude")
    if on_path:
        return on_path
    # The Claude desktop app ships the CLI under a versioned folder; take the newest.
    pattern = os.path.join(os.environ.get("APPDATA", ""), "Claude", "claude-code", "*", "*", "claude.exe")
    found = sorted(glob.glob(pattern), key=lambda p: [int(x) for x in re.findall(r"\d+", Path(p).parts[-3])])
    if found:
        return found[-1]
    raise SystemExit("no claude CLI found: put it on PATH or set [triage] claude in rig.toml")


def failure_text(log: Path) -> str:
    """The FAILURES section of a pytest -rA log, which names every failing assertion."""
    if not log.exists():
        return ""
    text = log.read_text(encoding="utf-8", errors="replace")
    start = text.find("= FAILURES =")
    end = text.find("short test summary info")
    chunk = text[start:end] if start >= 0 else text[-MAX_LOG_CHARS:]
    if len(chunk) > MAX_LOG_CHARS:
        chunk = chunk[:MAX_LOG_CHARS] + "\n[... truncated ...]"
    return chunk


def gh_json(*args):
    r = subprocess.run(["gh", *args], capture_output=True, text=True, encoding="utf-8")
    return json.loads(r.stdout) if r.returncode == 0 and r.stdout.strip() else None


def previous_release(platform: str, tag: str) -> str | None:
    """The newest published (non-pre) release before this tag."""
    rels = gh_json("release", "list", "-R", REPOS[platform], "-L", "40",
                   "--json", "tagName,isPrerelease,publishedAt") or []
    tags = [r["tagName"] for r in rels if not r["isPrerelease"] and r["tagName"] != tag]
    return tags[0] if tags else None


def commits_between(repo: str, base: str, head: str, limit: int = 80) -> list[str]:
    data = gh_json("api", f"repos/{repo}/compare/{base}...{head}")
    if not data:
        return [f"(could not compare {base}...{head} in {repo})"]
    out = []
    for c in data.get("commits", [])[-limit:]:
        out.append(f"{c['sha'][:10]} {c['commit']['message'].splitlines()[0]}")
    return out or ["(no commits)"]


def build_prompt(run: Path, rig) -> tuple[str, dict]:
    info = json.loads((run / "build-info.json").read_text(encoding="utf-8"))
    reports = sorted(run.glob("hil-report-*.json"))
    if not reports:
        raise SystemExit(f"{run}: no hil-report-*.json")
    rep = json.loads(reports[0].read_text(encoding="utf-8"))
    bad = [r for r in rep["results"] if r["status"] != "pass"]
    platform, tag = info["platform"], info["tag"]

    prev = previous_release(platform, tag)
    fw_commits = commits_between(REPOS[platform], prev, tag) if prev else ["(no previous release found)"]
    lsc_commits = []
    if prev:
        lsc_commits = commits_between(LSC_REPO, prev, tag)   # LSC is tagged with each platform release

    tri = rig.extra.get("triage", {})
    known = ""
    if tri.get("known_issues") and Path(tri["known_issues"]).exists():
        known = Path(tri["known_issues"]).read_text(encoding="utf-8")
    facts = tri.get("bench_facts", "")

    parts = [
        f"# Candidate\n{tag} (platform {platform}), commit {info['commit']}. Previous public release: {prev or 'unknown'}.",
        f"# Run\nbench {rep['bench']}, {rep['created']}, markers {rep.get('run', {}).get('markers')!r}, "
        f"interference at start: {rep.get('run', {}).get('interference') or 'none'}.",
        "# Items that did not pass\n" + json.dumps(bad, indent=1),
        f"# Items that passed\n{', '.join(r['id'] for r in rep['results'] if r['status'] == 'pass') or 'none'}",
        "# pytest failure text\n" + (failure_text(run / "pytest.log") or "(no pytest.log)"),
        "# Firmware commits since the previous release\n" + "\n".join(fw_commits),
        "# log-and-stream-common commits since the previous release\n" + ("\n".join(lsc_commits) or "(none)"),
        "# Known issues\n" + (known or "(none supplied)"),
        "# Bench facts\n" + (facts or "(none supplied)"),
        "# Test documentation\nThe checks are described in docs/SHIMMER3_TEST_PROCEDURE.md §10 and Test/hil/README.md; "
        "the test code is under Test/hil/ - read it when deciding harness_defect.",
    ]
    return "\n\n".join(parts), {"report": rep, "bad": bad, "info": info, "prev": prev}


def render(result: dict, ctx: dict, cost: float | None) -> str:
    info = ctx["info"]
    lines = [f"# Triage: {info['tag']}", "",
             f"Generated by Claude from the run's report, log and commit history. **Advice, not a verdict:** "
             f"the release gate's decision stands as the report gives it.", "",
             result.get("summary", ""), "",
             "| Item | Classification | Confidence | Next step |", "|---|---|---|---|"]
    for it in result.get("items", []):
        cls = it["classification"] + (f" ({it['known_issue']})" if it.get("known_issue") else "")
        lines.append(f"| `{it['id']}` | {cls} | {it['confidence']} | {it['next_step']} |")
    for it in result.get("items", []):
        lines += ["", f"## `{it['id']}`", "", it["evidence"]]
        if it.get("suspect_commits"):
            lines += ["", "Suspect commits: " + ", ".join(f"`{c}`" for c in it["suspect_commits"])]
        if it.get("source_refs"):
            lines += ["", "Source read: " + ", ".join(f"`{s}`" for s in it["source_refs"])]
    if result.get("new_issues"):
        lines += ["", "## Draft issues (not filed)", ""]
        for d in result["new_issues"]:
            lines += [f"### {d['title']}", "", d["body"], ""]
    if cost is not None:
        lines += ["", f"_Triage cost: ${cost:.2f}._"]
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("run", type=Path, help="run directory written by run_rc.py")
    ap.add_argument("--rig", default=None)
    ap.add_argument("--dry-run", action="store_true", help="write the prompt to triage-prompt.md and stop")
    ap.add_argument("--render-only", action="store_true",
                    help="re-render triage.md from the reply already in triage-raw.json; sends nothing")
    ap.add_argument("--budget-usd", type=float, default=None)
    ap.add_argument("--model", default=None)
    a = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    rig = rig_config.load(a.rig)
    prompt, ctx = build_prompt(a.run, rig)
    (a.run / "triage-prompt.md").write_text(prompt, encoding="utf-8")
    if not ctx["bad"]:
        print("every item passed - nothing to triage")
        return 0
    if a.dry_run:
        print(f"prompt written to {a.run / 'triage-prompt.md'} ({len(prompt)} chars); nothing sent")
        return 0
    if a.render_only:
        raw = a.run / "triage-raw.json"
        if not raw.exists():
            raise SystemExit(f"{raw}: no earlier reply to render")
        return finish(a.run, json.loads(raw.read_text(encoding="utf-8")), ctx)

    tri = rig.extra.get("triage", {})
    budget = a.budget_usd or tri.get("budget_usd", DEFAULT_BUDGET_USD)
    cmd = [find_claude(rig), "-p", "--output-format", "json", "--no-session-persistence",
           "--strict-mcp-config", "--tools", "Read,Grep,Glob", "--allowedTools", "Read,Grep,Glob",
           "--max-budget-usd", str(budget), "--append-system-prompt", SYSTEM,
           "--json-schema", json.dumps(SCHEMA)]
    for d in tri.get("source_dirs", []):
        cmd += ["--add-dir", d]
    model = a.model or tri.get("model")
    if model:
        cmd += ["--model", model]
    r = subprocess.run(cmd, input=prompt, capture_output=True, text=True, encoding="utf-8",
                       cwd=HERE.parent.parent, timeout=1800)
    (a.run / "triage-raw.json").write_text(r.stdout, encoding="utf-8")   # kept for diagnosis
    try:
        out = json.loads(r.stdout)
    except json.JSONDecodeError:
        print(r.stdout[-2000:], r.stderr[-2000:], sep="\n")
        return 2
    if out.get("is_error") or out.get("subtype", "success") != "success":
        print(f"claude: {out.get('subtype')}: {str(out.get('result'))[:500]} "
              f"(cost ${out.get('total_cost_usd') or 0:.2f})")
        return 2
    return finish(a.run, out, ctx)


def extract_result(out: dict) -> dict | None:
    """The structured answer: --json-schema's structured_output if the CLI
    honoured it, otherwise the JSON object in the text reply (the CLI has been
    seen to return it inside a ```json fence instead)."""
    if out.get("structured_output") is not None:
        return out["structured_output"]
    text = out.get("result") or ""
    m = re.search(r"```(?:json)?\s*(\{.*\})\s*```", text, re.S)
    candidate = m.group(1) if m else text[text.find("{"):text.rfind("}") + 1]
    try:
        result = json.loads(candidate)
    except json.JSONDecodeError:
        return None
    return result if isinstance(result, dict) and "items" in result else None


def finish(run: Path, out: dict, ctx: dict) -> int:
    result = extract_result(out)
    if result is None:
        print(f"claude returned no structured result (cost ${out.get('total_cost_usd') or 0:.2f}); "
              f"reply kept in {run / 'triage-raw.json'}")
        return 2
    bad_ids = {r["id"] for r in ctx["bad"]}
    missed = sorted(bad_ids - {it.get("id") for it in result.get("items", [])})
    if missed:
        result.setdefault("items", []).extend(
            {"id": i, "classification": "unclear", "confidence": "low",
             "evidence": "Not addressed by the model.", "next_step": "Triage by hand."} for i in missed)
    md = render(result, ctx, out.get("total_cost_usd"))
    (run / "triage.md").write_text(md, encoding="utf-8")
    (run / "triage.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
