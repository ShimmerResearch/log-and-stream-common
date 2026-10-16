#!/usr/bin/env python3
"""Record a manual release sign-off as a test report the release gate can read.

Until the hardware-in-the-loop rig covers an item, a person covers it, and this
is how that person's result reaches the gate. It walks the required items for
the platform, asks for each result, and writes a report in the same format the
rig produces (hil_gate.py describes it; docs/SHIMMER3_TEST_PROCEDURE.md §10
explains it).

    python hil_signoff.py --tag LogAndStream_Shimmer3R_v1.01.022
    python hil_signoff.py --tag ... --upload        # also attach it to the pre-release

The commit and version are read from the release candidate itself with the gh
CLI, so the report is bound to the build that was actually tested. Answer each
item with p (pass), f (fail) or n (not covered). A fail or not-covered item can
carry a waiver - the issue key under which someone accepted it - and only then
will the gate let the release through.

Standard library plus the gh CLI.
"""

from __future__ import annotations

import argparse
import datetime as dt
import getpass
import json
import platform as host_platform
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hil_gate  # noqa: E402

REPOS = {
    "shimmer3": ("ShimmerResearch/shimmer3-firmware", "LogAndStream_Shimmer3_v"),
    "shimmer3r": ("ShimmerResearch/shimmer3r-firmware", "LogAndStream_Shimmer3R_v"),
}


def gh(*args: str) -> str:
    return subprocess.run(["gh", *args], check=True, capture_output=True,
                          text=True, encoding="utf-8").stdout.strip()


def platform_from_tag(tag: str) -> str:
    # Longest prefix first: the Shimmer3R prefix starts with the Shimmer3 one.
    for p, (_, prefix) in sorted(REPOS.items(), key=lambda kv: -len(kv[1][1])):
        if tag.startswith(prefix):
            return p
    raise SystemExit(f"{tag}: not a Shimmer3 or Shimmer3R release tag")


def tag_commit(repo: str, tag: str) -> str:
    """The commit the tag points at, peeling an annotated tag."""
    ref = json.loads(gh("api", f"repos/{repo}/git/ref/tags/{tag}"))
    obj = ref["object"]
    while obj["type"] == "tag":
        obj = json.loads(gh("api", f"repos/{repo}/git/tags/{obj['sha']}"))["object"]
    return obj["sha"]


def ask(prompt: str, choices: str) -> str:
    while True:
        a = input(prompt).strip().lower()
        if a and a[0] in choices:
            return a[0]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--tag", required=True, help="release candidate tag")
    ap.add_argument("--operator", default=getpass.getuser())
    ap.add_argument("--bench", default=host_platform.node())
    ap.add_argument("--out", type=Path, help="report path (default: hil-report-manual-<operator>-<time>.json)")
    ap.add_argument("--upload", action="store_true", help="attach the report to the pre-release")
    a = ap.parse_args(argv)
    # The required items cite sections with '§'; a Windows pipe defaults to cp1252.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    plat = platform_from_tag(a.tag)
    repo, prefix = REPOS[plat]
    version = a.tag[len(prefix):]
    commit = tag_commit(repo, a.tag)
    required = hil_gate.load_required(plat)

    print(f"\n{plat} v{version} at {commit[:12]}  ({repo})")
    print("For each item: p = pass, f = fail, n = not covered.  "
          "Section numbers are docs/SHIMMER3_TEST_PROCEDURE.md.\n")

    results = []
    for item in required:
        print(f"[{item['ref']}] {item['title']}")
        status = {"p": "pass", "f": "fail", "n": "not_covered"}[ask("  result [p/f/n]: ", "pfn")]
        r = {"id": item["id"], "status": status}
        note = input("  note (units, what was seen; Enter to skip): ").strip()
        if note:
            r["note"] = note
        if status != "pass":
            while True:
                w = input("  waiver issue key (Enter for none - the gate will block): ").strip().upper()
                if not w or hil_gate.WAIVER_RE.fullmatch(w):
                    break
                print("  that is not an issue key such as ABC-123")
            if w:
                r["waiver"] = w
        results.append(r)
        print()

    now = dt.datetime.now(dt.timezone.utc).replace(microsecond=0)
    rep = {
        "schema": hil_gate.REPORT_SCHEMA,
        "kind": "manual",
        "platform": plat,
        "commit": commit,
        "firmware_version": version,
        "bench": a.bench,
        "operator": a.operator,
        "created": now.isoformat().replace("+00:00", "Z"),
        "results": results,
    }
    problems = hil_gate.validate_report("report", rep)
    if problems:
        print("\n".join(problems), file=sys.stderr)
        return 2

    safe_op = re.sub(r"[^A-Za-z0-9_.-]", "_", a.operator)
    out = a.out or Path(f"hil-report-manual-{safe_op}-{now.strftime('%Y%m%dT%H%M%SZ')}.json")
    out.write_text(json.dumps(rep, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {out}")

    v = hil_gate.evaluate(plat, commit, version, {out.name: rep}, required)
    print("this report alone would " + ("PASS" if v.ok else "BLOCK") + " the gate")
    for p in v.problems:
        print(f"  - {p}")

    if a.upload:
        gh("release", "upload", a.tag, str(out), "-R", repo)
        print(f"attached to {repo} release {a.tag}")
    else:
        print(f"attach it with:  gh release upload {a.tag} {out} -R {repo}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
