#!/usr/bin/env python3
"""Test a release candidate on this bench: download, flash, run, report.

    python Test/hil/run_rc.py LogAndStream_Shimmer3R_v1.01.022
    python Test/hil/run_rc.py <tag> --tier smoke --upload       # attach the report to the pre-release
    python Test/hil/run_rc.py <tag> --no-flash                  # unit already runs the candidate

1. Downloads the image and build-info.json from the GitHub release with gh.
   Releases made before build-info.json existed get one synthesised from the
   tag, so older builds can be measured too (their report is still bound to
   the tag's commit).
2. Flashes every rig unit of the candidate's platform (rig.toml: flash method
   "dfu" by default, "swd" through the unit's own ST-Link), and confirms each
   reports the candidate's version.
3. Runs the tier with pytest, which writes hil-report-<bench>.json.
4. With --upload, attaches the report and the pytest log (as
   hil-log-<bench>.log) to the release.

Everything lands in Test/hil/out/<tag>/<time>/ (git-ignored), which is what
to archive alongside the report.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from hil_gate import normalise_version  # noqa: E402
from rig import config as rig_config  # noqa: E402
from rig import flash as rig_flash  # noqa: E402

REPOS = {"LogAndStream_Shimmer3R_v": ("shimmer3r", "ShimmerResearch/shimmer3r-firmware", "*.hex"),
         "LogAndStream_Shimmer3_v": ("shimmer3", "ShimmerResearch/shimmer3-firmware", "*.txt")}


def gh(*args, **kw) -> str:
    return subprocess.run(["gh", *args], check=True, capture_output=True, text=True,
                          encoding="utf-8", **kw).stdout.strip()


def parse_tag(tag: str):
    for prefix, (platform, repo, pattern) in sorted(REPOS.items(), key=lambda kv: -len(kv[0])):
        if tag.startswith(prefix):
            return platform, repo, pattern, tag[len(prefix):]
    raise SystemExit(f"{tag}: not a Shimmer3 or Shimmer3R release tag")


def tag_commit(repo: str, tag: str) -> str:
    obj = json.loads(gh("api", f"repos/{repo}/git/ref/tags/{tag}"))["object"]
    while obj["type"] == "tag":
        obj = json.loads(gh("api", f"repos/{repo}/git/tags/{obj['sha']}"))["object"]
    return obj["sha"]


def fetch(tag: str, out: Path) -> tuple[dict, Path]:
    platform, repo, pattern, version = parse_tag(tag)
    gh("release", "download", tag, "-R", repo, "-p", pattern, "-D", str(out), "--clobber")
    images = sorted(out.glob(pattern))
    if len(images) != 1:
        raise SystemExit(f"{tag}: expected one {pattern} image on the release, found {len(images)}")
    info_path = out / "build-info.json"
    # The report is bound to the commit in build-info.json, and the gate binds
    # the release to the commit its tag points at, so the two must agree.
    commit = tag_commit(repo, tag)
    # Only a release that has no build-info.json gets one synthesised. A
    # download that fails for any other reason (network, auth) stops the run:
    # going on would bind the report to metadata nobody read.
    assets = gh("release", "view", tag, "-R", repo, "--json", "assets", "-q", ".assets[].name").splitlines()
    if "build-info.json" in assets:
        gh("release", "download", tag, "-R", repo, "-p", "build-info.json", "-D", str(out), "--clobber")
        info = json.loads(info_path.read_text(encoding="utf-8"))
    else:
        info = {"schema": "shimmer-build-info/1", "platform": platform, "tag": tag,
                "firmware_version": version, "commit": commit,
                "image": images[0].name, "synthesised": True}
        info_path.write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
    if info["platform"] != platform or normalise_version(info["firmware_version"]) != normalise_version(version):
        raise SystemExit(f"build-info.json does not match the tag: {info}")
    if str(info.get("commit", "")).lower() != commit:
        raise SystemExit(f"build-info.json names commit {info.get('commit')}, but {tag} points at {commit}")
    # The report needs a lowercase SHA (hil_report.validate_report): use the
    # tag's, which the API gives in lowercase, whatever case build-info.json had.
    info["commit"] = commit
    info_path.write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
    return info, images[0]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("tag")
    ap.add_argument("--tier", default="smoke or fleet",
                    help="pytest -m expression (default: \"smoke or fleet\" - each platform runs its own)")
    ap.add_argument("--rig", default=None)
    ap.add_argument("--no-flash", action="store_true", help="test what is on the units now")
    ap.add_argument("--upload", action="store_true", help="attach the report to the GitHub release")
    ap.add_argument("--waive", action="append", default=[], metavar="ITEM=ISSUE",
                    help="accept a known failure or gap of ITEM under ISSUE, e.g. smoke.sd_log=ABC-123; "
                         "the key goes into the report and the gate shows it (repeatable)")
    ap.add_argument("pytest_args", nargs="*", help="extra pytest arguments, after --")
    a = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    rig = rig_config.load(a.rig)
    platform, repo, _, version = parse_tag(a.tag)
    stamp = dt.datetime.now().strftime("%Y%m%dT%H%M%S")
    out = HERE / "out" / a.tag / stamp
    out.mkdir(parents=True, exist_ok=True)

    info, image = fetch(a.tag, out)
    print(f"candidate {a.tag}: commit {info['commit'][:12]}, image {image.name}")

    units = rig.units_for(platform)
    if not units:
        print(f"rig {rig.bench} has no {platform} unit - nothing to test")
        return 1
    if not a.no_flash and platform == "shimmer3":
        if not rig.base6_serial:
            raise SystemExit("rig.toml names no Base6; Shimmer3 flashing goes through its slots")
        from rig.base6 import Base6
        from rig.s3_flash import flash_slot
        # InfoMem backups live beside the run: the one copy of each unit's
        # configuration and calibration if a flash fails half way.
        with Base6(rig.base6_serial) as b6:
            for u in units:
                if not u.base6_slot:
                    print(f"{u.name}: no base6_slot, not flashed")
                    continue
                print(f"{u.name}: flashing {image.name} through Base6 slot {u.base6_slot} ...", flush=True)
                flash_slot(b6, u.base6_slot, image, out / "infomem-backup", expect_version=version)
    elif not a.no_flash:
        if not rig.stm32_programmer:
            raise SystemExit("rig.toml names no stm32_programmer")
        # A unit that cannot be flashed would still be tested, on whatever it
        # runs now, so refuse before flashing any of them.
        unflashable = [u.name for u in units if not u.usb_serial]
        if unflashable:
            raise SystemExit(f"{', '.join(unflashable)}: no usb_serial in rig.toml, so not flashable. Add it, "
                             "or test what the units run now with --no-flash.")
        for u in units:
            stlink = rig.stlink_for(u)          # this unit's ST-Link, never another unit's
            method = "swd" if stlink else "dfu"
            print(f"{u.name}: flashing {image.name} by {method} ...", flush=True)
            v = rig_flash.flash(rig.stm32_programmer, image, u.usb_serial, method,
                                stlink, expect_version=version)
            print(f"{u.name}: now v{v.text}")

    report = out / f"hil-report-{rig.bench}.json"
    cmd = [sys.executable, "-m", "pytest", str(HERE / "tests"), "-m", a.tier,
           "--rig", str(rig.path), "--candidate", str(out / "build-info.json"),
           "--hil-report", str(report), "-p", "no:cacheprovider", "-rA",
           *[f"--waive={w}" for w in a.waive], *a.pytest_args]
    log = out / "pytest.log"
    with open(log, "w", encoding="utf-8") as f:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, encoding="utf-8", errors="replace")
        for line in p.stdout:
            sys.stdout.write(line)
            f.write(line)
        rc = p.wait()

    if not report.exists():
        print("no report was written")
        return rc or 1
    gate = subprocess.run([sys.executable, str(HERE / "hil_gate.py"), "--platform", platform,
                           "--commit", info["commit"], "--version", version,
                           "--required", str(HERE / "required" / f"{platform}.json"), str(report)],
                          capture_output=True, text=True, encoding="utf-8")
    print("\nThis report alone, against the required list:\n" + gate.stdout)

    # The log goes up under the bench's name, so two benches' logs do not
    # overwrite each other on the release. It does not match hil-report*.json,
    # so the gate never reads it as a report.
    attached = out / f"hil-log-{rig.bench}.log"
    shutil.copyfile(log, attached)
    if a.upload:
        gh("release", "upload", a.tag, str(report), str(attached), "-R", repo, "--clobber")
        print(f"attached {report.name} and {attached.name} to {repo} {a.tag}")
    else:
        print(f"report: {report}\nattach with: gh release upload {a.tag} \"{report}\" \"{attached}\" -R {repo}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
