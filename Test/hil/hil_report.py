"""Collect test outcomes into the report the release gate reads.

Each hardware test carries an item id (@pytest.mark.hil("smoke.boot", ...)).
Several tests may share an id - one per unit, say - and the item's status is
the worst of them:

    fail  >  not_covered  >  pass

A pass on one unit does not hide a failure on another, and an item is only
"pass" if every test behind it ran and passed. Each result also lists every
unit's own status under "units", so a waiver can be held to the units it was
given for. A test that could not run
because the bench lacks something (a hub, a programmer, a Bluetooth unit) is
not_covered, never pass: the gate then knows to look for that item in another
bench's report.

The report format is the one hil_gate.py validates, so a report this writes is
checked by the same code that will judge it.
"""

from __future__ import annotations

import argparse
import datetime as dt
import getpass
import json
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path

import hil_gate

RANK = {"pass": 0, "not_covered": 1, "fail": 2}


def parse_waiver(text: str) -> tuple[str, str]:
    """`ITEM=ISSUE` or `ITEM@UNIT=ISSUE` -> (key, ISSUE), the key being `ITEM` or
    `ITEM@UNIT`. Raises ValueError for anything else."""
    key, sep, issue = text.partition("=")
    key, issue = key.strip(), issue.strip().upper()
    item, at, unit = key.partition("@")
    if not sep or not item.strip() or (at and not unit.strip()) or not hil_gate.WAIVER_RE.fullmatch(issue):
        raise ValueError(f"{text}: expected ITEM=ISSUE or ITEM@UNIT=ISSUE, e.g. smoke.sd_log=ABC-123")
    return (f"{item.strip()}@{unit.strip()}" if at else item.strip()), issue


def waiver_for(item_id: str, status: str, units: dict[str, str] | None,
               waivers: dict[str, str]) -> str | None:
    """The issue key that accepts a result that did not pass, or None.

    `ITEM` waives every unit. `ITEM@UNIT` waives that unit only, so it applies
    when every unit that did not pass the item has one, all under the same
    issue: the gate takes one key per item. Without per-unit statuses (`units`
    missing, as in reports written before they existed) only `ITEM` applies."""
    if status == "pass":
        return None
    if item_id in waivers:
        return waivers[item_id]
    bad = [u for u, s in (units or {}).items() if s != "pass"]
    if not bad:
        return None
    keys = {waivers.get(f"{item_id}@{u}") for u in bad}
    if None in keys or len(keys) != 1:
        return None
    return keys.pop()


@dataclass
class Item:
    id: str
    status: str = "pass"
    notes: list[str] = field(default_factory=list)
    tests: int = 0
    units: dict[str, str] = field(default_factory=dict)     # each unit's worst status

    def add(self, status: str, note: str | None = None, unit: str | None = None):
        self.tests += 1
        if RANK[status] > RANK[self.status]:
            self.status = status
        if note:
            self.notes.append(note)
        if unit:
            if unit not in self.units or RANK[status] > RANK[self.units[unit]]:
                self.units[unit] = status


class Collector:
    def __init__(self):
        self.items: dict[str, Item] = {}

    def record(self, item_id: str, status: str, note: str | None = None, unit: str | None = None):
        if status not in RANK:
            raise ValueError(status)
        self.items.setdefault(item_id, Item(item_id)).add(status, note, unit)

    def results(self, waivers: dict[str, str] | None = None) -> list[dict]:
        """Each item's result. `waivers` maps `ITEM` or `ITEM@UNIT` to the issue
        key under which someone accepted a failure or gap (run_rc.py --waive;
        waiver_for says which applies). It is written into the item's result
        only if the item did not pass, as a sign-off's waiver is, and the gate
        treats it the same way."""
        waivers = waivers or {}
        out = []
        for item in sorted(self.items.values(), key=lambda i: i.id):
            r = {"id": item.id, "status": item.status}
            key = waiver_for(item.id, item.status, item.units, waivers)
            if key:
                r["waiver"] = key
            if item.units:
                r["units"] = dict(sorted(item.units.items()))
            if item.notes:
                # Keep the report readable: the first few notes say enough, and
                # the full detail is in the pytest log archived beside it.
                note = "; ".join(item.notes[:4])
                if len(item.notes) > 4:
                    note += f"; (+{len(item.notes) - 4} more)"
                r["note"] = note[:500]
            out.append(r)
        return out

    def report(self, *, platform: str, commit: str, firmware_version: str,
               bench: str, operator: str, extra: dict | None = None,
               waivers: dict[str, str] | None = None) -> dict:
        rep = {
            "schema": hil_gate.REPORT_SCHEMA,
            "kind": "automated",
            "platform": platform,
            "commit": commit,
            "firmware_version": firmware_version,
            "bench": bench,
            "operator": operator,
            "created": dt.datetime.now(dt.timezone.utc).replace(microsecond=0)
                         .isoformat().replace("+00:00", "Z"),
            "results": self.results(waivers),
        }
        if extra:
            rep["run"] = extra
        problems = hil_gate.validate_report("report", rep)
        if problems:
            raise ValueError("; ".join(problems))
        return rep


def tested_units(items, platform: str, only: str = "") -> list:
    """The units a session's collected items ran against on this platform, in order:
    the ones the report is about, not every unit rig.toml describes."""
    seen, out = set(), []
    for item in items:
        unit = item.callspec.params.get("unit") if hasattr(item, "callspec") else None
        if unit is None or unit.platform != platform or (only and unit.name != only) or unit.name in seen:
            continue
        seen.add(unit.name)
        out.append(unit)
    return out


def read_radios(units, read, lacks) -> dict[str, dict[str, str]]:
    """Each unit's Bluetooth module firmware as the unit reports it, for run.hardware.

    read(unit) returns the module string over the unit's dock; lacks(unit) says
    why the bench cannot reach that dock, or None. Each unit gets either
    {"radio_reported": <string>} or {"radio_unread": <why>}. A unit that could
    not be read is still tested: this records what was fitted, it judges nothing.
    """
    out = {}
    for unit in units:
        why = lacks(unit)
        if not why:
            try:
                text = read(unit)
            except Exception as e:      # a dock that does not answer, a port that is gone
                why = f"{type(e).__name__}: {e}"
            else:
                why = None if text else "the dock replied with an empty string"
        out[unit.name] = {"radio_unread": why[:200]} if why else {"radio_reported": text}
    return out


def hardware(host: dict, kit: dict, units, radios: dict[str, dict] | None = None) -> dict:
    """run.hardware: the PC, the bench's fixed kit and the units tested, so that a
    figure can be traced to what gave it and benches told apart.

    `radios` (read_radios) adds what each unit's module reported beside the
    `radio` rig.toml gives it: a module can be re-flashed, and its build moves
    the Bluetooth figures a long way."""
    radios = radios or {}
    return {"host": host, "kit": kit, "units": [{**u.describe(), **radios.get(u.name, {})} for u in units]}


def write(rep: dict, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rep, indent=2) + "\n", encoding="utf-8")
    return path


def waive(rep: dict, waivers: dict[str, str], by: str, at: str | None = None) -> list[str]:
    """Add waivers decided after a run to a report, in place; return what was done.

    The same rules as a waiver given at run time (waiver_for), so a per-unit
    waiver needs the per-unit statuses a report carries from when they were
    added. A result that passed, or already carries a waiver, is left alone.
    Each waiver added is recorded under run.waived_after_run, with who added
    it and when. The gate does not read that; it is there for the reader."""
    at = at or dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    done = []
    ids = {r["id"] for r in rep["results"]}
    wanted = {k.partition("@")[0] for k in waivers}
    for r in rep["results"]:
        if r["id"] not in wanted:
            continue
        if r["status"] == "pass":
            done.append(f"{r['id']}: passed, nothing to waive")
            continue
        if r.get("waiver"):
            done.append(f"{r['id']}: already waived under {r['waiver']}, left alone")
            continue
        key = waiver_for(r["id"], r["status"], r.get("units"), waivers)
        if not key:
            why = ("this report has no per-unit statuses, so only a whole-item waiver (ITEM=ISSUE) applies"
                   if "units" not in r else
                   f"not every unit that did not pass is waived under one issue: {r['units']}")
            done.append(f"{r['id']}: NOT waived - {why}")
            continue
        r["waiver"] = key
        given = sorted(k for k in waivers if k.partition("@")[0] == r["id"])
        rep.setdefault("run", {}).setdefault("waived_after_run", []).append(
            {"item": r["id"], "issue": key, "as": given, "by": by, "at": at})
        done.append(f"{r['id']}: waived under {key}")
    for k in sorted(waivers):
        if k.partition("@")[0] not in ids:
            done.append(f"{k}: no such item in this report")
    return done


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Add waivers decided after a run to a HIL report.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    w = sub.add_parser("waive", help="write issue keys into results that did not pass, keeping the original "
                                     "beside it as <name>.before-waivers.json")
    w.add_argument("reports", nargs="+", type=Path)
    w.add_argument("--waive", action="append", required=True, metavar="ITEM[@UNIT]=ISSUE")
    w.add_argument("--by", default=getpass.getuser(), help="who decided (default: this user)")
    a = ap.parse_args(argv)
    try:
        waivers = dict(parse_waiver(x) for x in a.waive)
    except ValueError as e:
        ap.error(str(e))
    rc = 0
    for path in a.reports:
        rep = json.loads(path.read_text(encoding="utf-8"))
        problems = hil_gate.validate_report(path.name, rep)
        if problems:
            print(f"{path}: not a usable report: {'; '.join(problems)}")
            rc = 2
            continue
        done = waive(rep, waivers, a.by)
        print(f"{path}:")
        for line in done:
            print(f"  {line}")
        if any(" waived under " in line for line in done):
            backup = path.with_name(path.stem + ".before-waivers.json")
            if not backup.exists():
                shutil.copy2(path, backup)
            write(rep, path)
        if any("NOT waived" in line for line in done):
            rc = max(rc, 1)
    return rc


if __name__ == "__main__":
    sys.exit(main())
