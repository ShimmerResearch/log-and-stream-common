"""Collect test outcomes into the report the release gate reads.

Each hardware test carries an item id (@pytest.mark.hil("smoke.boot", ...)).
Several tests may share an id - one per unit, say - and the item's status is
the worst of them:

    fail  >  not_covered  >  pass

A pass on one unit does not hide a failure on another, and an item is only
"pass" if every test behind it ran and passed. A test that could not run
because the bench lacks something (a hub, a programmer, a Bluetooth unit) is
not_covered, never pass: the gate then knows to look for that item in another
bench's report.

The report format is the one hil_gate.py validates, so a report this writes is
checked by the same code that will judge it.
"""

from __future__ import annotations

import datetime as dt
import json
from dataclasses import dataclass, field
from pathlib import Path

import hil_gate

RANK = {"pass": 0, "not_covered": 1, "fail": 2}


@dataclass
class Item:
    id: str
    status: str = "pass"
    notes: list[str] = field(default_factory=list)
    tests: int = 0

    def add(self, status: str, note: str | None = None):
        self.tests += 1
        if RANK[status] > RANK[self.status]:
            self.status = status
        if note:
            self.notes.append(note)


class Collector:
    def __init__(self):
        self.items: dict[str, Item] = {}

    def record(self, item_id: str, status: str, note: str | None = None):
        if status not in RANK:
            raise ValueError(status)
        self.items.setdefault(item_id, Item(item_id)).add(status, note)

    def results(self) -> list[dict]:
        out = []
        for item in sorted(self.items.values(), key=lambda i: i.id):
            r = {"id": item.id, "status": item.status}
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
               bench: str, operator: str, extra: dict | None = None) -> dict:
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
            "results": self.results(),
        }
        if extra:
            rep["run"] = extra
        problems = hil_gate.validate_report("report", rep)
        if problems:
            raise ValueError("; ".join(problems))
        return rep


def write(rep: dict, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rep, indent=2) + "\n", encoding="utf-8")
    return path
