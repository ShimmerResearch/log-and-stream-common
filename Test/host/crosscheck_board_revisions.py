#!/usr/bin/env python3
"""Cross-check the board revision gates against docs/SHIMMER3_BOARD_REVISIONS.md.

WHY THIS EXISTS. AGENTS.md says two things about that document:

  - its per-product tables are a conversion of a hardware workbook that is NOT
    in this repository, so nobody outside Shimmer can check them;
  - where the tables and the firmware disagree, THE FIRMWARE WINS, and the
    disagreement is to be reported rather than silently reconciled either way.

Those two together are what makes a stale table dangerous: it is believed, on
two platforms, by people who cannot verify it. This script is the reporting
mechanism. It reads the gate table out of the document, asks the compiled
firmware gate what it actually does for each revision named there, and prints
every disagreement.

IT DOES NOT DECIDE WHO IS RIGHT. A disagreement can mean the document is stale
OR that the firmware has a bug, and the two need different fixes - so this exits
non-zero, names the revision, and leaves the judgement to a person. Do not
"fix" it by editing the table to match the code.

Run by .github/workflows/host-tests.yml, after building test_boards.

Usage: crosscheck_board_revisions.py <path-to-test_boards-binary>
"""
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DOC = os.path.join(REPO, "docs", "SHIMMER3_BOARD_REVISIONS.md")

# The gate rows this script knows how to check, keyed by the text in the
# document's "Gate" column. The value is the column index of the dumped gate
# matrix that the firmware hook writes.
#
#   test_boards --dump prints:
#     srId,major,minor,bmp581,lis3mdl,adxl371,ads7028,i2c4,boot0default
GATE_COLUMN = {
    "BMP581 replaces BMP390": 3,
}

# Board family prefixes, as the document writes them (SR31-11-2), mapped to the
# exp_brd_id the firmware stores in EEPROM. From enum SR_BOARD_CODES in
# Boards/shimmer_boards.h.
SR_TO_BRD_ID = {
    31: 31,  # Shimmer3 IMU
    38: 38,  # Proto3 Deluxe
    47: 47,  # ExG unified
    48: 48,  # GSR+ unified
    49: 49,  # Bridge Amplifier unified
}

SR_RE = re.compile(r"\bSR(\d+)-(\d+)-(\d+)\b")


def load_gate_matrix(binary):
    """{(srId, major, minor): [gate flags]} straight from the firmware."""
    out = subprocess.run([binary, "--dump"], capture_output=True, text=True, check=True)
    matrix = {}
    for line in out.stdout.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [int(p) for p in line.split(",")]
        matrix[(parts[0], parts[1], parts[2])] = parts
    if not matrix:
        raise SystemExit("FAIL: test_boards --dump produced no rows")
    return matrix


def documented_revisions():
    """{gate name: [(srId, major, minor), ...]} from the document's gate table."""
    if not os.path.isfile(DOC):
        raise SystemExit(f"FAIL: cannot find {DOC}")

    found = {}
    with open(DOC, encoding="utf-8") as handle:
        for line in handle:
            if not line.startswith("|"):
                continue
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if len(cells) < 2:
                continue
            gate = cells[0]
            if gate not in GATE_COLUMN:
                continue
            revs = []
            for sr, major, minor in SR_RE.findall(cells[1]):
                sr = int(sr)
                if sr in SR_TO_BRD_ID:
                    revs.append((SR_TO_BRD_ID[sr], int(major), int(minor)))
            found[gate] = revs

    missing = set(GATE_COLUMN) - set(found)
    if missing:
        # A renamed or deleted row must not turn this check into a no-op.
        raise SystemExit(
            "FAIL: these gate rows are no longer in "
            "docs/SHIMMER3_BOARD_REVISIONS.md: " + ", ".join(sorted(missing))
        )
    return found


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2

    matrix = load_gate_matrix(sys.argv[1])
    documented = documented_revisions()

    failures = 0
    checked = 0

    for gate, revs in documented.items():
        column = GATE_COLUMN[gate]
        if not revs:
            print(f"  FAIL '{gate}': the document names no revisions this script "
                  f"could parse - the row's format has changed")
            failures += 1
            continue

        for sr_id, major, minor in revs:
            key = (sr_id, major, minor)
            if key not in matrix:
                print(f"  FAIL SR{sr_id}-{major}-{minor} is documented under "
                      f"'{gate}' but is outside the range test_boards dumps")
                failures += 1
                continue

            checked += 1
            if not matrix[key][column]:
                print(f"  DISAGREEMENT SR{sr_id}-{major}-{minor}: the document "
                      f"lists it under '{gate}', the firmware gate says no")
                failures += 1

            # The revision immediately below must NOT match, or the boundary the
            # document describes is not the boundary the firmware implements.
            if minor > 0:
                below = (sr_id, major, minor - 1)
                if below in matrix and matrix[below][column]:
                    print(f"  DISAGREEMENT SR{sr_id}-{major}-{minor - 1}: the "
                          f"document puts the '{gate}' boundary at minor {minor}, "
                          f"but the firmware gate already matches one below it")
                    failures += 1

    # The SR48 two-window case, stated explicitly. The document's own note says
    # SR48-8-2 is where the production line picks the BMP581 back up, which means
    # 8-0 and 8-1 must NOT have it even though the earlier 7-2 dev build does.
    # A plain ">= 7.2" in the firmware would pass every check above and still get
    # these two wrong, so they are asserted directly.
    bmp581 = GATE_COLUMN["BMP581 replaces BMP390"]
    for major, minor, want in ((7, 2, True), (8, 0, False), (8, 1, False), (8, 2, True)):
        key = (48, major, minor)
        if key not in matrix:
            print(f"  FAIL SR48-{major}-{minor} is not in the dumped range")
            failures += 1
            continue
        checked += 1
        got = bool(matrix[key][bmp581])
        if got != want:
            print(f"  FAIL SR48-{major}-{minor}: firmware says "
                  f"{'BMP581' if got else 'no BMP581'}, the document's rev index "
                  f"says {'BMP581' if want else 'no BMP581'}")
            failures += 1

    if failures:
        print(f"\nboard revision cross-check: {failures} DISAGREEMENT(S)")
        print("\nThe firmware is the authority (see AGENTS.md). Work out which "
              "side is stale\nand fix that one - do not edit the document's "
              "tables just to match the code.")
        return 1

    print(f"\nboard revision cross-check: the firmware gates agree with "
          f"docs/SHIMMER3_BOARD_REVISIONS.md over {checked} revisions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
