#!/usr/bin/env python3
"""Three-way check: firmware headers, the protocol document, the Python host.

WHY THIS EXISTS. From SHIMMER3_RELEASE_AND_VERSIONING.md §6: "The firmware
carries no compatibility logic. Every gate is host-side." That cuts both ways -
the firmware cannot protect a host from a change it did not expect, and it will
not tell you when one has happened. Worse, older firmware *silently ignores*
unknown opcodes rather than NACKing them, so a host that sends a command the
firmware does not implement gets no response at all, indistinguishable from a
dropped packet.

Extras/python_scripts/ carries a host implementation that restates firmware
constants in Python. It is the only host in this repository, so it is the only
one CI can check - and the drift it catches is the same class that breaks
Consensys, the Java driver and the web SDK, which restate the same constants
again and cannot be reached from here.

WHAT MAKES THIS WORK IS THE THIRD PARTY. A two-way firmware/host comparison is
too noisy to act on: the firmware has renamed opcodes for clarity (ACCEL ->
WR_ACCEL / LN_ACCEL, PRES -> PRESSURE) without touching a single value, and the
host deliberately carries opcodes the firmware never implemented. Neither is a
fault. SHIMMER3_BT_COMMUNICATION_PROTOCOL.md already classifies every opcode -
FW_ONLY, JAVA_ONLY, SDK_MISSING - so the document arbitrates, and only genuine
disagreements are left.

THE CONTRACT IS THE VALUE, NOT THE NAME. Opcodes are compared by number; a name
that differs at the same value is reported, never failed.

  a host opcode with no firmware opcode at that value, and not documented
  as host-only                                                          FAIL
  one value with two different documented meanings                      FAIL
  an enum value that disagrees between firmware and host                 FAIL
  names differing at a shared value                                     NOTE
  firmware opcodes the host reference does not carry                     NOTE

Run by .github/workflows/host-tests.yml. Needs no device and no build.
"""
import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]
HOST = REPO / "Extras" / "python_scripts" / "Shimmer_common"
PROTOCOL_DOC = REPO / "docs" / "SHIMMER3_BT_COMMUNICATION_PROTOCOL.md"

def firmware_opcodes():
    """{value: {names}} from #define NAME 0xNN - opcode-shaped only, so buffer
    sizes and bitmasks with a U suffix are excluded."""
    out = {}
    for line in (REPO / "Comms" / "shimmer_bt_uart.h").read_text().splitlines():
        m = re.match(r"#define\s+([A-Z][A-Z0-9_]*)\s+0x([0-9A-Fa-f]{1,2})\s*(?://.*)?$", line)
        if m:
            out.setdefault(int(m.group(2), 16), set()).add(m.group(1))
    if not out:
        raise SystemExit("FAIL: no opcodes parsed from Comms/shimmer_bt_uart.h")
    return out


# The opcode tables all carry this header. Anchoring on it keeps the parser off
# the document's other 0x-shaped tables - battery levels, SD file-transfer status
# codes, baud rate enums - which share no namespace with opcodes and whose values
# collide with them harmlessly.
OPCODE_TABLE_HEADER = (
    "| Opcode | FW name | Kind | Args | Response opcode | Response payload length "
    "| Gen | Blocked while sensing | Java name (if different) "
    "| SDK name (if different) | Notes |"
)
COL_FW_NAME = 1
COL_JAVA_NAME = 8
COL_NOTES = 10


def documented_opcodes():
    """{value: (fw_name_or_None, java_name_or_None, tag)} from the opcode tables."""
    out = {}
    in_table = False
    saw_header = False
    for line in PROTOCOL_DOC.read_text().splitlines():
        if line.strip() == OPCODE_TABLE_HEADER:
            in_table, saw_header = True, True
            continue
        if not line.startswith("|"):
            in_table = False
            continue
        if not in_table:
            continue

        cells = [c.strip().strip("`") for c in line.strip().strip("|").split("|")]
        if len(cells) <= COL_NOTES:
            continue
        m = re.fullmatch(r"0x([0-9A-Fa-f]{2})", cells[0])
        if not m:
            continue

        def cell(i):
            v = cells[i]
            return None if v in ("", "\u2014", "-") else v

        out.setdefault(int(m.group(1), 16), []).append(
            (cell(COL_FW_NAME), cell(COL_JAVA_NAME), cells[COL_NOTES])
        )

    if not saw_header:
        raise SystemExit(
            f"FAIL: the opcode table header was not found in {PROTOCOL_DOC.name}. "
            "If the table's columns changed, update OPCODE_TABLE_HEADER here in "
            "the same commit - otherwise this check silently stops checking."
        )
    if not out:
        raise SystemExit(f"FAIL: no opcode rows parsed from {PROTOCOL_DOC.name}")
    return out


def firmware_enum(path, name):
    text = path.read_text()
    m = re.search(r"enum\s+" + re.escape(name) + r"\s*\{(.*?)\}", text, re.S)
    if not m:
        raise SystemExit(f"FAIL: enum {name} not found in {path.name} - renamed?")
    return {k: int(v) for k, v in re.findall(r"([A-Z][A-Z0-9_]*)\s*=\s*(\d+)", m.group(1))}


def python_class_constants(path, class_name):
    text = path.read_text()
    m = re.search(r"^class\s+" + re.escape(class_name) + r"\b.*?:\n(.*?)(?=^\S)", text, re.S | re.M)
    if not m:
        raise SystemExit(f"FAIL: class {class_name} not found in {path.name}")
    out = {}
    for line in m.group(1).splitlines():
        d = re.match(r"\s+([A-Z][A-Z0-9_]*)\s*=\s*(0x[0-9A-Fa-f]+|\d+)\s*(?:#.*)?$", line)
        if d:
            out[d.group(1)] = int(d.group(2), 0)
    if not out:
        raise SystemExit(f"FAIL: class {class_name} in {path.name} yielded no constants")
    return out


def check_opcodes(failures, notes):
    renamed = []
    fw = firmware_opcodes()
    doc = documented_opcodes()
    host = python_class_constants(HOST / "shimmer_comms_bluetooth.py", "BtCmds")

    host_by_value = {}
    for name, value in host.items():
        host_by_value.setdefault(value, set()).add(name)

    for value, names in sorted(host_by_value.items()):
        if value in fw:
            if not (names & fw[value]):
                renamed.append(
                    f"0x{value:02X} {'/'.join(sorted(names))} -> "
                    f"{'/'.join(sorted(fw[value]))}"
                )
            continue

        documented = doc.get(value, [])
        # The document's "FW name" column is empty for an opcode the firmware has
        # never implemented. A host carrying one of those is correct by design -
        # the Java driver's legacy ExG calibration commands, for instance.
        if documented and all(fw_name is None for fw_name, _, _ in documented):
            continue

        failures.append(
            f"opcode 0x{value:02X} ({'/'.join(sorted(names))}): the host reference "
            f"sends it and no firmware opcode has that value"
            + (
                f", but the protocol document says the firmware implements it as "
                f"{documented[0][0]} - so it has been removed or renumbered"
                if documented
                else ", and the protocol document does not list it at all"
            )
        )

    # One value carrying two different documented meanings would make a packet
    # ambiguous on the wire - the worst failure in this file.
    for value, entries in sorted(doc.items()):
        distinct = {fw_name for fw_name, _, _ in entries if fw_name}
        if len(distinct) > 1:
            failures.append(
                f"opcode 0x{value:02X} is documented as two different firmware "
                f"commands: {', '.join(sorted(distinct))} - a packet carrying it "
                f"would be ambiguous on the wire"
            )

    if renamed:
        # Not a fault: the firmware tidied these names (ACCEL -> WR_ACCEL /
        # LN_ACCEL, PRES -> PRESSURE) without moving a value. Worth surfacing
        # because a reader comparing the two by name will otherwise think the
        # host is missing commands it in fact implements.
        notes.append(
            f"{len(renamed)} opcodes carry different names on the two sides at "
            f"the same value, so the wire is unaffected: "
            + "; ".join(renamed[:4])
            + (f"; ... and {len(renamed) - 4} more" if len(renamed) > 4 else "")
        )

    absent = sorted(v for v in fw if v not in host_by_value)
    if absent:
        notes.append(
            f"{len(absent)} firmware opcodes are not in the host reference "
            f"(expected for FW_ONLY and for commands newer than the scripts): "
            + ", ".join(f"0x{v:02X}" for v in absent[:10])
            + (" ..." if len(absent) > 10 else "")
        )
    return len(host_by_value)


def check_enum(failures, notes, label, fw_path, fw_enum, py_path, py_class, name_map=None):
    fw = firmware_enum(fw_path, fw_enum)
    py = python_class_constants(py_path, py_class)
    checked = 0

    for py_name, py_value in sorted(py.items()):
        fw_name = next((k for k, v in (name_map or {}).items() if v == py_name), py_name)
        checked += 1
        if fw_name not in fw:
            failures.append(
                f"{label}: the host reference defines {py_name} = {py_value}, "
                f"the firmware has no {fw_name}"
            )
        elif fw[fw_name] != py_value:
            failures.append(
                f"{label}: {fw_name} is {fw[fw_name]} in the firmware, "
                f"{py_name} is {py_value} in the host reference"
            )

    mapped = set((name_map or {}).keys()) or set()
    absent = sorted(set(fw) - set(py) - mapped)
    if absent:
        notes.append(f"{label}: not in the host reference - {', '.join(absent)}")
    return checked


def main():
    failures, notes = [], []
    checked = check_opcodes(failures, notes)
    checked += check_enum(
        failures, notes, "expansion board codes",
        REPO / "Boards" / "shimmer_boards.h", "SR_BOARD_CODES",
        HOST / "shimmer_device.py", "SrBoardCodes",
    )
    checked += check_enum(
        failures, notes, "hardware IDs",
        REPO / "Boards" / "shimmer_boards.h", "SR_HW_IDS",
        HOST / "shimmer_device.py", "SrHwVer",
        name_map={"HW_ID_SHIMMER3": "SHIMMER3", "HW_ID_SHIMMER3R": "SHIMMER3R"},
    )

    for note in notes:
        print(f"  NOTE {note}")

    if failures:
        print()
        for f in failures:
            print(f"  FAIL {f}")
        print(f"\nhost constants cross-check: {len(failures)} FAILURE(S)")
        print(
            "\nA host cannot detect this for itself: the firmware carries no\n"
            "compatibility logic, and older firmware ignores an unknown opcode\n"
            "rather than NACKing it, so the symptom is silence. If a constant\n"
            "moved deliberately, every host that restates it needs the same\n"
            "change - this reference, Consensys, the Java driver, the web SDK -\n"
            "and the protocol document needs to say so."
        )
        return 1

    print(
        f"\nhost constants cross-check: {checked} constants agree across the "
        f"firmware,\nthe protocol document and the Python host reference"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
