#!/usr/bin/env python3
"""Reference implementation of the host timestamp-unwrap rule, and its vectors.

The rule itself is specified in docs/SHIMMER3_STREAMING_DATA_FORMAT.md section 2.1.
This file is the executable copy of it: the vectors in Test/conformance are
generated from the functions below, so the prose, the vectors and the four host
implementations that consume them cannot drift apart at the source.

  --check <json>   recompute every expectation and diff against the file (CI)
  --write <json>   regenerate the file from the definitions below
  --emit csharp    print the vectors as a C# static array, for a host whose test
                   project has no way to load a data file
  --emit swift     the same, as Swift

The vector *inputs* (id, width, window, raw sequence) live here; every expected
output is computed, never typed by hand.
"""

import argparse
import json
import sys

RTC_TICKS_PER_SECOND = 32768.0
INVALID_ZERO_WINDOW_TICKS = 32768
REORDER_PERIODS = 8
MAX_WINDOW_DIVISOR = 8
SCHEMA_VERSION = 1
REVISION = 1
SPEC = "docs/SHIMMER3_STREAMING_DATA_FORMAT.md#21-the-timestamp"


def reorder_window_ticks(sampling_rate_hz, modulo):
    """Ticks a sample may lag its predecessor and still count as reordered.

    Sized in sample periods rather than as a fraction of the modulo: a packet is
    reordered by a handful of samples, whereas a dropout that spans the counter's
    wrap point is most of a modulo. Sizing it by the modulo confuses the two -
    at 2^16 a dropout between 1.75 s and 2.0 s reads as a reorder and the wrap is
    silently lost.

    Zero - the branch disabled - when the rate is not known. Never guess: an
    unknown rate must not turn into an infinite window, which would classify
    every backward step as a reorder and lose every wrap.
    """
    if sampling_rate_hz is None:
        return 0.0
    rate = float(sampling_rate_hz)
    if rate != rate or rate in (float("inf"), float("-inf")) or rate <= 0.0:
        return 0.0
    window = REORDER_PERIODS * RTC_TICKS_PER_SECOND / rate
    # Clamped so a very low rate cannot produce a window at or above the modulo,
    # which would leave no backward step large enough to be read as a wrap.
    return min(window, modulo / float(MAX_WINDOW_DIVISOR))


def unwrap_sequence(raw_values, modulo, window):
    """Run the rule over a whole sequence. Returns (unwrapped, rejected, cycle).

    State is the previous *raw* value and the previous unwrapped value. Hosts
    that keep a wrap count instead derive the raw value back out of it; the two
    formulations agree, but only if the comparison is made on the modular
    forward distance as it is here. Comparing the unwrapped candidate against
    the previous unwrapped value instead misses a packet that arrives late from
    before a wrap boundary, and reads the next sample as a second wrap.
    """
    unwrapped_out = []
    rejected_out = []
    last_raw = None
    last_unwrapped = 0.0

    for raw in raw_values:
        if last_raw is None:
            last_raw = raw
            last_unwrapped = float(raw)
            unwrapped_out.append(last_unwrapped)
            rejected_out.append(False)
            continue

        forward = (raw - last_raw) % modulo
        backwards = modulo - forward

        if forward == 0:
            # A duplicate: hold the timeline where it is.
            candidate = last_unwrapped
        elif backwards <= window:
            # 1. Reordered or duplicated packet, on either side of a boundary.
            #    Placed where it was actually taken, which is below its
            #    predecessor - honest rather than monotonic.
            candidate = last_unwrapped - backwards
        elif modulo == (1 << 24) and raw == 0 and last_raw < modulo - INVALID_ZERO_WINDOW_TICKS:
            # 2. A record the firmware never stamped. The timeline holds and the
            #    caller is told; nothing about this sample moves the state, so
            #    the next real sample is an ordinary step forward.
            unwrapped_out.append(last_unwrapped)
            rejected_out.append(True)
            continue
        else:
            # 3. Forward motion - a wrap when raw < last_raw. This is the
            #    DEFAULT, which is what keeps a wrap preceded by heavy packet
            #    loss classified as a wrap.
            candidate = last_unwrapped + forward

        last_raw = raw
        last_unwrapped = candidate
        unwrapped_out.append(candidate)
        rejected_out.append(False)

    cycle = 0 if not unwrapped_out else int(unwrapped_out[-1] // modulo)
    return unwrapped_out, rejected_out, cycle


# (id, description, bits, window, raw[])
VECTOR_INPUTS = [
    ("monotonic-24bit",
     "Ordinary forward motion at 504.123 Hz; nothing is classified.",
     24, 520, [1000, 1065, 1130]),
    ("wrap-24bit",
     "A genuine roll-over: the counter reached its last tick and started again.",
     24, 520, [16777200, 16]),
    ("wrap-lands-on-zero-24bit",
     "A roll-over that lands exactly on zero. Its predecessor is at the top of "
     "the range, which is what separates it from a record that was never stamped.",
     24, 520, [16777116, 0]),
    ("invalid-zero-signature-24bit",
     "The signature recovered from an affected recording: a stall, then a record "
     "the firmware never stamped. Read as a roll-over it costs 512 s.",
     24, 520, [7406116, 7406506, 0, 7406571]),
    ("invalid-zero-no-cascade-24bit",
     "Rejecting one record must not disturb the next: the following sample reads "
     "above the retained predecessor and is accepted normally.",
     24, 520, [7406506, 0, 7406571, 7406636]),
    ("first-sample-zero-24bit",
     "A stream may legitimately open on zero; nothing precedes it to contradict it.",
     24, 520, [0, 65, 130]),
    ("wrap-16bit",
     "The 2-byte counter older firmware uses wraps every 2 s.",
     16, 5120, [65436, 28]),
    ("zero-on-16bit-is-a-wrap",
     "The invalid-zero rule is scoped to the 3-byte counter. A 2-byte counter's "
     "whole range is 2 s, so a stall really can cross it.",
     16, 5120, [30000, 0]),
    ("backward-step-outside-window-is-a-wrap-24bit",
     "Only an exact zero is exempt. A value of 1 is read as a roll-over, as before.",
     24, 520, [7406506, 1]),
    ("duplicate-24bit",
     "The same counter value twice: hold the timeline, do not count a wrap.",
     24, 520, [1000, 1065, 1065, 1130]),
    ("reorder-one-period-24bit",
     "Two adjacent packets swapped. Each is placed where it was taken, so the "
     "output is not monotonic - and no modulo is added.",
     24, 520, [1000, 1130, 1065, 1195]),
    ("reorder-one-period-16bit",
     "The same swap on the 2-byte counter at 51.2 Hz (640 ticks per sample).",
     16, 5120, [40000, 41280, 40640, 41920]),
    ("reorder-across-wrap-boundary-24bit",
     "A packet arriving late from BEFORE a roll-over. In the modular form this is "
     "a small step back across the boundary; a host comparing unwrapped values "
     "instead sees forward motion of nearly a modulo and never recovers.",
     24, 520, [16777206, 5, 16777206, 70]),
    ("wrap-after-heavy-loss-24bit",
     "A roll-over preceded by a long dropout. Forward motion is the default, so "
     "this stays a wrap however much was lost before it.",
     24, 520, [16000000, 100]),
    ("wrap-after-heavy-loss-16bit",
     "The same on the 2-byte counter.",
     16, 5120, [60000, 1000]),
    ("wrap-spanning-dropout-1p8s-16bit",
     "A 1.8 s dropout across the 2 s counter - an ordinary Bluetooth gap. The "
     "window must be small enough that this is still read as forward motion.",
     16, 5120, [60000, 53446]),
    ("wrap-spanning-dropout-152s-24bit",
     "A long dropout spanning the 3-byte counter's roll-over.",
     24, 520, [16000000, 4222784]),
    ("rate-unknown-backward-step-is-a-wrap-24bit",
     "With no rate the reorder branch is disabled, so a swapped pair is read as a "
     "roll-over. Worse than knowing the rate, identical to older hosts, and safe.",
     24, 0, [1000, 1130, 1065, 1195]),
    ("rate-unknown-zero-still-rejected-24bit",
     "Rejecting an unstamped record needs no rate, so it still happens.",
     24, 0, [7406506, 0, 7406571]),
    ("zero-within-window-of-origin-24bit",
     "A zero close enough to the origin to be a reordered packet is kept as one, "
     "not rejected - the reorder test is applied first.",
     24, 520, [300, 365, 0, 430]),
    ("zero-within-window-after-wrap-24bit",
     "The origin recurs after every roll-over, so the same ordering applies there.",
     24, 520, [16777100, 100, 165, 0, 230]),
    ("reorder-window-boundary-inclusive-24bit",
     "Exactly at the window: reordered. 512 Hz gives a window of exactly 512 ticks "
     "in every language, with no rounding to argue about.",
     24, 512, [10512, 10000]),
    ("reorder-window-boundary-exclusive-24bit",
     "One tick past the window: a roll-over.",
     24, 512, [10513, 10000]),
    ("low-rate-clamp-16bit",
     "At 1 Hz the unclamped window would exceed the 2-byte modulo and no backward "
     "step could ever be a wrap. The clamp keeps wraps detectable.",
     16, 8192, [60000, 1000]),
    ("high-rate-reorder-24bit",
     "At 1024 Hz the window is 256 ticks; a swapped pair is still caught.",
     24, 256, [5000, 5032, 5000, 5064]),
    ("reorder-beyond-eight-periods-is-a-wrap-24bit",
     "A packet more than eight sample periods late is indistinguishable from a "
     "roll-over and is read as one. This is the limit of what a counter can say.",
     24, 256, [5000, 5288, 5000]),
    ("reorder-onto-origin-then-earlier-packet-24bit",
     "A reorder that lands exactly on the counter's origin, followed by a packet "
     "from just before it. A host that keeps an unwrapped value and a cycle count "
     "rather than the previous raw value has to encode 'no sample yet' somehow, "
     "and (0, 0) is the obvious choice - but this sequence reaches (0, 0) mid "
     "stream, so that host reads the third packet as a first sample and places it "
     "a whole modulo late. The state has to be distinguishable from the value.",
     24, 520, [520, 0, (1 << 24) - 16]),
]

# (samplingRateHz, timestampBits, tolerance)
DERIVATION_INPUTS = [
    (32768.0 / 65, 24, 0.0),
    (51.2, 16, 0.0),
    (51.2, 24, 0.0),
    (512.0, 24, 0.0),
    (1024.0, 24, 0.0),
    (1.0, 16, 0.0),
    (1.0, 24, 0.0),
    (0.0, 24, 0.0),
    (None, 24, 0.0),
    (-5.0, 24, 0.0),
    (312500.0 / 610, 24, 1e-4),
]


def build():
    vectors = []
    for vid, description, bits, window, raw in VECTOR_INPUTS:
        modulo = 1 << bits
        unwrapped, rejected, cycle = unwrap_sequence(raw, modulo, window)
        ticks_per_sample = window / REORDER_PERIODS if window else None
        vectors.append({
            "id": vid,
            "description": description,
            "timestampBits": bits,
            "modulo": modulo,
            "samplingRateHz": (RTC_TICKS_PER_SECOND / ticks_per_sample) if ticks_per_sample else None,
            "ticksPerSample": ticks_per_sample,
            "reorderWindowTicks": window,
            "raw": list(raw),
            "expectedUnwrapped": [int(v) for v in unwrapped],
            "expectedRejected": rejected,
            "expectedFinalCycle": cycle,
        })

    cases = []
    for rate, bits, tolerance in DERIVATION_INPUTS:
        cases.append({
            "samplingRateHz": rate,
            "timestampBits": bits,
            "expectedReorderWindowTicks": reorder_window_ticks(rate, 1 << bits),
            "tolerance": tolerance,
        })

    return {
        "schemaVersion": SCHEMA_VERSION,
        "revision": REVISION,
        "spec": SPEC,
        "ticksPerSecond": int(RTC_TICKS_PER_SECOND),
        "invalidZeroWindowTicks": INVALID_ZERO_WINDOW_TICKS,
        "reorderPeriods": REORDER_PERIODS,
        "maxWindowDivisor": MAX_WINDOW_DIVISOR,
        "windowDerivation": {
            "rule": "0 when the rate is unknown, NaN, infinite or <= 0; otherwise "
                    "min(reorderPeriods * ticksPerSecond / rateHz, modulo / maxWindowDivisor). "
                    "The tick domain is the 32768 Hz real-time clock the packet counter runs on, "
                    "never a TCXO sampling clock.",
            "cases": cases,
        },
        "vectors": vectors,
    }


def dump(doc):
    return json.dumps(doc, indent=2) + "\n"


def cmd_check(path):
    with open(path, "r", encoding="utf-8") as handle:
        on_disk = json.load(handle)
    expected = build()
    if on_disk == expected:
        print("%s: %d vectors, %d derivation cases, revision %d - all expectations reproduce"
              % (path, len(expected["vectors"]), len(expected["windowDerivation"]["cases"]),
                 expected["revision"]))
        return 0

    print("%s does not match the reference implementation." % path, file=sys.stderr)
    by_id = {v["id"]: v for v in on_disk.get("vectors", [])}
    for vector in expected["vectors"]:
        found = by_id.pop(vector["id"], None)
        if found is None:
            print("  missing vector: %s" % vector["id"], file=sys.stderr)
        elif found != vector:
            print("  vector differs: %s" % vector["id"], file=sys.stderr)
            for key in vector:
                if found.get(key) != vector[key]:
                    print("    %s: file %r, expected %r" % (key, found.get(key), vector[key]),
                          file=sys.stderr)
    for extra in by_id:
        print("  unexpected vector: %s" % extra, file=sys.stderr)
    for key in expected:
        if key != "vectors" and on_disk.get(key) != expected[key]:
            print("  %s: file %r, expected %r" % (key, on_disk.get(key), expected[key]),
                  file=sys.stderr)
    print("Regenerate with --write.", file=sys.stderr)
    return 1


def cmd_write(path):
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(dump(build()))
    print("wrote %s" % path)
    return 0


def cs_literal(value):
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    return repr(value)


def cmd_emit_csharp():
    doc = build()
    print("// Generated by Test/host/crosscheck_timestamp_unwrap.py --emit csharp")
    print("// Source: Test/conformance/timestamp_unwrap.json revision %d" % doc["revision"])
    print("// Do not hand-edit; regenerate when the vector file changes.")
    print("internal static readonly UnwrapVector[] Vectors =")
    print("{")
    for vector in doc["vectors"]:
        raw = ", ".join(str(v) for v in vector["raw"])
        unwrapped = ", ".join(str(v) for v in vector["expectedUnwrapped"])
        rejected = ", ".join("true" if v else "false" for v in vector["expectedRejected"])
        print("    new UnwrapVector(")
        print('        "%s",' % vector["id"])
        print("        %d, %s," % (vector["modulo"], repr(float(vector["reorderWindowTicks"]))))
        print("        new double[] { %s }," % raw)
        print("        new double[] { %s }," % unwrapped)
        print("        new bool[] { %s }," % rejected)
        print("        %d)," % vector["expectedFinalCycle"])
    print("};")
    print()
    print("internal const int VectorCount = %d;" % len(doc["vectors"]))
    return 0


def cmd_emit_swift():
    """The same vectors as a Swift source file.

    Swift's XCTest bundles can carry a resource, but wiring one into a classic
    Xcode project means four hand-edited entries in project.pbxproj for a file
    nothing compiles. A generated array costs one entry and cannot go stale:
    this function is the only thing that writes it, and it reads the same
    definitions the JSON is built from.
    """
    doc = build()
    print("// Generated by Test/host/crosscheck_timestamp_unwrap.py --emit swift")
    print("// Source: Test/conformance/timestamp_unwrap.json revision %d" % doc["revision"])
    print("// Do not hand-edit; regenerate when the vector file changes.")
    print()
    print("let sharedVectorRevision = %d" % doc["revision"])
    print("let sharedTicksPerSecond = %d" % doc["ticksPerSecond"])
    print("let sharedInvalidZeroWindowTicks = %d" % doc["invalidZeroWindowTicks"])
    print("let sharedReorderPeriods = %d" % doc["reorderPeriods"])
    print("let sharedMaxWindowDivisor = %d" % doc["maxWindowDivisor"])
    print()
    print("let sharedUnwrapVectors: [UnwrapVector] = [")
    for vector in doc["vectors"]:
        raw = ", ".join(str(v) for v in vector["raw"])
        unwrapped = ", ".join(str(v) for v in vector["expectedUnwrapped"])
        rejected = ", ".join("true" if v else "false" for v in vector["expectedRejected"])
        print("    UnwrapVector(")
        print('        id: "%s",' % vector["id"])
        print("        modulo: %d," % vector["modulo"])
        print("        reorderWindowTicks: %s," % repr(float(vector["reorderWindowTicks"])))
        print("        raw: [%s]," % raw)
        print("        expectedUnwrapped: [%s]," % unwrapped)
        print("        expectedRejected: [%s]," % rejected)
        print("        expectedFinalCycle: %d)," % vector["expectedFinalCycle"])
    print("]")
    print()
    print("let sharedWindowDerivationCases: [WindowDerivationCase] = [")
    for case in doc["windowDerivation"]["cases"]:
        rate = case["samplingRateHz"]
        rate_literal = "nil" if rate is None else repr(float(rate))
        print("    WindowDerivationCase(")
        print("        samplingRateHz: %s," % rate_literal)
        print("        timestampBits: %d," % case["timestampBits"])
        print("        expectedReorderWindowTicks: %s,"
              % repr(float(case["expectedReorderWindowTicks"])))
        print("        tolerance: %s)," % repr(float(case["tolerance"])))
    print("]")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", metavar="JSON")
    parser.add_argument("--write", metavar="JSON")
    parser.add_argument("--emit", choices=["csharp", "swift"])
    args = parser.parse_args()

    if args.check:
        return cmd_check(args.check)
    if args.write:
        return cmd_write(args.write)
    if args.emit == "csharp":
        return cmd_emit_csharp()
    if args.emit == "swift":
        return cmd_emit_swift()
    parser.print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())
