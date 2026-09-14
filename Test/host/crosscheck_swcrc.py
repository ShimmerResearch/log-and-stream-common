#!/usr/bin/env python3
"""Cross-check the firmware's software CRC against the Python host reference.

The firmware and the host tooling each carry their own implementation of the
same CRC. If they ever disagree, a host silently fails to verify CRCs the
device produced - a shipped-product bug that nothing else in this repository
would catch. This asserts they agree over every length the link can carry.

Run by .github/workflows/host-tests.yml, after building test_swcrc.

Usage: crosscheck_swcrc.py <path-to-test_swcrc-binary>
"""
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "Extras", "python_scripts", "Shimmer_common"))

import shimmer_crc  # the host reference implementation, unmodified


def corpus(n):
    """Must match fill_corpus() in test_swcrc.c exactly."""
    return [(i * 7 + 13) & 0xFF for i in range(n)]


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2

    out = subprocess.run([sys.argv[1], "--dump"], capture_output=True, text=True, check=True)
    lines = [ln for ln in out.stdout.splitlines() if ln.strip()]
    if not lines:
        print("FAIL: the C test produced no output")
        return 1

    buf = corpus(600)
    failures = 0
    for line in lines:
        length_str, crc_str = line.split(",")
        length = int(length_str)
        from_c = int(crc_str, 16)
        from_py = shimmer_crc.calc_crc(length, buf)
        if from_c != from_py:
            print(f"  FAIL len {length}: firmware 0x{from_c:04X} != host reference 0x{from_py:04X}")
            failures += 1
            if failures >= 10:
                print("  ... stopping after 10")
                break

    # The reference's own documented vectors, so a regression in shimmer_crc.py
    # is caught even if the C changed to match it.
    for length, want in ((8, 0x48AA), (9, 0x2A5D), (10, 0x8B17), (11, 0x794E)):
        got = shimmer_crc.calc_crc(length, list(range(1, 17)))
        if got != want:
            print(f"  FAIL host reference len {length}: 0x{got:04X} != 0x{want:04X}")
            failures += 1

    if failures:
        print(f"\ncross-check: {failures} FAILURE(S)")
        return 1
    print(f"\ncross-check: firmware and host reference agree over {len(lines)} lengths")
    return 0


if __name__ == "__main__":
    sys.exit(main())
