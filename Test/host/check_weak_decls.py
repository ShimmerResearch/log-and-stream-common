#!/usr/bin/env python3
"""No header in this repository may declare a function weak.

WHY. The library gives each platform hook a weak default definition, and each
firmware replaces the ones it needs with a plain definition of its own. A weak
attribute on the hook's prototype in a header defeats that: every definition
that sees the prototype becomes weak too, the firmware's override included, and
the linker keeps whichever weak copy it meets first. The override then wins or
loses by link order, and nothing reports which.

The attribute belongs on the default definition in the .c file and nowhere
else. test_platform_seam.c proves that at link time for the hooks whose
defaults the host build can compile; this check covers every header, including
the ones the host build cannot include yet.

  a declaration in a header carrying __weak, PLATFORM_WEAK or
  __attribute__((weak))                                                 FAIL

Defining the macros (#define __weak ...) is fine, as is mentioning them in a
comment.

Run by .github/workflows/host-tests.yml. Needs no compiler and no build.
"""
import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]

WEAK = re.compile(r"\b(__weak|PLATFORM_WEAK)\b|__attribute__\s*\(\(\s*(__)?weak(__)?\b")
COMMENT = re.compile(r"/\*.*?\*/|//[^\n]*", re.S)


def blank_comments(text):
    """Comments out, line numbers kept."""
    return COMMENT.sub(lambda m: "\n" * m.group(0).count("\n"), text)


def offending_lines(path):
    lines = blank_comments(path.read_text(encoding="utf-8", errors="replace")).splitlines()
    in_directive = False
    for number, line in enumerate(lines, start=1):
        stripped = line.strip()
        directive = in_directive or stripped.startswith("#")
        in_directive = directive and stripped.endswith("\\")
        if not directive and WEAK.search(line):
            yield number, stripped


def main():
    # Hidden directories hold no source of ours, and can hold whole other
    # checkouts of this repository (.git/, editor and agent worktrees).
    headers = sorted(
        p
        for p in REPO.rglob("*.h")
        if not any(part.startswith(".") for part in p.relative_to(REPO).parts)
    )
    failures = [
        (header.relative_to(REPO).as_posix(), number, text)
        for header in headers
        for number, text in offending_lines(header)
    ]

    for name, number, text in failures:
        print(f"  FAIL {name}:{number}: {text}")

    if failures:
        print(
            "\nA header declares a function weak, which makes every override of\n"
            "it weak as well. Put the attribute on the default definition in the\n"
            ".c file and leave the prototype plain."
        )
        return 1

    print(f"weak declarations: none in {len(headers)} headers")
    return 0


if __name__ == "__main__":
    sys.exit(main())
