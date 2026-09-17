# Host tests

Everything about this library that can be checked without an MCU.

```sh
make                 # build and run everything (~10 s)
make platform-check  # compile every host-clean module for BOTH MCUs
make test_rtc        # one suite, both platform builds
make clean
```

CI runs this same Makefile (`.github/workflows/host-tests.yml`), so a suite that
passes locally passes there, and adding a suite here adds it to CI.

The release procedure these sit inside is
[`docs/SHIMMER3_TEST_PROCEDURE.md`](../../docs/SHIMMER3_TEST_PROCEDURE.md).

## Why bother, when the device is right there

Three kinds of fault are cheap here and expensive or impossible on a bench:

- **Interleavings you cannot ask hardware for.** The DEV-1023 field fault needed
  an SD write to delay a gather past a fail-safe, in a particular order. Here it
  is a handful of function calls.
- **Inputs you do not have.** Asserting the BMP581 revision gate across its
  boundaries needs eight PCB revisions on the desk, some of which were never
  built in quantity. Here each one is three bytes.
- **Ranges too large to walk.** `test_rtc` converts every day from 2000 to 2099
  in both directions, which covers the leap-year rule and all twelve month
  lengths in a few milliseconds.

## What is here

| File | |
|---|---|
| `Makefile` | The runner. Suite lists at the top; add yours there |
| `host_test.h` | `expectU` / `expectX` / `expectStr` / `expectTrue`, and the tally |
| `host_stubs.{c,h}` | The PC "platform" — see below |
| `stubs/` | Headers the consuming firmware owns. `stubs/README.md` explains each |
| `test_*.c` | The suites |
| `crosscheck_*.py` | Comparisons against references that share no code with the firmware |

`crosscheck_host_constants.py` is the odd one out: it needs no compiler and no
binary, so it runs from a bare checkout with `make host-constants`. It compares
the firmware headers against the protocol document and the Python host reference
in `Extras/`, and it is the only automated check in the repository that looks at
host compatibility at all — see `docs/SHIMMER3_TEST_PROCEDURE.md` §6.

## The platform seam

`log_and_stream_externs.h` declares what each platform firmware must implement.
Shimmer3 implements it against the MSP430 HAL, Shimmer3R against the STM32 HAL,
and `host_stubs.c` against nothing at all — **a third platform, for a PC**.

A module is host-testable exactly to the degree that it reaches the platform
through that contract. Anything that needs a new stub is telling you it reached
around the abstraction, which is worth knowing on its own.

Nothing in `stubs/` redefines a type, struct or constant from this repository.
Only the `log_and_stream_includes.h` aggregate is shadowed, and only to cut the
subsystems whose headers need firmware-side files that do not exist here. Every
struct layout a test sees is the firmware's own — a stubbed `gConfigBytes` would
drift from the real one and the tests would stay green while the device was
wrong.

## Two platforms, one library

This repository is a submodule of both `shimmer3-firmware` (MSP430) and
`shimmer3r-firmware` (STM32U5), and every change here ships to both.

`test_rtc`, `test_battery` and `test_boards` are therefore built and run
**twice**, once with `-DSHIMMER3` and once with `-DSHIMMER3R`. `make
platform-check` goes further and compiles every host-clean module both ways
without running it. That is the cheapest check there is that a change has not
broken the platform you were not looking at.

## What this cannot check

**The host compiler's `int` is 32 bits. The MSP430's is 16.** An expression that
overflows on Shimmer3 is silently correct here, every time. Only the Code
Composer build catches that class of fault, which is why it is a release gate
and not something CI stands in for.

Also out of reach: peripherals, timing, power, radio, and anything that needs a
card in a slot.

## Adding a suite

1. `test_<module>.c`, wrapped in `#if defined(SHIMMER_HOST_TEST)`. **The guard
   is not optional** — both firmware projects compile every `.c` under this
   repository with no exclusions, and your file defines `main()`. Without it you
   get a duplicate-symbol link failure in the MSP430 *and* STM32 builds, which
   is a confusing thing to inflict on whoever hits it next.
2. Include `host_test.h`. Include `log_and_stream_includes.h` if the module
   needs the platform.
3. Add `SRC_test_<module>` and put the name in one of the suite lists in the
   `Makefile`. Prefer `DUAL_PLATFORM_SUITES` wherever the module contains any
   `#if defined(SHIMMER3...)`.
4. Call `hostStub_reset()` at the top of each case. The stub state and the two
   globals are file-scope and shared across a binary, so a case that does not
   reset inherits whatever the last one left — which is how a suite starts
   passing in one order and failing in another.

`docs/SHIMMER3_TEST_PROCEDURE.md` §9 lists the next modules worth covering and
what each needs first.
