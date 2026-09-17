# log-and-stream-common

Hardware-agnostic C library implementing the LogAndStream behaviour — SD logging, BT streaming,
sensor config and calibration, battery, LEDs, button, RTC, EEPROM, task scheduling.

## This repo ships to two different MCUs
It is consumed as a submodule by **both**:
- `ShimmerResearch/shimmer3-firmware` — Shimmer3, **MSP430**, TI Code Composer
- `ShimmerResearch/shimmer3r-firmware` — Shimmer3R, **STM32U5**, STM32CubeIDE

Every change lands on both. That is the single most important thing to hold in mind here:
a fix that assumes STM32 word sizes, endianness, timer behaviour or toolchain builtins will
break the MSP430 build, and nothing in this repo will tell you at edit time.

Anything platform-specific goes behind the abstraction — `log_and_stream_externs.h` declares the
functions each platform firmware must implement. Add to that contract rather than `#ifdef`-ing by MCU.

## Layout
`log_and_stream_common.c/h` is the lifecycle entry point. `log_and_stream_globals.h` holds shared
state (`shimmerStatus`, `batteryStatus`). Subsystems are one directory each: `Comms/`, `SDCard/`,
`Sensing/`, `Calibration/`, `Battery/`, `Button/`, `LEDs/`, `RTC/`, `EEPROM/`, `SDSync/`, `TaskList/`.

## Read the docs first
`docs/` holds 22 reference documents covering both platforms — protocol, memory maps, calibration,
SD format, timekeeping, board revisions. Consult them before reading code. Note the naming split:
`SHIMMER3_*` is shared or Shimmer3-specific, `SHIMMER3R_*` is Shimmer3R-only.

## Test before you push
`make -C Test/host` builds and runs the whole host suite in about ten seconds, with no cross-compiler
and no device. CI runs the same Makefile, so green locally is green there.

`make -C Test/host platform-check` is the one to run first. It compiles every host-clean module for
**both** MCUs, which is the cheapest guard there is against the hazard at the top of this file — and
the only one that acts at edit time rather than at release time.

The suite covers the CRC, the sample ring, the string/number helpers, the RTC conversions, the
battery classification and the board revision gates; three of those are additionally cross-checked
against references that share no code with the firmware. `Test/host/README.md` has the mechanics and
`docs/SHIMMER3_TEST_PROCEDURE.md` §9 lists the next modules worth covering.

**What it cannot catch:** the host's `int` is 32 bits and the MSP430's is 16, so an expression that
overflows on Shimmer3 passes here every time. Add a test *and* build for the target.

`make -C Test/host host-constants` is the one to run after touching an opcode, a config byte or a
board code. It checks the firmware headers against the protocol document and the Python host
reference in `Extras/`, because a constant that moves here is silently wrong in every host that
restates it — Consensys included. It needs no compiler.

When you fix a bug, add the case that would have caught it, in the same PR.

## Formatting is applied for you, after the fact
`clang-format-check.yml` runs on every push with **`inplace: True`** and then commits the result back
as "Committing clang-format changes". It does not reject a badly formatted push — it reformats it and
pushes a commit on top of your branch. Two consequences:

- **Your branch moves under you.** Pull before your next push, and **fetch before tagging a release**,
  or the tag misses the formatting commit.
- **That commit gets no CI run of its own.** GitHub does not trigger workflows for pushes made with
  `GITHUB_TOKEN`, so the host tests that passed ran on the *pre-format* tree. Harmless for whitespace,
  worth knowing.

It fires often — around 15% of commits in this repo are auto-format commits, and they consistently
touch a subset of the files the preceding commit touched. That is not a tooling fault, it is the
formatter not being run before pushing.

**Run `.githooks/install.sh` (or `.githooks\install.bat`) once per clone and the bot commit never
appears.** The `pre-commit` hook clang-formats the `.c`/`.h` files staged for the commit and re-stages
them, so what you commit is already correct. Only staged files, so it is sub-second. It never blocks a
commit: no formatter on the machine, or a file only partly staged, and it says so and lets the commit
through. `git commit --no-verify` bypasses it. `.githooks/README.md` has the details.

From a firmware checkout the installer configures this repository too — commits made inside the
submodule are its commits, so it needs its own hook configuration.

> CI pins clang-format **17**; the bundled `clang-format.exe` is **18.1.8**. They currently produce
> byte-identical output on this codebase — reformatting `main` with 18 changes 0 of 60 files — so the
> difference is not a live problem. It is recorded because a future version bump on either side could
> make it one, and because it is otherwise the obvious thing to blame for churn it does not cause.

## Versioning
`scripts/increment_version.sh` is called by the *consuming* firmware's release workflow, not by this
repo. There is no release pipeline here.

## Keep the docs in step with the code
These docs are the authoritative source, and this file tells you to consult them before reading code.
That only holds while they are current — **a stale doc here is worse than no doc, because it will be
believed**, and believed on two platforms at once.

The mapping is the directory name: most subsystem folders have a doc named after them.

| Code | Doc |
|---|---|
| `Comms/` | `SHIMMER3_BT_COMMUNICATION_PROTOCOL.md`, `SHIMMER3_DOCK_PROTOCOL.md` |
| `Battery/` | `SHIMMER3_BATTERY_AND_CHARGING.md` |
| `Boards/` | `SHIMMER3_BOARD_REVISIONS.md` |
| `Calibration/` | `SHIMMER3_CALIBRATION.md` |
| `Configuration/` | `SHIMMER3_CONFIGURATION_INFOMEM.md` |
| `EEPROM/` | `SHIMMER3_EEPROM_MEMORY_MAP.md` |
| `GSR/` | `SHIMMER3_GSR_AUTORANGE.md` |
| `LEDs/` | `SHIMMER3_LED_FEEDBACK.md` |
| `RTC/` | `SHIMMER3_TIMEKEEPING.md` |
| `SDCard/`, `Sensing/` | `SHIMMER3_SD_CARD_FORMAT.md`, `SHIMMER3_STREAMING_DATA_FORMAT.md` |
| `SDSync/` | `SHIMMER3_SD_SYNC.md` |
| `Test/` | `SHIMMER3R_FACTORY_TEST_REPORT.md`, `SHIMMER3_TEST_PROCEDURE.md` |
| `Platform/` | `SHIMMER3R_PERIPHERAL_ALLOCATION.md` |

Change behaviour in a subsystem, update its doc **in the same PR**. Scope this to behaviour —
register maps, protocol bytes, memory layouts, state machines — not renames or refactors that change
nothing observable.

`SHIMMER3_BOARD_REVISIONS.md` is a special case. Its per-product tables are a conversion of an
internal hardware workbook that is **not in this repository**, so a reader outside Shimmer cannot
check them. Do not edit those tables to match code.

But the workbook is provenance, not specification: **where the tables and the firmware disagree, the
firmware wins.** The revision gates in that document name the functions that read them
(`ShimBrd_isBmp581PresentPerSrNumber()` and friends), and those are the authority for behaviour.
Report the disagreement rather than silently reconciling either side.

This repository is public. Keep internal paths, customer names and ticket identifiers out of the
docs and out of this file.
