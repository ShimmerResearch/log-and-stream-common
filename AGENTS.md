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
`docs/` holds 21 reference documents covering both platforms — protocol, memory maps, calibration,
SD format, timekeeping, board revisions. Consult them before reading code. Note the naming split:
`SHIMMER3_*` is shared or Shimmer3-specific, `SHIMMER3R_*` is Shimmer3R-only.

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
| `Test/` | `SHIMMER3R_FACTORY_TEST_REPORT.md` |
| `Platform/` | `SHIMMER3R_PERIPHERAL_ALLOCATION.md` |

Change behaviour in a subsystem, update its doc **in the same PR**. Scope this to behaviour —
register maps, protocol bytes, memory layouts, state machines — not renames or refactors that change
nothing observable.

Two docs are derived from elsewhere and should not be edited to match code: `SHIMMER3_BOARD_REVISIONS.md`
comes from `Shimmer_PCBREV_INDEX.xlsx`, and anything it feeds. If code and doc disagree there, raise it
in the PR — the workbook is the source of truth, and the code may be what is wrong.
