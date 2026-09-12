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

## Working copies
There is no standalone clone on this machine. Worktrees are hosted from the submodule gitdir inside
shimmer3r-firmware (`.git/modules/LogAndStream_Shimmer3R/log-and-stream-common/`), which means they
break if that superproject is re-cloned or the submodule is deinitialised.
