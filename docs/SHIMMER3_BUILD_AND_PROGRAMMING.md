# Shimmer3 / Shimmer3R Build and Programming

Building both firmwares, including headless, and getting the result onto a
device. The counterpart to a DFU document: this platform pair updates over a
wire, not over the air.

> **Verified against** — the revisions these claims were read from.
>
> - `log-and-stream-common` @ `f3cf73e`; `shimmer3-firmware` @ `2765ff4`;
>   `shimmer3r-firmware` @ `a8f105e5`.
> - Toolchain observations are from the build configurations present in those
>   repositories at those revisions.

**Source references:**

| Layer | Location |
|---|---|
| Shimmer3 project | `shimmer3-firmware/LogAndStream_Shimmer3/` |
| Shimmer3R project | `shimmer3r-firmware/LogAndStream_Shimmer3R/` |
| Shared module | `log-and-stream-common/` (submodule of both) |
| Version numbers | [SHIMMER3_RELEASE_AND_VERSIONING.md](SHIMMER3_RELEASE_AND_VERSIONING.md) |
| Bootloader entry | [SHIMMER3_DOCK_PROTOCOL.md](SHIMMER3_DOCK_PROTOCOL.md) §4.2 |

---

## 1. The two toolchains

| | Shimmer3 | Shimmer3R |
|---|---|---|
| MCU | MSP430F5437A | STM32U5 |
| IDE | Code Composer Studio | STM32CubeIDE |
| Compiler | TI MSP430 | GCC ARM |
| Headless binary | `eclipsec.exe` | `stm32cubeidec.exe` |
| Configurations | `Debug`, `Release` | `Debug`, `Release` |
| Linker script | `lnk_msp430f5437a.cmd` | `.ld` |

Both are Eclipse-based, so both support headless workspace builds — which is
what makes CI and a scripted both-platform check possible.

## 2. Cloning

`log-and-stream-common` is a submodule of both platform repositories:

```bash
git clone --recurse-submodules <platform-repo>
```

or, after a plain clone:

```bash
git submodule update --init --recursive
```

> **The submodule pin is per platform repository.** Changing `common` and
> pushing does not change either platform until its pin is bumped. A change can
> therefore be "merged" and still absent from both firmwares.

## 3. Building both platforms for a shared change

**Any change to `log-and-stream-common` must build on both.** A change guarded
by `#if defined(SHIMMER3R)` still has to leave the Shimmer3 build valid, and
the two compilers disagree about enough — integer widths, `__attribute__`
support, warning sets — that "it compiles for one" is weak evidence.

The A/B recipe:

1. Note the current submodule pin in each platform repository.
2. Point each at the revision under test.
3. Build each headless (§4, §5) and capture the full log.
4. Compare error and warning counts against the recorded baseline for that
   platform.
5. Restore the original pins if you were only testing.

> **Compare against a baseline, not against zero.** Both projects build with
> pre-existing warnings, so "the build produced warnings" is not a signal. The
> signal is a *change* in the count. Record the baseline before you start.

> **`PLATFORM_WEAK` defaults hide missing work at link time.** A shared change
> that needs new platform support will link on a platform that has not
> implemented it, and fail silently at run time. See
> [SHIMMER3_ARCHITECTURE_OVERVIEW.md](SHIMMER3_ARCHITECTURE_OVERVIEW.md) §3.

## 4. Shimmer3 — Code Composer Studio

### 4.1 Headless build

CCS ships an `eclipsec.exe` that drives a workspace build from the command
line, taking a workspace directory, a project to import, and a build
configuration. Capture stdout and stderr; the exit code alone does not
distinguish "built with warnings" from "built clean".

### 4.2 Shimmer3 ships from the *Debug* configuration

> **This is the single most surprising fact about this build.** Releases are
> produced from the **Debug** CCS configuration. The Release configuration does
> not build, and that is a known, parked issue — not a regression to
> investigate and not something to "fix" as a side quest.

If you are producing a Shimmer3 release, build `Debug`. If you are evaluating
whether a change is safe, build `Debug` — that is the configuration that ships.

### 4.3 Do not commit regenerated makefiles

A headless build can rewrite the generated `Release/*.mk` files even when you
built `Debug`. Those rewrites are noise: check `git status` after a headless
build and revert any generated makefile changes you did not intend.

### 4.4 Programming

The MSP430 is programmed over its JTAG/Spy-Bi-Wire header from CCS, or through
the bootstrap loader. There is no over-the-air path.

## 5. Shimmer3R — STM32CubeIDE

### 5.1 Headless build

`stm32cubeidec.exe` drives a headless workspace build with `-importAll` /
`-import` and `-build`.

> **`-import` needs a Windows path with backslashes.** A forward-slash path
> produces `no file system for scheme: C` — Eclipse parses the `C:` as a URI
> scheme. This is the single most common way a scripted Shimmer3R build fails,
> and the message does not suggest the cause.

### 5.2 Programming

| Route | Notes |
|---|---|
| SWD | Debugger, from the IDE or `STM32_Programmer_CLI` |
| USB DFU | Via the STM32 system bootloader |
| Firmware update from the application | `UART_PROP_ENTER_BOOTLOADER` over the dock protocol, or `TASK_JUMP_TO_BOOT_LOADER` |

`TASK_JUMP_TO_BOOT_LOADER` exists only on Shimmer3R
([SHIMMER3_ARCHITECTURE_OVERVIEW.md](SHIMMER3_ARCHITECTURE_OVERVIEW.md) §4).
While the bootloader check is pending, the lower LED flashes purple —
the highest-priority indication there is
([SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) §6).

### 5.3 The version record

Confirm `fw_version_struct` survives `--gc-sections`; the `.version` section
needs an explicit `KEEP`. See
[SHIMMER3_RELEASE_AND_VERSIONING.md](SHIMMER3_RELEASE_AND_VERSIONING.md) §3.

## 6. Formatting

Both platform repositories run automatic formatters in CI that **commit onto
the pull-request branch**:

- `clang-format` on the firmware repositories.
- `prettier` (pinned) on the web repositories, HTML only.

Two consequences:

> **Run the pinned formatter locally before pushing**, or the bot will push a
> commit on top of yours and any follow-up push will be rejected as non-fast-
> forward. Fetch before pushing again.

> **Line-ending noise is not a formatting problem.** With `core.autocrlf=true`,
> a local formatter check can flag every file on line endings alone while CI —
> which checks out LF — sees nothing. Do not "fix" files the bot is not
> touching.

## 7. Build-time configuration

Behaviour is selected by preprocessor symbols rather than runtime
configuration:

| Symbol | Effect |
|---|---|
| `SHIMMER3` / `SHIMMER3R` / `SHIMMER4_SDK` | Selects the platform branch throughout `common` |
| `USE_FATFS` | Compiles the SD card paths |
| `USE_SD`, `USE_BT` | Compile the logging and Bluetooth paths |
| `USE_FREERTOS` | Selects the RTOS task-list variant (neither generation uses it) |
| `TEST_TASK_MONITOR` | Enables the stuck-task watchdog |
| `TEST_RTC_ERR_FLASH_OFF` | Suppresses the RTC-not-set LED warning |
| `IS_SUPPORTED_TCXO`, `IS_SUPPORTED_SINGLE_TOUCH` | Capability gates enforced in configuration validation |

> **`TEST_*` symbols are not test-only.** `TEST_TASK_MONITOR` gates the
> stuck-task watchdog, so a build without it has no protection at all, and
> `TEST_RTC_ERR_FLASH_OFF` removes a user-visible warning. Check what a shipping
> build actually defines rather than assuming the names mean development-only.

## 8. Verifying a build on hardware

The device's own factory self-test is the fastest confidence check: it reports
the firmware version it is running as its first line, then exercises every bus
and IC. See
[SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md).

## Still unverified / not found in code

- **Exact headless command lines.** The `eclipsec.exe` and `stm32cubeidec.exe`
  invocations vary by IDE version and installation path, and the working
  recipes are held outside these repositories. The gotchas in §4.1, §4.3 and
  §5.1 are the parts that are stable across versions; the argument lists are
  not reproduced here because a stale command line is worse than none.
- **Baseline warning counts.** Both projects build with pre-existing warnings
  and §3 depends on knowing the number, but the counts move with toolchain
  version and are not recorded in either repository. Capture your own before
  relying on the comparison.
- ~~Whether CI builds both platforms on a `common` change~~ — resolved: **no**.
  `log-and-stream-common/.github/workflows` holds only `clang-format-check.yml`.
  Each platform repository has a `build-release-firmware.yml`, but both are
  `workflow_dispatch` only (manual: pick the version part to bump and the
  build mode) — Shimmer3R via `xanderhendriks/action-build-stm32cubeide`,
  Shimmer3 via a cached CCS install under `/opt/ti/ccs`. Nothing builds on push
  or pull request, so a `common` change is compile-checked only when someone
  runs those workflows or builds locally.
- **The Shimmer3 Release configuration failure.** Known and parked; not
  reproduced for this document, but the `.cproject` offers a likely cause: the
  `Debug` configuration defines `__MSP430F5437A__ SHIMMER3`, the `Release`
  configuration defines only `__MSP430F5437A__`. Without `SHIMMER3` the shared
  module's platform selection has no target.
- ~~STM32 bootloader specifics~~ — resolved as far as the firmware goes. Two
  entry paths, both into ST's **system-memory bootloader at `0x0BF90000`**
  (`Shimmer_Driver/hal_bootloader.c`): (1) at boot, `JumpToBootloaderIfRequired`
  polls the user button every 100 ms and jumps once it has been held for
  `BOOTLOADER_ENTRY_THRESHOLD_MS`; (2) over the dock, `UART_PROP_ENTER_BOOTLOADER`
  (`0x09`) arms an RTC alarm that reboots into the bootloader after the given
  number of seconds (`0` cancels). `checknBoot0OptionByte` keeps the `nBOOT0`
  option byte in the state the board revision needs. The protocol is whatever
  ST's bootloader speaks on the enumerated interface (USB DFU, or UART on
  `PA2`/`PA3` `BSL_TX`/`BSL_RX`); there is **no anti-rollback** — the system
  bootloader has none and the application adds none.
- ~~Which build-time symbols shipping builds define~~ — resolved from the two
  `.cproject` files: Shimmer3 (`Debug`, the configuration that ships)
  `__MSP430F5437A__ SHIMMER3`; Shimmer3R `SHIMMER3R USE_HAL_DRIVER
  UX_INCLUDE_USER_DEFINE_FILE DEBUG` plus the CubeMX device symbol. Everything
  else in §7 is at its header default in both.
