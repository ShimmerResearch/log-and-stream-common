# Shimmer3 / Shimmer3R Build and Programming

Building both firmwares, including headless, and getting the result onto a
device. The counterpart to a DFU document: this platform pair updates over a
wire, not over the air.

> **Verified against** — the revisions these claims were read from.
>
> - `log-and-stream-common` @ `f3cf73e`; `shimmer3-firmware` @ `2765ff4`;
>   `shimmer3r-firmware` @ `a8f105e5`.
> - `TEST_LOG_WITH_USB` (§7): `log-and-stream-common` @ `126e9fb` —
>   `log_and_stream_definitions.h`, `Sensing/shimmer_sensing.c`,
>   `log_and_stream_common.c`. Only that row was checked at this revision.
> - Toolchain observations are from the build configurations present in those
>   repositories at those revisions.
> - §4.5 is newer. Its settings were read from `shimmer3-firmware` @
>   `a311203`, with this repository @ `cea18c2` as its submodule. The stack and
>   printf settings arrive in `shimmer3-firmware` `641366c`, so a revision that
>   does not contain that commit still has the old 1600-byte stack and full
>   printf.
> - §4.5's stack figures and its one-handler bound are newer again. They were
>   measured with `ofd430 --call_graph` on the `-O0` Debug image of
>   `shimmer3-firmware` @ `ac30299`, with this repository @ `94ac899` as its
>   submodule, where Bluetooth start runs from the main loop. The
>   interrupt-context claims were read from `ShimBt_handleBtRfCommStateChange`,
>   `ShimSdSync_startBtForSync` and `TIMER0_A1_ISR` here, and from that
>   revision's `msp430_clock_enable()` and `BtStop()`. A build that lacks either
>   commit still lets interrupts nest, and needs room for `main` plus two
>   handlers.
> - §4.2's optimisation settings, and §4.5's `-O2` stack figures, are newer
>   still. The settings were read from `shimmer3-firmware` @ `a886ad7` with
>   Release raised to `-O2 --opt_for_speed=2`. The figures and §4.2's
>   throughput bench used `shimmer3-firmware` @ `d82c9ef` with this repository
>   @ `08c84d8` as its submodule.
> - What CI builds (§4.2, and "Whether CI builds both platforms" under "Still
>   unverified") was read from `build-check.yml` in `shimmer3-firmware` @
>   `f67cf0a` and `shimmer3r-firmware` @ `c8c69e71`.

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

```bat
eclipsec.exe -noSplash -data <ws> -application com.ti.ccstudio.apps.projectImport -ccs.location <path>\LogAndStream_Shimmer3
eclipsec.exe -noSplash -data <ws> -application com.ti.ccstudio.apps.projectBuild -ccs.projects LogAndStream_Shimmer3 -ccs.configuration Debug -ccs.buildType full
```

These are the CCS 12.8.1 application names, the version CI uses; 12.7.1 calls
them `importProject` and `buildProject`. The image is
`<configuration>/LogAndStream_Shimmer3.txt`. Build a scratch copy of the
project, so `Debug/` and `Release/` are not written into a checkout. From
Windows PowerShell 5.1, run these through `cmd /c`: it splits arguments such as
`-ccs.location` at the dot, and `eclipsec` then prints only its usage text.

### 4.2 Shimmer3 ships from the *Release* configuration

> **Releases are produced from the Release CCS configuration**, at `-O2
> --opt_for_speed=2`. The release workflow's `build_mode` input defaults to
> `Release`, the workflow refuses a candidate built from anything else, and
> `promote-release.yml` promotes nothing else. Releases up to and including
> `LogAndStream_Shimmer3_v1.01.006` were built from Debug, at `-O0`.

`Release` has the same compiler and linker settings as `Debug` apart from the
optimisation level:

| Configuration | Optimisation | Ships |
|---|---|---|
| `Debug` | `-O0 --opt_for_speed=0` | no: up to v1.01.006 only |
| `Release` | `-O2 --opt_for_speed=2` | yes |

In both, `main.c` overrides only the speed trade-off, to `--opt_for_speed=5`,
so it takes its configuration's `-O` level. A headless Debug build at
`LogAndStream_Shimmer3_v1.01.006` is byte-identical to that release's `.txt`.
Before it was brought in line with Debug, the Release configuration was a
stale template that did not build:

- no include paths
- no `SHIMMER3` define
- minimal printf
- no code/data model options
- a 160-byte stack and heap, which never boots
- `libc.a` before `libmath.a`

**Why `-O2` at speed 2.** The choice was between speeds 2 and 3, and a bench
on 19 Oct 2026 found no reason to pay for 3:

- **Throughput.** At 1024 Hz with five sensors over an RN42 v6.15 link, which
  the data outruns, both delivered what v1.01.006 does: about 19,600 packets
  per 20 s. The same source at `-O0` delivered 14% fewer. That loss is a timing
  effect, not a cost: a few cycles in the I2C wait loop moved it.
- **Code.** Speed 3 spends about 4 KB more flash, inlining command handling and
  SD configuration code. The sampling and Bluetooth paths are essentially the
  same at both.
- **Stack.** The worst case is the same at both (§4.5).

**The first Release-built candidate needs the full bench procedure** before it
is promoted. No Release image has been through it yet, and optimisation makes
the sample-timer re-arm and task-loop lost wake-up races likelier, so that
bench is the first that shows the two fixes holding under `-O2`.

Build `Release` to produce a release or to judge whether a change is safe.
Debug stays useful for stepping through code in the debugger, and a failure
that appears only in Release means something depends on optimisation. When a
change touches interrupt handlers, the Bluetooth data path or the I2C wait,
bench Release specifically: optimisation moves exactly that timing.

**Keep the two configurations in step.** Make project-setting changes under
*[All configurations]* in CCS, so Release cannot drift from Debug again. The
optimisation level is the only intended difference.

**CI builds both.** `shimmer3-firmware`'s `build-check.yml` builds Debug and
Release on every pull request and every push to `main`, one matrix leg each.
A setting that breaks only Release now fails in review. Before, nothing built
Release, so it went stale unnoticed. A green Release leg shows only that
Release compiles and links. It is not a substitute for the bench.

### 4.3 Do not commit regenerated makefiles

A headless build can rewrite the generated `Release/*.mk` files even when you
built `Debug`. Those rewrites are noise: check `git status` after a headless
build and revert any generated makefile changes you did not intend.

### 4.4 Programming

The MSP430 is programmed over its JTAG/Spy-Bi-Wire header from CCS, or through
the bootstrap loader. There is no over-the-air path.

### 4.5 Stack size and printf support

Both configurations link with `--stack_size=2200` and compile with
`--printf_support=nofloat`.

**The stack size comes from a measured worst case.** Figures are for the `-O0`
image:

| Path | Bytes |
|---|---|
| Deepest from `main`: the factory test's SD check, `ShimSd_test1()` → FatFs → SD driver | 1,070 |
| Deepest interrupt: `TIMER1_A0_ISR`, the millisecond clock's callbacks → RN4x bring-up → `sprintf` | 450 |
| Next deepest interrupt: `DMA_ISR`, Bluetooth RX → RN4x command response → `sprintf` | 446 |
| `main` plus one interrupt | 1,520 |
| `--stack_size` (680 B of margin) | 2,200 |

**One interrupt is counted because handlers do not nest.** The MSP430 clears
GIE on entry to a handler, and nothing a handler can reach sets it again.
Starting the Bluetooth module is what used to: it starts the millisecond clock,
and `msp430_clock_enable()` set GIE. Two changes keep that out of interrupt
context:

- **Bluetooth start runs from the main loop.** SD sync's start callback only
  queues `TASK_BT_TURN_ON_AFTER_BOOT`, and the disconnect handler queues
  `TASK_BT_MODE_CHECK` instead of re-checking the radio itself.
- **`msp430_clock_enable()` leaves GIE as its caller had it.** The RN4x
  set-up still calls it from `DMA_ISR` and `TIMER1_A0_ISR`, at the baud-change
  step of `runSetCommands()`. That set-up is a state machine driven by those
  two interrupts, and the main loop already runs with GIE set.

Handlers do reach one GIE write: the `__set_interrupt_state()` in the Bluetooth
TX ring's push. It restores the state saved a few lines earlier, so inside a
handler it leaves GIE clear.

**The size could now come down.** It was set while two nested handlers had to
fit on top of `main`: 1,070 B, plus 462 B for `DMA_ISR`, plus 450 B for
`TIMER1_A0_ISR`, makes 1,982 B. `DMA_ISR`'s deepest path then ran through SD
sync's Bluetooth start, which now runs from the main loop, so it is 446 B in
the table above. A build without the two changes above still needs that
1,982 B. The size stays at 2,200 B until the run-time high-water mark below has
been read on units and the image has been re-measured at the optimisation level
that ships.

**At `-O2` the bound is about the same.** For Release, `-O2
--opt_for_speed=2`, the same three paths measure 1,080, 452 and 448 B. `main`
plus one interrupt is 1,532 B, which leaves 668 B of margin. Speed 3 gives the
same figures. The functions that can turn GIE on are the same nine as at
`-O0`, so the one-handler bound holds there too (step 6 below).

A frame that holds a FatFs `FIL` costs over 500 bytes, because each `FIL`
carries a sector buffer. The `sdlog.cfg` parser keeps its `FIL` and its
configuration backup at file scope for this reason. A local `FIL` or a 512-byte
configuration copy in a new function is the quickest way to undo this sizing.

**Re-measuring.** Do it after anything that could move the figures: a new local
buffer, a new call made from an interrupt, or a different optimisation level.

1. Run `ofd430 --call_graph -o cg.xml LogAndStream_Shimmer3.out`.
2. Walk the graph from `main` and from every interrupt handler, adding each
   function's `stack_size`, which already includes the 4-byte return address.
3. Resolve indirect calls by hand. The graph records only that a function makes
   one. Apart from the C library's own (printf's output callbacks, and
   start-up initialisation before `main`), the targets are whatever is
   registered through the following:
   - `DMA0_transferDoneFunction()` and `DMA2_transferDoneFunction()`
   - `UCA0_isrRegister()` and `UART_init()`
   - `msp430_register_timer_cb()` and `BT_startDone_cb()`
   - `ShimSdSync_init()`
   - the `ShimBt_writeToTxBufAndSend` pointer
4. Follow `__TI_printfi_nofloat` wherever the graph shows `__TI_printfi`. The
   linker resolves the call there, but the graph lists `__TI_printfi` as an
   empty leaf.
5. Count `send_cmd()` in the SD driver twice. It calls itself once, because an
   application command (ACMD) is preceded by CMD55.
6. Check that no handler can reach an instruction that turns interrupts on.
   Build with `--keep_asm` and look for `EINT`, and for `BIS` or `MOV` of an
   immediate into `SR` with bit 3 (GIE) set. A write of a saved register value
   restores the caller's state and is fine. If a handler can reach one of the
   others, handlers can nest, and `main` plus one handler is no longer the
   bound.

The ceiling is the RAM the map shows unused below `.stack`. When the size was
set, that was 2,456 B, leaving 256 B unallocated.

**The stack's high-water mark at run time.** `main()` fills the stack with
`0xA5` at boot. The factory test's MCU section reports the deepest point
reached between boot and the test starting, as
`- Stack high-water = <used> of 2200 bytes`. It appends `(FULL)` if the fill
pattern is gone from the very bottom of the stack, which means the stack may
have overflowed.

**`nofloat` printf.** No format string in the firmware has a floating-point
conversion. A value with a fraction is printed as integers instead, as
`ShimSdCfgFile_generate()` does for the sampling rate. Dropping the float
formatter took 234 B off each of the four deep interrupt paths, which all end in
`sprintf` or `snprintf`, and about 4.2 KB off the image.

The compiler enforces this rule: under `nofloat`, a literal `%f`, `%e`, `%g` or
`%a` fails the build with error #2631. `minimal` support is not an option,
because it also drops field widths, which `%02X` and `%03d` depend on.

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
| `TEST_LOG_WITH_USB` | **Bench builds only, never ship.** Shimmer3R: allows SD logging with USB-C in but not docked, so a battery-less unit can log and be read back without undocking. Starting a log takes the card from USB MSC; it goes back when sensing stops, so with a stream still running after the log, only once that stream ends too. Default 0 |
| `IS_SUPPORTED_TCXO`, `IS_SUPPORTED_SINGLE_TOUCH` | Capability gates enforced in configuration validation |

> **`TEST_*` symbols are not test-only.** `TEST_TASK_MONITOR` gates the
> stuck-task watchdog, so a build without it has no protection at all, and
> `TEST_RTC_ERR_FLASH_OFF` removes a user-visible warning. Check what a shipping
> build actually defines rather than assuming the names mean development-only.
> `TEST_LOG_WITH_USB` is the exception: it is genuinely bench-only, and a
> shipping build must leave it at 0.

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
  This repository's workflows (`clang-format-check.yml`, `host-tests.yml`,
  `docs-in-step.yml`) build neither platform.
  - Each platform repository's `build-check.yml` builds only that platform, on
    every pull request and push to `main`. Shimmer3R builds its Release
    configuration, and Shimmer3 builds both Debug and Release.
  - So a `common` change is compile-checked against a platform only by that
    platform's pull request bumping the pin.
  - Each platform's `build-release-firmware.yml` is `workflow_dispatch` only
    (manual: pick the version part to bump and the build mode). Shimmer3R
    builds via `xanderhendriks/action-build-stm32cubeide`, Shimmer3 via a
    cached CCS install under `/opt/ti/ccs`.
- ~~The Shimmer3 Release configuration failure~~ — resolved. Release
  lacked the include paths, then the `SHIMMER3` define, and had several other
  stale settings. It now matches Debug apart from its optimisation level
  (§4.2).
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
  `.cproject` files: Shimmer3 (`Release`, the configuration that ships, and
  `Debug` alike) `__MSP430F5437A__ SHIMMER3`; Shimmer3R `SHIMMER3R USE_HAL_DRIVER
  UX_INCLUDE_USER_DEFINE_FILE DEBUG` plus the CubeMX device symbol. Everything
  else in §7 is at its header default in both.
