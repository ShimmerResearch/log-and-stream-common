# Shimmer3 / Shimmer3R Firmware Test Procedure

What has to pass before a LogAndStream firmware build is released, who runs it,
and — as importantly — what each layer of testing cannot tell you.

> **Verified against** — the revisions these claims were read from. A pinned
> commit is a citation, not a claim of currency.
>
> - **Firmware:** `log-and-stream-common` @ `03db4a2` —
>   `Test/host/` in full, `.github/workflows/host-tests.yml`,
>   `.github/workflows/clang-format-check.yml`,
>   `.github/workflows/docs-in-step.yml`,
>   `Extras/python_scripts/` (the on-device suites).
> - **Platform firmware:** `shimmer3-firmware` @ `eb6c400`;
>   `shimmer3r-firmware` @ `4dc5253` — the release workflows and the
>   Shimmer3R CubeMX guard check.

> **How to read this document.** **S3** = Shimmer3 (MSP430F5437A); **S3R** =
> Shimmer3R (STM32U5A5VJTXQ). LogAndStream only — BtStream and SDLog are
> deprecated and are not covered.

**Source references:**

| Layer | Where it lives |
|---|---|
| Host tests | `Test/host/` (this repository), `make -C Test/host` |
| Host-test CI | `.github/workflows/host-tests.yml` |
| Formatting gate | `clang-format-check.yml`, all three repositories |
| Documentation gate | `.github/workflows/docs-in-step.yml` |
| CubeMX guard (S3R) | `shimmer3r-firmware` `scripts/check_cubemx_guards.sh` |
| On-device suites | `Extras/python_scripts/` |
| Factory self-test | [SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md) |
| Release mechanics | [SHIMMER3_RELEASE_AND_VERSIONING.md](SHIMMER3_RELEASE_AND_VERSIONING.md) |
| Build setup | [SHIMMER3_BUILD_AND_PROGRAMMING.md](SHIMMER3_BUILD_AND_PROGRAMMING.md) |

---

## 1. The shape of the problem

Three repositories, two MCUs, one shared library:

```
shimmer3-firmware  ──┐                        MSP430F5437A, Code Composer
                     ├── log-and-stream-common
shimmer3r-firmware ──┘                        STM32U5A5VJTXQ, STM32CubeIDE
```

Two things follow, and they shape everything below.

**Every change to the shared library ships to both platforms.** A change made
while thinking about one MCU lands on the other, and nothing in the shared
repository says so at edit time. This is the single largest source of
release-blocking surprises, so the first gate in §2 is a both-platform compile
and several host suites are built twice.

**The two platforms version and release independently.** A Shimmer3 release and
a Shimmer3R release are separate events with separate sign-offs, even when they
carry the same submodule commit. §7 is per platform.

### 1.1 The layers, and what each one is actually good for

| # | Layer | Runs | Catches | Blind to |
|---|---|---|---|---|
| 1 | Formatting and docs gates | Every push | Style drift; a behaviour change with no doc update | Everything about behaviour |
| 2 | Host tests | Every push, ~10 s | Logic: arithmetic, state machines, revision gates, protocol framing, hysteresis | Word sizes, peripherals, timing, power, radio |
| 3 | Platform builds | Per release, by hand | What only the real toolchains see — **MSP430 16-bit `int`**, section overflow, linker limits | Anything not exercised at runtime |
| 4 | On-device automated | Per release, per platform | The BT and dock protocols, SD transfer, end to end against real silicon | Long-run behaviour, power, RF range, multi-device |
| 5 | Manual bench | Per release, per platform | Everything the above cannot reach — and the reason for §6 | Whatever nobody thought to try |

**No layer above replaces the one below it, and layer 2 is the one most often
mistaken for layer 3.** Host tests compile with a 32-bit `int`. The MSP430's is
16 bits. An expression that silently overflows on Shimmer3 is correct on the
host, every time. Only §3 catches that class of fault.

---

## 2. Gate 1 — automated, every push

All three repositories run these. Nothing here needs hardware, a vendor IDE or a
person, and nothing below should be started until they are green.

### 2.1 Formatting

`clang-format-check.yml`, in all three repositories, runs
`DoozyX/clang-format-lint-action` with **`inplace: True`** and then commits the
result back with `EndBug/add-and-commit`. So it does not fail a badly formatted
push — it **reformats it and pushes a "Committing clang-format changes" commit
on top**.

Two consequences worth knowing before a release:

- A formatting commit can appear on your branch after you pushed, so **fetch
  before you build the commit you intend to release**. The tag must point at the
  formatted commit, not the one you pushed.
- Formatting is never a release blocker, but it is also never enforced at review
  time — the diff you reviewed may not be the diff that shipped.

Format locally to avoid both:

- Shimmer3 / Shimmer3R: `Extras/clang-format-all-win64/LogAndStream-Shimmer3*.bat`
- Shimmer3R IDE profile: `STM32CubeIDE_Format_Profile.xml` at the repo root
- CI pins **clang-format 17**; a different local version can produce a different
  result and so a surprise commit

> The `AGENTS.md` in all three repositories currently states that this workflow
> "checks rather than reformatting, so a badly formatted push fails CI instead
> of being silently fixed". That is not what the workflow does at the pinned
> revisions. Either the text or the workflow needs to change — this document
> describes the workflow as it is.

### 2.2 Host tests

```sh
make -C Test/host              # build and run everything
make -C Test/host platform-check   # compile every host-clean module for BOTH MCUs
make -C Test/host test_rtc     # one suite, both platform builds
```

CI runs this same Makefile, so a suite that passes locally passes there.

**`platform-check` runs first, and is the most valuable ten seconds in the
pipeline.** It compiles every host-clean module twice, once with `-DSHIMMER3`
and once with `-DSHIMMER3R`, and only compiles — nothing runs. It catches the
cheap and common half of cross-platform breakage: a symbol used outside the
`#if` that defines it, an `#ifdef` arm that stopped compiling, a header that
only resolves on one platform. Those are most of what actually breaks between
the two, and this puts them in a pull request rather than in a release build.

What the suites cover today:

| Suite | Module | Builds | What it pins |
|---|---|---|---|
| `test_swcrc` | `CRC/shimmer_swCrc.c` | ×1 | The CRC polynomial, against wire-format vectors |
| `test_crc` | `CRC/shimmer_crc.c` | ×1 | Mode dispatch, CRC placement, corruption detection, and the firmware's own `testCrcDriver()` |
| `test_packet_ring` | `Sensing/shimmer_packet_ring.c` | ×1 | The sample ring's interleavings, including the DEV-1023 field fault |
| `test_util` | `Util/shimmer_util.c` | ×1 | SD directory naming, config-file 64-bit values, BT response lengths, MAC byte order |
| `test_rtc` | `RTC/shimmer_rtc.c` | ×2 | Every day from 2000 to 2099 round-trips; leap years; date validation |
| `test_battery` | `Battery/shimmer_battery.c` | ×2 | Charge-band hysteresis, charger states, LED colours, low-battery auto-stop |
| `test_boards` | `Boards/shimmer_boards.c` | ×2 | Every revision gate, across 25 board revisions |

Three of them are additionally cross-checked against a reference that shares no
code with the firmware — an oracle, not a restatement:

| Cross-check | Oracle | Why it is worth having |
|---|---|---|
| `crosscheck_swcrc.py` | The Python host CRC in `Extras/python_scripts/` | If firmware and host tooling disagree, a host silently fails to verify CRCs a device produced |
| `crosscheck_rtc.py` | Python's `datetime` | A conversion pair can be each other's exact inverse and both be a day out. A round-trip cannot see that; an independent calendar can |
| `crosscheck_board_revisions.py` | The gate table in [SHIMMER3_BOARD_REVISIONS.md](SHIMMER3_BOARD_REVISIONS.md) | Reports drift between the document and the firmware — see §2.4 |

### 2.3 Documentation

`docs-in-step.yml` is **advisory** and never fails a build. It writes a note on
the PR summary when a documented subsystem changed and nothing under `docs/`
did. Silence it with the `docs-not-needed` label when a change genuinely has no
observable behaviour — a rename, a refactor.

Take the note seriously anyway. `AGENTS.md` puts it plainly: a stale doc here is
worse than no doc, because it gets believed, and believed on two platforms at
once.

### 2.4 When the board-revision cross-check fails

It does not fail the way other tests fail, and it must not be treated the same
way.

`AGENTS.md` says the per-product tables in `SHIMMER3_BOARD_REVISIONS.md` are a
conversion of an internal hardware workbook that is **not in this repository**,
so nobody outside Shimmer can check them — and that **where the tables and the
firmware disagree, the firmware wins.**

So a disagreement means one of two different things, needing two different
fixes:

- **The document is stale** — hardware changed, the gate was updated, the table
  was not. Fix the table.
- **The firmware has a bug** — the gate does not implement what the hardware
  actually does. Fix the gate, and expect it to be a release blocker: it means
  some revision of some board is talking to a part that is not fitted.

**Do not reconcile it by editing the table to match the code.** That converts a
real question into a silent wrong answer. Work out which side is stale, fix
that one, and say which in the PR.

### 2.5 CubeMX guards — Shimmer3R only

`scripts/check_cubemx_guards.sh` runs on every push and fails the build if a
CubeMX regeneration has eaten hand-written code in a generated region. It is a
backstop for damage that has already happened once, not a substitute for reading
the diff (§3.3).

---

## 3. Gate 2 — the platform builds

**Required for every release, both platforms, by hand.** Neither vendor toolchain
runs in CI, and this is the only gate that sees the real word sizes.

| | Shimmer3 | Shimmer3R |
|---|---|---|
| IDE | Code Composer Studio **12.8.1.00005** | STM32CubeIDE **1.15.1** |
| Compiler | TI MSP430 **v21.6.1.LTS** | arm-none-eabi (bundled) |
| `int` width | **16 bits** | 32 bits |
| Ships from | **Debug** (§3.1) | **Release** |

Version pinning is deliberate. Check the README table before upgrading either.

### 3.1 Build the configuration that actually ships

**They are not the same on the two platforms**, and getting this wrong means
signing off a configuration nobody releases.

| Platform | Build | Note |
|---|---|---|
| Shimmer3 | **Debug** | The Release configuration does **not** build. Known and parked — not a regression to chase. See [SHIMMER3_BUILD_AND_PROGRAMMING.md](SHIMMER3_BUILD_AND_PROGRAMMING.md) §4.2 |
| Shimmer3R | **Release**, and Debug as well | A Debug-only failure means something depends on optimisation, which is worth knowing before a customer finds it |

For Shimmer3, `Debug` is the configuration that ships, so it is the one to
evaluate a change against. The release workflow takes a build-mode input for
both platforms; pick the one that matches the table.

> A headless Shimmer3 build can rewrite the generated `Release/*.mk` files even
> when you built `Debug`. Check `git status` afterwards and revert generated
> makefile changes you did not intend.

### 3.2 Read the warnings, on Shimmer3 especially

The MSP430's 16-bit `int` is the trap the host tests cannot spring. The pattern
to look for is an arithmetic expression whose operands are all narrow but whose
*result* is not:

```c
/* uint8_t hours, and 3600 fits an int - so on MSP430 this is 16-bit
 * arithmetic, and it overflows from 10:00 onwards. */
seconds += data->hours * RTC_SECONDS_PER_HOUR;      /* wrong on S3 */
seconds += (uint32_t) data->hours * RTC_SECONDS_PER_HOUR;  /* right */
```

Widen one operand with a cast. A constant that does not fit 16 bits (`32768`,
`86400`) is already `long` and promotes the expression on its own — it is the
ones that *do* fit (`3600`, `1000`) that bite.

**Review rule:** in shared code, any multiplication whose mathematical result
can exceed 32767 needs an explicitly widened operand, even when both operands
are narrow. The host tests will not tell you.

### 3.3 Shimmer3R — after any CubeMX regeneration

Generation rewrites everything outside `USER CODE` blocks, and this project
keeps a lot of hand-written code in CubeMX-owned regions.

1. **Always generate on a branch, then read the whole diff.** Not
   `git diff --stat` — the real diff.
2. Restore what you did not ask for. `AGENTS.md` lists the files known to carry
   hand-written code that generation removes, and the three things in the diff
   that are **not** yours to keep.
3. If you added hand-written code to a CubeMX-owned region, **add a guard for it
   in the same commit** — `check_cubemx_guards.sh` only knows about damage that
   has already happened once.
4. Rebuild in the IDE, not from the command line: `Release/` and `Debug/` hold
   stale generated makefiles referencing files you removed.

### 3.4 Check the size

Record flash and RAM for both configurations and compare against the previous
release. A jump nobody can explain is a finding, not a curiosity — this is a
`MSP430F5437A` with 256 KB flash and 16 KB RAM, and the margin is not large.

---

## 4. Gate 3 — on-device automated

**Required for every release, on at least one unit per platform.** These drive a
real device over a real link. They are `unittest` suites, so they run
unattended and report properly.

| Suite | Link | Covers |
|---|---|---|
| `Extras/python_scripts/Bluetooth commands/test_bt_cmds.py` | Bluetooth | Every get/set command pair, response framing, length bytes |
| `Extras/python_scripts/Docked commands/test_docked_cmds.py` | Dock UART / USB-C | The dock protocol, which is a different protocol and not a subset |
| `Extras/python_scripts/Bluetooth commands/test_sd_transfer.py` | Bluetooth | SD file transfer (Shimmer3R) |
| `Extras/python_scripts/Bluetooth commands/btCalv1Test.py` | Bluetooth | Calibration read-back |

Run them against a **factory-reset configuration first**, then against the
configuration a customer is most likely to have. A device only ever tested from
a known-good config will not tell you that a config path is broken.

### 4.1 The factory self-test — Shimmer3R

Trigger the on-device self-test and read the report. Every fitted part answers
on its bus; `S3R_TEST_nnnn - PASS|FAIL|WARNING` per line, with a bitmask summary.
See [SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md) for the
registry and the report format.

**Check it against the board revision.** A `FAIL` for a part that revision does
not carry is a revision-gate problem (§2.4), not a hardware fault — and a `PASS`
for a part that is not fitted is worse.

---

## 5. Gate 4 — manual bench

**Required for every release, per platform.** What follows is the minimum. It is
ordered so that a failure stops you early.

Record, for each run: firmware version, board SR number, host tool and version.

### 5.1 Boot and identity

| # | Step | Expected |
|---|---|---|
| 5.1.1 | Power on undocked | Boots to idle; LED matches charge band ([SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md)) |
| 5.1.2 | Read firmware version over BT | Matches the release, zero-padded (`v1.01.012`) |
| 5.1.3 | Read the daughter-card ID | Correct SR number and human-readable name |
| 5.1.4 | Power on with no SD card | Boots; reports no card; does not hang |
| 5.1.5 | Power on with an unformatted card | Boots; reports bad card; does not hang |

### 5.2 Configuration

| # | Step | Expected |
|---|---|---|
| 5.2.1 | Write a full config over BT, power-cycle, read back | Byte-identical |
| 5.2.2 | Write via dock, read over BT | Identical — one config, two routes |
| 5.2.3 | Set the real-world clock, power-cycle | Time correct, no RTC error flash |
| 5.2.4 | Boot with the clock never set | RTC error flash, if enabled in config |
| 5.2.5 | Write `sdlog.cfg` by hand, boot | Parsed; a malformed key is rejected without taking the rest of the file with it |

### 5.3 Logging and streaming

| # | Step | Expected |
|---|---|---|
| 5.3.1 | Log 10 min, all channels, max rate | No gaps; timestamps monotonic |
| 5.3.2 | Stream 10 min, all channels, max rate | No dropped packets |
| 5.3.3 | **Log and stream simultaneously, max rate** | Both intact — this is the headline feature and the hardest case |
| 5.3.4 | Log across a file-split boundary | Files continuous, no lost samples at the join |
| 5.3.5 | Pull the card mid-log | Clean stop; the file up to that point is readable |
| 5.3.6 | Start/stop 20 times | No leak, no drift, no stuck state |

Check 5.3.1–5.3.3 in Consensys, not just for file size. **A packet with a zero
timestamp reads as a 24-bit roll-over worth 512 s** — that is the DEV-1023
signature, and `test_packet_ring` covers the mechanism, but only a real
long-run recording covers the whole path.

### 5.4 Battery and charging

`test_battery` covers the classification logic exhaustively; what it cannot do
is confirm the ADC actually reads what the cell is doing.

| # | Step | Expected |
|---|---|---|
| 5.4.1 | Dock a part-charged unit | Steady red, then green at full |
| 5.4.2 | Undock at each charge band | LED colour matches the band |
| 5.4.3 | Run to the auto-stop threshold with the option enabled | Logging stops; the card is readable |
| 5.4.4 | The same with it disabled | Logging continues |

### 5.5 Bluetooth

| # | Step | Expected |
|---|---|---|
| 5.5.1 | Pair, connect, disconnect, reconnect ×5 | Reliable, no reset needed |
| 5.5.2 | Walk to the edge of range while streaming | Degrades and recovers; no lockup |
| 5.5.3 | Power off the host mid-stream | Device returns to idle by itself |
| 5.5.4 | BLE, where supported (S3 RN4678, S3R CYW20820) | Connects and streams |

### 5.6 Multi-device, if SD sync is in the release

| # | Step | Expected |
|---|---|---|
| 5.6.1 | Three units, one centre, 30 min | All files carry usable offsets |
| 5.6.2 | Power-cycle a node mid-session | Rejoins; the gap is visible in the data, not silent |

### 5.7 Power

| # | Step | Expected |
|---|---|---|
| 5.7.1 | Sleep current, undocked, idle | Within spec for the board |
| 5.7.2 | Overnight idle undocked | Still responsive; battery drop as expected |

---

## 6. Regression cases

Faults that reached a customer, or nearly did. **Every one of these is on the
list because it got past the testing of its day**, which is the only reason a
case earns a permanent place here.

| Case | Symptom | Covered by |
|---|---|---|
| DEV-1023 | Zero timestamp in a logged packet, read as a 512 s jump | `test_packet_ring` — plus 5.3.1 |
| DEV-1019 | Unprogrammed EEPROM reported as a real board, SR0-0-0 | `test_boards` |
| DEV-1003 | CubeMX regeneration deleted 165 lines of `main.c` | `check_cubemx_guards.sh` — plus §3.3 |
| DEV-1026 | I2C bus completion flags stale between gathers | 5.3.1, all channels |
| DEV-866 | A board with a dead LSE hangs at boot | 5.1.1 on an affected unit |
| DEV-818 | BMP581 / BMP390 fitted per revision | `test_boards`, `crosscheck_board_revisions.py` |
| BMP581 SR48 window | A plain `>= 7.2` wrongly claims SR48-8-0 and 8-1 | `test_boards` — both layers catch it |

**When a fault escapes, add its case here and a test for it in the same PR.** A
regression list that only grows by hand stops growing.

---

## 7. Release sign-off

Per platform. Release mechanics — versioning, tags, the workflow inputs — are in
[SHIMMER3_RELEASE_AND_VERSIONING.md](SHIMMER3_RELEASE_AND_VERSIONING.md); this
is what has to be true before you trigger it.

- [ ] Gate 1 green on the exact commit being released (§2)
- [ ] `make -C Test/host` green locally as well as in CI
- [ ] The shipping configuration builds clean — Shimmer3 **Debug**, Shimmer3R **Release** (§3.1)
- [ ] Shimmer3 build warnings read, with §3.2 in mind
- [ ] Flash and RAM recorded and compared with the previous release (§3.4)
- [ ] Submodule pointer is at the intended `log-and-stream-common` commit
- [ ] **The other platform still builds** against that submodule commit (§1)
- [ ] On-device suites pass (§4)
- [ ] Factory self-test read and checked against the board revision (§4.1)
- [ ] §5 walked on at least one unit, results recorded
- [ ] §6 regression cases considered against what changed
- [ ] `version.h` — all four values changed together, string zero-padded
- [ ] `FirmwareIdentifierList.txt` in step, if a build was added
- [ ] Documentation updated for every behaviour change in the release
- [ ] Release notes name what changed and what was tested

Then trigger `build-release-firmware.yml` by **workflow_dispatch**. The push
trigger is commented out on purpose, so releases are never accidental.

### 7.1 A shared-library change spans two releases

A change to `log-and-stream-common` is not released. It ships when a platform
firmware bumps its submodule pointer and releases — so a shared change needs
**both** platforms taken through §3 onwards before it can be called released,
even if only one of them ships first.

---

## 8. Extending the host tests

This is the cheapest testing you have, and the list in §2.2 is short because of
what is reachable, not because of what is worth covering.

### 8.1 What makes a module reachable

`log_and_stream_externs.h` declares what each platform firmware must implement.
Shimmer3 implements it against the MSP430 HAL, Shimmer3R against the STM32 HAL,
and `Test/host/stubs/` against nothing at all — a third platform, for a PC.

**A module is host-testable exactly to the degree that it reaches the platform
through that contract.** Anything needing a new stub is telling you it reached
around the abstraction, which is worth knowing on its own.

`Test/host/stubs/README.md` has the mechanics. The short version: nothing there
redefines a type, struct or constant from this repository — only
`log_and_stream_includes.h` is shadowed, and only to cut the subsystems whose
headers need firmware-side files. Struct layouts a test sees are the firmware's
own, which is what stops a green test from meaning nothing.

### 8.2 The next candidates, roughly by value

| Module | What a test would pin | What it needs first |
|---|---|---|
| `SDCard/shimmer_sd_cfg_file.c` | `sdlog.cfg` parse and generate, round-trip, malformed input | A `ff.h` stub |
| `Configuration/shimmer_config.c` | The 512-byte InfoMem image: validation, defaults, clamping | `ADXL371/`, `LSM6DSV/` header stubs |
| `TaskList/shimmer_taskList.c` | Task priority and the set/clear/get-list bitmask | The firmware-side `shimmer_definitions.h` |
| `Comms/shimmer_bt_uart.c` | Command framing and the parser, against the protocol document | An `RN4678.h` stub; it is a large module |
| `GSR/gsr.c` | Auto-range switching and its settling behaviour | Detangling from the HAL headers |
| `SDCard/shimmer_sd_header.c` | The binary file header, against the SD format document | `ff.h`, `fx_api.h` stubs |
| `EEPROM/shimmer_eeprom.c` | The memory map and the branding record | The `BAUD_*` enum out of `Comms/` |

Each stub added brings its whole subsystem within reach, so the order above is
roughly the order of return.

### 8.3 Adding a suite

1. Write `Test/host/test_<module>.c`, guarded with
   `#if defined(SHIMMER_HOST_TEST)` — **the guard is not optional.** Both
   firmware projects compile every `.c` under this repository, and the file
   defines `main()`.
2. Include `host_test.h` for the assertion helpers.
3. Add it to `SRC_test_<module>` and one of the suite lists in
   `Test/host/Makefile`. CI picks it up with no workflow change.
4. If it should be built for both platforms, put it in
   `DUAL_PLATFORM_SUITES`. Prefer that where the module has any `#if
   defined(SHIMMER3...)` in it at all.

### 8.4 What a good case here looks like

The suites in §2.2 follow four habits worth keeping:

- **Test against the real caller.** `test_util` covers `ShimUtil_ItoaWith0` at
  three digits because that is what `ShimSdDataFile_makeBasedir()` passes, not
  because three is a round number.
- **Pin behaviour you do not endorse, and say so.** Several cases are marked
  *PINNED, NOT ENDORSED* — the SD directory counter wrapping at 1000, the
  cumulative low-battery count. They document a limit so that changing it is a
  decision rather than an accident.
- **Prefer an exhaustive sweep to a handful of examples** where it is cheap.
  `test_rtc` converts every day from 2000 to 2099 in a few milliseconds, which
  covers the leap-year rule and all twelve month lengths without anyone listing
  them.
- **Reach for an independent oracle** where one exists. A round-trip proves
  self-consistency; Python's `datetime` proves correctness.

---

## Still unverified / not found in code

- **Sleep-current figures (§5.7.1).** The acceptance limits are per board and
  live in the hardware documentation, not in this repository. The step is listed
  without a number deliberately.
- **Bluetooth range expectations (§5.5.2).** No documented pass criterion; the
  step checks graceful degradation and recovery, not a distance.
- **SD sync accuracy (§5.6).** [SHIMMER3_SD_SYNC.md](SHIMMER3_SD_SYNC.md)
  describes the mechanism and how to apply the offsets, but no accuracy budget
  is stated anywhere in the firmware or docs, so §5.6.1 asks only that the
  offsets be usable.
- **Which on-device suites are run per release today.** §4 lists what exists in
  `Extras/python_scripts/`. Whether all four are currently part of a release
  run, and on how many units, is a process question this repository does not
  record.
- **Shimmer3 factory self-test.** §4.1 covers the Shimmer3R report, which is
  what [SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md)
  documents. `hal_FactoryTest.h` exists on both platforms; the Shimmer3 report
  format is not documented here.
