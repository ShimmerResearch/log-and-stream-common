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
| 5 | Functional, on hardware | Per release, per platform | Firmware behaviour that is per-model, per-radio or per-revision: packet layout, scaling, driver selection, radio bring-up | Hardware faults (that is §4.1's job), and whatever nobody thought to try |
| 6 | Host compatibility | Per release, both platforms | Whether the firmware still works with the host software customers have: Consensys, the Java driver, the web SDK | Nothing a host does not touch |

**Layer 6 is the one with the widest blast radius.** Layers 1-5 all ask whether
the firmware is correct. Layer 6 asks whether it is still *compatible*, which is
a different question with a different answer: a change can be entirely correct
and still break every customer's analysis, because the firmware carries no
compatibility logic and every gate is host-side (§6.1).

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

That is deliberate and it does save time — nobody applies formatting review
comments by hand. Three consequences worth knowing before a release:

- **A formatting commit appears on your branch after you pushed.** Pull before
  your next push, and **fetch before you build the commit you intend to
  release** — the tag must point at the formatted commit, not the one you pushed.
- **That commit gets no CI run of its own.** GitHub does not trigger workflows
  for pushes made with `GITHUB_TOKEN`, so the checks that passed ran on the
  *pre-format* tree. Harmless for whitespace; worth knowing it is not the tree
  the checks saw.
- **Fork pull requests are not covered.** All three workflows are `on: [push]`,
  with the `pull_request` block commented out, so an external contribution is
  never formatted.

It fires often — roughly **15% of commits in `log-and-stream-common`** are
auto-format commits, and they consistently touch a subset of the files the
preceding commit touched. That is the formatter not being run before pushing,
not a tooling fault.

**The fix is the checked-in `pre-commit` hook.** One command per clone —
`.githooks/install.sh`, or `.githooks\install.bat` on Windows — after which the
`.c`/`.h` files staged for a commit are clang-formatted and re-staged as part of
that commit. The bot commit never appears, your branch never moves under you,
and the diff that is reviewed is the diff that ships. It needs nothing installed
on Windows: Git for Windows supplies the shell and the formatter is already in
the clone at `Extras/clang-format-all-win64/clang-format.exe`.

It is a convenience, not a gate — it never blocks a commit, `--no-verify`
bypasses it, and a fresh clone has it off until someone runs the installer.
**That is why the CI auto-fix stays**: the hook removes the round-trip for the
common case, CI catches everyone else, including the fork PRs the hook cannot
reach. `.githooks/README.md` has the rest, including why a partly staged file
(`git add -p`) is deliberately left alone.

**The exclusion list lives in one file.** `.clang-format-exclude` at each
repository root names the source directory and the paths clang-format must not
touch; the workflow and the hook read it through
`scripts/clang-format-exclude.sh`, and the `.bat` reads it directly. It replaced
three hand-maintained copies that had already drifted — the workflow and the
`.bat` disagreed about `lis303ah-pid` and about how `ezsapi` was spelled. The
conversion was checked by comparing the excluded file set before and after over
every `.c`/`.h` in each repository: identical, 449 files on Shimmer3R and 74 on
Shimmer3.

- Shimmer3 / Shimmer3R whole-project format: `Extras/clang-format-all-win64/LogAndStream-Shimmer3*.bat`
- Shimmer3R IDE profile: `STM32CubeIDE_Format_Profile.xml` at the repo root
- `.clang-format` lives in each project directory, not at the repo root

> **The version pins differ and it does not currently matter.** CI pins
> clang-format **17**; the bundled `clang-format.exe` is **18.1.8**. Reformatting
> `log-and-stream-common`'s `main` with 18 changes **0 of 60 files**, so the two
> agree on this codebase and the difference is not a cause of churn. Recorded so
> that it is not blamed for churn it does not cause, and so that a future bump on
> either side is known to need checking.

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
| `crosscheck_host_constants.py` | The protocol document **and** the Python host reference | The only automated host-compatibility check there is — see §6.3. Needs no compiler, so it also runs standalone: `make -C Test/host host-constants` |

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

## 5. Gate 4 — functional testing on hardware

**Required for every release, per platform.** Ordered so a failure stops you
early; §5.1 comes first because a unit that will not boot cannot be tested for
anything else.

> **This section tests firmware, not hardware.** Hardware qualification is a
> different activity with a different owner — the factory self-test (§4.1) finds
> a part that is missing, dead or out of tolerance, and it does so better than a
> person with a bench can.
>
> What is being asked here is narrower and harder to see: **given this hardware,
> does the firmware do the right thing?** The firmware takes a different path per
> board revision, per fitted sensor, per radio and per radio firmware, and the
> failures that ship are the ones where a path is wrong but nothing looks broken
> — a channel in the wrong slot, a value out by a constant factor, a radio the
> firmware misidentifies. None of those show up as a dead sensor, and none of
> them are caught by anything earlier in this document.
>
> So where a step below involves a physical stimulus — rotating a unit, injecting
> a square wave, presenting a known resistance — the stimulus is a means of
> knowing what the firmware *should* have emitted. The part's health is not the
> measurement.

Record, for each run: firmware version, board SR number, **radio module and its
firmware version**, host tool and version.

### 5.1 The boot matrix — board × radio × radio firmware

**This is the most combinatorial thing in the product, and on Shimmer3 it has the
harshest failure mode in the firmware.** Both platforms walk a ladder of baud
rates when the radio does not answer, and both set
`BOOT_STAGE_BLUETOOTH_FAILURE` when the ladder runs out. What happens next
differs, and the difference matters for what you are looking for:

| Platform | When the baud ladder is exhausted | What you see |
|---|---|---|
| **Shimmer3** | `while (1) { __bis_SR_register(LPM3_bits + GIE); }` — the boot never completes | **The unit is halted.** No Bluetooth, no logging, no dock response. Yellow at 5 Hz, because the blink timer still runs on ACLK |
| **Shimmer3R** | `break` — boot continues without the radio | The unit comes up and logs, but has no Bluetooth. Yellow at 5 Hz |

> `shimmer3-firmware` `LogAndStream_Shimmer3/main.c:300-321`;
> `shimmer3r-firmware` `LogAndStream_Shimmer3R/Core/Src/main.c:732-739`;
> the LED pattern is [SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) §5.
>
> The Shimmer3R behaviour is the friendlier one and the Shimmer3 behaviour is
> the one that generates support calls — but **the Shimmer3R case is easier to
> miss**, because a unit that boots and logs looks fine until someone tries to
> connect to it. Check for the LED pattern on both, not just for "it booted".

Two modules on Shimmer3, one on Shimmer3R, and **six recognised firmware
versions on the RN4678 alone**:

| Platform | Module | Firmware the firmware recognises | Notes |
|---|---|---|---|
| Shimmer3 | Microchip RN41 | v4.77 | Classic only. Fitted to units with no EEPROM |
| Shimmer3 | Microchip RN42 | v4.77, v6.15 | Classic only |
| Shimmer3 | Microchip RN42 | **v6.30** | **Deliberately refused** — `triggerShimmerErrorState()` |
| Shimmer3 | Microchip RN4678 | v1.00.5, v1.11.0, v1.13.5, v1.22.0, v1.23.0, v1.24.0 | Dual-mode, classic + BLE |
| Shimmer3R | Infineon CYW20820 (Vela IF820), EZ-Serial | per the module's own release | Dual-mode |

> Detection is `Comms/shimmer_bt_uart.c:341-463`. The RN4678 branch keys the
> version on **characters 10 and 11 of the banner only** — `00`, `11`, `13`,
> `22`, `23`, `24` — then waits for an exact further byte count chosen per
> version (`RN4678_VERSION_LEN_V1_*` in `Shimmer_Driver/RN4X/RN4X.h`). That is
> what makes this fragile in a way no amount of code reading fixes:
>
> - **Expected length too long** for the banner the module actually sends → the
>   receive never completes, the baud ladder exhausts, and the unit halts as
>   above.
> - **Expected length too short** → the surplus bytes fall through to the
>   command parser as garbage.
> - **An unrecognised module** does not hang; it proceeds with
>   `BT_FW_VER_UNKNOWN`, which then feeds every version-gated decision — baud
>   selection, BLE availability, error-LED support. The unit boots and
>   misbehaves, which is harder to spot than a unit that does not boot.

**The test.** Load the release firmware onto one unit of each row, power-cycle
and confirm it reaches idle:

| # | Step | Expected |
|---|---|---|
| 5.1.1 | Power on | Reaches idle. **Not** yellow flashing at 5 Hz |
| 5.1.2 | `GET_BT_VERSION_STR_COMMAND` | Returns the banner, and it matches the module actually fitted |
| 5.1.3 | Connect over classic Bluetooth | Connects and streams |
| 5.1.4 | Connect over BLE, where the row supports it | Connects and streams |
| 5.1.5 | Power-cycle five times | Boots every time — an intermittent bring-up is a bring-up failure |

Two things make this cheaper than it looks:

- **The radio firmware is the variable, not the board.** A given RN4678 firmware
  behaves the same across board revisions, so one unit per radio-firmware row
  covers it — you do not need the full board × radio cross product. Use whatever
  boards you have; the board axis is covered by §5.4.
- **It is the reprogramming that takes the time**, not the test. Where a rig can
  reflash the radio, this is a batch job.

> **Record the banner string, not just "passed".** It is the only ground truth
> for what was actually on the unit, and a matrix run without it cannot be
> reproduced when a customer reports a fault six months later.

If a row fails, capture: the banner string, the baud the unit settled on, and
whether the failure is at bring-up or at first connect. Those three separate the
three mechanisms above.

### 5.2 Boot and identity

| # | Step | Expected |
|---|---|---|
| 5.2.1 | Power on undocked | Boots to idle; LED matches charge band ([SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md)) |
| 5.2.2 | Read firmware version over BT | Matches the release, zero-padded (`v1.01.012`) |
| 5.2.3 | Read the daughter-card ID | Correct SR number and human-readable name |
| 5.2.4 | Power on with no SD card | Boots; reports no card; does not hang |
| 5.2.5 | Power on with an unformatted card | Boots; reports bad card; does not hang |
| 5.2.6 | Power on with no EEPROM fitted (pre-SR31-7-0) | Boots; reports the card ID as unprogrammed (`0xFF`), **not** as SR0-0-0 |

### 5.3 Configuration

| # | Step | Expected |
|---|---|---|
| 5.3.1 | Write a full config over BT, power-cycle, read back | Byte-identical |
| 5.3.2 | Write via dock, read over BT | Identical — one config, two routes |
| 5.3.3 | Set the real-world clock, power-cycle | Time correct, no RTC error flash |
| 5.3.4 | Boot with the clock never set | RTC error flash, if enabled in config |
| 5.3.5 | Write `sdlog.cfg` by hand, boot | Parsed; a malformed key is rejected without taking the rest of the file with it |

### 5.4 Per-model behaviour — what the firmware does differently

**The question here is not whether the part works. It is whether the firmware
handles this variant correctly.**

That distinction decides what is worth testing. A channel stuck at zero is a
hardware fault and the factory self-test already finds it. What ships instead is
a channel carrying *plausible* values in the wrong slot, at the wrong scale, or
in the wrong byte order — because the firmware takes a different code path per
model and one of those paths is wrong. That is invisible on the device, survives
every automated gate, and surfaces weeks later as data nobody can reconcile.

`test_boards` proves the firmware's *belief* about what is fitted matches the SR
number. It cannot prove the firmware then does the right thing with that belief.
This section does.

Four things branch per model, and each is a separate failure:

| What branches | Decided by | How it fails |
|---|---|---|
| Which driver runs | Revision gates (`ShimBrd_is*Present`) | Wrong driver for the fitted part |
| Channel order and widths in the packet | Board + platform + enabled channels | Everything after the offending channel decodes as garbage |
| Calibration defaults and scaling | Sensor in use, range, board generation | Values wrong by a constant factor |
| Config validation | Board capability | A setting silently corrected — or silently not |

The fitted set is a function of generation, not board alone — see
[SHIMMER3_BOARD_REVISIONS.md](SHIMMER3_BOARD_REVISIONS.md):

| Generation | Pressure | Gyro / LN accel | WR accel | Mag | Alt mag | Mic | Radio |
|---|---|---|---|---|---|---|---|
| First | BMP180 | MPU-9150 / KXRB5-2042 | LSM303DLHC | LSM303DLHC | MPU-9150 | — | RN42 |
| Second | BMP280 | MPU-9250 / KXTC9-2050 | LSM303AHTR | LSM303AHTR | MPU-9250 | — | RN42 |
| Third | BMP280 | ICM-20948 / KXTC9-2050 | LSM303AHTR | LSM303AHTR | ICM-20948 | — | RN4678 |
| Fourth (S3R) | BMP390 | LSM6DSV | LIS2DW12 | LIS2MDL | LIS3MDL (to `.1`) | MP23DB01HP | Vela IF820 |
| Fourth `.2`+ | **BMP581** | LSM6DSV | LIS2DW12 | LIS2MDL | — | IM68D121 (from `.3`) | Vela IF820 |

#### 5.4.1 Packet layout, decoded against the document

The single highest-value test in this section. Enable every channel the board
carries, capture one packet, and **decode it by hand against
[SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §4** for
that exact model — not with Consensys, which shares assumptions with the
firmware and will agree with it about a shared mistake.

| # | Check | Why this model matters |
|---|---|---|
| 5.4.1a | Channel order matches the documented order for this board | Shimmer3 emits temperature-then-pressure; Shimmer3R pressure-then-temperature, and **the widths differ** (§4.3) |
| 5.4.1b | Magnetometer axis order | An LSM303DLHC board emits X, **Z**, **Y** — not X, Y, Z (§4.1) |
| 5.4.1c | VBATT position | On an SR48-6-0 the MCU ADCs are configured first, so VBATT is **not** last (§4.2) |
| 5.4.1d | Total packet length | Matches the sum of the enabled channel widths — a length that is right by accident on one board is wrong on another |
| 5.4.1e | Timestamp advances by one sample period | And is never `00 00 00` |

#### 5.4.2 Scaling and calibration, per sensor in use

Values in the right slot but the wrong size. Each of these is a known trap with a
documented cause:

| # | Check | The trap |
|---|---|---|
| 5.4.2a | Uncalibrated magnetometer magnitude is plausible | LSM303AH boards are clamped to mag range 0, whose default seed carries the wrong sensitivities — reads ~5× high ([calibration](SHIMMER3_CALIBRATION.md) §6.1) |
| 5.4.2b | Calibration bias and sensitivity read back correctly | They are **big-endian**, unlike most of the config (§3.1) |
| 5.4.2c | Low-noise and wide-range accel calibrations are not swapped | The SD header order is not the InfoMem order (§4.2) |
| 5.4.2d | ExG millivolts in 16-bit mode | The 16-bit word is bits 22:7 of the 24-bit conversion — the denominator needs a factor of 2 ([streaming](SHIMMER3_STREAMING_DATA_FORMAT.md) §7.5) |
| 5.4.2e | Shimmer3R ADC / battery millivolts | 12-bit at 3.0 V; the divide-by-four belongs to the MCU's own VBAT debug channel, not these (§7.2) |
| 5.4.2f | Gyro range on Shimmer3R | The range's high bit lives in config byte 130 — getting it wrong is a 16× error ([InfoMem](SHIMMER3_CONFIGURATION_INFOMEM.md) §4.1) |

Axis identity is a firmware question here, not a hardware one: rotate the unit
through six orientations only to establish **which emitted channel is which
axis and with what sign**, then check that against the documented mapping for
the model. You are testing the firmware's channel assignment, not the part.

#### 5.4.3 Driver selection across a revision boundary

The revision gates choose a driver. The interesting units are the ones either
side of a boundary, because that is where the gate is load-bearing:

| # | Step | Expected |
|---|---|---|
| 5.4.3a | A `.1` board and a `.2` board of the same family, same firmware | The `.2` uses the **BMP581** path, the `.1` the **BMP390** path — different calibration coefficients and a different packet encoding |
| 5.4.3b | A board with LIS3MDL and one without (`.1` dropped it) | Alt-mag channels present on one, absent on the other, and the packet shortens accordingly |
| 5.4.3c | SR48-6-0 versus SR48-7-0 | MCU ADCs versus the ADS7028 — a different acquisition path entirely |
| 5.4.3d | An SR31-11-1 | Keeps the ADXL371 — the IMU-board exception to the `.1` rule |

A mismatch here is a **firmware** finding even though it presents as a sensor
problem, and per `AGENTS.md` it is the gate that is authoritative, not the
hardware workbook. Report it rather than reconciling either side (§2.4).

#### 5.4.4 Firmware-computed outputs

Nothing to do with a part being fitted — these are the firmware's own arithmetic,
and a bench unit is the only place the whole chain runs:

| # | Step | Expected |
|---|---|---|
| 5.4.4a | Enable derived channels | Computed values match the inputs they are derived from — `derivedChannels.py` |
| 5.4.4b | Sweep known resistances into GSR | Each auto-range band **entered, held and reported** correctly, including the 80 ms settling hold ([GSR](SHIMMER3_GSR_AUTORANGE.md) §5.1). The Shimmer3R tree carries a rig driver for exactly this — `Shimmer_Driver/GSRTestRig/`, an AD5242 pot and ADG715 switch bank presenting known resistances to 1 MΩ, which makes this a calibration check rather than "the number moved" |
| 5.4.4c | Configure each supported sampling rate | The packet rate is `32768 / samplingRateTicks` and matches — `samplingRate.py` |
| 5.4.4d | GSR boards with reversed control pins (SR48-4-1) | The reversal flag applied — values not out by a large constant factor ([GSR](SHIMMER3_GSR_AUTORANGE.md) §2) |
| 5.4.4e | Set an illegal channel combination | Silently corrected, and corrected the same way the InfoMem document says ([InfoMem](SHIMMER3_CONFIGURATION_INFOMEM.md) §10) |
| 5.4.4f | Enable skin temperature alongside GSR | GSR wins the shared ADC input — documented, and the config read-back should show it (§10.2) |
| 5.4.4g | Clear the expansion-power bit with GSR/PPG enabled | Those channels read as unpowered noise. No firmware rule derives this bit — confirm the behaviour rather than expecting a correction (§10.7) |

**Existing scripts, worth using rather than rewriting**, all under
`Extras/python_scripts/Bluetooth commands/`: `aAccel5Hz.py`,
`exgSquareWave512Hz.py`, `samplingRate.py`, `derivedChannels.py`,
`bmp390_plot.py`, `bmp581_plot.py`, `bmp_compare.py` (overlays a BMP581 and a
BMP390 stream — how the `.2` change was checked rather than assumed),
`btCalV2Rx.py` / `btCalV2Tx.py`.

#### 5.4.5 Coverage, honestly

You will not have one of every model. Prioritise by **how much firmware is
unique to the row**, not by how many units exist:

1. One board per **generation** — the widest code-path differences.
2. Both sides of a **live revision boundary** (§5.4.3), for the release's gates.
3. One board per **expansion type** — ExG, GSR+, Bridge Amp, Proto3 — since each
   brings its own channels and calibration.

Everything else is a repeat of a path already walked. Record which rows you
actually covered; an untested row is not a passed row.

### 5.5 Throughput and the rate ladder

The firmware has a **built-in throughput test**: `SET_DATA_RATE_TEST` (`0xA4`)
streams 5-byte packets — one header byte plus a `uint32_t` counter that
increments once per packet — as fast as the link will carry them. A gap in the
counter is a dropped packet, and it needs no sensor configuration at all, so it
separates *link* throughput from *sampling* throughput.

`Extras/python_scripts/Bluetooth commands/SpeedTest/` drives it:
`speedTest.py` (classic), `speedTestBle.py` (BLE), `speedTestPlot.py`.

| # | Step | Expected |
|---|---|---|
| 5.5.1 | Data rate test, classic, 10 min | No counter gaps; rate recorded and compared with the previous release |
| 5.5.2 | Data rate test, BLE, 10 min, where supported | As above; BLE is expected to be slower, but *record the number* |
| 5.5.3 | Repeat at each baud the fitted module supports | See the ladder below |
| 5.5.4 | Climb the sampling-rate ladder with all channels on until packets drop | Note the rate it breaks at, and compare with the previous release |

**The baud ladder is per module, and the constraints are real:**

| Baud | Supported on |
|---|---|
| 115200 | All. The RN41/RN42 default and the fallback |
| 1200, 230400, 460800, 921600 | RN42 only |
| 1000000 | **RN4678 v1.23 only** — v1.13.5 and v1.22 have known problems |
| 2000000 | CYW20820 only |

> `Comms/shimmer_bt_uart.h:323-338`. The 1000000 row is the reason §5.1 insists
> on recording the banner: a unit that reports "RN4678" but is running v1.22 will
> accept the baud and then misbehave under load, which looks like a firmware
> regression and is not one.

Sampling rate is stored as a tick divider — the packet rate is
`32768 / samplingRateTicks` — so the ladder is not linear in the configured
value. [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md)
§3.1 has the encoding.

### 5.6 Logging and streaming

| # | Step | Expected |
|---|---|---|
| 5.6.1 | Log 10 min, all channels, max rate | No gaps; timestamps monotonic |
| 5.6.2 | Stream 10 min, all channels, max rate | No dropped packets |
| 5.6.3 | **Log and stream simultaneously, max rate** | Both intact — this is the headline feature and the hardest case |
| 5.6.4 | Log across a file-split boundary | Files continuous, no lost samples at the join |
| 5.6.5 | Pull the card mid-log | Clean stop; the file up to that point is readable |
| 5.6.6 | Start/stop 20 times | No leak, no drift, no stuck state |

Check 5.6.1–5.6.3 in Consensys, not just for file size. **A packet with a zero
timestamp reads as a 24-bit roll-over worth 512 s** — that is the DEV-1023
signature, and `test_packet_ring` covers the mechanism, but only a real long-run
recording covers the whole path.

### 5.7 Battery and charging

`test_battery` covers the classification logic exhaustively; what it cannot do
is confirm the ADC actually reads what the cell is doing.

| # | Step | Expected |
|---|---|---|
| 5.7.1 | Dock a part-charged unit | Steady red, then green at full |
| 5.7.2 | Undock at each charge band | LED colour matches the band |
| 5.7.3 | Run to the auto-stop threshold with the option enabled | Logging stops; the card is readable |
| 5.7.4 | The same with it disabled | Logging continues |

### 5.8 Bluetooth link behaviour

| # | Step | Expected |
|---|---|---|
| 5.8.1 | Pair, connect, disconnect, reconnect ×5 | Reliable, no reset needed |
| 5.8.2 | Walk to the edge of range while streaming | Degrades and recovers; no lockup |
| 5.8.3 | Power off the host mid-stream | Device returns to idle by itself |
| 5.8.4 | BLE, where supported | Connects and streams |

### 5.9 Multi-device, if SD sync is in the release

| # | Step | Expected |
|---|---|---|
| 5.9.1 | Three units, one centre, 30 min | All files carry usable offsets |
| 5.9.2 | Power-cycle a node mid-session | Rejoins; the gap is visible in the data, not silent |

### 5.10 Power

| # | Step | Expected |
|---|---|---|
| 5.10.1 | Sleep current, undocked, idle | Within spec for the board |
| 5.10.2 | Overnight idle undocked | Still responsive; battery drop as expected |

### 5.11 Soak, and the counters that survive it

The faults worth soaking for are intermittent by definition, and the firmware
already keeps a persistent tally of the four that matter. `gEepromSensorSettings`
carries, in EEPROM and across power cycles:

| Counter | What it records |
|---|---|
| `btCntDisconnectWhileStreaming` | The link dropped mid-stream |
| `btCntUnsolicitedReboot` | The radio rebooted on its own |
| `btCntRtsLockup` | Flow control locked up |
| `btCntDataRateTestBlockage` | The data rate test stalled |

> `EEPROM/shimmer_eeprom.{h,c}`;
> [SHIMMER3_EEPROM_MEMORY_MAP.md](SHIMMER3_EEPROM_MEMORY_MAP.md).

**Read them before the soak, reset them, and read them again after.** A soak that
ends with a device still streaming has told you very little on its own; the same
soak with a delta of zero on all four counters has told you a great deal, and a
non-zero delta names which mechanism to chase.

| # | Step | Expected |
|---|---|---|
| 5.11.1 | Reset the counters, stream + log overnight, read them back | All four still zero |
| 5.11.2 | Repeat on one unit per radio firmware from §5.1 | As above — this is where a bad radio build shows itself |

---

## 6. Gate 5 — host-side and Consensys compatibility

**Required for every release, both platforms.** This is the gate with the widest
blast radius: a firmware change that breaks a host does not break one unit on a
bench, it breaks every customer's analysis the day they update.

### 6.1 Why the firmware cannot help you here

Two facts, both already documented, and together they are the whole reason this
section exists:

> **"The firmware carries no compatibility logic. Every gate is host-side."**
> — [SHIMMER3_RELEASE_AND_VERSIONING.md](SHIMMER3_RELEASE_AND_VERSIONING.md) §6

> **Older firmware silently ignores unknown opcodes rather than NACKing them**,
> so sending a newer command to older firmware produces *no response at all* —
> indistinguishable from a dropped packet.

So the firmware will not refuse an incompatible host, will not announce a
changed layout, and will not report a command it no longer implements. Nothing
in gates 1–4 looks at a host at all. **If a firmware change breaks host
software, this gate is the only thing between it and a customer.**

The corollary is worth stating plainly: because every gate is host-side, a
firmware change that is *correct* can still be breaking. Renumbering an opcode,
reusing a config byte, adding a channel to the middle of a packet — each is a
reasonable firmware decision and each invalidates every host that was not
updated in step.

### 6.2 The five surfaces a host depends on

Anything crossing one of these is a compatibility change, whatever it looked
like in the diff:

| Surface | What a host does with it | Reference |
|---|---|---|
| **BT command protocol** | Opcodes, argument and response lengths, ACK/NACK, CRC mode | [SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md) |
| **Streaming packet layout** | Channel order, widths, encodings, timestamps | [SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) |
| **Configuration image** | The 512-byte InfoMem byte map, and its SD-header twin | [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) |
| **SD card format** | Directory naming, file header, sample records, `sdlog.cfg`, the calibration file | [SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md) |
| **Dock protocol** | A *different* protocol, not a subset of the BT one | [SHIMMER3_DOCK_PROTOCOL.md](SHIMMER3_DOCK_PROTOCOL.md) |

> **The InfoMem and the SD header are not parallel layouts.** `config2SdHead`
> is a field-by-field copy to different offsets, which is why Appendix A of the
> InfoMem document exists. A change to one needs the other checked, and both
> need the host checked.

### 6.3 What CI already checks

`make -C Test/host host-constants` compares the firmware headers, the protocol
document and the Python host reference in `Extras/python_scripts/`. It runs on
every push, needs no device and no compiler, and is the only automated
host-compatibility check that exists.

It is deliberately **value-centric, not name-centric** — the wire contract is
the number. The firmware has renamed two dozen opcodes for clarity (`ACCEL` →
`WR_ACCEL` / `LN_ACCEL`, `PRES` → `PRESSURE`) without moving a single value;
those are reported and not failed. What fails:

| Finding | Why it is a failure |
|---|---|
| A host opcode with no firmware opcode at that value, where the document says the firmware implements it | Removed or renumbered — the host will get silence |
| An opcode value documented as two different firmware commands | A packet carrying it is ambiguous |
| A board code or hardware ID whose value differs between firmware and host | Devices identified as the wrong model |

The protocol document arbitrates, which is what keeps the check quiet enough to
act on: its "FW name" column is empty for opcodes the firmware never
implemented — the Java driver's legacy ExG calibration commands, for instance —
so a host carrying those is correct by design rather than a finding. **If the
opcode table's columns are ever restructured, the check fails closed** with a
message saying so, rather than silently passing.

> **This only covers the host that lives in this repository.** Consensys, the
> Java driver and the web SDK restate the same constants again and cannot be
> reached from CI. The Python reference is a proxy — a good one, because it
> drifts the same way for the same reasons, but a proxy. §6.4 is not optional
> because §6.3 is green.

### 6.4 Manual: the current host stack against the new firmware

The core of the gate. Run against a device carrying the release candidate, with
the **shipping** version of each host — not a development build.

| # | Step | Expected |
|---|---|---|
| 6.4.1 | Consensys: discover, connect, read the configuration | Device identified with the right model and firmware version |
| 6.4.2 | Consensys: write a configuration, power-cycle, read it back | Byte-identical, and the UI shows what it wrote |
| 6.4.3 | Consensys: stream every channel the board carries | All channels plotted, correctly labelled, correctly scaled |
| 6.4.4 | Consensys: import a recording made on the release candidate | Parses; sample count and duration match the trial |
| 6.4.5 | Consensys: import a recording made on the **previous** release | Still parses — a format change must not orphan existing data |
| 6.4.6 | The dock route: configure and read back over the dock | Matches the BT route |
| 6.4.7 | Web Bluetooth / TypeScript SDK tools, where they cover the change | Connect, configure, stream |
| 6.4.8 | The in-repo Python suites (§4) against the release candidate | Pass |

6.4.5 is the one people skip and the one that hurts. **A host update ships to
customers after the firmware, or never** — so the firmware must keep working
with the host version already installed, and old recordings must keep opening in
the new host.

### 6.5 Both directions of the version skew

A release is used in four combinations, not one. Walk the two diagonals:

| | Old firmware | New firmware |
|---|---|---|
| **Old host** | The baseline | **Test this.** The common case in the field: firmware updated, host not |
| **New host** | **Test this.** A customer with a mixed fleet | The happy path everyone tests |

| # | Step | Expected |
|---|---|---|
| 6.5.1 | New firmware, previous shipping host | Everything the old host supported still works |
| 6.5.2 | Previous firmware, new host | The host gates on version rather than probing — no hangs |
| 6.5.3 | A new command sent to previous firmware | The host times out cleanly and says something useful |

6.5.3 is worth doing explicitly because the failure is *silence*, not an error.
A host that probes instead of gating on the version tuple
`(hardwareVersion, firmwareIdentifier, major, minor, patch)` will appear to hang.

### 6.6 When a compatibility change is unavoidable

Sometimes it is. Then the job is to make it loud rather than to avoid it:

1. **Bump the version so hosts can gate on it**, and say which field moved —
   [SHIMMER3_RELEASE_AND_VERSIONING.md](SHIMMER3_RELEASE_AND_VERSIONING.md).
   A host cannot gate on something that did not change.
2. **Update the document for the surface that changed**, in the same PR. The
   documents in `docs/` are what host teams implement from; a change that
   reaches a device before it reaches the document is a change nobody can
   implement against.
3. **Update the Python host reference** in `Extras/python_scripts/`, so
   `host-constants` stays green and the next person sees the new shape.
4. **Say so in the release notes, naming the host versions required.** "Requires
   Consensys ≥ x.y" is a sentence that saves a support cycle.
5. **Never reuse a retired opcode or config byte for a new meaning.** An old host
   will parse it as the old thing and be confidently wrong, which is far worse
   than getting nothing back. Take the next free value instead.

---

## 7. Regression cases

Faults that reached a customer, or nearly did. **Every one of these is on the
list because it got past the testing of its day**, which is the only reason a
case earns a permanent place here.

| Case | Symptom | Covered by |
|---|---|---|
| DEV-1023 | Zero timestamp in a logged packet, read as a 512 s jump | `test_packet_ring` — plus §5.6.1 |
| DEV-1019 | Unprogrammed EEPROM reported as a real board, SR0-0-0 | `test_boards` — plus §5.2.6 |
| DEV-1003 | CubeMX regeneration deleted 165 lines of `main.c` | `check_cubemx_guards.sh` — plus §3.3 |
| DEV-1026 | I2C bus completion flags stale between gathers | §5.6.1, all channels |
| DEV-866 | A board with a dead LSE hangs at boot | §5.2.1 on an affected unit |
| DEV-818 | BMP581 / BMP390 fitted per revision | `test_boards`, `crosscheck_board_revisions.py` |
| Radio bring-up | A unit halts at boot, yellow at 5 Hz, on some radio-firmware combinations | §5.1 — nothing automated reaches this |
| Host constant drift | A host restates a firmware constant and the firmware moves it; the device is fine and every host is wrong | `crosscheck_host_constants.py` for the in-repo host, §6.4 for the rest |
| BMP581 SR48 window | A plain `>= 7.2` wrongly claims SR48-8-0 and 8-1 | `test_boards` — both layers catch it |

**When a fault escapes, add its case here and a test for it in the same PR.** A
regression list that only grows by hand stops growing.

---

## 8. Release sign-off

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
- [ ] **§5.1 boot matrix** walked — one unit per radio firmware, banner strings recorded
- [ ] §5.4 walked on the models the release's changes actually touch (§5.4.5), rows recorded
- [ ] Rest of §5 walked on at least one unit, results recorded
- [ ] **§6.4 host stack** exercised against the release candidate, with the *shipping* host versions
- [ ] **§6.4.5** — a recording from the previous release still imports
- [ ] **§6.5** version skew walked in both directions
- [ ] Any compatibility change handled per §6.6: version bumped, document updated, host reference updated, release notes name the host version required
- [ ] §7 regression cases considered against what changed
- [ ] `version.h` — all four values changed together, string zero-padded
- [ ] `FirmwareIdentifierList.txt` in step, if a build was added
- [ ] Documentation updated for every behaviour change in the release
- [ ] Release notes name what changed and what was tested

Then trigger `build-release-firmware.yml` by **workflow_dispatch**. The push
trigger is commented out on purpose, so releases are never accidental.

### 8.1 A shared-library change spans two releases

A change to `log-and-stream-common` is not released. It ships when a platform
firmware bumps its submodule pointer and releases — so a shared change needs
**both** platforms taken through §3 onwards before it can be called released,
even if only one of them ships first.

---

## 9. Extending the host tests

This is the cheapest testing you have, and the list in §2.2 is short because of
what is reachable, not because of what is worth covering.

### 9.1 What makes a module reachable

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

### 9.2 The next candidates, roughly by value

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

### 9.3 Adding a suite

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

### 9.4 What a good case here looks like

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

- **Sleep-current figures (§5.10.1).** The acceptance limits are per board and
  live in the hardware documentation, not in this repository. The step is listed
  without a number deliberately.
- **Throughput acceptance figures (§5.5).** `SET_DATA_RATE_TEST` and the
  `SpeedTest/` scripts measure a rate, but no expected rate is recorded anywhere
  in the firmware or docs, per module or per baud. The steps therefore say
  "record and compare with the previous release" rather than naming a threshold.
  **Capturing one release's numbers would turn §5.5 from a trend check into a
  pass/fail gate**, and is the single cheapest improvement available to this
  section.
- **Which CYW20820 / EZ-Serial firmware versions are qualified (§5.1).** The
  Shimmer3 side names six RN4678 versions and three RN41/RN42 versions in code,
  so the matrix rows are exact. The Shimmer3R side has no equivalent version
  list in the firmware — `Extras/WsOtaUpgrade/` carries one EZ-Serial build
  (`v1.4.16.16`), but whether others are supported is not recorded here.
- **The maximum sampling rate per board and channel set (§5.5.4).** The ladder
  step asks where packets start dropping; no documented ceiling exists to check
  it against.
- **Consensys, the Java driver and the web SDK are not reachable from CI
  (§6.3).** `crosscheck_host_constants.py` checks the one host that lives in
  this repository. The others restate the same constants in other repositories,
  so §6.4 is a manual step and there is no automated equivalent. Whether a
  contract test could be shared across those repositories is an open question
  worth asking; it would be the largest single reduction in risk available to
  this document.
- **Which Consensys version is "the shipping version" at any time (§6.4).** The
  procedure says to test against it rather than a development build; the
  version itself is a release-management fact this repository does not record.
- **`SHIMMER4_SDK` (board code 58) is absent from the Python host reference's
  `SrBoardCodes`.** Reported by `crosscheck_host_constants.py` as a note rather
  than a failure, because the reference is a test tool and not a shipping host.
  Listed here so it is a known gap rather than an unexamined one.
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
