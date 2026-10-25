# Shimmer3R Factory Test Report

The on-device self-test that production uses to accept or reject a board: what
it runs, what the report looks like, what each test ID means, and how to read
the result bitmask.

> **Verified against** — the revisions these claims were read from. A pinned
> commit is a citation, not a claim of currency.
>
> - **Firmware:** `log-and-stream-common` @ `f3cf73e` —
>   `Test/shimmer_test.{h,c}`, `Test/shimmer_test_leds_states.c`.
> - **Platform firmware:** `shimmer3r-firmware` @ `a8f105e5` —
>   `Shimmer_Driver/hal_FactoryTest.{h,c}` (the test bodies, the
>   `S3R_TEST_*` bitmask and the report strings).

> **Shimmer3R focus.** The shared harness in `Test/shimmer_test.c` builds for
> both generations, but the test bodies and the `S3R_TEST_*` registry are in
> the Shimmer3R platform repository. Shimmer3's equivalents were not read for
> this document.

**Source references:**

| Layer | File |
|---|---|
| Harness, targets, truncation | `Test/shimmer_test.c` |
| Test enums and result strings | `Test/shimmer_test.h` |
| LED state walkthrough | `Test/shimmer_test_leds_states.c` |
| Test bodies and ID registry | `shimmer3r-firmware` `Shimmer_Driver/hal_FactoryTest.{h,c}` |

---

## 1. Running a test

`ShimFactoryTest_setup(target, testToRun)` then `ShimFactoryTest_run()`, driven
by `TASK_FACTORY_TEST`.

### 1.1 Test suites

| Constant | Value | Runs |
|---|---:|---|
| `FACTORY_TEST_MAIN` | 0 | The full production suite |
| `FACTORY_TEST_LEDS` | 1 | LED exercise only |
| `FACTORY_TEST_ICS` | 2 | Integrated-circuit tests only |
| `FACTORY_TEST_LED_STATES` | 3 | Walk the operational LED states |
| `FACTORY_TEST_POWER_PROFILE` | 4 | **Not a suite.** Arms the power-profile run (§6), over the dock only, on Shimmer3R v1.01.024 and later |
| `FACTORY_TEST_COUNT` | 5 | Count, not a suite |

Every value is spelled out in `factory_test_t`, because hosts restate them:
`Test/host/crosscheck_host_constants.py` checks them against the in-repo host
reference. Before v1.01.024 the enum ended at `FACTORY_TEST_LED_STATES` and
`FACTORY_TEST_COUNT` was 4.

`FACTORY_TEST_POWER_PROFILE` never reaches `ShimFactoryTest_run()`: the dock
handles it inline and the Bluetooth `0xA8` refuses it with a NACK
([SHIMMER3_DOCK_PROTOCOL.md](SHIMMER3_DOCK_PROTOCOL.md) §4.4).

### 1.2 Output targets

| Constant | Value | Destination |
|---|---:|---|
| `PRINT_TO_DEBUGGER` | 0 | SWD / debug console |
| `PRINT_TO_DOCK_UART` | 1 | Dock connector |
| `PRINT_TO_BT_UART` | 2 | Bluetooth link |

`ShimFactoryTest_sendReportImpl` is `__weak` with an empty default body, so a
platform that does not override it produces **no output at all** while still
computing the result bitmask.

## 2. Report format

The report is **plain text**, not a structured response:

```
//**************************** TEST START ************************************//
Firmware version: <FW_VERSION_STRING>
S3R_TEST_0003 - PASS: <daughter card id>
S3R_TEST_0007 - PASS: VRef = 1650mV (1600-1700mV)
...
Overall Result = PASS
//***************************** TEST END *************************************//
```

The firmware version is emitted first, deliberately, so a captured report
records which build produced it.

Per-line format is `S3R_TEST_nnnn - <PASS|FAIL|WARNING>: <detail>`, and where a
measurement is involved the detail carries the value **and the accepted range**,
so a marginal result is visible without consulting a spec.

The overall line appears only for `FACTORY_TEST_MAIN` and `FACTORY_TEST_ICS`:

```
Overall Result = PASS
Overall Result = FAIL (0x00A0C008)
```

The hexadecimal value is the result bitmask (§4).

### 2.1 Truncation

`MAX_TEST_REPORT_LENGTH` is **128**. `ShimFactoryTest_sendReport` truncates any
longer string, then emits it.

> **Truncation is silent and lossy.** A string over 128 characters is cut with
> no marker, and because the harness emits one string per call rather than a
> line-oriented stream, a truncated string can glue what should have been two
> lines together in the captured text. A host parsing the report must tolerate
> a line that ends mid-sentence and a line that contains two `S3R_TEST_` tokens.

## 3. Result strings

| Constant | Text |
|---|---|
| `SELF_TEST_STR_PASS` | `PASS` |
| `SELF_TEST_STR_FAIL` | `FAIL` |
| `SELF_TEST_STR_EMPTY` | *(empty)* |
| `SELF_TEST_STR_CHIP_DETECTION` | ` - Chip not detected` |
| `SELF_TEST_STR_SIGNAL_ISSUE` | ` - Signal issue` |
| `SELF_TEST_STR_TEMPERATURE_ISSUE` | ` - Temperature issue` |
| `SELF_TEST_STR_DRDY_ISSUE` | ` - DRDY/INT issue` |
| `SELF_TEST_STR_UNKNOWN` | ` - Unknown` |

The suffixes distinguish *why* a chip test failed: absent from the bus, present
but producing bad data, a self-test temperature out of range, or a
data-ready/interrupt line not toggling.

> **`- Chip not detected` and `- Signal issue` point at different faults.** The
> first is a bus or power problem; the second means the part answered and its
> data was wrong. Do not treat them as the same reject category.

## 4. Test ID registry

`shimmerStatus.testResult` is a 32-bit mask. **Bit position is
`testNumber - 1`**, so `S3R_TEST_0003` is bit 2 (`0x00000004`).

| ID | Bit | Mask | Subsystem |
|---|---:|---|---|
| `S3R_TEST_0003` | 2 | `0x00000004` | Shimmer model set in EEPROM |
| `S3R_TEST_0007` | 6 | `0x00000040` | MCU VRef |
| `S3R_TEST_0008` | 7 | `0x00000080` | MCU VCore |
| `S3R_TEST_0009` | 8 | `0x00000100` | MCU VBatt pin |
| `S3R_TEST_0010` | 9 | `0x00000200` | MCU temperature |
| `S3R_TEST_0011` | 10 | `0x00000400` | Battery voltage |
| `S3R_TEST_0012` | 11 | `0x00000800` | Battery charger chip |
| `S3R_TEST_0013` | 12 | `0x00001000` | SD card |
| `S3R_TEST_0014` | 13 | `0x00002000` | Bluetooth module |
| `S3R_TEST_0015` | 14 | `0x00004000` | SPI1 — ADS7028 external ADC |
| `S3R_TEST_0016` | 15 | `0x00008000` | SPI1 — LSM6DSV |
| `S3R_TEST_0017` | 16 | `0x00010000` | SPI1 — BMP390 |
| `S3R_TEST_0018` | 17 | `0x00020000` | SPI1 — ADXL371 |
| `S3R_TEST_0019` | 18 | `0x00040000` | SPI2 — LIS3MDL |
| `S3R_TEST_0020` | 19 | `0x00080000` | SPI2 — LIS2DW12 |
| `S3R_TEST_0021` | 20 | `0x00100000` | SPI3 — ADS1292R (ExG) |
| `S3R_TEST_0022` | 21 | `0x00200000` | I2C1 — LIS2MDL |
| `S3R_TEST_0023` | 22 | `0x00400000` | I2C1 — CAT24C16 |
| `S3R_TEST_0024` | 23 | `0x00800000` | I2C4 — CAT24C16 or GSR rig |
| `S3R_TEST_0025` | 24 | `0x01000000` | GSR signal |
| `S3R_TEST_0026` | 25 | `0x02000000` | Microphone |
| *(0027)* | — | — | **LED visual check — no bit** |
| `S3R_TEST_0028` | 27 | `0x08000000` | MCU LSE crystal frequency error |

> **The numbering has gaps, and they are load-bearing.** Tests 1, 2, 4, 5 and 6
> are not defined in the current firmware, and **27 is deliberately skipped**.
> The source comment explains why: shipped reports already carry the heading
> "LED test (S3R_TEST_0027)" as a visual check with no pass/fail bit, and
> renumbering would break comparison against archived reports. **Do not close
> the gaps.** A new test takes the next unused number, not a vacated one.

> **`S3R_TEST_0027` produces a report line but never sets a bit.** A report can
> therefore say `Overall Result = PASS` while the LED check was never actually
> confirmed by anyone — it is an operator's visual judgement, not a measurement.

### 4.1 Decoding a failure mask

`Overall Result = FAIL (0x00A0C008)`:

| Bit | Mask | Test |
|---:|---|---|
| 3 | `0x00000008` | *(no test at bit 3 — see below)* |
| 14 | `0x00004000` | `S3R_TEST_0015` ADS7028 |
| 15 | `0x00008000` | `S3R_TEST_0016` LSM6DSV |
| 21 | `0x00200000` | `S3R_TEST_0022` LIS2MDL |
| 23 | `0x00800000` | `S3R_TEST_0024` I2C4 |

> Bit 3 corresponds to test 4, which is not defined. A set bit with no
> corresponding test means either an older firmware produced the report or a
> test was removed — check the firmware version line.

## 5. Notable tests

### 5.1 `S3R_TEST_0003` — model set in EEPROM

Passes when the daughter-card identity is set, failing with
`FAIL: not set` otherwise. See
[SHIMMER3_EEPROM_MEMORY_MAP.md](SHIMMER3_EEPROM_MEMORY_MAP.md) §3.

An unset model makes several later tests report "not applicable for this model"
rather than failing, so **this test failing invalidates much of the rest of the
report**.

### 5.2 `S3R_TEST_0024` and `S3R_TEST_0025` — the test rig dependency

`0024` looks for a test rig on I2C4 — either a second CAT24C16 or a GSR
fixture. `0025` tests the GSR signal path through that fixture.

When no rig is detected, both fail together, with `0025` reporting
`FAIL: GSR - refer to S3R_TEST_0024`.

> **A GSR failure is very often a missing or unrecognised test rig, not a
> faulty board.** Always read `0024` before acting on `0025`. Both also report
> "not applicable for this model" on hardware without GSR, which is a pass, not
> a skip to investigate.

### 5.3 `S3R_TEST_0028` — LSE crystal frequency

Measures the 32.768 kHz low-speed external oscillator against the 16 MHz
high-speed external reference and reports the error in **tenths of a ppm**,
alongside the limit and which load-capacitor revision the board carries:

```
S3R_TEST_0028 - PASS: 32k LSE vs 16M HSE error = -1.4 ppm (limit +/-20.0 ppm, 22pF caps rev...)
```

When the measurement cannot be made it reports a distinct failure with a
diagnostic code:

| Code | Meaning |
|---|---|
| `LSE_MEAS_OK` | Measured |
| `LSE_MEAS_ERR_LSE_NOT_READY` | LSE oscillator not running |
| `LSE_MEAS_ERR_HSE_NOT_READY` | 16 MHz reference not running |
| `LSE_MEAS_ERR_TIMEOUT` | Capture stream never delivered |
| `LSE_MEAS_ERR_GAP` | Capture gap too long to reconstruct |

> **This test exists because a real drift problem was traced to under-specified
> load capacitors**, and the report names the capacitor revision so a marginal
> result can be attributed to the board build rather than the crystal. See
> [SHIMMER3_TIMEKEEPING.md](SHIMMER3_TIMEKEEPING.md) §8.

> **"Not measurable" is not the same as "out of tolerance".** The four error
> codes distinguish a crystal that is running badly from one that is not
> running at all, or a reference that is missing — different faults with
> different fixes.

### 5.4 LED tests

`FACTORY_TEST_LED_STATES` calls `ShimLeds_testOperationalStates`, which drives
the LEDs through the operational patterns so an operator can confirm each
colour and each state renders. It bypasses the normal state machine entirely.

`FACTORY_TEST_LEDS` is the simpler exercise. Neither sets a result bit; both
are visual checks. Each LED state is held for `DELAY_BETWEEN_LED_CHANGES_MS`,
**2000 ms** (`hal_FactoryTest.h`), not 1 s: an operator script that times the
states must allow 2 s each.

A host that needs the operator to confirm each colour can drive the LEDs
itself instead, with the dock's `LED_SET`
([SHIMMER3_DOCK_PROTOCOL.md](SHIMMER3_DOCK_PROTOCOL.md) §4.2): it holds the
state until released, so a forced-choice check need not race a timer. See
[SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) §7.

## 6. The power-profile run

**Shimmer3R v1.01.024 and later.** A fixed sequence a production test arms over
the dock and the unit runs at its **next boot**, powered from a battery or a
bench supply with no cable attached, so a power analyser can capture what each
part draws. The firmware measures nothing itself: it runs the steps in a known
order and timing, marks the trace so a capture can be aligned and attributed
to the unit, and records what happened.

| Piece | Where |
|---|---|
| Record, schedule and readback layouts | `Test/shimmer_factory_record.{h,c}` |
| Arm, disarm and readback | `Comms/shimmer_dock_usart.c`; [SHIMMER3_DOCK_PROTOCOL.md](SHIMMER3_DOCK_PROTOCOL.md) §4.4 |
| The run and the schedule | `shimmer3r-firmware` `Shimmer_Driver/hal_PowerProfile.c` |
| The EEPROM page | [SHIMMER3_EEPROM_MEMORY_MAP.md](SHIMMER3_EEPROM_MEMORY_MAP.md) §2 |

### 6.1 States

The record lives in one EEPROM page (bytes 1920-1935). Its `state`:

| Value | State | Set by | Meaning |
|---:|---|---|---|
| 0 | `DISARMED` | dock disarm | nothing will run |
| 1 | `ARMED` | dock arm | runs at the next boot |
| 2 | `RUNNING` | the boot that runs it | a run is under way, or was cut short |
| 3 | `DONE` | the run | every step ran |
| 4 | `SKIPPED` | the boot | that boot was docked or on USB |
| 5 | `ABORTED` | a boot, or the run | a boot found it `RUNNING`, or a dock or USB cable appeared mid-run |
| 0xFF | `NONE` | (readback only) | the page holds no valid record |

- **An arm is consumed by the very next boot**, whatever that boot does with
  it. It runs only if the unit is neither docked nor on USB
  (`LogAndStream_isDockedOrUsbIn()`); otherwise the boot marks it `SKIPPED`.
- **`RUNNING` is written, and read back, before anything else happens.** If
  that write fails the run does not start, so a reset can never run it twice.
  A boot that finds `RUNNING` marks it `ABORTED` and does not re-run it: its
  trace is already wrong.
- **`run_seq` is written before each step**, so a record left `RUNNING` or
  `ABORTED` says which step the run was on. It equals the step count once
  `DONE`. `step_ok` gets bit *i* when step *i* completed without a driver
  error.

### 6.2 The sequence

The run takes the boot over between the configuration stage and the end of
boot, on its own boot stage (`BOOT_STAGE_POWER_PROFILE`), and lasts about a
minute. First it **quiets the unit**: the LED blink timer stops, the SD card is
unmounted and unpowered, and Bluetooth is stopped. Every wait sleeps (`WFI`)
and is timed from the RTC, which runs from the 32.768 kHz LSE, not from the
SysTick; each wait is also bounded by the HAL tick, so a stopped clock cannot
hold the boot. Nothing in the run reaches `Error_Handler()` on a part that does
not answer: the part's step loses its `step_ok` bit and the run carries on.

Each step is followed by a **return-to-baseline window** (1000 ms) with nothing
on. Which steps run follows the board, and the readback lists exactly the steps
this board runs:

| Id | Step | ms | Notes |
|---:|---|---:|---|
| 1 | Marker | 2100 | §6.3 |
| 2 | Baseline | 5000 | Bluetooth, SD and LEDs off, sensing rail off |
| 3 | Sensing rail | 3000 | rail on, every part idle; the part probe runs here (§6.5) |
| 4 | LSM6DSV | 2000 | accel and gyro at 7680 Hz, then `restore_default` |
| 5 | LIS3MDL | 2000 | where fitted: ultra-high-performance 155 Hz, then `restore_default` |
| 6 | LIS2DW12 | 2000 | high performance 1600 Hz, then `standby` |
| 7 | LIS2MDL | 2000 | 100 Hz continuous, then `sleep` |
| 8 | ADXL371 | 2000 | where fitted: full-bandwidth measurement at 5120 Hz, then `reset_chip` |
| 9 / 10 | BMP390 / BMP581 | 2000 | whichever was identified at boot, at its fastest rate; then BMP390 soft reset, BMP581 deep standby |
| 11 | ADS1292R | 2000 | ExG boards: both chips converting, then powered off |
| 12 | ADS7028 / GSR | 3000 | ADS7028 converting the GSR channel; on a GSR board the front end on its 40 kohm range |
| 13 | Expansion power | 3000 | GSR, bridge-amplifier and Proto3 Deluxe boards: the board's power switch on |
| 14 | Microphone | 2000 | microphone power on |
| 15 | SD | 4000 | SD power, initialisation and mount; left mounted, as an undocked boot leaves it |
| 16-21 | LED channels | 1000 each | lower red, green, blue, then upper red, green, blue, each alone at full brightness |
| 22 | Bluetooth | 8000 | started as after boot (`InitialiseBtAfterBoot()`) |

Each step's time includes bringing its part up, so the activity can start a
few ms after the step's pulse. The sensors are configured with a sampling
frequency of 0, which keeps their data-ready interrupts off.

**The EEPROM sits on the sensing rail.** The record is written at every step
boundary, never inside a step or a window. Before step 3 the rail is off, so
each of those writes powers it up for the write and down again: three short
blips at the first three boundaries, outside every window.

The steps and their durations are versioned (the record's schedule version).
They are first figures, to be tuned on the bench against a capture.

### 6.3 The marker

Marker 1 (`PP_MARKER_NONCE16`):

1. **Sync:** 500 ms with all six LED channels at full brightness.
2. **Nonce:** the 16-bit nonce from the arm, most significant bit first, one
   100 ms slot per bit. The LEDs are on for the first **30 ms** of a slot for a
   `0` and the first **70 ms** for a `1`.
3. **Step boundaries:** every later step starts with a **20 ms** pulse of all
   LED channels.

The nonce ties a capture to the unit it was armed on, with no label to read: a
host arms each unit with a fresh random nonce, decodes it from the trace, and
checks it against the readback.

### 6.4 The readback

Dock `GET COMP_TEST/0x04`, at most 128 bytes, little-endian. It is built by the
same function the run takes its schedule from, so it lists exactly what this
board runs:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | layout version, `1` |
| 1 | 1 | state (§6.1) |
| 2 | 1 | `run_seq` |
| 3 | 2 | nonce |
| 5 | 4 | `step_ok` |
| 9 | 2 | `parts_found` (§6.5) |
| 11 | 2 | parts expected for the board's SR number |
| 13 | 2 | return-to-baseline window, ms |
| 15 | 1 | marker id |
| 16 | 1 | n, the number of steps |
| 17 | n × 3 | per step: step id u8, duration ms u16 |

With no valid record, `state` reads `0xFF` and the fields from `run_seq` to
`parts_found` read 0. At 3 bytes a step, n is at most 37.

### 6.5 The part probe

With the sensing rail on, the run probes the SPI parts whose fitting varies by
board. `parts_found` and the expected parts share these bits:

| Bit | Part | Probe |
|---:|---|---|
| 0 | LIS3MDL | `WHO_AM_I` (0Fh) reads 3Dh |
| 1 | ADXL371 | its device, MEMS and part IDs (`isAdxl371Detected()`) |
| 2 | ADS7028 | `SYSTEM_STATUS` at its default (`ads7028_factoryTestChipId()`) |
| 3 | BMP390 | identified by chip ID at boot |
| 4 | BMP581 | identified by chip ID at boot |
| 5 | ADS1292R | `EXG_init()` succeeds; probed only where the SR number says ExG |

A BMP chosen at boot by the SR number, because neither chip answered, is not
counted as found. The LSM6DSV, LIS2DW12 and LIS2MDL are on every Shimmer3R and
are not probed; neither is the microphone. The expected parts come from the SR
number's presence gates, so a mismatch narrows down a wrong SR number or a
missing part; it does not prove the SR number right.

## 7. Host capture

Over Bluetooth the report arrives as a stream of text chunks, not one message.
A host must:

1. Accumulate chunks until the `TEST END` banner.
2. Decode as UTF-8 with a **streaming** decoder — a multi-byte sequence can
   split across chunks.
3. Tolerate lines split across chunks, and lines glued together by the
   128-character truncation of §2.1.
4. Parse `Overall Result` for the mask, and the per-line `S3R_TEST_nnnn` tokens
   for detail.

> **Do not parse by fixed line count or fixed offsets.** Lines are conditional
> on the model, on rig presence, and on which suite was run.

## Appendix A. Acceptance limits (Shimmer3R)

Constants from `Shimmer_Driver/hal_FactoryTest.h` at the pinned revision. The
report prints the limit next to each measurement, so a captured report is
self-describing; this table is for reading the source or planning a change.

| Measurement | Lower | Upper | Notes |
|---|---|---|---|
| ADC reference (`VREF`) | `VREF_EXTERNAL_SUPPLY_MV − 80` | `+ 80` | 3000 mV on product hardware, 3300 mV on the Nucleo build (`hal_Board.h`) |
| MCU core voltage (`VCORE`) | 900 mV | 1800 mV | measured on ADC4's `VCORE` channel |
| `VBATT` pin (1.8 V regulator boards) | 1710 mV | 2200 mV | Asymmetric: healthy boards read high when warm and on USB (1924 mV seen at 36 °C), never low. Allows for the ÷4 VBAT bridge's ±5 % error (DEV-1113) |
| `VBATT` pin (SR48-6-0, 1.9 V regulator) | 1850 mV | 1950 mV | |
| MCU die temperature | 10 °C | 40 °C | |
| Battery voltage | 2980 mV | 4750 mV | |
| IMU temperature | 10.0 °C | 40.0 °C | `−1.0` is the invalid marker |
| LSE (32.768 kHz) error, HSE-cap-fixed boards | | 35.0 ppm | stored ×10 (`350`) |
| LSE error, deployed-fleet boards | | 100.0 ppm | stored ×10 (`1000`); applies below the HSE-cap-fix revision |
| GSR test-rig resistors | ±7 % / ±8 % | | two tolerance bands selected per range |
| Bluetooth module firmware | `"v01.04.16.16"` or `"v01.04.18.18"` | | exact match against either. v1.4.16.16 is the production module firmware; a production test that needs it checks the printed version |
| LED step delay | 2000 ms | | visual check pacing, not a limit |

The GSR and alternate-EEPROM tests talk to the test rig over `I2C4`
(`hi2c4`), which is otherwise unused by the application.

## Still unverified / not found in code

- **The power-profile run (§6) has not been on the bench.** It is later than
  the pin in **Verified against**. The marker timing, the step durations, the
  part probe on each board, and the current each step draws are all to be
  confirmed against a capture; until then the durations are first figures.

- **Tests 1, 2, 4, 5 and 6.** Not defined in the current firmware. Whether they
  existed historically, and what they covered, is not determinable from the
  present source — which matters when reading an archived report whose mask has
  those bits set.
- ~~The Shimmer3 test registry~~ — resolved: Shimmer3 has **no numbered
  registry and no pass/fail mask**. `hal_run_factory_test`
  (`Shimmer_Driver/5xx_HAL/hal_FactoryTest.c`) runs a fixed free-text sequence:
  model, MCU details, SD card, Bluetooth module, I2C, SPI (for `MAIN` and
  `ICS`), then the LED sequence (for `MAIN` and `LEDS`). A Shimmer3 report is
  parsed by reading the text, not by decoding bits.
- ~~Per-test acceptance limits~~ — resolved: tabulated in the appendix below
  from `Shimmer_Driver/hal_FactoryTest.h`.
- **Whether `FACTORY_TEST_LEDS` and `FACTORY_TEST_ICS` are used in production**
  or only `FACTORY_TEST_MAIN`. The suites exist; which the production script
  invokes was not established.
- **Which suite the production script invokes.** The *mechanism* is resolved:
  over the dock, `UART_SET` with component `UART_COMP_TEST` (`0x0B`) and the
  property byte equal to the suite index (`0`-`3`) calls
  `ShimFactoryTest_setup(PRINT_TO_DOCK_UART, suite)`, queues
  `TASK_FACTORY_TEST` and ACKs; the Bluetooth opcode does the same with the
  Bluetooth target. Which suite index the production fixture sends is a
  process fact, not a code fact.
- ~~`micTestResult_t`~~ — resolved: `{ const char *message; uint8_t
  selfTestResult; }` — the microphone self-test result byte paired with the
  report line to print for it. A formatting helper, not a data path.
