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
| `FACTORY_TEST_COUNT` | 4 | Count, not a suite |

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
> [SHIMMER3_TIMEKEEPING.md](SHIMMER3_TIMEKEEPING.md) §7.

> **"Not measurable" is not the same as "out of tolerance".** The four error
> codes distinguish a crystal that is running badly from one that is not
> running at all, or a reference that is missing — different faults with
> different fixes.

### 5.4 LED tests

`FACTORY_TEST_LED_STATES` calls `ShimLeds_testOperationalStates`, which drives
the LEDs through the operational patterns so an operator can confirm each
colour and each state renders. It bypasses the normal state machine entirely.

`FACTORY_TEST_LEDS` is the simpler exercise. Neither sets a result bit; both
are visual checks. See
[SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) §7.

## 6. Host capture

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

## Still unverified / not found in code

- **Tests 1, 2, 4, 5 and 6.** Not defined in the current firmware. Whether they
  existed historically, and what they covered, is not determinable from the
  present source — which matters when reading an archived report whose mask has
  those bits set.
- **The Shimmer3 test registry.** This document covers Shimmer3R. Shimmer3 runs
  the same harness but its test bodies and IDs are in `shimmer3-firmware` and
  were not read.
- **Per-test acceptance limits.** The report prints each limit alongside the
  measurement, so limits are discoverable from any captured report, but they
  are not tabulated here — they live in `hal_FactoryTest.c` and vary by model.
- **Whether `FACTORY_TEST_LEDS` and `FACTORY_TEST_ICS` are used in production**
  or only `FACTORY_TEST_MAIN`. The suites exist; which the production script
  invokes was not established.
- **How the test is triggered in production.** `TASK_FACTORY_TEST` runs it, and
  a Bluetooth command and the dock protocol's `UART_COMP_TEST` component both
  reach it, but the production sequence was not traced.
- **`micTestResult_t`.** A per-message structure for the microphone test whose
  handling was not examined.
