# Shimmer3 / Shimmer3R GSR Range Control

Galvanic skin response is measured through a switched-feedback amplifier with
four selectable resistors. This document covers how the range is chosen, how it
travels to the host inside the sample word, and the settling behaviour that
makes a range change visible in the data.

> **Verified against** — the revisions these claims were read from. A pinned
> commit is a citation, not a claim of currency.
>
> - **Firmware:** `log-and-stream-common` @ `f3cf73e` — `GSR/gsr.{h,c}` in
>   full.
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4`;
>   `shimmer3r-firmware` @ `a8f105e5` — the A0/A1 GPIO drive.

> **How to read this document.** **S3** = Shimmer3; **S3R** = Shimmer3R.
> LogAndStream only. ADC values are raw converter counts.

**Source references:**

| Layer | File |
|---|---|
| Ranges, thresholds, smoothing | `GSR/gsr.c` |
| Range enum and resistor table | `GSR/gsr.h` |
| Sample encoding | [SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §5.4, §7.3 |
| Configuration bits | [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §10.3 |

---

## 1. The ranges and feedback resistors

| Code | Constant | Feedback resistor |
|---:|---|---:|
| 0 | `HW_RES_40K` | 40,200 Ω |
| 1 | `HW_RES_287K` | 287,000 Ω |
| 2 | `HW_RES_1M` | 1,000,000 Ω |
| 3 | `HW_RES_3M3` | 3,300,000 Ω |
| 4 | `GSR_AUTORANGE` | **Not a resistor** — configuration only |

Values are from `GSR_FEEDBACK_RESISTORS_OHMS[]`. The nominal names are rounded:
`HW_RES_40K` is actually 40.2 kΩ.

The range is selected in configuration byte 9, bits 3-1, and is clamped to
`GSR_AUTORANGE` if written greater than 4
([SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §10.3).
Auto-range is the factory default.

`GSR_init` selects the configured resistor when it is 0-3, and falls back to
`HW_RES_40K` for anything else — including `GSR_AUTORANGE`, which is where
auto-ranging starts from.

## 2. Resistor selection and enable

Two GPIO lines, driven by `GSR_setA0` and `GSR_setA1`, select the resistor.

`gsrRangePinsAreReversed` inverts the mapping for boards wired the other way
round; it is set by the board layer.

> **A board with the reversal flag set wrong selects the wrong resistor
> silently.** Nothing verifies the selection against a measurement, so the only
> symptom is GSR values that are wrong by the ratio of two resistors — a factor
> of about 7 between adjacent ranges. If GSR readings are consistently out by a
> large constant factor, check this flag before the calibration.

## 3. The sample word

`GSR_range()` packs the range into the sample:

```c
*((uint16_t *) buf) = ADC_val | (current_active_resistor << 14);
```

| Bits | Field |
|---|---|
| 15-14 | Active resistor, 0-3 |
| 13-0 | ADC value |

**Every sample carries its own range.** Under auto-range the resistor changes
mid-stream, so the range is not a per-trial constant and a host must read the
two bits from each sample.

> **Range code 4 can never appear in a sample.** `GSR_AUTORANGE` is a
> configuration value; the two bits always hold the resistor actually in
> circuit. A host that sees 4 has mis-parsed.

> **Mask the value with `0x3FFF` before using it.** Treating the whole 16-bit
> word as the measurement adds up to 49152 counts of offset depending on the
> range in force, which looks like a step change in skin conductance.

## 4. Auto-range switching

`GSR_controlRange` runs per sample when the configured range is
`GSR_AUTORANGE`. Thresholds are in raw ADC counts, with hysteresis:

| Current resistor | Switch to a larger resistor below | Switch to a smaller resistor above |
|---|---:|---:|
| `HW_RES_40K` | 1120 → `HW_RES_287K` | — |
| `HW_RES_287K` | 1490 → `HW_RES_1M` | 3960 → `HW_RES_40K` |
| `HW_RES_1M` | 1630 → `HW_RES_3M3` | 3700 → `HW_RES_287K` |
| `HW_RES_3M3` | 1125 | 3930 → `HW_RES_1M` |

Constants, with the source's own comments:

| Constant | Value | Comment in source |
|---|---:|---|
| `HW_RES_40K_MIN_ADC_VAL` | 1120 | 10k to 56k; changed from 1159 → 1140 → 1120 for linear conversion |
| `HW_RES_287K_MAX_ADC_VAL` | 3960 | 56k to 220k; was 4000, measured 3948, then 3800, now 3960 for linear conversion |
| `HW_RES_287K_MIN_ADC_VAL` | 1490 | 56k to 220k; 1510 → 1490 |
| `HW_RES_1M_MAX_ADC_VAL` | 3700 | 220k to 680k |
| `HW_RES_1M_MIN_ADC_VAL` | 1630 | 220k to 680k; 1650 → 1630 |
| `HW_RES_3M3_MAX_ADC_VAL` | 3930 | 680k to 4M7 |
| `HW_RES_3M3_MIN_ADC_VAL` | 1125 | 680k to 4M7 |

> **These are empirically tuned, not derived.** Every one carries a comment
> recording what it was changed from and why. Do not "correct" them from first
> principles.

> **`HW_RES_40K` never switches upward** — there is no smaller resistor. A
> skin resistance below the 40 kΩ range's floor saturates the ADC with no
> indication beyond the value pinning high.

> **The `HW_RES_287K` band uses an inclusive window.** The code keeps the
> current resistor while `ADC_val <= 3960 && ADC_val >= 1490`, and only then
> tests the two directions. The other bands test the window the same way.

## 5. Settling after a range change

Switching a feedback resistor is a hardware transient. Two mechanisms hide it.

### 5.1 `GSR_smoothTransition` — the live path

Called from `GSR_range` before `GSR_controlRange`. When the resistor has just
changed it computes how many samples the settling period covers:

```c
transient_sample = ceil(SETTLING_TIME / sampling_period);
```

with `SETTLING_TIME` = **2621 ticks**, which the source annotates as
`32768 * 0.08` — **80 ms**.

While `transient_sample` is non-zero the function returns 1, and `GSR_range`
substitutes the **last valid ADC value** and the **previous resistor code**:

```c
if (GSR_smoothTransition(&current_active_resistor, gsrSamplingRateTicks))
    ADC_val = lastGsrVal;
else
    GSR_controlRange(ADC_val);
```

> **Repeated identical GSR samples around a range change are expected.** They
> are the firmware holding the last good value through the settling window, not
> dropped samples. At 51.2 Hz that is `ceil(2621 / 640)` = 5 samples; at 512 Hz
> it is 41.

> **The range bits also hold the *previous* resistor during settling**, so the
> repeated samples are self-consistent — value and range both describe the
> pre-switch state.

> **The comment says the settling time is "approx 40 ms" but the constant is
> 80 ms.** The comment above `GSR_smoothTransition` describes 40 ms; the
> constant and its own comment both say 0.08 s. The constant is what executes.

### 5.2 `GSR_smoothSample` — resistance-domain smoothing

A second, separate mechanism that works on computed resistance rather than raw
counts, limiting how far the reported resistance may step per sample:

| Constant | Value | Meaning |
|---|---:|---|
| `MAX_RESISTANCE_STEP` | 5000 | Normal maximum step, ohms |
| `ONE_HUNDRED_OHM_STEP` | 100 | Reduced step immediately after a switch |
| `NUM_SMOOTHING_SAMPLES` | 64 | Samples spent at the reduced step |
| `NUM_SAMPLES_TO_IGNORE` | 6 | Samples discarded after a switch |
| `STARTING_RESISTANCE` | 10,000,000 | Initial value, ohms |

After a resistor change the allowed step drops to 100 Ω for 64 samples, then
returns to 5000 Ω.

> **`GSR_smoothSample` is not called from `GSR_range`.** The live sample path
> uses only `GSR_smoothTransition`. This function converts to resistance, which
> the firmware does not do on the streaming path — samples go out raw
> ([SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §1).
> Whether anything calls it is listed below as unverified.

## 6. Converting to physical units

```c
uint32_t GSR_calcResistance(float gsrMv)
{
    return GSR_FEEDBACK_RESISTORS_OHMS[gsrActiveResistor]
           / (((gsrMv / 1000) / 0.5) - 1.0);
}
```

Rearranged: with `V` in volts and `Rf` the feedback resistor,

```
R_skin = Rf / ((V / 0.5) - 1)
```

The 0.5 is the amplifier's reference in volts. Conductance in microsiemens is
`1e6 / R_skin`.

A host must do this itself — the firmware transmits raw counts. Steps:

1. `range = word >> 14`, `adc = word & 0x3FFF`.
2. Convert `adc` to millivolts using the converter's reference and resolution.
3. Apply the formula with `GSR_FEEDBACK_RESISTORS_OHMS[range]`.

> **The formula has a singularity at exactly 0.5 V**, where the denominator is
> zero, and returns a negative resistance below it. The firmware does not guard
> against either. A host should clamp or reject readings at and below 0.5 V —
> they correspond to a skin resistance at or beyond the top of the selected
> range.

> **`GSR_calcResistance` uses `gsrActiveResistor`, the global**, not a range
> passed in. It is therefore only correct when called on the current sample,
> and cannot be used to reprocess a stored buffer.

## 7. Channel conflicts

GSR shares its ADC input with an internal ADC channel — `INTERNAL_ADC_1` on
Shimmer3, `INTERNAL_ADC_3` on Shimmer3R. Enabling both makes the firmware
silently disable the internal ADC channel
([SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §10.1).

Enabling skin temperature or the resistance amplifier forces that same internal
ADC channel **on**, which then collides with GSR.

On Shimmer3, GSR must be the **last analog channel** in the packet — the source
says so explicitly. See
[SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §4.1.

## Still unverified / not found in code

- ~~Whether `GSR_smoothSample` is ever called~~ — **it is not.** No caller in
  `log-and-stream-common`, `shimmer3-firmware` or `shimmer3r-firmware`. The
  resistance-domain smoothing of §5.2 is dead code; only `GSR_smoothTransition`
  runs.
- **`got_first_sample` and `last_resistance`.** Set by `GSR_initSmoothing` and
  used by `GSR_smoothSample`; their behaviour was not traced beyond that.
- **The ADC reference and resolution.** Needed to turn counts into millivolts
  in §6, and defined per platform. The 14-bit field width implies at most
  16383 counts full scale, but the reference voltage is not in this module.
- **The 0.5 V reference in `GSR_calcResistance`.** A literal in the formula
  with no named constant and no comment explaining its origin beyond "*uses op
  amp equation*".
- **The 40 ms versus 80 ms discrepancy** between the settling comment and
  `SETTLING_TIME`. Which is intended was not established; the constant is what
  runs.
- **Why `HW_RES_3M3` switches down at 1125 rather than a value derived from
  the band above it.** Every other lower bound sits above 1400; this one does
  not, and the comment gives only the resistance range.
