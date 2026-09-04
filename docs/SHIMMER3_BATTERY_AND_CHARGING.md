# Shimmer3 / Shimmer3R Battery and Charging

How the firmware measures the battery, classifies its charge and charging
state, reports both to a host, drives the status LED, and stops a trial when
the cell is nearly flat.

> **Verified against** — the revisions these claims were read from. A pinned
> commit is a citation, not a claim of currency.
>
> - **Firmware:** `log-and-stream-common` @ `f3cf73e` —
>   `Battery/shimmer_battery.{h,c}` in full.
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4`;
>   `shimmer3r-firmware` @ `a8f105e5` — the ADC sampling and charger-chip
>   pin reads that feed `ShimBatt_updateStatus`.

> **How to read this document.** **S3** = Shimmer3 (MSP430); **S3R** =
> Shimmer3R (STM32U5). LogAndStream only.

**Source references:**

| Layer | File |
|---|---|
| Thresholds, state enums, the status struct | `Battery/shimmer_battery.h` |
| Classification, LED colours, auto-stop | `Battery/shimmer_battery.c` |
| LED priority and display | [SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) §4.4 |
| Reporting to a host | [SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md) |
| Configuration bits | [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §8 |

---

## 1. What is measured

Three inputs, all supplied by the platform layer to `ShimBatt_updateStatus`:

| Input | Meaning |
|---|---|
| `adc_battVal` | Raw ADC counts from the battery divider |
| `battValMV` | The same reading converted to millivolts |
| `lm3658sdStat1`, `lm3658sdStat2` | Two charger-chip status pins |

These are packed into `BattStatusRaw`, a three-byte union:

| Bytes | Field |
|---|---|
| 0-1 | `adcBattVal`, `uint16` |
| 2 | bit 7 `STAT2`, bit 6 `STAT1`, bits 5-0 unused |

**Those three bytes are what a host receives** in the battery status response
and over the dock — the raw counts and the raw charger pins, not the derived
classification. A host that wants the band or the charging state must apply the
rules below itself, or read the derived LED colour.

> **The classification thresholds are in raw ADC counts, not millivolts.** Only
> the two error bounds (§3) are expressed in mV. Comparing a stored threshold
> against a millivolt reading will be wrong by whatever the divider and
> reference happen to be.

On Shimmer3 the platform conversion is, from `adc.c`:

```c
battValMV = (((uint32_t) raw * 3000) >> 12) * 2;   /* 3.0 V ref, 12-bit, x2 divider */
```

so counts convert at about 1.465 mV each, and the millivolt column in the
tables below is derived with that integer arithmetic. On Shimmer3R the MCU's
battery channel is *internally divided by 4* (`hal_adc.c`), so the same count
does not mean the same voltage on the two generations.

## 2. State of charge

Three bands, reported through `battStat`:

| Constant | Value |
|---|---:|
| `BATT_LOW` | `0x01` |
| `BATT_MID` | `0x02` |
| `BATT_HIGH` | `0x04` |

### 2.1 Thresholds with hysteresis

`ShimBatt_rankBattUndockedVoltage` uses **different thresholds depending on the
band it is currently in**, which is what gives the display its hysteresis:

| Constant | Counts | ≈ mV (Shimmer3) |
|---|---:|---:|
| `BATT_LOW_MAX` | 2618 | 3834 |
| `BATT_MID_MIN` | 2568 | 3760 |
| `BATT_MID_MAX` | 2767 | 4052 |
| `BATT_HIGH_MIN` | 2717 | 3978 |

Currently `BATT_MID`:

| Reading | New band |
|---|---|
| `< BATT_MID_MIN` (2568) | `BATT_LOW` |
| `< BATT_MID_MAX` (2767) | `BATT_MID` |
| otherwise | `BATT_HIGH` |

Currently `BATT_LOW`:

| Reading | New band |
|---|---|
| `< BATT_LOW_MAX` (2618) | `BATT_LOW` |
| `< BATT_MID_MAX` (2767) | `BATT_MID` |
| otherwise | `BATT_HIGH` |

Currently `BATT_HIGH`:

| Reading | New band |
|---|---|
| `< BATT_MID_MIN` (2568) | `BATT_LOW` |
| `< BATT_HIGH_MIN` (2717) | `BATT_MID` |
| otherwise | `BATT_HIGH` |

The overlaps are the point: leaving `BATT_LOW` upward requires 2618, but
entering it from `BATT_MID` requires dropping below 2568 — a 50-count dead
band that stops the indication flickering at a boundary.

> **A reading between 2568 and 2618 is ambiguous by design** — it can be either
> `LOW` or `MID` depending on where the device came from. A host cannot
> reproduce the band from a single raw reading without also knowing the previous
> band. This is why the raw counts, not the band, are what gets transmitted.

> **The hysteresis is asymmetric between the upper and lower boundaries.** The
> `LOW`/`MID` band has a 50-count overlap and the `MID`/`HIGH` band also 50
> counts, but the `HIGH` branch tests `BATT_MID_MIN` for the drop to `LOW`
> rather than `BATT_LOW_MAX` — so falling from `HIGH` straight to `LOW` uses a
> different threshold than falling from `MID` to `LOW`. The two paths agree in
> this case because both use 2568, but the code does not share the constant.

### 2.2 Initial state

`ShimBatt_resetBatteryUndockedStatus` sets the band to `BATT_MID` and
recomputes the LED. Starting mid-band means the first reading resolves cleanly
in either direction rather than being biased by an arbitrary starting point.

## 3. Charging state

`ShimBatt_rankBattChargingStatus` maps the two charger-chip status pins to a
`chargingStatus_t`.

The pins are read as one byte, `STAT2` in bit 7 and `STAT1` in bit 6, and the
chip encodes its state in the pair — **a low pin means the indicator is on**:

| Byte | Constant | STAT2 | STAT1 |
|---:|---|---|---|
| `0xC0` | `CHRG_CHIP_STATUS_SUSPENDED` | high (off) | high (off) |
| `0x80` | `CHRG_CHIP_STATUS_PRECONDITIONING` | high (off) | low (on) |
| `0x40` | `CHRG_CHIP_STATUS_FULLY_CHARGED` | low (on) | high (off) |
| `0x00` | `CHRG_CHIP_STATUS_BAD_BATTERY` | low (on) | low (on) |
| `0xFF` | `CHRG_CHIP_STATUS_UNKNOWN` | — | sentinel, not a pin state |

Mapping to the reported status:

| Chip byte | `chargingStatus_t` |
|---|---|
| `CHRG_CHIP_STATUS_SUSPENDED` | `CHARGING_STATUS_SUSPENDED` |
| `CHRG_CHIP_STATUS_FULLY_CHARGED` | `CHARGING_STATUS_FULLY_CHARGED` |
| `CHRG_CHIP_STATUS_PRECONDITIONING` | `CHARGING_STATUS_CHARGING` |
| `CHRG_CHIP_STATUS_BAD_BATTERY` | `CHARGING_STATUS_BAD_BATTERY` |
| `CHRG_CHIP_STATUS_UNKNOWN` | `CHARGING_STATUS_UNKNOWN` |
| anything else | `CHARGING_STATUS_ERROR` |

> **"Preconditioning" is reported as "charging".** The chip distinguishes
> trickle-charging a deeply discharged cell from normal charging; the firmware
> does not. Both show as `CHARGING_STATUS_CHARGING` and a steady red LED.

> **`CHRG_CHIP_STATUS_UNKNOWN` is `0xFF`, which is not a reachable pin
> combination** — bits 5-0 are unused and would read zero. It is a sentinel the
> firmware writes itself, in `ShimBatt_resetBatteryChargingStatus`, to mean "not
> yet determined".

### 3.1 The voltage override

Before consulting the pins at all:

```c
if (battValMV > BATTERY_ERROR_VOLTAGE_MAX)   // 4500 mV
    battChargingStatus = CHARGING_STATUS_CHECKING;
```

An implausibly high reading is treated as the measurement not having settled,
and reported as `CHARGING_STATUS_CHECKING` — which the LED shows as red, the
same as charging.

> **`BATTERY_ERROR_VOLTAGE_MIN` (3200 mV) is defined but never used.** No code
> in the shared module reads it. There is no corresponding low-voltage override,
> so an implausibly *low* reading passes through to the pin-based
> classification unchallenged.

### 3.2 A bad-battery reading also forces the chip byte

`ShimBatt_updateStatus` can set `battStatusRaw.rawBytes[2]` to
`CHRG_CHIP_STATUS_BAD_BATTERY` before ranking, so the transmitted raw byte
reflects that judgement rather than only the pins.

## 4. Low-battery protection

Independent of the band display, and gated on the `lowBatteryAutoStop`
configuration bit (byte 218, bit 0).

```c
if (lowBatteryAutoStop && adcBattVal < BATT_CUTOFF_3_65VOLTS)   // 2500 counts
{
    ShimBatt_incrementBatteryCriticalCount();
    if (ShimBatt_checkIfBatteryCritical() && sensing)
        ShimTask_setStopSensing();
}
```

`BATT_CUTOFF_3_65VOLTS` is **2500 counts**, commented as approximately 3.65 V
and roughly 10 % remaining — the Shimmer3 formula gives 3662 mV.

`ShimBatt_checkIfBatteryCritical` latches:

```c
if (battCriticalCount > 2)
    battCritical = 1;
return battCritical;
```

So it takes **three** consecutive low readings to trip — the count must exceed
2, and each call to the update path increments it once.

> **`battCritical` latches; the voltage recovering does not clear it, docking
> does.** Once tripped, `checkIfBatteryCritical` keeps returning 1 regardless of
> the current voltage. The latch is cleared in two places in
> `log_and_stream_common.c`: the count is reset when the device is docked
> (`LogAndStream_setupDock`, commented "to allow logging to begin again if
> auto-stop on low-power is enabled"), and `battCritical` itself is cleared on a
> new undock event. A dock cycle therefore resets the protection; nothing else
> does.

> **The count is never decremented on a good reading.** It only increments,
> inside the `< 2500` branch. Three low readings spread across an entire trial
> are therefore enough to trip it, not three in a row. The word "consecutive"
> would be wrong.

> **Auto-stop is off by default.** `ShimConfig_setDefaultConfig` does not set
> `lowBatteryAutoStop`, so a factory-defaulted device logs until the hardware
> cuts out.

### 4.1 Sampling interval

| Constant | Value |
|---|---:|
| `BATT_INTERVAL_SECS_UNDOCKED` | 60 |
| `BATT_INTERVAL_SECS_DOCKED` | 2 |

Shimmer3 additionally derives tick counts at 32768 Hz
(`BATT_INTERVAL_TICKS_UNDOCKED`, `BATT_INTERVAL_TICKS_DOCKED`); Shimmer3R uses
the seconds values directly.

> **At the undocked interval, three low readings take at least two minutes.**
> The auto-stop cannot react faster than that — 60 s between samples and three
> samples needed. A trial does not stop the instant the cell crosses 3.65 V.

## 5. Indication and reporting

The lower LED shows battery and charging state whenever nothing of higher
priority is displayed. The colour mapping, the flash behaviour, and the full
priority order are in
[SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) §4.

In brief: docked shows the charging state continuously; undocked shows the
charge band as a 0.1 s pulse every 5 seconds.

`battStatLedFlash` is set **only** for `CHARGING_STATUS_BAD_BATTERY` and
`CHARGING_STATUS_ERROR`, and is cleared at the top of
`ShimBatt_determineChargingLedState` on every call.

### 5.1 Reporting to a host

The three `BattStatusRaw` bytes are what travel over Bluetooth and the dock.
See
[SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md)
for the command and reply framing.

A host receives raw counts and raw charger pins. To present a charge band it
must implement §2 itself, including keeping the previous band for the
hysteresis — or accept a single-reading approximation that will disagree with
the device's own LED near a boundary.

> **There is no command that returns the firmware's derived
> `chargingStatus_t`.** The mapping in §3 has to be reimplemented host-side
> from the raw byte.

## 6. Diagnosing a suspicious reading

In the order to check:

1. **Solid red undocked, with no 5-second pulse** — almost always the host
   `TOGGLE_LED` override, not the battery. It hides the battery indication
   entirely and is reported in status-byte bit 7
   ([SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) §4.2).
2. **The band disagrees with a voltage you measured** — the thresholds are raw
   counts, not millivolts, and the band depends on the previous band (§2.1).
   Convert with `mV = raw × 3000 / 4096 × 2` before comparing.
3. **Logging stops on a battery that reads fine** — the auto-stop latch (§4). It
   trips on any three low readings across a trial, not three consecutive, and
   only a dock cycle clears it.
4. **Steady red on the dock** — charging, not a fault. *Flashing* red is the fault.
5. **Nothing lit on the dock** — charger status unknown, deliberately shown as
   nothing rather than guessed.

## Still unverified / not found in code

- ~~The Shimmer3R count-to-millivolt conversion~~ — resolved
  (`saveBatteryVoltageAndUpdateStatus`, `hal_adc.c`):
  `mV = raw × VREF_EXTERNAL_SUPPLY_MV / 4095 × 2`, 12-bit, with
  `VREF_EXTERNAL_SUPPLY_MV` = **3000** on product hardware (3300 only under
  `S3R_NUCLEO`). That is within 0.03 % of the Shimmer3 formula
  (`raw × 3000 / 4096 × 2`), so the millivolt column in §2 applies to both
  platforms: 2500 counts ≈ 3663 mV. The `/4` divider belongs to the *MCU*
  `VBAT` debug channel, not to the battery measurement.
- **Which charger part is fitted on which board.** The status-pin naming
  (`lm3658sdStat1` / `Stat2`) points at the LM3658SD, but board-to-charger
  mapping lives in the platform repositories and the hardware documentation.
- ~~`BATTERY_ERROR_VOLTAGE_MIN`~~ — resolved: it is used, in
  `ShimBatt_updateStatus` (`Battery/shimmer_battery.c`). When the charger
  reports `SUSPENDED` and the measured voltage is at or below 3200 mV, the raw
  status byte is overridden to `BAD_BATTERY` before the ranking runs.
- ~~Whether `battStat` is transmitted anywhere~~ — resolved: yes, once.
  `GET_CHARGE_STATUS_LED_COMMAND` answers `CHARGE_STATUS_LED_RESPONSE` followed
  by the single `batteryStatus.battStat` byte (`Comms/shimmer_bt_uart.c`). The
  richer reports carry `battStatusRaw`.
