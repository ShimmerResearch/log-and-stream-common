# Shimmer3 / Shimmer3R LED Feedback

What the LEDs mean. Every state the firmware can display, the priority order
that decides which one wins when several apply, and the differences between the
two generations' very different LED hardware.

> **Verified against** — the revisions these claims were read from. A pinned
> commit is a citation, not a claim of currency.
>
> - **Firmware:** `log-and-stream-common` @ `f3cf73e` —
>   `LEDs/shimmer_leds.{h,c}` (the whole state machine, the blink-timer
>   primitives, boot indication), `Battery/shimmer_battery.c`
>   (`ShimBatt_determineChargingLedState`,
>   `ShimBatt_determineUndockedLedState` — the battery colours the lower LED
>   displays), `Test/shimmer_test_leds_states.c` (diagnostic sequences).
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4`;
>   `shimmer3r-firmware` @ `a8f105e5` — the `Board_led*` implementations.

> **How to read this document.** **S3** = Shimmer3 (MSP430); **S3R** =
> Shimmer3R (STM32U5). This document covers **LogAndStream** only.

**Source references:**

| Layer | File |
|---|---|
| State machine, priorities, timers | `LEDs/shimmer_leds.c` |
| LED bit masks | `LEDs/shimmer_leds.h` |
| Battery and charging colours | `Battery/shimmer_battery.c` |
| Diagnostic sequences | `Test/shimmer_test_leds_states.c` |
| Physical drive | platform `Board_ledOn` / `Board_ledLwrSetColourRgb` |

---

## 1. The two LED groups

Both generations present **two logical LEDs**, referred to throughout the
firmware as *upper* and *lower*. They carry different kinds of information and
are driven by two independent state machines:

| Group | Conveys |
|---|---|
| **Upper** | What the device is doing — idle, configuring, logging, streaming, connected, syncing |
| **Lower** | How the device is — button feedback, errors, battery and charging state |

That split is the single most useful thing to know: **the upper LED is
activity, the lower LED is health.**

### 1.1 Hardware differs completely between generations

**Shimmer3** has five discrete single-colour LEDs, addressed as a bit mask:

| Constant | Value | Group |
|---|---:|---|
| `LED_LWR_RED` | `0x01` | Lower |
| `LED_LWR_GREEN` | `0x02` | Lower |
| `LED_LWR_YELLOW` | `0x04` | Lower |
| `LED_UPR_GREEN` | `0x08` | Upper |
| `LED_UPR_BLUE` | `0x10` | Upper |
| `LED_ALL` | `0xFF` | Both |
| `LED_ALL_OFF` | `0x00` | — |

Combinations are produced by lighting more than one physical LED at once.

**Shimmer3R** has two RGB LEDs, driven by PWM through
`Board_ledUprSetColourRgb(r, g, b)` and `Board_ledLwrSetColourRgb(r, g, b)`
with `LED_PWM_OFF` (0) / `LED_PWM_ON` (255) per channel. The battery code uses
named 24-bit colours from `hal_Board.h` — `LED_RGB_RED` `0xFF0000`,
`LED_RGB_GREEN` `0x00FF00`, `LED_RGB_YELLOW` `0xFFFF00`, `LED_RGB_ALL_OFF` `0`.

> **The same firmware state can look different on the two generations.** The
> "RTC not set" state lights green **and** blue together on Shimmer3, which the
> eye reads as a blue-green mixture from two adjacent packages; on Shimmer3R it
> is a single clean cyan. Do not describe LED behaviour to a user without
> knowing which generation they hold.

> **Shimmer3R can show colours Shimmer3 cannot.** The BSL indication (§6) is
> purple, which has no Shimmer3 equivalent — the Shimmer3 branch of that
> function does not exist.

## 2. Blink timing

All blinking is derived from one periodic tick,
`SHIMMER_BLINK_TIMER_PERIOD_MS` (100 ms, `LEDs/shimmer_leds.h`), driving
`ShimLeds_incrementCounters`. Two
free-running counters advance **once every 0.1 s**:

| Counter | Wraps at | Period |
|---|---:|---|
| `blinkCnt20` | 20 | 2 s |
| `blinkCnt50` | 50 | 5 s |

Five predicates are built on them:

| Predicate | Expression | True when |
|---|---|---|
| `ShimLeds_isBlinkTimerCnt200ms` | `blinkCnt20 % 2` | Every other tick — a 5 Hz square wave |
| `ShimLeds_isBlinkTimerCnt500ms` | `blinkCnt20 % 5 == 0` | Once every 0.5 s |
| `ShimLeds_isBlinkTimerCnt1s` | `blinkCnt20 % 10 == 0` | Once every 1 s |
| `ShimLeds_isBlinkTimerCnt2s` | `blinkCnt20 == 0` | Once every 2 s |
| `ShimLeds_isBlinkTimerCnt5s` | `blinkCnt50 == 0` | Once every 5 s |

> **These are not all the same shape, despite the naming.** `...200ms` is true
> for *half of all ticks* — it is a duty cycle, and code using it lights the LED
> on true and clears it on false, giving a 5 Hz flash. The other four are true
> for **one tick in N**, so code using them either toggles (giving a square wave
> at half that rate) or turns the LED on for a single 0.1 s pulse. A
> `...1s` toggle produces a **2-second** period, not a 1-second one.

> **`...2s` and `...5s` are single-tick pulses used for a "still alive" blip.**
> Idle indications light the LED for 0.1 s every 2 s or every 5 s. That is
> intentionally brief and easy to miss.

## 3. Upper LED — activity

`ShimLeds_blinkSetUprState` picks exactly one branch, **first match wins**:

| Priority | Condition | State |
|---:|---|---|
| 1 | `rwcErrorFlash && !sensing` | RTC not set (§3.1) |
| 2 | `configuring` | Configuring (§3.2) |
| 3 | `btSupportEnabled && !sdSyncEnabled` | Log-and-stream mode (§3.3) |
| 4 | *otherwise* | Logging / SD-sync mode (§3.4) |

> **The RTC warning is suppressed while sensing.** It is a pre-trial warning
> only. Once logging starts, the upper LED reverts to showing activity even
> though the clock is still unset, so the resulting file's timestamps remain
> wrong with no ongoing indication.

### 3.1 RTC not set

| Platform | Appearance |
|---|---|
| S3 | Upper green + upper blue together, flashing at 5 Hz |
| S3R | Cyan, flashing at 5 Hz |

Set and cleared through `ShimLeds_setRtcErrorFlash`. Gated by the
`rtcErrorEnable` configuration bit (byte 217 bit 4).

### 3.2 Configuring

Upper green toggles on every `...200ms` tick, so it alternates at 5 Hz. Blue is
not touched, so whatever it was showing persists underneath.

### 3.3 Log-and-stream mode

Reached when Bluetooth is enabled and SD sync is not. Sub-states, in the order
tested:

| Condition | Appearance |
|---|---|
| Sensing, **streaming only** | Blue toggles on each `...1s` tick → 0.5 Hz square wave. Green off |
| Sensing, **logging only, BT connected** | Green and blue alternate on a three-slot rotation, one slot green and two blue |
| Sensing, **logging only, not connected** | Green toggles on each `...1s` tick → 0.5 Hz. Blue off |
| Sensing, **logging and streaming** | Green and blue alternate with an off phase between: off → green → off → blue → … |
| Sensing, neither ready | Both off |
| Not sensing, **BT connected** | Blue solid on, green off |
| Not sensing, **RN4678 link up but RFCOMM not open** (S3 only) | Blue toggles, green off |
| Not sensing, **advertising / idle** | Green off; blue pulses for 0.1 s every 2 s if Bluetooth is powered |

> **"Connected and logging" is a three-slot rotation, not a 50/50 alternation.**
> `lastLedToggleCnt` counts 0, 1, 2 and only slot 2 shows green — so the pattern
> is blue, blue, green, repeating at 1-second slots. It reads as "mostly blue
> with a green heartbeat".

> **"Logging and streaming" inserts an off phase.** The code turns both off if
> either is on, otherwise lights one. So the sequence is
> off, green, off, blue, off, green, … at 1-second slots — distinguishably
> slower and gappier than the connected-and-logging pattern.

### 3.4 Logging / SD-sync mode

Reached when Bluetooth is disabled, or SD sync is enabled. Green and blue are
decided **independently** here, unlike §3.3.

**Green** — the activity indication:

| Condition | Appearance |
|---|---|
| Sensing | On for `blinkCnt20 < 10`, off otherwise → 1 s on, 1 s off, a 0.5 Hz square wave |
| Not sensing | On for one 0.1 s tick every 2 s |

**Blue** — the sync indication, first match wins:

| Condition | Appearance |
|---|---|
| BT powered and connected | Green forced **off**, blue toggles |
| Node that has not yet received its first sync offset, while sensing | Blue on, except when green is on (to avoid a colour clash) |
| Otherwise | Quick double flash when BT is powered — on at `blinkCnt20 == 12` and `== 14`, off otherwise |

> **Blue can suppress green in this mode.** When a Bluetooth connection is
> established the code explicitly turns green off "to avoid visual clash", so a
> connected device in SD-sync mode shows no logging indication at all on the
> upper LED. That is deliberate, not a fault.

> **The "waiting for first sync" indication is inverted against green.** It
> lights blue only while green is off, producing an alternating green/blue that
> looks similar to the log-and-stream patterns of §3.3 but means something
> completely different. The distinguishing feature is that it stops once the
> first offset arrives.

The source retains the original SD-sync blink logic commented out beneath the
current implementation, with a `TODO` about which to keep. The behaviour
documented here is the live path.

## 4. Lower LED — health

`ShimLeds_blinkSetLwrState` picks exactly one branch, **first match wins**:

| Priority | Condition | State |
|---:|---|---|
| 1 | `buttonPressed` | Solid green (§4.1) |
| 2 | `toggleLedRedCmd` | Solid red (§4.2) |
| 3 | Undocked **and** (bad SD file or no card) **and** `sdErrorEnable` | SD error flash (§4.3) |
| 4 | *otherwise* | Battery / charging status (§4.4) |

> **Button feedback outranks every error and the battery display.** While the
> button is held the lower LED is solid green regardless of a missing SD card,
> a bad file, or a critically low battery. Release the button before reading
> the lower LED for diagnosis.

> **The host can override the lower LED indefinitely.** `toggleLedRedCmd`, set
> by the `TOGGLE_LED` command, sits above every error indication and is never
> cleared by the firmware — only by the host sending the command again. A
> device left in that state shows solid red and hides all battery and SD
> status. This is a common source of "the device says it has a flat battery"
> reports that turn out to be a host that never cleared the toggle.

### 4.1 Button pressed

Solid green — `LED_LWR_GREEN` on Shimmer3, PWM green on Shimmer3R. Red and
yellow are explicitly cleared.

### 4.2 Host-commanded red

Solid red. Green and yellow cleared.

The override is toggled by `TOGGLE_LED` (`toggleLedRedCmd ^= 1`) and is the
only thing that changes it. It is reported back in the status byte, **bit 7**,
so a host can detect that it left the override armed.

### 4.3 SD card error

Green flashing at 5 Hz — on during `...200ms`, off otherwise.

> **A green flash is the SD error, not a healthy indication.** The colour is
> counter-intuitive: green normally reads as "fine". The distinguishing feature
> is that it *flashes* at 5 Hz, where a healthy battery indication is a brief
> pulse every 5 seconds.

Requires all three of: undocked, a card fault (`sdBadFile` or `!sdInserted`),
and `sdErrorEnable` set in configuration byte 217 bit 0. With that bit clear
there is no card-error indication at all.

### 4.4 Battery and charging

The default state. What it shows depends on whether the device is drawing
power:

**Docked or USB connected** — the charging state, continuously:

| Charging status | Colour | Flashing |
|---|---|---|
| `CHARGING_STATUS_CHECKING` | Red | No |
| `CHARGING_STATUS_CHARGING` | Red | No |
| `CHARGING_STATUS_FULLY_CHARGED` | Green | No |
| `CHARGING_STATUS_SUSPENDED` | Yellow | No |
| `CHARGING_STATUS_BAD_BATTERY` | Red | **Yes, 5 Hz** |
| `CHARGING_STATUS_ERROR` | Red | **Yes, 5 Hz** |
| Unknown | All off | — |

> **Steady red means charging; flashing red means the battery is bad.** Both
> are red and the only difference is the flash, set by `battStatLedFlash`. A
> steady red on the dock is normal and expected.

> **All-off while docked is "status unknown", not "no power".** The firmware
> deliberately shows nothing rather than guess.

**Undocked** — the charge band, as a 0.1 s pulse every 5 seconds:

| Band | Colour |
|---|---|
| `BATT_HIGH` | Green |
| `BATT_MID` | Yellow |
| `BATT_LOW` | Red |
| Unrecognised | Red |

Thresholds and the hysteresis between bands are in
[SHIMMER3_BATTERY_AND_CHARGING.md](SHIMMER3_BATTERY_AND_CHARGING.md).

> **Undocked battery indication is easy to miss entirely** — one tenth of a
> second in every five. If a user reports "no LEDs at all", have them watch for
> at least six seconds before concluding the device is dead.

## 5. Boot indication

`ShimLeds_controlDuringBoot` runs instead of the normal state machine until
boot completes. It has two modes.

**Progressing normally** — the three *lower* LEDs cycle in a back-and-forth
sweep, one at a time, changing every 0.5 s:

```
green → yellow → red → yellow → green → yellow → red → ...
```

The order is `bootLedOrder[3] = { LED_LWR_GREEN, LED_LWR_YELLOW, LED_LWR_RED }`
with the direction reversing at each end. All LEDs are cleared before each
step, so exactly one is lit at any moment.

**A stage has timed out** — the sweep stops and a fixed error pattern takes
over:

| Boot stage | Pattern | Meaning |
|---|---|---|
| `BOOT_STAGE_I2C` past `BOOT_STAGE_TIMEOUT_MS_I2C` (1000 ms) | Alternating red / yellow at 5 Hz | Sensor bus fault |
| `BOOT_STAGE_BLUETOOTH_FAILURE` | Yellow flashing at 5 Hz | Bluetooth fault |
| `BOOT_STAGE_CONFIGURATION` past `BOOT_STAGE_TIMEOUT_MS_CONFIGURATION` (2000 ms) | Green flashing at 5 Hz | SD card fault |

> **The Bluetooth error has no timeout condition.** It triggers on the stage
> value alone, whereas the other two require both the stage *and* an elapsed
> time. A Bluetooth failure therefore shows immediately.

> **The boot SD-card error and the runtime SD-card error look identical** —
> both are green at 5 Hz, because they call the same function. Context is the
> only way to tell them apart: during boot, the upper LED is dark.

On Shimmer3R only, if `bslCheckTimeMs > 0` the bootloader indication (§6) pre-
empts everything above.

## 6. Bootloader (Shimmer3R only)

`ShimLeds_blinkSetLwrEnteringBslMode` sets the upper LED off and toggles the
lower LED **purple** (`128, 0, 128`) on every `...200ms` tick.

This has the highest priority of any indication: it is tested first in
`ShimLeds_controlDuringBoot` and returns immediately.

There is no Shimmer3 equivalent — the function is inside
`#if defined(SHIMMER3R)`.

## 7. Diagnostic and test behaviour

`Test/shimmer_test_leds_states.c` drives the LEDs through fixed sequences for
the factory self-test, bypassing the state machine entirely so an operator can
confirm each LED and colour works. See
[SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md).

## 8. Quick reference

**Upper LED**

| What you see | What it means |
|---|---|
| Cyan / green+blue, fast flash | Real-world clock not set, not yet logging |
| Green, fast toggle | Configuring |
| Blue, slow toggle | Streaming only |
| Green, slow toggle | Logging only, no Bluetooth connection |
| Blue, blue, green repeating | Logging, Bluetooth connected |
| Off, green, off, blue repeating | Logging and streaming |
| Blue solid | Connected, not sensing |
| Blue blip every 2 s | Advertising, idle |
| Green 1 s on / 1 s off | Logging, SD-sync mode |
| Green blip every 2 s | Idle, SD-sync mode |
| Blue double-flash every 2 s | SD-sync mode, Bluetooth powered, not connected |
| Nothing | Bluetooth off and not sensing, or an override is active |

**Lower LED**

| What you see | What it means |
|---|---|
| Green solid | Button held |
| Red solid, undocked | Host set the LED override |
| Red solid, docked | Charging |
| Green solid, docked | Fully charged |
| Yellow solid, docked | Charging suspended |
| Red fast flash, docked | Bad battery or charger error |
| Green fast flash | SD card missing or bad file |
| Red / yellow alternating fast | Sensor bus fault at boot |
| Yellow fast flash | Bluetooth fault at boot |
| Purple fast flash (S3R) | Entering bootloader |
| One green / yellow / red sweep | Booting normally |
| Brief green / yellow / red blip every 5 s | Battery high / mid / low, undocked |

## Still unverified / not found in code

- **Whether the commented-out SD-sync blink logic is still intended to
  return.** The source carries a `TODO` about keeping the simplified version;
  no decision is recorded.
