# Shimmer3 / Shimmer3R LED Feedback

What the LEDs mean. Every state the firmware can display, the priority order
that decides which one wins when several apply, and the differences between the
two generations' very different LED hardware.

> **Verified against** — the revisions these claims were read from. A pinned
> commit is a citation, not a claim of currency.
>
> - **Firmware:** `log-and-stream-common` @ `e8bc116` —
>   `LEDs/shimmer_leds.{h,c}` (the whole state machine, the blink-timer
>   primitives, boot indication, `ShimLeds_phaseSync`),
>   `LEDs/shimmer_leds_phase.{h,c}` (frame and phase arithmetic, the four level
>   patterns in §3.3), `Battery/shimmer_battery.c`
>   (`ShimBatt_determineChargingLedState`,
>   `ShimBatt_determineUndockedLedState` — the battery colours the lower LED
>   displays), `log_and_stream_common.c` (`LogAndStream_blinkTimerCommon`,
>   `LogAndStream_dockedStateChange`, `LogAndStream_setBootStage`),
>   `Comms/shimmer_bt_uart.c` (the LED commands and the status byte),
>   `Configuration/shimmer_config.c` (Bluetooth disable), and
>   `Test/shimmer_test_leds_states.c` (the operational-state test).
> - **Platform firmware:** `shimmer3-firmware` @ `7d86034` — `main.c` (boot
>   stages, the Bluetooth baud ladder, the blink timer,
>   `triggerShimmerErrorState`), `hal_FactoryTest.c`, `hal_CAT24C16.c`.
>   `shimmer3r-firmware` @ `24a609a3` — `Core/Src/main.c` (boot stages, the
>   Bluetooth retries, `JumpToBootloaderIfRequired`), `Core/Src/tim.c`
>   (`platform_ledTickShift`, `ledBlinkTimerCallback`), `Core/Src/spi.c`,
>   `Core/Src/rtc.c`, `Shimmer_Driver/hal_Board.c` and `hal_FactoryTest.c`.
>   Both supply the `Board_led*` implementations.

> **How to read this document.** **S3** = Shimmer3 (MSP430); **S3R** =
> Shimmer3R (STM32U5). This document covers **LogAndStream** only.

**Source references:**

| Layer | File |
|---|---|
| State machine, priorities, timers | `LEDs/shimmer_leds.c` |
| Frame and phase arithmetic | `LEDs/shimmer_leds_phase.c` |
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

> **Shimmer3R can show colours Shimmer3 cannot.** The bootloader indication
> (§6) is purple, and the SPI fault trap (§7.3) lights both LEDs white. Neither
> has a Shimmer3 equivalent.

## 2. Blink timing

All blinking is derived from one periodic tick,
`SHIMMER_BLINK_TIMER_PERIOD_MS` (100 ms, `LEDs/shimmer_leds.h`), driving
`ShimLeds_incrementCounters`. It advances one counter, `blinkCntFrame`, through
the 600 tenths of a **60-second frame**. The two cycles the patterns use are
read from it:

| Counter | Wraps at | Period |
|---|---:|---|
| `blinkCntFrame` | 600 | 60 s |
| `blinkCnt20` = `blinkCntFrame % 20` | 20 | 2 s |
| `blinkCnt50` = `blinkCntFrame % 50` | 50 | 5 s |

60 s is a multiple of every pattern period in this document (0.2, 0.4, 1, 2,
3, 4 and 5 s), so every pattern is a function of `blinkCntFrame` alone and
repeats cleanly across the frame's wrap.

Five predicates are built on the counters:

| Predicate | Expression | True when |
|---|---|---|
| `ShimLeds_isBlinkTimerCnt200ms` | `blinkCnt20 % 2` | Every other tick — a 5 Hz square wave |
| `ShimLeds_isBlinkTimerCnt500ms` | `blinkCnt20 % 5 == 0` | Once every 0.5 s |
| `ShimLeds_isBlinkTimerCnt1s` | `blinkCnt20 % 10 == 0` | Once every 1 s |
| `ShimLeds_isBlinkTimerCnt2s` | `blinkCnt20 == 0` | Once every 2 s |
| `ShimLeds_isBlinkTimerCnt5s` | `blinkCnt50 == 0` | Once every 5 s |

> **These are not all the same shape, despite the naming.** `...200ms` is true
> for *half of all ticks*, every other one — it is a duty cycle. Code that
> lights the LED on true and clears it on false gives a 5 Hz flash, 0.1 s on and
> 0.1 s off. The bootloader indication (§6) instead toggles on each true tick,
> which gives 2.5 Hz. The other four are true for **one tick in N**. `...500ms`
> steps the boot sweep (§5), and `...2s` and `...5s` light the LED for a single
> 0.1 s pulse. `...1s` drives no LED at all: it only paces the auto-stop check
> in `LogAndStream_blinkTimerCommon`.

> **`...2s` and `...5s` are single-tick pulses used for a "still alive" blip.**
> Idle indications light the LED for 0.1 s every 2 s or every 5 s. That is
> intentionally brief and easy to miss.

### 2.1 Phase-locked to the real-world clock

With `LED_PHASE_LOCK_TO_RWC` set (`log_and_stream_definitions.h`, on by
default), the frame is tied to the real-world clock: `blinkCntFrame` is the
tenth of the current minute. Every sensor whose clock was set from the same
source shows the same frame, so sensors in the same state flash together,
the 5-second battery blip included.

Once a second the blink tick queues `TASK_LED_PHASE_SYNC`, the lowest-priority
task, which runs `ShimLeds_phaseSync`:

1. It reads the clock, and subtracts how long ago the blink tick fired
   (`platform_ledTickElapsedRtcTicks`). That gives the time of the tick itself,
   however long the task waited behind others.
2. If `blinkCntFrame` disagrees with the tenth that time falls in, it is
   corrected. That happens after the clock is set, and when an SD-sync offset
   first arrives.
3. If the tick fired more than about 5 ms from the middle of its tenth, the
   next tick is moved (`platform_ledTickShift`). Ticks are aimed mid-tenth so
   interrupt latency never pushes one across a boundary.

The sync's decision is `ShimLedsPhase_correction` (`LEDs/shimmer_leds_phase.c`).

| Platform | Blink timer | Correction |
|---|---|---|
| S3 | TB0 CCR3 on ACLK, the RTC's own crystal | Exact: CCR3 is moved by whole ticks |
| S3R | TIM6, 800 Hz from APB1, which runs from the MSI, independent of the LSE | In 1.25 ms steps (`ShimLedsPhase_counterTimerShift`): earlier steps the counter forward; later lengthens the next period through the ARR preload, and the tick interrupt restores it. Holds every tick in its tenth for a timer up to 3% fast or slow |

> **On S3R a delay must not step the counter back.** The sync runs just after
> the tick, when TIM6's counter is 0 or 1, so a counter step has almost nothing
> to step back over. Before that was fixed the delay did exactly that, clipped
> at 0, so a TIM6 running even slightly fast drifted early until its tick fell
> in the previous tenth. Step 2 then renumbered it into that tenth, and the next tick
> repeated the whole-second count. Every 50 ms of drift counted a second twice:
> 0.15% fast doubled about one second in 35. A slow TIM6 was always corrected,
> because stepping forward has room. The max_exp_len auto-stop counted those
> seconds, which is why it no longer does
> ([SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md),
> bytes 222-223). `test_leds_phase` runs the lock closed loop against a timer
> from 3% slow to 3% fast, and against the old clip.

**On an SD-sync node the LEDs follow the centre's clock**, not their own. Once
`rcFirstOffsetRxed` is set, the offset sync measured
([SHIMMER3_SD_SYNC.md](SHIMMER3_SD_SYNC.md) §5) is applied to the clock reading
the LEDs use. It is never applied to the clock itself, which logged timestamps
and the recorded offset both depend on. Until the first offset arrives a node
follows its own clock.

> **How closely sensors agree depends on how their clocks were set, not on this
> code.** Two sensors set over Bluetooth in turn differ by the latency of each
> set command. SD-sync nodes differ by the error in their measured offset, and
> by drift since the last sync round. Uncompensated drift of 20 ppm moves two
> sensors apart by about 70 ms an hour in the worst case, so a fleet set once
> drifts visibly apart over a day; re-setting the clocks on each connection
> pulls it back.

> **A clock set or a new offset is a step, not a stall.** The frame jumps to the
> right tenth on the next sync, so a pattern can skip or repeat part of a cycle
> once. Setting the clock repeatedly, as a host does while docked, causes
> nothing worse.

> **Shimmer3 before its clock is set follows time since boot**, so it is in step
> with nothing. Shimmer3R keeps its calendar across a reset but not a power
> cycle.

With `LED_PHASE_LOCK_TO_RWC` at 0 the frame free-runs from boot, as it did
before.

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

Upper green is on for 0.2 s and off for 0.2 s, read from the frame
(`ShimLedsPhase_isOnAlternate200ms`). Blue is not touched, so whatever it was
showing persists underneath.

### 3.3 Log-and-stream mode

Reached when Bluetooth is enabled and SD sync is not. Sub-states, in the order
tested:

| Condition | Appearance |
|---|---|
| Sensing, **streaming only** | Blue 1 s on, 1 s off → 0.5 Hz square wave. Green off |
| Sensing, **logging only, BT connected** | Green and blue alternate on a three-slot rotation, one slot green and two blue |
| Sensing, **logging only, not connected** | Green 1 s on, 1 s off → 0.5 Hz. Blue off |
| Sensing, **logging and streaming** | Green 1 s, off 1 s, blue 1 s, off 1 s, repeating |
| Sensing, neither ready | Both off |
| Not sensing, **BT connected** | Blue solid on, green off |
| Not sensing, **RN4678 link up but RFCOMM not open** (S3 only) | Blue flashing at 5 Hz, green off |
| Not sensing, **advertising / idle** | Green off; blue pulses for 0.1 s every 2 s if the radio is powered, and both are off if it is not |

> **"Connected and logging" is a three-slot rotation, not a 50/50 alternation.**
> The second of the frame, modulo 3, picks the slot, and only slot 2 shows
> green, so the pattern is blue, blue, green, repeating at 1-second slots. It
> reads as "mostly blue with a green heartbeat".

> **"Logging and streaming" inserts an off phase.** The second of the frame,
> modulo 4, gives green, off, blue, off at 1-second slots — distinguishably
> slower and gappier than the connected-and-logging pattern.

> **These four patterns used to be toggles**, so their phase depended on when
> the state was entered, and two sensors in the same state could be out of
> step. They are now levels read from the frame (`LEDs/shimmer_leds_phase.c`).
> The two blue toggles (RN4678 link up here, and connected in §3.4) remain
> toggles. Toggled once per 0.1 s tick, they flash at 5 Hz and look the same at
> any phase.

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
| BT powered and connected — to a host, or to another sensor for a sync round | Green forced **off**, blue toggled every tick: a 5 Hz flash |
| SD-sync node, sensing, that has not yet received its first sync offset | Blue on whenever green is off: green 1 s, blue 1 s, alternating |
| Otherwise | Quick double flash when the radio is powered — on at `blinkCnt20 == 12` and `== 14`, 0.1 s each, off otherwise. Nothing when it is not |

> **Blue can suppress green in this mode.** When a Bluetooth connection is
> established the code explicitly turns green off "to avoid visual clash", so a
> connected device in SD-sync mode shows no logging indication at all on the
> upper LED. That is deliberate, not a fault.

> **Each sync round shows as a blue burst.** A sync round is a Bluetooth
> connection like any other, so for as long as it lasts the upper LED flashes
> blue at 5 Hz and the logging green disappears. On a logging SD-sync sensor
> that burst is the visible sign that syncing is happening.

> **The "waiting for first sync" indication is inverted against green.** It
> lights blue only while green is off, producing green 1 s, blue 1 s with no
> off phase. That looks similar to the log-and-stream patterns of §3.3 but means
> something completely different. The distinguishing feature is that it stops
> once the first offset arrives.

> **With Bluetooth disabled this mode shows no blue at all.** Disabling
> Bluetooth powers the radio down (`ShimConfig_checkBtModeFromConfig`), and
> every blue branch above needs it powered. The upper LED is then a green blip
> every 2 s when idle, and green 1 s on, 1 s off while logging. An SD-sync
> sensor shows the same two green patterns. Green 1 s on, 1 s off is also
> identical, phase included, to §3.3's "logging only, not connected", so on its
> own it means "logging" and says nothing about which mode the sensor is in.

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

> **The host can override the lower LED until it clears it, or the device
> resets.** `toggleLedRedCmd`, set by the `TOGGLE_LED` command, sits above every
> error indication. Nothing in the running firmware clears it: only the host
> sending the command again, or a reset or power cycle, which zeroes
> `shimmerStatus` (`LogAndStream_init`). A device left in that state shows solid
> red and hides all battery and SD status. This is a common source of "the
> device says it has a flat battery" reports that turn out to be a host that
> never cleared the toggle. Docked, or on USB, it looks exactly like charging
> (§4.4).

### 4.1 Button pressed

Solid green — `LED_LWR_GREEN` on Shimmer3, PWM green on Shimmer3R. Red and
yellow are explicitly cleared.

### 4.2 Host-commanded red

Solid red. Green and yellow cleared.

The override is toggled by `TOGGLE_LED` (`0x06`, `toggleLedRedCmd ^= 1`) and,
short of a reset, is the only thing that changes it. It is reported back in the
status byte, **bit 7**, so a host can detect that it left the override armed.

`TOGGLE_LED` is the only host command that changes what the LEDs show. The two
others with LED in their names do not:

| Command | Opcode | What it does |
|---|---|---|
| `TOGGLE_LED_COMMAND` | `0x06` | Flips the override above |
| `SET_CHARGE_STATUS_LED_COMMAND` | `0x30` | NACKed. It was never implemented: there is nothing for it to set |
| `GET_CHARGE_STATUS_LED_COMMAND` | `0x32` | Returns the battery band (`batteryStatus.battStat`), not the state of an LED |

There is no command to blink a device to identify it, or to set either LED
directly. The commands are specified in
[SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md).

### 4.3 SD card error

Green flashing at 5 Hz — on during `...200ms`, off otherwise.

> **A green flash is the SD error, not a healthy indication.** The colour is
> counter-intuitive: green normally reads as "fine". The distinguishing feature
> is that it *flashes* at 5 Hz, where a healthy battery indication is a brief
> pulse every 5 seconds.

Requires all three of: undocked, a card fault (`sdBadFile` or `!sdInserted`),
and `sdErrorEnable` set in configuration byte 217 bit 0. With that bit clear
there is no card-error indication at all.

> **"Undocked" means out of the dock; USB does not count.** The test is
> `!docked` alone. A Shimmer3R on USB-C but out of the dock, with no card or a
> bad file, shows this green flash instead of its charging colour (§4.4).
> Docked, no card error is shown at all.

### 4.4 Battery and charging

The default state. What it shows depends on whether the device is drawing
power:

**Docked or USB connected** (`LogAndStream_isDockedOrUsbIn`; Shimmer3 has no
USB, so for it this means docked) — the charging state, continuously:

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

> **All-off while docked or on USB is "status unknown", not "no power".** The
> firmware deliberately shows nothing rather than guess. Every dock or USB
> change shows it for a moment: `LogAndStream_dockedStateChange` resets the
> charging status to unknown, and the colour returns at the next battery read.

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

`ShimLeds_controlDuringBoot` runs instead of the normal state machine while
`shimmerStatus.booting` is set. It has two modes.

**Progressing normally** — the *lower* LED steps through green, yellow and red
in a back-and-forth sweep, one colour at a time, changing every 0.5 s:

```
green → yellow → red → yellow → green → yellow → red → ...
```

The order is `bootLedOrder[3] = { LED_LWR_GREEN, LED_LWR_YELLOW, LED_LWR_RED }`
with the direction reversing at each end. All LEDs are cleared before each
step, so exactly one is lit at any moment and the upper LED is dark. The sweep
runs for the whole of the boot, not once.

**A stage is overrunning** — the sweep stops and an error pattern takes over.
The two generations draw the sensor-bus pattern differently:

| Boot stage | Shimmer3 | Shimmer3R | Usual cause |
|---|---|---|---|
| `BOOT_STAGE_I2C` past `BOOT_STAGE_TIMEOUT_MS_I2C` (1000 ms) | Red flashing at 5 Hz, 0.1 s on and 0.1 s off. Yellow is never lit | Yellow and red alternating every 0.1 s, with no off phase | Sensor bus. On Shimmer3R this stage also hands the SD card to the MCU (`LogAndStream_setupUndock`), so a slow card can cause it too |
| `BOOT_STAGE_BLUETOOTH_FAILURE` | Yellow flashing at 5 Hz, until reset | Effectively never shown (below) | The radio did not respond at any baud rate |
| `BOOT_STAGE_CONFIGURATION` past `BOOT_STAGE_TIMEOUT_MS_CONFIGURATION` (2000 ms) | Green flashing at 5 Hz | Green flashing at 5 Hz | Configuration and calibration load, which reads the SD card |

> **The timed patterns are stage overruns, not latched faults.** They show only
> while their stage runs long. `LogAndStream_setBootStage` zeroes the stage
> timer, so if the slow stage finishes, the sweep resumes at the next one. A
> slow but healthy step can therefore show an error pattern briefly.

> **The Bluetooth failure comes only after the radio has been retried at every
> baud rate,** and the sweep runs throughout. The two generations then part
> ways:
>
> - **Shimmer3** retries for about 66 s: twelve baud rates, 5 s each, with a
>   0.5 s restart between (`InitialiseBt`, `main.c`). It then sets
>   `BOOT_STAGE_BLUETOOTH_FAILURE` and halts in low-power mode. The blink timer
>   keeps running, so the yellow flash continues until the unit is reset.
> - **Shimmer3R** retries for about 27 s: five attempts of 5 s at four baud
>   rates, with a 0.5 s restart between. It then sets
>   `BOOT_STAGE_BLUETOOTH_FAILURE` and returns to the boot sequence, which sets
>   `BOOT_STAGE_CONFIGURATION` a few calls later (`Core/Src/main.c`). The yellow
>   pattern is overwritten before a blink tick is likely to draw it, and the
>   unit boots on without Bluetooth. Whether that is intended is an open
>   question (see the end of this document).

> **The boot SD-card error and the runtime SD-card error look identical** —
> both are green at 5 Hz, because they call the same function. Context is the
> only way to tell them apart: during boot, the upper LED is dark.

On Shimmer3R only, if `bslCheckTimeMs > 0` the bootloader indication (§6) pre-
empts everything above.

## 6. Bootloader (Shimmer3R only)

`ShimLeds_blinkSetLwrEnteringBslMode` sets the upper LED off and toggles the
lower LED **purple** at half brightness (`128, 0, 128`) on every `...200ms`
tick: 0.2 s on, 0.2 s off, 2.5 Hz.

This has the highest priority of any indication: it is tested first in
`ShimLeds_controlDuringBoot` and returns immediately.

It shows only while the firmware is timing the user button at power-up.
`JumpToBootloaderIfRequired` (`Core/Src/main.c`) runs as soon as the LED timers
start. With the button held it counts `bslCheckTimeMs` up in 100 ms steps, and
at 3 s (`BOOTLOADER_ENTRY_THRESHOLD_MS`) it jumps to the bootloader. Released
sooner, the count is zeroed and the unit boots normally. So purple lasts at
most 3 s.

Entering the bootloader from the dock shows no purple. `UART_PROP_ENTER_BOOTLOADER`
sets an alarm, and `TASK_JUMP_TO_BOOT_LOADER` jumps when it fires, without the
button check.

There is no Shimmer3 equivalent — the function is inside
`#if defined(SHIMMER3R)`.

## 7. Diagnostic, test and fault behaviour

### 7.1 Operational-state test

`ShimLeds_testOperationalStates` (`Test/shimmer_test_leds_states.c`,
`FACTORY_TEST_LED_STATES`) does not drive the LEDs itself. It sets
`shimmerStatus` flags for each state in turn — Bluetooth disabled, Bluetooth
enabled, SD sync, then configuring and clock not set — holds each for 5 s
(`SHIMMER_LEDS_TEST_INTERVAL_MS`), and lets the normal state machine draw it.
An operator therefore sees the real patterns of §3 and §4. It restores
`shimmerStatus` when it finishes. See
[SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md).

### 7.2 Colour walk

The per-LED check, which lights each LED and colour in turn, is the platform's
`led_test` (`hal_FactoryTest.c`, `FACTORY_TEST_LEDS`). It stops the blink timer
and drives the LEDs directly, then restarts the timer.

### 7.3 Patterns outside the state machine

| Platform | What you see | Cause |
|---|---|---|
| S3R | Both LEDs white, flashing at 5 Hz | `SpiSensing` (`Core/Src/spi.c`) found its sensor index past the end of its list. That should never happen, and the trap loops forever toggling `LED_ALL` with `HAL_Delay(100)` between. Whether it flashes or freezes depends on whether `HAL_Delay` can advance in the context it is reached from, which has not been checked |
| S3 | All LEDs off for 1 s, then lower yellow, lower red, upper blue and upper green lit one by one, 0.5 s apart, repeating | `triggerShimmerErrorState` (`main.c`): the RN42 radio reported firmware v6.30, which is refused. The unit does nothing else until reset |
| Both | The LEDs hold whatever they last showed | Nothing is updating them: the blink timer is stopped, or `timerBlinkEnabled` is clear. The colour walk (§7.2) does this deliberately |

Two more LED drivers exist but do not run in a release build:

- **`TEST_TASK_MONITOR`** (`log_and_stream_definitions.h`, 0 by default). With
  it on, a task stuck past its timeout turns off every LED but lower red, then
  alternates lower red and green, and finally resets the unit.
- **Shimmer3's RN4678 error pattern**, in the TB0 CCR3 interrupt (`main.c`):
  the lower LEDs against the upper ones at 5 Hz. It needs
  `RN4678_isErrorLedsEnabled()`, and the only call that sets that passes 0.

## 8. Quick reference

**Upper LED**

"Fast" is 5 Hz, 0.1 s on and 0.1 s off. "On USB" applies to Shimmer3R only.

| What you see | What it means |
|---|---|
| Cyan (S3R) or green and blue together (S3), fast | Real-world clock not set, not sensing |
| Green 0.2 s on, 0.2 s off | Configuring |
| Blue 1 s on, 1 s off | Streaming only |
| Green 1 s on, 1 s off | Logging with no Bluetooth connection. The same pattern whether Bluetooth is enabled, disabled or in SD-sync mode |
| Blue, blue, green, 1 s each, repeating | Logging, Bluetooth connected |
| Green, off, blue, off, 1 s each, repeating | Logging and streaming |
| Green 1 s, blue 1 s, alternating | SD-sync node logging, waiting for its first sync offset |
| Blue solid | Connected, not sensing |
| Blue fast, green off | SD-sync mode, connected — to a host, or for a sync round, which shows as a burst while logging. On Shimmer3 with an RN4678, also a link up with no RFCOMM channel open |
| Blue blip every 2 s | Bluetooth on, idle, not connected |
| Green blip every 2 s | Idle, with Bluetooth disabled or in SD-sync mode |
| Blue double flash every 2 s, beside the green | SD-sync mode, radio powered, not connected |
| Nothing | Bluetooth enabled but the radio powered down, not sensing; or sensing with neither logging nor streaming active; or booting (§5) |

**Lower LED**

| What you see | What it means |
|---|---|
| Green solid | Button held — or, docked or on USB, fully charged |
| Red solid, undocked | Host set the LED override (`TOGGLE_LED`) |
| Red solid, docked or on USB | Charging — or the host override, which looks the same |
| Yellow solid, docked or on USB | Charging suspended |
| Red fast, docked or on USB | Bad battery or charger error |
| Nothing, docked or on USB | Charging status not known yet. Normal for a moment after docking or plugging in |
| Green fast, out of the dock | SD card missing or bad file (on Shimmer3R, also while on USB) |
| Brief green / yellow / red blip every 5 s, undocked and not on USB | Battery high / mid / low |

**At power-on**

| What you see | What it means |
|---|---|
| Lower green, yellow, red, yellow, green …, 0.5 s per step; upper off | Booting normally. The sweep repeats until boot completes |
| Lower red fast (S3); lower yellow and red alternating, no off phase (S3R) | Sensor-bus stage taking over 1 s. On Shimmer3R it includes the SD card hand-over |
| Lower green fast; upper off | Configuration stage taking over 2 s, usually the SD card |
| Lower yellow fast (S3) | The radio did not respond after about 66 s of retries. Halted until reset |
| Lower purple, 0.2 s on and 0.2 s off, half brightness; upper off (S3R) | Button held at power-up. At 3 s the unit enters its bootloader |

**Both LEDs**

| What you see | What it means |
|---|---|
| White, fast (S3R) | SPI sensing fault trap (§7.3) |
| All off 1 s, then yellow, red, blue and green lit 0.5 s apart, repeating (S3) | RN42 radio firmware v6.30, refused (§7.3) |
| Stuck on one state | Nothing is updating the LEDs (§7.3) |

## Still unverified / not found in code

- **Whether the commented-out SD-sync blink logic is still intended to
  return.** The source carries a `TODO` about keeping the simplified version;
  no decision is recorded.
- **Whether Shimmer3R booting on without Bluetooth is intended.** After its
  retries fail, `BOOT_STAGE_BLUETOOTH_FAILURE` is overwritten by
  `BOOT_STAGE_CONFIGURATION` almost at once (§5), so the yellow pattern is
  probably never seen and the unit runs with no radio. If the pattern was meant
  to show, that is a firmware fault, not a documentation one. Whether the yellow
  is ever visible has not been checked on a bench.
- **Whether the SD error should treat USB like the dock.** The test is `!docked`
  only (§4.3); no decision is recorded on whether a Shimmer3R on USB should
  show its charging colour instead.
- **Whether the Shimmer3R SPI trap flashes or freezes** (§7.3), and whether the
  Shimmer3 RN42 v6.30 pattern draws as described: it is called while the radio's
  version reply is parsed, and whether anything else interrupts it has not been
  checked.
- **How long the boot stages take with a slow card**, docked or on USB, and so
  whether a healthy unit can briefly show a boot overrun pattern (§5).
- **Perceived colours**, above all the half-brightness purple, and how Shimmer3's
  green and blue in two packages read side by side.
