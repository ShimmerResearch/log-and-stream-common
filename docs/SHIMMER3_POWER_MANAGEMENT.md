# Shimmer3 / Shimmer3R Power Management

Which rails the firmware switches, when it sleeps, and what makes a device draw
more or less current. The battery *measurement* side is in
[SHIMMER3_BATTERY_AND_CHARGING.md](SHIMMER3_BATTERY_AND_CHARGING.md); this
document is about consumption.

> **Verified against** — the revisions these claims were read from.
>
> - **Firmware:** `log-and-stream-common` @ `f3cf73e` —
>   `Platform/platform_api.{h,c}`, `TaskList/shimmer_taskList.c`,
>   `log_and_stream_common.c`, `Configuration/shimmer_config.c`,
>   `Battery/shimmer_battery.{h,c}`, `Sensing/shimmer_sensing.c`,
>   `Comms/shimmer_sd_file_transfer.c`.
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4`;
>   `shimmer3r-firmware` @ `a8f105e5`.

> **Scope.** The shared module decides *when* rails go on and off and when the
> part may sleep; the actual low-power modes are platform code. This document
> covers the decisions, and is explicit about where the mechanism is elsewhere.

**Source references:**

| Layer | File |
|---|---|
| Sleep hook | `Platform/platform_api.{h,c}` — `platform_sleepWhenNoTask` |
| Idle detection | `TaskList/shimmer_taskList.c` |
| Rail control | `log_and_stream_externs.h` — `Board_set*Power` |
| Dock/USB transitions | `log_and_stream_common.c` |
| Auto-stop | `Battery/shimmer_battery.c` |

---

## 1. Switched rails

| Function | Controls |
|---|---|
| `Board_setSdPower(state)` | SD card rail |
| `Board_setExpansionBrdPower(state)` | Expansion board rail |
| `Board_enableSensingPower(SENSE_PWR_SENSING, state)` | Sensor rails (Shimmer3R) |

### 1.1 SD card

Powered on demand rather than continuously:

| Call site | Action |
|---|---|
| `ShimSdDataFile_fileInit` | On, if not already |
| `ShimConfig_loadSensorConfigAndCalib` | On, to read configuration |
| `ShimSdCfgFile_generate` / `_readSdConfiguration` | On, then off after |
| `LogAndStream_setupDock` path | Off |
| SD file transfer release | Off |

> **The card being powered down is a state the file-transfer path has to
> recover from explicitly.** A bare rail restore is not enough — the SDMMC
> peripheral retains state from before the card lost power, so a full bring-up
> is needed. See
> [SHIMMER3_SD_FILE_TRANSFER.md](SHIMMER3_SD_FILE_TRANSFER.md) §2.1. This is the
> mechanism behind a device that appeared dead after a second connection.

### 1.2 Expansion board

Gated by the `expansionBoardPower` configuration bit (byte 9 bit 0), applied at
the start of sensing:

```c
if (ShimConfig_isExpansionBoardPwrEnabled())
    Board_setExpansionBrdPower(1);
```

**Default is off.** A trial using an expansion board that needs power must set
the bit; the symptom of forgetting is a board that reads as absent or returns
zeros.

### 1.3 Sensor rails (Shimmer3R)

`Board_enableSensingPower(SENSE_PWR_SENSING, 1)` is called when sensing starts,
so the sensor supplies are only up while a trial runs.

## 2. Sleeping

`ShimTask_NORM_manage` sleeps whenever the task bitmask is empty:

```c
executingTask = ShimTask_popNext();
if (executingTask == TASK_NONE)
    platform_sleepWhenNoTask();
```

> **`platform_sleepWhenNoTask` is `PLATFORM_WEAK` with an empty default body.**
> A platform that does not override it never sleeps — the main loop spins. The
> failure is silent: nothing warns, and the only symptom is current
> consumption. If a board's idle current is unexpectedly high, confirm the
> override exists before looking anywhere else.

Waking from an interrupt requires the ISR to force the main context to run.
On Shimmer3R, `ShimTask_set` does this when called from an ISR:

```c
if (__get_IPSR() != 0) {
    HAL_PWR_DisableSleepOnExit();
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}
```

Without it the Cortex-M would return from the interrupt straight back to sleep
and the queued task would not run until something else woke the part.

> **`ShimTask_set` returns whether the device was idle when the task was
> queued** (`!taskList && !executingTask`). That return value is how a caller
> can tell it has just woken the device, and it is easy to discard by accident.

## 3. What keeps a device awake

Anything that keeps queuing tasks:

| Source | Cadence |
|---|---|
| Sample timer | The configured sampling rate |
| Battery read | 60 s undocked, 2 s docked |
| LED blink timer | Every 0.1 s |
| Bluetooth activity | Per packet |
| SD sync | Once per second while running |
| Dock/USB detection | Polled |

> **The LED blink timer runs continuously**, at 0.1 s, whether or not any LED is
> lit — the counters in `ShimLeds_incrementCounters` always advance. That sets a
> floor on how long the device can stay asleep, independent of sampling.

> **Docking multiplies the battery-read rate by thirty**, from 60 s to 2 s. That
> is deliberate — the charging display needs to be responsive — but it means
> docked idle behaviour is not representative of undocked idle behaviour.

## 4. Radio power

`ShimConfig_checkBtModeFromConfig` decides whether the radio should be on,
from three inputs: the `bluetoothDisable` configuration bit, the `syncEnable`
bit, and the EEPROM radio-enable bits.

```c
btSupportEnabled = configBytes->bluetoothDisable ? 0 : 1;
sdSyncEnabled    = btSupportEnabled && configBytes->syncEnable;
```

It stops the radio when it is powered but should not be, when the sync mode
disagrees with the current state, or when the EEPROM's BLE / classic enables
disagree with what is running. It starts the radio when normal log-and-stream
mode is enabled and the radio is off.

> **In SD-sync mode the radio is cycled per node**, not left on. The centre
> starts Bluetooth, attempts one node, and stops it again before the next —
> which is the dominant cost of a sync round and the reason a round takes
> minutes. See [SHIMMER3_SD_SYNC.md](SHIMMER3_SD_SYNC.md) §6.1.

> **`bluetoothDisable` is the only configuration route to a radio-off device.**
> The EEPROM enables select *which* radio, not whether one runs. See
> [SHIMMER3_EEPROM_MEMORY_MAP.md](SHIMMER3_EEPROM_MEMORY_MAP.md) §4.2.

## 5. Low-battery auto-stop

Gated on `lowBatteryAutoStop` (byte 218 bit 0), which is **off by default**.
Three low readings — not necessarily consecutive — below
`BATT_CUTOFF_3_65VOLTS` (2500 counts) latch a critical state and stop sensing.

At the undocked 60 s battery interval that takes at least two minutes to
trigger. Full detail in
[SHIMMER3_BATTERY_AND_CHARGING.md](SHIMMER3_BATTERY_AND_CHARGING.md) §4.

## 6. Sampling rate and consumption

The rate is a divider of 32768 Hz
([SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §3.1),
and the firmware refuses anything above **4096 Hz**.

Every sample queues `TASK_GATHER_DATA`, so the sampling rate sets how often the
device wakes and how much of its time it spends awake. Two secondary effects:

- **Buffer size** (byte 2) sets how many samples accumulate before a Bluetooth
  packet. Raising it reduces radio wake-ups at the cost of latency.
- **SD writes** happen when a 512-byte buffer fills, so a higher rate or more
  channels means more frequent writes.

> **Enabled channel count matters as much as rate.** A part that is enabled is
> powered and read every sample period, so disabling unused channels reduces
> both bus activity and rail load — not just packet size.

## 7. Measuring idle current

A caution from experience rather than from the source:

> **An attached debugger inflates the measurement, and the inflation persists
> until a power-on reset.** A device that has been connected to SWD can hold an
> elevated draw even after the debugger is detached; only a true power cycle
> clears it. Measure on a unit that has not been debugged since its last power
> cycle, or the number is meaningless.

> **Erased or invalid configuration changes idle behaviour.** Configuration
> validity is decided by the MAC bytes alone
> ([SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md)
> §11.1), so a partially erased configuration can pass validation and leave the
> device in an unexpected state rather than falling back to defaults.

## Still unverified / not found in code

- **The actual low-power modes used.** `platform_sleepWhenNoTask` is a weak
  hook; which STM32U5 mode (Sleep, Stop 0/1/2) or MSP430 LPM the platforms
  select was not read. Nothing in `common` constrains the choice.
- **Current consumption figures.** No numbers appear in the firmware. Nothing
  in this document quantifies consumption, only what drives it.
- **`SENSE_PWR_SENSING` and its siblings.** The enum is referenced from
  `ShimSens_startSensing` but its definition was not located in
  `log_and_stream_definitions.h`; it is presumably Shimmer3R platform code, and
  whether other `SENSE_PWR_*` domains exist is unknown.
- **Whether sensor rails are dropped when sensing stops.** The enable call at
  start was found; a matching disable was not traced.
- **Dock and undock debounce intervals.** `g_dock_usb_last_tick` and
  `time_newUnDockEvent` implement them; the thresholds were not extracted.
- **Whether the independent watchdog (`hiwdg`) affects sleep depth** on
  Shimmer3R. It is initialised; its interaction with low-power modes was not
  examined.
