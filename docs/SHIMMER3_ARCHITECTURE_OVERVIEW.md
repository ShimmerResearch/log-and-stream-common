# Shimmer3 / Shimmer3R Firmware Architecture

How LogAndStream is put together: the shared module, the platform boundary, the
boot sequence, and the cooperative task loop that everything runs on.

This is the entry point for someone new to the codebase, and the document to
read before adding a subsystem.

> **Verified against** — the revisions these claims were read from. A pinned
> commit is a citation, not a claim of currency.
>
> - **Firmware:** `log-and-stream-common` @ `f3cf73e` —
>   `log_and_stream_common.c`, `log_and_stream_definitions.h`,
>   `log_and_stream_externs.h`, `TaskList/shimmer_taskList.{h,c}`,
>   `Platform/platform_api.c`, `Sensing/shimmer_sensing.c`.
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4`;
>   `shimmer3r-firmware` @ `a8f105e5`.

> **How to read this document.** **S3** = Shimmer3 (MSP430F5437A, Code
> Composer Studio); **S3R** = Shimmer3R (STM32U5, STM32CubeIDE). LogAndStream
> only.

---

## 1. Repository structure

LogAndStream is **one shared module plus two platform applications**:

```
shimmer3-firmware/                     shimmer3r-firmware/
  LogAndStream_Shimmer3/                 LogAndStream_Shimmer3R/
    adc.c  i2c.c  spi.c  main.c            Core/Src/  Shimmer_Driver/
    Shimmer_Driver/                        FATFS/  USBX/  Middlewares/
    log-and-stream-common/  <───── same submodule ─────>  log-and-stream-common/
```

`log-and-stream-common` is a git submodule pinned independently by each
platform repository. It holds everything that is not MCU-specific: the
protocol, configuration, calibration, SD handling, sync, LEDs, battery logic
and the task list.

> **The submodule pins can differ between the two platform repositories.** A
> change to `common` reaches Shimmer3 and Shimmer3R only when each platform's
> pin is bumped, so the two can be running different versions of the shared
> code at the same time. Always check both pins before concluding a behaviour
> is or is not present on a given generation.

> **Anything in `common` must compile for both platforms.** Every change needs
> building on Shimmer3 *and* Shimmer3R — see
> [SHIMMER3_BUILD_AND_PROGRAMMING.md](SHIMMER3_BUILD_AND_PROGRAMMING.md). A
> change guarded only by `#if defined(SHIMMER3R)` still has to leave the
> Shimmer3 build valid.

### 1.1 Modules

| Directory | Responsibility | Reference |
|---|---|---|
| `Battery/` | Charge classification, charging state | [SHIMMER3_BATTERY_AND_CHARGING.md](SHIMMER3_BATTERY_AND_CHARGING.md) |
| `Boards/` | Board and expansion-board identification | [SHIMMER3_BOARD_REVISIONS.md](SHIMMER3_BOARD_REVISIONS.md) |
| `Button/` | User button debounce and gestures | |
| `CRC/` | CRC-16 for comms and stored records | |
| `Calibration/` | Calibration blob and per-sensor blocks | [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) |
| `Comms/` | Bluetooth, dock UART, SD file transfer | [protocol](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md), [dock](SHIMMER3_DOCK_PROTOCOL.md) |
| `Configuration/` | The 512-byte configuration image | [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) |
| `EEPROM/` | CAT24C16 map, branding, radio settings | [SHIMMER3_EEPROM_MEMORY_MAP.md](SHIMMER3_EEPROM_MEMORY_MAP.md) |
| `GSR/` | GSR range control | [SHIMMER3_GSR_AUTORANGE.md](SHIMMER3_GSR_AUTORANGE.md) |
| `LEDs/` | Status indication | [SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) |
| `Platform/` | The platform abstraction | §3 |
| `RTC/` | Clock conversions and validation | [SHIMMER3_TIMEKEEPING.md](SHIMMER3_TIMEKEEPING.md) |
| `SDCard/` | Card, header, data file, `sdlog.cfg` | [SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md) |
| `SDSync/` | Multi-device synchronisation | [SHIMMER3_SD_SYNC.md](SHIMMER3_SD_SYNC.md) |
| `Sensing/` | Channel configuration and the sample path | [SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) |
| `TaskList/` | The scheduler | §4 |
| `Test/` | Factory self-test | [SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md) |
| `Util/` | Shared helpers | |

## 2. Global state

One structure, `shimmerStatus` (`STATTypeDef`), is the single source of truth
for what the device is doing: `booting`, `docked`, `usbPluggedIn`, `sensing`,
`configuring`, `sdLogging`, `btStreaming`, `btConnected`, `btPowerOn`,
`sdSyncEnabled`, `buttonPressed`, `sdInserted`, `sdBadFile`, and the boot-stage
timers.

`batteryStatus` (`BattStatus`) is the equivalent for power.

Both are `extern` in `log_and_stream_externs.h` and both are declared
`volatile`, because interrupt handlers write them.

> **`shimmerStatus` is read by nearly every module and written by several.**
> It is a deliberately flat global rather than a message-passing design, which
> keeps the code small on an MSP430 but means the ordering of writes matters.
> `LogAndStream_init` clears the whole structure with `memset` after every
> module's own init has run.

## 3. Hardware and platform abstraction

`log_and_stream_externs.h` declares **62 `extern` symbols** the shared code
requires each platform to provide. That header *is* the platform contract.

Grouped by what they cover:

| Group | Examples |
|---|---|
| Board control | `Board_setExpansionBrdPower`, `Board_sd2Pc`, `Board_sd2Mcu`, `Board_isDocked`, `Board_isSdInserted`, `Board_isUsbPluggedIn`, `Board_isBtnPressed` |
| LEDs | `Board_ledOn`, `Board_ledOff`, `Board_ledToggle`, `Board_ledLwrSetColourRgb` |
| Clock | `RTC_get32`, `RTC_get64`, `RTC_getRwcTimeDiffPtr`, `RTC_isRwcTimeSet` |
| Timers | `SampleTimerStart`, `SampleTimerStop` |
| Bluetooth | `InitialiseBtAfterBoot`, `BtStop`, `isBtModuleOverflowPinHigh`, `saveBtError` |
| Dock / USB | `DockUart_init`, `DockUart_deinit`, `USB_init`, `USB_deinit`, `MX_USBX_Device_Init` |
| Sensor buses | `ADC_configureChannels`, `I2C_configureChannels`, `SPI_configureChannels`, and their `*_startSensing` counterparts |
| Platform-specific sensors | `MPU9150_startMagMeasurement`, `BMPX80_startMeasurement` (Shimmer3 only) |
| Fault handling | `triggerShimmerErrorState` |

`Platform/platform_api.h` adds a second, softer boundary: eleven
`PLATFORM_WEAK` functions with default implementations in `platform_api.c`, so
a platform overrides only what it needs. `PLATFORM_WEAK` resolves to the right
attribute for GCC/Clang, IAR and ARMCC.

> **The two boundaries fail differently.** A missing `extern` from
> `log_and_stream_externs.h` is a link error. A missing `platform_*` override
> links fine and silently uses the default — `platform_sleepWhenNoTask` does
> nothing, `platform_isUsbUartInitialised` returns false. Prefer the extern
> list for anything whose absence must be caught.

> **The boundary is a flat list of `extern` functions, not a struct of
> pointers.** Adding a platform means providing every symbol; the linker is the
> only thing that checks. A missing symbol is a link error, but a symbol with
> the right name and the wrong semantics is not caught at all.

> **Some externs exist only for one platform.**
> `MPU9150_startMagMeasurement` and `BMPX80_startMeasurement` are Shimmer3-only
> parts, and the tasks that call them are inside `#if defined(SHIMMER3)`. The
> declarations are unconditional, so a Shimmer3R build declares functions it
> never links.

## 4. The task list

The scheduler is a **32-bit bitmask**. There is no queue, no priority field and
no per-task data.

```c
uint32_t taskList;   // one bit per task
```

`TaskId_t` assigns each task a distinct bit:

| Bit | Task | Platform |
|---:|---|---|
| 0 | `TASK_SETUP_DOCK` | both |
| 1 | `TASK_BATT_READ` | both |
| 2 | `TASK_DOCK_PROCESS_CMD` | both |
| 3 | `TASK_DOCK_RESPOND` | both |
| 4 | `TASK_BT_PROCESS_CMD` | both |
| 5 | `TASK_USB_PROCESS_CMD` / `TASK_CFGCH` | S3R / S3 |
| 6 | `TASK_BT_RESPOND` | both |
| 7 | `TASK_RCCENTERR1` | both |
| 8 | `TASK_RCNODER10` | both |
| 9 | `TASK_SAMPLE_MPU9150_MAG` | S3 |
| 10 | `TASK_SAMPLE_BMPX80_PRESS` | S3 |
| 11 | `TASK_GATHER_DATA` | both |
| 12 | `TASK_STOPSENSING` | both |
| 13 | `TASK_SDLOG_CFG_UPDATE` | both |
| 14 | `TASK_STARTSENSING` | both |
| 15 | `TASK_SAVEDATA` | both |
| 16 | `TASK_SDWRITE` | both |
| 17 | `TASK_FACTORY_TEST` | both |
| 18 | `TASK_DOCK_OR_USB_STATE_CHANGE` | both |
| 19 | `TASK_BT_TX_BUF_CLEAR` | both |
| 20 | `TASK_BT_TURN_ON_AFTER_BOOT` | both |
| 21 | `TASK_JUMP_TO_BOOT_LOADER` | S3R |
| 22 | `TASK_WRITE_RADIO_DETAILS` | S3 |
| 23 | `TASK_SD_FILE_TRANSFER` | S3R |

> **Bit 5 means two different things.** `TASK_USB_PROCESS_CMD` on Shimmer3R,
> `TASK_CFGCH` on Shimmer3. The bit numbers are not a stable cross-platform
> identifier.

### 4.1 Priority is bit order

```c
for (i = 0; i < TASK_SIZE; i++) {
    task = 1UL << i;
    if (taskList & task) { ShimTask_clear(task); return task; }
}
```

`ShimTask_popNext` scans from bit 0 upward and returns the **lowest set bit**.
So **a task's bit number is its priority**, lower being higher priority, and
the enum order is the scheduling policy.

> **This is why the enum has ordering comments.** `TASK_SD_FILE_TRANSFER` is at
> bit 23 with a comment noting it is "*deliberately below `TASK_SDWRITE` and
> `TASK_DOCK_OR_USB_STATE_CHANGE`*" — a bulk transfer must never starve the
> logging write path or a dock event. **Inserting a task in the middle
> silently reprioritises everything above it.** Add new tasks at the top of the
> numbering unless you have thought about where they belong.

> **A task can only be pending once.** Setting a bit that is already set is a
> no-op, so two events arriving before the task runs produce one execution.
> Anything that must not be coalesced needs its own counter.

### 4.2 The main loop

```c
void ShimTask_NORM_manage(void)
{
    executingTask = ShimTask_popNext();
    if (executingTask == TASK_NONE)
        platform_sleepWhenNoTask();
    else
        switch (executingTask) { /* dispatch */ }
}
```

Cooperative and run-to-completion: a task runs to the end before the next is
considered. **There is no preemption between tasks.**

> **A long task delays everything, including sampling.** The sample path runs
> as `TASK_GATHER_DATA`, so a slow SD write or a long Bluetooth response pushes
> samples late. That is what the stuck-task watchdog in §4.4 is for.

### 4.3 Setting a task from an interrupt

`ShimTask_set` returns whether the device was idle when the task was queued,
and on Shimmer3R does extra work when called from an ISR:

```c
if (__get_IPSR() != 0) {
    HAL_PWR_DisableSleepOnExit();
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}
```

Without this the Cortex-M would return from the interrupt straight back to
sleep and the task would not run until something else woke the part.

> **There is no Shimmer3 equivalent** — the MSP430's sleep exit is handled
> differently, through the return-from-interrupt status bits.

### 4.4 Stuck-task watchdog

| Constant | S3 | S3R |
|---|---|---|
| `TASK_STUCK_TIMEOUT_S` | 5 s | 5 s |
| `TASK_STUCK_RESET_TIMEOUT_S` | 10 s | 10 s |
| Units | 32768 Hz ticks | milliseconds |

`ShimTask_executionStart` and `ShimTask_executionEnd` bracket each dispatch when
`TEST_TASK_MONITOR` is enabled: a task still running after 5 s is considered
stuck, and 10 s triggers a reset.

> **The monitor is behind `TEST_TASK_MONITOR`.** If that is not defined in a
> given build, there is no stuck-task protection at all.

### 4.5 FreeRTOS variant

The header aliases every entry point:

```c
#if USE_FREERTOS
#define ShimTask_manage  S4_RTOS_Task_manage
...
#else
#define ShimTask_manage  ShimTask_NORM_manage
#endif
```

Neither Shimmer3 nor Shimmer3R uses it; it exists for the Shimmer4 SDK. All
behaviour described here is the `NORM` path.

## 5. Boot

`boot_stage_t` sequences startup, and the LEDs display progress and faults
([SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) §5):

| Stage | Meaning |
|---|---|
| `BOOT_STAGE_START` | Entry |
| `BOOT_STAGE_I2C` | Sensor bus bring-up |
| `BOOT_STAGE_BLUETOOTH` | Radio bring-up |
| `BOOT_STAGE_BLUETOOTH_FAILURE` | Radio failed |
| `BOOT_STAGE_CONFIGURATION` | Configuration and calibration load |
| `BOOT_STAGE_END` | Normal operation |

`shimmerStatus.bootTimePerStageMs` accumulates per stage and is compared
against per-stage timeouts.

> **`BOOT_STAGE_BLUETOOTH_FAILURE` is a stage, not a flag.** It sits between
> `BLUETOOTH` and `CONFIGURATION` in the enum, and the LED code tests for it
> directly rather than testing a timeout — which is why a radio failure shows
> immediately while the other two faults wait for their timeouts.

`LogAndStream_init` initialises every module in a fixed order, then clears
`shimmerStatus`, then queues `TASK_BATT_READ` as the first work item:

```
ShimTask_init, ShimBatt_init, ShimConfig_reset, ShimSd_init,
ShimSdDataFile_init, ShimSdCfgFile_init, ShimSdHead_reset, ShimSens_init,
ShimDock_resetVariables, ShimBrd_init, ShimEeprom_init, ShimLeds_varsInit,
ShimBtn_init, ShimRtc_init
```

> **The `memset` of `shimmerStatus` happens *after* every module init.** Any
> module that sets a status flag in its own init has that flag erased. Set
> status only from `LogAndStream_init` onwards.

## 6. The data path

1. A hardware timer fires at the configured rate.
2. The ISR queues `TASK_GATHER_DATA`.
3. `ShimSens_gatherData` collects the enabled channels into the packet buffer,
   in the fixed order `ShimSens_configureChannels` established.
4. If logging, `ShimSdDataFile_writeToBuff` takes the record from
   `PACKET_TIMESTAMP_IDX` for `dataLen - 1` bytes.
5. If streaming, the CRC is appended per the session mode and the packet is
   handed to the Bluetooth writer.
6. When an SD buffer fills, `TASK_SDWRITE` flushes it.

Channel order and encodings are in
[SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §4-5.

> **Sampling above 4096 Hz is refused.** `ShimSens_startSensing` bails out with
> a comment — "*Please don't go too fast, Thx, Best Regards.*" — leaving
> `sensing` clear. A host requesting a higher rate gets no error, just a device
> that does not start.

> **Zero enabled channels is also refused**, silently, in the same way.

## 7. Runtime coordination

On this platform the runtime-coordination problem is **who owns the SD card**.

The most intricate state machine outside sync, in `log_and_stream_common.c`:

| Function | Purpose |
|---|---|
| `LogAndStream_updateDockedStateAndCheckChanged` | Debounced dock/USB detection |
| `LogAndStream_dockOrUsbStateUpdate` | Reacts to a change |
| `LogAndStream_setupDock` / `setupUndock` | Transitions |
| `LogAndStream_assignSdToUsb` / `assignSdToDock` / `releaseSdToMcu` | Hands the card between owners |
| `LogAndStream_sdWaitAndAbort` | Waits for in-flight SD activity to finish |

The card has exactly one owner at a time: the MCU for logging, or the host as
mass storage. Handing it over requires the current owner to quiesce first,
which is what `LogAndStream_sdWaitAndAbort` does.

> **A stale card state across a re-dock is a real failure mode on Shimmer3R.**
> `Board_sd2Mcu()` is what re-establishes MCU ownership, and omitting it after
> a power cycle of the card leaves the SDMMC peripheral pointing at a card that
> is no longer in the state it thinks. Anything that changes this path needs
> testing across repeated dock/undock cycles, not just one.

An undock event is deliberately delayed — `undockEvent` and
`time_newUnDockEvent` implement a settling period so a bouncing connector does
not start and stop logging repeatedly. Dock/USB state changes are debounced at
`DOCK_USB_DEBOUNCE_MS` = 50 ms, and the undock-start path waits `TIMEOUT_100_MS`
(3277 ticks, 100 ms) after a new undock event before acting.

## 8. Adding a subsystem

1. Put platform-independent logic in a new `common` directory.
2. Declare any hardware access you need in `log_and_stream_externs.h` and
   implement it in **both** platform repositories.
3. If it needs deferred work, add a `TaskId_t` — think about where in the bit
   order it belongs (§4.1) — and a `switch` case in `ShimTask_NORM_manage`.
4. Add configuration to `gConfigBytes` in the reserved ranges, and mirror to
   the SD header if a trial needs to record it.
5. Build for both platforms.
6. Add a document to `docs/` and link it from `docs/README.md`.

## Still unverified / not found in code

- **Which platforms override which `platform_*` weak defaults.**
  `Platform/platform_api.h` declares eleven functions — `platform_reset`,
  `platform_delayMs`, `platform_getTick`, `platform_processHwRevision`,
  `platform_initGpioForRevision`, `platform_gatherData`, `platform_crcData`,
  `platform_crcData16`, `platform_isDockUartInitialised`,
  `platform_isUsbUartInitialised` and `platform_sleepWhenNoTask` — each
  `PLATFORM_WEAK` with a no-op or false-returning default in
  `platform_api.c`. Which of them each platform actually overrides was not
  enumerated, and an un-overridden one fails silently rather than at link
  time.
- **Where the sample timer ISR queues `TASK_GATHER_DATA`.** The task exists and
  `ShimSens_gatherData` is its handler, but the ISR is in the platform
  repositories and was not read.
- **`TASK_CFGCH`.** Shimmer3-only, and no handler for it was found in the
  `ShimTask_NORM_manage` switch as read.
- **Whether `TEST_TASK_MONITOR` is enabled in shipping builds.** The constant
  gates the stuck-task watchdog; its definition is per-platform.
- **The `USE_FREERTOS` path.** Aliased throughout but not built by either
  generation; the `S4_RTOS_*` implementations were not examined.
