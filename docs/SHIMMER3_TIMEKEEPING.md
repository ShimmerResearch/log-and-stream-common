# Shimmer3 / Shimmer3R Timekeeping

Three clocks, two generations that keep them differently, and one contract that
has to be right for a recording to be placeable on a real timeline.

> **The timebase contract, stated once, plainly.** The Shimmer3 and Shimmer3R
> real-world clock is **UTC**, not local civil time. A host must convert to UTC
> before writing it and from UTC after reading it. This is the opposite of the
> Verisense convention, and getting it backwards produces recordings whose
> timestamps are wrong by the local UTC offset — which nothing detects, and
> which only becomes visible when files split at what should be midnight.

> **Verified against** — the revisions these claims were read from. A pinned
> commit is a citation, not a claim of currency.
>
> - **Firmware:** `log-and-stream-common` @ `ff242a6` —
>   `RTC/shimmer_rtc.{h,c}` in full; `Sensing/shimmer_sensing.h`
>   (`PACKET_TIMESTAMP_LEN`); `SDCard/shimmer_sd_data_file.{h,c}`
>   (`BIN_FILE_SPLIT_TIME_TICKS`, `ShimSdDataFile_writeSdHeaderToFile`);
>   `SDCard/shimmer_sd_header.c` (`SDH_RTC_DIFF_*`).
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4`;
>   `shimmer3r-firmware` @ `8f800952` — `RTC_get64`,
>   `RTC_getRwcTimeDiffPtr`, `RTC_isRwcTimeSet`.

> **How to read this document.** **S3** = Shimmer3 (MSP430); **S3R** =
> Shimmer3R (STM32U5). LogAndStream only.

**Source references:**

| Layer | File |
|---|---|
| Conversions, validation, error flag | `RTC/shimmer_rtc.{h,c}` |
| Platform clock access | `log_and_stream_externs.h` — `RTC_get64`, `RTC_getRwcTimeDiffPtr` |
| Packet timestamps | [SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §2.1 |
| File header timestamps | [SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md) §3.2, §3.3 |
| Multi-device alignment | [SHIMMER3_SD_SYNC.md](SHIMMER3_SD_SYNC.md) |
| LED indication | [SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) §3.1 |

---

## 1. The clocks

| Clock | Width | Rate | Purpose |
|---|---:|---|---|
| **Sample tick counter** | 24 bits in a packet | 32768 Hz | Per-sample timestamp. Wraps every 512 s |
| **Real-world clock (RWC)** | 64 bits of ticks | 32768 Hz | Absolute time since the Unix epoch |
| **Configuration time** | 32 bits of seconds | 1 Hz | When the configuration was written |

Everything is counted in **32768 Hz ticks**, not seconds. `RTC_get64()` returns
ticks since the Unix epoch; dividing by 32768 gives Unix seconds.

> **One tick is about 30.5 µs, and 32768 ticks is exactly one second.** The
> power-of-two rate means seconds convert exactly; sub-second fractions are
> exact multiples of 1/32768 and do not round.

## 2. How the real-world clock is kept

This is the root of almost every timekeeping surprise in this codebase.

**Shimmer3** — the MSP430 RTC runs as a free-running counter that **cannot be
set**. To represent absolute time the firmware keeps a 64-bit *offset* between
that counter and real-world time. Setting the clock stores a new offset; it
does not touch the counter.

**Shimmer3R** — the STM32 RTC **can** be set directly. There is no offset:

```c
#if defined(SHIMMER3R)
#define RTC_getRwcTime RTC_get64
#endif
```

On Shimmer3R the real-world clock and the tick counter are the same thing.

| | Shimmer3 | Shimmer3R |
|---|---|---|
| Hardware clock settable | No | Yes |
| Absolute time | counter + stored offset | the counter itself |
| Offset variable | `RTC_getRwcTimeDiffPtr()` | none |
| `RTC_getRwcTime` | separate function | alias for `RTC_get64` |

**Consequence for the SD header.** `SDH_RTC_DIFF_0..7` carries the offset on
Shimmer3 and is repurposed on Shimmer3R to hold the top three bytes of the
64-bit initial timestamp. Full detail in
[SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md) §3.3.

> **The same eight header bytes mean two different things.** A parser must
> branch on the hardware generation before touching them. Reading a Shimmer3R
> file with Shimmer3 logic yields a nonsensical multi-thousand-year offset,
> which at least fails loudly; the reverse yields a plausible-looking
> timestamp that is silently wrong.

## 3. Setting and reading the clock

The clock is set by the host, over Bluetooth (`SET_RWC_COMMAND`) or over the
dock (`UART_PROP_RWC_CFG_TIME`). The value is 64-bit ticks since the Unix
epoch, **UTC**.

The platform's `RTC_isRwcTimeSet()` reports whether the clock has been set
this power cycle. It is declared in `log_and_stream_externs.h` and implemented
by each platform, so it is the one to call.

`ShimRtc_isTimeSet()` looks like a second way to ask, and is not: it is
declared in `RTC/shimmer_rtc.h` and defined nowhere in this repository or
either platform, so a caller would fail to link. Ignore it.

> **The clock does not survive a power cycle on either generation** unless the
> hardware has a backup supply. A host should set it on every connection rather
> than assuming it is still valid.

### 3.1 Configuration time is separate

`rwcConfigTime64` is a distinct 64-bit value recording when the configuration
was written, exposed by `ShimRtc_getRwcConfigTime()` and stored in
configuration bytes 211-214 — **as 32 bits of seconds, big-endian**, not as
ticks.

`ShimRtc_isRwcConfigTimeSet()` is simply `rwcConfigTime64 != 0`, so a
configuration written at exactly the Unix epoch would read as unset.

Configuration time also names the experiment directory on the SD card
([SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md) §1.1), which is what
groups a multi-device trial's files together. Two devices in one trial must be
configured with the **same** configuration time or their directories will not
match.

## 4. The timebase contract

The Shimmer3 / Shimmer3R real-world clock is **UTC**. The firmware never applies a
timezone — it treats the RWC as a tick count since the Unix epoch — and nothing in
a file or a stream records which convention the host used, so a host that writes
local civil time produces a recording whose timestamps are silently offset by the
local UTC offset. The most visible consequence is where files split.

Data files roll over every `BIN_FILE_SPLIT_TIME_TICKS` = 32768 × 3600 — one
hour **of sample time**, measured from when logging started.

> **Files do not split at wall-clock hour boundaries, and nothing splits at
> midnight.** A trial started at 09:17 produces files beginning at 09:17, 10:17,
> 11:17 and so on. Any host expectation of midnight-aligned files is a host-side
> convention imposed after the fact, and it is that convention which makes the
> UTC-versus-local-time question visible: a UTC-based split falls at a different
> wall-clock moment than a local-time one, by exactly the local offset.

That is the mechanism behind the class of bug this document opens with. The
firmware is consistent — it applies no zone at all — and the failure comes from
a host writing local civil time into the RWC and then interpreting the result
as UTC, or vice versa.

> **The reference host writes the plain Unix epoch, on both of its links.** The
> Bluetooth path sends `System.currentTimeMillis()` scaled by 32.768
> (`bluetooth/ShimmerBluetooth.java:691-696`,
> `driverUtilities/UtilShimmer.java:convertMilliSecondsToShimmerRtcDataBytes*`)
> and the dock path sends the same value to `RTC_CFG_TIME`
> (`comms/wiredProtocol/CommsProtocolWiredShimmerViaDock.java:writeRealWorldClockFromPcTime`).
> Neither applies a zone offset. The driver does carry
> `getCurrentLocalTimezoneOffsetMillis`, and its only callers are in playback,
> shifting recorded timestamps back to the civil time of the trial for
> *display* — which is the correct place for a zone, and the reason it appears
> nowhere on the write path.
>
> So this is not a convention this platform gets to choose per host: write Unix
> epoch ticks, convert for display only, and a sensor set by the Java driver, by
> desktop Consensys or by the web SDK all read back the same instant.
>
> **Local civil time is the Verisense convention**, and it does not belong here.
> Verisense sets its real-world clock to the base station's local civil time, so
> code or documentation moved across from it will be a whole timezone offset
> adrift on a Shimmer3 or Shimmer3R — a mistake made at least twice in these
> repositories, once in a host page and once in the protocol document's
> `SET_RWC_COMMAND` caution, which now says the same as this section.

## 5. Time-of-day arithmetic and scheduling

Four helpers, all operating on `SHIM_RTC_t`:

| Function | Direction |
|---|---|
| `ShimRtc_rtc2Unix` | Broken-down date/time → Unix seconds |
| `ShimRtc_unix2Rtc` | Unix seconds → broken-down |
| `ShimRtc_ticks2Rtc` | 32768 Hz ticks → broken-down |
| `ShimRtc_isDateValid` | Range check |

`SHIM_RTC_t`:

| Field | Range | Notes |
|---|---|---|
| `seconds` | 0-59 | |
| `subseconds` | 0-32767 | Downcounter; reload is Shimmer3R's `SynchPrediv` = 32767, so 32768 steps per second |
| `minutes` | 0-59 | |
| `hours` | 0-23 | 24-hour |
| `weekday` | 1-7 | **Monday is 1** |
| `date` | 1-31 | |
| `month` | 1-12 | |
| `year` | 0-99 | **Offset from 2000** |
| `unix` | — | Seconds since 1970-01-01 |
| `ticks` | — | Ticks since 1970-01-01 |

> **`subseconds` counts DOWN, and on Shimmer3R its step is the same 1/32768 s
> as a tick.** Shimmer3R's `rtc.c` sets `AsynchPrediv = 0` and
> `SynchPrediv = 32767`, so the register runs 32767 → 0 once per second. Being
> a downcounter is the part that catches people: the elapsed fraction of the
> current second is `(32767 - subseconds) / 32768`, so a register value of
> 16383 means half a second has passed, not 16383/32768 of one.
>
> The field's own comment in `shimmer_rtc.h` says `RTC_SYNC_PREDIV` = 0x3FF,
> 1024 steps. That is stale: no such symbol is defined on either platform, and
> the value contradicts the prescaler the Shimmer3R actually configures. Do not
> take 1024 from it. Shimmer3 has no prescaler at all — its real-world clock is
> the 32 kHz tick counter itself.

> **`year` is 0-99 meaning 2000-2099.** `ShimRtc_rtc2Unix` adds 2000 and returns
> **0** for any year before `RTC_OFFSET_YEAR` (1970) — which cannot happen given
> the encoding, so the guard is unreachable. The real limit is 2099, after which
> the field cannot represent the date at all.

> **Weekday numbering is Monday = 1**, computed as `(days + 3) % 7 + 1`. Both
> `unix2Rtc` and `ticks2Rtc` derive it; nothing validates a caller-supplied
> weekday against the date, so `ShimRtc_isDateValid` will accept 29 February on
> a leap year with the wrong weekday.

Leap years use the full proleptic Gregorian rule
(`RTC_LEAP_YEAR`): divisible by 4, except centuries, except multiples of 400.

## 6. Error indication

`ShimRtc_rwcErrorCheck`:

```c
state = (!RTC_isRwcTimeSet()) && storedConfig->rtcErrorEnable;
ShimLeds_setRtcErrorFlash(state);
```

The upper LED then flashes cyan (Shimmer3R) or green-plus-blue (Shimmer3) at
5 Hz. Two things gate it:

- `rtcErrorEnable`, configuration byte 217 bit 4. Clear it and there is no
  warning at all.
- `TEST_RTC_ERR_FLASH_OFF`, a build-time switch that disables the check
  entirely.

> **The warning stops as soon as logging starts.** The LED state machine
> suppresses it while `sensing`
> ([SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) §3). So a device that
> starts logging with an unset clock shows the warning right up to the moment
> it starts producing files with wrong timestamps, and then stops warning.

> **`RTC_isRwcTimeSet` compares ticks against a millisecond constant on
> Shimmer3R.** The body is
> `RTC_get64() > 1735689600000`, commented as *"the timestamp for
> 2025-01-01T00:00:00Z"* (`shimmer3r-firmware` `Core/Src/rtc.c:707-710`) —
> but `RTC_get64` returns **32768 Hz ticks**, and 2025-01-01 in ticks is
> 56,875,076,812,800, some 32768× larger. As written the threshold is crossed
> once the counter passes about 613 days, so it still distinguishes a
> never-set clock from a set one — a device that has just booted reads near
> zero — but it does not do what its comment says, and it would accept a clock
> set to any date after 1971. Shimmer3 has no such problem: there the test is
> `rwcTimeDiff64 != 0`
> (`shimmer3-firmware` `Shimmer_Driver/5xx_HAL/hal_RTC.c:98-101`). Treat the
> flag as "probably set", not as a validity check on the value.

## 7. Placing a recording on an absolute timeline

### 7.1 Streamed data

Three bytes per packet, little-endian, wrapping every 512 s — two bytes and
2 s on Shimmer3 firmware below LogAndStream 0.5.4. Unwrapping and its failure
mode are in
[SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §2.1.

To place a stream on an absolute timeline, read the RWC at session start and
anchor the first packet against it. **How good that anchor can be depends on
the generation**, and it follows directly from §2: one platform's packet
timestamp is a slice of the real-world clock, the other's is not.

**Shimmer3R — exact.** The packet takes `RTC_get32()` and the clock
`RTC_get64()`, and on this platform `RTC_getRwcTime` *is* `RTC_get64`
(`RTC/shimmer_rtc.h:25-28`; the two functions have identical bodies in
`shimmer3r-firmware` `Core/Src/rtc.c:374-408`, and the packet is filled at
`Sensing/shimmer_sensing.c:445-476`). The three bytes in the packet are
therefore the **low 24 bits of the value `GET_RWC` returns**. So:

```
candidate = (rwcTicks & ~0xFFFFFF) | packetTicks
absTicks  = candidate adjusted by whole multiples of 2^24 so that it is
            nearest to rwcTicks + (hostNow - hostAtRwcRead) * 32.768
```

Host elapsed time is used only to pick the wrap, so it has to be good to
±256 s — no calibration, no latency assumption. The result is right to the
tick, and one `GET_RWC` per session is enough.

**Shimmer3 — estimated.** The counter cannot be set, so the clock is the
counter plus a stored offset
(`shimmer3-firmware` `Shimmer_Driver/5xx_HAL/hal_RTC.c:73-76`, set at `:78-86`),
and that offset **never goes over Bluetooth** — only into an SD header as
`SDH_RTC_DIFF_*`. The low bits of the clock and of the packet do not line up,
so the offset has to be inferred from the exchange itself:

```
hostMid       = (hostBefore + hostAfter) / 2      // of the GET_RWC exchange
counterAtRead = firstPacketTicks - (hostAtFirstPacket - hostMid) * 32.768
K             = rwcTicks - counterAtRead
absTicks(n)   = unwrap(packetTicks(n)) + K
```

Carry `(hostAfter - hostBefore) / 2` as the anchor's stated uncertainty: it is
the link's latency asymmetry, tens of milliseconds over Bluetooth. Sample
**spacing** is still exact on both platforms, being the device's own counter;
only the constant offset differs in quality.

**Neither — host-anchored.** When the clock has never been set, or the
firmware predates `GET_RWC` (LogAndStream 0.5.4), the only anchor left is the
host's own arrival time for the first packet:
`unixMs(n) = deviceMs(n) + (hostMs(first) - deviceMs(first))`. This is what
Consensys plots against, and it inherits the first packet's buffering delay as
a constant offset. Check the *clock has been set* status bit (§6) before
preferring an RWC anchor to this one.

> **Setting the clock mid-stream moves the anchor.** `SET_RWC_COMMAND` is one
> of the few writes permitted while sensing, and on Shimmer3R it steps the very
> counter the packets carry. Re-anchor after any write; do not carry an
> anchor across one.

### 7.2 Logged data

The file header carries a 64-bit initial timestamp — assembled differently per
generation ([SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md) §3.2 and
§3.3) — and each record carries the same 3-byte tick counter.

```
absoluteTicks(n) = initialTimestamp + unwrap(recordTick(n) - recordTick(0))
absoluteUnixSeconds = absoluteTicks / 32768
```

### 7.3 Multi-device

SD sync records each node's offset from the centre without adjusting any clock.
Apply it after the above. See [SHIMMER3_SD_SYNC.md](SHIMMER3_SD_SYNC.md) §7.

## 8. Accuracy and drift

The 32768 Hz timebase comes from a watch crystal. Two things follow.

**Rate error.** A crystal's frequency error is specified in parts per million;
at 20 ppm a clock drifts about 1.7 s per day. Over a multi-day trial that is
enough to matter when correlating against an external event log.

**Load capacitance sensitivity.** The crystal only oscillates at its nominal
frequency with the load capacitance it was specified for. Under-loading pulls
the frequency high; the error is a rate error, so it accumulates linearly and
looks exactly like a bad crystal.

> **This is not hypothetical on this hardware.** A Shimmer3R drift problem was
> traced to load capacitors below the crystal's specification, and reworking
> them corrected it. If a device shows a consistent, repeatable drift rate,
> check the load capacitors before suspecting firmware.

There is also a boot-time recovery path on Shimmer3R for a low-speed external
oscillator that fails to start, and a factory self-test that measures the
crystal — see
[SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md).

Neither generation compensates for drift in firmware. The `tcxo` configuration
bit (byte 218 bit 4) selects a temperature-compensated oscillator on hardware
that has one, and is forced off elsewhere
([SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §10.4).

## 9. Rules for a host

1. **Write UTC.** Convert from local time before setting the clock.
2. **Read as UTC.** Convert to local time for display only.
3. **Set the clock on every connection.** It does not survive a power cycle.
4. **Set the same configuration time on every device in a trial**, or their SD
   directories will not match.
5. **Branch on hardware generation** before interpreting `SDH_RTC_DIFF_*`.
6. **Check `RTC_isRwcTimeSet` before trusting a recording's absolute time.** A
   device that logged with an unset clock produces files whose timestamps start
   near the epoch.
7. **Do not assume file boundaries mean anything in wall-clock terms.**

## Still unverified / not found in code

- **Where the UTC convention is enforced.** The firmware treats the RWC as an
  opaque 64-bit tick count since the Unix epoch and never applies a timezone,
  which is what makes UTC the only coherent interpretation — but no comment or
  constant in `log-and-stream-common` states it. The statement at the head of
  this document is a contract inferred from the absence of any offset handling,
  and from the failure mode observed when a host wrote local civil time.
- **Whether either platform has a backup supply for the RTC.** The claim that
  the clock does not survive a power cycle follows from `ShimRtc_init` setting
  the configuration time to zero and from `RTC_isRwcTimeSet` existing at all,
  but the hardware question was not checked.
- ~~`RTC_SYNC_PREDIV`~~ — resolved: the symbol exists on neither platform and
  the `0x3FF` in the comment is stale. Shimmer3R `rtc.c` configures
  `AsynchPrediv = 0`, `SynchPrediv = 32767`, so the sub-second register counts
  32767 → 0 once per second at **1/32768 s** resolution. Shimmer3 has no
  prescaler at all — its RTC is the 32 kHz tick counter itself.
- **Crystal specification and the resulting ppm figure.** The 20 ppm used in §8
  is a worked illustration, not a specification read from this hardware's part.
- ~~`ShimRtc_isTimeSet`~~ — resolved: a dead prototype. It is declared in
  `shimmer_rtc.h` and defined nowhere — not in `common`, not in either
  platform. Any caller would fail to link; `RTC_isRwcTimeSet` is the live
  check.
- **Leap seconds.** Unix time ignores them and so does this firmware. Whether
  that matters for a given application is the application's problem.
