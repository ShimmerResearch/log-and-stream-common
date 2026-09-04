# Shimmer3 / Shimmer3R Dock Protocol

The wired command-and-control protocol spoken over the dock connector's UART,
and — on Shimmer3R — over the USB-C CDC-ACM virtual serial port.

It is a **different protocol from the Bluetooth one**, with its own framing,
its own command set and its own error codes. It is not a subset or a
re-encoding: a host that speaks one does not speak the other.

> **Verified against** — the revisions these claims were read from. A pinned
> commit is a citation, not a claim of currency.
>
> - **Firmware:** `log-and-stream-common` @ `f3cf73e` —
>   `Comms/shimmer_dock_usart.{h,c}` in full.
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4`;
>   `shimmer3r-firmware` @ `a8f105e5` — the UART and USB CDC transports that
>   feed `ShimDock_rxCallback`.

> **How to read this document.** **S3** = Shimmer3 (MSP430); **S3R** =
> Shimmer3R (STM32U5). LogAndStream only. The protocol is referred to in the
> source as "WP uart 3.0".

**Source references:**

| Layer | File |
|---|---|
| Framing, commands, components, properties | `Comms/shimmer_dock_usart.h` |
| RX state machine and command dispatch | `Comms/shimmer_dock_usart.c` |
| Configuration byte meanings | [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) |
| The Bluetooth protocol, for contrast | [SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md) |

---

## 1. Transport

| Platform | Transport |
|---|---|
| Shimmer3 | Dock connector UART |
| Shimmer3R | Dock connector UART **and** USB-C CDC-ACM |

On Shimmer3R the same `ShimDock_rxCallback` serves both, so a USB-C connection
presents the dock protocol — **not** the Bluetooth one. A host connecting over
USB-C gets configuration access and no streaming.

> **The dock protocol cannot stream sensor data.** There is no data-packet
> equivalent. It is configuration, identity and diagnostics only. Streaming is
> Bluetooth-only.

`ShimDock_rxCallback` returns immediately while `shimmerStatus.booting`, so
commands sent during boot are discarded silently.

## 2. Framing

```
+-----+---------+--------+-----------+----------+-------+
| '$' | command | length | component | property | ...   |  + CRC (2 bytes)
+-----+---------+--------+-----------+----------+-------+
   0       1         2         3           4       5...
```

| Offset | Constant | Content |
|---:|---|---|
| 0 | `UART_RXBUF_START` | Literal `'$'` (0x24) |
| 1 | `UART_RXBUF_CMD` | `UART_SET` or `UART_GET` |
| 2 | `UART_RXBUF_LEN` | Number of bytes that follow, **excluding the CRC** |
| 3 | `UART_RXBUF_COMP` | Component |
| 4 | `UART_RXBUF_PROP` | Property |
| 5+ | `UART_RXBUF_DATA` | Payload |
| last 2 | — | CRC-16, LSB order |

The CRC covers `length + 3` bytes — the `'$'`, the command, the length byte
and the payload, but not the CRC itself.

| Constant | Value |
|---|---:|
| `UART_DATA_LEN_MAX` | 138 |
| `UART_RSP_PACKET_SIZE` | 138 |

138 is sized for the worst case: `'$'` + get + length + component + property +
InfoMem length + two offset bytes + 128 data bytes + two CRC bytes.

> **The CRC is always 2 bytes here.** Unlike the Bluetooth link, there is no
> negotiable CRC mode — `uartCrc2Wait = 2` is unconditional.

### 2.1 The receive state machine

`ShimDock_rxCallback` is a byte-at-a-time state machine:

| State | Constant | Waiting for |
|---:|---|---|
| 4 | `UART_STEP_WAIT4_CMD` | Command byte |
| 3 | `UART_STEP_WAIT4_LEN` | Length byte |
| 2 | `UART_STEP_WAIT4_DATA` | `length` payload bytes |
| 1 | `UART_STEP_WAIT4_CRC` | 2 CRC bytes |
| 0 | `UART_STEP_WAIT4_NONE` | A `'$'` to start a frame |

In state 0 every byte other than `'$'` is discarded, which is what
resynchronises the parser after a corrupt frame.

**A 100 ms inter-byte timeout resets the parser.** If more than 100 ms passes
between two bytes of a frame, `uartSteps` is zeroed and the partial frame is
abandoned. A host must send a frame as a contiguous burst; a slow or
character-at-a-time writer will never complete one.

> **The timeout uses a different clock on each platform.** Shimmer3 compares
> `RTC_get64()` against `TIMEOUT_100_MS`; Shimmer3R compares `HAL_GetTick()`
> against the literal 100. A `TODO` in the source notes this should be unified.

An unrecognised command byte in state 4 aborts immediately with
`UART_BAD_CMD_RESPONSE` — the parser does not wait for the rest of the frame.

## 3. Commands and responses

| Constant | Value | Direction |
|---|---:|---|
| `UART_SET` | `0x01` | Host to device |
| `UART_RESPONSE` | `0x02` | Device to host |
| `UART_GET` | `0x03` | Host to device |
| `UART_BAD_CMD_RESPONSE` | `0xFC` | Unrecognised command |
| `UART_BAD_ARG_RESPONSE` | `0xFD` | Unrecognised component or property |
| `UART_BAD_CRC_RESPONSE` | `0xFE` | CRC check failed |
| `UART_ACK_RESPONSE` | `0xFF` | Set accepted |

> **The three error codes are distinct and worth acting on differently.**
> `0xFE` means retry — the frame was damaged in transit. `0xFC` and `0xFD` mean
> the request itself is wrong and retrying will not help.

## 4. Components and properties

A request addresses a **component** and a **property** of it.

### 4.1 Components

| Constant | Value | Notes |
|---|---:|---|
| `UART_COMP_SHIMMER` | `0x01` | The device itself |
| `UART_COMP_BAT` | `0x02` | Battery — treated as a sensor |
| `UART_COMP_DAUGHTER_CARD` | `0x03` | Expansion board |
| `UART_COMP_D_ACCEL` | `0x04` | Digital accelerometer |
| `UART_COMP_GSR` | `0x05` | GSR |
| `UART_COMP_RADIO_802154` | `0x09` | ShimmerGQ builds only |
| `UART_COMP_BT` | `0x0A` | Bluetooth |
| `UART_COMP_TEST` | `0x0B` | Factory test |

### 4.2 `UART_COMP_SHIMMER` properties

| Constant | Value | Meaning |
|---|---:|---|
| `UART_PROP_ENABLE` | `0x00` | Generic enable, shared across sensors |
| `UART_PROP_SAMPLE_RATE` | `0x01` | |
| `UART_PROP_MAC` | `0x02` | Bluetooth MAC address |
| `UART_PROP_VER` | `0x03` | Firmware and hardware version |
| `UART_PROP_RWC_CFG_TIME` | `0x04` | Real-world clock and configuration time |
| `UART_PROP_CURR_LOCAL_TIME` | `0x05` | Current local time |
| `UART_PROP_INFOMEM` | `0x06` | Configuration bytes, paged |
| `UART_PROP_LED0_STATE` | `0x07` | ShimmerGQ only |
| `UART_PROP_DEVICE_BOOT` | `0x08` | ShimmerGQ only |
| `UART_PROP_ENTER_BOOTLOADER` | `0x09` | **Shimmer3R only** |

> **`UART_PROP_ENTER_BOOTLOADER` exists only on Shimmer3R** and is the wired
> route into firmware update. See
> [SHIMMER3_BUILD_AND_PROGRAMMING.md](SHIMMER3_BUILD_AND_PROGRAMMING.md).

### 4.3 Other components' properties

`UART_COMP_BAT`:

| Constant | Value |
|---|---:|
| `UART_PROP_VALUE` | `0x02` |

`UART_COMP_DAUGHTER_CARD`:

| Constant | Value | Meaning |
|---|---:|---|
| `UART_PROP_CARD_ID` | `0x02` | The 3-byte expansion-board identity |
| `UART_PROP_CARD_MEM` | `0x03` | Raw EEPROM access |

> **`UART_PROP_CARD_MEM` reaches the whole EEPROM**, including the brand record
> and the radio settings page. See
> [SHIMMER3_EEPROM_MEMORY_MAP.md](SHIMMER3_EEPROM_MEMORY_MAP.md) §6.

`UART_COMP_D_ACCEL`:

| Constant | Value |
|---|---:|
| `UART_PROP_DATA_RATE` | `0x02` |
| `UART_PROP_RANGE` | `0x03` |
| `UART_PROP_LP_MODE` | `0x04` |
| `UART_PROP_HR_MODE` | `0x05` |
| `UART_PROP_FREQ_DIVIDER` | `0x06` |
| `UART_PROP_CALIBRATION` | `0x07` |

`UART_COMP_GSR` has no properties of its own — the header lists
`UART_PROP_ENABLE`, `UART_PROP_SAMPLE_RATE`, `UART_PROP_RANGE` and
`UART_PROP_DIVIDER` **all commented out**.

> **Property numbers are reused across components with different meanings.**
> `0x02` is `UART_PROP_MAC` on the Shimmer component, `UART_PROP_VALUE` on the
> battery, `UART_PROP_CARD_ID` on the expansion board, and
> `UART_PROP_DATA_RATE` on the accelerometer. A property number is meaningless
> without its component.

> **Several properties are declared only as commented-out lines**, which makes
> them look supported when reading the header. The GSR component in particular
> has none live.

## 5. Command handling

`ShimDock_processCmd` runs from `TASK_DOCK_PROCESS_CMD`:

1. Verify the CRC over `length + 3` bytes. On failure, reply
   `UART_BAD_CRC_RESPONSE`.
2. Dispatch on `UART_GET` or `UART_SET`, then on component, then on property.
3. Unknown component or property replies `UART_BAD_ARG_RESPONSE`.

A `UART_GET` returns a `UART_RESPONSE` frame; a `UART_SET` returns
`UART_ACK_RESPONSE`.

On Shimmer3R the receive buffer is reset with `USBX_CDC_ACM_RxBuf_Reset()` once
a complete frame is captured.

## 6. The legacy command set

An older, simpler protocol is still defined:

| Constant | Value | Meaning |
|---|---:|---|
| `UART_CMD_MAC` | 1 | MAC address |
| `UART_CMD_VER` | 2 | Version |
| `UART_CMD_BAT` | 3 | Battery |
| `UART_CMD_MEM` | 4 | Memory |
| `UART_CMD_RTC` | 5 | Real-time clock |
| `UART_CMD_RCT` | 6 | |
| `UART_CMD_RDT` | 7 | |
| `UART_CMD_TIM` | 8 | |

with `CBUF_SIZE` = 27 and `CBUF_PARAM_LEN_MAX` = 21.

New host code should use the `'$'`-framed protocol of §2. The legacy constants
are retained for compatibility with older dock firmware.

## 7. Interaction with logging

Docking has side effects beyond the protocol:

- `ShimSens_checkStartLoggingConditions` requires **not** docked, so logging
  cannot start while docked.
- Starting to log calls `DockUart_deinit()`, and on Shimmer3R also
  `USB_deinit()`, so the MCU takes exclusive ownership of the SD card.
- `ShimCalib_ram2File` is deferred while docked, because the card may be
  presented to the host as mass storage. See
  [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) §8.4.
- The battery sampling interval drops from 60 s to 2 s while docked. See
  [SHIMMER3_BATTERY_AND_CHARGING.md](SHIMMER3_BATTERY_AND_CHARGING.md) §4.1.

## Appendix A. Property payload layouts

From `ShimDock_processCmd` (request validation) and `ShimDock_sendRsp`
(response construction) at the pinned revision. Every frame is
`'$'`, command, `LEN`, component, property, value bytes, CRC-16 LE. `LEN`
counts component + property + value. Multi-byte integers are little-endian.

### GET requests and their responses

| Component | Property | Request value bytes | Response value bytes |
|---|---|---|---|
| `SHIMMER` `0x01` | `MAC` `0x02` | none (`LEN` must be 2) | 6 bytes MAC (`LEN` 8) |
| `SHIMMER` | `VER` `0x03` | none (`LEN` 2) | `DEVICE_VER`, `FW_IDENTIFIER` u16, `FW_VERSION_MAJOR` u16, `FW_VERSION_MINOR`, `FW_VERSION_PATCH` (`LEN` 9) |
| `SHIMMER` | `RWC_CFG_TIME` `0x04` | none (`LEN` 2) | 8 bytes: the 64-bit tick value the clock was last set to (`LEN` 10) |
| `SHIMMER` | `CURR_LOCAL_TIME` `0x05` | none (`LEN` 2) | 8 bytes: current 64-bit real-world-clock ticks (`LEN` 10) |
| `SHIMMER` | `INFOMEM` `0x06` | `length` u8, `offset` u16 — `length ≤ 128`, `offset ≤ 0x1FF`, `length + offset ≤ 0x200` | `length` bytes of InfoMem from `offset` (`LEN` = length + 2) |
| `SHIMMER` | `CALIB_DUMP` `0x07`* | `length` u8, `offset` u16 — `length ≤ 128`, within `SHIMMER_CALIB_RAM_MAX` | `length` bytes of the calibration RAM image |
| `BAT` `0x02` | `VALUE` `0x02` | none (`LEN` 2) | 3 bytes `battStatusRaw` — ADC LSB, ADC MSB, charger status (`LEN` 5) |
| `DAUGHTER_CARD` `0x03` | `CARD_ID` `0x02` | `length` u8, `offset` u8 — `length ≤ 16`, `offset ≤ 15`, sum `≤ 16` | `length` bytes of the 16-byte daughter-card ID |
| `DAUGHTER_CARD` | `CARD_MEM` `0x03` | `length` u8, `offset` u16 — `length ≤ 128`, `offset ≤ 2031`, sum `≤ 2032` | `length` bytes of EEPROM read from `offset + 16` |
| `BT` `0x0A` | `VER` `0x03` | none (`LEN` 2) | the module's version string, as many bytes as it has (`LEN` = 2 + strlen) |

\* `CALIB_DUMP` is compiled in only when `EN_CALIB_DUMP_RSP` is set; the
property value is whatever `UART_PROP_CALIB_DUMP` expands to in
`shimmer_dock_usart.h`.

### SET requests (all answered with `UART_ACK_RESPONSE` `0xFF` on success)

| Component | Property | Value bytes | Effect |
|---|---|---|---|
| `SHIMMER` | `RWC_CFG_TIME` `0x04` | 8 bytes, 64-bit ticks (`LEN` must be 10) | `RTC_setTimeFromTicksPtr`, clears `rtcSetByBt`, updates InfoMem byte `NV_SD_TRIAL_CONFIG0` and the SD header |
| `SHIMMER` | `INFOMEM` `0x06` | `length` u8, `offset` u16, then `length` bytes — bounded by `STOREDCONFIG_SIZE` | `ShimConfig_storedConfigSet`, `ShimConfig_checkAndCorrectConfig`, flags the SD config file for rewrite, `LogAndStream_infomemUpdate` |
| `SHIMMER` | `CALIB_DUMP`* | `length` u8, `offset` u16, then `length` bytes | `ShimmerCalib_ramWrite` (return value ignored; always ACKs) |
| `SHIMMER` | `ENTER_BOOTLOADER` `0x09` (Shimmer3R only) | 1 byte: seconds until reboot; `0` cancels | arms `RTC_setAlarmRebootToBootloader`, sets `bslRebootPending` |
| `DAUGHTER_CARD` | `CARD_ID` `0x02` | `length` u8, `offset` u8, then `length` bytes — `length ≤ 16`, `offset < 16` | writes EEPROM bytes 0-15 and refreshes the in-RAM daughter-card ID |
| `DAUGHTER_CARD` | `CARD_MEM` `0x03` | `length` u8, `offset` u16, then `length` bytes | `ShimEeprom_writeDaughterCardMem` — writes at `offset + 16`; rejects zero length, over-long or out-of-range writes with `BAD_ARG` |
| `TEST` `0x0B` | *suite index* | none | property byte `< FACTORY_TEST_COUNT` → `ShimFactoryTest_setup(PRINT_TO_DOCK_UART, suite)` and `TASK_FACTORY_TEST`; otherwise `BAD_CMD` |

Any component/property pair not in these tables → `UART_BAD_CMD_RESPONSE`
(`0xFC`); a recognised pair with a bad `LEN` or an out-of-range argument →
`UART_BAD_ARG_RESPONSE` (`0xFD`); a CRC mismatch → `UART_BAD_CRC_RESPONSE`
(`0xFE`). Error responses are three bytes: `'$'`, code, CRC-16 (the CRC is over
the two leading bytes).

On Shimmer3R the response goes to USB-CDC when USB is plugged in and initialised,
otherwise to the dock UART; on Shimmer3 always to the dock UART.

## Still unverified / not found in code

- ~~The UART baud rate~~ — resolved: **115200** on both platforms. Shimmer3
  `hal_UartA0.c` clocks `UCA0` from a 24 MHz SMCLK with `UCBR` 13 and `UCOS16`
  (115 384 baud, 0.16 % high) and carries the comment that 1 Mbaud would divide
  more cleanly "*but Consensys is currently set to use 115200*"; Shimmer3R
  `usart.c` initialises `huart1` (the dock UART) at 115200.
- ~~The CRC polynomial and seed used here~~ — resolved: the **same CRC as the
  Bluetooth link**. `ShimDock_uartCheckCrc` and `ShimDock_sendRsp` both call
  `platform_crcData` — on Shimmer3 the MSP430 CRC16 module seeded with
  `CRC_INIT` (`0xB0CA`, `hal_CRC.c`), on Shimmer3R the shared software CRC
  (`CRC/shimmer_crc.c`; the hardware CRC peripheral is configured for
  `0x1021`/`0xB0CA` but its wrapper is misnamed and never called). The check
  covers `'$'`, the command byte, the length byte and the payload (`LEN + 3`
  bytes) and compares against the two little-endian CRC bytes that follow;
  responses append their CRC the same way.
- ~~Per-property payload layouts~~ — resolved: enumerated in the appendix
  below from `ShimDock_processCmd` and `ShimDock_sendRsp`.
- ~~`UART_PROP_ENABLE` semantics~~ — resolved: it has none in this firmware.
  There is no `case UART_PROP_ENABLE` in `ShimDock_processCmd`, so a request
  for it — like every other property of the legacy per-sensor set
  (`SAMPLE_RATE`, `DATA_RATE`, `RANGE`, `LP_MODE`, `HR_MODE`) — falls to the
  `default` branch and is answered `UART_BAD_CMD_RESPONSE` (`0xFC`). The
  header comment describes an intent that was never implemented here.
- **`UART_CMD_RCT`, `UART_CMD_RDT` and `UART_CMD_TIM`.** Legacy constants with
  no expansion of the abbreviations anywhere in the source.
- ~~Whether the legacy command set is still handled~~ — resolved: it is not.
  The receive state machine accepts only `UART_SET` (`0x01`) and `UART_GET`
  (`0x03`) as the command byte; any other value aborts the frame and queues a
  `UART_BAD_CMD_RESPONSE`. The `UART_CMD_*` constants are unreachable.
