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

## Still unverified / not found in code

- **The UART baud rate.** Set in the platform transport layer, not in
  `log-and-stream-common`. No baud constant appears in this module.
- **The CRC polynomial and seed used here.** `ShimDock_uartCheckCrc` is the
  entry point; whether it shares `CRC_INIT = 0xB0CA` with the Bluetooth CRC was
  not confirmed.
- **Per-property payload layouts.** This document covers the framing and the
  component/property registry. The byte layout of each property's value —
  what a `UART_PROP_VER` response actually contains, for instance — is in the
  per-property branches of `ShimDock_processCmd` and was not enumerated.
- **`UART_PROP_ENABLE` semantics.** Described in the header as "*this is for
  all sensors*" but appearing under `UART_COMP_SHIMMER`. Which sensor a bare
  enable addresses was not established.
- **`UART_CMD_RCT`, `UART_CMD_RDT` and `UART_CMD_TIM`.** Legacy constants with
  no expansion of the abbreviations anywhere in the source.
- **Whether the legacy command set is still handled.** The constants are
  defined; no dispatch path for them was found in `ShimDock_processCmd`.
