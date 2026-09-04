# Shimmer3 / Shimmer3R Bluetooth Communication Protocol

This document describes the byte-level command/response protocol spoken between a
**host** (PC, phone, browser, or another Shimmer acting as a sync centre) and a
**Shimmer3** or **Shimmer3R** running **LogAndStream** firmware. The protocol is
transport-agnostic: the same byte sequences travel over classic Bluetooth SPP, over
BLE, and over the dock's serial link.

> **Verified against** — the revisions these byte-level claims were read from.
> A pinned commit is a citation, not a claim of currency: when the firmware
> moves on, this document needs re-checking against it rather than the stamp
> being wrong, and the `file:line` references throughout only resolve because
> the revision is pinned here.
>
> - **Firmware (authority for bytes):** `log-and-stream-common` @ `c13cbde` —
>   `Comms/shimmer_bt_uart.h` (opcode `#define`s), `Comms/shimmer_bt_uart.c`
>   (`ShimBt_dmaConversionDone` argument counts, `ShimBt_processCmd` handlers,
>   `ShimBt_sendRsp` response assembly, `ShimBt_assembleStatusBytes`,
>   `ShimBt_isCmdBlockedWhileSensing`, `ShimBt_isCmdAllowedWhileSdSyncing`),
>   `Comms/shimmer_sd_file_transfer.{h,c}`, `Configuration/shimmer_config.h`,
>   `Calibration/shimmer_calibration.h`, `log_and_stream_definitions.h`.
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4`
>   (`LogAndStream_Shimmer3/`), `shimmer3r-firmware` @ `a8f105e5`
>   (`LogAndStream_Shimmer3R/`).
> - **Host reference implementations:** `Shimmer-Java-Android-API` @ `edc3f7d9`
>   (v0.11.8_beta) — `driver/ShimmerObject.java`,
>   `bluetooth/ShimmerBluetooth.java`, `bluetooth/BtCommandDetails.java`;
>   `shimmer-web-sdk` @ `8f78313` — `devices/shimmer3r/constants.ts`,
>   `devices/shimmer3/protocol.ts`, `devices/shimmer3r/sdTransfer/protocol.ts`;
>   `Extras/python_scripts/` in this repository.

> **How to read this document.** **S3** = Shimmer3 (MSP430, LogAndStream);
> **S3R** = Shimmer3R (STM32U5). The **Gen** column in the opcode tables says
> which generation *serves* an opcode — that is, whether firmware compiled for
> that platform actually acts on it: `both` = served on both, `S3` / `S3R` =
> served only there (the code sits behind a `#if defined(SHIMMER3)` /
> `#if defined(SHIMMER3R)` guard), and `— (reserved)` = the opcode number is
> declared in the shared header but no LogAndStream code on either platform
> references it. Note that nearly every opcode `#define` itself is
> unconditional, so header visibility is not the same thing as being served.
>
> **Response payload length** counts the bytes that follow the response opcode,
> excluding both the leading `ACK` byte and any CRC bytes — the same convention
> the Java driver's `mExpectedResponseByteLength` uses. Lengths that depend on
> runtime state are given as a formula in the firmware's own terms.
>
> This document describes **LogAndStream** only. It is the single firmware lineage
> that both streams over Bluetooth and logs to SD. The historical **BtStream**
> (stream-only) and **SDLog** (log-only) firmware images share most of these
> opcodes but differ in which ones they serve and in some argument semantics;
> where that matters it is confined to the appendices, and any statement in the
> body of this document is a statement about LogAndStream.
>
> An opcode's number is protocol-wide even when only one generation serves it:
> hosts must not reuse a reserved number. Older firmware silently ignores an
> unknown command byte (`default:` in `ShimBt_dmaConversionDone`) rather than
> NACKing it, so hosts gate optional features on `GET_FW_VERSION_COMMAND`
> rather than probing.

**Source references:**

| Layer | File |
|---|---|
| Opcode definitions | `Comms/shimmer_bt_uart.h` |
| Command parsing / argument counts | `Comms/shimmer_bt_uart.c` — `ShimBt_dmaConversionDone` |
| Command handlers | `Comms/shimmer_bt_uart.c` — `ShimBt_processCmd` |
| Response assembly | `Comms/shimmer_bt_uart.c` — `ShimBt_sendRsp` |
| Status byte assembly | `Comms/shimmer_bt_uart.c` — `ShimBt_assembleStatusBytes` |
| Command gating while sensing / syncing | `Comms/shimmer_bt_uart.c` — `ShimBt_isCmdBlockedWhileSensing`, `ShimBt_isCmdAllowedWhileSdSyncing` |
| CRC modes | `CRC/shimmer_crc.{h,c}` |
| SD file transfer (Shimmer3R) | `Comms/shimmer_sd_file_transfer.{h,c}` |
| SD sync | `SDSync/shimmer_sd_sync.{h,c}` |
| Configuration byte layout | `Configuration/shimmer_config.h` — see [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) |
| Streaming packet format | `Sensing/shimmer_sensing.h` — see [SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) |
| Calibration payloads | `Calibration/shimmer_calibration.{h,c}` — see [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) |
| Host reference (Python) | `Extras/python_scripts/` |
| Host reference (Java) | `Shimmer-Java-Android-API` — `driver/ShimmerObject.java`, `bluetooth/ShimmerBluetooth.java` |
| Host reference (TypeScript) | `shimmer-web-sdk` — `devices/shimmer3r/constants.ts`, `devices/shimmer3/protocol.ts` |

---

## 1. Overview

The protocol is a **host-initiated request/response protocol over an unframed
byte stream**, with one asynchronous exception: once streaming or a bulk
transfer has been started, the device emits packets of its own accord.

Every exchange has the same shape. The host writes a command byte followed by
that command's arguments; the firmware stages the command, executes it, and
answers with an acknowledgement byte optionally followed by a response opcode
and its payload — all in a single transmission.

```
Host                                       Shimmer
  |                                           |
  |--- [opcode] [args...] ------------------->|
  |                                           |   ShimBt_processCmd()
  |                                           |
  |<-- [ACK] [rspOpcode] [payload] [crc?] ----|   ShimBt_sendRsp()
```

For a command with nothing to report the response is the bare acknowledgement:

```
  |--- [opcode] [args...] ------------------->|
  |<-- [ACK] ---------------------------------|
```

And for a refused command it is a bare negative acknowledgement, with no
response opcode and no payload:

```
  |--- [opcode] [args...] ------------------->|
  |<-- [NACK] --------------------------------|
```

> The pairing is structural: `ShimBt_processCmd`
> (`Comms/shimmer_bt_uart.c:822-1636`) decides what happens and sets exactly one
> of `sendAck` / `sendNack` plus, for a read, `getCmdWaitingResponse`;
> `ShimBt_sendRsp` (`:1753-2349`) then writes the acknowledgement byte first and
> switches on `getCmdWaitingResponse` to append the payload.

Once `START_STREAMING_COMMAND` or `START_SDBT_COMMAND` has been accepted, data
packets flow unsolicited and interleave with command responses on the same
stream ([§6](#6-streaming-data-flow)). Three other things also arrive
unsolicited: in-stream status pushes when the device's own state changes
([§5](#5-status-acknack-and-in-stream-responses)), data-rate test packets
([§7.12](#712-control-and-test)), and SD file-transfer frames
([§7.13](#713-sd-file-transfer-shimmer3r)). Everything else is a direct answer to
something the host sent.

**Lineage.** This document describes **LogAndStream**, the firmware that both
streams over Bluetooth and logs to SD. It is the only lineage still developed for
Shimmer3 and Shimmer3R, and the two platforms share the command implementation
from a single module (`log-and-stream-common`), which is why the great majority
of the protocol is identical between them. The differences that remain are almost
all of two kinds: a field that is wider on Shimmer3R because the newer hardware
has more of something, and an opcode whose *name* dates from a Shimmer3 part that
Shimmer3R replaced with a different one. Both kinds are called out where they
occur.

The historical **BtStream** (stream-only) and **SDLog** (log-only) images share
most of these opcode numbers but differ in which they serve and in some argument
semantics. They are out of scope; see
[Appendix B](#appendix-b-java-only-and-legacy-opcodes) for the opcodes that
survive only in host software as a result.

## 2. Transport

The same byte sequences travel over every transport. The firmware's command
parser sits behind a single UART to the Bluetooth module and cannot tell which
link a byte arrived on, so nothing in this document is transport-specific except
the framing consequences in [§2.3](#23-framing-guarantees-per-transport).

### 2.1 Classic Bluetooth SPP

Classic Bluetooth uses the Serial Port Profile, which presents a **raw
bidirectional byte stream with no framing, no message boundaries and no length
field**. RFCOMM guarantees ordering and delivery; it guarantees nothing about how
the bytes are grouped.

| Platform | Module | Notes |
|---|---|---|
| Shimmer3 | Microchip RN41 / RN42 | Classic only, no BLE. Fitted to units without an EEPROM |
| Shimmer3 | Microchip RN4678 | Dual-mode, classic + BLE |
| Shimmer3R | Infineon CYW20820 (Vela IF820 module) running EZ-Serial | Dual-mode, classic + BLE |

> Module detection and version parsing are in
> `Comms/shimmer_bt_uart.c:341-463`, which recognises RN41 v4.77, RN42 v4.77 and
> v6.15, and RN4678 v1.00.5 / v1.11.0 / v1.13.5 / v1.22.0 / v1.23.0 / v1.24.0.
> RN42 v6.30 is explicitly **not supported** and puts the device into its error
> state (`:378-383`). The version string is readable with
> `GET_BT_VERSION_STR_COMMAND` ([§7.2](#72-version-information)).

**Module status strings (Shimmer3).** The RN41/RN42/RN4678 modules can be
configured to announce link events in band — `%CONNECT,001BDC06A3D5%`,
`%DISCONN%`, `%REBOOT%` and so on — delimited by
`RN4678_STATUS_STRING_SEPARATOR`, the ASCII `%` character (0x25). These strings
arrive on the *same* UART as host command bytes, so the firmware's receive state
machine treats a leading `0x25` as the start of a status string and hands the
following bytes to `RN4678_parseStatusString` rather than to the command parser.

> `Comms/shimmer_bt_uart.c:534-539, 753-760`;
> `shimmer3-firmware` `Shimmer_Driver/RN4X/RN4X.h:201-241` for the separator and
> every status-string length.

`0x25` is `DEVICE_VERSION_RESPONSE` — a *response* opcode, never a command — so
no legitimate host command collides with it. It does, however, mean that on a
Shimmer3 with status strings enabled, a host that transmits arbitrary bytes while
trying to resynchronise can put the parser into status-string mode and stall it
until the expected number of bytes arrives. Recover by waiting out the receive
timeout rather than by sending more filler.

### 2.2 BLE

BLE is available on both generations, gated on the fitted module and on an EEPROM
bit — not on the platform. Shimmer3 units with an RN4678 and Shimmer3R units with
a CYW20820 both support it; Shimmer3 units with an RN41/RN42, and any unit
without an EEPROM, are classic-only.

> `ShimBt_startCommon`, `Comms/shimmer_bt_uart.c:161-175`, which enables
> classic-only for sync mode or for a unit without an EEPROM and otherwise takes
> both enables from `ShimEeprom_isBtClassicEnabled()` /
> `ShimEeprom_isBleEnabled()`. The re-check on a configuration change is
> `ShimConfig_checkBtModeFromConfig`
> (`Configuration/shimmer_config.c:637-668`), keyed on
> `ShimBrd_doesDeviceSupportBle()`.

The BLE link is a **cable-replacement service provided by the Bluetooth module
itself**, not by the firmware. The module terminates GATT and presents the
reassembled byte stream to the MCU over the same UART that carries classic
traffic; nothing in `log-and-stream-common` references a GATT service,
characteristic or UUID. The consequence for a host is that the byte-level
protocol is *identical* on BLE — but the packetisation is not.

**MTU and fragmentation.** A BLE notification carries at most one ATT MTU worth
of payload, so any response longer than that is split across notifications. The
Shimmer3 firmware's own working figure is `BLE_MTU_SIZE` 157
(`shimmer3-firmware` `Shimmer_Driver/RN4X/RN4X.h:242`). A host must therefore
**reassemble across notifications** and must not assume that one notification is
one protocol message. Conversely, it must not assume the opposite either: the
module is free to coalesce, and a single notification may contain the tail of one
message and the head of the next.

Because the transmit ring is much deeper on Shimmer3R (4096 bytes versus 256) and
a single hand-off to the UART is capped at 1024 bytes
(`BT_TX_MAX_DMA_CHUNK`), a Shimmer3R response can be produced faster than the
link drains it — which is the situation the SD file transfer's 256-byte reserve
exists to protect.

> `Comms/shimmer_bt_uart.h:25-41`.

**Reserved first bytes on Shimmer3R.** The CYW20820's UART receive path is
demultiplexed in firmware between EZ-Serial module traffic and Shimmer command
traffic, keyed on the first byte. The EZ-Serial binary start-of-frame values are
routed to the module's parser and never reach the command parser:

| Byte | EZ-Serial meaning |
|---|---|
| `0x80` | `EZS_BINARY_TYPE_EVENT` |
| `0xC0` | `EZS_BINARY_TYPE_CMDRSP` |
| `0xD0` | `EZS_BINARY_TYPE_CMDRSP \| EZS_COMMAND_SCOPE_FLASH` |

> `shimmer3r-firmware` `Shimmer_Driver/CYW20820/hal_CYW20820.c:271-276`;
> the constants at `Shimmer_Driver/CYW20820/EZ-Serial/ezsapi.h:119-124`.

**No command opcode may take any of those three values**, which is why
`SD_LIST_DIR_COMMAND` was assigned `0xCC` rather than the `0xC0` its response
number would suggest — see the note in
[§7.13](#713-sd-file-transfer-shimmer3r). `0x80` is `MYID_RESPONSE`, a response
opcode, so it never appears as a first byte from a host. The demultiplexer is
bypassed whenever the parser is mid-command (`ShimBt_isWaitingForArgs()`), so
these values are unrestricted **inside** an argument payload.

### 2.3 Framing guarantees per transport

**There is no framing.** No length prefix, no delimiter, no escape, no
start-of-frame byte. The receiver is a state machine that reads one command byte,
looks up how many argument bytes that opcode takes, and waits for exactly that
many.

> `ShimBt_dmaConversionDone`, `Comms/shimmer_bt_uart.c:222-805`. The fixed
> argument counts are the `switch (data)` at `:594-776`; the variable-length
> cases are handled by the `waitingForArgs` block at `:468-588`.

The implications are worth stating plainly, because they drive most of a host
implementation's error handling.

1. **Both sides must agree on every argument count.** An opcode the host thinks
   takes two arguments and the firmware thinks takes three leaves the parser
   consuming the next command byte as an argument, and every subsequent byte is
   misinterpreted. The argument counts in [§4](#4-opcode-table) are extracted
   from that switch and are the authority.
2. **An unknown command byte is silently discarded.** The `default:` case
   (`Comms/shimmer_bt_uart.c:773-775`) re-arms for the next command byte and
   returns. There is no NACK, no error and no way for a host to tell that a
   command was not understood. **Hosts must gate optional features on
   `GET_FW_VERSION_COMMAND`, never on probing.**
3. **Responses are not self-delimiting either.** A host must frame the *response*
   stream by knowing each response opcode's payload length — which is why this
   document gives an exact length or an exact formula for every one, and why an
   unexpected extra byte from the device is not a harmless anomaly but a
   permanent loss of synchronisation (see the
   `GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND` warning in
   [§5](#5-status-acknack-and-in-stream-responses)).
4. **Resynchronisation is by silence, not by a marker.** There is no byte the
   host can send to force a reset. A parser stuck mid-command clears when the
   receive timeout expires: `BT_RX_COMMS_TIMEOUT_TICKS` is 328 ticks of a
   32768 Hz clock, i.e. **10 ms** (`Comms/shimmer_bt_uart.h:255-256`). A host
   that suspects desynchronisation should stop transmitting for well over that,
   discard everything it receives, and then re-establish state with
   `TEST_CONNECTION_COMMAND` or `GET_FW_VERSION_COMMAND`.
5. **`DUMMY_COMMAND` (0xB5) is the only completely inert byte.** It is consumed
   with no response and no side effect ([§7.12](#712-control-and-test)), which
   makes it the safe choice for padding. It is not a resynchronisation marker —
   if the parser is mid-command it will be consumed as an argument like any other
   byte.

A Bluetooth disconnect resets the parser and several session settings; see
[§8](#8-connection-session-workflow).

### 2.4 Baud rates

The `BT_BAUD_RATE` enumeration is stored in the configuration image at InfoMem 30
(`NV_BT_COMMS_BAUD_RATE`) and describes the **MCU-to-module UART**, not anything
the host can observe. Over Bluetooth the host never sees a baud rate at all;
over a wired dock link the dock's own rate applies.

| Value | Baud | Supported by |
|---|---|---|
| `0` | 115200 | all |
| `1` | 1200 | RN42 only |
| `2` | 2400 | all |
| `3` | 4800 | all |
| `4` | 9600 | all |
| `5` | 19200 | all |
| `6` | 38400 | all |
| `7` | 57600 | all |
| `8` | 230400 | RN42 only |
| `9` | 460800 | RN42 only |
| `10` | 921600 | RN42 only |
| `11` | 1000000 | RN4678 v1.23 only — v1.13.5 and v1.22 have known problems |
| `12` | 2000000 | CYW20820 only |
| `0xFF` | `BAUD_INVALID` | sentinel; triggers the default below |

> `Comms/shimmer_bt_uart.h:307-324`. The enumeration order is fixed because the
> value is persisted to EEPROM.

**Defaults.** On Shimmer3 the default is chosen at run time by
`getDefaultBaudForBtVersion()`: `BAUD_115200` while in SD sync mode — higher
rates lose module status-string bytes when logging and syncing at once — and
otherwise 1000000 if the fitted module supports it, else 460800.

> `shimmer3-firmware` `Shimmer_Driver/RN4X/RN4X.c:2430-2444`. The stored byte is replaced
> with this value whenever it reads back as `0xFF`
> (`Configuration/shimmer_config.c:206-209`).

On Shimmer3R the default is `12` (2 Mbaud), and the link rate is negotiated at
boot rather than read from configuration: the firmware tries `BAUD_TO_USE`
(2000000, or 115200 on an SR48-6.0 board) and, on repeated initialisation
failure, walks a fallback ladder of 115200 → 460800 → 2000000 → 500000.

> `shimmer3r-firmware` `Core/Src/main.c:645-720, 834-837`;
> `Shimmer_Driver/CYW20820/CYW20820.h:21-25`.

Shimmer3 additionally overrides a stored `BAUD_1200` to `BAUD_2400` when the
fitted module is an RN4678, which does not support 1200
(`ShimBt_setBtBaudRateToUse`, `Comms/shimmer_bt_uart.c:2997-3015`).

⚠️ **`SET_BT_COMMS_BAUD_RATE` (0x6A) no longer changes anything, and still
ACKs.** The whole handler body is commented out behind
`//TODO changing BAUD rate is not going to be supported`, leaving only `break;`
(`Comms/shimmer_bt_uart.c:1352-1367`). Observed behaviour:

- the command returns a normal ACK, so a host cannot tell it did nothing;
- `storedConfig->btCommsBaudRate` is not updated, so an immediately following
  `GET_BT_COMMS_BAUD_RATE` (0x6C) returns the **old** value
  (`:2146-2151`) — a read-after-write check does detect it;
- it is nonetheless listed in `ShimBt_isCmdBlockedWhileSensing` (`:2969`), so it
  is NACKed while sensing despite doing nothing.

The stored byte at InfoMem 30 is still read at startup and is still reachable
through `SET_INFOMEM_COMMAND`, so the configuration field is live even though the
dedicated command is not. The web SDK omits all three baud opcodes, which is the
right call; the Java driver names them `SET_BAUD_RATE_COMMAND` /
`BAUD_RATE_RESPONSE` / `GET_BAUD_RATE_COMMAND`
([§4.5](#45-deprecated--no-op)).

## 3. Message structure

### 3.1 Command frame

```
[opcode] [fixed args ...] [in-band length] [payload ...]
```

An opcode takes a fixed number of argument bytes, and a small number of opcodes
follow those with a variable payload whose length is one of the argument bytes.
There are exactly two shapes:

| Shape | Length byte | Opcodes |
|---|---|---|
| Length is the **first** argument | `args[0]` | `SET_INFOMEM_COMMAND` 0x8C, `SET_CALIB_DUMP_COMMAND` 0x98, `SET_DAUGHTER_CARD_MEM_COMMAND` 0x67, `SET_DAUGHTER_CARD_ID_COMMAND` 0x64, `SET_CENTER_COMMAND` 0x76, `SET_SHIMMERNAME_COMMAND` 0x79, `SET_EXPID_COMMAND` 0x7C, `SET_CONFIGTIME_COMMAND` 0x85 |
| Length is the **third** argument | `args[2]` | `SET_EXG_REGS_COMMAND` 0x61 |

> `Comms/shimmer_bt_uart.c:470-511`. The `SET_EXG_REGS_COMMAND` special case is
> the `if (gAction == SET_EXG_REGS_COMMAND) waitingForArgsLength = args[2];`
> at `:477-480`.

The Shimmer3R SD file-transfer commands are a third shape — the path length is
the **last** fixed argument byte, whichever position that is — described in
[§7.13](#713-sd-file-transfer-shimmer3r).

**`MAX_COMMAND_ARG_SIZE` is 131** (`Comms/shimmer_bt_uart.h:44-45`), sized for
the largest command in the set: a 128-byte daughter-card memory write plus its
three fixed argument bytes.

> **An oversized payload is NACKed, not truncated.** If a host-supplied length
> byte would overrun `args[]`, the copy is clamped to what fits and a
> `argsPayloadTruncated` flag is raised; `ShimBt_processCmd` then rejects the
> command outright rather than dispatching it. The in-band length byte is
> deliberately left at the host's original value, because rewriting it to the
> clamped length would turn a malformed oversized request into an **accepted
> partial write** of configuration, calibration or EEPROM.
> `Comms/shimmer_bt_uart.c:559-577` (the clamp, with its reasoning) and
> `:843-846` (the rejection).

### 3.2 Response frame and ACK

```
[ACK] [rspOpcode] [payload ...] [crc ...]      success
[NACK] [crc ...]                               refusal
```

- `ACK_COMMAND_PROCESSED` is `0xFF`; `NACK_COMMAND_PROCESSED` is `0xFE`.
- The acknowledgement byte is written **first**, before the response payload, and
  the whole frame is handed to the transmit path in one call — a host will never
  see an ACK and its response separated by an unrelated command reply.
- A NACK **replaces the entire frame**. There is no response opcode and no
  payload, so a NACK is one byte (plus any CRC).

> `ShimBt_sendRsp`, `Comms/shimmer_bt_uart.c:1766-1777` (prologue) and
> `:2334-2339` (the override that collapses a frame to a single NACK when a
> response case discovers the request cannot be served on the fitted hardware).
> The single hand-off is the `ShimBt_writeToTxBufAndSend` call at `:2347`.

**`INSTREAM_CMD_RESPONSE` (0x8A) wrapping.** Responses that the firmware may also
need to emit *while data packets are flowing* are wrapped in an extra `0x8A`
byte, so that a host parsing a stream can recognise "this is a command response,
not a data packet" from the first byte after the ACK. The wrapped responses are:

| Command | Framing |
|---|---|
| `GET_STATUS_COMMAND` 0x72 | `[ACK][0x8A][0x71][status…]` |
| `GET_VBATT_COMMAND` 0x95 | `[ACK][0x8A][0x94][adc u16][chargerStatus]` |
| `GET_DIR_COMMAND` 0x89 | `[ACK][0x8A][0x88][len][name…]` |
| unsolicited status push | `[ACK?][0x8A][0x71][status…]` |
| `SD_FILE_DATA_RESPONSE` 0xC5 | `[0x8A][0xC5]…` — **no ACK** |
| `SD_FILE_STATUS_RESPONSE` 0xC6 | `[0x8A][0xC6]…` — **no ACK** |

Note that `DATA_PACKET` is `0x00` and every response opcode is non-zero, so the
first byte after an ACK already distinguishes the two; the `0x8A` wrapper exists
so that the *nested* opcode can be reused between the solicited and unsolicited
forms.

The leading ACK on an **unsolicited** status push is controlled by
`SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE` (0xA3) and is on by default; solicited
responses always carry it. See
[§5](#5-status-acknack-and-in-stream-responses).

### 3.3 CRC modes

`SET_CRC_COMMAND` (0x8B) selects one of three modes for the rest of the session:

| `COMMS_CRC_MODE` | Value | Bytes appended |
|---|---|---|
| `CRC_OFF` | 0 | none |
| `CRC_1BYTE_ENABLED` | 1 | 1 — the CRC's low byte |
| `CRC_2BYTES_ENABLED` | 2 | 2 — low byte then high byte |

> `CRC/shimmer_crc.h:12-18`; `ShimBt_setCrcMode` at
> `Comms/shimmer_bt_uart.c:2372-2382`, which falls back to `CRC_OFF` for any
> value of 3 or more rather than rejecting it.

**The mode is a session setting and it resets to off on every connect.**
`ShimBt_btCommsProtocolInit` sets `CRC_OFF` at startup (`:116`) and
`ShimBt_handleBtRfCommStateChange` sets it again on every disconnect (`:2543`).
A host must therefore re-issue `SET_CRC_COMMAND` after every reconnection; there
is no way to make the setting sticky.

**What the CRC covers.** When enabled it is appended to:

- every command response and NACK assembled by `ShimBt_sendRsp` (`:2341-2346`);
- every unsolicited in-stream status push (`:2462-2466`);
- every streaming data packet (see
  [SHIMMER3_STREAMING_DATA_FORMAT.md §2](SHIMMER3_STREAMING_DATA_FORMAT.md#2-packet-layout-and-timestamp)).

It is **not** applied to data-rate test packets, and the SD file-transfer data
and status frames carry their own unconditional CRC-16 instead — see
[§7.12](#712-control-and-test) and
[§7.13](#713-sd-file-transfer-shimmer3r).

**Algorithm.** A CRC-16 seeded with `CRC_INIT` = `0xB0CA`, computed over the
whole frame before the CRC bytes, with **an odd-length payload padded by one
trailing `0x00` byte**. The padding is not transmitted; it exists because the
implementation is the MSP430's hardware CRC unit, which is fed 16 bits at a
time.

The most readable statement of the algorithm is the host reference:

> `Extras/python_scripts/Shimmer_common/shimmer_crc.py` — `crc_byte()` is the
> per-byte step, `calc_crc()` seeds with `CRC_INIT` and appends the `0x00` pad
> when `length % 2 == 1`. The firmware side is
> `calculateCrcAndInsert` / `checkCrc` in `CRC/shimmer_crc.c`, over the
> platform's `platform_crcData()`; the MSP430 implementation and its odd-length
> special case are in `shimmer3-firmware`
> `Shimmer_Driver/5xx_HAL/hal_CRC.c:12-35`.

`CRC_MAX_SUPPORTED_BYTES` (3) is a validation sentinel and **must not** be sent
as a mode.

### 3.4 Size limits and paging

| Limit | Shimmer3 | Shimmer3R | Constant |
|---|---|---|---|
| Maximum response frame | **133** bytes | **1024** bytes | `RESPONSE_PACKET_SIZE` |
| Maximum command arguments | 131 | 131 | `MAX_COMMAND_ARG_SIZE` |
| Bluetooth transmit ring | 256 bytes | 4096 bytes | `BT_TX_BUF_SIZE` |
| Single UART hand-off | 256 bytes | 1024 bytes | `BT_TX_MAX_DMA_CHUNK` |

> `log_and_stream_definitions.h:28-32`; `Comms/shimmer_bt_uart.h:25-45`. The
> response buffer is declared as `RESPONSE_PACKET_SIZE + 2` to leave room for
> two CRC bytes (`Comms/shimmer_bt_uart.c:1757`).

**Every bulk read and write is capped at 128 bytes per command**, independently of
the response-frame limit, so that a single page fits inside Shimmer3's 133-byte
budget:

| Region | Total size | Per-command cap | Commands |
|---|---|---|---|
| Configuration (InfoMem) | 384 modelled, 512 addressable | 128 | `0x8E` / `0x8C` |
| Calibration RAM | 1024 | 128 | `0x9A` / `0x98` |
| Daughter-card memory | 2032 | 128 | `0x69` / `0x67` |
| Daughter-card identity page | 16 | 16 | `0x66` / `0x64` |

A host therefore **pages**. To read the whole configuration image it issues three
requests at offsets 0, 128 and 256, each for 128 bytes, and concatenates the
payloads — which is exactly what the Python reference does
(`Extras/python_scripts/Shimmer_common/shimmer_comms_bluetooth.py:318-339`).
The addressable window is 512 bytes even though only 384 are modelled, so offsets
384-511 are legal but describe nothing.

Paging a *write* is not symmetric with paging a read, because the firmware runs
its whole-image correction pass after each chunk and, for the calibration dump,
accumulates chunks towards a declared total. Both are covered in
[§7.5](#75-infomem) and
[§7.6](#76-calibration-dump-and-per-sensor-calibration); the short version is
**write ascending offsets, start at offset 0, and read back**.

Shimmer3R's much larger response budget is what makes the SD file transfer
possible, and its one-shot responses are nonetheless kept under about 250 bytes
so the same wire format could later be served within Shimmer3's 133-byte limit
(`Comms/shimmer_sd_file_transfer.c:30-35`).

## 4. Opcode table

_The tables in this section are generated mechanically from the firmware header,
the firmware command parser and response assembler, and the two host
implementations. Do not hand-edit them._

The **Notes** column carries comparison flags, all of which are statements about
*named constants* rather than about behaviour:

| Flag | Meaning |
|---|---|
| `FW_ONLY` | No constant with this byte value exists in the Java driver. The driver may still handle the byte inline — this flags a missing *name*, not necessarily missing support. |
| `JAVA_ONLY` | The Java driver defines a constant for this byte value but the firmware header does not. Almost all are Shimmer2r-era or BtStream/SDLog-era opcodes; see [Appendix B](#appendix-b-java-only-and-legacy-opcodes). |
| `SDK_MISSING` | The TypeScript web SDK has no entry for this opcode. |
| `LEN_MISMATCH` | The Java registry's expected response payload length disagrees with the length the firmware assembles. |
| `no LogAndStream handler` | No code compiled for either platform references the opcode; the number is reserved only. |

> **Response payload lengths.** The column counts the bytes the firmware writes
> after the response opcode. Three commands write a byte inside a loop or in
> mutually exclusive branches, which a static count of the source cannot
> resolve; their lengths are curated from the firmware with a citation each
> (`GET_VBATT` 0x95, `GET_MPU9150_MAG_SENS_ADJ_VALS` 0x5D and
> `GET_PRESSURE_CALIBRATION_COEFFICIENTS` 0xA7) and the generator flags any
> further case with that shape rather than reporting a number for it.


### 4.1 Core

| Opcode | FW name | Kind | Args | Response opcode | Response payload length | Gen | Blocked while sensing | Java name (if different) | SDK name (if different) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| `0x00` | `DATA_PACKET` | DATA | 0 |  |  | both |  |  |  |  |
| `0x01` | `INQUIRY_COMMAND` | GET | 0 | `INQUIRY_RESPONSE` | S3: 8 + numberOfChannels<br>S3R: 11 + numberOfChannels | both |  |  |  |  |
| `0x02` | `INQUIRY_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x03` | `GET_SAMPLING_RATE_COMMAND` | GET | 0 | `SAMPLING_RATE_RESPONSE` | 2 | both |  |  |  |  |
| `0x04` | `SAMPLING_RATE_RESPONSE` | RSP |  |  |  | both |  |  |  | SDK payload len 2 |
| `0x05` | `SET_SAMPLING_RATE_COMMAND` | SET | 2 |  |  | both | yes |  |  |  |
| `0x06` | `TOGGLE_LED_COMMAND` | CTRL | 0 |  |  | both |  |  |  |  |
| `0x07` | `START_STREAMING_COMMAND` | CTRL | 0 |  |  | both |  |  |  |  |
| `0x08` | `SET_SENSORS_COMMAND` | SET | 3 |  |  | both | yes |  |  |  |
| `0x09` | `SET_WR_ACCEL_RANGE_COMMAND` | SET | 1 |  |  | both | yes | `SET_ACCEL_SENSITIVITY_COMMAND` |  | Java also:  |
| `0x0A` | `WR_ACCEL_RANGE_RESPONSE` | RSP |  |  |  | both |  | `ACCEL_SENSITIVITY_RESPONSE` |  | Java also:  |
| `0x0B` | `GET_WR_ACCEL_RANGE_COMMAND` | GET | 0 | `WR_ACCEL_RANGE_RESPONSE` | 1 | both |  | `GET_ACCEL_SENSITIVITY_COMMAND` |  | Java also:  |
| `0x0E` | `SET_CONFIG_SETUP_BYTES_COMMAND` | SET | 4 |  |  | both | yes | `SET_CONFIG_BYTE0_COMMAND` |  |  |
| `0x0F` | `CONFIG_SETUP_BYTES_RESPONSE` | RSP |  |  |  | both |  | `CONFIG_BYTE0_RESPONSE` |  |  |
| `0x10` | `GET_CONFIG_SETUP_BYTES_COMMAND` | GET | 0 | `CONFIG_SETUP_BYTES_RESPONSE` | 4 | both |  | `GET_CONFIG_BYTE0_COMMAND` |  |  |
| `0x20` | `STOP_STREAMING_COMMAND` | CTRL | 0 |  |  | both |  |  |  |  |
| `0x21` | `SET_GSR_RANGE_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x22` | `GSR_RANGE_RESPONSE` | RSP |  |  |  | both |  |  |  | SDK payload len 1 |
| `0x23` | `GET_GSR_RANGE_COMMAND` | GET | 0 | `GSR_RANGE_RESPONSE` | 1 | both |  |  |  |  |
| `0x25` | `DEVICE_VERSION_RESPONSE` | RSP |  |  |  | both |  | `GET_SHIMMER_VERSION_RESPONSE` |  | SDK payload len 1 |
| `0x2E` | `GET_FW_VERSION_COMMAND` | GET | 0 | `FW_VERSION_RESPONSE` | 6 | both |  |  |  |  |
| `0x2F` | `FW_VERSION_RESPONSE` | RSP |  |  |  | both |  |  |  | SDK payload len 6 |
| `0x30` | `SET_CHARGE_STATUS_LED_COMMAND` | SET | 1 |  |  | both |  | `SET_BLINK_LED` |  |  |
| `0x31` | `CHARGE_STATUS_LED_RESPONSE` | RSP |  |  |  | both |  | `BLINK_LED_RESPONSE` |  |  |
| `0x32` | `GET_CHARGE_STATUS_LED_COMMAND` | GET | 0 | `CHARGE_STATUS_LED_RESPONSE` | 1 | both |  | `GET_BLINK_LED` |  |  |
| `0x35` | `BUFFER_SIZE_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x36` | `GET_BUFFER_SIZE_COMMAND` | GET | 0 | `BUFFER_SIZE_RESPONSE` | 1 | both |  |  |  |  |
| `0x37` | `SET_MAG_GAIN_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x38` | `MAG_GAIN_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x39` | `GET_MAG_GAIN_COMMAND` | GET | 0 | `MAG_GAIN_RESPONSE` | 1 | both |  |  |  |  |
| `0x3A` | `SET_MAG_SAMPLING_RATE_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x3B` | `MAG_SAMPLING_RATE_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x3C` | `GET_MAG_SAMPLING_RATE_COMMAND` | GET | 0 | `MAG_SAMPLING_RATE_RESPONSE` | 1 | both |  |  |  |  |
| `0x3D` | `UNIQUE_SERIAL_RESPONSE` | RSP |  |  |  | both |  |  |  | FW_ONLY |
| `0x3E` | `GET_UNIQUE_SERIAL_COMMAND` | GET | 0 | `UNIQUE_SERIAL_RESPONSE` | S3: 8<br>S3R: 12 | both |  |  |  | FW_ONLY |
| `0x3F` | `GET_DEVICE_VERSION_COMMAND` | GET | 0 | `DEVICE_VERSION_RESPONSE` | 1 | both |  | `GET_SHIMMER_VERSION_COMMAND_NEW` |  |  |
| `0x40` | `SET_WR_ACCEL_SAMPLING_RATE_COMMAND` | SET | 1 |  |  | both | yes | `SET_ACCEL_SAMPLING_RATE_COMMAND` |  | Java also:  |
| `0x41` | `WR_ACCEL_SAMPLING_RATE_RESPONSE` | RSP |  |  |  | both |  | `ACCEL_SAMPLING_RATE_RESPONSE` |  | Java also:  |
| `0x42` | `GET_WR_ACCEL_SAMPLING_RATE_COMMAND` | GET | 0 | `WR_ACCEL_SAMPLING_RATE_RESPONSE` | 1 | both |  | `GET_ACCEL_SAMPLING_RATE_COMMAND` |  | Java also:  |
| `0x43` | `SET_WR_ACCEL_LPMODE_COMMAND` | SET | 1 |  |  | both | yes | `SET_LSM303DLHC_ACCEL_LPMODE_COMMAND` |  | Java also:  |
| `0x44` | `WR_ACCEL_LPMODE_RESPONSE` | RSP |  |  |  | both |  | `LSM303DLHC_ACCEL_LPMODE_RESPONSE` |  | Java also:  |
| `0x45` | `GET_WR_ACCEL_LPMODE_COMMAND` | GET | 0 | `WR_ACCEL_LPMODE_RESPONSE` | 1 | both |  | `GET_LSM303DLHC_ACCEL_LPMODE_COMMAND` |  | Java also:  |
| `0x46` | `SET_WR_ACCEL_HRMODE_COMMAND` | SET | 1 |  |  | both | yes | `SET_LSM303DLHC_ACCEL_HRMODE_COMMAND` |  | Java also:  |
| `0x47` | `WR_ACCEL_HRMODE_RESPONSE` | RSP |  |  |  | both |  | `LSM303DLHC_ACCEL_HRMODE_RESPONSE` |  | Java also:  |
| `0x48` | `GET_WR_ACCEL_HRMODE_COMMAND` | GET | 0 | `WR_ACCEL_HRMODE_RESPONSE` | 1 | both |  | `GET_LSM303DLHC_ACCEL_HRMODE_COMMAND` |  | Java also:  |
| `0x49` | `SET_GYRO_RANGE_COMMAND` | SET | 1 |  |  | both | yes | `SET_LSM6DSV_GYRO_RANGE_COMMAND` |  | Java also: SET_MPU9150_GYRO_RANGE_COMMAND |
| `0x4A` | `GYRO_RANGE_RESPONSE` | RSP |  |  |  | both |  | `LSM6DSV_GYRO_RANGE_RESPONSE` |  | Java also: MPU9150_GYRO_RANGE_RESPONSE |
| `0x4B` | `GET_GYRO_RANGE_COMMAND` | GET | 0 | `GYRO_RANGE_RESPONSE` | 1 | both |  | `GET_LSM6DSV_GYRO_RANGE_COMMAND` |  | Java also: GET_MPU9150_GYRO_RANGE_COMMAND |
| `0x4C` | `SET_GYRO_SAMPLING_RATE_COMMAND` | SET | 1 |  |  | both | yes | `SET_LSM6DSV_SAMPLING_RATE_COMMAND` |  | Java also: SET_MPU9150_SAMPLING_RATE_COMMAND |
| `0x4D` | `GYRO_SAMPLING_RATE_RESPONSE` | RSP |  |  |  | both |  | `LSM6DSV_SAMPLING_RATE_RESPONSE` |  | Java also: MPU9150_SAMPLING_RATE_RESPONSE |
| `0x4E` | `GET_GYRO_SAMPLING_RATE_COMMAND` | GET | 0 | `GYRO_SAMPLING_RATE_RESPONSE` | 1 | both |  | `GET_LSM6DSV_SAMPLING_RATE_COMMAND` |  | Java also: GET_MPU9150_SAMPLING_RATE_COMMAND |
| `0x4F` | `SET_ALT_ACCEL_RANGE_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x50` | `ALT_ACCEL_RANGE_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x51` | `GET_ALT_ACCEL_RANGE_COMMAND` | GET | 0 | `ALT_ACCEL_RANGE_RESPONSE` | 1 | both |  |  |  |  |
| `0x52` | `SET_PRESSURE_OVERSAMPLING_RATIO_COMMAND` | SET | 1 |  |  | both | yes | `SET_BMP180_PRES_RESOLUTION_COMMAND` |  | Java also:  |
| `0x53` | `PRESSURE_OVERSAMPLING_RATIO_RESPONSE` | RSP |  |  |  | both |  | `BMP180_PRES_RESOLUTION_RESPONSE` |  | Java also:  |
| `0x54` | `GET_PRESSURE_OVERSAMPLING_RATIO_COMMAND` | GET | 0 | `PRESSURE_OVERSAMPLING_RATIO_RESPONSE` | 1 | both |  | `GET_BMP180_PRES_RESOLUTION_COMMAND` |  | Java also:  |
| `0x5A` | `RESET_TO_DEFAULT_CONFIGURATION_COMMAND` | CTRL | 0 |  |  | both | yes |  |  |  |
| `0x5E` | `SET_INTERNAL_EXP_POWER_ENABLE_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x5F` | `INTERNAL_EXP_POWER_ENABLE_RESPONSE` | RSP |  |  |  | both |  |  |  | SDK payload len 1 |
| `0x60` | `GET_INTERNAL_EXP_POWER_ENABLE_COMMAND` | GET | 0 | `INTERNAL_EXP_POWER_ENABLE_RESPONSE` | 1 | both |  |  |  |  |
| `0x61` | `SET_EXG_REGS_COMMAND` | SET | 3 + [args[2]] bytes |  |  | both | yes |  |  |  |
| `0x62` | `EXG_REGS_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x63` | `GET_EXG_REGS_COMMAND` | GET | 3 | `EXG_REGS_RESPONSE` | 1 + exgLength | both |  |  |  |  |
| `0x64` | `SET_DAUGHTER_CARD_ID_COMMAND` | SET | 2 + [args[0]] bytes |  |  | both | yes |  |  | FW_ONLY |
| `0x65` | `DAUGHTER_CARD_ID_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x66` | `GET_DAUGHTER_CARD_ID_COMMAND` | GET | 2 | `DAUGHTER_CARD_ID_RESPONSE` | 1 + dcMemLength | both |  |  |  |  |
| `0x67` | `SET_DAUGHTER_CARD_MEM_COMMAND` | SET | 3 + [args[0]] bytes |  |  | both | yes |  |  | FW_ONLY |
| `0x68` | `DAUGHTER_CARD_MEM_RESPONSE` | RSP |  |  |  | both |  |  |  | FW_ONLY |
| `0x69` | `GET_DAUGHTER_CARD_MEM_COMMAND` | GET | 3 | `DAUGHTER_CARD_MEM_RESPONSE` | 1 + dcMemLength | both |  |  |  | FW_ONLY |
| `0x6D` | `SET_DERIVED_CHANNEL_BYTES` | SET | 8 |  |  | both | yes |  |  |  |
| `0x6E` | `DERIVED_CHANNEL_BYTES_RESPONSE` | RSP |  |  |  | both |  |  |  | Java registry expects 3 payload bytes, FW emits 8 (S3); LEN_MISMATCH |
| `0x6F` | `GET_DERIVED_CHANNEL_BYTES` | GET | 0 | `DERIVED_CHANNEL_BYTES_RESPONSE` | 8 | both |  |  |  |  |
| `0x8A` | `INSTREAM_CMD_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x8B` | `SET_CRC_COMMAND` | SET | 1 |  |  | both |  |  |  |  |
| `0x8C` | `SET_INFOMEM_COMMAND` | SET | 3 + [args[0]] bytes |  |  | both | yes |  |  |  |
| `0x8D` | `INFOMEM_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x8E` | `GET_INFOMEM_COMMAND` | GET | 3 | `INFOMEM_RESPONSE` | 1 + infomemLength | both |  |  |  |  |
| `0x94` | `VBATT_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x95` | `GET_VBATT_COMMAND` | GET | 0 | `INSTREAM_CMD_RESPONSE` | 4 | both |  |  |  |  |
| `0x96` | `TEST_CONNECTION_COMMAND` | CTRL | 0 |  |  | both |  |  |  |  |
| `0xA1` | `GET_BT_VERSION_STR_COMMAND` | GET | 0 | `BT_VERSION_STR_RESPONSE` | 1 + btVerStrLen | both |  | `GET_BT_FW_VERSION_STR_COMMAND` |  |  |
| `0xA2` | `BT_VERSION_STR_RESPONSE` | RSP |  |  |  | both |  | `BT_FW_VERSION_STR_RESPONSE` |  |  |
| `0xA3` | `SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE` | SET | 1 |  |  | both |  |  |  | FW_ONLY |
| `0xA4` | `SET_DATA_RATE_TEST` | SET | 1 |  | 0 | both | yes |  |  | FW_ONLY |
| `0xA5` | `DATA_RATE_TEST_RESPONSE` | RSP |  |  |  | both |  |  |  | FW_ONLY |
| `0xA8` | `SET_FACTORY_TEST` | SET | 1 |  |  | both | yes | `SET_TEST` |  |  |
| `0xAC` | `SET_ALT_ACCEL_SAMPLING_RATE_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0xAD` | `ALT_ACCEL_SAMPLING_RATE_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0xAE` | `GET_ALT_ACCEL_SAMPLING_RATE_COMMAND` | GET | 0 | `ALT_ACCEL_SAMPLING_RATE_RESPONSE` | 1 | both |  |  |  |  |
| `0xB2` | `SET_ALT_MAG_SAMPLING_RATE_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0xB3` | `ALT_MAG_SAMPLING_RATE_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0xB4` | `GET_ALT_MAG_SAMPLING_RATE_COMMAND` | GET | 0 | `ALT_MAG_SAMPLING_RATE_RESPONSE` | 1 | both |  |  |  |  |
| `0xB5` | `DUMMY_COMMAND` | CTRL | 0 |  |  | both |  |  |  | FW_ONLY |
| `0xB6` | `RESET_BT_ERROR_COUNTS` | CTRL | 0 |  |  | both |  |  |  | FW_ONLY |
| `0xB7` | `SET_FEATURE` | SET | 2 |  |  | both |  |  |  |  |
| `0xFE` | `NACK_COMMAND_PROCESSED` | ACK | 0 |  |  | both |  |  |  | FW_ONLY |
| `0xFF` | `ACK_COMMAND_PROCESSED` | ACK | 1 |  |  | both |  |  |  | permitted while SD syncing |

### 4.2 SD / trial configuration

| Opcode | FW name | Kind | Args | Response opcode | Response payload length | Gen | Blocked while sensing | Java name (if different) | SDK name (if different) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| `0x70` | `START_SDBT_COMMAND` | CTRL | 0 |  |  | both |  |  |  |  |
| `0x71` | `STATUS_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x72` | `GET_STATUS_COMMAND` | GET | 0 | `INSTREAM_CMD_RESPONSE` | 1 + ShimBt_assembleStatusBytes() | both |  |  |  |  |
| `0x73` | `SET_TRIAL_CONFIG_COMMAND` | SET | 3 |  |  | both | yes |  |  |  |
| `0x74` | `TRIAL_CONFIG_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x75` | `GET_TRIAL_CONFIG_COMMAND` | GET | 0 | `TRIAL_CONFIG_RESPONSE` | 3 | both |  |  |  |  |
| `0x76` | `SET_CENTER_COMMAND` | SET | 1 + [args[0]] bytes |  |  | both | yes |  |  |  |
| `0x77` | `CENTER_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x78` | `GET_CENTER_COMMAND` | GET | 0 | `CENTER_RESPONSE` | 1 | both |  |  |  |  |
| `0x79` | `SET_SHIMMERNAME_COMMAND` | SET | 1 + [args[0]] bytes |  |  | both | yes |  |  |  |
| `0x7A` | `SHIMMERNAME_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x7B` | `GET_SHIMMERNAME_COMMAND` | GET | 0 | `SHIMMERNAME_RESPONSE` | 1 + shimmer_name_len | both |  |  |  |  |
| `0x7C` | `SET_EXPID_COMMAND` | SET | 1 + [args[0]] bytes |  |  | both | yes |  |  |  |
| `0x7D` | `EXPID_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x7E` | `GET_EXPID_COMMAND` | GET | 0 | `EXPID_RESPONSE` | 1 + exp_id_name_len | both |  |  |  |  |
| `0x7F` | `SET_MYID_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x80` | `MYID_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x81` | `GET_MYID_COMMAND` | GET | 0 | `MYID_RESPONSE` | 1 | both |  |  |  |  |
| `0x82` | `SET_NSHIMMER_COMMAND` | SET | 1 |  |  | both | yes |  |  |  |
| `0x83` | `NSHIMMER_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x84` | `GET_NSHIMMER_COMMAND` | GET | 0 | `NSHIMMER_RESPONSE` | 1 | both |  |  |  |  |
| `0x85` | `SET_CONFIGTIME_COMMAND` | SET | 1 + [args[0]] bytes |  |  | both | yes |  |  |  |
| `0x86` | `CONFIGTIME_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x87` | `GET_CONFIGTIME_COMMAND` | GET | 0 | `CONFIGTIME_RESPONSE` | 1 + cfgtime_name_len | both |  |  |  |  |
| `0x88` | `DIR_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x89` | `GET_DIR_COMMAND` | GET | 0 | `INSTREAM_CMD_RESPONSE` | 2 + dir_len | both |  |  |  |  |
| `0x8F` | `SET_RWC_COMMAND` | SET | 8 |  |  | both |  |  |  |  |
| `0x90` | `RWC_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x91` | `GET_RWC_COMMAND` | GET | 0 | `RWC_RESPONSE` | 8 | both |  |  |  |  |
| `0x92` | `START_LOGGING_COMMAND` | CTRL | 0 |  |  | both |  | `START_LOGGING_ONLY_COMMAND` |  |  |
| `0x93` | `STOP_LOGGING_COMMAND` | CTRL | 0 |  |  | both |  | `STOP_LOGGING_ONLY_COMMAND` |  |  |
| `0x97` | `STOP_SDBT_COMMAND` | CTRL | 0 |  |  | both |  |  |  |  |
| `0x9C` | `UPD_SDLOG_CFG_COMMAND` | CTRL | 0 |  |  | both | yes |  |  | FW aliases: SET_I2C_BATT_STATUS_FREQ_COMMAND, UPD_SDLOG_CFG_COMMAND |
| `0xE0` | `SET_SD_SYNC_COMMAND` | SET | SYNC_PACKET_PAYLOAD_SIZE + BT_SD_SYNC_CRC_MODE |  |  | both |  | `ROUTINE_COMMUNICATION` |  | permitted while SD syncing |
| `0xE1` | `SD_SYNC_RESPONSE` | RSP |  |  |  | both |  |  |  | FW_ONLY |

### 4.3 Calibration

| Opcode | FW name | Kind | Args | Response opcode | Response payload length | Gen | Blocked while sensing | Java name (if different) | SDK name (if different) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| `0x11` | `SET_LN_ACCEL_CALIBRATION_COMMAND` | SET | SC_DATA_LEN_STD_IMU_CALIB |  |  | both | yes | `SET_ACCEL_CALIBRATION_COMMAND` |  | Java also:  |
| `0x12` | `LN_ACCEL_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  | `ACCEL_CALIBRATION_RESPONSE` |  | Java also:  |
| `0x13` | `GET_LN_ACCEL_CALIBRATION_COMMAND` | GET | 0 | `(dynamic: ShimBt_getExpectedRspForGetCmd())` | ShimBt_replySingleSensorCalibCmd() | both |  | `GET_ACCEL_CALIBRATION_COMMAND` |  | Java also:  |
| `0x14` | `SET_GYRO_CALIBRATION_COMMAND` | SET | SC_DATA_LEN_STD_IMU_CALIB |  |  | both | yes |  |  |  |
| `0x15` | `GYRO_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x16` | `GET_GYRO_CALIBRATION_COMMAND` | GET | 0 | `(dynamic: ShimBt_getExpectedRspForGetCmd())` | ShimBt_replySingleSensorCalibCmd() | both |  |  |  |  |
| `0x17` | `SET_MAG_CALIBRATION_COMMAND` | SET | SC_DATA_LEN_STD_IMU_CALIB |  |  | both | yes |  |  |  |
| `0x18` | `MAG_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x19` | `GET_MAG_CALIBRATION_COMMAND` | GET | 0 | `(dynamic: ShimBt_getExpectedRspForGetCmd())` | ShimBt_replySingleSensorCalibCmd() | both |  |  |  |  |
| `0x1A` | `SET_WR_ACCEL_CALIBRATION_COMMAND` | SET | SC_DATA_LEN_STD_IMU_CALIB |  |  | both | yes | `SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND` |  | Java also:  |
| `0x1B` | `WR_ACCEL_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  | `LSM303DLHC_ACCEL_CALIBRATION_RESPONSE` |  | Java also:  |
| `0x1C` | `GET_WR_ACCEL_CALIBRATION_COMMAND` | GET | 0 | `(dynamic: ShimBt_getExpectedRspForGetCmd())` | ShimBt_replySingleSensorCalibCmd() | both |  | `GET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND` |  | Java also:  |
| `0x2C` | `GET_ALL_CALIBRATION_COMMAND` | GET | 0 | `ALL_CALIBRATION_RESPONSE` | S3: ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd()<br>S3R: ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() + ShimBt_replySingleSensorCalibCmd() | both |  |  |  |  |
| `0x2D` | `ALL_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0x58` | `BMP180_CALIBRATION_COEFFICIENTS_RESPONSE` | RSP |  |  |  | S3 |  |  |  |  |
| `0x59` | `GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND` | GET | 0 | `S3 only: BMP180_CALIBRATION_COEFFICIENTS_RESPONSE` | S3: BMP180_CALIB_DATA_SIZE<br>S3R: 0 (NACK on unsupported hardware) | both |  |  |  |  |
| `0x5B` | `RESET_CALIBRATION_VALUE_COMMAND` | CTRL | 0 |  |  | both | yes |  |  |  |
| `0x5C` | `MPU9150_MAG_SENS_ADJ_VALS_RESPONSE` | RSP |  |  |  | S3 |  |  |  |  |
| `0x5D` | `GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND` | GET | 0 | `S3: MPU9150_MAG_SENS_ADJ_VALS_RESPONSE<br>S3R: ACK_COMMAND_PROCESSED` | S3: 3<br>S3R: 0 | both |  |  |  |  |
| `0x98` | `SET_CALIB_DUMP_COMMAND` | SET | 3 + [args[0]] bytes |  |  | both | yes |  |  |  |
| `0x99` | `RSP_CALIB_DUMP_COMMAND` | RSP |  |  |  | both |  |  |  |  |
| `0x9A` | `GET_CALIB_DUMP_COMMAND` | GET | 3 | `RSP_CALIB_DUMP_COMMAND` | 3 + calibRamLength | both |  |  |  |  |
| `0x9B` | `UPD_CALIB_DUMP_COMMAND` | CTRL | 0 |  |  | both | yes |  |  |  |
| `0x9F` | `BMP280_CALIBRATION_COEFFICIENTS_RESPONSE` | RSP |  |  |  | S3 |  |  |  |  |
| `0xA0` | `GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND` | GET | 0 | `S3 only: BMP280_CALIBRATION_COEFFICIENTS_RESPONSE` | S3: BMP280_CALIB_DATA_SIZE<br>S3R: 0 (NACK on unsupported hardware) | both |  |  |  |  |
| `0xA6` | `PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0xA7` | `GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND` | GET | 0 | `PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE` | 2 + n | both |  |  |  |  |
| `0xA9` | `SET_ALT_ACCEL_CALIBRATION_COMMAND` | SET | SC_DATA_LEN_STD_IMU_CALIB |  |  | both | yes |  |  |  |
| `0xAA` | `ALT_ACCEL_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0xAB` | `GET_ALT_ACCEL_CALIBRATION_COMMAND` | GET | 0 | `(dynamic: ShimBt_getExpectedRspForGetCmd())` | ShimBt_replySingleSensorCalibCmd() | both |  |  |  |  |
| `0xAF` | `SET_ALT_MAG_CALIBRATION_COMMAND` | SET | SC_DATA_LEN_STD_IMU_CALIB |  |  | both | yes |  |  |  |
| `0xB0` | `ALT_MAG_CALIBRATION_RESPONSE` | RSP |  |  |  | both |  |  |  |  |
| `0xB1` | `GET_ALT_MAG_CALIBRATION_COMMAND` | GET | 0 | `(dynamic: ShimBt_getExpectedRspForGetCmd())` | ShimBt_replySingleSensorCalibCmd() | both |  |  |  |  |

### 4.4 Shimmer3R-served extensions

| Opcode | FW name | Kind | Args | Response opcode | Response payload length | Gen | Blocked while sensing | Java name (if different) | SDK name (if different) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| `0xC1` | `SD_LIST_DIR_RESPONSE` | RSP |  |  |  | S3R |  |  |  | FW_ONLY |
| `0xC2` | `SD_FILE_STAT_COMMAND` | CTRL | 1 + path |  | ShimSdFileTransfer_buildStatRsp() | S3R |  |  |  | FW_ONLY |
| `0xC3` | `SD_FILE_STAT_RESPONSE` | RSP |  |  |  | S3R |  |  |  | FW_ONLY |
| `0xC4` | `SD_FILE_READ_COMMAND` | CTRL | 11 + path |  |  | S3R |  |  |  | FW_ONLY |
| `0xC5` | `SD_FILE_DATA_RESPONSE` | RSP |  |  |  | S3R |  |  |  | FW_ONLY |
| `0xC6` | `SD_FILE_STATUS_RESPONSE` | RSP |  |  |  | S3R |  |  |  | FW_ONLY |
| `0xC7` | `SD_TRANSFER_ABORT_COMMAND` | CTRL | 0 |  |  | S3R |  |  |  | FW_ONLY |
| `0xC8` | `SD_FREE_SPACE_COMMAND` | CTRL | 0 |  | ShimSdFileTransfer_buildFreeSpaceRsp() | S3R |  |  |  | FW_ONLY |
| `0xC9` | `SD_FREE_SPACE_RESPONSE` | RSP |  |  |  | S3R |  |  |  | FW_ONLY |
| `0xCA` | `SD_DELETE_COMMAND` | CTRL | 1 + path |  | ShimSdFileTransfer_buildDeleteRsp() | S3R |  |  |  | FW_ONLY |
| `0xCB` | `SD_DELETE_RESPONSE` | RSP |  |  |  | S3R |  |  |  | FW_ONLY |
| `0xCC` | `SD_LIST_DIR_COMMAND` | CTRL | 4 + path |  | ShimSdFileTransfer_buildListDirRsp() | S3R |  |  |  | FW_ONLY |

### 4.5 Deprecated / no-op

| Opcode | FW name | Kind | Args | Response opcode | Response payload length | Gen | Blocked while sensing | Java name (if different) | SDK name (if different) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| `0x24` | `DEPRECATED_GET_DEVICE_VERSION_COMMAND` | CTRL | 0 | `DEVICE_VERSION_RESPONSE` | 1 | both |  | `GET_SHIMMER_VERSION_COMMAND` |  | SDK_MISSING |
| `0x6A` | `SET_BT_COMMS_BAUD_RATE` | SET | 1 |  |  | both | yes | `SET_BAUD_RATE_COMMAND` |  | SDK_MISSING |
| `0x6B` | `BT_COMMS_BAUD_RATE_RESPONSE` | RSP |  |  |  | both |  | `BAUD_RATE_RESPONSE` |  | SDK_MISSING |
| `0x6C` | `GET_BT_COMMS_BAUD_RATE` | GET | 0 | `BT_COMMS_BAUD_RATE_RESPONSE` | 1 | both |  | `GET_BAUD_RATE_COMMAND` |  | SDK_MISSING |

### 4.6 Not served by LogAndStream

| Opcode | FW name | Kind | Args | Response opcode | Response payload length | Gen | Blocked while sensing | Java name (if different) | SDK name (if different) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| `0x0C` | — |  |  |  |  | — |  | `SET_5V_REGULATOR_COMMAND` |  | JAVA_ONLY |
| `0x0D` | — |  |  |  |  | — |  | `SET_PMUX_COMMAND` |  | JAVA_ONLY |
| `0x26` | — |  |  |  |  | — |  | `SET_EMG_CALIBRATION_COMMAND` |  | JAVA_ONLY |
| `0x27` | — |  |  |  |  | — |  | `EMG_CALIBRATION_RESPONSE` |  | JAVA_ONLY |
| `0x28` | — |  |  |  |  | — |  | `GET_EMG_CALIBRATION_COMMAND` |  | JAVA_ONLY |
| `0x29` | — |  |  |  |  | — |  | `SET_ECG_CALIBRATION_COMMAND` |  | JAVA_ONLY |
| `0x2A` | — |  |  |  |  | — |  | `ECG_CALIBRATION_RESPONSE` |  | JAVA_ONLY |
| `0x2B` | — |  |  |  |  | — |  | `GET_ECG_CALIBRATION_COMMAND` |  | JAVA_ONLY |
| `0x33` | — |  |  |  |  | — |  | `SET_GYRO_TEMP_VREF_COMMAND` |  | JAVA_ONLY |
| `0x34` | — |  |  |  |  | — |  | `SET_BUFFER_SIZE_COMMAND` |  | JAVA_ONLY |
| `0x55` | — |  |  |  |  | — |  | `SET_BMP180_PRES_CALIBRATION_COMMAND` |  | JAVA_ONLY |
| `0x56` | — |  |  |  |  | — |  | `BMP180_PRES_CALIBRATION_RESPONSE` |  | JAVA_ONLY |
| `0x57` | — |  |  |  |  | — |  | `GET_BMP180_PRES_CALIBRATION_COMMAND` |  | JAVA_ONLY |
| `0x9D` | `RSP_I2C_BATT_STATUS_COMMAND` | RSP |  |  |  | S4 only |  |  |  | no LogAndStream handler; FW_ONLY, SDK_MISSING |
| `0x9E` | `GET_I2C_BATT_STATUS_COMMAND` | GET |  |  |  | S4 only |  |  |  | no LogAndStream handler; FW_ONLY, SDK_MISSING |

## 5. Status, ACK/NACK and in-stream responses

### 5.1 ACK and NACK

| Byte | Constant | Meaning |
|---|---|---|
| `0xFF` | `ACK_COMMAND_PROCESSED` | The command was staged, dispatched and executed |
| `0xFE` | `NACK_COMMAND_PROCESSED` | The command was refused |

`ACK` means "I ran this", not "this had the effect you wanted". Almost every
out-of-range argument in this protocol is silently clamped and then ACKed, so a
host that needs to know the resulting value must read it back. The exceptions —
the handful of commands that genuinely NACK — are listed below.

`ACK` is also the only reply for the majority of commands: a `SET` has nothing to
report, so its whole response is one byte.

### 5.2 The three NACK preconditions

Three checks run at the top of `ShimBt_processCmd`, before the command's own
handler, and any one of them replaces the response with a bare NACK.

> `Comms/shimmer_bt_uart.c:830-846`.

1. **Sync-mode exclusivity.** `storedConfig->syncEnable XOR
   ShimBt_isCmdAllowedWhileSdSyncing(gAction)`. While SD sync is enabled the only
   permitted commands are `SET_SD_SYNC_COMMAND` (0xE0) and
   `ACK_COMMAND_PROCESSED` (0xFF); while it is disabled those two are the only
   *forbidden* ones. See [§7.14](#714-sd-sync) — this is the usual explanation
   for a device that connects and then NACKs everything.
2. **Blocked while sensing.** `shimmerStatus.sensing &&
   ShimBt_isCmdBlockedWhileSensing(gAction)`. **43** commands are on that list —
   effectively every configuration and calibration write.
   `ShimBt_isCmdBlockedWhileSensing` (`:2939-2995`) is the authoritative list,
   and the **Blocked while sensing** column of the [§4](#4-opcode-table) tables
   is extracted from it.
3. **Truncated payload.** `argsPayloadTruncated` — the host declared an in-band
   length that did not fit in `args[]` ([§3.1](#31-command-frame)).

A handful of individual handlers also NACK on their own account, and those are
the only per-command refusals in the protocol:

| Command | NACKs when |
|---|---|
| `SET_DAUGHTER_CARD_MEM_COMMAND` 0x67 | The EEPROM write is out of bounds |
| `RESET_BT_ERROR_COUNTS` 0xB6 | Shimmer3R always; Shimmer3 without an EEPROM |
| `SET_FEATURE` 0xB7 | Unrecognised feature id |
| `SET_SD_SYNC_COMMAND` 0xE0 | Not actually in a running sync session |
| `ACK_COMMAND_PROCESSED` 0xFF | Received outside a running sync session |
| `GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND` 0x59 | Shimmer3R (no such sensor) |
| `GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND` 0xA0 | Shimmer3R (no such sensor) |

The last two are set inside the *response* switch, after the ACK byte has
already been staged; an override then rewrites the frame to a single NACK and
discards the response bytes.

> `Comms/shimmer_bt_uart.c:2328-2339`, whose comment states the mechanism.

**Everything else that goes wrong produces no NACK.** Collected here because it
is the single most important thing for a host implementer to internalise:

| Situation | Observed reply |
|---|---|
| Unknown command byte | **nothing** — silently discarded ([§2.3](#23-framing-guarantees-per-transport)) |
| `DUMMY_COMMAND` 0xB5 | **nothing**, by design |
| `SET_INFOMEM_COMMAND` out of bounds | **nothing** — the handler returns before the ACK ([§7.5](#75-infomem)) |
| `SET_DAUGHTER_CARD_ID_COMMAND` with `len = 0` | **nothing**, and the parser is left mid-command ([§7.10](#710-daughter-card)) |
| `GET_INFOMEM_COMMAND` out of bounds | bare `ACK`, no `INFOMEM_RESPONSE` |
| `GET_DAUGHTER_CARD_ID`/`MEM` out of bounds | bare `ACK`, no response opcode |
| `GET_EXG_REGS_COMMAND` invalid arguments | `ACK`, `0x62`, `0x00` — a zero-length read |
| `SET_CALIB_DUMP_COMMAND` out of order or out of range | normal `ACK`, data discarded ([§7.6](#76-calibration-dump-and-per-sensor-calibration)) |
| Any setter given an out-of-range value | normal `ACK`, value silently clamped ([§7.8](#78-sensor-settings)) |
| `SET_BT_COMMS_BAUD_RATE` 0x6A | normal `ACK`, nothing happens ([§2.4](#24-baud-rates)) |
| `SET_CHARGE_STATUS_LED_COMMAND` 0x30 | normal `ACK`, no handler exists ([§7.4](#74-battery)) |
| `SET_FACTORY_TEST` 0xA8 with an out-of-range id | normal `ACK`, no test run |
| `GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND` 0x5D without an MPU-9x50 | `ACK` **then a second ACK** — desynchronises the host ([§7.6](#76-calibration-dump-and-per-sensor-calibration)) |

### 5.3 The status bytes

`ShimBt_assembleStatusBytes` produces the payload used by both
`GET_STATUS_COMMAND` and the unsolicited push. **One byte on Shimmer3, two on
Shimmer3R** (`STATUS_BYTE_COUNT`).

**Byte 0** — bit field, both generations:

| Bit | Mask | Field | Meaning |
|---|---|---|---|
| 7 | `0x80` | `toggleLedRedCmd` | Red-LED toggle state, as flipped by `TOGGLE_LED_COMMAND` |
| 6 | `0x40` | `sdBadFile` | The SD card or its file is unusable |
| 5 | `0x20` | `sdInserted` | A card is present in the slot |
| 4 | `0x10` | `btStreaming` | Streaming over Bluetooth |
| 3 | `0x08` | `sdLogging` | Logging to SD |
| 2 | `0x04` | `RTC_isRwcTimeSet()` | The real-world clock has been set this power cycle |
| 1 | `0x02` | `sensing` | Sensing (either or both of the two above) |
| 0 | `0x01` | `docked` | Sitting in a dock |

**Byte 1** — Shimmer3R only:

| Bit | Field |
|---|---|
| 0 | `usbPluggedIn` |
| 1-7 | zero |

> `Comms/shimmer_bt_uart.c:2920-2932`. Byte 1 is written inside
> `#if defined(SHIMMER3R)` at `:2927-2929`, so on Shimmer3 it is not merely zero
> — it is **not transmitted**. `STATUS_BYTE_COUNT` at
> `Comms/shimmer_bt_uart.h:260-264`. The underlying flags are the
> `STATTypeDef` bitfields at `log_and_stream_definitions.h:79-133`.

Bits 7, 6, 5 and 2, and the second byte, were added over time; the Java driver
gates each on a firmware version
(`isSupportedRedLedStateInStatus`, `isSupportedSdInfoInStatus`,
`isSupportedRtcStateInStatus`, `isSupportedUSBPluggedInStatus`). See
[Appendix A](#appendix-a-firmware-version-gates). On firmware older than a
given threshold the bit reads as zero rather than being absent — the byte count
does not change — so a host may read the bit unconditionally but must not treat
a zero as authoritative unless the version gate is satisfied. The **second byte**
is different: it is absent, not zero, on Shimmer3 and on Shimmer3R firmware
before the threshold, so a host must take its expected length from the
generation and version, not from the response.

### 5.4 In-stream responses

`INSTREAM_CMD_RESPONSE` (0x8A) is a wrapper, not a command. It marks a frame as
"a command response, on a link that may also be carrying data packets", so the
same nested response opcode can serve both the solicited and unsolicited forms of
a message. The wrapped set is listed in
[§3.2](#32-response-frame-and-ack).

**Solicited** in-stream responses are ordinary replies and always carry the
leading ACK:

```
[ACK] [0x8A] [0x71] [status ...] [crc ...]        GET_STATUS_COMMAND
```

**Unsolicited** status pushes have the same shape, but their leading ACK is
optional and controlled by `SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE` (0xA3):

```
[ACK] [0x8A] [0x71] [status ...] [crc ...]        prefix on  (default)
      [0x8A] [0x71] [status ...] [crc ...]        prefix off
```

> `ShimBt_instreamStatusRespSend`, `Comms/shimmer_bt_uart.c:2445-2470`. The
> prefix flag is `useAckPrefixForInstreamResponses`, defaulted to `1` by
> `ShimBt_resetBtResponseVars` (`:191-201`) — which runs at startup and on every
> disconnect (`:2545`), so **the prefix is on again after every reconnection**.

An unsolicited push carries the session CRC if one is enabled, exactly like a
solicited response (`:2462-2466`).

**When a push happens.** The firmware pushes status whenever its own state
changed for a reason the host did not cause:

| Trigger | Call site |
|---|---|
| Sensing started | `TASK_STARTSENSING` — `TaskList/shimmer_taskList.c:129-131` |
| Sensing stopped | `TASK_STOPSENSING` — `TaskList/shimmer_taskList.c:132-135` |
| Docked | `LogAndStream_setupDock` — `log_and_stream_common.c:514` |
| Undocked | `LogAndStream_setupUndock` — `log_and_stream_common.c:530-534` |

The two sensing triggers go through
`ShimBt_instreamStatusRespSendIfNotBtCmd`, which **suppresses the push when the
state change was caused by a Bluetooth command** — the host already knows,
because it got an ACK. A push therefore means "something happened that you did
not ask for": a button press, a dock or undock, a trial-duration expiry, or a
low-battery auto-stop.

> `Comms/shimmer_bt_uart.c:2430-2443`, whose comment enumerates exactly those
> hardware causes. The flag is `sensingStateChangeFromBtCmd`, set by each of the
> six start/stop handlers (`:915-960`).

The dock and undock pushes call `ShimBt_instreamStatusRespSend` **directly**, so
they are not suppressed. Undock only pushes when the user button is enabled or no
usable card is present; otherwise the push is deferred to the logging start that
follows.

**Normative rules for hosts.**

1. A host must be able to receive `[0xFF] 0x8A 0x71 …` at **any** time while
   connected — including between a command and its reply, and whether or not
   streaming is active.
2. Do not treat the leading `0xFF` of a push as the acknowledgement of an
   outstanding command. This is the one place where a stray-looking ACK is
   legitimate, and it is why command/response correlation must be by response
   opcode, not by counting ACKs.
3. Re-send `SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE` after every reconnect if you
   rely on the prefix being off.

## 6. Streaming data flow

### 6.1 The six start/stop commands

Two independent activities — streaming over Bluetooth and logging to SD — are
started and stopped by three pairs of commands. All six take no arguments and
answer with a bare ACK.

| Command | Opcode | Effect |
|---|---|---|
| `START_STREAMING_COMMAND` | `0x07` | Start Bluetooth streaming only |
| `STOP_STREAMING_COMMAND` | `0x20` | Stop Bluetooth streaming; leave logging running |
| `START_LOGGING_COMMAND` | `0x92` | Start SD logging only |
| `STOP_LOGGING_COMMAND` | `0x93` | Stop SD logging; leave streaming running |
| `START_SDBT_COMMAND` | `0x70` | Start **both** streaming and logging |
| `STOP_SDBT_COMMAND` | `0x97` | Stop **all** sensing — both activities |

> `Comms/shimmer_bt_uart.c:915-960`. Each handler sets
> `sensingStateChangeFromBtCmd = 1` (so no unsolicited status push follows, see
> [§5.4](#54-in-stream-responses)) and then queues the corresponding task.

> **The start commands are conditional and the ACK does not mean "started".**
> They queue `ShimTask_setStartStreamingIfReady()`,
> `ShimTask_setStartLoggingIfReady()` or
> `ShimTask_setStartStreamingAndLoggingIfReady()` — the *IfReady* is load-bearing.
> A start that cannot proceed (no card, bad file, docked, already sensing) is
> ACKed and then quietly does nothing. **Confirm with the status bits**: bit 4
> for streaming, bit 3 for logging, bit 1 for sensing overall
> ([§5.3](#53-the-status-bytes)). Because a command-initiated change suppresses
> the unsolicited push, the host must poll `GET_STATUS_COMMAND` rather than wait
> for one.

The Java driver names 0x92 and 0x93 `START_LOGGING_ONLY_COMMAND` /
`STOP_LOGGING_ONLY_COMMAND` ([§4.2](#42-sd--trial-configuration)), which is a
clearer description of what they do.

Not every stop is host-initiated: a trial-duration expiry, a low-battery
auto-stop, a button press or a dock event can stop sensing on their own, and each
of those *does* produce an unsolicited status push.

### 6.2 Fixing the packet layout

`INQUIRY_COMMAND` is what determines how a data packet is to be parsed, and the
inquiry handler calls `ShimSens_configureChannels()` immediately before
assembling its reply.

> `Comms/shimmer_bt_uart.c:1785-1790`.

**A host must therefore re-issue `INQUIRY_COMMAND` after any configuration
change and before starting a stream.** The channel-ID list in the response is a
snapshot of the layout at that instant; there is no notification when it changes,
and a data packet carries no description of its own contents. Concretely, the
sequence for every session is:

```
GET_DEVICE_VERSION_COMMAND      -> generation, so the inquiry payload can be parsed
GET_FW_VERSION_COMMAND          -> feature gates
… configuration writes …
INQUIRY_COMMAND                 -> channel order and count
START_STREAMING_COMMAND         -> data packets begin
```

Skipping the re-inquiry after a write is the most common cause of a stream that
decodes into plausible-looking nonsense: the widths still sum, so nothing fails
loudly, and every channel is simply attributed to the wrong signal.

### 6.3 Interleaving

Once streaming has started, the same byte stream carries data packets and command
replies. A data packet begins with `DATA_PACKET` (`0x00`); every response opcode
is non-zero, so the first byte after any frame boundary tells the host which it
is looking at.

```
[0x00] [ts u24] [channel data ...] [crc ...]     data packet
[0xFF] [rspOpcode] [payload ...]   [crc ...]     command response
[0xFF] [0x8A] [0x71] [status ...]  [crc ...]     unsolicited status push
```

The packet layout, the 3-byte timestamp and its 512-second wrap at 32768 Hz, and
every channel's encoding are in
[SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md).

**A host must keep servicing command replies throughout a stream.** Commands
remain accepted while sensing — all the `GET`s, and the 43-entry blocked list is
only writes ([§10](#10-important-boundaries)) — and their replies are queued into
the same transmit ring as data packets, so a reply appears at whatever packet
boundary the ring reaches it. The same applies during an SD file transfer, whose
data frames interleave with command replies by design
([§7.13](#713-sd-file-transfer-shimmer3r)).

There is **no host-side flow control**. If the host cannot keep up, the device's
transmit ring fills and the firmware drops what will not fit rather than blocking
its sampling loop: `ShimBt_pushBytesToBtTxBuf` returns a failure when the ring
has insufficient space, and `ShimBt_writeToTxBufAndSend` returns without queueing
anything.

> `Comms/shimmer_bt_uart.c:2646-2691`, `:2905-2918`.

A disconnect stops streaming unconditionally, whether or not the host asked
(`ShimBt_handleBtRfCommStateChange`, `:2533-2534`), and on Shimmer3 records a
`BT_ERROR_DISCONNECT_WHILE_STREAMING` error count (`:2527-2532`).

## 7. Command reference

### 7.1 Inquiry

`INQUIRY_COMMAND` is the only command that tells a host how to parse a data
packet. Everything else about the streaming format — channel widths, channel
order — follows from the channel-ID list this response carries.

- **Request:** `[0x01]` (no arguments)
- **Response:** `[ACK][0x02][payload]`

Before assembling the response the handler calls `ShimSens_configureChannels()`,
which recomputes the enabled-channel list and the packet length from the current
configuration.

> `Comms/shimmer_bt_uart.c:1785-1810` — `case INQUIRY_COMMAND`, with the
> `ShimSens_configureChannels()` call at `:1788`.

**Payload — Shimmer3** (8 fixed bytes, then the channel list):

| Offset | Size | Field | Encoding / Notes |
|---|---|---|---|
| 0..1 | 2 | Sampling-rate divider | `uint16_le`, InfoMem 0-1. Sample rate in Hz = `32768 / value` (`Configuration/shimmer_config.c:306`) |
| 2 | 1 | Config setup byte 0 | InfoMem 6 |
| 3 | 1 | Config setup byte 1 | InfoMem 7 |
| 4 | 1 | Config setup byte 2 | InfoMem 8 |
| 5 | 1 | Config setup byte 3 | InfoMem 9 |
| 6 | 1 | `numberOfChannels` | Number of channel-ID bytes that follow |
| 7 | 1 | `bufferSize` | InfoMem 2. Always `1` in current firmware (`ShimConfig_setDefaultConfig`, `shimmer_config.c:155`) |
| 8..8+n-1 | n | Channel IDs | One byte per channel, **in packet order** |

**Payload — Shimmer3R** (11 fixed bytes, then the channel list): identical, with
three extra configuration bytes inserted after config setup byte 3, so the
counts shift by three.

| Offset | Size | Field | Encoding / Notes |
|---|---|---|---|
| 0..1 | 2 | Sampling-rate divider | `uint16_le` |
| 2..5 | 4 | Config setup bytes 0-3 | InfoMem 6-9 |
| 6 | 1 | Config setup byte 4 | InfoMem 130 |
| 7 | 1 | Config setup byte 5 | InfoMem 131 |
| 8 | 1 | Config setup byte 6 | InfoMem 132 |
| 9 | 1 | `numberOfChannels` | |
| 10 | 1 | `bufferSize` | |
| 11..11+n-1 | n | Channel IDs | |

A host must therefore know the generation *before* it can find
`numberOfChannels`, and the only way to know the generation is
`GET_DEVICE_VERSION_COMMAND` ([§7.2](#72-version-information)). Read the version
first, then inquire.

**Normative rules for hosts.**

1. Re-issue `INQUIRY_COMMAND` after **every** configuration write that could
   change the enabled-sensor set or a sensor's range. The channel list is a
   snapshot, not a subscription.
2. Take channel widths and order from this list, never from the sensor
   enable bitmap. Channel IDs `0x0D`-`0x13` mean different physical inputs on
   the two generations, and the Shimmer3 magnetometer axis order depends on
   the fitted part — see [SHIMMER3_STREAMING_DATA_FORMAT.md §3](SHIMMER3_STREAMING_DATA_FORMAT.md#3-channel-id-registry)
   and [§4 Channel order rules](SHIMMER3_STREAMING_DATA_FORMAT.md#4-channel-order-rules).
3. The configuration bytes echoed here are the same bytes readable through
   `GET_CONFIG_SETUP_BYTES_COMMAND` and `GET_INFOMEM_COMMAND`; they are
   *post-correction* values (see [§7.5](#75-infomem)), so what the inquiry
   reports may differ from what the host last wrote.

> **Caution.** `GET_CONFIG_SETUP_BYTES_COMMAND` (0x10) returns only 4 bytes
> (InfoMem 6-9) on **both** generations, while the Shimmer3R inquiry response
> carries 7. There is no dedicated command for setup bytes 4-6; read them
> through the inquiry response or `GET_INFOMEM_COMMAND`, and write them through
> `SET_INFOMEM_COMMAND` or the individual rate/range commands.
> `Comms/shimmer_bt_uart.c:2077-2084` versus `:1798-1802`.

### 7.2 Version information

Four independent identity commands. A host should issue at least the first two
before anything else: the device version selects the channel and InfoMem
vocabulary, and the firmware version gates every optional feature
([Appendix A](#appendix-a-firmware-version-gates)).

#### `GET_DEVICE_VERSION_COMMAND` (0x3F)

- **Request:** `[0x3F]`
- **Response:** `[ACK][0x25][deviceVer]` — 1 payload byte

| `deviceVer` | Hardware |
|---|---|
| `3` | Shimmer3 |
| `10` | Shimmer3R |
| `58` | Shimmer4 SDK (not covered by this document) |

> The byte is the platform's `DEVICE_VER` macro, emitted at
> `Comms/shimmer_bt_uart.c:2100-2105`. Values from
> `shimmer3-firmware` `Shimmer_Driver/shimmer_definitions.h:54` and
> `shimmer3r-firmware` `Shimmer_Driver/shimmer_definitions.h:12`. They match the
> Java driver's `ShimmerVerDetails.HW_ID` constants
> (`driverUtilities/ShimmerVerDetails.java:28,35`).

`DEPRECATED_GET_DEVICE_VERSION_COMMAND` (0x24) is served by the identical
handler and returns the identical response — the two cases fall through
together. It is deprecated only because the byte `0x24` is ASCII `$`, which the
RN42 module interprets as the start of its remote-configuration escape when
that feature is enabled; the command byte could therefore be swallowed by the
module and never reach the firmware.

> `Comms/shimmer_bt_uart.h:79-81` carries the reason in a comment;
> `Comms/shimmer_bt_uart.c:2100-2101` shows the shared handler.

**Hosts should send 0x3F.** 0x24 exists so that pre-0x3F firmware can still be
identified.

#### `GET_FW_VERSION_COMMAND` (0x2E)

- **Request:** `[0x2E]`
- **Response:** `[ACK][0x2F][payload]` — 6 payload bytes

| Offset | Size | Field | Encoding |
|---|---|---|---|
| 0..1 | 2 | Firmware identifier | `uint16_le` |
| 2..3 | 2 | Version major | `uint16_le` |
| 4 | 1 | Version minor | `uint8` |
| 5 | 1 | Version patch | `uint8` |

| Identifier | Firmware lineage |
|---|---|
| `3` | LogAndStream (both generations) |
| `12` | Shimmer4 SDK LogAndStream |

> `Comms/shimmer_bt_uart.c:2107-2117`. `FW_IDENTIFIER` is `3` on both Shimmer3
> and Shimmer3R; the Java driver's `FW_ID.LOGANDSTREAM` is likewise `3`
> (`driverUtilities/ShimmerVerDetails.java:186`). Major/minor/patch come from
> the generated `version.h`, e.g. `FW_VERSION_STRING "v1.01.006"`.

The identifier does **not** distinguish the generations — both report `3`.
Generation comes from `GET_DEVICE_VERSION_COMMAND`; the version triple is only
meaningful within a lineage. See
[Appendix A](#appendix-a-firmware-version-gates) for why the four tag lineages
are not comparable with one another.

#### `GET_UNIQUE_SERIAL_COMMAND` (0x3E)

- **Request:** `[0x3E]`
- **Response:** `[ACK][0x3D][uid]` — **8** payload bytes on Shimmer3, **12** on
  Shimmer3R

The payload is the MCU's factory-programmed unique device ID, copied verbatim:
`HAL_GetUID()` on the MSP430, the three `HAL_GetUIDw0/1/2()` words on the
STM32U5. It is not a Shimmer serial number, it has no defined text form, and
nothing in the firmware interprets it — treat it as an opaque per-unit
identifier of generation-dependent length.

> `Comms/shimmer_bt_uart.c:2130-2145`.

⚠️ **Java driver gap.** The Java driver defines no constant for either `0x3D` or
`0x3E` (flagged `FW_ONLY` in [§4.1](#41-core)), so a Java-based host cannot ask
for the unique serial without sending the raw byte. The web SDK covers it.

#### `GET_BT_VERSION_STR_COMMAND` (0xA1)

- **Request:** `[0xA1]`
- **Response:** `[ACK][0xA2][len][ascii × len]` — `1 + len` payload bytes

The string is the Bluetooth module's own firmware banner, captured during module
bring-up and cached. On Shimmer3 it is the RN41/RN42/RN4678 version response with
any trailing `CMD>` prompt stripped; on Shimmer3R it is the CYW20820 EZ-Serial
application version. It is **not** null-terminated on the wire — take the length
from the length byte. The buffer is 100 bytes
(`Comms/shimmer_bt_uart.c:53`), which is sized for the longest banner either
module produces.

> `Comms/shimmer_bt_uart.c:2085-2093`; the capture and `CMD>` strip are at
> `:341-463`.

This is a diagnostic string with no stable format. Hosts should display it and
log it, and must not parse it to make protocol decisions — use
`GET_FW_VERSION_COMMAND` for that.

### 7.3 Device status

#### `GET_STATUS_COMMAND` (0x72)

- **Request:** `[0x72]`
- **Response:** `[ACK][0x8A][0x71][status × STATUS_BYTE_COUNT]`

The reply is wrapped in `INSTREAM_CMD_RESPONSE` (0x8A) because the firmware also
pushes this exact frame unsolicited while streaming — see
[§5](#5-status-acknack-and-in-stream-responses) for the byte layout, the bit
field, and the unsolicited-push rules. `STATUS_BYTE_COUNT` is **1** on Shimmer3
and **2** on Shimmer3R.

> `Comms/shimmer_bt_uart.c:1841-1847`; `ShimBt_assembleStatusBytes` at
> `:2920-2932`; `STATUS_BYTE_COUNT` at `Comms/shimmer_bt_uart.h:260-264`.

#### `GET_DIR_COMMAND` (0x89)

- **Request:** `[0x89]`
- **Response:** `[ACK][0x8A][0x88][len][ascii × len]` — `2 + len` payload bytes

Returns the *directory* portion of the current SD data-file path — the firmware
takes the cached file-name pointer and subtracts 3 from its length, dropping the
per-file numeric suffix. The string is not null-terminated on the wire.

> `Comms/shimmer_bt_uart.c:1905-1915`. Note `dir_len = strlen(fileNamePtr) - 3`
> at `:1908`.

The value is only meaningful once a logging session has established a path; the
firmware does not validate that a file exists. Like `GET_STATUS_COMMAND` this
reply is `INSTREAM_CMD_RESPONSE`-wrapped, so it is safe to issue while
streaming.

#### `TEST_CONNECTION_COMMAND` (0x96)

- **Request:** `[0x96]`
- **Response:** `[ACK]` — nothing else

The handler body is empty; the command exists purely so that a host can prove
the link is alive and the firmware is answering. Use it as a keep-alive or a
post-connect handshake probe.

> `Comms/shimmer_bt_uart.c:906-909`.

`DUMMY_COMMAND` (0xB5) is the *silent* equivalent and does not even ACK — see
[§7.12](#712-control-and-test).

#### `TOGGLE_LED_COMMAND` (0x06)

- **Request:** `[0x06]`
- **Response:** `[ACK]`

Flips `shimmerStatus.toggleLedRedCmd`. This is a **toggle**, not a set: there is
no command to drive the state to a known value, so a host that needs a known
state must read it back from status bit 7 and toggle again if it guessed wrong.
The bit is reported in the status byte on firmware new enough to carry it
(`isSupportedRedLedStateInStatus`, [Appendix A](#appendix-a-firmware-version-gates)).

> `Comms/shimmer_bt_uart.c:910-914`; the status bit at `:2922`.

### 7.4 Battery

#### `GET_VBATT_COMMAND` (0x95)

- **Request:** `[0x95]`
- **Response:** `[ACK][0x8A][0x94][adcLo][adcHi][chargerStatus]`

The handler forces a fresh measurement (`manageReadBatt(1)`) before assembling
the reply, so each call costs an ADC conversion and returns current data rather
than the cached periodic sample.

| Offset | Size | Field | Encoding / Notes |
|---|---|---|---|
| 0..1 | 2 | Battery ADC value | `uint16_le`, raw ADC counts — **not** millivolts |
| 2 | 1 | Charger status | bit 6 = `STAT1`, bit 7 = `STAT2`, bits 0-5 unused |

The two charger-chip pins are **active low** — a `0` bit means the pin is
asserted:

| bit7 `STAT2` | bit6 `STAT1` | Byte value (mask `0xC0`) | Meaning |
|---|---|---|---|
| 1 | 1 | `0xC0` | Charging suspended / no charger |
| 1 | 0 | `0x80` | Pre-conditioning |
| 0 | 1 | `0x40` | Fully charged |
| 0 | 0 | `0x00` | Bad battery |

> `Comms/shimmer_bt_uart.c:1848-1860`; the `BattStatusRaw` union, the
> `STAT1`/`STAT2` bit positions and the `CHRG_CHIP_STATUS_*` values are at
> `Battery/shimmer_battery.h:40-72`.

Converting the ADC value to millivolts needs the platform's ADC reference and
divider ratio, which are outside this document — see
[SHIMMER3_STREAMING_DATA_FORMAT.md §7.2](SHIMMER3_STREAMING_DATA_FORMAT.md#72-adc-and-battery),
which describes the same conversion for the `VBATT` streaming channel. The
firmware's own thresholds are in `Battery/shimmer_battery.h:17-26`.

The three status bytes are written in a loop, so the full reply with CRC off is
`[ACK][0x8A][0x94][b0][b1][b2]` — six bytes, four of them payload after the
`0x8A` response opcode. `Comms/shimmer_bt_uart.c:1853-1858`.

#### `GET_CHARGE_STATUS_LED_COMMAND` (0x32)

- **Request:** `[0x32]`
- **Response:** `[ACK][0x31][battStat]` — 1 payload byte

`battStat` is the firmware's own coarse battery band, computed from the ADC value
with hysteresis, not a raw reading:

| Value | Constant | Meaning |
|---|---|---|
| `0x01` | `BATT_LOW` | Low |
| `0x02` | `BATT_MID` | Medium |
| `0x04` | `BATT_HIGH` | High |

> `Comms/shimmer_bt_uart.c:2118-2123`; constants at
> `Battery/shimmer_battery.h:13-20`.

#### `SET_CHARGE_STATUS_LED_COMMAND` (0x30)

- **Request:** `[0x30][value]`
- **Response:** `[ACK]`

⚠️ **This command has no handler.** 0x30 is armed for one argument byte
(`Comms/shimmer_bt_uart.c:675`) and reaches `ShimBt_processCmd`, but there is no
`case SET_CHARGE_STATUS_LED_COMMAND` in the switch, so it falls to `default:`
(`:1607-1610`) and is ACKed without effect. The following
`GET_CHARGE_STATUS_LED_COMMAND` returns the firmware's own computed band,
unchanged. The Java driver names the pair `SET_BLINK_LED` / `GET_BLINK_LED`,
which reflects its historical Shimmer2 meaning; on LogAndStream the *get* is a
battery-level read and the *set* is inert. Do not use it.

### 7.5 InfoMem

InfoMem is the non-volatile configuration image. This section covers only how it
is read and written over Bluetooth; every byte's meaning is in
[SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md).

**Sizes.** The modelled configuration image is **384** bytes
(`NV_TOTAL_NUM_CONFIG_BYTES`) inside a **512**-byte addressable window
(`NV_NUM_RWMEM_BYTES` = `STOREDCONFIG_SIZE`). Both commands validate against the
512-byte window, so offsets 384-511 are addressable even though the firmware
models nothing there.

> `Configuration/shimmer_config.h:56,64-66`.

#### `GET_INFOMEM_COMMAND` (0x8E)

- **Request:** `[0x8E][len][offsetLo][offsetHi]`
- **Response:** `[ACK][0x8D][len][data × len]` — `1 + len` payload bytes

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | `len` — bytes to read, **max 128** |
| 1..2 | 2 | `offset` — `uint16_le` byte offset into the 512-byte window |

Validation is `len <= 128 && offset <= 511 && len + offset <= 512`.

> `Comms/shimmer_bt_uart.c:1383-1393` (validation),
> `:2279-2286` (response assembly).

> **Caution — a rejected read still ACKs.** When the bounds check fails the
> handler simply does not set `getCmdWaitingResponse`, so the command falls
> through to the common tail which sends a **bare ACK with no
> `INFOMEM_RESPONSE`** (`Comms/shimmer_bt_uart.c:1618-1626`). A host that waits
> for `0x8D` after seeing the ACK will hang. Validate `len` and `offset`
> host-side rather than relying on the firmware to reject them.

#### `SET_INFOMEM_COMMAND` (0x8C)

- **Request:** `[0x8C][len][offsetLo][offsetHi][data × len]`
- **Response:** `[ACK]`

Same field layout and the same 128-byte / 512-byte validation as the read.

> `Comms/shimmer_bt_uart.c:1394-1438`.

Writing a chunk does considerably more than copy bytes. In order:

1. The payload is copied into the RAM configuration image at `offset`.
2. **If `offset == 128`**, the six MAC-address bytes at InfoMem 224-229 are
   overwritten with the module's real MAC, unconditionally — the firmware does
   not check whether the host's chunk even reached byte 224. Whatever the host
   supplied for those bytes is discarded.
3. `ShimConfig_checkAndCorrectConfig()` runs, over the **whole** image, after
   **every** chunk. It clamps out-of-range fields and fills in defaults, so it
   can change bytes the host did not write, and it can change bytes the host
   *did* write in this very chunk.
4. The corrected image is flushed to InfoMem in full
   (`LogAndStream_infomemUpdate()`).
5. **If `offset == 0`**, the calibration dump's first half is regenerated from
   configuration bytes 0-127 (`ShimCalib_configBytes0To127ToCalibDumpBytes`) and
   the on-card calibration file is queued for rewrite.
6. **On Shimmer3R only, if `offset == 128`**, the dump's second half is
   likewise regenerated from bytes 128-255.
7. The SD header mirror is rebuilt and the SD configuration file is queued for
   rewrite.

> The `offset == 0` / `offset == 128` tests are written as
> `INFOMEM_SEG_D_ADDR_MSP430 - INFOMEM_OFFSET_MSP430` (= 0) and
> `INFOMEM_SEG_C_ADDR_MSP430 - INFOMEM_OFFSET_MSP430` (= 128) — the MSP430
> InfoMem segment addresses that the layout is still named after.
> `shimmer3-firmware` `Shimmer_Driver/5xx_HAL/hal_InfoMem.h:57,62-63`.

> **Caution — a rejected write is answered with silence.** On a bounds-check
> failure the handler executes a bare `return`
> (`Comms/shimmer_bt_uart.c:1433-1436`), which leaves `ShimBt_processCmd`
> *before* the ACK/NACK tail. The host gets **no reply at all** — not an ACK,
> not a NACK. This is the only command in the protocol that answers a malformed
> request with silence, and it is why a host needs a response timeout even on a
> reliable transport.

**Normative rules for hosts.**

1. Page the image in 128-byte chunks: offsets 0, 128, 256 (and 384 if you use
   the unmodelled tail). This is what both reference hosts do — the Python
   helper reads exactly `384` bytes in three 128-byte requests
   (`Extras/python_scripts/Shimmer_common/shimmer_comms_bluetooth.py:318-339`)
   and writes in the same stride (`:297-316`).
2. **Always read back after writing.** Step 3 above means the stored image is
   not necessarily what you sent, and there is no error report when it differs.
3. Write **ascending** offsets. Because the correction pass runs per chunk and
   sees a partially-updated image, a descending or interleaved write order can
   have the pass "correct" a field against bytes that have not been updated yet.
4. Do not attempt to preserve the MAC bytes across a write to offset 128;
   the firmware owns them.
5. After the last chunk, issue `UPD_SDLOG_CFG_COMMAND` (0x9C) if you want the
   SD configuration file rewritten promptly rather than at the next natural
   trigger — see [§7.7](#77-sd-logging-and-trial-configuration).

### 7.6 Calibration dump and per-sensor calibration

Two independent mechanisms reach the same calibration state: a whole-blob
transfer (the *calibration dump*), and six per-sensor 21-byte accessors. The
blob is authoritative; the per-sensor commands are a convenience path that
writes into the configuration image and then syncs the dump. The blob's internal
structure is in [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md).

Firmware calibration RAM is **1024** bytes (`SHIMMER_CALIB_RAM_MAX`,
`Calibration/shimmer_calibration.h:18`).

#### `GET_CALIB_DUMP_COMMAND` (0x9A)

- **Request:** `[0x9A][len][offsetLo][offsetHi]`
- **Response:** `[ACK][0x99][len][offsetLo][offsetHi][data × len]` —
  `3 + len` payload bytes

The response echoes the requested length and offset before the data, so a host
can match a reply to a request without keeping its own ordering state.

Validation happens inside `ShimCalib_ramRead`, not in the command handler:
`len <= 128 && offset <= 1023 && len + offset <= 1024`. On a violation the
destination is **zero-filled** rather than left unwritten, so an out-of-range
read yields a well-formed response full of zeros.

> `Comms/shimmer_bt_uart.c:2241-2250`; `Calibration/shimmer_calibration.c:372-386`.

The blob's first two bytes are its own total length (`uint16_le`, excluding those
two bytes), which is how a host discovers how much to read. The Python reference
requests 128 bytes, reads the length out of the first two payload bytes, and
continues from there
(`Extras/python_scripts/Shimmer_common/shimmer_comms_bluetooth.py:260-295`).

#### `SET_CALIB_DUMP_COMMAND` (0x98)

- **Request:** `[0x98][len][offsetLo][offsetHi][data × len]`
- **Response:** `[ACK]`

⚠️ **The firmware's own usage comments are wrong on both counts.**
`Comms/shimmer_bt_uart.c:1142-1158` documents the argument order as
`offset, offset, length` and labels the *get* case with opcode `0x98`. The code
reads `args[0]` as the **length** and `args[1..2]` as the little-endian
**offset**, and `GET_CALIB_DUMP_COMMAND` is `0x9A`. The argument-count machinery
agrees with the code — it arms three fixed bytes and then takes the payload
length from `args[0]` (`:470-487`) — as do both reference hosts. The wire format
is `[opcode][len][offsetLo][offsetHi]`, matching `GET`/`SET_INFOMEM_COMMAND` and
`GET`/`SET_DAUGHTER_CARD_MEM_COMMAND`. **Never take the argument order from
those comments.**

Writes are **accumulated**, not applied per chunk:

1. A chunk at `offset < 2` is treated as the start of a new transfer. The
   firmware reads the blob's declared total length out of the first two bytes,
   adds 2, and stores that as the expected total.
2. Every later chunk adds its own length to a running counter.
3. When the running counter reaches the declared total, the staging buffer is
   promoted to live calibration RAM, the version header is stamped, and the
   configuration bytes plus SD header are regenerated from the new dump.

> `Calibration/shimmer_calibration.c:176-215`. The staging-buffer semantics are
> in the function's own comment at `:189-191`: "to start a new calib_dump
> transmission, the sw must use offset = 0 to setup the correct length …
> starting with offset > 2 is not accepted."

**Normative rules for hosts.**

1. **Start at offset 0 and write strictly forward.** A transfer that begins at
   any offset ≥ 2 never establishes a total length, so it never completes.
2. **Do not expect a NACK for a malformed transfer.** The handler acts only on
   the `1` return (transfer complete) and ignores both the "still accumulating"
   `0` and the out-of-range `0xFF`
   (`Comms/shimmer_bt_uart.c:1158-1162`). An out-of-order or oversized write is
   discarded silently, after a normal ACK.
3. Chunk at 128 bytes, as the Python reference does
   (`shimmer_comms_bluetooth.py:240-258`).
4. Read the blob back with `GET_CALIB_DUMP_COMMAND` to confirm.

ℹ️ **`SET_CALIB_DUMP_COMMAND` validates in the callee, not in the handler.**
`SET_INFOMEM_COMMAND` checks length and offset inline before touching anything
(`Comms/shimmer_bt_uart.c:1398-1400`); `SET_CALIB_DUMP_COMMAND` passes them
straight to `ShimCalib_ramWrite` (`:1158`), which applies the equivalent test
itself — `length <= 128`, `offset <= SHIMMER_CALIB_RAM_MAX - 1`,
`length + offset <= SHIMMER_CALIB_RAM_MAX`, returning `0xFF` otherwise
(`Calibration/shimmer_calibration.c:332-340`). With the 131-byte `args[]`
truncation clamp (`:559-577`) in front of both, an oversized or misaligned
write is contained on either path. The difference is where the check lives, not
whether there is one: a host reading only the handler will not find it.

#### `UPD_CALIB_DUMP_COMMAND` (0x9B)

- **Request:** `[0x9B]`
- **Response:** `[ACK]`

Re-applies the current calibration RAM to the configuration bytes and SD header,
and queues the on-card calibration file for rewrite. This is *not* required after
a normal `SET_CALIB_DUMP_COMMAND` sequence — step 3 above already does it once
the declared length is reached. Use it to force a re-apply, for example after
writing calibration bytes indirectly through `SET_INFOMEM_COMMAND`.

> `Comms/shimmer_bt_uart.c:1165-1170`.

#### Per-sensor calibration (six GET/SET pairs)

Each pair moves one 21-byte kinematic calibration block
(`SC_DATA_LEN_STD_IMU_CALIB`, `Calibration/shimmer_calibration.h:119`).

| Sensor | SET | RSP | GET | Shimmer3 part | Shimmer3R part |
|---|---|---|---|---|---|
| Low-noise accel | `0x11` | `0x12` | `0x13` | analog accel | LSM6DSV accel |
| Gyroscope | `0x14` | `0x15` | `0x16` | MPU9x50 / ICM-20948 | LSM6DSV gyro |
| Magnetometer | `0x17` | `0x18` | `0x19` | LSM303 mag | LIS2MDL mag |
| Wide-range accel | `0x1A` | `0x1B` | `0x1C` | LSM303 accel | LIS2DW12 accel |
| Alternative accel | `0xA9` | `0xAA` | `0xAB` | MPU9x50 / ICM-20948 accel | ADXL371 accel |
| Alternative mag | `0xAF` | `0xB0` | `0xB1` | MPU9x50 / ICM-20948 mag | LIS3MDL mag |

- **GET request:** `[opcode]` (no arguments)
- **GET response:** `[ACK][rspOpcode][21 bytes]`
- **SET request:** `[opcode][21 bytes]`
- **SET response:** `[ACK]`

The same opcode addresses a **different physical sensor on each generation** —
the table above is the whole story, and a host that has not read
`GET_DEVICE_VERSION_COMMAND` cannot label the block it just fetched. The read
path also stamps the block's range field from live configuration, so the reply
describes calibration *for the range currently configured*.

> Sensor-ID selection for the write path is at
> `Comms/shimmer_bt_uart.c:1183-1232` and `:1454-1479`; the read path, including
> the per-sensor range stamping, is `ShimBt_replySingleSensorCalibCmd` at
> `:1678-1751`. The response opcode is looked up by
> `ShimBt_getExpectedRspForGetCmd` (`:2351-2370`), which is why the generated
> table shows these rows as `(dynamic: ShimBt_getExpectedRspForGetCmd())`.

A SET writes through to the configuration image, flushes those 21 InfoMem bytes,
updates the SD header, syncs the block into calibration RAM, and queues the
on-card calibration file for rewrite.

> `ShimBt_calibrationChangeCommon`, `Comms/shimmer_bt_uart.c:1649-1661`.

#### `GET_ALL_CALIBRATION_COMMAND` (0x2C)

- **Request:** `[0x2C]`
- **Response:** `[ACK][0x2D][blocks]`

Concatenates the per-sensor blocks with no separators and no count:

| Generation | Blocks | Payload bytes |
|---|---|---|
| Shimmer3 | low-noise accel, gyro, mag, wide-range accel | 4 × 21 = **84** |
| Shimmer3R | the same four, then alternative accel, alternative mag | 6 × 21 = **126** |

> `Comms/shimmer_bt_uart.c:2171-2195`. The two extra calls are inside
> `#if defined(SHIMMER3R)` at `:2187-2193`.

The payload length is the only thing that distinguishes the two shapes, so a host
must know the generation before parsing. Prefer the calibration dump for a full
read: it is self-describing.

#### `RESET_CALIBRATION_VALUE_COMMAND` (0x5B)

- **Request:** `[0x5B]`
- **Response:** `[ACK]`

Re-seeds calibration RAM with the firmware's built-in defaults
(`ShimCalib_init()`), re-applies it to the configuration bytes and SD header, and
queues the on-card file for rewrite. **This discards per-unit factory
calibration** — there is no undo and no confirmation step. Blocked while
sensing.

> `Comms/shimmer_bt_uart.c:1301-1307`.

#### Pressure-sensor coefficients

The barometric sensor's calibration coefficients are read straight off the chip
and are not part of the calibration dump. Three commands expose them: one modern
sensor-agnostic command and two legacy per-part ones.

**`GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND` (0xA7)** — use this one.

- **Request:** `[0xA7]`
- **Response:** `[ACK][0xA6][1 + n][sensorId][coeffs × n]` — `2 + n` payload bytes

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | Length of what follows, i.e. `1 + n` |
| 1 | 1 | `sensorId` |
| 2..1+n | n | Raw coefficient bytes, exactly as read from the sensor |

| `sensorId` | Sensor | `n` |
|---|---|---|
| `0` | BMP180 | 22 |
| `1` | BMP280 | 24 |
| `2` | BMP390 | 21 |
| `3` | BMP581 | **0** |

> `Comms/shimmer_bt_uart.c:1988-2024`; the `PRESSURE_SENSOR_*` enumeration at
> `Comms/shimmer_bt_uart.h:280-286`. Lengths from
> `shimmer3-firmware` `Shimmer_Driver/BMPX80/bmpX80.h:99-100` (22, 24) and
> `shimmer3r-firmware` `Shimmer_Driver/BMP3/BMP3_SensorAPI/bmp3_defs.h:439`
> (`BMP3_LEN_CALIB_DATA` 21).

The BMP581 outputs pre-compensated data and therefore has no coefficients. The
firmware answers with the sensor-ID byte and **zero** coefficient bytes rather
than NACKing, deliberately: a NACK would be ambiguous with older firmware that
does not implement 0xA7 at all, whereas an in-band ID lets a host positively
identify the fitted part.

> The reasoning is in the firmware's own comment at
> `Comms/shimmer_bt_uart.c:1992-1996`.

On Shimmer3 only BMP180 and BMP280 drivers exist, so `sensorId` is 0 or 1 in
practice; the `PRESSURE_SENSOR_BMP390` fall-through is reachable but that
platform has no BMP390 driver and would report `n = 0`.

The payload is `2 + bmpCalibByteLen` on **both** generations: one length byte and
one sensor-ID byte ahead of the coefficients. Shimmer3 selects the ID from three
mutually exclusive branches, only one of which runs
(`Comms/shimmer_bt_uart.c:2004-2016`).

**`GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND` (0x59)** and
**`GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND` (0xA0)** — legacy, Shimmer3
only.

| Command | Response on Shimmer3 | Payload bytes | Response on Shimmer3R |
|---|---|---|---|
| `0x59` | `[ACK][0x58][coeffs]` | 22 | `[NACK]` |
| `0xA0` | `[ACK][0x9F][coeffs]` | 24 | `[NACK]` |

> `Comms/shimmer_bt_uart.c:1951-1968` (0x59), `:1969-1987` (0xA0). The Shimmer3R
> `sendNack = 1` at `:1965` and `:1984` is collapsed to a single NACK byte by the
> override at `:2334-2339`.

> **Caution.** On Shimmer3 these commands answer even when the requested part is
> **not** fitted: the payload is filled with the constant byte `0x01` repeated
> for the full length rather than being NACKed
> (`Comms/shimmer_bt_uart.c:1958-1962`, `:1977-1981`). A host cannot tell a
> genuine coefficient block from that filler except by its implausibility. Use
> `0xA7`, which reports the fitted sensor explicitly.

#### `GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND` (0x5D)

- **Request:** `[0x5D]`
- **Response, MPU-9x50 fitted (Shimmer3 only):** `[ACK][0x5C][adjX][adjY][adjZ]`
  — 3 payload bytes
- **Response, otherwise:** `[ACK][ACK]` — see the warning below

The three bytes are the AK8975 magnetometer's factory sensitivity-adjustment
values, read out of the MPU-9x50's embedded compass. The handler power-cycles the
MPU before reading them.

> `Comms/shimmer_bt_uart.c:2055-2076`.

⚠️ **On any board without an MPU-9x50 this command replies with a second ACK
instead of a NACK.** The feature does not exist in the ICM-20948, so on every
ICM-20948 Shimmer3 and on **every Shimmer3R** the `else` branch writes
`ACK_COMMAND_PROCESSED` where the response opcode should be. Since `sendAck` has
already written one ACK before the switch (`Comms/shimmer_bt_uart.c:1768-1772`),
the host receives `FF FF`.

Why this matters more than it looks:

- It is neither a NACK (which would mean "unsupported") nor a well-formed
  response, so a host waiting for `MPU9150_MAG_SENS_ADJ_VALS_RESPONSE` (0x5C)
  times out with an unexplained `0xFF` sitting in its stream.
- On an unframed transport the stray byte **desynchronises the parser
  permanently**. A host that length-frames the byte stream sees a leading `0xFF`
  as a 1-byte ACK, consumes the extra byte as a phantom ACK, and every subsequent
  ACK and response is off by one command.

The neighbouring `GET_BMP180`/`GET_BMP280_CALIBRATION_COEFFICIENTS` cases handle
the same situation correctly, setting `sendNack = 1` so the frame collapses to a
single NACK byte (`:1964-1966`, `:1983-1985`).

**Host guidance:** do not send 0x5D unless
`GET_DEVICE_VERSION_COMMAND` reported Shimmer3 **and** the daughter-card or
board identity establishes an MPU-9x50. If you must send it speculatively, treat
a second `0xFF` as "unsupported" and consume it, rather than letting it fall
through to your ACK accounting.

### 7.7 SD logging and trial configuration

These commands configure an autonomous SD-logging trial: what the unit is called,
which trial it belongs to, its role in a synchronised group, and the wall-clock
time it stamps data with. All of them except the two clock commands are blocked
while sensing.

#### `GET_TRIAL_CONFIG_COMMAND` (0x75) / `SET_TRIAL_CONFIG_COMMAND` (0x73)

- **GET request:** `[0x75]`
- **GET response:** `[ACK][0x74][cfg0][cfg1][interval]` — 3 payload bytes
- **SET request:** `[0x73][cfg0][cfg1][interval]`
- **SET response:** `[ACK]`

The three bytes are InfoMem 217, 218 and 219 (`NV_SD_TRIAL_CONFIG0`,
`NV_SD_TRIAL_CONFIG1`, `NV_SD_BT_INTERVAL`) — the trial flag bytes and the
sync Bluetooth interval in seconds. Individual bit meanings are in
[SHIMMER3_CONFIGURATION_INFOMEM.md §8](SHIMMER3_CONFIGURATION_INFOMEM.md#8-sd-logging-and-trial-configuration).

The write is a straight raw-byte copy of all three, then a flush of InfoMem
217-219 and the matching SD-header field, via `ShimBt_settingChangeCommon` —
which also runs the whole-image correction pass, so read back.

> `Comms/shimmer_bt_uart.c:980-989` (set), `:1861-1868` (get),
> `:1638-1647` (`ShimBt_settingChangeCommon`).

#### `GET_CENTER_COMMAND` (0x78) / `SET_CENTER_COMMAND` (0x76)

- **GET request:** `[0x78]`
- **GET response:** `[ACK][0x77][masterEnable]` — 1 payload byte
- **SET request:** `[0x76][len][data × len]`
- **SET response:** `[ACK]`

The *set* form is variable-length — the parser arms one byte, takes it as a
length, and then collects that many more
(`Comms/shimmer_bt_uart.c:500-511`) — but the handler reads only
`args[0] & 0x01`, the master-enable bit. Send `[0x76][0x01][0x00 or 0x01]`.

⚠️ **`SET_CENTER_COMMAND` does not survive a reboot.** The handler writes
`masterEnable` into the RAM image correctly, but then asks for the wrong InfoMem
byte to be flushed:

```c
case SET_CENTER_COMMAND:
{
  storedConfigPtr->masterEnable = args[0] & 0x01;
  ShimBt_settingChangeCommon(NV_CONFIG_SETUP_BYTE4, SDH_CONFIG_SETUP_BYTE4, 1);
  break;
}
```

> `Comms/shimmer_bt_uart.c:990-995`. `masterEnable` is bit 1 of the
> `SDTrialConfig0` bitfield group (`Configuration/shimmer_config.h:381-383`), which
> the struct layout places at InfoMem **217** (`NV_SD_TRIAL_CONFIG0`, i.e.
> `128 + 89`). The flush is directed at InfoMem **130**
> (`NV_CONFIG_SETUP_BYTE4`).

Observed consequences:

1. `GET_CENTER_COMMAND` reads back the new value for the rest of the session,
   because it reads RAM (`Comms/shimmer_bt_uart.c:1869-1873`). The command
   *appears* to work.
2. InfoMem 217 is never written, so **the setting is lost on the next boot**.
3. InfoMem 130 is flushed instead, rewriting the alternative-accel rate and the
   pressure-oversampling / low-power-mode / gyro-range MSB bits with their
   current RAM values.
4. The SD-header mirror is updated at the wrong offset for the same reason.

**Host workaround:** set the master-enable bit through
`SET_TRIAL_CONFIG_COMMAND` instead, which writes InfoMem 217 correctly
(`:980-989`) and is read back by `GET_TRIAL_CONFIG_COMMAND`. A host that must
know whether the setting persisted should verify with `GET_TRIAL_CONFIG_COMMAND`
or an InfoMem read, never with `GET_CENTER_COMMAND`.

#### `GET_SHIMMERNAME_COMMAND` (0x7B) / `SET_SHIMMERNAME_COMMAND` (0x79)

- **GET request:** `[0x7B]`
- **GET response:** `[ACK][0x7A][len][ascii × len]` — `1 + len` payload bytes
- **SET request:** `[0x79][len][ascii × len]`
- **SET response:** `[ACK]`

Stored in 12 InfoMem bytes at 187 (`NV_SD_SHIMMER_NAME`), not
null-terminated. Longer input is silently truncated to 12 bytes
(`ShimConfig_shimmerNameSet`, `Configuration/shimmer_config.c`). The read
returns the leading run of printable characters, at most 12; if the first byte is
not printable the firmware **regenerates the default name** `Shimmer_XXXX`
(where `XXXX` is the last four MAC hex digits) and returns that instead of an
empty string.

> `Comms/shimmer_bt_uart.c:996-1003` (set), `:1875-1884` (get);
> `ShimConfig_parseShimmerNameFromConfigBytes` and
> `ShimConfig_setDefaultShimmerName` in `Configuration/shimmer_config.c:244-245,782-792`.

A zero-length write (`[0x79][0x00]`) is dispatched and blanks the field, which
means the very next read returns the regenerated default — a blank name is not
representable.

Unlike most setters this one writes InfoMem directly and does **not** call
`ShimBt_settingChangeCommon`, so the whole-image correction pass does not run.

#### `GET_EXPID_COMMAND` (0x7E) / `SET_EXPID_COMMAND` (0x7C)

Identical shape to the Shimmer-name pair: 12 InfoMem bytes at 199
(`NV_SD_EXP_ID_NAME`), same truncation, same printable-prefix read, default
`DefaultTrial`.

- **GET response:** `[ACK][0x7D][len][ascii × len]`
- **SET request:** `[0x7C][len][ascii × len]`

> `Comms/shimmer_bt_uart.c:1004-1011`, `:1885-1894`;
> `Configuration/shimmer_config.c:250`.

#### `GET_CONFIGTIME_COMMAND` (0x87) / `SET_CONFIGTIME_COMMAND` (0x85)

- **GET request:** `[0x87]`
- **GET response:** `[ACK][0x86][len][ascii × len]` — `1 + len` payload bytes
- **SET request:** `[0x85][len][ascii × len]`
- **SET response:** `[ACK]`

The configuration timestamp is stored as a **4-byte big-endian** integer at
InfoMem 211 (`NV_SD_CONFIG_TIME`), but crosses the wire as a **decimal ASCII
string**. The write parses it with `atol` after copying at most 10 characters;
the read formats the stored integer back to decimal ASCII. There is no
null terminator on the wire in either direction.

> `Comms/shimmer_bt_uart.c:1012-1019` (set), `:1895-1904` (get);
> `ShimConfig_configTimeSetFromStr` in `Configuration/shimmer_config.c`;
> byte order at `:261,270`.

The value is a host-chosen epoch-seconds stamp identifying the configuration
generation. Nothing in the firmware validates it or compares it to the real-world
clock.

#### `GET_MYID_COMMAND` (0x81) / `SET_MYID_COMMAND` (0x7F)

- **GET response:** `[ACK][0x80][myTrialID]` — 1 payload byte
- **SET request:** `[0x7F][myTrialID]`

The unit's index within a synchronised trial. InfoMem 215
(`NV_SD_MYTRIAL_ID`). Stored verbatim — no range check.

> `Comms/shimmer_bt_uart.c:1026-1031`, `:1922-1927`.

#### `GET_NSHIMMER_COMMAND` (0x84) / `SET_NSHIMMER_COMMAND` (0x82)

- **GET response:** `[ACK][0x83][numberOfShimmers]` — 1 payload byte
- **SET request:** `[0x82][numberOfShimmers]`

The number of units in the trial. InfoMem 216 (`NV_SD_NSHIMMER`). Stored
verbatim.

> `Comms/shimmer_bt_uart.c:1020-1025`, `:1916-1921`.

#### `GET_RWC_COMMAND` (0x91) / `SET_RWC_COMMAND` (0x8F)

- **GET request:** `[0x91]`
- **GET response:** `[ACK][0x90][ticks × 8]` — 8 payload bytes
- **SET request:** `[0x8F][ticks × 8]`
- **SET response:** `[ACK]`

The real-world clock is a **64-bit little-endian count of 32768 Hz ticks**.
Milliseconds = ticks / 32.768.

> `Comms/shimmer_bt_uart.c:1439-1453` (set), `:2161-2170` (get). The Java driver
> encodes with `UtilShimmer.convertMilliSecondsToShimmerRtcDataBytesLSB`
> (`bluetooth/ShimmerBluetooth.java:691-696`) and decodes by reversing the eight
> bytes and dividing by 32.768 (`:1721-1728`).

Setting the clock has three side effects beyond the clock itself: it sets the
`rtcSetByBt` bit in InfoMem 217 and flushes that byte, re-runs the real-world
clock error check, and on Shimmer3R re-arms the periodic battery-read RTC alarm.
Once set, status bit 2 reports "clock has been set" on firmware new enough to
carry it (`isSupportedRtcStateInStatus`,
[Appendix A](#appendix-a-firmware-version-gates)).

> **Caution — the epoch is a host convention, not a firmware one.** The firmware
> stores and returns whatever 64-bit tick value it was given; it has no notion of
> time zone or of UTC. Host software has settled on **local civil time**, because
> that is what makes logged data split at local midnight. A host that writes UTC
> will produce files whose day boundaries do not match the operator's calendar.
> Nothing in the firmware detects or corrects this.

`SET_RWC_COMMAND` is **not** blocked while sensing — it is one of the few writes
permitted mid-recording ([§10](#10-important-boundaries)) — and its handler is
deliberately reached as early as possible after the argument bytes arrive, to
minimise the delay between the host reading its own clock and the device
adopting the value.

#### `UPD_SDLOG_CFG_COMMAND` (0x9C)

- **Request:** `[0x9C]`
- **Response:** `[ACK]`

Queues a rewrite of the SD card's `sdlog.cfg` from the current configuration
image (`TASK_SDLOG_CFG_UPDATE`). Blocked while sensing.

> `Comms/shimmer_bt_uart.c:1171-1175`.

Most configuration writes already queue this rewrite themselves, so an explicit
call is only needed after writes that do not — principally a sequence of
`SET_INFOMEM_COMMAND` chunks where the host wants the card updated before it
disconnects. The command number is shared with the Shimmer4 SDK's
`SET_I2C_BATT_STATUS_FREQ_COMMAND` (`Comms/shimmer_bt_uart.h:194-200`), which is
why [§4.2](#42-sd--trial-configuration) shows both names; on LogAndStream only
`UPD_SDLOG_CFG_COMMAND` exists.

### 7.8 Sensor settings

Every sensor setting is reachable two ways: through `SET_INFOMEM_COMMAND`, and
through a dedicated command. The dedicated commands are narrower — each writes
one field, clamps it, flushes one to four InfoMem bytes and mirrors them into the
SD header — but they share the InfoMem path's behaviour, because they all funnel
through `ShimBt_settingChangeCommon`, which runs the whole-image correction pass
before flushing.

> `ShimBt_settingChangeCommon`, `Comms/shimmer_bt_uart.c:1638-1647`.

#### `SET_SENSORS_COMMAND` (0x08)

- **Request:** `[0x08][sensors0][sensors1][sensors2]`
- **Response:** `[ACK]`

Three raw bitmap bytes copied to InfoMem 3-5 (`NV_SENSORS0..2`). There is no
matching *get*: read the enabled set back from the inquiry response's channel
list, which is what actually determines the packet layout, or from
`GET_INFOMEM_COMMAND`. Bit assignments are in
[SHIMMER3_CONFIGURATION_INFOMEM.md §3](SHIMMER3_CONFIGURATION_INFOMEM.md#3-sampling-rate-buffer-size-and-sensor-enables-bytes-0-5).

> `Comms/shimmer_bt_uart.c:961-966`.

Shimmer3R has two further enable bytes at InfoMem 128-129 (`NV_SENSORS3`,
`NV_SENSORS4`) which this command **cannot reach** — they are writable only
through `SET_INFOMEM_COMMAND` at offset 128.

The correction pass resolves mutually exclusive channels rather than rejecting
them: GSR wins over the internal ADC channel it shares a pin with, the bridge
amplifier wins over its two ADC channels, a 24-bit ExG enable wins over the
16-bit enable for the same chip, and skin temperature or the resistance
amplifier force their shared ADC channel on. None of this is reported back.

> `ShimConfig_checkAndCorrectConfig`, `Configuration/shimmer_config.c:432-504`.

#### `SET_SAMPLING_RATE_COMMAND` (0x05) / `GET_SAMPLING_RATE_COMMAND` (0x03)

- **SET request:** `[0x05][divLo][divHi]`
- **GET response:** `[ACK][0x04][divLo][divHi]` — 2 payload bytes

The value is a **divider**, not a frequency: sample rate in Hz =
`32768 / divider`. Both platforms clock sampling from a nominal 32768 Hz source
(`samplingClockFreqGet()` returns `32768.0f` on each).

> `Comms/shimmer_bt_uart.c:1135-1141` (set), `:1812-1818` (get);
> `Configuration/shimmer_config.c:306,632-635`.

> **Caution.** The divider is stored with **no validation whatsoever** — the
> handler is a bare `storedConfigPtr->samplingRateTicks = *(uint16_t *) args;`
> and `ShimConfig_checkAndCorrectConfig` does not touch the field. A divider of
> `0` is accepted and stored. Hosts must range-check before sending.

#### `SET_CONFIG_SETUP_BYTES_COMMAND` (0x0E) / `GET_CONFIG_SETUP_BYTES_COMMAND` (0x10)

- **SET request:** `[0x0E][b0][b1][b2][b3]`
- **GET response:** `[ACK][0x0F][b0][b1][b2][b3]` — 4 payload bytes

Bulk access to InfoMem 6-9. On both generations this is 4 bytes only; Shimmer3R's
setup bytes 4-6 (InfoMem 130-132) are not covered — see the caution in
[§7.1](#71-inquiry).

> `Comms/shimmer_bt_uart.c:1129-1134`, `:2077-2084`.

#### Per-setting triplets

Each row is a `SET` / response-opcode / `GET` triple. All the `SET`s are blocked
while sensing; none of the `GET`s are.

| Setting | SET | RSP | GET | InfoMem byte(s) flushed | Shimmer3 target | Shimmer3R target |
|---|---|---|---|---|---|---|
| Wide-range accel range | `0x09` | `0x0A` | `0x0B` | 6 | LSM303 accel range | LIS2DW12 accel range |
| Wide-range accel rate | `0x40` | `0x41` | `0x42` | 6 | LSM303DLHC ODR | LIS2DW12 ODR |
| Wide-range accel low-power mode | `0x43` | `0x44` | `0x45` | 6 | LSM303DLHC LP | LIS2DW12 LP |
| Wide-range accel high-resolution mode | `0x46` | `0x47` | `0x48` | 6 | LSM303DLHC HR | LIS2DW12 HR |
| Gyroscope rate | `0x4C` | `0x4D` | `0x4E` | 7 | MPU9x50 sample-rate divider | LSM6DSV ODR |
| Magnetometer gain / range | `0x37` | `0x38` | `0x39` | 8 | LSM303 mag range | **LIS3MDL alternative-mag range** |
| Magnetometer rate | `0x3A` | `0x3B` | `0x3C` | 8 | LSM303DLHC mag ODR | LIS2MDL ODR |
| Gyroscope range | `0x49` | `0x4A` | `0x4B` | 8 | MPU9x50 gyro range | LSM6DSV gyro range (3 bits) |
| Alternative accel range | `0x4F` | `0x50` | `0x51` | 9 | MPU9x50 accel range | **LSM6DSV low-noise accel range** |
| Pressure oversampling ratio | `0x52` | `0x53` | `0x54` | 9 | BMP180/BMP280 OSS | BMP390/BMP581 OSR (3 bits) |
| GSR range | `0x21` | `0x22` | `0x23` | 9 | | |
| Internal expansion power | `0x5E` | `0x5F` | `0x60` | 9 | | |
| Alternative accel rate | `0xAC` | `0xAD` | `0xAE` | 130 | | ADXL371 rate |
| Alternative mag rate | `0xB2` | `0xB3` | `0xB4` | 131 | *unused, forced to 0* | LIS3MDL ODR |

- **SET request:** `[setOpcode][value]` → `[ACK]`
- **GET response:** `[ACK][rspOpcode][value]` — 1 payload byte

> Handlers at `Comms/shimmer_bt_uart.c:1032-1037, 1053-1128, 1233-1238,
> 1480-1491`; responses at `:1819-1840, 1928-1945, 2025-2053, 2094-2099,
> 2209-2220`.

⚠️ **Two opcodes address a different sensor on Shimmer3R than their name
suggests.** `SET`/`GET_MAG_GAIN_COMMAND` (0x37/0x39) writes and reads
`altMagRange` — the LIS3MDL *alternative* magnetometer — not the primary LIS2MDL
(`Comms/shimmer_bt_uart.c:1065-1075`, `:1825-1834`). `SET`/`GET_ALT_ACCEL_RANGE_COMMAND`
(0x4F/0x51) writes and reads `lnAccelRange` — the LSM6DSV *low-noise* accel, the
primary one — not an alternative part (`:1106-1116`, `:2031-2041`). Both are
consistent between the set and the get, so a host that only round-trips values
will not notice; a host that labels the value for a user will mislabel it. The
Java driver preserves the legacy Shimmer3 names for these opcodes, which is why
[§4.1](#41-core) shows aliases such as `SET_LSM6DSV_GYRO_RANGE_COMMAND` for
0x49.

> **Every setter clamps silently.** Out-of-range values are replaced with a
> hard-coded fallback and then ACKed as if accepted — for example a
> wide-range-accel rate above the sensor's maximum becomes 100 Hz, a GSR range
> above 4 becomes auto-range, a gyro range above the maximum becomes 500 dps,
> and a pressure oversampling ratio above the fitted sensor's maximum becomes
> no-oversampling. There is no NACK and no indication in the ACK.
> `Configuration/shimmer_config.c:309-427` (`ShimConfig_gyroRangeSet`,
> `gyroRateSet`, `configBytePressureOversamplingRatioSet`,
> `configByteMagRateSet`, `configByteAltMagRateSet`),
> `Comms/shimmer_bt_uart.c:1034,1056-1060,1069-1071,1109-1112,1235`.
> **Always read the value back.**

#### `GET_BUFFER_SIZE_COMMAND` (0x36)

- **Request:** `[0x36]`
- **Response:** `[ACK][0x35][bufferSize]` — 1 payload byte

Returns InfoMem 2, which current firmware fixes at `1` (one sample per data
packet). There is no `SET` — the Java driver's `SET_BUFFER_SIZE_COMMAND` (0x34)
is a legacy opcode this firmware does not define, see
[Appendix B](#appendix-b-java-only-and-legacy-opcodes).

> `Comms/shimmer_bt_uart.c:2124-2129`; `Configuration/shimmer_config.c:155`.

### 7.9 ExG registers

Direct access to the two ADS1292R analogue front-end chips' register banks, ten
registers each, mirrored in InfoMem at 10-19 (chip 0) and 20-29 (chip 1).

#### `GET_EXG_REGS_COMMAND` (0x63)

- **Request:** `[0x63][chip][startAddr][len]`
- **Response:** `[ACK][0x62][len][data × len]` — `1 + len` payload bytes

| Field | Valid range |
|---|---|
| `chip` | 0 or 1 |
| `startAddr` | 0..9 |
| `len` | 0..10 |

The values are read from the InfoMem mirror, not from the chip over SPI, so they
reflect what the firmware believes it configured.

> `Comms/shimmer_bt_uart.c:1038-1052` (validation), `:2221-2240` (response).

> **Caution.** On an invalid argument triple the firmware sets `len = 0` and
> still answers, so the reply is `[ACK][0x62][0x00]` — a well-formed
> zero-length read, not an error. A host cannot distinguish "you asked for zero
> bytes" from "your arguments were rejected". Note also that `startAddr + len`
> is **not** checked against 10; `startAddr = 9, len = 10` passes validation.

#### `SET_EXG_REGS_COMMAND` (0x61)

- **Request:** `[0x61][chip][startAddr][len][data × len]`
- **Response:** `[ACK]`

Note the length byte is the **third** argument here, not the first — this is the
one variable-length command whose in-band length sits at `args[2]` rather than
`args[0]` (`Comms/shimmer_bt_uart.c:477-480`). Same validation as the read; an
invalid triple writes nothing and still ACKs.

The handler writes the requested range into the InfoMem mirror, flushes exactly
those bytes, mirrors them into the SD header, and queues the SD configuration
file for rewrite.

> `Comms/shimmer_bt_uart.c:1239-1273`.

**The SR47-4 clock-line fix.** On an EXG-unified expansion board of major
revision 4 or greater, bit 3 of chip 0's `CONFIG2` register is forced to 1 after
the host's bytes are applied, because that revision ties the ADS1292R clock
lines together and the bit is required for correct clocking.

> `Comms/shimmer_bt_uart.c:1255-1263`; the same forcing is applied on every
> correction pass by `ShimConfig_checkAndCorrectConfig`
> (`Configuration/shimmer_config.c:541-547`), keyed on
> `ShimBrd_areADS1292RClockLinesTied()`.

> **Caution.** The forced bit is set in RAM, but the flush that follows writes
> only the host's requested range (`InfoMem_write(exgConfigOffset + exgStartAddr,
> …, exgLength)`, `Comms/shimmer_bt_uart.c:1265-1266`). A write to chip 0 that
> does not include `CONFIG2` (InfoMem 11, i.e. `startAddr` 1) therefore leaves
> the forced bit unpersisted until some later write or correction pass flushes
> that byte. Hosts writing chip 0 should write the whole 10-byte bank in one
> command — `[0x61][0x00][0x00][0x0A][…]` — and read it back.

### 7.10 Daughter card

Two windows onto the same 2048-byte CAT24C16 EEPROM on the expansion board.
Page 0 (absolute bytes 0-15) is the daughter-card identity page; the *daughter
card memory* commands address everything above it, with host offset 0 mapping to
absolute byte 16.

> `EEPROM/shimmer_eeprom.h:15-32`; `CAT24C16_TOTAL_SIZE` 2048 and
> `CAT24C16_PAGE_SIZE` 16 from the platform `CAT24C16/CAT24C16.h`.
> The `+ 16` offset is applied at `Comms/shimmer_bt_uart.c:2275` (read) and
> inside `ShimEeprom_writeDaughterCardMem` (write).

#### Identity page — `GET_DAUGHTER_CARD_ID_COMMAND` (0x66) / `SET_DAUGHTER_CARD_ID_COMMAND` (0x64)

- **GET request:** `[0x66][len][offset]`
- **GET response:** `[ACK][0x65][len][data × len]` — `1 + len` payload bytes
- **SET request:** `[0x64][len][offset][data × len]`
- **SET response:** `[ACK]`

Note `offset` is a **single byte** here, not the little-endian pair the memory
commands use. Validation is `len <= 16 && offset <= 15 && len + offset <= 16` on
both directions. The read comes from the firmware's cached, parsed identity page
rather than from the EEPROM, and a write updates both the EEPROM and that cache
so a read-back immediately reflects it.

> `Comms/shimmer_bt_uart.c:1308-1330` (both handlers), `:2261-2270` (response).

The page holds the expansion-board ID, major and minor revision that the
firmware keys hardware-dependent behaviour on — including the SR47-4 ExG fix in
[§7.9](#79-exg-registers) — so writing it wrongly changes how the firmware
drives the board. Board identities are catalogued in
[SHIMMER3_BOARD_REVISIONS.md](SHIMMER3_BOARD_REVISIONS.md).

> **Caution — never send `len = 0` to `SET_DAUGHTER_CARD_ID_COMMAND`.** The
> argument-collection branch for this opcode reads the two fixed bytes, and if
> `args[0]` is zero it returns without arming for more data **and without
> dispatching the command** (`Comms/shimmer_bt_uart.c:488-499`). The result is
> no response at all and a parser left mid-command, which will consume the next
> two host bytes as a fresh length/offset pair. This is the only variable-length
> command with that shape; the name/expId/configTime group falls through and
> dispatches with a zero length instead (`:500-511`).

#### User area — `GET_DAUGHTER_CARD_MEM_COMMAND` (0x69) / `SET_DAUGHTER_CARD_MEM_COMMAND` (0x67)

- **GET request:** `[0x69][len][offsetLo][offsetHi]`
- **GET response:** `[ACK][0x68][len][data × len]` — `1 + len` payload bytes
- **SET request:** `[0x67][len][offsetLo][offsetHi][data × len]`
- **SET response:** `[ACK]` on success, `[NACK]` on a rejected write

Validation is `len <= 128 && offset <= 2031 && len + offset <= 2032`. The read
path performs the check in the command handler; the write path performs the
equivalent check inside `ShimEeprom_writeDaughterCardMem` and — unlike almost
every other rejected write in this protocol — genuinely **NACKs** on failure.

> `Comms/shimmer_bt_uart.c:1331-1351`, `:2271-2278`;
> `EEPROM/shimmer_eeprom.c:393-403`.

A rejected *read*, by contrast, follows the usual pattern: bare ACK, no
`DAUGHTER_CARD_MEM_RESPONSE`.

**Two regions inside the user area have side effects when written.** Writing a
range that overlaps the radio-settings byte makes the firmware re-read its
sensor-settings page (which is how the classic-Bluetooth / BLE enable bits are
changed at runtime); writing a range that overlaps the 80-byte branding record at
host offset 1936 makes it re-read the branding details. Neither is re-seeded
mid-write, so a multi-chunk host write is safe, and new advertising names take
effect at the next Bluetooth initialisation — which `SET_FEATURE`'s
reboot-on-disconnect option ([§7.12](#712-control-and-test)) exists to trigger.

> `EEPROM/shimmer_eeprom.c:405-418`.

⚠️ **Java driver gap.** Three of these four opcodes (0x67, 0x68, 0x69) plus
`SET_DAUGHTER_CARD_ID_COMMAND` (0x64) have no named constant in the Java driver
— flagged `FW_ONLY` in [§4.1](#41-core). Only `GET_DAUGHTER_CARD_ID_COMMAND` and
its response are named there.

### 7.11 Derived channels

- **SET request:** `[0x6D][b0][b1][b2][b3][b4][b5][b6][b7]` — 8 fixed argument bytes
- **SET response:** `[ACK]`
- **GET request:** `[0x6F]`
- **GET response:** `[ACK][0x6E][b0..b7]` — **8** payload bytes

The eight bytes are stored in **two non-contiguous InfoMem regions**: bytes 0-2
go to InfoMem 31-33 (`NV_DERIVED_CHANNELS_0..2`) and bytes 3-7 to InfoMem 118-122
(`NV_DERIVED_CHANNELS_3..7`). The split is historical — the original three bytes
sat immediately below the calibration block, and the five later ones had to go
above it. The command hides the split: a host always sees eight contiguous
bytes.

> `Comms/shimmer_bt_uart.c:1368-1382` (set — two `memcpy`s, two `InfoMem_write`s,
> two SD-header writes), `:2152-2160` (get — two `ShimConfig_storedConfigGet`
> calls); offsets at `Configuration/shimmer_config.h:99,107`.

The firmware **only stores these bytes**. Nothing in LogAndStream reads them to
change sampling, channel selection or packet content — they are flags describing
which derived signals the *host's* processing chain should compute, carried in
the device configuration so that the recipe travels with the unit and lands in
the SD file header. Bit meanings are a host-software contract, not a firmware
one; see
[SHIMMER3_CONFIGURATION_INFOMEM.md §6](SHIMMER3_CONFIGURATION_INFOMEM.md#6-bluetooth-baud-and-derived-channels).

⚠️ **The Java registry declares the wrong response length.** `BtCommandDetails`
gives `DERIVED_CHANNEL_BYTES_RESPONSE` (0x6E) **3** payload bytes; the firmware
emits **8**. The firmware is correct — `Comms/shimmer_bt_uart.c:2152-2160` writes
3 bytes from `NV_DERIVED_CHANNELS_0` followed by 5 from `NV_DERIVED_CHANNELS_3`.
The registry length is used only by the Java driver's blocking-read path, so the
consequence is a host-side truncation that leaves five bytes in the receive
buffer, not a firmware fault. This is the **only** length disagreement between
the three sources ([§4.1](#41-core), `LEN_MISMATCH`). New hosts must expect 8.

The Java driver gates the 5-byte extension on
`isSupportedEightByteDerivedSensors` (LogAndStream 0.7.1,
[Appendix A](#appendix-a-firmware-version-gates)), so on older firmware only the
first three bytes are meaningful — but the firmware in this repository always
sends eight.

### 7.12 Control and test

#### `SET_CRC_COMMAND` (0x8B)

- **Request:** `[0x8B][mode]`
- **Response:** `[ACK]` — with the new CRC mode already applied

Sets the session CRC mode: `0` off, `1` one byte, `2` two bytes. Any value of 3
or more falls back to **off** rather than being rejected. See
[§3.3](#33-crc-modes) for the algorithm and what the CRC covers. Not blocked
while sensing.

> `Comms/shimmer_bt_uart.c:933-937`; `ShimBt_setCrcMode` at `:2372-2382`.

#### `SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE` (0xA3)

- **Request:** `[0xA3][state]`
- **Response:** `[ACK]`

Controls whether **unsolicited** in-stream status pushes carry a leading ACK
byte. Non-zero (the default) prefixes them; zero does not. It does not affect
solicited responses, which always carry an ACK. See
[§5](#5-status-acknack-and-in-stream-responses).

> `Comms/shimmer_bt_uart.c:938-942`; the flag is consumed at `:2454-2457` and
> defaulted to 1 by `ShimBt_resetBtResponseVars` (`:191-201`), which runs on
> every disconnect (`:2545`).

⚠️ **No Java constant exists for 0x A3** (`FW_ONLY`, [§4.1](#41-core)); the Java
driver's in-stream parser assumes the prefix is present. Hosts that disable the
prefix must be sure their own parser expects that.

#### `SET_DATA_RATE_TEST` (0xA4)

- **Request:** `[0xA4][enable]`
- **Response:** `[ACK]`, then a continuous stream of test packets while enabled

Test packet: `[0xA5][counter u32 LE]` — 5 bytes
(`DATA_RATE_TEST_PACKET_SIZE`), with a counter that increments once per
transmitted packet.

The ordering is deliberate and asymmetric: **stopping** takes effect *before* the
ACK is sent (and flushes the transmit buffer, so queued test packets are
discarded), while **starting** takes effect *after* the ACK has been queued, so
the host reliably sees the ACK first.

> `Comms/shimmer_bt_uart.c:1274-1284` (stop path), `:2251-2260` (start path),
> `ShimBt_loadTxBufForDataRateTest` at `:2867-2902`;
> `DATA_RATE_TEST_PACKET_SIZE` at `Comms/shimmer_bt_uart.h:258`.

Test packets are **not** ACK-prefixed, are **not** wrapped in
`INSTREAM_CMD_RESPONSE`, and carry **no CRC** regardless of the session CRC
mode. They saturate the link deliberately: on Shimmer3R the packets are written
straight to the UART, bypassing the transmit ring. Blocked while sensing. A
Shimmer3-only watchdog declares a blockage if the counter has not advanced for
more than 2 seconds (`ShimBt_checkForBtDataRateTestBlockage`, `:3070-3093`).

Send `[0xA4][0x00]` to stop. A disconnect also stops it
(`ShimBt_handleBtRfCommStateChange`, `:2536`).

#### `SET_FACTORY_TEST` (0xA8)

- **Request:** `[0xA8][testId]`
- **Response:** `[ACK]`, then **free-form human-readable text**

| `testId` | Test |
|---|---|
| `0` | `FACTORY_TEST_MAIN` |
| `1` | `FACTORY_TEST_LEDS` |
| `2` | `FACTORY_TEST_ICS` |
| `3` | `FACTORY_TEST_LED_STATES` |

> `Comms/shimmer_bt_uart.c:1285-1293`; the enum is `factory_test_t` in
> `Test/shimmer_test.h:20-27`, and `FACTORY_TEST_COUNT` (4) is the bound.

> **Caution.** The test's output is printed to the Bluetooth UART as plain text
> (`PRINT_TO_BT_UART`), **not** as protocol frames. Everything a host's command
> parser sees after the ACK is unframed diagnostic text of unpredictable length,
> which will desynchronise a state machine keyed on opcodes. Treat the link as
> text-mode until the device is reconnected. An out-of-range `testId` is
> silently ACKed with no test run and no output. Blocked while sensing.

#### `SET_FEATURE` (0xB7)

- **Request:** `[0xB7][feature][value]`
- **Response:** `[ACK]`, or `[NACK]` for an unrecognised `feature`

| `feature` | Name | Platforms | Effect |
|---|---|---|---|
| `0` | `FEATURE_NONE` | both | Shimmer3: disables the RN4678 error LEDs. Shimmer3R: no effect. `value` ignored |
| `1` | `FEATURE_RN4678_ERROR_LEDS` | Shimmer3 only | Enables/disables the RN4678 error LEDs per `value`; silently ignored if the fitted module is not an RN4678 |
| `2` | `FEATURE_REBOOT_ON_DISCONNECT` | both | Arms (`value` non-zero) or disarms a one-shot soft reboot that fires when the host disconnects |

> `Comms/shimmer_bt_uart.c:1526-1556`; the enum at
> `Comms/shimmer_bt_uart.h:288-298`. On Shimmer3R, `feature = 1` reaches the
> final `else` and is **NACKed**, because the RN4678 branch is inside
> `#if defined(SHIMMER3)`.

`FEATURE_REBOOT_ON_DISCONNECT` exists so that a host can apply settings the
firmware only reads at boot — principally the EEPROM branding record's
advertising names ([§7.10](#710-daughter-card)) — without asking a user to
power-cycle the device by hand. The reboot cannot happen while the host is
connected, because the link has to drop for the Bluetooth module to re-read its
name. The request is strictly one-shot and is **skipped while sensing**, so an
armed reboot can never truncate an active recording; the flag is cleared either
way, so it never lingers into a later disconnect.

> `ShimBt_handleBtRfCommStateChange`, `Comms/shimmer_bt_uart.c:2562-2577`.

#### `RESET_BT_ERROR_COUNTS` (0xB6)

- **Request:** `[0xB6]`
- **Response:** `[ACK]` on Shimmer3 with an EEPROM fitted, `[NACK]` otherwise

Clears the persistent Bluetooth error counters and writes the sensor-settings
page back to EEPROM. **NACKed on all Shimmer3R units** — the handler body is
`sendNack = 1` outside `#if defined(SHIMMER3)` — and on any Shimmer3 without an
EEPROM. The counters themselves are the `BT_ERROR_*` bit set at
`Comms/shimmer_bt_uart.h:326-336`.

> `Comms/shimmer_bt_uart.c:1509-1525`.

#### `RESET_TO_DEFAULT_CONFIGURATION_COMMAND` (0x5A)

- **Request:** `[0x5A]`
- **Response:** `[ACK]`

Replaces the entire configuration image with the firmware's compiled-in
defaults, flushes it to InfoMem, rebuilds the SD header and queues the SD
configuration file for rewrite. Calibration is untouched — use
`RESET_CALIBRATION_VALUE_COMMAND` ([§7.6](#76-calibration-dump-and-per-sensor-calibration))
for that. Blocked while sensing.

> `Comms/shimmer_bt_uart.c:1294-1300`; `ShimConfig_setDefaultConfig` at
> `Configuration/shimmer_config.c:150-240`, whose own
> `LogAndStream_infomemUpdate()` at `:239` is what persists the result.

The Shimmer-name default is `Shimmer_XXXX` with the last four MAC hex digits
substituted; the trial-ID default is `DefaultTrial`; the default sample rate is
51.2 Hz. Full default values are in
[SHIMMER3_CONFIGURATION_INFOMEM.md §11](SHIMMER3_CONFIGURATION_INFOMEM.md#11-defaults).

#### `DUMMY_COMMAND` (0xB5)

- **Request:** `[0xB5]`
- **Response:** **none** — not even an ACK

The receive state machine consumes the byte, re-arms for the next command byte,
and returns without scheduling any processing. It is the only command handled
entirely inside `ShimBt_dmaConversionDone`.

> `Comms/shimmer_bt_uart.c:596-599`.

Its purpose is to give a host a byte that is guaranteed to be swallowed without
side effects — useful for flushing a transport's write buffer or for padding.
Use `TEST_CONNECTION_COMMAND` ([§7.3](#73-device-status)) when you want an
answer.

⚠️ **No Java constant exists for 0xB5** (`FW_ONLY`, [§4.1](#41-core)).

### 7.13 SD file transfer (Shimmer3R)

Eleven opcodes let a host walk the SD card and pull file content over the same
command channel. The design is deliberately **stateless on the host's behalf**:
the host walks the tree with `SD_LIST_DIR` and `SD_FILE_STAT`, then asks for
content one *window* at a time with `SD_FILE_READ`. Recovering from any
interruption is a fresh `SD_FILE_READ` from the last byte offset the host is sure
it holds. There is no session to re-establish.

> `Comms/shimmer_sd_file_transfer.h:1-22` states the design intent;
> `Comms/shimmer_sd_file_transfer.c` implements it. The host reference is
> `Extras/python_scripts/Shimmer_common/shimmer_comms_bluetooth.py:391-576`
> and `shimmer-web-sdk` `devices/shimmer3r/sdTransfer/protocol.ts`.

**Served on Shimmer3R only.** The opcodes are reserved protocol-wide, but every
handler sits behind `#if defined(SHIMMER3R)`. On Shimmer3 the command bytes fall
to the receive state machine's `default:` and are silently ignored — no ACK, no
NACK. Hosts must gate on `GET_FW_VERSION_COMMAND` and
`GET_DEVICE_VERSION_COMMAND`, not on probing.

> `Comms/shimmer_bt_uart.h:227-235`; the guarded arming cases at
> `Comms/shimmer_bt_uart.c:662-665, 698-701, 726-738`.

> **Opcode 0xCC is not a typo.** `SD_LIST_DIR_COMMAND` sits at `0xCC` while its
> response is `0xC1`, breaking the otherwise consecutive block, because the
> CYW20820's UART receive demultiplexer routes the EZ-Serial binary start-of-frame
> bytes `0x80`, `0xC0` and `0xD0` to the module's own parser rather than to the
> Shimmer command parser. `0xC0` would never have arrived. See
> [§2.3](#23-framing-guarantees-per-transport).

#### Access gate

Every command in this block is checked against the same gate before it acts, and
reports the verdict **in band** through a status byte rather than by timing out
or NACKing:

| Value | Constant | Meaning |
|---|---|---|
| `0x00` | `SD_FT_STATUS_OK` | Proceeding |
| `0x01`-`0x13` | — | Raw FatFs `FRESULT` code, passed through |
| `0xF0` | `SD_FT_STATUS_SD_UNAVAILABLE` | Docked, USB-C attached, the MCU does not own the card, no card, or a bad card |
| `0xF1` | `SD_FT_STATUS_BUSY` | Sensing, logging or streaming |
| `0xF2` | `SD_FT_STATUS_BAD_ARGS` | Path missing, empty, or longer than 96 bytes |

> `Comms/shimmer_sd_file_transfer.h:29-46`; `sdFtAccessCheck` in
> `Comms/shimmer_sd_file_transfer.c`.

The v1 policy is idle-only: transfers are served only when the device is not
sensing, logging or streaming, and the MCU owns the SD card (not docked, no
USB-C). If the card was powered down when the last Bluetooth session ended, the
gate performs a full card bring-up on first use, which costs a few hundred
milliseconds — so the first command of a session can be noticeably slower than
the rest.

#### Path arguments

Every path-bearing command carries the path length in its **last fixed argument
byte**, followed by that many ASCII bytes. Valid lengths are 1..96
(`SD_FT_MAX_PATH_LEN`). A zero or oversized length is not armed for: the command
proceeds with whatever was received and the handler reports
`SD_FT_STATUS_BAD_ARGS` in its response — never silence.

> `Comms/shimmer_bt_uart.c:512-531`; `sdFtCopyPathArg` in
> `Comms/shimmer_sd_file_transfer.c`.

#### `SD_LIST_DIR_COMMAND` (0xCC)

- **Request:** `[0xCC][startIdxLo][startIdxHi][maxEntries][pathLen][path × pathLen]`
- **Response:** `[ACK][0xC1][status][startIdx u16][entriesLen u16][nEntries][flags][entries…]`

`maxEntries` is clamped to 1..16 (`SD_FT_LIST_MAX_ENTRIES`); zero or oversized
becomes 16. The response header is 8 bytes including the opcode, then
`nEntries` variable-length entries:

| Offset in entry | Size | Field |
|---|---|---|
| 0 | 1 | `attr` — bit 0 `SD_FT_ATTR_DIR`, bit 1 `SD_FT_ATTR_NAME_TRUNCATED` |
| 1..4 | 4 | File size in bytes, `uint32_le` |
| 5..6 | 2 | FAT date, `uint16_le` |
| 7..8 | 2 | FAT time, `uint16_le` |
| 9 | 1 | `nameLen` |
| 10.. | `nameLen` | Name, ASCII, not null-terminated |

`flags` bit 0 means **more entries exist**: either `maxEntries` was reached or
the response hit its 240-byte budget. Page by re-issuing the command with
`startIdx` advanced by the `nEntries` already received. Names longer than 64
bytes (`SD_FT_LIST_NAME_MAX`) are truncated and flagged per entry. Dot entries
are skipped.

> `ShimSdFileTransfer_buildListDirRsp` and `ShimSdFileTransfer_stageListDir` in
> `Comms/shimmer_sd_file_transfer.c`; the 240-byte budget and its rationale at
> `:30-39`, chosen so that ACK + response + 2 CRC bytes stays under 253 and the
> same wire format could later serve Shimmer3's 133-byte response budget.

#### `SD_FILE_STAT_COMMAND` (0xC2)

- **Request:** `[0xC2][pathLen][path × pathLen]`
- **Response:** `[ACK][0xC3][status][size u32][fdate u16][ftime u16][attr]` — 11 payload bytes

On any failure the size, date and time fields are zero-filled and `status`
carries the reason.

#### `SD_FREE_SPACE_COMMAND` (0xC8)

- **Request:** `[0xC8]`
- **Response:** `[ACK][0xC9][status][freeKb u32][totalKb u32]` — 9 payload bytes

Both figures are in kilobytes, computed with 64-bit intermediates and saturated
at `0xFFFFFFFF` so a large exFAT card cannot overflow them. This command mounts
and interrogates the filesystem, so it can take seconds on a large card — the
Python reference allows a 20-second timeout
(`shimmer_comms_bluetooth.py:461`).

#### `SD_DELETE_COMMAND` (0xCA)

- **Request:** `[0xCA][pathLen][path × pathLen]`
- **Response:** `[ACK][0xCB][status]` — 1 payload byte

**Deletion is confined to the data directory.** The path must begin with
`data/` or `/data/` (case-insensitive), must name something inside it, and must
not contain `..`; anything else is `SD_FT_STATUS_BAD_ARGS`. Any cached read
handle is closed first, because FatFs with file locking enabled refuses to
unlink an open file (`FR_LOCKED`). `status` is otherwise the raw FatFs result —
`FR_OK` is `0`, which is also `SD_FT_STATUS_OK`.

> `sdFtIsDeletablePath` and `ShimSdFileTransfer_buildDeleteRsp` in
> `Comms/shimmer_sd_file_transfer.c`.

#### `SD_FILE_READ_COMMAND` (0xC4)

- **Request:** `[0xC4][offset u32][windowLen u32][blockPayloadLen u16][pathLen][path × pathLen]`
  — 11 fixed argument bytes, then the path
- **Immediate response:** `[ACK]` only

The window verdict then arrives **asynchronously**, as a run of data frames
followed by exactly one status frame. Nothing else is returned synchronously.

| Field | Meaning |
|---|---|
| `offset` | First byte of the file to send, `uint32_le` |
| `windowLen` | How many bytes to send, `uint32_le`. Saturates at the 32-bit end of the file |
| `blockPayloadLen` | Preferred payload bytes per data frame, `uint16_le` |
| `pathLen` / `path` | File to read |

`blockPayloadLen` is normalised: `0` becomes 512 (`SD_FT_BLOCK_PAYLOAD_DEFAULT`),
values below 64 become 64 (`SD_FT_BLOCK_PAYLOAD_MIN`), values above 1024 become
1024 (`SD_FT_BLOCK_PAYLOAD_MAX`), and the result is rounded **down to a multiple
of 4** so successive blocks keep the payload buffer aligned for FatFs's
whole-sector reads. The host reassembles from each frame's own length field, so
it is indifferent to the rounding.

> `ShimSdFileTransfer_startRead` in `Comms/shimmer_sd_file_transfer.c`; the
> constants at `Comms/shimmer_sd_file_transfer.h:52-57`.

Issuing a new `SD_FILE_READ_COMMAND` while a window is in flight **supersedes**
it: the old window is abandoned and a status frame with `SD_FT_XFER_SUPERSEDED`
is queued for the old session before the new one starts.

**Data frame** — `SD_FILE_DATA_RESPONSE` (0xC5):

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | `0x8A` `INSTREAM_CMD_RESPONSE` |
| 1 | 1 | `0xC5` |
| 2 | 1 | `sessionId` |
| 3..4 | 2 | `seq`, `uint16_le`, zero-based within the window |
| 5..6 | 2 | `len`, `uint16_le`, payload bytes in this frame |
| 7..7+len-1 | `len` | Payload |
| 7+len | 2 | CRC-16, `uint16_le`, over bytes 0..6+len |

**Status frame** — `SD_FILE_STATUS_RESPONSE` (0xC6), 10 bytes:

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | `0x8A` |
| 1 | 1 | `0xC6` |
| 2 | 1 | `sessionId` |
| 3 | 1 | `SD_FT_XFER_*` status |
| 4..7 | 4 | `nextOffset`, `uint32_le` — the file offset to resume from |
| 8..9 | 2 | CRC-16, `uint16_le`, over bytes 0..7 |

| Status | Constant | Meaning |
|---|---|---|
| `0` | `SD_FT_XFER_WINDOW_COMPLETE` | The requested window was delivered in full |
| `1` | `SD_FT_XFER_EOF` | End of file reached before the window was filled |
| `2` | `SD_FT_XFER_HOST_ABORT` | `SD_TRANSFER_ABORT_COMMAND` was received |
| `3` | `SD_FT_XFER_SD_LOST` | Card became unavailable |
| `4` | `SD_FT_XFER_FS_ERROR` | Filesystem read error |
| `5` | `SD_FT_XFER_SUPERSEDED` | A newer `SD_FILE_READ_COMMAND` replaced this window |
| `6` | `SD_FT_XFER_DENIED` | Access gate refused, or the path was invalid |
| `7` | `SD_FT_XFER_NOT_FOUND` | File not found |

> `Comms/shimmer_sd_file_transfer.h:38-63` for the codes and frame lengths;
> frame assembly at `Comms/shimmer_sd_file_transfer.c:305-311` (status) and
> `:596-604` (data).

> **These frames are framed differently from every other response.** They carry
> **no leading ACK byte**, and their trailing CRC-16 is **always present and
> independent of `SET_CRC_COMMAND`** — it is computed by the transfer module
> itself, not by the response assembler, because a bulk transfer needs integrity
> checking whether or not the session negotiated a CRC. A host must therefore
> parse `0x8A 0xC5` and `0x8A 0xC6` as self-delimiting frames, using the
> `len` field, and must not apply its session-CRC expectations to them.

**Pacing and interleaving.** Data frames are pushed from a low-priority task
that copies only what fits in the Bluetooth transmit ring per pass, keeping 256
bytes (`SD_FT_TX_RESERVE`) free so command responses and status frames always
have room. Pacing is inherited from the UART transmit-complete interrupt and the
module's flow control; up to 4 blocks (`SD_FT_BLOCKS_PER_PASS`) are pushed per
task invocation before yielding. **Command responses can therefore appear
interleaved between data frames**, and a host must keep parsing command replies
throughout a transfer.

#### `SD_TRANSFER_ABORT_COMMAND` (0xC7)

- **Request:** `[0xC7]`
- **Response:** `[ACK]`, then a status frame with `SD_FT_XFER_HOST_ABORT`

Tears down the active window and queues the status frame so the host learns the
resume offset. Safe to send in any state — a no-op when no window is active,
in which case no status frame follows.

> `ShimSdFileTransfer_abort`, declared at
> `Comms/shimmer_sd_file_transfer.h:76-79`.

A Bluetooth disconnect drops the transfer **silently** — there is no link to
report on — so a reconnecting host must resume from its own last-known offset
rather than expecting a status frame
(`Comms/shimmer_bt_uart.c:2538-2539`).

### 7.14 SD sync

SD sync distributes a common time base across a group of units logging
autonomously to their own cards: one unit is the **centre**, the rest are
**nodes**, and the centre connects to each node in turn over classic Bluetooth
to hand it the centre's clock. The two opcodes below are that exchange. Full
timing behaviour is in `SDSync/shimmer_sd_sync.{h,c}` and is outside this
document's scope.

**Sync mode is exclusive.** While `syncEnable` is set in the configuration, the
firmware NACKs every command except `SET_SD_SYNC_COMMAND` and
`ACK_COMMAND_PROCESSED` — and, symmetrically, NACKs those two when
`syncEnable` is clear. The check is a single XOR at the top of
`ShimBt_processCmd`:

```c
if (storedConfigPtr->syncEnable ^ ShimBt_isCmdAllowedWhileSdSyncing(gAction))
{
  sendNack = 1;
}
```

> `Comms/shimmer_bt_uart.c:830-834`; `ShimBt_isCmdAllowedWhileSdSyncing` at
> `:2934-2937`.

A host connecting to a sync-enabled unit therefore cannot do anything at all —
not even read the firmware version. To configure such a unit, clear `syncEnable`
(InfoMem 217 bit 2) while it is not in sync mode. This is the single most
common cause of a device that "connects but NACKs everything".

#### Centre → node: `SET_SD_SYNC_COMMAND` (0xE0)

- **Request:** `[0xE0][flag][time × 8][crc × 1]` — 10 argument bytes
- **Response:** `[ACK][0xE1][flag]` from the node, assembled by the sync module

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | Status / flag byte (`SYNC_PACKET_FLG_IDX`) |
| 1..8 | 8 | Centre's clock, 32768 Hz ticks, little-endian (`SYNC_PACKET_TIME_IDX`) |
| 9 | 1 | CRC, 1 byte — **always present**, `BT_SD_SYNC_CRC_MODE` is fixed at `CRC_1BYTE_ENABLED` |

The argument count the parser arms is
`SYNC_PACKET_PAYLOAD_SIZE + BT_SD_SYNC_CRC_MODE` = 9 + 1 = 10, which is why
[§4.2](#42-sd--trial-configuration) shows it symbolically. The sync CRC is
independent of `SET_CRC_COMMAND` — this exchange is always CRC-protected.

> `Comms/shimmer_bt_uart.c:766-772` (arming, with `ShimSdSync_saveLocalTime()`
> called *before* anything else so the node's own clock is captured as close as
> possible to the moment the bytes arrived), `:1492-1508` (handler);
> `SDSync/shimmer_sd_sync.h:22,37-46`.

The handler reassembles the full packet — putting the opcode back in front of
the arguments — and hands it to the node routine. If the unit is not actually in
sync mode with a sync session running, it NACKs.

Two flag values are named: `SYNC_PACKET_RESEND` (`0x01`) and `SYNC_FINISHED`
(`0xFF`) (`SDSync/shimmer_sd_sync.h:63-64`).

Note that `SET_SD_SYNC_COMMAND` is **not** answered by the generic ACK path: the
common tail at `Comms/shimmer_bt_uart.c:1618` excludes it, because the reply is
built by the sync module instead.

#### Node → centre: `ACK_COMMAND_PROCESSED` carrying a command byte

The node's reply is an ACK **followed by a command byte and a flag** — the one
place in this protocol where `0xFF` is not a bare acknowledgement:

- **Node sends:** `[0xFF][0xE1][flag]`

The centre's receive state machine handles this by arming one argument byte after
an `0xFF`, and then, if that byte is `SD_SYNC_RESPONSE` (0xE1), arming one more
for the flag.

> `Comms/shimmer_bt_uart.c:541-557` (the two-stage arming) and `:1589-1606`
> (the handler, which routes the flag to the centre routine). Outside sync mode
> a bare `ACK_COMMAND_PROCESSED` from the peer is NACKed (`:1601-1604`).

⚠️ **`SD_SYNC_RESPONSE` (0xE1) has no Java constant** (`FW_ONLY`,
[§4.2](#42-sd--trial-configuration)); the Java driver names `SET_SD_SYNC_COMMAND`
`ROUTINE_COMMUNICATION`, its original name. A host implementation does not
normally need either — sync is a device-to-device exchange, and a host's only
involvement is enabling or disabling it in the configuration.

## 8. Connection session workflow

### 8.1 What a connect resets

Several things are per-connection state, not configuration, and a host must
re-establish them every time the link comes up.

| Reset on connect | Reset on disconnect | Detail |
|---|---|---|
| Receive parser | — | `gAction` set to an unsupported value, `waitingForArgs` cleared (`ShimBt_resetBtRxVariablesOnConnect`, `Comms/shimmer_bt_uart.c:203-209`) |
| — | CRC mode | Forced to `CRC_OFF` (`:2543`) |
| — | ACK prefix for pushes | Back to on (`ShimBt_resetBtResponseVars`, `:2545`) |
| — | Streaming | Stopped unconditionally (`:2534`) |
| — | Data-rate test | Stopped (`:2536`) |
| — | SD file transfer | Dropped silently (`:2539`) |
| — | Transmit ring | Cleared (`:2541`) |

Nothing in the configuration image is affected. Conversely, **no session setting
survives a reconnection** — if the host wants a CRC, it must ask again.

### 8.2 Recommended session sequence

```
connect
  |
  +-- GET_DEVICE_VERSION_COMMAND      0x3F   generation: fixes inquiry + channel vocabulary
  +-- GET_FW_VERSION_COMMAND          0x2E   feature gates (Appendix A)
  +-- SET_CRC_COMMAND                 0x8B   optional, and only after the version is known
  |
  +-- GET_INFOMEM_COMMAND × 3         0x8E   offsets 0, 128, 256 - the whole configuration
  +-- GET_CALIB_DUMP_COMMAND × n      0x9A   from offset 0; length comes from the first 2 bytes
  +-- GET_PRESSURE_CALIBRATION_...    0xA7   fitted barometer + its coefficients
  +-- GET_DAUGHTER_CARD_ID_COMMAND    0x66   expansion board identity
  |
  +-- … configuration writes, if any …
  |
  +-- GET_INFOMEM_COMMAND × 3         0x8E   READ BACK - the firmware may have corrected you
  +-- INQUIRY_COMMAND                 0x01   channel order and count, AFTER the last write
  |
  +-- START_STREAMING_COMMAND         0x07
  +-- GET_STATUS_COMMAND              0x72   confirm bit 4 - the ACK did not mean "started"
  |
 data packets …
```

The ordering constraints, all of which have been established earlier in this
document, are:

1. **Device version before anything that depends on payload shape.** The inquiry
   response's fixed part is 8 bytes on Shimmer3 and 11 on Shimmer3R, so a host
   cannot even locate `numberOfChannels` without it ([§7.1](#71-inquiry)).
2. **Firmware version before optional features.** An unrecognised command byte
   produces no reply at all, so probing is not a viable substitute
   ([§2.3](#23-framing-guarantees-per-transport)).
3. **`SET_CRC_COMMAND` after the version read**, so the version exchange itself
   is not subject to a CRC the host has not confirmed the firmware supports.
4. **`INQUIRY_COMMAND` last, after every write** ([§6.2](#62-fixing-the-packet-layout)).
5. **Read back after every write.** The whole-image correction pass runs on each
   InfoMem chunk and each single-setting write, and every setter clamps silently
   ([§7.5](#75-infomem), [§7.8](#78-sensor-settings)).
6. **Confirm a start with the status bits**, because the start commands are
   conditional ([§6.1](#61-the-six-startstop-commands)).

For comparison, the Java driver's own connect state machine runs: a dummy
sampling-rate read to flush the write buffer, a CRC-mode reset, then
`readShimmerVersionNew()`; on the Shimmer3 path it reads the configuration bytes
and pressure coefficients if `getFirmwareVersionCode() >= 6`, else falls back to
reading each setting individually; then the LED command, and — each behind its
version gate — the status, the battery and the Bluetooth version string; then the
calibration dump; and finally either a fixed-configuration write or
`inquiry()`.

> `bluetooth/ShimmerBluetooth.java:2547-2561` (`initialize`) and `:2629-2751`
> (`initializeShimmer3`).

Note one hardware-driven exception the Java driver encodes: it skips the
calibration-dump read on a **docked Shimmer3**, because that platform cannot
reach its SD card while docked. Shimmer3R can, so the read proceeds there.

> `bluetooth/ShimmerBluetooth.java:2723-2728`.

### 8.3 What must be re-read after a write

| After writing | Re-read |
|---|---|
| Any single setting (`0x05`, `0x08`, `0x09`, `0x0E`, `0x21`, `0x37`…) | The matching `GET`, because the value is clamped |
| Any InfoMem chunk (`0x8C`) | All three InfoMem pages, because the correction pass is whole-image |
| Anything that changes the enabled sensor set or a range | `INQUIRY_COMMAND` |
| A calibration block (`0x11`…`0xB1`) or dump chunk (`0x98`) | `GET_CALIB_DUMP_COMMAND`, or the matching per-sensor `GET` |
| `SET_CENTER_COMMAND` (`0x76`) | `GET_TRIAL_CONFIG_COMMAND`, **not** `GET_CENTER_COMMAND` ([§7.7](#77-sd-logging-and-trial-configuration)) |
| `SET_SHIMMERNAME_COMMAND` (`0x79`) with a blank value | `GET_SHIMMERNAME_COMMAND` — the firmware substitutes a default |
| Anything, before disconnecting | Consider `UPD_SDLOG_CFG_COMMAND` (`0x9C`) so the card is updated |

### 8.4 Timeouts and error recovery

- **Always use a response timeout**, even on a reliable transport. Two
  situations produce no reply at all: an unknown command byte, and an
  out-of-bounds `SET_INFOMEM_COMMAND` ([§5.2](#52-the-three-nack-preconditions)).
- **On a suspected desynchronisation**, stop transmitting for well over the
  firmware's 10 ms receive timeout
  (`BT_RX_COMMS_TIMEOUT_TICKS`, [§2.3](#23-framing-guarantees-per-transport)),
  discard everything received, then probe with `TEST_CONNECTION_COMMAND` (0x96).
  Do not send filler bytes: on a Shimmer3 with module status strings enabled a
  stray `0x25` puts the parser into status-string mode.
- **Correlate replies by response opcode, not by counting ACKs.** Unsolicited
  status pushes carry a leading `0xFF` by default
  ([§5.4](#54-in-stream-responses)), and 0x5D on unsupported hardware emits a
  spurious second `0xFF` ([§7.6](#76-calibration-dump-and-per-sensor-calibration)).
- **A device that NACKs every command, including `GET_FW_VERSION_COMMAND`, is in
  SD sync mode.** Clear `syncEnable` in the configuration
  ([§7.14](#714-sd-sync)).

## 9. Host-side implementations

Four host implementations of this protocol exist. All four speak the same bytes;
they differ in coverage and in where their opcode vocabulary lives.

| Implementation | Repository | Opcode registry | Coverage |
|---|---|---|---|
| **Python** (reference) | `Extras/python_scripts/` in this repository | `Shimmer_common/shimmer_comms_bluetooth.py` — the `BtCmds` class | Configuration and calibration paging, SD file transfer, dock link. Deliberately small and readable |
| **Java / Android** | `Shimmer-Java-Android-API` | `bluetooth/ShimmerBluetooth.java` — a `BtCommandDetails` map keyed by opcode, carrying each command's expected response opcode and payload length | The most complete: full connect state machine, calibration maths, channel parsing, per-version feature gates |
| **TypeScript (web)** | `shimmer-web-sdk` | `devices/shimmer3r/constants.ts`, `devices/shimmer3/protocol.ts`, `devices/shimmer3r/sdTransfer/protocol.ts` | Web Bluetooth and Web Serial. Covers 178 of the 197 named opcodes; the gaps are deliberate ([§4.1](#41-core), `SDK_MISSING`) |
| **C# (.NET)** | separate Shimmer distribution | — | Windows desktop applications |

**Where they disagree, the firmware wins.** This document records the
disagreements as `⚠️` notes; they are:

| Disagreement | Firmware behaviour | Section |
|---|---|---|
| `DERIVED_CHANNEL_BYTES_RESPONSE` 0x6E length | 8 payload bytes; the Java registry declares 3 | [§7.11](#711-derived-channels) |
| Calibration-dump argument order | `[len][offsetLo][offsetHi]`; the firmware's *own comments* say otherwise | [§7.6](#76-calibration-dump-and-per-sensor-calibration) |
| 27 opcodes with no Java constant | Served; the driver simply never named them | [§4](#4-opcode-table) (`FW_ONLY`) |
| 13 opcodes named only by Java | Not defined by this firmware | [Appendix B](#appendix-b-java-only-and-legacy-opcodes) |
| 45 opcodes with a different Java name | Same byte, legacy name | [§4](#4-opcode-table) |

The 45 name aliases are almost entirely the pre-Shimmer3R vocabulary — the Java
driver still calls the wide-range accelerometer's range `ACCEL_SENSITIVITY`, the
configuration setup bytes `CONFIG_BYTE0`, and the generic `WR_*` commands
`LSM303DLHC_*`. The [§4](#4-opcode-table) tables carry **both** names rather than
treating either as canonical, because a host maintainer reading Java source and a
host maintainer reading firmware source both need to find the row.

## 10. Important boundaries

This document covers the **Shimmer3 / Shimmer3R LogAndStream command-and-control
protocol** over classic Bluetooth SPP, BLE and the dock serial link.

It does **not** define:

- the streaming packet and channel encodings — see
  [SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md);
- the configuration byte layout — see
  [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md);
- the calibration blob's internal structure or the calibration maths — see
  [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md);
- the SD card's file and header format;
- the SD-sync timing algorithm (only its two opcodes,
  [§7.14](#714-sd-sync));
- the firmware update process;
- BtStream and SDLog, which share opcode numbers but differ in behaviour
  ([§1](#1-overview)).

### 10.1 What the protocol does not provide

These are absences by design, and a host implementation has to compensate for
each of them.

- **No framing.** No length prefix, no delimiter, no escape, no start-of-frame
  byte, in either direction. Both sides must agree on every argument count and
  every response length. [§2.3](#23-framing-guarantees-per-transport)
- **No negative acknowledgement of an unknown opcode.** An unrecognised command
  byte is silently discarded, so a host cannot discover capability by probing and
  must gate on `GET_FW_VERSION_COMMAND`.
  [§2.3](#23-framing-guarantees-per-transport)
- **No error reporting for a clamped value.** Out-of-range arguments are replaced
  with a hard-coded fallback and ACKed as if accepted.
  [§5.2](#52-the-three-nack-preconditions)
- **No atomic multi-byte configuration write.** The 512-byte configuration image
  is written 128 bytes at a time, and the whole-image correction pass runs after
  *each* chunk, so intermediate states are visible to the firmware's own
  validation. There is no transaction and no rollback.
  [§7.5](#75-infomem)
- **No flow control from the host.** The host cannot ask the device to pause. If
  it cannot keep up, the device's transmit ring fills and the firmware discards
  what will not fit rather than stalling its sampling loop.
  [§6.3](#63-interleaving)
- **No retransmission.** CRC modes let a host *detect* corruption
  ([§3.3](#33-crc-modes)); nothing in the protocol lets it request a resend. The
  only recovery is to reissue the command. The SD file transfer is the exception
  that proves the rule: it was given an explicit resume-by-offset design
  precisely because no general mechanism exists.
  [§7.13](#713-sd-file-transfer-shimmer3r)
- **No notification when the packet layout changes.** `INQUIRY_COMMAND` returns a
  snapshot, and a data packet does not describe its own contents.
  [§6.2](#62-fixing-the-packet-layout)
- **No sequence numbers or command identifiers.** Replies are correlated by
  response opcode and by the fact that the firmware processes one command at a
  time. A host must not pipeline commands whose replies it could not tell
  apart.
- **No authentication or encryption at the protocol layer.** Link security is
  whatever the Bluetooth pairing provides.

### 10.2 Commands rejected while sensing

43 commands are NACKed whenever `shimmerStatus.sensing` is set — that is,
whenever the device is streaming, logging, or both. The list is effectively
"every write that could change what a sample means", which is exactly the
property that makes a stream parseable: the layout a host fixed with
`INQUIRY_COMMAND` cannot change under it.

Grouped by what they would have changed:

| Group | Opcodes |
|---|---|
| Sample rate and enabled sensors | `0x05`, `0x08` |
| Configuration setup bytes | `0x0E` |
| Wide-range accel | `0x09`, `0x40`, `0x43`, `0x46` |
| Gyroscope | `0x49`, `0x4C` |
| Magnetometer | `0x37`, `0x3A` |
| Alternative accel / mag | `0x4F`, `0xAC`, `0xB2` |
| Pressure, GSR, expansion power | `0x52`, `0x21`, `0x5E` |
| ExG registers | `0x61` |
| Daughter card | `0x64`, `0x67` |
| Bluetooth baud (a no-op, but still blocked) | `0x6A` |
| Derived channels | `0x6D` |
| Trial configuration and identity | `0x73`, `0x76`, `0x79`, `0x7C`, `0x7F`, `0x82`, `0x85` |
| Whole configuration image | `0x8C` |
| Calibration writes | `0x11`, `0x14`, `0x17`, `0x1A`, `0xA9`, `0xAF`, `0x98`, `0x9B` |
| Resets | `0x5A`, `0x5B` |
| SD configuration file rewrite | `0x9C` |
| Test modes | `0xA4`, `0xA8` |

> `ShimBt_isCmdBlockedWhileSensing`, `Comms/shimmer_bt_uart.c:2939-2995`, which
> carries the opcode number beside each `case` label. The **Blocked while
> sensing** column of the [§4](#4-opcode-table) tables is extracted from the same
> function and is the authoritative per-opcode answer.

**Everything not on that list is permitted while sensing** — every `GET`, the
start/stop commands, `SET_CRC_COMMAND`, `SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE`,
`SET_FEATURE`, `SET_CHARGE_STATUS_LED_COMMAND`, `RESET_BT_ERROR_COUNTS`,
`TOGGLE_LED_COMMAND` and, notably, **`SET_RWC_COMMAND`**. The clock can be set
mid-recording.

The Shimmer3R SD file-transfer commands are not on the blocked list either, but
they are gated separately and in band: their access check reports
`SD_FT_STATUS_BUSY` (`0xF1`) while sensing, logging or streaming
([§7.13](#713-sd-file-transfer-shimmer3r)).

## Appendix A. Firmware version gates

Host software gates optional protocol and layout features on the firmware
version reported by `GET_FW_VERSION_COMMAND`. The **host gate** column is the
normative answer for host authors: it names the Java driver predicate and the
LogAndStream version threshold it applies.

The final column is corroborating evidence only, obtained by locating the commit
that changed the corresponding firmware symbol earliest (`git log -S`) and taking
the earliest release tag per lineage that contains it. Read it with three
caveats:

- **`≤` means inconclusive.** The introducing commit predates that lineage's
  first tag, so the tag shown is merely the earliest tag that exists — the
  feature is at or before it. Old releases were tagged sparsely.
- **A symbol's earliest commit can be a rename, not the feature's birth.** Where
  the date is much later than the host threshold suggests, the symbol was
  probably renamed or moved into the shared module at that commit.
- **Tag lineages are separate product lines.** `LogAndStream_Shimmer3`,
  `LogAndStream_Shimmer3_BLE`, `LogAndStream_Shimmer3R` and the legacy
  unprefixed `LogAndStream` version numbers are not comparable with each other.

Release provenance is taken from the two platform repositories. The shared
module's own history begins when it was extracted, so its tags date the
extraction rather than the feature; it is consulted only for features that
exist nowhere else, which are exactly the ones added after the extraction.

| Area | Feature | Host gate / threshold | Firmware symbol | Earliest release tag containing the introducing commit |
|---|---|---|---|---|
| Status response | status bit 7 carries the red-LED toggle state | `isSupportedRedLedStateInStatus`<br>LogAndStream 0.7.10 | `toggleLedRedCmd` | Shimmer3: `LogAndStream_Shimmer3_BLE_v0.16.011`, `LogAndStream_Shimmer3_v1.00.003` (introducing commit c42e0f6, 2024-10-07)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 65aa6176, 2023-11-09) |
| Status response | status bits 6 and 5 carry SD bad-file / SD-inserted | `isSupportedSdInfoInStatus`<br>LogAndStream 0.7.12 | `sdBadFile` | Shimmer3: `LogAndStream_Shimmer3_BLE_v0.16.012`, `LogAndStream_Shimmer3_v1.00.003` (introducing commit ed662e8, 2024-11-05)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 42a86589, 2024-11-05) |
| Status response | status bit 2 carries "real-world clock has been set" | `isSupportedRtcStateInStatus`<br>LogAndStream 0.7.14 | `RTC_isRwcTimeSet` | Shimmer3: `LogAndStream_Shimmer3_BLE_v1.00.000`, `LogAndStream_Shimmer3_v1.00.003` (introducing commit e501346, 2025-06-30)<br>Shimmer3R: `LogAndStream_Shimmer3R_v1.00.027` (introducing commit b4964c95, 2025-06-30) |
| Status response | a SECOND status byte carrying USB-plugged-in (STATUS_BYTE_COUNT 2) | `isSupportedUSBPluggedInStatus`<br>LogAndStream 1.0.24 (Shimmer3R) | `usbPluggedIn` | Shimmer3R: `LogAndStream_Shimmer3R_v1.00.019` (introducing commit d1f311cf, 2025-02-28) |
| Commands | GET_BT_VERSION_STR_COMMAND (Bluetooth module firmware string) | `isSupportedBtFwVerRequest`<br>_no version threshold (hardware-keyed or unconditional)_ | `GET_BT_VERSION_STR_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000` (introducing commit 372a4de, 2023-02-08)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Commands | GET_STATUS_COMMAND | `isSupportedBtStatusRequest`<br>LogAndStream 0.5.2 (Shimmer3) | `GET_STATUS_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 8b7e616, 2014-07-22)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Commands | GET_VBATT_COMMAND | `isSupportedBtBatteryRequest`<br>LogAndStream 0.5.9 (Shimmer3) | `GET_VBATT_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 15dd04c, 2016-02-11)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Commands | UPD_SDLOG_CFG_COMMAND (rewrite the SD config file after an InfoMem write) | `isBtMemoryUpdateCommandSupported`<br>_no version threshold (hardware-keyed or unconditional)_ | `UPD_SDLOG_CFG_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 5eae006, 2016-09-16)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Calibration | GET/SET/UPD_CALIB_DUMP_COMMAND and the calibration-dump blob | `isSupportedCalibDump`<br>_no version threshold (hardware-keyed or unconditional)_ | `SET_CALIB_DUMP_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 5eae006, 2016-09-16)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Commands | SD logging commands | `isSupportedSdCardAccess`<br>_no version threshold (hardware-keyed or unconditional)_ | `START_LOGGING_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 15dd04c, 2016-02-11)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Dock | configuration over the dock UART | `isSupportedConfigViaUart`<br>_no version threshold (hardware-keyed or unconditional)_ | — | _no firmware symbol identified_ |
| Dock | real-world clock set over the dock UART | `isSupportedRtcConfigViaUart`<br>_no version threshold (hardware-keyed or unconditional)_ | — | _no firmware symbol identified_ |
| InfoMem | the MPL/9-DoF fusion calibration block | `isSupportedMpl`<br>_no version threshold (hardware-keyed or unconditional)_ | `NV_MPL_GYRO_CALIBRATION` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 00425d6, 2015-03-30)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| InfoMem | derived-channel bytes 3..7 at InfoMem 118-122 (8 bytes total, not 3) | `isSupportedEightByteDerivedSensors`<br>LogAndStream 0.7.1 | `NV_DERIVED_CHANNELS_3` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit d97bbd6, 2017-05-24)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| SD header | the expansion-board id in the SD file header | `isSupportedExpansionBrdIdInSdHeader`<br>LogAndStream 0.6.13 | — | _no firmware symbol identified_ |
| Commands | SET_FEATURE (runtime feature toggles) | — | `SET_FEATURE` | Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 52bb8f0e, 2024-03-27) |
| Commands | SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE | — | `SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE` | Shimmer3: `LogAndStream_Shimmer3_BLE_v0.16.001`, `LogAndStream_Shimmer3_v0.15.004` (introducing commit 25ae95d, 2023-02-17)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Commands | SET_CRC_COMMAND (response CRC modes) | — | `SET_CRC_COMMAND` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 392f693, 2015-03-30)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Calibration | GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND (sensor-agnostic pressure coefficients) | — | `GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND` | Shimmer3: `LogAndStream_Shimmer3_BLE_v0.16.006`, `LogAndStream_Shimmer3_v1.00.003` (introducing commit a5151db, 2024-07-08)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 707d1387, 2024-07-01) |
| Calibration | alternative accel / alternative mag calibration commands | — | `SET_ALT_ACCEL_CALIBRATION_COMMAND` | Shimmer3: `LogAndStream_Shimmer3_BLE_v0.16.013`, `LogAndStream_Shimmer3_v1.00.003` (introducing commit 6459f66, 2024-12-12)<br>Shimmer3R: `LogAndStream_Shimmer3R_v1.00.005` (introducing commit 15947690, 2024-11-28) |
| SD transfer | SD file transfer over Bluetooth (Shimmer3R) | — | `SD_FILE_READ_COMMAND` | shared module: `LogAndStream_Shimmer3R_v1.01.009` (introducing commit 71d283f, 2026-08-18) |
| InfoMem | configuration setup bytes 4-6 at InfoMem 130-132 | — | `NV_CONFIG_SETUP_BYTE4` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, ≤ `LogAndStream_v0.8.0` (introducing commit 00425d6, 2015-03-30)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| InfoMem | the Bluetooth PIN byte at InfoMem 231 | — | `NV_BT_SET_PIN` | Shimmer3: ≤ `LogAndStream_Shimmer3_BLE_v0.15.003`, ≤ `LogAndStream_Shimmer3_v0.15.000`, `LogAndStream_v0.9.0` (introducing commit 87ff27c, 2017-08-18)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit 39041ebe, 2023-11-01) |
| Commands | RESET_BT_ERROR_COUNTS | — | `RESET_BT_ERROR_COUNTS` | shared module: `LogAndStream_Shimmer3R_v1.00.049`, `LogAndStream_Shimmer3_v1.00.015` (introducing commit 039dbbd, 2025-11-27) |
| Commands | SET_DATA_RATE_TEST / DATA_RATE_TEST_RESPONSE | — | `SET_DATA_RATE_TEST` | Shimmer3: `LogAndStream_Shimmer3_BLE_v0.16.002`, `LogAndStream_Shimmer3_v1.00.003` (introducing commit 7e8558f, 2023-12-15)<br>Shimmer3R: ≤ `LogAndStream_Shimmer3R_v0.00.002` (introducing commit da26df24, 2023-12-15) |

Repository history starts: common 2025-01-28; s3 2013-10-16; s3r 2023-10-17.

## Appendix B. Java-only and legacy opcodes

Thirteen byte values are named by the Java driver but **not defined by this
firmware at all**. They are listed here so that a host maintainer looking at Java
source can tell "this firmware does not implement it" from "I have the wrong
opcode number" — and so that nobody reuses one of these numbers for something
new. They are flagged `JAVA_ONLY` in
[§4.6](#46-not-served-by-logandstream).

| Opcode | Java name | Why it is not here |
|---|---|---|
| `0x0C` | `SET_5V_REGULATOR_COMMAND` | Shimmer2 only — annotated as such in the Java source |
| `0x0D` | `SET_PMUX_COMMAND` | Shimmer2 only — annotated as such in the Java source |
| `0x26` | `SET_EMG_CALIBRATION_COMMAND` | Superseded by the calibration dump |
| `0x27` | `EMG_CALIBRATION_RESPONSE` | Superseded by the calibration dump |
| `0x28` | `GET_EMG_CALIBRATION_COMMAND` | Superseded by the calibration dump |
| `0x29` | `SET_ECG_CALIBRATION_COMMAND` | Superseded by the calibration dump |
| `0x2A` | `ECG_CALIBRATION_RESPONSE` | Superseded by the calibration dump |
| `0x2B` | `GET_ECG_CALIBRATION_COMMAND` | Superseded by the calibration dump |
| `0x33` | `SET_GYRO_TEMP_VREF_COMMAND` | No handler in any current firmware |
| `0x34` | `SET_BUFFER_SIZE_COMMAND` | Buffer size is fixed at 1; only the `GET` (0x36) survives |
| `0x55` | `SET_BMP180_PRES_CALIBRATION_COMMAND` | Superseded by `GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND` (0xA7) |
| `0x56` | `BMP180_PRES_CALIBRATION_RESPONSE` | Superseded by 0xA6 |
| `0x57` | `GET_BMP180_PRES_CALIBRATION_COMMAND` | Superseded by 0xA7 |

> The `// only Shimmer 2` annotations on 0x0C and 0x0D are in the driver's own
> registry at `bluetooth/ShimmerBluetooth.java:390-391`. All thirteen also appear
> in the generated `comms/radioProtocol/ShimmerLiteProtocolInstructionSet.java`
> enumeration, which is a superset covering every Shimmer generation.

Sending any of these to LogAndStream firmware produces **no reply at all** — the
byte falls to the receive state machine's `default:` case and is discarded
([§2.3](#23-framing-guarantees-per-transport)). Their arguments, if any, are then
interpreted as fresh command bytes, so a host that sends one will likely
desynchronise. This is the practical reason not to carry a legacy registry
forward untrimmed.

Two further categories are *not* in this appendix but are worth distinguishing
from it:

- **Served but useless.** `DEPRECATED_GET_DEVICE_VERSION_COMMAND` (0x24) and the
  three baud-rate opcodes (0x6A, 0x6B, 0x6C) are defined and reach a handler; the
  first works and is merely superseded, the others do nothing.
  [§4.5](#45-deprecated--no-op)
- **Reserved, never served.** `RSP_I2C_BATT_STATUS_COMMAND` (0x9D) and
  `GET_I2C_BATT_STATUS_COMMAND` (0x9E) exist only under `SHIMMER4_SDK`, and 0x9C
  is `SET_I2C_BATT_STATUS_FREQ_COMMAND` there rather than
  `UPD_SDLOG_CFG_COMMAND`. No LogAndStream code on either supported platform
  references them, and **hosts must not reuse the numbers**.
  [§4.6](#46-not-served-by-logandstream)

Conversely, 27 opcodes are served by this firmware but have **no named constant
in the Java driver** (`FW_ONLY`). Twelve are the Shimmer3R SD file transfer
(0xC1-0xCC), two are the Shimmer4 pair above, and `NACK_COMMAND_PROCESSED`
(0xFE) is handled inline rather than through a constant. The remaining ones are
genuinely usable commands a Java-based host has to send as raw bytes:

| Opcode | Firmware name | Section |
|---|---|---|
| `0x3D` / `0x3E` | `UNIQUE_SERIAL_RESPONSE` / `GET_UNIQUE_SERIAL_COMMAND` | [§7.2](#72-version-information) |
| `0x64` | `SET_DAUGHTER_CARD_ID_COMMAND` | [§7.10](#710-daughter-card) |
| `0x67` / `0x68` / `0x69` | daughter-card memory set / response / get | [§7.10](#710-daughter-card) |
| `0xA3` | `SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE` | [§7.12](#712-control-and-test) |
| `0xA4` / `0xA5` | `SET_DATA_RATE_TEST` / `DATA_RATE_TEST_RESPONSE` | [§7.12](#712-control-and-test) |
| `0xB5` | `DUMMY_COMMAND` | [§7.12](#712-control-and-test) |
| `0xB6` | `RESET_BT_ERROR_COUNTS` | [§7.12](#712-control-and-test) |
| `0xE1` | `SD_SYNC_RESPONSE` | [§7.14](#714-sd-sync) |

## Still unverified / not found in code

Everything above is derived from source. The following statements are either
absent from the code, or present in a form that only bench measurement can
settle. They are listed so that a host author knows where the document stops
being authoritative, and so that a later pass can close them.

**Transport and framing**

- **BLE GATT service and characteristic UUIDs, and the negotiated ATT MTU.**
  Nothing in `log-and-stream-common` or either platform repository declares a
  GATT service: the Bluetooth module terminates GATT itself and hands the
  firmware a reassembled byte stream ([§2.2](#22-ble)). The one MTU figure in the
  firmware is Shimmer3's `BLE_MTU_SIZE` 157
  (`shimmer3-firmware` `Shimmer_Driver/RN4X/RN4X.h:242`), which is the RN4678's
  working value and not necessarily what a given host negotiates. The Shimmer3R
  equivalent is inside the CYW20820's EZ-Serial configuration. **Needs a
  capture** from a host stack on each generation.
- **Whether BLE ever coalesces two firmware messages into one notification.**
  [§2.2](#22-ble) tells hosts to assume it can, which is the safe assumption, but
  the firmware hands each response to the UART as a single write and the
  coalescing decision belongs entirely to the module. Not observable from source.
  **Needs a capture.**
- **Whether the module's classic-Bluetooth status strings can reach a host.** The
  firmware consumes them on the MCU-side UART
  ([§2.1](#21-classic-bluetooth-spp)); whether a misconfigured module could also
  emit them over RFCOMM has not been established.

**Status and in-stream responses**

- **The default state of the `SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE` (0xA3)
  prefix as observed by a host.** The firmware defaults
  `useAckPrefixForInstreamResponses` to 1 and resets it on every disconnect
  (`Comms/shimmer_bt_uart.c:196`, `:2545`), which is what
  [§5.4](#54-in-stream-responses) states. What has **not** been verified is
  whether any shipped host relies on the opposite, and whether the reset is
  reliably observed across a BLE reconnection that the module handles without the
  firmware seeing a disconnect event.
- **Unsolicited-push timing.** The four triggers are known
  ([§5.4](#54-in-stream-responses)); the latency between the physical event and
  the push, and whether two pushes can be queued back to back (dock immediately
  followed by a logging stop, say), are not derivable from source. **Needs bench
  observation.**
- **The Shimmer3R two-byte status over BLE.** The second byte is compiled in
  under `#if defined(SHIMMER3R)` and carries `usbPluggedIn`. That it survives BLE
  fragmentation intact — i.e. that no host stack truncates a 4- or 5-byte
  notification — has not been confirmed on hardware.
- **CRC behaviour on unsolicited in-stream pushes.** The code appends the session
  CRC (`Comms/shimmer_bt_uart.c:2462-2466`), so [§3.3](#33-crc-modes) states that
  it does. Whether every host implementation validates it on a *push* rather than
  only on a solicited response is unverified, and a host that does not will see
  spurious trailing bytes.
- **NACK-while-sensing over BLE.** The NACK path is transport-independent in the
  firmware, but a bare single-byte `0xFE` notification arriving among data-packet
  notifications is exactly the case most likely to be mis-attributed by a host
  reassembler. **Needs bench confirmation** on both generations.

**Hardware-keyed values**

- **The TCXO clock values.** `ShimConfig_checkAndCorrectConfig` force-clears the
  `tcxo` configuration bit behind `#if !IS_SUPPORTED_TCXO`
  (`Configuration/shimmer_config.c:518-524`), and `samplingClockFreqGet()`
  returns a flat `32768.0f` on both platforms
  (`shimmer3r-firmware` `Core/Src/main.c:829-832`). The sampling-rate divider in
  [§7.1](#71-inquiry) and [§7.8](#78-sensor-settings) therefore assumes 32768 Hz
  unconditionally, which is correct for every build in the available source:
  `IS_SUPPORTED_TCXO` is defined (as `0`) only in Shimmer3's `main.c` and not at
  all on Shimmer3R, so it is **not visible in `shimmer_config.c`'s translation
  unit on either platform** and the guard is always taken. **What a
  TCXO-equipped board would actually clock at, and whether the divider
  arithmetic would change, is therefore not established by any code path that
  currently compiles.**
- **The ADS7028 external-ADC reference voltage and bit depth.** Needed to convert
  the external ADC channels to volts; not present in this repository. See
  [SHIMMER3_STREAMING_DATA_FORMAT.md §7.2](SHIMMER3_STREAMING_DATA_FORMAT.md#72-adc-and-battery).
- **The battery ADC reference and divider ratio.** Same gap, for the
  `GET_VBATT_COMMAND` payload ([§7.4](#74-battery)) and the `VBATT` streaming
  channel.
- **RN4678 baud codes 11 and 12.** `BAUD_1000000` is annotated "Only supported in
  RN4678 v1.23 (issues with v1.13.5 & v1.22)" and `BAUD_2000000` "Only supported
  on CYW20820" (`Comms/shimmer_bt_uart.h:321-322`). Those annotations are the
  only evidence; the failure mode on v1.13.5/v1.22, and whether 12 is rejected or
  silently ignored on a Shimmer3, have not been measured. Neither is
  host-visible, so this matters only for firmware work.

**SD sync**

- **The SD-sync exchange as a whole.** [§7.14](#714-sd-sync) documents the two
  opcodes, their byte layouts, the always-on 1-byte CRC and the exclusivity rule
  — all from source. The **timing** — the centre's 12-second per-node connection
  window, the 54-second minimum broadcast interval, the retry and resend
  behaviour, and what a node does when a sync is missed — is in
  `SDSync/shimmer_sd_sync.{h,c}` but has not been read out into a
  specification, and the observable behaviour of a multi-unit group has not been
  characterised. A host has no role in the exchange, so this is a gap in the
  documentation rather than in the host contract.

**Firmware provenance**

- **The first firmware tag for several version gates.** Where
  [Appendix A](#appendix-a-firmware-version-gates) shows a `≤` marker, the
  introducing commit predates the earliest tag in that lineage, so the tag shown
  is merely the earliest one that exists. For features that live only in the
  shared module, the column could not be established at all: **the shared
  repository's history begins in January 2025**, so its tags date the extraction
  of the module rather than the introduction of the feature. Those gates predate
  the available history.
