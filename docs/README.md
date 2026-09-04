# Shimmer3 / Shimmer3R Firmware Documentation

Reference documentation for the LogAndStream firmware shared by Shimmer3
(MSP430) and Shimmer3R (STM32U5). These documents are the authoritative source
for host-side integrations — Java, JavaScript/TypeScript, Python, C#.

Every document carries a **Verified against** block naming the exact firmware
revisions its byte-level claims were read from, and a **Still unverified /
not found in code** section listing what could not be confirmed. A pinned
commit is a citation, not a claim of currency: when the firmware moves on,
a document needs re-checking against it rather than the stamp being wrong.

## Start here

| Document | What it covers |
|---|---|
| [SHIMMER3_ARCHITECTURE_OVERVIEW.md](SHIMMER3_ARCHITECTURE_OVERVIEW.md) | How the firmware is put together: the shared submodule, the platform boundary, the task scheduler, boot, and the sample path. **Read this first.** |
| [SHIMMER3_BOARD_REVISIONS.md](SHIMMER3_BOARD_REVISIONS.md) | Which hardware is which, and the firmware-relevant revision gates |

## Host integration

The documents a host application needs to talk to a device or read its data.

| Document | What it covers |
|---|---|
| [SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md) | The Bluetooth command-and-control protocol: framing, opcodes, ACK/NACK, per-command reference |
| [SHIMMER3_DOCK_PROTOCOL.md](SHIMMER3_DOCK_PROTOCOL.md) | The wired protocol over the dock UART and Shimmer3R USB-C. A different protocol, not a subset |
| [SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) | Live data packets: layout, the channel registry, per-platform channel order, encodings, raw-to-physical conversion |
| [SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md) | The on-card layout: directory structure, the binary file header, sample records, `sdlog.cfg`, the calibration file |
| [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) | The 512-byte configuration image: full byte map, validation rules, defaults, and the InfoMem-to-SD-header mapping |
| [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) | Calibration in both its shapes, the 21-byte kinematic block, default seeds, and the conversion maths |
| [SHIMMER3_SD_FILE_TRANSFER.md](SHIMMER3_SD_FILE_TRANSFER.md) | Retrieving logged files over Bluetooth (Shimmer3R only) |
| [SHIMMER3_TIMEKEEPING.md](SHIMMER3_TIMEKEEPING.md) | The three clocks, the UTC contract, and placing a recording on an absolute timeline |
| [SHIMMER3_SD_SYNC.md](SHIMMER3_SD_SYNC.md) | Multi-device synchronisation and how to apply the recorded offsets |

## Device behaviour

| Document | What it covers |
|---|---|
| [SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md) | Every LED state, the priority order, and a quick reference keyed on what the user sees |
| [SHIMMER3_BATTERY_AND_CHARGING.md](SHIMMER3_BATTERY_AND_CHARGING.md) | Charge bands and hysteresis, charger states, low-battery auto-stop |
| [SHIMMER3_POWER_MANAGEMENT.md](SHIMMER3_POWER_MANAGEMENT.md) | Switched rails, sleeping, what keeps a device awake |
| [SHIMMER3_GSR_AUTORANGE.md](SHIMMER3_GSR_AUTORANGE.md) | The four feedback resistors, auto-range switching, and the settling behaviour visible in the data |

## Hardware and production

| Document | What it covers |
|---|---|
| [SHIMMER3_EEPROM_MEMORY_MAP.md](SHIMMER3_EEPROM_MEMORY_MAP.md) | The CAT24C16 map: expansion-board identity, radio settings, the branding record |
| [SHIMMER3R_PERIPHERAL_ALLOCATION.md](SHIMMER3R_PERIPHERAL_ALLOCATION.md) | Which STM32U5 bus and peripheral instance serves which part |
| [SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md) | The self-test report format and the `S3R_TEST_nnnn` registry |

## Development

| Document | What it covers |
|---|---|
| [SHIMMER3_BUILD_AND_PROGRAMMING.md](SHIMMER3_BUILD_AND_PROGRAMMING.md) | Building both platforms, headless builds, programming routes, build-time symbols |
| [SHIMMER3_RELEASE_AND_VERSIONING.md](SHIMMER3_RELEASE_AND_VERSIONING.md) | Where the version lives, what a host sees, and what bumps when |

---

## Things that catch people out

A short index of the traps documented across this set, for anyone debugging
something that "should work".

| Symptom | Look at |
|---|---|
| Every channel after pressure decodes as garbage | Shimmer3 emits temperature-then-pressure; Shimmer3R emits pressure-then-temperature, and the widths differ — [streaming](SHIMMER3_STREAMING_DATA_FORMAT.md) §4.3 |
| Magnetometer Y and Z swapped | Shimmer3 with an LSM303DLHC emits X, **Z**, **Y** — [streaming](SHIMMER3_STREAMING_DATA_FORMAT.md) §4.1 |
| Calibrated values out by orders of magnitude | Bias and sensitivity are **big-endian** — [calibration](SHIMMER3_CALIBRATION.md) §3.1 |
| Gyro range reads wrong by 16x on Shimmer3R | The range's high bit is in configuration byte 130 — [InfoMem](SHIMMER3_CONFIGURATION_INFOMEM.md) §4.1 |
| Low-noise and wide-range accel calibrations swapped | The SD header order is not the InfoMem order — [calibration](SHIMMER3_CALIBRATION.md) §4.2 |
| A written configuration reads back different | The firmware silently corrects illegal combinations — [InfoMem](SHIMMER3_CONFIGURATION_INFOMEM.md) §10 |
| Device not discoverable over classic Bluetooth | The EEPROM radio-enable bits, which nothing corrects — [EEPROM](SHIMMER3_EEPROM_MEMORY_MAP.md) §4.2 |
| Timestamps wrong by the local UTC offset | The real-world clock is **UTC** — [timekeeping](SHIMMER3_TIMEKEEPING.md) |
| A Shimmer3R recording cannot be placed in absolute time | The RTC-diff bytes are repurposed on Shimmer3R — [SD card](SHIMMER3_SD_CARD_FORMAT.md) §3.3 |
| No sync offset recorded for a node | The first sync round is deliberately discarded — [SD sync](SHIMMER3_SD_SYNC.md) §4.3 |
| Repeated identical GSR samples | The 80 ms settling hold after a range change — [GSR](SHIMMER3_GSR_AUTORANGE.md) §5.1 |
| GSR values out by a large constant factor | The range-pin reversal flag — [GSR](SHIMMER3_GSR_AUTORANGE.md) §2 |
| Solid red LED and no battery indication | The host `TOGGLE_LED` override, never cleared by the firmware — [LEDs](SHIMMER3_LED_FEEDBACK.md) §4 |
| A newer command gets no response at all | Older firmware ignores unknown opcodes rather than NACKing — [versioning](SHIMMER3_RELEASE_AND_VERSIONING.md) §6 |
| `sdlog.cfg` edit reset unrelated settings | Parsing starts from a blank configuration; the file is authoritative — [SD card](SHIMMER3_SD_CARD_FORMAT.md) §5.2 |
| Headless Shimmer3R build fails with `no file system for scheme: C` | `-import` needs backslashes — [build](SHIMMER3_BUILD_AND_PROGRAMMING.md) §5.1 |
| Shimmer3 Release configuration will not build | Known and parked; releases ship from **Debug** — [build](SHIMMER3_BUILD_AND_PROGRAMMING.md) §4.2 |

## No Shimmer3/3R counterpart, by design

Two Verisense documents describe mechanisms this platform pair does not have.
They are listed here so nobody looks for the equivalent:

| Verisense document | Why there is no counterpart |
|---|---|
| `VERISENSE_FLASH_STORAGE_MEMORY_ALLOCATION` | Verisense manages raw NAND in banks; Shimmer3/3R use FAT on a removable SD card — see [SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md) |
| `VERISENSE_PAYLOAD_LOOKUP_TABLE` | The lookup table indexes those raw banks. FAT directory entries serve the same purpose here |

## Conventions

- **S3** means Shimmer3, **S3R** means Shimmer3R. Where a document says
  "both", the behaviour is identical on the two generations.
- Byte offsets are zero-based. Bit numbering follows the firmware structs: bit
  0 is the least-significant bit.
- Endianness is stated per field, because it is **not** consistent across this
  firmware — the sampling rate is little-endian while the configuration time
  and both experiment lengths are big-endian, in the same configuration block.
- These documents describe **LogAndStream**. BtStream and SDLog used related
  but not identical layouts.

## Contributing

- The firmware is the authority. Where the Java driver and the firmware
  disagree, the firmware wins and the disagreement is worth documenting.
- Trace every claim to a file and a symbol.
- Put anything you could not confirm under **Still unverified / not found in
  code** rather than guessing. That section is as valuable as the rest.
- Update the **Verified against** block when you re-check a document against a
  newer revision.
- This is a public repository: no ticket identifiers in document bodies.
  Ticket references belong in commit messages.
