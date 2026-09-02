# log-and-stream-common

Shared C firmware library for Shimmer wearable sensor devices. This library provides the core logic for logging sensor data to an SD card and streaming data over Bluetooth, and is intended to be included as a submodule by platform-specific firmware projects (e.g. Shimmer3, Shimmer3R, Shimmer4 SDK).

## Overview

`log-and-stream-common` contains hardware-agnostic modules that handle the main firmware responsibilities of a Shimmer device:

- Sensor configuration, sampling, and calibration
- SD card data logging (data files, config files, headers, sync)
- Bluetooth UART streaming and dock USART communication
- Battery monitoring, LED indication, and button handling
- Real-time clock management and EEPROM access
- Task scheduling and boot sequencing
- A platform abstraction API for porting to different MCU targets

## Repository Structure

```
log_and_stream_common.c/h   – Top-level init and device lifecycle logic
log_and_stream_definitions.h – Shared type definitions and macros
log_and_stream_globals.h     – Global state variables (shimmerStatus, batteryStatus)
log_and_stream_externs.h     – External function declarations (implemented by platform)
log_and_stream_includes.h    – Convenience header that includes all modules

Battery/        – Battery voltage monitoring
Boards/         – Hardware revision detection and board-level helpers
Button/         – Button debounce and event handling
Calibration/    – Sensor calibration data management
Comms/          – Bluetooth UART and dock USART drivers
Configuration/  – Device configuration (shimmer_config)
CRC/            – CRC calculation utilities
EEPROM/         – EEPROM read/write abstraction
GSR/            – Galvanic skin response sensor support
LEDs/           – LED state and blink management
Platform/       – Platform abstraction API (platform_api.h / platform_api.c)
RTC/            – Real-time clock access
SDCard/         – SD card management, data files, config files, and file headers
SDSync/         – SD card synchronisation over Bluetooth
Sensing/        – Peripheral sensor start/stop and polling coordination
TaskList/       – Cooperative task list for the main firmware loop
Test/           – LED state and factory test helpers
Util/           – General utility functions
Extras/         – Python scripts and calibration tools
scripts/        – Firmware version management scripts
```

## Platform Abstraction

Hardware-specific behaviour is provided to the library via two mechanisms:

1. **Weak function stubs** declared in `Platform/platform_api.h` — implement these in your platform project to supply tick counters, delays, hardware resets, GPIO setup, and peripheral initialisation checks.
2. **External function declarations** in `log_and_stream_externs.h` — these must be implemented by the platform firmware (e.g. board GPIO, UART drivers, RTC, ADC/I2C/SPI sensing, Bluetooth).

## Supported Platforms

The library is conditionally compiled for the following targets using preprocessor defines:

| Define | Platform |
|---|---|
| `SHIMMER3` | Shimmer3 (MSP430-based) |
| `SHIMMER3R` | Shimmer3R |
| `SHIMMER4_SDK` | Shimmer4 SDK |

## Documentation

Reference documentation lives in [`docs/`](docs/). The protocol, configuration,
streaming and calibration documents are written against the firmware in this
repository and are the authority for the byte layouts that host software must
match.

| Document | Covers |
|---|---|
| [SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](docs/SHIMMER3_BT_COMMUNICATION_PROTOCOL.md) | The host ↔ device command/response protocol: transports, framing, the full opcode table, ACK/NACK and status semantics, and a per-command reference. |
| [SHIMMER3_CONFIGURATION_INFOMEM.md](docs/SHIMMER3_CONFIGURATION_INFOMEM.md) | The 512-byte InfoMem configuration block: byte-and-bit map, defaults, the firmware's validation and correction rules, and the SD-header mirror. |
| [SHIMMER3_STREAMING_DATA_FORMAT.md](docs/SHIMMER3_STREAMING_DATA_FORMAT.md) | The streaming data packet: header and timestamp, the channel-ID registry, channel ordering rules, per-channel encodings, and raw-to-physical conversion. |
| [SHIMMER3_CALIBRATION.md](docs/SHIMMER3_CALIBRATION.md) | Per-device kinematic calibration: the dump blob format, the 21-byte per-sensor block, default seeds, the calibration maths, and persistence precedence. |
| [SHIMMER3_BOARD_REVISIONS.md](docs/SHIMMER3_BOARD_REVISIONS.md) | Shimmer3 and Shimmer3R board revisions, sensor generations, and the firmware-relevant revision gates. |

## Integration

Add this repository as a submodule in your firmware project:

```sh
git submodule add https://github.com/ShimmerResearch/log-and-stream-common
```

Then include `log_and_stream_includes.h` to pull in all module headers, and call `LogAndStream_init()` during your firmware boot sequence.

## Firmware Versioning

Version management is handled by the scripts in `scripts/`. See [`scripts/README_versioning.txt`](scripts/README_versioning.txt) for full details.

```sh
# Increment the patch version (default)
./scripts/increment_version.sh

# Increment minor or major
./scripts/increment_version.sh minor
./scripts/increment_version.sh major
```

## Code Style

C and H source files are automatically formatted using **clang-format** (version 17, style defined in `.clang-format`) via a GitHub Actions workflow that runs on every push.
