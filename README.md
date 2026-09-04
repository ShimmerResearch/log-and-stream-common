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

Reference documentation lives in [`docs/`](docs/), written against the firmware
in this repository — so it is the authority for the byte layouts host software
must match, ahead of any host-side driver.

**[`docs/README.md`](docs/README.md) is the index**, and it opens with a
"things that catch people out" table that maps a symptom to the document and
section that explains it. Start there.

Twenty documents cover the firmware in four groups:

| Group | Documents |
|---|---|
| **Start here** | Architecture overview; board revisions |
| **Host integration** | Bluetooth protocol; dock protocol; streaming data format; SD card format; configuration (InfoMem); calibration; SD file transfer; timekeeping; SD sync |
| **Device behaviour** | LED feedback; battery and charging; power management; GSR range control |
| **Hardware and production** | EEPROM memory map; Shimmer3R peripheral allocation; Shimmer3R factory test report |
| **Development** | Build and programming; release and versioning |

Every document carries a **Verified against** block naming the firmware
revisions its claims were read from, and a **Still unverified / not found in
code** section listing what could not be confirmed from the source. Treat the
second as part of the content: it is where the open questions are recorded
rather than guessed at.

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
