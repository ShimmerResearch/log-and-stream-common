# Host-test stubs

`log_and_stream_externs.h` declares the functions each platform firmware must
implement. Shimmer3 implements them against the MSP430 HAL, Shimmer3R against
the STM32 HAL. **This directory is a third implementation, for a PC.**

That is the whole idea. A subsystem here is testable on a host exactly to the
degree that it talks to the platform through that contract, so anything that
needs a new stub is telling you it reached around the abstraction.

Two kinds of file live here:

- **Headers the consuming firmware owns.** `CAT24C16/`, `BMPX80/`,
  `hal_*.h` and friends are vendor or HAL headers that live in
  `shimmer3-firmware` / `shimmer3r-firmware`, not in this repository, so a host
  compile cannot find them. The stubs carry only the declarations and constants
  the code under test actually reads. **Where a constant has a value (page
  sizes, register widths), it must match the real header** — a wrong value here
  makes a green test meaningless. Each one names its source below.

- **`host_stubs.c`** — the externs themselves, plus the `shimmerStatus` and
  `batteryStatus` globals that `log_and_stream_globals.h` defines for the
  firmware. Tests drive the platform by writing to these directly.

## Do not stub the module under test

Nothing here shadows a real header from this repository. `-I .` still resolves
every `Battery/`, `Boards/`, `RTC/`, `Configuration/` header to the real one, so
a test always sees the real struct layouts, the real bitfields and the real
constants. A stubbed `gConfigBytes` would drift from the firmware's and the
test would keep passing while the device was wrong.

## Constant provenance

| Stub | Mirrors | Checked against |
|---|---|---|
| `CAT24C16/CAT24C16.h` | `Shimmer_Driver/CAT24C16/CAT24C16.h` | page 16 B, total 2048 B |
| `BMPX80/bmpX80.h` | `Shimmer_Driver/BMPX80/bmpX80.h` (Shimmer3 only) | `isBmp280InUse()` predicate only |
| `MPU9150/mpu9150.h` | `Shimmer_Driver/MPU9150/mpu9150.h` (Shimmer3 only) | declaration only |
| `hal_FactoryTest.h` | both firmwares' `hal_FactoryTest.h` | declaration only |

If one of those real headers changes a value, change it here in the same PR.
