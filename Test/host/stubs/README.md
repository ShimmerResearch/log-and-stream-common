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
  `batteryStatus` globals. `log_and_stream_externs.h` declares those two and
  `log_and_stream_common.c` defines them for the firmware; the host tests do
  not link that file, so `host_stubs.c` defines them instead. Tests drive the
  platform by writing to these directly.

A stub header can stop at declarations and leave the definitions to the one
suite that drives them. `ff.h` does: `test_calibration.c` defines the FatFs
calls itself, as a fake SD card it can make fail.

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
| `ff.h` | Shimmer3 `Shimmer_Driver/FatFs/ff.h` (FatFs R0.08b); Shimmer3R `Middlewares/Third_Party/FatFs/src/ff.h` (R0.12c) | `_FATFS` 8237 / 68300; the `FRESULT` codes; `FA_READ` 0x01, `FA_OPEN_EXISTING` 0x00, `FA_WRITE` 0x02, `FA_CREATE_ALWAYS` 0x08, `FA_CREATE_NEW` 0x04; Shimmer3R's `FSIZE_t` is 64-bit (`_FS_EXFAT` 1 in its `ffconf.h`), `_MAX_LFN` 255, `AM_DIR` 0x10, and `f_size()` reads `obj.objsize` as the real macro does. Struct members only, not layouts |
| `shimmer_definitions.h` | both firmwares' `Shimmer_Driver/shimmer_definitions.h` | `DEVICE_VER` 3 (Shimmer3) / 10 (Shimmer3R); `FW_IDENTIFIER` 3 on both |
| `version.h` | both firmwares' `Shimmer_Driver/version.h` | release-stamped: v1.01.006 / v1.01.021 when written. Drift is harmless, because nothing asserts a version |
| `Shimmer_Driver/hal_Infomem.h` | Shimmer3R `Shimmer_Driver/hal_Infomem.h` | `INFOMEM_CALIB_SIZE` 0x400, which sizes the calibration RAM |
| `hal_InfoMem.h` | Shimmer3 `Shimmer_Driver/5xx_HAL/hal_InfoMem.h` | forwards to `Shimmer_Driver/hal_Infomem.h` under `SHIMMER3R` — see below |
| `Shimmer_Driver/LSM6DSV/lsm6dsv-pid/lsm6dsv_reg.h` | Shimmer3R submodule `lsm6dsv-pid`, v2.0.0 | `lsm6dsv_xl_full_scale_t` 0x0-0x3; `lsm6dsv_gy_full_scale_t` 0x0-0x4, then 0xc for 4000 dps |
| `Shimmer_Driver/LIS2DW12/lis2dw12-pid/lis2dw12_reg.h` | Shimmer3R submodule `lis2dw12-pid`, v2.0.0 | `lis2dw12_fs_t` 0-3. The mode enumerators are deliberately absent |
| `Shimmer_Driver/LIS3MDL/lis3mdl-pid/lis3mdl_reg.h` | Shimmer3R submodule `lis3mdl-pid`, v2.0.0 | `lis3mdl_fs_t` 0-3 |
| `Shimmer_Driver/5xx_HAL/hal_RTC.h` | Shimmer3 `Shimmer_Driver/5xx_HAL/hal_RTC.h` | declarations only |
| `msp430.h` | the TI compiler's MSP430 device header | `_delay_cycles()` as a no-op |
| `stm32u5xx.h` | Shimmer3R's CMSIS device header, which includes `stm32u5xx_hal.h` | `HAL_Delay()` declaration only |
| `stm32u5xx_hal.h` | Shimmer3R `Core/Inc/stm32u5xx_hal_conf.h` (`USE_FULL_ASSERT` 1) | `assert_param()` as that header defines it; `assert_failed()` is the suite's, as it is an empty function in the firmware's `main.c` |
| `main.h`, `fatfs.h` | the Shimmer3R firmware headers of those names | empty — `SDCard/shimmer_sd_data_file.c` includes them and takes nothing from them that a host test reaches |
| `Shimmer_Driver/5xx_HAL/hal_Board.h`, `LSM303AHTR/lsm303ahtr.h`, `LSM303DLHC/lsm303dlhc.h`, `RN4X/RN4X.h` | the Shimmer3 headers of those names | empty — `Calibration/` includes them and takes nothing from them |

If one of those real headers changes a value, change it here in the same PR.

## `hal_InfoMem.h` and `hal_Infomem.h`

The two platforms' InfoMem headers differ only in the case of one letter.
Linux keeps the two stubs apart. Windows and macOS file systems ignore case,
so there a Shimmer3R `#include "hal_Infomem.h"` finds the Shimmer3 stub first,
through `-I stubs`. That stub therefore forwards a Shimmer3R build on to
`Shimmer_Driver/hal_Infomem.h`, and every host OS sees the same definitions.
