# Shimmer3 / Shimmer3R Board Revisions

> **How to read this document:** The **Firmware-relevant revision gates** and
> **Shimmer3R revision quick reference** sections are the curated summaries and
> should be treated as the reference for current development. The **per-product
> tables** further down are a conversion of the source workbook, retained for
> traceability. Source of truth:
> `Shimmer - Shimmer\Projects\026 Legacy Designs\Shimmer_PCBREV_INDEX.xlsx`
> (tabs "Shimmer3 Board Versions&ICs" and "Shimmer3 & 3R Generations").
> Last synced: 2026-08-10.

A board identity is `SR<board id>-<major rev>-<minor rev>`, stored in the
expansion-board EEPROM and read at runtime via `ShimBrd_getDaughtCardId()`
(`Boards/shimmer_boards.c`). The **actual PCB** and the **programmed part
number** can differ: assembly variants of the same PCB (parts placed/not
placed) are distinguished by programming a higher minor revision.

## Firmware-relevant revision gates

The Shimmer3R "Fourth generation" families share synchronized assembly-variant
minors. Base board IDs never change; the revision signifies the variant.

| Gate | Revisions (>= within same board ID) | Firmware hook |
| --- | --- | --- |
| BMP581 replaces BMP390 | SR31-11-2, SR38-4-2, SR47-8-2, SR48-8-2, SR49-4-2, plus dev build SR48-7-2 | `ShimBrd_isBmp581PresentPerSrNumber()` (log-and-stream-common PR #111 / DEV-818) |
| HSE (16 MHz) load-cap fix 6.8 → 15 pF | SR31-11-2, SR38-4-2, SR47-8-2, SR48-8-2, SR49-4-2 — **not** SR48-7-2 (dev build predates the cap change) | `hseCapFixFitted()` in shimmer3r-firmware `hal_FactoryTest.c` (DEV-866); picks the S3R_TEST_0028 pass limit (±35 ppm fixed / ±100 ppm pre-fix) |
| IM68D121JV01 replaces MP23DB01HP microphone | SR31-11-3, SR38-4-3, SR47-8-3, SR48-8-3, SR49-4-3 | none yet (DEV-686; both mics are PDM, same interface) |
| LIS3MDL + ADXL371 no longer placed | `.1` minors (SR31-11-1 keeps ADXL371; see tables) | `ShimBrd_isLis3mdlPresent()` / `ShimBrd_isAdxl371Present()` |

Notes:

- The crystal cap change is **HSE-only** and is **not** tracked per-row in the
  source workbook (it is bundled into the "Fourth (BMP-581, IM68D121JV01
  fitted, XTAL cap change)" generation column); the `.2` gate above is per
  DEV-866. **The 32 kHz LSE caps stay at 12 pF on all revisions**: hardware
  measurement (2026-08-11, overnight RTC-vs-host drift runs on three boards)
  showed the S3R LSE near-spec at 12 pF (−7 ± 2 ppm — the STM32's pin strays
  complete the load, unlike the Verisense nRF52840 whose identical BOM ran
  +40…+65 ppm fast), while 22 pF over-loads it (−54 / −113 ppm measured).
  A board's boot LSE drive level (factory-test report line "LSE drive applied
  at boot") is the runtime cross-check — production boards lock LOW / run
  MEDIUMLOW.
- Hand-reworked bench units may carry corrected caps and/or a BMP581 under a
  pre-fix revision; revision gates cannot see rework.

### Legacy detection quirks

- **`SRx-x-171` rule:** firmware reports minor revision **171** when it detects
  an LSM303AHTR + BMP280 (second-generation IMU set) on any board ID other
  than SR31, SR47, SR48, SR49 and SR59 — it marks "newer IMUs attached" for
  Consensys on boards whose EEPROM predates the scheme (e.g. SR44-2-171).
- **"Third (no LSM fitted)" programmed minors:** third-generation boards
  assembled without the LSM303AHTR (ICM-20948 serves as WR accel + mag) are
  the same PCB programmed one minor higher: SR31-9-1, SR47-5-1, SR48-4-1/4-2,
  SR49-3-1, SR38-3-1, SR36-3-1.

## Shimmer3R revision quick reference (Fourth generation)

| Family | Board | First proto | Production base | `.1` (LIS3MDL/ADXL371 unplaced) | `.2` (BMP581 + crystal caps) | `.3` (IM68D121 mic) |
| --- | --- | --- | --- | --- | --- | --- |
| IMU | SR31 | SR31-11-0 | SR31-11-0 | SR31-11-1 (keeps ADXL371) | SR31-11-2 | SR31-11-3 |
| ExG | SR47 | SR47-7-0 (BOOT0 ECO: 7-1) | SR47-8-0 | SR47-8-1 | SR47-8-2 | SR47-8-3 |
| GSR+ | SR48 | SR48-6-0, SR48-7-0 (BOOT0 ECO: 7-1; BMP581 dev: 7-2) | SR48-8-0 | SR48-8-1 | SR48-8-2 | SR48-8-3 |
| Bridge Amplifier | SR49 | — | SR49-4-0 | SR49-4-1 | SR49-4-2 | SR49-4-3 |
| Proto3 Deluxe | SR38 | — | SR38-4-0 | SR38-4-1 | SR38-4-2 | SR38-4-3 |

SR47-8-0 / SR48-8-0 fixed design issues with dock detect and VDDIO2 found on
the prototypes; SR48-6-0 (first S3R prototype) additionally has no DETECT_N
wired (`SUPPORT_SR48_6_0` build flag).

## Sensor generations summary

| Generation | Introduced | Pressure | Gyro | LN accel | WR accel | Mag | Alt mag | Mic | BT | High-g |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| First | Q4-2013 | BMP180 | MPU-9150 | KXRB5-2042 | LSM303DLHC | LSM303DLHC | MPU-9150 | — | RN42 | — |
| Second | Q2-2017 | BMP280 | MPU-9250 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | MPU-9250 | — | RN42 | — |
| Third | Q2-2022 | BMP280 | ICM-20948 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | ICM-20948 | — | RN4678 | — |
| Third (no LSM fitted) | Q1-2022 | BMP280 | ICM-20948 | KXTC9-2050 | ICM-20948 | ICM-20948 | ICM-20948 | — | RN4678 | — |
| Fourth | Q4-2025 | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | LIS3MDL (until `.1`) | MP23DB01HP | Vela IF820 | ADXL371 (until `.1`, IMU keeps it) |
| Fourth (BMP-581, IM68D121, XTAL cap change) | Q3-2026 | BMP-581 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | IM68D121JV01 (from `.3`) | Vela IF820 | ADXL371 (IMU only) |

Full electrical/performance specs per generation are in the
"Shimmer3 & 3R Generations" tab of the source workbook.

## Per-product revision tables

Columns: **PCB** = actual PCB revision; **PN** = programmed part number where
it differs; **Uni** = unified PCB; **EEP** = EEPROM fitted; Gen = IMU
generation.

### Shimmer3 / Shimmer3R IMU (SR31)

| PCB | PN | Uni | EEP | TCXO | Pressure | Gyro | LN accel | WR accel | Mag | Alt mag | Mic | BT | High-g | Gen | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| SR31-1-0 to SR31-5-0 | | — | N | Y | BMP180 | MPU-9150 | KXRB5-2042 | LSM303DLHC | LSM303DLHC | MPU-9150 | — | RN42 | — | First | |
| SR31-6-0 | | — | N | ? | BMP280 | MPU-9250 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | MPU-9250 | — | RN42 | — | Second | |
| SR31-7-0 | | — | Y | Y | BMP280 | MPU-9250 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | MPU-9250 | — | RN42 | — | Second | |
| SR31-8-0 to SR31-9-0 | | — | Y | N | BMP280 | ICM-20948 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | ICM-20948 | — | RN4678 | — | Third | |
| SR31-9-0 | SR31-9-1 | — | Y | N | BMP280 | ICM-20948 | KXTC9-2050 | ICM-20948 | ICM-20948 | ICM-20948 | — | RN4678 | — | Third (no LSM) | |
| SR31-10-0 | | — | Y | N | BMP280 | ICM-20948 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | ICM-20948 | — | RN4678 | — | Third | HW provision to program RN4678 FW via MSP430; HW provision for BMP390 test fit |
| SR31-11-0 | | — | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | LIS3MDL | MP23DB01HP | Vela IF820 | ADXL371 | Fourth | Shimmer3R |
| SR31-11-0 | SR31-11-1 | — | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | MP23DB01HP | Vela IF820 | ADXL371 | Fourth | LIS3MDL not placed |
| SR31-11-0 | SR31-11-2 | — | Y | N | BMP-581 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | MP23DB01HP | Vela IF820 | ADXL371 | Fourth | BMP-581 placed |
| SR31-11-0 | SR31-11-3 | — | Y | N | BMP-581 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | IM68D121JV01 | Vela IF820 | ADXL371 | Fourth | IM68D121JV01 placed |

### Shimmer3 / Shimmer3R ExG (SR37, SR47)

| PCB | PN | Uni | EEP | TCXO | Pressure | Gyro | LN accel | WR accel | Mag | Alt mag | Mic | BT | High-g | Gen | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| SR37-1-0 to SR37-3-0 | | N | Y | — | | | | | | | | | | | As per attached Shimmer3 IMU (SR31-1-0 to SR31-7-0) |
| SR47-1-0 to SR47-2-0 | | Y | Y | Y | BMP180 | MPU-9150 | KXRB5-2042 | LSM303DLHC | LSM303DLHC | MPU-9150 | — | RN42 | — | First | |
| SR47-3-0 to SR47-4-0 | | Y | Y | ? | BMP280 | MPU-9250 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | MPU-9250 | — | RN42 | — | Second | |
| SR47-5-0 | | Y | Y | N | BMP280 | ICM-20948 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | ICM-20948 | — | RN4678 | — | Third | |
| SR47-5-0 | SR47-5-1 | Y | Y | N | BMP280 | ICM-20948 | KXTC9-2050 | ICM-20948 | ICM-20948 | ICM-20948 | — | RN4678 | — | Third (no LSM) | |
| SR47-6-0 | | Y | Y | N | BMP280 | ICM-20948 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | ICM-20948 | — | RN4678 | — | Third | RN4678 FW programming provision; SD card location moved back from PCB edge; PCB dimensions corrected |
| SR47-7-0 | | Y | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | LIS3MDL | MP23DB01HP | Vela IF820 | ADXL371 | Fourth | First S3R prototype |
| SR47-7-0 | SR47-7-1 | Y | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | LIS3MDL | MP23DB01HP | Vela IF820 | ADXL371 | Fourth | ECO applied to remove inversion of BOOT0 |
| SR47-8-0 | | Y | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | LIS3MDL | MP23DB01HP | Vela IF820 | ADXL371 | Fourth | Fixed design issues with dock detect and VDDIO2 |
| SR47-8-0 | SR47-8-1 | Y | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | MP23DB01HP | Vela IF820 | — | Fourth | LIS3MDL and ADXL371 not placed |
| SR47-8-0 | SR47-8-2 | Y | Y | N | BMP-581 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | MP23DB01HP | Vela IF820 | — | Fourth | BMP-581 placed |
| SR47-8-0 | SR47-8-3 | Y | Y | N | BMP-581 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | IM68D121JV01 | Vela IF820 | — | Fourth | IM68D121JV01 placed |

### Shimmer3 / Shimmer3R GSR+ (SR14, SR48)

| PCB | PN | Uni | EEP | TCXO | Pressure | Gyro | LN accel | WR accel | Mag | Alt mag | Mic | BT | High-g | Gen | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| SR14-1-0 to SR14-3-0 | | N | Y | — | | | | | | | | | | | As per attached Shimmer3 IMU (SR31-1-0 to SR31-7-0) |
| SR48-1-0 to SR48-2-0 | | Y | Y | Y | BMP180 | MPU-9150 | KXRB5-2042 | LSM303DLHC | LSM303DLHC | MPU-9150 | — | RN42 | — | First | |
| SR48-3-0 | | Y | Y | ? | BMP280 | MPU-9250 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | MPU-9250 | — | RN42 | — | Second | |
| SR48-4-0b | SR48-4-0 | Y | Y | N | BMP280 | ICM-20948 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | ICM-20948 | — | RN4678 | — | Third | |
| SR48-4-0a | SR48-4-1 | Y | Y | N | BMP280 | ICM-20948 | KXTC9-2050 | ICM-20948 | ICM-20948 | ICM-20948 | — | RN4678 | — | Third (no LSM) | SR48-4-0a had two PCB errors, resolved in SR48-4-0b |
| SR48-4-0b | SR48-4-2 | Y | Y | N | BMP280 | ICM-20948 | KXTC9-2050 | ICM-20948 | ICM-20948 | ICM-20948 | — | RN4678 | — | Third (no LSM) | |
| SR48-5-0 | | Y | Y | N | BMP280 | ICM-20948 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | ICM-20948 | — | RN4678 | — | Third | RN4678 FW programming provision; SD card location moved back from PCB edge; PCB dimensions corrected |
| SR48-6 | SR48-6-0 | Y | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | LIS3MDL | MP23DB01HP | Vela IF820 | ADXL371 | Fourth | First S3R prototype (`SUPPORT_SR48_6_0`) |
| SR48-7-0 | SR48-7-0 | Y | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | LIS3MDL | MP23DB01HP | Vela IF820 | ADXL371 | Fourth | Second S3R prototype, first-round issues resolved |
| SR48-7-0 | SR48-7-1 | Y | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | LIS3MDL | MP23DB01HP | Vela IF820 | ADXL371 | Fourth | ECO applied to remove inversion of BOOT0 |
| SR48-7-0 | SR48-7-2 | Y | Y | N | BMP-581 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | LIS3MDL | MP23DB01HP | Vela IF820 | ADXL371 | Fourth | BMP-581 placed on old rev for development of SR48-8-2 |
| SR48-8-0 | | Y | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | LIS3MDL | MP23DB01HP | Vela IF820 | ADXL371 | Fourth | Fixed design issues with dock detect and VDDIO2 |
| SR48-8-0 | SR48-8-1 | Y | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | MP23DB01HP | Vela IF820 | — | Fourth | LIS3MDL and ADXL371 not placed |
| SR48-8-0 | SR48-8-2 | Y | Y | N | BMP-581 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | MP23DB01HP | Vela IF820 | — | Fourth | BMP-581 placed |
| SR48-8-0 | SR48-8-3 | Y | Y | N | BMP-581 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | IM68D121JV01 | Vela IF820 | — | Fourth | IM68D121JV01 placed |

### Shimmer3 / Shimmer3R Bridge Amplifier (SR8, SR49)

| PCB | PN | Uni | EEP | TCXO | Pressure | Gyro | LN accel | WR accel | Mag | Alt mag | Mic | BT | High-g | Gen | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| SR8-1-0 to SR8-4-0 | | N | Y | — | | | | | | | | | | | As per attached Shimmer3 IMU (SR31-1-0 to SR31-7-0) |
| SR49-1-0 | | Y | Y | Y | BMP180 | MPU-9150 | KXRB5-2042 | LSM303DLHC | LSM303DLHC | MPU-9150 | — | RN42 | — | First | |
| SR49-2-0 | | Y | Y | ? | BMP280 | MPU-9250 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | MPU-9250 | — | RN42 | — | Second | |
| SR49-3-0 | | Y | Y | N | BMP280 | ICM-20948 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | ICM-20948 | — | RN4678 | — | Third | RN4678 FW programming provision; BMP390 test-fit provision |
| SR49-3-0 | SR49-3-1 | Y | Y | N | BMP280 | ICM-20948 | KXTC9-2050 | ICM-20948 | ICM-20948 | ICM-20948 | — | RN4678 | — | Third (no LSM) | |
| SR49-4-0 | | Y | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | LIS3MDL | MP23DB01HP | Vela IF820 | ADXL371 | Fourth | Shimmer3R |
| SR49-4-0 | SR49-4-1 | Y | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | MP23DB01HP | Vela IF820 | — | Fourth | |
| SR49-4-0 | SR49-4-2 | Y | Y | N | BMP-581 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | MP23DB01HP | Vela IF820 | — | Fourth | BMP-581 placed |
| SR49-4-0 | SR49-4-3 | Y | Y | N | BMP-581 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | IM68D121JV01 | Vela IF820 | — | Fourth | IM68D121JV01 placed |

### Shimmer3 / Shimmer3R Proto3 Deluxe (SR38)

| PCB | PN | Uni | EEP | TCXO | Pressure | Gyro | LN accel | WR accel | Mag | Alt mag | Mic | BT | High-g | Gen | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| SR38-1-0 to SR38-2-0 | | N | Y | — | | | | | | | | | | | As per attached (glued) Shimmer3 IMU (SR31-1-0 to SR31-7-0) |
| SR38-3-0 | | N | Y (not populated; on >=SR31-9-0) | N | BMP280 | ICM-20948 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | ICM-20948 | — | RN4678 | — | Third | As per attached (glued) Shimmer3 IMU (SR31-9-0) |
| SR38-3-0 | SR38-3-1 | N | | N | BMP280 | ICM-20948 | KXTC9-2050 | ICM-20948 | ICM-20948 | ICM-20948 | — | RN4678 | — | Third (no LSM) | As per attached (glued) Shimmer3 IMU (SR31-9-1) |
| SR38-4-0 | | N | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | LIS3MDL | MP23DB01HP | Vela IF820 | ADXL371 | Fourth | Shimmer3R |
| SR38-4-0 | SR38-4-1 | N | Y | N | BMP-390 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | MP23DB01HP | Vela IF820 | — | Fourth | |
| SR38-4-0 | SR38-4-2 | N | Y | N | BMP-581 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | MP23DB01HP | Vela IF820 | — | Fourth | BMP-581 placed |
| SR38-4-0 | SR38-4-3 | N | Y | N | BMP-581 | LSM6DSV | LSM6DSV | LIS2DW12 | LIS2MDL | — | IM68D121JV01 | Vela IF820 | — | Fourth | IM68D121JV01 placed |

### Shimmer3 Proto3 Mini (SR36)

| PCB | PN | Uni | EEP | TCXO | Pressure | Gyro | LN accel | WR accel | Mag | Alt mag | Mic | BT | High-g | Gen | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| SR36-1-0 to SR36-2-0 | | N | Y | — | | | | | | | | | | | As per attached Shimmer3 IMU (SR31-1-0 to SR31-7-0) |
| SR36-3-0 | | N | Y (not populated; on >=SR31-9-0) | N | BMP280 | ICM-20948 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | ICM-20948 | — | RN4678 | — | Third | As per attached Shimmer3 IMU >=SR31-9-0 (SR36-3-0 programmed by user in Consensys) |
| SR36-3-0 | SR36-3-1 | N | | N | BMP280 | ICM-20948 | KXTC9-2050 | ICM-20948 | ICM-20948 | ICM-20948 | — | RN4678 | — | Third (no LSM) | As per SR31-9-1; actual board rev SR36-3-0, SR36-3-1 programmed by user in Consensys |
| SR36-4-0 | | N | | N | BMP280 | ICM-20948 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | ICM-20948 | — | RN4678 | — | Third | As per attached Shimmer3 IMU SR31; RN4678 FW programming provision; BMP390 test-fit provision |

### Shimmer3 200g (SR44)

| PCB | PN | Uni | EEP | TCXO | Pressure | Gyro | LN accel | WR accel | Mag | Alt mag | Mic | BT | High-g | Gen | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| SR44-1-0 | | N | Y | — | | | | | | | | | ADXL377 | | As per attached (glued) Shimmer3 IMU (SR31-1-0 to SR31-7-0) |
| SR44-2-0 | | N | Y (not populated; on >=SR31-9-0) | N | BMP280 | ICM-20948 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | ICM-20948 | — | RN4678 | ADXL377 | Third | FW reports SR44-2-171 to indicate newer IMUs attached (see `SRx-x-171` rule) |

### Any board (`SRx-x-171` rule)

| PCB | PN | Pressure | Gyro | LN accel | WR accel | Mag | BT | Gen | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| X | SRx-x-171 | BMP280 | MPU-9250 | KXTC9-2050 | LSM303AHTR | LSM303AHTR | RN42 | Second | Firmware reports minor 171 if both LSM303AHTR and BMP280 are detected and the board ID is anything other than SR31, SR47, SR48, SR49 and SR59 |
