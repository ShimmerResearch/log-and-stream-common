# Shimmer3 / Shimmer3R EEPROM Memory Map

The CAT24C16 serial EEPROM holds three things the configuration bytes do not:
the expansion-board identity, the Bluetooth radio settings, and the custom
branding record that decides what name the device advertises under. On
Shimmer3R it also holds a backup of the Bluetooth MAC, used only while the
radio module is factory reset (§7).

Unlike the configuration bytes, this content is **not** part of a trial's
configuration — it describes the hardware and survives a configuration reset.

> **Verified against** — the revisions these byte-level claims were read from.
> A pinned commit is a citation, not a claim of currency.
>
> - **Firmware:** `log-and-stream-common` @ `f3cf73e` —
>   `EEPROM/shimmer_eeprom.{h,c}` in full.
> - **Platform firmware:** `shimmer3r-firmware` @ `a8f105e5` —
>   `Shimmer_Driver/CAT24C16/CAT24C16.h` (`CAT24C16_PAGE_SIZE`,
>   `CAT24C16_TOTAL_SIZE`); `shimmer3-firmware` @ `2765ff4`.

> **How to read this document.** **S3** = Shimmer3 (MSP430); **S3R** =
> Shimmer3R (STM32U5). LogAndStream only.

**Source references:**

| Layer | File |
|---|---|
| Address map, record layouts | `EEPROM/shimmer_eeprom.h` |
| Validation, seeding, accessors | `EEPROM/shimmer_eeprom.c` |
| Device driver | platform `Shimmer_Driver/CAT24C16/` |
| Host access | [SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md) — expansion-board memory commands |

---

## 1. The device

| Property | Value |
|---|---:|
| Part | CAT24C16 |
| Total size | 2048 bytes |
| Page size | 16 bytes |
| Pages | 128 |
| Bus | I2C |

`ShimEeprom_isPresent()` reports whether the chip answered. **Everything in
this document is conditional on that** — a board without an EEPROM falls back
to compile-time defaults throughout.

## 2. Address map

Regions grow **downward from the top of the device**, which is why adding the
brand record moved nothing else:

| Start | End | Size | Constant | Content |
|---:|---:|---:|---|---|
| 0 | 15 | 16 | `EEPROM_ADDRESS_HW_DETAILS` | Expansion-board ID page (§3) |
| 16 | 1919 | 1904 | — | General expansion-board storage, host-writable |
| 1920 | 1935 | 16 | `EEPROM_ADDRESS_POWER_PROFILE` | Power-profile record, **S3R only** (§8). Unused on S3, where it stays general storage |
| 1936 | 1951 | 16 | `EEPROM_ADDRESS_BT_MAC_BACKUP` | Bluetooth MAC backup record, **S3R only** (§7). Unused on S3, where it stays general storage |
| 1952 | 2031 | 80 | `EEPROM_ADDRESS_BRAND_DETAILS` | Brand record, 5 pages (§5) |
| 2032 | 2047 | 16 | `EEPROM_ADDRESS_BLUETOOTH_DETAILS` | Radio settings page (§4) |

Derived as:

```
EEPROM_ADDRESS_BLUETOOTH_DETAILS = CAT24C16_TOTAL_SIZE - CAT24C16_PAGE_SIZE
                                 = 2048 - 16 = 2032
EEPROM_BRAND_DETAILS_SIZE        = 5 * CAT24C16_PAGE_SIZE = 80
EEPROM_ADDRESS_BRAND_DETAILS     = 2032 - 80 = 1952
EEPROM_ADDRESS_BT_MAC_BACKUP     = 1952 - 16 = 1936   (S3R only)
EEPROM_ADDRESS_POWER_PROFILE     = 1936 - 16 = 1920   (S3R only)
EEPROM_AVAILABLE_SIZE            = 2048 - 16 = 2032
```

> **`EEPROM_AVAILABLE_SIZE` (2032) is the host-offset bound, and it covers the
> brand record deliberately.** It is the limit the daughter-card memory
> commands enforce — `ShimEeprom_writeDaughterCardMem` accepts a write while
> `writeEnd < EEPROM_AVAILABLE_SIZE` — so every host offset from 0 to 2031 is
> addressable, the brand record among them. That is not an oversight: writing
> the record through `SET_DAUGHTER_CARD_MEM` is how a host provisions
> advertising names, and the firmware re-reads the record whenever a write
> overlaps it (`ShimEeprom_readBrandDetails`), precisely so the change takes
> effect.
>
> The same is true one page further up. The Bluetooth details page is reachable
> at host offsets 2016-2031, and the firmware re-reads the sensor settings page
> when a write overlaps the radio-settings byte at host offset 2018.
>
> So the caution for a host is not that the constant is too large — it is that
> this range is **not empty**. Writing arbitrary data across the top of it
> overwrites the brand record at host 1936-2015 and the radio settings above
> it. Bound your own writes to the region you own, not to the constant.

> **`EEPROM_ADDRESS_BLUETOOTH_DETAILS_HOST_OFFSET_ALIAS` (2016) is one page
> below the Bluetooth page.** It exists because a host addressing the
> expansion-board memory works in a different offset space from the firmware's
> absolute addresses. Do not treat it as a second copy of the settings.

## 3. Expansion-board ID page (0-15)

`daughter_card_id_page`, one 16-byte page:

| Offset | Field | Constant |
|---:|---|---|
| 0 | Expansion board ID | `DAUGHT_CARD_ID` |
| 1 | Major revision | `DAUGHT_CARD_REV` |
| 2 | Special revision | `DAUGHT_CARD_SPECIAL_REV` |
| 3-15 | Padding | — |

These three bytes are copied into the SD-card file header at
`SDH_DAUGHTER_CARD_ID_BYTE0` (offsets 214-216) so a logged trial records which
expansion board was fitted. See
[SHIMMER3_SD_CARD_FORMAT.md](SHIMMER3_SD_CARD_FORMAT.md) §3.1 and
[SHIMMER3_BOARD_REVISIONS.md](SHIMMER3_BOARD_REVISIONS.md) for what the IDs
mean.

## 4. Radio settings page (2032-2047)

`gEepromSensorSettings`, one 16-byte page:

| Offset | Field | Type | Notes |
|---:|---|---|---|
| 0 | `radioHwVer` | `uint8` | See §4.1 |
| 1 | `baudRate` | `uint8` | Encoding differs by platform, §4.3 |
| 2 bit 0 | `bleEnabled` | 1 bit | §4.2 |
| 2 bit 1 | `btClassicEnabled` | 1 bit | §4.2 |
| 2 bit 2 | `usbHighSpeed` | 1 bit | 1 = high speed, 0 = full speed. **Shimmer3R only** |
| 2 bits 3-7 | unused | 5 bits | |
| 3-4 | `btCntDisconnectWhileStreaming` | `uint16` | S3 RN4678 error count |
| 5-6 | `btCntUnsolicitedReboot` | `uint16` | S3 RN4678 error count |
| 7-8 | `btCntRtsLockup` | `uint16` | S3 RN4678 error count |
| 9-10 | `btCntDataRateTestBlockage` | `uint16` | S3 RN4678 error count |
| 11-15 | Padding | 5 bytes | |

### 4.1 Radio hardware version

| Value | Constant | Part |
|---:|---|---|
| 0 | `RN42` | Microchip RN42 |
| 1 | `RN4678` | Microchip RN4678 |
| 2 | `RN41` | Microchip RN41 |
| 3 | `CYW20820` | Infineon CYW20820 |
| 0xFF | `BT_HW_VER_UNKNOWN` | |

> **The enumeration is not in chronological or numeric part order** — RN41 is 2
> while RN42 is 0. Do not infer capability from ordering.

### 4.2 The radio-enable bits

`bleEnabled` and `btClassicEnabled` decide which radio mode the firmware brings
up. `ShimConfig_checkBtModeFromConfig` compares them against the current state
and stops the radio if they disagree.

> **This is the first thing to check on any classic-Bluetooth problem.** The
> firmware selects the radio mode from these two bits, and
> `ShimEeprom_updateRadioDetails` explicitly leaves them alone —
> "*leave `eepromSensorSettingsPage.bleDisabled` as is*". So a device whose
> EEPROM was never seeded with the right bit will not come up in classic mode,
> and nothing in the normal boot path corrects it. The symptom is a device that
> is simply not discoverable over classic Bluetooth, with no error anywhere.

> **Only `bleEnabled` is validated, and only in one direction.**
> `ShimEeprom_areRadioDetailsIncorrect` flags a mismatch when
> `!ShimBrd_doesDeviceSupportBle() && bleEnabled != 0` — a board that cannot do
> BLE must not claim it. There is no equivalent check on `btClassicEnabled`,
> and no check that at least one of the two is set.

### 4.3 Baud rate

Stored differently on the two platforms:

| Platform | Stored as |
|---|---|
| S3 | The baud enum directly. Forced to `BAUD_115200` when the radio is an RN41 or RN42 |
| S3R | `ShimEeprom_baudRateToEnum(ShimBt_getBtBaudRateToUse())` |

`ShimEeprom_areRadioDetailsIncorrect` returns true when the stored radio
version disagrees with the detected one, when the baud rate is `BAUD_INVALID`,
or when the platform-specific rules above are violated;
`ShimEeprom_updateRadioDetails` then rewrites the page.

### 4.4 Bluetooth error counts (Shimmer3 only)

Four 16-bit counters recording RN4678 faults seen in the field.
`ShimEeprom_checkBtErrorCounts` treats `0xFFFF` in *any* of the four as
"uninitialised" and calls `ShimEeprom_resetBtErrorCounts` to zero **all four**.

> **One corrupted counter resets the other three.** The check is an OR across
> all four fields but the reset is unconditional across all four, so a single
> `0xFFFF` discards three counters' worth of field history.

On Shimmer3R these bytes are declared but the functions that maintain them are
inside `#if defined(SHIMMER3)`.

## 5. Brand record (1952-2031)

Eighty bytes — five pages — defining the names the device presents over classic
Bluetooth, BLE and USB. This is what makes custom-branded units possible
without a custom firmware build.

### 5.1 Layout

| Offset | Field | Size | Notes |
|---:|---|---:|---|
| 0-1 | `magic` | 2 | `EEPROM_BRAND_MAGIC` = `0x5342` |
| 2 | `layoutVer` | 1 | `EEPROM_BRAND_LAYOUT_VER` = 2 |
| 3 | `flags` | 1 | §5.3 |
| 4 | `btClassicLen` | 1 | Used length of `btClassic` |
| 5 | `bleLen` | 1 | |
| 6 | `usbProductLen` | 1 | |
| 7 | `usbManufacturerLen` | 1 | |
| 8-23 | `btClassic` | 16 | Classic Bluetooth name prefix |
| 24-33 | `ble` | 10 | BLE name prefix |
| 34-49 | `usbProduct` | 16 | USB product string prefix |
| 50-73 | `usbManufacturer` | 24 | USB `iManufacturer` |
| 74-77 | Padding | 4 | |
| 78-79 | `crc` | 2 | CRC-16 over bytes 0-77 |

> **`magic` is `0x5342` stored little-endian, so a hexdump reads `42 53` —
> "BS", not "SB".** The header comment says so explicitly. Tools that search a
> dump for the ASCII "SB" will not find it.

> **The name fields are length-prefixed, not NUL-terminated.** The length bytes
> at offsets 4-7 are authoritative; the character arrays are fixed-width and
> whatever follows the used length is not guaranteed to be anything. Reading
> them as C strings is wrong.

> **They are name *prefixes*.** The device appends its own identifier, so a
> `btClassic` of `Shimmer3` becomes something like `Shimmer3-ABCD` in the
> advertised name. The 16- and 10-character limits are on the prefix alone.

### 5.2 Validation

`ShimEeprom_isBrandRecordValid` requires **all** of:

1. `magic == 0x5342`
2. `layoutVer == 2`
3. All four name fields valid per §5.4
4. `checkCrc(CRC_2BYTES_ENABLED, rawBytes, 78)` passes

The CRC is the same comms CRC used on the Bluetooth link, over the first 78
bytes — the record excluding its own CRC field.

### 5.3 Flags byte

| Bits | Mask | Meaning |
|---|---|---|
| 0 | `EEPROM_BRAND_FLAG_RESERVED0` (`0x01`) | Reserved |
| 1-2 | `EEPROM_BRAND_PLATFORM_MASK` (`0x06`) | Platform, shifted by `EEPROM_BRAND_PLATFORM_SHIFT` = 1 |
| 3-7 | — | Unused |

| Platform value | Constant |
|---:|---|
| 0 | `BRAND_PLATFORM_UNKNOWN` |
| 1 | `BRAND_PLATFORM_SHIMMER3` |
| 2 | `BRAND_PLATFORM_SHIMMER3R` |
| 3 | `BRAND_PLATFORM_SHIMMER4_SDK` |

> **The platform field is recorded but never enforced.** The seeding code
> stamps `BRAND_PLATFORM_CURRENT`, and the validity check ignores the field
> entirely. The source explains why: *"A valid record is always honoured —
> there is no re-seeding based on which platform wrote it, as the unified PCB
> means the EEPROM cannot move between hardware models."* So a record stamped
> Shimmer3 will be used unchanged on a Shimmer3R.

### 5.4 Name field validation

`ShimEeprom_isBrandNameFieldValid` rejects a field when:

- the length is **0**, or greater than the field's maximum, or
- any character in the used length is outside `0x20`-`0x7E`, or
- any character is a **comma**.

> **A comma is rejected specifically**, on top of the printable-ASCII rule.
> The reason is not stated in the source, but commas are field separators in
> several of the text formats these names end up in.

> **An empty name is invalid, not "use the default".** A zero length fails
> validation, which invalidates the *whole record* — including the other three
> names. Blanking one field to fall back to a default silently reverts all four.

### 5.5 Seeding

`ShimEeprom_readBrandDetails(seedIfInvalid)`:

1. Read 80 bytes.
2. Validate.
3. If `seedIfInvalid` and invalid, write a record built from the compile-time
   defaults and re-validate.
4. Refresh the four NUL-terminated working strings — from the record if valid,
   from the compile-time defaults if not.

Compile-time defaults:

| Platform | `btClassic` | `ble` | `usbProduct` |
|---|---|---|---|
| Shimmer3 | `Shimmer3` | `S3BLE` | `Shimmer` |
| Shimmer3R | `Shimmer3R` | `Shimmer3R` | `Shimmer` |
| Shimmer4 SDK | `Shimmer4` | `S4BLE` | `Shimmer` |

`usbManufacturer` is `Shimmer Research Ltd.` on all platforms.

> **`seedIfInvalid` must be 0 when re-reading after a host write.** The
> parameter's documentation is explicit: a host writing the record in several
> chunks would otherwise be raced by a seed write the moment an intermediate
> state fails validation. The boot path passes 1; the post-write re-read passes
> 0.

> **On Shimmer3R the BLE default is the full `Shimmer3R`, not an `S3RBLE`
> abbreviation.** The other two platforms use short `S3BLE` / `S4BLE` forms.
> The BLE field's 10-character maximum is why the older platforms abbreviate;
> `Shimmer3R` is nine characters and fits.

### 5.6 Accessors

| Function | Returns |
|---|---|
| `ShimEeprom_isBrandValid()` | Whether the stored record passed validation |
| `ShimEeprom_getBrandBtClassic()` | NUL-terminated classic BT prefix |
| `ShimEeprom_getBrandBle()` | NUL-terminated BLE prefix |
| `ShimEeprom_getBrandUsbProduct()` | NUL-terminated USB product prefix |
| `ShimEeprom_getBrandUsbManufacturer()` | NUL-terminated USB manufacturer |

The accessors always return a usable string — the defaults when the record is
invalid — so callers never need to check validity first.

## 6. Host access

The general expansion-board memory commands read and write this device by
offset. Because those offsets reach the brand record and the radio settings
page, a host **can** corrupt either.

Two protections exist, and neither is complete:

- The brand record's CRC means a corrupted record fails validation and the
  firmware falls back to defaults rather than advertising garbage.
- `ShimEeprom_readBrandDetails(0)` is called after a host write so the working
  strings track what was written.

> **There is no write protection on the radio settings page.** A host write to
> offsets 2032-2047 changes the radio configuration with no validation at write
> time; the error only surfaces at the next boot, when
> `ShimEeprom_areRadioDetailsIncorrect` may or may not catch it depending on
> which field was damaged (§4.2).

## 7. Bluetooth MAC backup record (1936-1951, Shimmer3R only)

A factory reset of the CYW20820 (EZ-Serial `/RFAC`) wipes the Bluetooth
address the module was given. The firmware-driven reset (`btFactoryResetViaFw`
in the platform firmware) saves the MAC here before the reset and restores it
from here afterwards. Nothing else reads or writes this page, and nothing runs
the reset automatically.

`gEepromBtMacBackup`, one 16-byte page (host offset 1920):

| Offset | Field | Type | Notes |
|---:|---|---|---|
| 0-1 | `magic` | `uint16` LE | `EEPROM_BT_MAC_BACKUP_MAGIC` = `0x4D42`; a raw dump reads `42 4D` |
| 2 | `layoutVer` | `uint8` | `EEPROM_BT_MAC_BACKUP_LAYOUT_VER` = 1 |
| 3 | `state` | `uint8` | 0 = `BT_MAC_BACKUP_IDLE`, 1 = `BT_MAC_BACKUP_FR_PENDING` |
| 4-9 | `mac` | 6 bytes | Module byte order, as the CYW20820 returns it |
| 10-13 | Padding | 4 bytes | `0xFF` |
| 14-15 | `crc` | 2 bytes | Comms CRC over bytes 0-13 |

`ShimEeprom_readBtMacBackup` accepts the record only if the magic, layout
version and CRC all match **and** the MAC is valid: not all `0x00`, which a
fresh module can report, and not all `0xFF`, which is erased EEPROM.
`ShimEeprom_writeBtMacBackup` reads the record back and reports success only
on a match.

The reset sequence:

1. Read the module's MAC.
2. If the record is valid and `FR_PENDING`, an earlier reset never confirmed
   its restore, so the module may already be on its post-reset address. The
   saved MAC is used and the record is left alone.
3. Otherwise the module's MAC must be valid and must verify in the record as
   `FR_PENDING`. If either fails, the module is **not** reset.
4. `/RFAC`, then restore the saved MAC with flash-scoped `SBA`.
5. Read the MAC back. On a match the state returns to `IDLE`. On a mismatch
   it stays `FR_PENDING`, so the next reset restores it.

The two EEPROM steps run from main context, never from the UART receive
interrupt that drives the rest of the sequence. The I2C transfer waits out
the EEPROM write cycle with `HAL_Delay`.

> **The page sits inside the host-writable window.** A host write to host
> offsets 1920-1935 overwrites the record. The CRC means a damaged record is
> simply ignored: the reset then falls back to the module's own MAC, or
> refuses to run if that is invalid too. The reverse risk is that a reset
> overwrites whatever a host stored in this page. Before this record existed,
> the page was general expansion-board storage.

## 8. Power-profile record (1920-1935, Shimmer3R only)

The arm, state and outcome of the power-profile run that a production test
arms over the dock
([SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md) §6). The
dock's arm and disarm write it, and so does the run at each step boundary;
nothing else does. Firmware before v1.01.024 does not touch the page.

One 16-byte page (host offset 1904), built byte by byte by
`ShimFactoryRecord_encodePpRecord` (`Test/shimmer_factory_record.c`), so it
does not depend on a compiler's struct packing:

| Offset | Field | Type | Notes |
|---:|---|---|---|
| 0-1 | `magic` | `uint16` LE | `PP_RECORD_MAGIC` = `0x5050` ("PP") |
| 2 | `layoutVer` | `uint8` | `PP_RECORD_LAYOUT_VER` = 1 |
| 3 | `state` | `uint8` | `PP_STATE_*`: 0 disarmed, 1 armed, 2 running, 3 done, 4 skipped, 5 aborted |
| 4 | `run_seq` | `uint8` | index of the step the run is on, written before each step |
| 5-6 | `nonce` | `uint16` LE | from the arm |
| 7-10 | `step_ok` | `uint32` LE | bit *i* set when step *i* completed |
| 11-12 | `parts_found` | `uint16` LE | `PP_PART_*` |
| 13 | schedule version | `uint8` | of the schedule the run used |
| 14-15 | `crc` | 2 bytes | `ShimSwCrc_calc` over bytes 0-13, low byte first: the comms CRC |

`ShimEeprom_readPowerProfile` accepts the page only if the magic, layout
version and CRC match; anything else, an erased page included, reads as no
record. `ShimEeprom_writePowerProfile` reads the page back and reports success
only on a byte-for-byte match: the dock ACKs an arm only then.

> **The page sits inside the host-writable window**, as the MAC backup does. A
> host write to host offsets 1904-1919 overwrites the record, and a damaged
> record reads as none, which disarms the unit. The page is clear of the
> region the factory EEPROM test uses as scratch (bytes 16-143).

## Still unverified / not found in code

- **What occupies bytes 16-1919.** Confirmed: no firmware structure claims
  the range and no default content is written. What *is* now established is
  the host access rule: `CARD_MEM` reads address `offset + 16` with
  `offset ≤ 2031`, so the host window is bytes 16-2047, and writes go through
  `ShimEeprom_writeDaughterCardMem`, which rejects zero-length and
  out-of-range writes and re-validates the brand record if a write overlaps
  it. The range is expansion-board storage by convention, not by declaration.
- **`EEPROM_BRAND_FLAG_RESERVED0`.** Defined as `0x01`, never read or written.
- ~~The exact form of the appended device identifier~~ — resolved
  (`log_and_stream_common.c`): `LogAndStream_buildShimmerMacSuffix` takes
  characters 8-11 of the 12-character MAC string — the **last four hex digits**
  — and substitutes `XXXX` if the MAC is not yet known.
  `LogAndStream_buildShimmerPrefix` produces `"<USB product> <SUFFIX>"`, e.g.
  `Shimmer AAAA`, where the product string is the brand record's USB product
  field (default `Shimmer`); the caller must provide 22 bytes (16-character
  brand + space + 4 + NUL).
- **Why commas are rejected in name fields.** The check is explicit; no comment
  explains it.
- ~~Whether `usbHighSpeed` is honoured~~ — resolved: yes, on Shimmer3R.
  `Core/Src/usb_otg.c` calls
  `USB_setSpeed(ShimEeprom_getSensorSettingsPage()->usbHighSpeed ? USB_SPEED_HIGH : USB_SPEED_FULL)`
  when the USB stack is brought up, so the bit selects the descriptor set and
  PHY speed at the next USB initialisation.
- **The layout-version-1 brand record.** `EEPROM_BRAND_LAYOUT_VER` is 2 and a
  version-1 record would fail validation and be re-seeded, so its layout is
  irrecoverable from the current source. If fielded units carry v1 records,
  they will have been silently reset to defaults on first boot with this
  firmware.
