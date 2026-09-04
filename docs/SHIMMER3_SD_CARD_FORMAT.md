# Shimmer3 / Shimmer3R SD Card Format

Everything a Shimmer3 or Shimmer3R writes to, or reads from, its microSD card:
the directory layout, the binary data-file header, the sample records that
follow it, the `sdlog.cfg` text configuration file, and the calibration file.

This is the on-disk counterpart to
[SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) — the
channel encodings are identical, only the framing around them differs.

> **Verified against** — the revisions these byte-level claims were read from.
> A pinned commit is a citation, not a claim of currency: when the firmware
> moves on, this document needs re-checking against it rather than the stamp
> being wrong.
>
> - **Firmware (authority for bytes):** `log-and-stream-common` @ `f3cf73e` —
>   `SDCard/shimmer_sd_header.{h,c}` (`SD_HEAD_SIZE`, the `SDH_*` offsets,
>   `ShimSdHead_config2SdHead`, `ShimSdHead_saveBmpCalibrationToSdHeader`),
>   `SDCard/shimmer_sd_data_file.{h,c}` (directory and file naming, the write
>   buffers, file rollover, `ShimSdDataFile_writeSdHeaderToFile`),
>   `SDCard/shimmer_sd_cfg_file.c` (`sdlog.cfg` generation and parsing),
>   `SDCard/shimmer_sd.c` (mount, self-test, timestamping),
>   `Calibration/shimmer_calibration.c` (`ShimCalib_ram2File`),
>   `Sensing/shimmer_sensing.c` (what is handed to the SD writer per sample).
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4`;
>   `shimmer3r-firmware` @ `a8f105e5`.

> **How to read this document.** **S3** = Shimmer3 (MSP430, LogAndStream);
> **S3R** = Shimmer3R (STM32U5). Where the two generations differ, both are
> given. This document covers **LogAndStream** only; SDLog wrote a related but
> not identical layout.

**Source references:**

| Layer | File |
|---|---|
| Header offsets and assembly | `SDCard/shimmer_sd_header.{h,c}` |
| Directory / file naming, rollover | `SDCard/shimmer_sd_data_file.{h,c}` |
| `sdlog.cfg` | `SDCard/shimmer_sd_cfg_file.c` |
| Mount and self-test | `SDCard/shimmer_sd.c` |
| Calibration file | `Calibration/shimmer_calibration.c` |
| Channel encodings | [SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §5 |
| Configuration byte meanings | [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) |

---

## 1. Card layout

A card in service holds up to four things:

```
/
├── sdlog.cfg                          text configuration (§5)
├── Calibration/
│   └── calib_<mac4>                   calibration blob (§6)
└── data/
    └── <experimentId>_<configTime>/    one directory per trial
        └── <shimmerName>-<NNNN>/       one directory per logging session
            ├── 000                     binary data file
            ├── 001
            └── ...
```

The `Calibration` directory is matched case-insensitively in the sense that the
firmware tries `/Calibration` first and `/calibration` second; if neither
exists it creates `/Calibration`.

### 1.1 Experiment directory

```
data/<experimentId>_<configTime>
```

- `<experimentId>` is the 12-character experiment ID from configuration bytes
  199-210, parsed to printable text.
- `<configTime>` is the 32-bit config time from bytes 211-214, rendered as
  decimal text.

Both come from `ShimConfig_expIdParseToTxtAndPtrGet()` and
`ShimConfig_configTimeParseToTxtAndPtrGet()`. Every device in a multi-device
trial that shares the same experiment ID and config time writes into a
same-named directory on its own card, which is what makes the trials line up
when the cards are collected.

### 1.2 Session directory

```
<experimentDir>/<shimmerName>-<NNNN>
```

`<shimmerName>` is the 12-character device name from bytes 187-198.
`<NNNN>` is a four-digit zero-padded counter.

The counter is chosen by scanning the experiment directory for existing
directories whose name matches this device's name, taking the highest suffix
found and adding one. So each time logging starts, a **new** session directory
appears; nothing is ever appended to an existing one.

> **The scan compares with `strncmp(fname, shimmerName, strlen(fname) - 4)`** —
> the length is taken from the *directory entry*, not from the device name.
> A directory whose name is shorter than expected therefore compares fewer
> characters, and one that is longer compares beyond the device name. In
> practice the names are fixed-width so this works, but a hand-created
> directory in the experiment folder can disturb the numbering.

### 1.3 Data files

Files within a session directory are named `000`, `001`, `002`, … — three
digits, zero-padded, no extension, incrementing from zero via `fileNum++`.

> **No extension, and the name is only three digits.** Past 999 the `sprintf`
> with `%03d` simply produces four digits, so the sequence continues correctly,
> but tools that assume a fixed-width name will break. At the one-hour rollover
> of §4.1, reaching 999 takes about 41 days of continuous logging.

## 2. Data file structure

```
+----------------------------------+
| SD header       SD_HEAD_SIZE     |   256 bytes (S3) / 384 bytes (S3R)
+----------------------------------+
| sample record 1                  |   3 + sum(channel widths)
| sample record 2                  |
| ...                              |
+----------------------------------+
```

### 2.1 Sample records

Each record is:

```
+---------------+------------------------+
| timestamp     | channel data           |
| 3 bytes, LE   | sum of channel widths  |
+---------------+------------------------+
```

The firmware writes `&dataBufferPtr[PACKET_TIMESTAMP_IDX]` for
`sensing.dataLen - 1` bytes, so relative to a Bluetooth data packet the SD
record:

- **omits the leading `0x00` data-packet header byte**, and
- **carries no CRC**, whatever the Bluetooth CRC mode is set to.

Everything else — the 3-byte little-endian 32768 Hz timestamp, the channel
order, and every channel encoding — is identical to the streamed form. See
[SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §2, §4
and §5.

> **Records are fixed-length within a file and only within a file.** The length
> is `3 + sum of the widths of the enabled channels`, which is fixed for a
> logging session because configuration cannot change while sensing. A parser
> must compute it from the header's channel information, then treat the
> remainder of the file as a flat array of that stride. There is no
> record delimiter and no record count.

> **The timestamp wraps every 512 seconds** and files roll over every hour
> (§4.1), so a file contains roughly seven wraps. Unwrap as described in the
> streaming document; the header's initial timestamp (§3.3) gives the absolute
> anchor for the first record.

### 2.2 Header size differs by generation

| Platform | `SD_HEAD_SIZE` |
|---|---:|
| Shimmer3 | 256 |
| Shimmer3R | 384 |

A parser must know the generation before it can find the first sample record.
The generation is in the header itself at `SDH_SHIMMERVERSION_BYTE_0/1`
(offsets 30-31), which is within the first 256 bytes on both — so read the
first 256 bytes, determine the generation, then read the remainder if needed.

## 3. The SD header

The header is filled with `0xFF` and then populated, so **any offset not listed
here is `0xFF`**, not zero. That distinction matters: `0xFF` is also the "no
calibration" sentinel, so an unpopulated calibration block and an absent one
look the same.

Most of it is a field-by-field copy of the configuration bytes, at different
offsets. That mapping is tabulated in
[SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md)
Appendix A and is not repeated here. What follows is the header's own layout.

### 3.1 Offset map

| Offset | Constant | Size | Content |
|---:|---|---:|---|
| 0-1 | `SDH_SAMPLE_RATE_0/1` | 2 | Sampling rate divider, LE |
| 2 | `SDH_BUFFER_SIZE` | 1 | |
| 3-7 | `SDH_SENSORS0..4` | 5 | Sensor enable bitmaps |
| 8-14 | `SDH_CONFIG_SETUP_BYTE0..6` | 7 | Setup bytes |
| 15 | — | 1 | Unused (`0xFF`) |
| 16-17 | `SDH_TRIAL_CONFIG0/1` | 2 | Trial config bit flags |
| 18 | `SDH_BROADCAST_INTERVAL` | 1 | Sync broadcast interval, seconds |
| 19 | `SDH_BT_COMMS_BAUD_RATE` | 1 | Stored, not applied |
| 20-21 | `SDH_EST_EXP_LEN_MSB/LSB` | 2 | Estimated length, **big-endian** |
| 22-23 | `SDH_MAX_EXP_LEN_MSB/LSB` | 2 | Max length, **big-endian** |
| 24-29 | `SDH_MAC_ADDR` | 6 | Read from the Bluetooth module |
| 30-31 | `SDH_SHIMMERVERSION_BYTE_0/1` | 2 | `DEVICE_VER`, **big-endian** |
| 32 | `SDH_MYTRIAL_ID` | 1 | |
| 33 | `SDH_NSHIMMER` | 1 | |
| 34-35 | `SDH_FW_VERSION_TYPE_0/1` | 2 | `FW_IDENTIFIER`, big-endian |
| 36-37 | `SDH_FW_VERSION_MAJOR_0/1` | 2 | Big-endian |
| 38 | `SDH_FW_VERSION_MINOR` | 1 | |
| 39 | `SDH_FW_VERSION_INTERNAL` | 1 | Patch version |
| 40-42 | `SDH_DERIVED_CHANNELS_0..2` | 3 | |
| 43 | — | 1 | Unused |
| 44-51 | `SDH_RTC_DIFF_0..7` | 8 | **Meaning differs by generation — see §3.3** |
| 52-55 | `SDH_CONFIG_TIME_0..3` | 4 | Config time, MSB order |
| 56-65 | `SDH_EXG_ADS1292R_1_*` | 10 | ExG chip 1 register image |
| 66-75 | `SDH_EXG_ADS1292R_2_*` | 10 | ExG chip 2 register image |
| 76-96 | `SDH_WR_ACCEL_CALIBRATION` | 21 | Wide-range accel calibration |
| 97-117 | `SDH_GYRO_CALIBRATION` | 21 | Gyroscope calibration |
| 118-138 | `SDH_MAG_CALIBRATION` | 21 | Magnetometer calibration |
| 139-159 | `SDH_LN_ACCEL_CALIBRATION` | 21 | Low-noise accel calibration |
| 160-181 | `SDH_TEMP_PRES_CALIBRATION` | 22 | Pressure coefficients |
| 182-189 | `SDH_WR_ACCEL_CALIB_TS` | 8 | Calibration timestamp |
| 190-197 | `SDH_GYRO_CALIB_TS` | 8 | |
| 198-205 | `SDH_MAG_CALIB_TS` | 8 | |
| 206-213 | `SDH_LN_ACCEL_CALIB_TS` | 8 | |
| 214-216 | `SDH_DAUGHTER_CARD_ID_BYTE0` | 3 | Expansion board ID |
| 217-221 | `SDH_DERIVED_CHANNELS_3..7` | 5 | |
| 222-223 | `SDH_TEMP_PRES_EXTRA_CALIB_BYTES` | 2 | BMP280 only |
| 251 | `SDH_INITIAL_TIMESTAMP_4` | 1 | **Most significant of the 5** |
| 252-255 | `SDH_INITIAL_TIMESTAMP_0..3` | 4 | Lower 4 bytes, LSB order |
| **256-282** | `SDH_ALT_ACCEL_CALIBRATION` | 21 | S3R only |
| **277-284** | `SDH_ALT_ACCEL_CALIB_TS` | 8 | S3R only |
| **285-305** | `SDH_ALT_MAG_CALIBRATION` | 21 | S3R only |
| **306-313** | `SDH_ALT_MAG_CALIB_TS` | 8 | S3R only |
| **314** | `SDH_NUM_ENABLED_CHANNELS` | 1 | S3R only |
| **315-364** | `SDH_CHANNEL_ID_BYTE_0` | up to 50 | S3R only — the resolved channel order |

Everything at offset 256 and above exists only in the 384-byte Shimmer3R
header.

### 3.2 The initial-timestamp field is not contiguous

The five bytes are laid out as:

| Offset | Constant | Byte of the value |
|---:|---|---|
| 251 | `SDH_INITIAL_TIMESTAMP_4` | bits 32-39 |
| 252 | `SDH_INITIAL_TIMESTAMP_0` | bits 0-7 (least significant) |
| 253 | `SDH_INITIAL_TIMESTAMP_1` | bits 8-15 |
| 254 | `SDH_INITIAL_TIMESTAMP_2` | bits 16-23 |
| 255 | `SDH_INITIAL_TIMESTAMP_3` | bits 24-31 |

The most significant of the five sits **before** the other four, not after.
Reading 251-255 as a straight little-endian 40-bit integer gives the wrong
answer. The constants are named in value order, not offset order — take the
offsets from this table.

### 3.3 `SDH_RTC_DIFF` means different things on the two platforms

This field is the single largest gotcha in the header.

**Shimmer3.** The MSP430 RTC runs as a free-running counter with no way to set
it, so the firmware keeps a 64-bit offset between that counter and real-world
time. `ShimSdHead_config2SdHead` copies `RTC_getRwcTimeDiffPtr()` into all
eight bytes, MSB order. Absolute time for a sample is
`initialTimestamp + rtcDiff + elapsedTicks`, converted at 32768 Hz.

**Shimmer3R.** The STM32 RTC can be set directly, so there is no separate
offset. The field is reused to carry the **top three bytes of the 64-bit
initial timestamp**, which does not fit in the five-byte
`SDH_INITIAL_TIMESTAMP_*` field:

| Offset | Constant | Content on S3R |
|---:|---|---|
| 44 | `SDH_RTC_DIFF_0` | Initial timestamp bits 56-63 |
| 45 | `SDH_RTC_DIFF_1` | Initial timestamp bits 48-55 |
| 46 | `SDH_RTC_DIFF_2` | Initial timestamp bits 40-47 |
| 47-51 | `SDH_RTC_DIFF_3..7` | Zero |

So on Shimmer3R the complete 64-bit initial timestamp is assembled from
**eight bytes across two non-adjacent fields**:

```
initialTs = (b[44] << 56) | (b[45] << 48) | (b[46] << 40)
          | (b[251] << 32)
          | (b[255] << 24) | (b[254] << 16) | (b[253] << 8) | b[252]
```

> **`ShimSdHead_config2SdHead` writes zeros into all eight RTC-diff bytes on
> Shimmer3R, and `ShimSdDataFile_writeSdHeaderToFile` then overwrites the top
> three at file-creation time.** Only the second of those runs when a data file
> is opened, so a written file does carry the timestamp. But the in-memory
> header between those two calls has zeros there, and anything that samples it
> in between — or any code path that writes the header without going through
> the data-file writer — will produce a file whose absolute time is 512-second
> ambiguous.

### 3.4 Calibration in the header

Four 21-byte kinematic blocks on both platforms, plus two more on Shimmer3R,
each with an 8-byte calibration timestamp except the pressure block.

> **The header's calibration order is not the InfoMem order.** The header runs
> wide-range accel, gyro, mag, low-noise accel; InfoMem runs low-noise accel,
> gyro, mag, wide-range accel. A parser that assumes they are parallel will
> swap the two accelerometers. Block layout and interpretation are in
> [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) §3 and §4.2.

Pressure coefficients are written by
`ShimSdHead_saveBmpCalibrationToSdHeader`:

| Part | Bytes | Where |
|---|---:|---|
| BMP180 | 22 | 160-181 |
| BMP280 | 24 | 160-181 plus 222-223 — it does not fit the 22-byte field |
| BMP390 | 21 | 160-180 |
| BMP581 | 0 | **Nothing written — the field stays `0xFF`** |

A BMP581 device therefore has an all-`0xFF` pressure calibration block, which
is correct: that part outputs pre-compensated data. A parser must not treat
that as a missing-calibration error.

### 3.5 Channel order on Shimmer3R

`SDH_NUM_ENABLED_CHANNELS` (314) and the list beginning at
`SDH_CHANNEL_ID_BYTE_0` (315) record the resolved channel order — the same
list the `INQUIRY_RESPONSE` reports, copied from `sensing.cc`.

**This makes a Shimmer3R data file self-describing.** A parser can slice the
sample records using the header alone.

> **Shimmer3 files are not self-describing.** Those offsets do not exist in the
> 256-byte header, so channel order must be reconstructed from the sensor
> enable bitmaps at offsets 3-7 plus the fixed per-bus ordering rules in
> [SHIMMER3_STREAMING_DATA_FORMAT.md](SHIMMER3_STREAMING_DATA_FORMAT.md) §4.1 —
> including the LSM303DLHC X/Z/Y magnetometer order, which depends on the
> board and is not recorded anywhere in the file.

## 4. Writing behaviour

### 4.1 Buffering, sync and rollover

| Constant | Value | Meaning |
|---|---:|---|
| `SD_WRITE_BUF_SIZE` | 512 | One write buffer |
| `NUM_SDWRBUF` | S3: 1, S3R: 4 | Number of buffers |
| `BIN_FILE_SYNC_TIME_TICKS` | 32768 × 60 | `f_sync` every 60 s |
| `BIN_FILE_SPLIT_TIME_TICKS` | 32768 × 3600 | New file every 3600 s |

Samples accumulate into a 512-byte buffer; when full it is written with
`f_write`. Shimmer3R rotates four buffers so sensing can continue during a
write, Shimmer3 has one.

Every 60 seconds of sample time the file is `f_sync`ed, bounding data loss on
power failure to about a minute. Every 3600 seconds a new file is opened.

> **Both intervals are measured in sample ticks, not wall time.** They use
> `sensing.latestTs`, so a file spans an hour *of sampling*. This matters when
> comparing against wall-clock expectations for a paused or restarted trial.

> **File splits are not aligned to any wall-clock boundary.** A file starts when
> logging starts and ends 3600 seconds of sample time later. Do not assume file
> boundaries fall on the hour.

### 4.2 Timestamps on the files themselves

`ShimSd_setFileTimestamp` sets the FAT modification timestamp on the experiment
directory, the session directory, and each data file as it is closed. This is
derived from the real-world clock, so if the clock has not been set the FAT
timestamps are meaningless while the in-file timestamps remain valid.

### 4.3 Ordering at file creation

`ShimSdDataFile_fileInit` runs: power the card, resolve the base directory,
create the session directory, **build the header from configuration**, open the
file, capture `sdFileCrTs = sdFileSyncTs = RTC_get64()`, then write the header.

The initial timestamp in the header is therefore captured *before* the header
is written but *after* the configuration snapshot — the header describes the
configuration in force at file creation, which cannot change while logging.

## 5. `sdlog.cfg`

A plain-text file at the card root that lets a device be configured by editing
a file rather than over Bluetooth.

### 5.1 Format

```
key=value\r\n
```

One setting per line, `\r\n` line endings, written with `"%s=%d\r\n"` for
integers and `"%s=%s\r\n"` for strings. No sections, no comments, no quoting,
no escaping.

Parsing matches a key only when the line **begins** with the key and the very
next character is `=` (`cfg_key_is`), and values are read with `atoi` for
integers. An unparseable value therefore becomes 0 rather than an error, and a
line with no `=` is skipped.

### 5.2 Direction of travel

| Function | Direction | When |
|---|---|---|
| `ShimSdCfgFile_generate` | configuration bytes → file | When the configuration has changed and the file needs refreshing |
| `ShimSdCfgFile_parse` | file → configuration bytes | On `ShimSdCfgFile_readSdConfiguration` |

`ShimSdCfgFile_generate` only runs when the device is **not docked**, a card is
present, and the card is not marked bad — and only when
`ShimConfig_areConfigBytesValid()` passes.

`ShimSdCfgFile_parse` starts by calling `ShimConfig_createBlankConfigBytes()`,
so **the file is authoritative, not a patch**. Any setting absent from the file
takes the blank-configuration value, not the value the device previously had.
After parsing, `ShimConfig_checkBtModeFromConfig()` reconciles the Bluetooth
state.

> **A missing key silently resets that setting.** Hand-editing the file down to
> just the lines you care about will clear everything else. Edit values in
> place; do not delete lines.

If the file does not exist, parsing returns without touching the configuration.

### 5.3 Keys

Sensor enables:

`accel`, `gyro`, `mag`, `exg1_24bit`, `exg2_24bit`, `exg1_16bit`,
`exg2_16bit`, `gsr`, `br_amp`, `str`, `vbat`, `accel_d`, `accel_alt`,
`mag_alt`, `accel_mpu`, `mag_mpu`, `pres`

ADC channels — note these differ by generation:

| Shimmer3 | Shimmer3R |
|---|---|
| `extch7`, `extch6`, `extch15` | `extch0`, `extch1`, `extch2` |
| `intch1`, `intch12`, `intch13`, `intch14` | `intch0`, `intch1`, `intch2`, `intch3` |

Rates and ranges:

`sample_rate`, `mg_internal_rate`, `mg_range`, `mag_alt_range`,
`mag_alt_rate`, `acc_internal_rate`, `acc_range`, `acc_lpm`, `acc_hrm`,
`accel_alt_range`, `accel_alt_rate`, `accel_ln_range`, `gyro_range`,
`gyro_samplingrate`, `gsr_range`, `exp_power`

Pressure precision — one key per part, so the applicable key depends on the
fitted sensor:

`pres_bmp180_prec`, `pres_bmp280_prec`, `pres_bmp390_prec`,
`pres_bmp581_prec`

Trial and sync:

`user_button_enable`, `rtc_error_enable`, `sd_error_enable`, `iammaster`,
`sync`, `singletouch`, `low_battery_autostop`, `interval`,
`bluetoothDisabled`, `max_exp_len`, `est_exp_len`, `node`, `center`,
`myid`, `Nshimmer`, `shimmername`, `experimentid`, `configtime`,
`derived_channels`

ExG registers, twenty keys, named after the register:

`EXG_ADS1292R_1_CONFIG1` … `EXG_ADS1292R_1_RESP2`,
`EXG_ADS1292R_2_CONFIG1` … `EXG_ADS1292R_2_RESP2`

> **Key naming is inconsistent.** Most keys are lower-case with underscores,
> but `Nshimmer` is capitalised, `bluetoothDisabled` is camel-case, and the
> twenty ExG keys are upper-case. Matching is exact — `nshimmer` will not be
> recognised.

> **`node` appears once per node**, not as a list; the parser accumulates
> repeated `node=` lines into the sync node table. `center` sets the sync
> centre address.

## 6. The calibration file

```
/Calibration/calib_<last 4 characters of the Bluetooth MAC address>
```

A byte-for-byte image of the in-RAM calibration dump blob, `length + 2` bytes
long, written in 128-byte chunks with no additional framing. Format and
semantics are in [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) §2 and
§4.3.

It is neither read nor written automatically as part of logging — only by
explicit `ShimCalib_ram2File` / `ShimCalib_file2Ram` calls.

## 7. Card handling

`ShimSd_mount` manages mounting. `ShimSd_test1` and `ShimSd_test2` are the
card self-tests used by the factory-test path — see
[SHIMMER3R_FACTORY_TEST_REPORT.md](SHIMMER3R_FACTORY_TEST_REPORT.md).
`ShimSd_findError` maps a `FRESULT` to the error signalled through the LEDs
when `sdErrorEnable` is set — see
[SHIMMER3_LED_FEEDBACK.md](SHIMMER3_LED_FEEDBACK.md).

While the device is docked, or USB is connected on Shimmer3R, the card may be
presented to the host as mass storage instead of being owned by the firmware.
Logging cannot start in that state, and the firmware tears the USB stack down
before taking the card for logging.

## Still unverified / not found in code

- ~~`BMPX80_PACKET_SIZE` on Shimmer3~~ — resolved: `BMPX80_TEMP_BUFF_SIZE`
  (`0x02`) + `BMPX80_PRESS_BUFF_SIZE` (`0x03`) = 5, matching the 2-byte
  temperature and 3-byte pressure widths.
- **Header offset 15 and offset 43.** Both fall between populated fields and
  are never written, so they remain `0xFF`. Whether they were once used is not
  determinable from the current source.
- ~~Behaviour past data file `999`~~ — resolved from the types: `fileNum` is a
  `uint16_t`, formatted with `"/%03d"` into a 7-byte buffer, so the name
  simply widens — `/1000` … `/65535` (six characters plus NUL exactly fills the
  buffer). Nothing caps it; the only wrap is the integer's, at 65536 files.
- ~~`sdlog.cfg` maximum line length~~ — resolved: the parser reads with
  `f_gets(buffer, 64, …)` into a 66-byte local, so a line longer than **63
  characters** is delivered in two pieces; the tail is parsed as a line of its
  own, matches no key and is ignored. Values are therefore truncated, not
  rejected.
- ~~Whether `ShimSdCfgFile_generate` is triggered on every configuration
  change~~ — resolved: five call sites, and none is "on every change". The
  flag is set whenever a host writes configuration (`ShimConfig_setFlagWriteCfgToSd(1, …)`)
  and the file is rewritten later, when the card is usable: (1) on undock
  (`shimmer_config.c`) if the flag is set; (2) `LogAndStream_syncConfigAndCalibOnSd`
  if the flag is set; (3) the `TASK` path (`shimmer_taskList.c`) when undocked,
  not sensing, card present and flag set; (4) on reading the card if
  `sdlog.cfg` is missing; (5) after parsing, if
  `ShimConfig_checkAndCorrectConfig` altered anything. Cases 4 and 5 ignore
  the flag.
