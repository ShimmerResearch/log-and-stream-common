# Shimmer3 / Shimmer3R Release and Versioning

Where the version lives, what a host sees, how host-side feature gates consume
it, and the linker requirement that has broken it before.

> **Verified against** — the revisions these claims were read from. A pinned
> commit is a citation, not a claim of currency.
>
> - **Firmware:** `log-and-stream-common` @ `f3cf73e` —
>   `Calibration/shimmer_calibration.c` (`ShimCalib_initVer`),
>   `SDCard/shimmer_sd_header.c` (`ShimSdHead_config2SdHead`),
>   `Test/shimmer_test.c` (report banner).
> - **Platform firmware:** `shimmer3-firmware` @ `2765ff4` —
>   `Shimmer_Driver/version.h` (`v1.01.003`),
>   `Shimmer_Driver/shimmer_definitions.h` (`DEVICE_VER` 3,
>   `FW_IDENTIFIER` 3); `shimmer3r-firmware` @ `a8f105e5` —
>   `Shimmer_Driver/version.h` (`v1.01.012`),
>   `Shimmer_Driver/shimmer_definitions.h` (`DEVICE_VER` 10),
>   `Core/Src/main.c` (`fw_version_struct`).

**Source references:**

| Layer | File |
|---|---|
| Version numbers | platform `Shimmer_Driver/version.h` |
| Hardware and firmware identifiers | platform `Shimmer_Driver/shimmer_definitions.h` |
| The linkable version record | `shimmer3r-firmware` `Core/Src/main.c` |
| Where the version is reported | [SHIMMER3_BT_COMMUNICATION_PROTOCOL.md](SHIMMER3_BT_COMMUNICATION_PROTOCOL.md) |
| Host-side gating | [SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §12 |

---

## 1. The version numbers

`version.h`, one per platform repository:

```c
#define FW_VERSION_MAJOR  1  //16-bit
#define FW_VERSION_MINOR  1  //8-bit
#define FW_VERSION_PATCH  12 //8-bit
#define FW_VERSION_STRING "v1.01.012"
```

| Field | Width | Notes |
|---|---:|---|
| `FW_VERSION_MAJOR` | 16 bits | |
| `FW_VERSION_MINOR` | 8 bits | |
| `FW_VERSION_PATCH` | 8 bits | Called "internal" in some interfaces |

> **`FW_VERSION_STRING` is maintained by hand alongside the three numbers.** It
> is not generated from them, so the two can disagree. Change all four together.

> **The string's zero-padding is part of the convention** — `v1.01.012`, not
> `v1.1.12`. Host code that string-matches will break on an unpadded value.

> **The two platforms version independently.** At the pinned revisions Shimmer3
> is `v1.01.003` and Shimmer3R is `v1.01.012`. The numbers are not comparable
> across generations, and the same version number on each does not imply the
> same shared-module content — see
> [SHIMMER3_ARCHITECTURE_OVERVIEW.md](SHIMMER3_ARCHITECTURE_OVERVIEW.md) §1.

## 2. Identifiers and other versioned artefacts

`shimmer_definitions.h`:

| Constant | Shimmer3 | Shimmer3R |
|---|---:|---:|
| `DEVICE_VER` | 3 | 10 |
| `FW_IDENTIFIER` | 3 | 3 |

`DEVICE_VER` identifies the hardware generation — historically 0-3 for Shimmer1
through Shimmer3. `FW_IDENTIFIER` identifies the application: **3 = LogAndStream
(BTSD)**, 2 = SDLog, 1 = BtStream.

> **`FW_IDENTIFIER` is the same on both generations** because it names the
> application, not the hardware. A host distinguishes generations by
> `DEVICE_VER`, and applications by `FW_IDENTIFIER`. It needs both.

> **The same header also defines `DEVICE_VER` 58 and `FW_IDENTIFIER` 12 — but
> under `SHIMMER4_SDK`, not `SHIMMER3R`.** That is a different platform sharing
> the source tree. Shimmer3R is 10 / 3. A reader grepping the file for
> `DEVICE_VER` sees both and should check the `#if`.

## 3. The linkable version record

```c
__attribute__((section(".version"), used))
const firmware_version_t fw_version_struct = { ... };
```

```c
typedef struct {
    uint16_t major;
    uint8_t  minor;
    uint8_t  patch;
} firmware_version_t;
```

A four-byte record in its own `.version` section, so external tooling can read
the version out of a binary without running it or parsing symbols.

> **`used` alone is not enough — the linker script must `KEEP` the section.**
> `__attribute__((used))` stops the *compiler* discarding it; with
> `--gc-sections` the *linker* will still drop a section nothing references. The
> `.version` output section needs an explicit `KEEP(*(.version))`. This has
> already removed the version record from a build once, producing a binary that
> ran correctly but whose version could not be read externally.

> **Anything that changes the linker script, the toolchain, or the optimisation
> flags should be checked against the built binary**, not just against a
> successful compile. The failure is invisible at build time.

## 4. Where the version surfaces

| Surface | Fields | Encoding |
|---|---|---|
| Bluetooth version response | major, minor, patch | See the protocol document |
| Dock `UART_PROP_VER` | as above | [SHIMMER3_DOCK_PROTOCOL.md](SHIMMER3_DOCK_PROTOCOL.md) §4.2 |
| SD file header, offsets 34-39 | `FW_IDENTIFIER`, major, minor, patch | **Big-endian** for the 16-bit fields |
| SD file header, offsets 30-31 | `DEVICE_VER` | Big-endian |
| Calibration dump header, bytes 2-9 | `DEVICE_VER`, `FW_IDENTIFIER`, major, minor, patch | **Little-endian** |
| Factory test report, first line | `FW_VERSION_STRING` | Text |

> **The SD header stores these big-endian and the calibration blob stores them
> little-endian.** The SD header code carries the comment "*little endian in fw,
> but they want big endian in sw*". Same values, opposite byte order, in two
> artefacts a host may read in the same session.

> **The calibration blob's version bytes are re-stamped by the device.** After a
> `SET_CALIB_DUMP`, `ShimCalib_ramWrite` calls `ShimCalib_initVer`, overwriting
> whatever the host uploaded. See
> [SHIMMER3_CALIBRATION.md](SHIMMER3_CALIBRATION.md) §2.1.

## 5. What bumps when

There is no rule enforced anywhere in the source. The convention the version
history reflects:

| Change | Bump |
|---|---|
| A behaviour change a host must adapt to — new opcode, changed payload, changed default | Minor |
| A fix or an addition that is transparent to existing hosts | Patch |
| A generational or architectural change | Major |

Because a host's feature gates are version comparisons (§6), **any change a
host needs to detect must be accompanied by a version bump** — otherwise the
gate cannot express it.

> **Bumping the version in `version.h` is not sufficient on its own for a
> release.** The submodule pin in the platform repository fixes which shared
> code is included, and two builds with the same `FW_VERSION_STRING` can differ
> if the pin moved between them. Tag the platform repository, not just the
> version header.

## 6. Host-side feature gating

The firmware carries no compatibility logic. Every gate is host-side, comparing
a `(hardwareVersion, firmwareIdentifier, major, minor, patch)` tuple against a
threshold, with a wildcard value that makes a field always match.

The pattern is `UtilShimmer.compareVersions` in the Java driver and its port in
the TypeScript SDK. Details and the current gates are in
[SHIMMER3_CONFIGURATION_INFOMEM.md](SHIMMER3_CONFIGURATION_INFOMEM.md) §12.

> **A host must gate on the version, not probe.** Older firmware silently
> ignores unknown opcodes rather than NACKing them, so sending a newer command
> to older firmware produces **no response at all** — indistinguishable from a
> dropped packet. The SD file-transfer commands are the current example: hosts
> are directed to gate on the firmware version before using them
> ([SHIMMER3R_SD_FILE_TRANSFER.md](SHIMMER3R_SD_FILE_TRANSFER.md)).

## 7. Release checklist

A release is two workflow runs in the platform repository, with testing in
between. The test side is
[SHIMMER3_TEST_PROCEDURE.md](SHIMMER3_TEST_PROCEDURE.md) §8 and §10.

**Before the candidate**

1. Confirm the submodule pin is the intended `log-and-stream-common` revision.
2. Confirm the other platform still builds against it if `common` changed. Every
   pull request now builds its own platform in CI (`build-check.yml`); the other
   platform needs its own pull request bumping the pin.

**Candidate — `build-release-firmware.yml`**, by workflow_dispatch:

3. It runs the host tests, bumps all four values in `version.h` together
   (`scripts/increment_version.sh`), builds the shipping configuration, commits
   the bump, tags the platform repository and this one, and publishes a GitHub
   **pre-release** with the image and `build-info.json`.
   The Jira fix version is created **unreleased**.
4. `dry_run: true` builds and uploads the image as a workflow artifact without
   committing, tagging, releasing or touching Jira. Use it to check a release
   build before cutting one.

**Test the candidate**

5. Flash the image from the pre-release, never a local build.
6. Confirm `fw_version_struct` survives into the binary (§3), and run the
   factory self-test and check that its version line matches.
7. Cover the required items and attach the reports to the pre-release
   (`Test/hil/hil_signoff.py --tag <tag> --upload` for a manual sign-off).

**Promote — `promote-release.yml`**, by workflow_dispatch with the tag:

8. The gate checks the attached reports against the tag's commit and version.
   If it passes, the workflow makes the release public, here and in this
   repository, marks the Jira fix version released, and appends the gate's
   summary to the release notes. If it is blocked, nothing changes and the job
   summary says why.

A candidate that fails is not deleted. It stays a pre-release, and the fix goes
out as the next version. That keeps every image that left the building
traceable to a tag.

## Still unverified / not found in code

- ~~Whether Shimmer3 has an equivalent of `fw_version_struct`~~ — resolved:
  yes. `main.c` defines
  `__attribute__((section(".version"), used)) const firmware_version_t fw_version_struct`,
  and `lnk_msp430f5437a.cmd` both retains it (`--retain="*(.version)"`, with a
  comment that TI's unused-section elimination would otherwise drop it) and
  places it (`.version : {} > FLASH`).
- **The `.version` section's address.** Now read on both platforms and it is
  **not fixed**: Shimmer3 allocates `.version` into `FLASH` wherever the
  linker puts it, and Shimmer3R uses `INSERT AFTER .text`. A tool must take
  the address from the `.map` file of the specific build, or search the image
  for the record; no constant offset exists to document.
- **Whether a release process document exists elsewhere.** The checklist in §7
  was originally assembled from the constraints found in the source. Its
  candidate → test → promote shape is now enforced by the workflows themselves,
  so the workflows are the authority if the two ever disagree.
- ~~Anti-rollback or version-downgrade protection~~ — resolved: **none**.
  Shimmer3R boots into ST's system-memory bootloader (see
  [SHIMMER3_BUILD_AND_PROGRAMMING.md](SHIMMER3_BUILD_AND_PROGRAMMING.md)),
  which has no version awareness, and Shimmer3's BSL likewise; the application
  adds none. Any image that programs is accepted.
