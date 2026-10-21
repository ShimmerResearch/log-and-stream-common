# Hardware-in-the-loop tests and the release gate

Tests that run against real Shimmer units on a bench, and the gate that turns
their results into the decision to publish a release. The rules and the report
format are in [docs/SHIMMER3_TEST_PROCEDURE.md](../../docs/SHIMMER3_TEST_PROCEDURE.md) §10.

## Testing a release candidate

```
python Test/hil/run_rc.py LogAndStream_Shimmer3R_v1.01.022            # flash, run smoke, write the report
python Test/hil/run_rc.py LogAndStream_Shimmer3R_v1.01.022 --upload   # ...and attach it to the pre-release
python Test/hil/run_rc.py <tag> --no-flash                           # the unit already runs the candidate
python Test/hil/run_rc.py <tag> --waive smoke.sd_log=ABC-123        # accept a known failure under an issue
python Test/hil/run_rc.py <tag> --manual-power                       # dock and undock by hand when prompted
```

An overnight soak of a Shimmer3R candidate, then of a Shimmer3 one (each run tests its own platform's units):

```
set HIL_SOAK_SCALE=3
python Test/hil/run_rc.py LogAndStream_Shimmer3R_v1.01.022 --no-flash --tier soak
python Test/hil/run_rc.py LogAndStream_Shimmer3_v1.01.007 --no-flash --tier soak
```

A failure in the report blocks the release, required item or not, unless it
carries a waiver. `--waive ITEM=ISSUE` puts the issue key into that item's
result when it fails or is not covered, as a manual sign-off's waiver does, and
the gate lists it as waived. A waiver for an item that passed is not used.

Output goes to `Test/hil/out/<tag>/<time>/`: the image, `build-info.json`, the
pytest log and `hil-report-<bench>.json`.

Without the runner:

```
python -m pytest Test/hil/tests -m smoke --candidate build-info.json --hil-report hil-report.json
```

A run without `--candidate` still tests, but writes no report: a report that
cannot name the build it tested is not evidence about any build.

For the items no rig covers yet, a person records a sign-off with
`hil_signoff.py --tag <tag> --upload`.

When something fails, `--triage` (or `python Test/hil/triage.py <run dir>`)
asks Claude for a classification of each failing item and writes `triage.md`
beside the report. `--dry-run` writes the prompt and sends nothing. It needs
the Claude Code CLI signed in. It is found on PATH, inside the Claude desktop
app, or at `[triage] claude` in `rig.toml`. Triage is advice; the gate does not
read it.

`[triage]` in `rig.toml` names the bench's known-issues file, its bench facts,
the firmware checkouts the model may read, and a spending cap
(`budget_usd`, default 2).

## Setting up a bench

1. `pip install pytest pyserial tomli bleak` (Python 3.10+; `tomli` only before 3.11; `bleak` for
   the Shimmer3R's BLE items, which are `not_covered` without it).
2. Copy `rig.example.toml` to `rig.toml` (git-ignored), or keep it anywhere and
   set `HIL_RIG`. Describe each unit by MAC and USB serial, never by COM number.
3. Pair each unit's Bluetooth once in Windows. Current LogAndStream asks for
   no PIN; only a unit with SD sync enabled uses the legacy "1234". The
   classic items need the **classic** side, which creates a "Standard Serial
   over Bluetooth link" COM port. A dual-mode unit (a Shimmer3 with an RN4678,
   or a Shimmer3R) is listed twice under the same name; the BLE entry is the
   one shown with a **PC** icon. Pairing only that one creates GATT services
   and no COM port, and the classic items then report not_covered.
   `python -m serial.tools.list_ports -v` should show a BTHENUM port whose
   hardware ID contains the unit's MAC. `smoke.ble_throughput` needs no
   pairing at all: it connects over BLE by the unit's MAC, through bleak, and
   needs the unit not connected to any other host.
4. Units configured for SD sync, or with Bluetooth disabled, keep their radio
   off. The run handles that itself (`rig/btmode.py`): before the first test,
   each unit whose tests need Bluetooth has those two bits of InfoMem byte 217
   cleared and is reset, and after the last its InfoMem is written back byte
   for byte and it is reset again. The read-back must match everywhere but
   byte 230 bit 0, `flagWriteCfgToSd`, which the firmware sets itself on
   every configuration write; that bit differing is expected, not a failed
   restore. A Shimmer3 is reset through its Base6 slot;
   a Shimmer3R through its ST-Link if `rig.toml` names one, otherwise through
   the ST bootloader. A unit that cannot be put back fails the run.
5. Close STM32CubeIDE and Consensys before a release run. The suite warns if
   they are open: the IDE pages every Bluetooth COM port when USB devices
   arrive, which distorts Bluetooth timing for most of a minute.

What the bench lacks is not a failure. Each test declares what it needs
(`@pytest.mark.hil("smoke.sd_log", needs=[...])`); on a bench without, say, the
managed hub, the item is reported `not_covered` and another bench's report has
to cover it. `--manual-power` makes the operator the dock: the suite prompts to
plug and unplug.

## Layout

| Path | What |
|---|---|
| `hil_gate.py`, `required/` | The release gate and the items it requires |
| `hil_signoff.py` | Manual sign-off, written as a report |
| `hil_report.py`, `conftest.py` | Turns pytest outcomes into a report; capability checks |
| `rig/config.py` | `rig.toml` |
| `rig/power.py` | Hub port power (through bench-tools `hub/hubctl.py`), or the operator |
| `rig/dock.py` | The dock protocol over the Shimmer3R USB CDC port |
| `rig/bt.py` | Bluetooth SPP: commands, status, streaming with timestamp checks, and commands sent mid-stream |
| `rig/midstream.py` | SD logs started and stopped inside a stream: the command schedule, and the stamps judged across each start (steps back, grid moves) |
| `rig/throughput.py` | Bluetooth throughput: the firmware's data-rate test, and a stream that outruns the link, against each unit's baseline |
| `rig/s3r_usb.py` | Finds a Shimmer3R's CDC port and SD drive by USB serial |
| `rig/usb_throughput.py` | The SD card's write and read rates over USB mass storage, timed around the PC's file cache |
| `rig/sdfile.py` | Reads a Shimmer3R SD data file and checks its records |
| `rig/flash.py` | Flashes a Shimmer3R by USB DFU or SWD, then confirms the version. USB DFU flashes only a DFU device that appeared after this unit was sent to its bootloader, and only if it is the only one |
| `rig/base6.py` | Base6 slot mux: which slots are occupied, select one, mask its BSL lines |
| `rig/msp430_bsl.py` | MSP430 ROM bootloader client (port of Consensys's): erase, write, CRC-verify |
| `rig/s3_flash.py` | Flashes a Shimmer3 in a Base6 slot, backing up and restoring its InfoMem |
| `rig/s3r_config.py` | Shimmer3R InfoMem fields the matrix varies: rate, sensors, ranges, and the ODRs to check |
| `rig/s3r_boards.py` | Which optional sensors a Shimmer3R board has fitted (ported from Boards/shimmer_boards.c) |
| `rig/decode.py` | Packets or SD records to per-channel arrays; calibration from a blob, an SD header or InfoMem |
| `rig/signals.py` | Checks on a resting unit: 1 g, gyro ≈ 0, stuck/staircase/saturated axes, VBatt |
| `matrix/s3r_release.toml` | The Shimmer3R release matrix: one row per configuration |
| `tests/` | The tests, marked `smoke` (Shimmer3R), `fleet` (Shimmer3), `release` (the matrix) or `soak` (long-running, advisory; `HIL_SOAK_SCALE` sets the length) |
| `triage.py` | Claude triage of a failed run: advice in `triage.md`, never a verdict |
| `test_hil_*.py` | Unit tests for the above that need no hardware; run by `make -C Test/host` |

## Throughput baselines

`fleet.bt_throughput` and `smoke.bt_throughput` measure two figures for each
unit:

- `link_bytes_per_s`: what the firmware's own data-rate test gets through the
  link;
- `stream_samples_per_s`: how many samples arrive per second of a 1024 Hz
  stream with LN accel, gyro, mag, WR accel and VBatt. That load outruns every
  shipped Shimmer3 radio.

Each unit is judged against figures for the same bench, recorded in
`rig.toml`. Once a bench has baselines, keep its `rig.toml` somewhere
versioned and point `HIL_RIG` at it, so every checkout and colleague reads
the same figures. Ours is `hil/rig-<bench>.toml` in the private bench-tools
repository, beside its known-issues file.

A Shimmer3's figures are what the last release gave on that unit:

```toml
[[unit]]
name = "s3-rn42-615"
# ...
throughput = { link_bytes_per_s = 39315, stream_samples_per_s = 989.1 }
```

A Shimmer3R's link rate follows its IF820 module build, and re-flashing the
module changes that build. So its figures go in a row per build, and each
unit's `radio` names its build exactly. TEST_PROCEDURE §5.5 has the reference
figures. Take a row's KB/s × 1024, and the low end of each range:

```toml
[throughput_by_radio]
"IF820 v1.4.16.16" = { link_bytes_per_s = 156672, tolerance = 0.10 }   # 153-154 KB/s
"IF820 v1.4.18.18" = { link_bytes_per_s = 77824, tolerance = 0.10 }    # 76-77 KB/s, public release

[[unit]]
name = "s3r-a"
radio = "IF820 v1.4.16.16"
```

- **Module check:** the item checks that the module reports the version
  `radio` names. A module's test builds report their release's version, so
  give each its own key, e.g. `"IF820 v1.4.18.18 <build>"`, and keep that
  `radio` up to date by hand when a unit is re-flashed.
- **Overrides:** a unit's own `throughput` overrides its radio row key by
  key.

`smoke.ble_throughput` makes the same two measurements over the Shimmer3R's
BLE link: the IF820's CYSPP serial service, with data as notifications, as
the web SDK receives it (`rig/ble.py`). Its figures go in tables of their own,
`[ble_throughput_by_radio]` and a unit's `ble_throughput`, keyed and judged
the same way. They are kept apart because a BLE figure depends on the
connection the host negotiates as well as on the module build:

```toml
[ble_throughput_by_radio]
"IF820 v1.4.16.16" = { link_bytes_per_s = ..., stream_samples_per_s = ..., tolerance = 0.10 }
```

`smoke.usb_throughput` measures the Shimmer3R's SD card over USB mass
storage, as a customer offloads a recording: a 32 MiB file written and read
back around the PC's file cache, compared, then removed. Its two figures,
`usb_write_bytes_per_s` and `usb_read_bytes_per_s`, are the unit's own only.
The card sets them more than any build, so there is no table by build, and a
new card means recording them again:

```toml
[[unit]]
name = "s3r-a"
usb_throughput = { usb_write_bytes_per_s = ..., usb_read_bytes_per_s = ..., tolerance = 0.10 }
```

How each figure is judged:

- **Below:** a figure more than 5% below its baseline fails. A row with its
  own `tolerance` uses that instead; the Shimmer3R rows use 10%, for the
  day-to-day spread §5.5 records.
- **Above:** one more than the same tolerance above is noted, to be recorded
  once that build is released.
- **No baseline:** the unit is reported `not_covered`, and the reason carries
  the line to paste. Take it from a run of the released image, not a
  candidate.

**A shortfall is a finding, not yet a verdict.** On the RN42 v6.15 a
saturated stream's throughput moves with the timing of the firmware's data
path. Two functionally correct changes cost 10-20% this way, and each build
repeats its figure within a fraction of a percent. So confirm a shortfall
with an interleaved A/B of the release and the candidate on that unit before
acting on it. Bench conditions move the figures too:
- close STM32CubeIDE and Consensys;
- keep other Bluetooth links on the PC idle during the run.

## What a timestamp check counts

Each sample stamp is placed on the sampling grid by its distance from the
first stamp (`rig/bt.py`, `classify_step`). A stamp less than three quarters
of a period late keeps its slot, because interrupt latency makes stamps late,
never early; exactly three quarters late counts as the next slot. Stamps are then sorted three ways:

- **lost samples:** a stamp skips a slot. The grid keeps its phase through a
  stall, so a lost sample is still counted when the stamp after it is on time
  or a tick early;
- **duplicates:** a stamp lands in a slot already taken, or before it;
- **jitter:** a stamp is off its slot by more than a tick.

So a sample stamped late - a long step then a short one - is one late stamp,
not a lost sample and a duplicate. That includes one exactly half a period
late, which a bench run showed. Smoke tests fail on any loss or duplicate, and
on jitter beyond a quarter period.

The `sd_start_midstream` items (`rig/midstream.py`) look for two more things,
because a log start mid-stream can re-phase the stream:

- **steps back:** a stamp at or behind the one before it. A host that unwraps
  the counter can take one for a wrap;
- **grid moves:** a new offset from the grid that the next three stamps keep.
  One stamp off and back is jitter.

The grid is moved at each move found, before the steps are sorted as above, so
one move is reported once, not as jitter on every stamp after it.

`run_rc.py` expects every unit it flashes to be docked (on USB) when it starts,
as the tests do, and stops before flashing anything if one is not.
