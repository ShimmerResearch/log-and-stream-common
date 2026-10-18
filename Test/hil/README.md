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

1. `pip install pytest pyserial tomli` (Python 3.10+; `tomli` only before 3.11).
2. Copy `rig.example.toml` to `rig.toml` (git-ignored), or keep it anywhere and
   set `HIL_RIG`. Describe each unit by MAC and USB serial, never by COM number.
3. Pair each unit's Bluetooth once in Windows. Current LogAndStream asks for
   no PIN; only a unit with SD sync enabled uses the legacy "1234". The tests
   need the **classic** side, which creates a "Standard Serial over Bluetooth
   link" COM port. An RN4678 is dual-mode and Windows lists it twice under the
   same name; the BLE entry is the one shown with a **PC** icon. Pairing that
   one creates GATT services and no COM port, and the Bluetooth tests then
   report not_covered. `python -m serial.tools.list_ports -v` should show a
   BTHENUM port whose hardware ID contains the unit's MAC.
4. Close STM32CubeIDE and Consensys before a release run. The suite warns if
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
| `rig/bt.py` | Bluetooth SPP: commands, status, streaming with timestamp checks |
| `rig/s3r_usb.py` | Finds a Shimmer3R's CDC port and SD drive by USB serial |
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
| `tests/` | The tests, marked `smoke` (Shimmer3R), `fleet` (Shimmer3), `release` (the matrix) or `soak` |
| `triage.py` | Claude triage of a failed run: advice in `triage.md`, never a verdict |
| `test_hil_*.py` | Unit tests for the above that need no hardware; run by `make -C Test/host` |

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

`run_rc.py` expects every unit it flashes to be docked (on USB) when it starts,
as the tests do, and stops before flashing anything if one is not.
