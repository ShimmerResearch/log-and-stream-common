"""Checks for open firmware defects: one report item per Jira ticket.

Each test reproduces one filed defect and fails while the defect is there.
They are marked `defect`, which no release tier selects (run_rc.py runs smoke,
fleet, release and soak), so a known failure here never blocks a release.
Once a fix lands the check passes on the fixed build. That is the time to move
it into its platform's tier, and into required/ if it should gate releases, in
the same PR as the fix.

    python -m pytest Test/hil/tests/test_defects.py -m defect -rA
    python -m pytest Test/hil/tests/test_defects.py -m defect -k dev1327 --unit s3-rn42-615

What a check changes on a unit it puts back before the next one starts: the
whole InfoMem over the dock, and the calibration dump over Bluetooth, from
copies taken first and also kept on disk (the `restored` fixture). The
read-only checks touch neither.

The Shimmer3s sit in the Base6, so nothing here logs to SD: a docked Shimmer3
refuses a log start, and a Base6 cannot undock one slot.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import pytest

from rig import bt, dock, throughput
from rig.s3_config import S3Config, calib_records
from rig.s3_flash import reset_slot, restore_infomem

pytestmark = pytest.mark.defect

RESET_SETTLE_S = 4.0            # radio bring-up after a slot reset, as rig/btmode.py waits


# --- shared ------------------------------------------------------------------

def _open_bt(unit, base6=None, attempts: int = 2, wait_s: float = 10.0) -> bt.BtLink:
    """Connect as test_s3_release._open_bt does: retry what is the PC's, and
    reset through the Base6 a unit that takes the link but answers nothing
    (DEV-1136), so the checks after it still have a unit to talk to."""
    port = bt.spp_port(unit.mac)
    if not port:
        pytest.skip(f"no paired SPP port for {unit.mac} - pair its classic side once in Windows")
    last, held = None, False
    for _ in range(attempts + 1):
        try:
            return bt.BtLink(port, connect_timeout_s=20)
        except Exception as e:      # serial.SerialException or BtError
            last = e
            held = "Access is denied" in str(e) or isinstance(getattr(e, "__context__", None), PermissionError)
            time.sleep(wait_s * (2 if held else 1))
    if held:
        pytest.skip(f"{port} is held open by another process ({last}) - not tested")
    if base6 is not None and isinstance(last, bt.BtError):
        reset_slot(base6, unit.base6_slot)
        time.sleep(RESET_SETTLE_S)
        pytest.fail(f"{port}: the unit took the link but answered nothing after {attempts + 1} attempts ({last}); "
                    f"slot {unit.base6_slot} reset for what follows (DEV-1136)")
    pytest.fail(f"{port}: no Bluetooth connection after {attempts + 1} attempts ({last})")


@dataclass
class Saved:
    infomem: bytes              # 512 bytes, over the dock
    dump: bytes                 # the calibration dump, over Bluetooth
    backup: Path


@pytest.fixture
def restored(unit, base6):
    """A Shimmer3's InfoMem and calibration dump, put back after the test.

    The dump first, as test_s3_release.saved does: writing it pushes its
    blocks into InfoMem (CALIBRATION §8.3), which the dock write after it then
    puts back. Twice, because a unit silent over Bluetooth is reset by the
    first try."""
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        infomem = bytes(d.read_infomem())
    backup = throughput.save_infomem(unit.name, infomem)
    with _open_bt(unit, base6) as link:
        dump = link.read_calib_dump()
    dump_backup = backup.with_name(backup.stem + "_calib_dump.bin")
    dump_backup.write_bytes(dump)
    yield Saved(infomem, dump, backup)

    problem = None
    for _ in range(2):
        problem = None
        try:
            with _open_bt(unit, base6) as link:
                if link.read_calib_dump() != dump:
                    link.write_calib_dump(dump)
                    if link.read_calib_dump() != dump:
                        problem = "read back differs"
            break
        except (Exception, pytest.skip.Exception, pytest.fail.Exception) as e:
            problem = repr(e)                     # InfoMem still goes back
    base6.select(unit.base6_slot)
    restore_infomem(base6, infomem, None, backup)
    if problem:
        pytest.fail(f"{unit.name}: calibration dump not restored ({problem}); the original is in {dump_backup}")


def _configure_over_dock(unit, base6, change) -> S3Config:
    """Apply change(cfg) to the unit's configuration bytes 0-9 (rate, sensors,
    setup bytes), write just those over the dock and return what the unit
    stored. A dock write leaves the calibration dump alone; a Bluetooth
    InfoMem write would re-stamp it."""
    base6.select(unit.base6_slot)
    with dock.open_when_ready(base6.uart_port) as d:
        cfg = S3Config(d.read_infomem(0, bt.INFOMEM_CONFIG_BYTES))
        change(cfg)
        d.write_infomem(cfg.bytes()[:10], offset=0)
        return S3Config(d.read_infomem(0, bt.INFOMEM_CONFIG_BYTES))


# --- DEV-1327: the mag rate set by command and by InfoMem ----------------------

MAG_RATE_220HZ = 7                          # LSM303DLHC_MAG_220HZ: 220 Hz on an LSM303DLHC


@pytest.mark.hil("defect.dev1327_mag_rate_220hz", needs=["shimmer3", "bt", "dock"])
def test_dev1327_mag_rate_command_matches_infomem(unit, base6, restored, hil_note):
    """Mag rate code 7 stored the same way whichever route sets it.

    SET_MAG_SAMPLING_RATE clamps 7 to 6 (75 Hz) in ShimConfig_configByteMagRateSet,
    while an InfoMem write stores 7, which Consensys writes and the release
    matrix relies on above 75 Hz (rig/s3_config.py MAG_FAST). The ticket asks
    whether 220 Hz is allowed. Either answer passes this check, if both routes
    give it: 7 from both, or 6 from both.
    """
    with _open_bt(unit, base6) as link:
        link.set_setting("mag_rate", MAG_RATE_220HZ)
        by_command = link.get_setting("mag_rate")
    stored = _configure_over_dock(unit, base6, lambda c: c.put("mag_rate", MAG_RATE_220HZ))
    with _open_bt(unit, base6) as link:
        by_infomem = link.get_setting("mag_rate")
    hil_note(f"mag rate 7 by command reads {by_command}; by InfoMem reads {by_infomem} "
             f"(stored {stored.get('mag_rate')})")
    assert by_command == by_infomem, (f"mag rate code 7: SET_MAG_SAMPLING_RATE stores {by_command}, "
                                      f"an InfoMem write stores {by_infomem}")
