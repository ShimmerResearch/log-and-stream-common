"""The Shimmer3's part of the soak tier's long log (rig/shared_log.py, soak.long_log).

A Shimmer3 logs only off the dock, and the Base6 cannot take one off: a slot
has no dock-detect line to switch (SDG drives the Shimmer's LED). So a person
lifts every Shimmer3 out at the start and puts them all back at the end, asked
once each way for the lot (rig/power.OperatorDock). The item needs "attended"
(conftest.py), so an unattended run reports it not_covered and never waits.

Leaving the dock starts each log. configure() turns the button and single
touch off, which makes an undock start logging (INFOMEM §8.1-§8.3), and turns
auto-stop off. Bluetooth then only confirms that each log is running, and stops
each after the wait, so the log lasts as long as the run says, however long
the operator takes to come back.

Each unit's card is read through the Base6's card reader (rig/s3_card.py), and
its session found by name. configure() gives every unit one new trial folder,
data/HIL_longlog_<config time>, with the config time the moment the steps were
made: the log's session is then the one new folder in it. It is copied off the
card and checked from the copy, which is kept under the run's output folder
with each unit's original InfoMem.

Units can come back in other slots, so every Base6 access here finds its unit
by MAC. The rest of the run, and the restore of each unit's own configuration
at its end (rig/btmode.py), reach a unit by its rig.toml slot. So after the
re-dock the operator is asked to move any unit that is not in its own slot,
and a unit left elsewhere fails its test.
"""

from __future__ import annotations

import time
from contextlib import contextmanager
from pathlib import Path

from . import bt, dock, s3_card, sdfile
from .power import listing
from .s3_config import TICKS_PER_S, TRIAL_CONFIG0, S3Config, divider_for, odr_codes
from .s3_flash import _masked, restore_infomem

LOG_EXPERIMENT_ID = "HIL_longlog"
UNDOCK_S = 30.0           # after the operator says they are out, for each slot to read empty
START_S = 30.0            # for a unit to report logging once it is off the dock
STOP_S = 30.0
FIND_S = 120.0            # for a unit to answer in some slot once it is back
SLOT_REQUESTS = 3         # times the operator is asked to put misplaced units in their own slots
DOCK_ATTEMPTS = 6
BT_ATTEMPTS, BT_RETRY_S = 3, 10.0
MIN_LOGGED = 0.95         # of the wait, that a session must span


class S3LogError(Exception):
    pass


def long_log_image(infomem: bytes, rate_hz: float, sensors, config_time: int) -> S3Config:
    """The unit's InfoMem with the long log's rate, sensors and their ODRs, an
    undock start, no auto-stop, and the log's own trial folder."""
    cfg = S3Config(infomem)
    cfg.divider = divider_for(rate_hz)
    cfg.set_sensors(set(sensors))
    for f, v in odr_codes(rate_hz).items():
        cfg.put(f, v)
    cfg.set_undock_start()
    cfg.clear_auto_stop()
    cfg.experiment_id = LOG_EXPERIMENT_ID
    cfg.config_time = config_time
    return cfg


def session_problems(sc: sdfile.SessionCheck, rate_hz: float, sensors, seconds: float) -> list[str]:
    """What a Shimmer3 session says beyond its records: whether its header has the
    log's configuration, and whether it spans the whole wait. A Shimmer3 header
    records no channel list, so a wrong sensor set would otherwise pass as a
    record layout of another length."""
    h = sc.header
    if h is None:
        return ["the session has no data file"]
    problems = []
    if h.divider != divider_for(rate_hz):
        problems.append(f"logged at divider {h.divider}, configured {divider_for(rate_hz)}")
    if h.sensors != frozenset(sensors):
        problems.append(f"logged {sorted(h.sensors or ())}, configured {sorted(sensors)}")
    logged_s = sc.span_ticks / TICKS_PER_S
    if logged_s < seconds * MIN_LOGGED:
        problems.append(f"the session spans {logged_s / 60:.1f} min of the {seconds / 60:.1f} min log")
    return problems


class S3LogSteps:
    """rig/shared_log.py's steps for Shimmer3s in the Base6: configured and restored
    over the dock, taken off it by the operator, confirmed and stopped over
    Bluetooth, and read through the Base6's card reader.

    `ask(text)` puts a request to the operator (conftest's hil_ask). Give
    OperatorDock `lambda: steps.ready` as its group: the units configured whose
    card could be read, which are the ones shared_log takes off the dock.
    """

    def __init__(self, base6, ask, out_dir: Path, rate_hz: float, sensors, *,
                 now=time.time, clock=time.monotonic, sleep=time.sleep, say=lambda m: None):
        self.base6, self.ask, self.out_dir = base6, ask, Path(out_dir)
        self.rate_hz, self.sensors = rate_hz, set(sensors)
        self.clock, self.sleep, self.say = clock, sleep, say
        self.config_time = int(now())       # one trial folder for every unit, as a shared trial has
        self.configured: list = []          # units configure() changed, in order
        self.ready: list = []               # and whose card was read before the log: the ones taken out
        self.ports: dict[str, str] = {}
        self.slots: dict[str, int] = {}     # unit name -> the slot it was last found in
        self.trials: dict[str, str] = {}
        self.backups: dict[str, Path] = {}
        self.before: dict[str, set] = {}    # unit name -> its sessions in the trial folder before the log
        self.misplaced: dict[str, int | None] = {}
        self.moved: set[str] = set()        # units the operator was asked to move between slots
        self._settled = False

    # --- finding a unit -------------------------------------------------------

    @contextmanager
    def _dock(self, slot: int, attempts: int = 10):
        self.base6.select(slot)
        with dock.open_when_ready(self.base6.uart_port, attempts=attempts) as d:
            yield d

    def _mac_in(self, slot: int) -> str | None:
        try:
            with self._dock(slot, DOCK_ATTEMPTS) as d:
                return d.mac().upper()
        except (dock.DockError, OSError):
            return None                     # a unit that does not answer cannot be named

    def _scan(self) -> dict[str, int]:
        """{MAC: slot} for every occupied slot whose unit answers on the dock."""
        found = {}
        for slot in self.base6.occupied():
            mac = self._mac_in(slot)
            if mac:
                found[mac] = slot
        for u in self.configured:
            if u.mac in found:
                self.slots[u.name] = found[u.mac]
        return found

    def _find(self, unit, timeout_s: float = 0.0) -> int:
        """The slot the unit is in now: where it was last seen if it answers there
        with its MAC, else wherever a scan of every occupied slot finds it."""
        slot = self.slots.get(unit.name) or unit.base6_slot
        if slot and slot in self.base6.occupied() and self._mac_in(slot) == unit.mac:
            self.slots[unit.name] = slot
            return slot
        end = self.clock() + timeout_s
        while True:
            found = self._scan()
            if unit.mac in found:
                self.slots[unit.name] = found[unit.mac]
                return found[unit.mac]
            if self.clock() >= end:
                raise S3LogError(f"{unit.mac} answers in no Base6 slot (units answer in slots "
                                 f"{sorted(found.values()) or 'none'})")
            self.sleep(5.0)

    def _scan_for_all(self, timeout_s: float) -> dict[str, int]:
        """_scan(), again every few seconds for up to timeout_s while a unit of the
        log answers in no slot: one just put back may still be booting."""
        end = self.clock() + timeout_s
        while True:
            found = self._scan()
            if all(u.mac in found for u in self.configured) or self.clock() >= end:
                return found
            self.sleep(5.0)

    def _settle_slots(self) -> None:
        """After the re-dock: every unit of the log in its own rig.toml slot, the
        operator asked to move any that is not, up to SLOT_REQUESTS times. A unit
        still elsewhere is noted, and restore() fails it."""
        for attempt in range(SLOT_REQUESTS + 1):
            found = self._scan_for_all(FIND_S)
            wrong = {u.name: (u.base6_slot, found.get(u.mac)) for u in self.configured
                     if u.base6_slot and found.get(u.mac) != u.base6_slot}
            if not wrong or attempt == SLOT_REQUESTS:
                self.misplaced = {n: now for n, (_, now) in wrong.items()}
                return
            self.moved |= {n for n, (_, now) in wrong.items() if now}
            self.ask("Move " + listing(f"{n} to slot {own} ({f'now in slot {now}' if now else 'not found'})"
                                       for n, (own, now) in wrong.items()))

    # --- shared_log.Steps -----------------------------------------------------

    def configure(self, unit):
        port = bt.spp_port(unit.mac)
        if not port:
            raise LookupError(f"no paired SPP port for {unit.mac}")
        self.ports[unit.name] = port
        slot = self._find(unit)
        with self._dock(slot) as d:
            original = bytes(d.read_infomem())
            if len(original) != dock.INFOMEM_SIZE:
                raise S3LogError(f"InfoMem read gave {len(original)} bytes, expected {dock.INFOMEM_SIZE}")
            if S3Config(original).radio_blocked:
                raise S3LogError(f"byte {TRIAL_CONFIG0} is {original[TRIAL_CONFIG0]:#04x}: SD sync or "
                                 "bluetoothDisable keeps the radio off, so the log could not be stopped "
                                 "(the run clears both for a Bluetooth test: rig/btmode.py)")
            self.out_dir.mkdir(parents=True, exist_ok=True)
            backup = self.out_dir / f"infomem_{unit.name}.bin"
            backup.write_bytes(original)
            want = long_log_image(original, self.rate_hz, self.sensors, self.config_time)
            try:
                d.write_infomem(want.bytes())
                back = bytes(d.read_infomem())
                diff = [i for i, (a, b) in enumerate(zip(_masked(want.bytes()), _masked(back))) if a != b]
                if len(back) != len(original) or diff:
                    # The firmware corrected something, the undock start's bits
                    # say (s3_flash.FIRMWARE_CORRECTED_BITS): the log would not
                    # be the one configured.
                    raise S3LogError(f"the log's configuration read back different at offsets {diff[:20]}")
            except BaseException as e:
                # shared_log restores only what configure() returned, so put the
                # unit's own configuration back here.
                try:
                    d.write_infomem(original)
                except Exception as e2:
                    raise S3LogError(f"{e}; its own configuration could not be put back ({e2}): "
                                     f"it is in {backup}") from e
                raise
        self.trials[unit.name] = want.trial_dir
        self.backups[unit.name] = backup
        self.configured.append(unit)
        return original

    def sessions(self, unit, timeout_s):
        """Its sessions in the log's trial folder. Before the log that proves the
        card can be read; after it, the new one is copied off the card, and the
        copy's root is returned with them."""
        after = unit.name in self.before
        if after and not self._settled:
            self._settled = True
            self._settle_slots()
        slot = self._find(unit, FIND_S if after else 0.0)
        trial = self.trials[unit.name]
        dest = self.out_dir / unit.name
        with s3_card.on_pc(self.base6, slot, mount_s=timeout_s) as card:
            names = s3_card.with_retries(card, lambda root: sdfile.trial_sessions(root, trial))
            if not after:
                self.before[unit.name] = names
                self.ready.append(unit)
                return dest, names
            new = sorted(names - self.before[unit.name])
            if not new:
                trials = s3_card.with_retries(card, _newest_trials)
                raise S3LogError(f"no session in data/{trial} on the card; its newest trial folders: {trials}")
            for rel in new:
                s3_card.with_retries(card, lambda root, rel=rel: s3_card.copy_session(root, rel, dest))
        self.say(f"{unit.name}: {', '.join(p.as_posix() for p in new)} copied off the card to {dest}")
        return dest, names

    def undocked(self, unit):
        slot = self.slots[unit.name]
        end = self.clock() + UNDOCK_S
        while slot in self.base6.occupied():
            if self.clock() >= end:
                raise S3LogError(f"still in slot {slot} {UNDOCK_S:.0f} s after the request to take it out")
            self.sleep(1.0)

    def _bt(self, unit) -> bt.BtLink:
        last = None
        for i in range(BT_ATTEMPTS):
            try:
                return bt.BtLink(self.ports[unit.name], connect_timeout_s=20)
            except Exception as e:          # serial.SerialException or BtError
                last = e
                if i + 1 < BT_ATTEMPTS:
                    self.sleep(BT_RETRY_S)
        raise S3LogError(f"no Bluetooth connection after {BT_ATTEMPTS} attempts ({last})")

    def start(self, unit):
        """Confirm the undock started its log. Only status is asked for: a unit
        the undock did not start fails here, rather than being started another way."""
        with self._bt(unit) as link:
            link.wait_status(bt.STATUS_LOGGING, True, START_S)

    def stop(self, unit):
        with self._bt(unit) as link:
            link.simple(bt.STOP_LOGGING)
            link.wait_status(bt.STATUS_LOGGING, False, STOP_S)

    def restore(self, unit, saved):
        slot = self._find(unit, FIND_S)
        self.base6.select(slot)
        restore_infomem(self.base6, saved, None, self.backups.get(unit.name))
        if unit.name in self.misplaced:
            raise S3LogError(f"its configuration is back, but it is in slot {slot}, not its own slot "
                             f"{unit.base6_slot}, where the rest of the run looks for it: move it")

    def check(self, session):
        return sdfile.check_session(session)

    def extra_sessions_reason(self, unit) -> str | None:
        """Why the unit may have more sessions than its log's (shared_log.Steps). A
        unit moved between slots after the re-dock was undocked again, with the
        log's undock-start configuration still in it, so the move started a short
        log of its own: on the bench, an empty session (24 Nov 2026)."""
        if unit.name in self.moved:
            return "the unit was moved between slots after the re-dock, and that undock started a log"
        return None


def _newest_trials(root: Path, n: int = 5) -> list[str]:
    data = Path(root) / "data"
    return sorted(p.name for p in data.iterdir() if p.is_dir())[-n:] if data.is_dir() else []
