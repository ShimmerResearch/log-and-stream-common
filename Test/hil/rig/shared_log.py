"""Several undocked units logging at once: the soak tier's long log (soak.long_log).

A long log leaves the PC idle for nearly all of its length. Each unit logs to
its own SD card, and Bluetooth is needed only to start and stop it. So every
unit logs in the same stretch, and the run waits for the log once rather than
once per unit.

Bluetooth and USB are still used one unit at a time:
- Each log is started and stopped over a short Classic link of its own, closed
  before the next unit's is opened. The PC's adapter holds only about three
  Classic links at once (DEV-742), so none is held for the length of the log.
- Units are undocked one after another, and re-docked one after another, each
  re-dock waiting for that unit's SD card to mount before the next. A unit's
  USB re-enumeration can stall the host's services (DEV-1368), and several at
  once would make that likelier.

A failure stays with the unit it happened to. A unit whose configuration,
undock or start fails drops out and the others log on. Whatever fails, an
interrupt included, every unit taken off the dock is put back on it and every
unit's configuration is put back. log_together returns each unit's own
outcome, and the caller judges each as a test of its own, so the report still
has one result per unit.

The platform's part is passed in (Steps), so the same order and guarantees can
serve another platform, and the offline tests can drive it with no hardware.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Protocol

DOCKED_DRIVE_S = 60.0       # how long a docked unit's SD card gets to mount, before the log
REDOCK_DRIVE_S = 120.0      # and after a re-dock, which re-enumerates its USB


class Steps(Protocol):
    """One platform's part of a shared log. Each step raises on failure."""

    def configure(self, unit) -> Any:
        """Put the docked unit in the log's configuration; return what restore() needs."""

    def sessions(self, unit, timeout_s: float) -> tuple[Path, set[Path]]:
        """The docked unit's SD card root, and its session directories relative to it."""

    def undocked(self, unit) -> None:
        """Return once the unit, its dock power switched off, has left USB."""

    def start(self, unit) -> None:
        """Start the undocked unit logging, over a link closed again before returning."""

    def stop(self, unit) -> None:
        """Stop its log the same way."""

    def restore(self, unit, saved) -> None:
        """Put back the configuration configure() replaced. The unit is docked."""

    def check(self, session: Path) -> Any:
        """Read the session the log wrote; the result is what the caller's test judges."""


@dataclass
class UnitLog:
    """One unit's part of a shared log."""
    unit: Any
    errors: list[str] = field(default_factory=list)    # "step: what went wrong", in the order met
    before: set = field(default_factory=set)           # its session directories before the log
    session: Path | None = None                        # the one session the log wrote
    check: Any = None                                  # Steps.check(session)

    @property
    def ok(self) -> bool:
        return not self.errors


def log_together(units, power, seconds: float, steps: Steps, *,
                 sleep: Callable[[float], None] = time.sleep,
                 say: Callable[[str], None] = lambda m: None) -> dict[str, UnitLog]:
    """Log on every unit at once for `seconds`; return each unit's UnitLog by name.

    In order: configure each unit and note its sessions; undock each through
    `power` (rig/power.py); start each; sleep once; stop each; re-dock each and
    find the one new session it wrote; restore each unit's configuration; check
    each session. A unit drops out at its first failure, and later steps skip
    it, apart from the re-dock and the restore, which no failure skips.
    """
    logs = {u.name: UnitLog(u) for u in units}
    saved = {}          # unit name -> what restore() needs
    off = []            # units taken off the dock and not yet put back

    def fail(u, step, what):
        text = what if isinstance(what, str) else f"{type(what).__name__}: {str(what)[:200]}"
        logs[u.name].errors.append(f"{step}: {text}")
        say(f"{u.name}: {step} failed: {text}")

    def live():
        return [u for u in units if logs[u.name].ok]

    try:
        for u in units:
            try:
                saved[u.name] = steps.configure(u)
                _, logs[u.name].before = steps.sessions(u, DOCKED_DRIVE_S)
            except Exception as e:
                fail(u, "configure", e)
        try:
            for u in live():
                off.append(u)       # before the switch, so a switch that half happened is undone too
                try:
                    power.off(u.hub_port, u.name)
                    steps.undocked(u)
                except Exception as e:
                    fail(u, "undock", e)
            for u in live():
                try:
                    steps.start(u)
                except Exception as e:
                    fail(u, "start", e)
            running = live()
            if running:
                until = time.strftime("%H:%M", time.localtime(time.time() + seconds))
                say(f"logging on {', '.join(u.name for u in running)} for {seconds / 60:.1f} min, "
                    f"until about {until}")
                sleep(seconds)
            for u in running:
                try:
                    steps.stop(u)
                except Exception as e:
                    fail(u, "stop", e)
            while off:
                u = off[0]
                try:
                    power.on(u.hub_port, u.name)
                except Exception as e:
                    fail(u, "re-dock", e)
                off.pop(0)
                if not logs[u.name].ok:
                    continue
                try:
                    root, after = steps.sessions(u, REDOCK_DRIVE_S)
                except Exception as e:
                    fail(u, "re-dock", e)
                    continue
                new = sorted(after - logs[u.name].before)
                if len(new) == 1:
                    logs[u.name].session = root / new[0]     # relative: the drive letter may have changed
                else:
                    fail(u, "session", f"expected one new session, found {len(new)}")
        finally:
            # Reached with units still off the dock only when the steps above
            # were cut short. Put them back without waiting for their drives.
            for u in off:
                try:
                    power.on(u.hub_port, u.name)
                except Exception as e:
                    fail(u, "re-dock", e)
            off.clear()
    finally:
        for u in units:
            if u.name in saved:
                try:
                    steps.restore(u, saved[u.name])
                except Exception as e:
                    fail(u, "restore", e)
    for u in units:
        lg = logs[u.name]
        if lg.session is not None:
            try:
                lg.check = steps.check(lg.session)
            except Exception as e:
                fail(u, "check", e)
    return logs
