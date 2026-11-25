"""Power to a unit's USB-C, which on Shimmer3R is also dock/undock.

HubPower switches a port on the Exsys managed hub through bench-tools'
hubctl.py. ManualPower asks the operator to plug or unplug, so with
--manual-power the same test runs on a bench without a hub - slower, but it
runs. Without the flag such a bench reports the docking tests not_covered, so
an unattended run never waits on a prompt. Both have the same interface; tests
never know which they got.

OperatorDock has the same interface for a Shimmer3, which is docked by sitting
in its Base6 slot and has nothing to switch: the operator takes a whole group
out with one request and puts it back with another. Tests that use it need
"attended" (conftest.py), so they too never wait on an unattended run.

hubctl is imported from a bench-tools checkout named in rig.toml rather than
copied here: it is the one place the hub's protocol and its safety rules live
(its README), and a copy would drift.
"""

from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

from .config import RigError


def import_hubctl(bench_tools: Path):
    """Load bench-tools/hub/hubctl.py as a module, once."""
    if "hubctl" in sys.modules:
        return sys.modules["hubctl"]
    path = Path(bench_tools) / "hub" / "hubctl.py"
    if not path.exists():
        raise RigError(f"{path}: not found - check tools.bench_tools in rig.toml")
    spec = importlib.util.spec_from_file_location("hubctl", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["hubctl"] = mod
    spec.loader.exec_module(mod)
    return mod


class Power:
    """Interface. `port` is the 1-based hub port (ignored by ManualPower)."""

    def on(self, port: int | None, label: str) -> None:
        raise NotImplementedError

    def off(self, port: int | None, label: str) -> None:
        raise NotImplementedError

    def is_on(self, port: int | None) -> bool | None:
        return None

    def close(self) -> None:
        pass


class HubPower(Power):
    def __init__(self, bench_tools: Path, hub_serial: str):
        self.hubctl = import_hubctl(bench_tools)
        hubs = [h for h in self.hubctl.find_hubs() if h.serial == hub_serial]
        if not hubs:
            raise RigError(f"Exsys hub with control-chip serial {hub_serial} is not connected")
        self.location = hubs[0]
        if not self.location.com:
            raise RigError(f"hub {hub_serial} has no COM port started")
        self.hub = self.hubctl.ExsysHub(self.location.com)
        self._initial: dict[int, bool] = {}     # port -> on before this run first switched it

    def _require(self, port):
        if not port:
            raise RigError("this unit has no hub_port in rig.toml")

    def _note(self, port):
        if port not in self._initial:
            self._initial[port] = self.is_on(port)

    def on(self, port, label):
        self._require(port)
        self._note(port)
        self.hub.switch([port], True)

    def off(self, port, label):
        self._require(port)
        self._note(port)
        self.hub.switch([port], False)

    def is_on(self, port):
        self._require(port)
        return bool(self.hub.states() & (1 << (port - 1)))

    def devices_on(self, port):
        """USB device nodes currently enumerated on a hub port."""
        return self.location.devices().get(port, [])

    def close(self):
        # Put back every port this run switched to how it was before the run.
        # A unit left undocked runs its battery down and the next run starts
        # from an unknown state; a port that was already off is not ours to
        # turn on.
        try:
            if self._initial:
                states = self.hub.states()
                now_on = {p: bool(states & (1 << (p - 1))) for p in self._initial}
                back_on = sorted(p for p, was in self._initial.items() if was and not now_on[p])
                back_off = sorted(p for p, was in self._initial.items() if not was and now_on[p])
                if back_on:
                    self.hub.switch(back_on, True)
                if back_off:
                    self.hub.switch(back_off, False)
                self._initial.clear()
        finally:
            self.hub.close()


def console_ask(text: str, detail: str = "") -> None:
    """A request to the operator at a console: printed, then Enter. Under pytest
    use conftest's hil_ask instead, which speaks it and reads stdin past the
    capture."""
    input(f"\n>>> {text}{f': {detail}' if detail else ''}, then press Enter ")


class ManualPower(Power):
    """For a bench without a hub: the operator is the switch.

    `ask(text, detail)` as OperatorDock's: conftest passes its hil_ask, since a
    bare input() under pytest's capture fails at once."""

    def __init__(self, ask=console_ask):
        self.ask = ask

    def on(self, port, label):
        self.ask(f"Dock {label}", "connect its USB-C")

    def off(self, port, label):
        self.ask(f"Undock {label}", "disconnect its USB-C")


def listing(items) -> str:
    """'a', 'a and b', 'a, b and c'."""
    items = [str(i) for i in items]
    return items[0] if len(items) == 1 else f"{', '.join(items[:-1])} and {items[-1]}"


class OperatorDock(Power):
    """Units only a person can take off the dock or put back, handled as one group.

    `group()` returns the units to take out (each with a name and a base6_slot),
    asked for when the first off() comes: the units configured for the shared
    log, say, which are known only once configuring is over. The first off()
    asks the operator to take every one of them out, and the first on() after
    it to put them all back, each in its own slot; the rest of the group's
    calls find them already out, or already back. So the operator is asked
    twice, at the start and the end, rather than once per unit.

    `ask(text, detail)` puts a request to the operator and returns once it is
    done (conftest's hil_ask: `text` spoken aloud, both printed, then Enter).
    The spoken part is short, "Undock the units" or "Dock the units", and the
    printed detail names them and their slots. A unit counts as out from just
    before its request, as shared_log counts a hub port, so a request cut short
    is still followed by the one to put the units back.

    The group's units call off() and on() at the same time, from shared_log's
    threads. A call that finds its unit in a request another call made waits
    until the operator has answered it. Returning at once gave that unit only
    its own short wait for the slot to empty, counted from the request rather
    than from the answer (bench, 24 Nov 2026).
    """

    def __init__(self, ask, group):
        self.ask, self.group = ask, group
        self.out: dict[str, int | None] = {}      # unit name -> its own slot, for those taken out
        self._asking = threading.Lock()

    def off(self, port, label):
        with self._asking:
            if label in self.out:
                return
            units = {u.name: u.base6_slot for u in self.group() if u.name not in self.out}
            if label not in units:
                units = {label: None}           # not in the group: asked for on its own
            self.out.update(units)
            self.ask(f"Undock the {'units' if len(units) > 1 else 'unit'}",
                     f"take {listing(units)} out of the Base6")

    def on(self, port, label):
        with self._asking:
            if label not in self.out:
                return
            where = [f"{n} in slot {s}" if s else n for n, s in self.out.items()]
            several = len(where) > 1
            self.ask(f"Dock the {'units, each in its own slot' if several else 'unit'}",
                     f"put {'them' if several else 'it'} back in the Base6: {listing(where)}")
            self.out.clear()


def speak(text: str) -> None:
    """Say `text` aloud through Windows' speech synthesiser, so an operator away
    from the screen hears a request. Best effort: elsewhere, or if it fails, nothing."""
    if sys.platform != "win32":
        return
    try:
        # The text goes in the environment, so no quoting of it can break the command.
        subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command",
                        "Add-Type -AssemblyName System.Speech; "
                        "(New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak($env:HIL_SPEAK)"],
                       env={**os.environ, "HIL_SPEAK": text}, capture_output=True, timeout=60)
    except (OSError, subprocess.SubprocessError):
        pass


def wait_for(predicate, timeout_s: float, interval_s: float = 0.25, what: str = "condition"):
    """Poll until predicate() is truthy; return its value or raise TimeoutError."""
    end = time.monotonic() + timeout_s
    while True:
        v = predicate()
        if v:
            return v
        if time.monotonic() >= end:
            raise TimeoutError(f"timed out after {timeout_s:.0f} s waiting for {what}")
        time.sleep(interval_s)
