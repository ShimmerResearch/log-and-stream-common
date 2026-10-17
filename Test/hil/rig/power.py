"""Power to a unit's USB-C, which on Shimmer3R is also dock/undock.

HubPower switches a port on the Exsys managed hub through bench-tools'
hubctl.py. ManualPower asks the operator to plug or unplug, so the same test
runs on a bench without a hub - slower, but it runs. Both have the same
interface; tests never know which they got.

hubctl is imported from a bench-tools checkout named in rig.toml rather than
copied here: it is the one place the hub's protocol and its safety rules live
(its README), and a copy would drift.
"""

from __future__ import annotations

import importlib.util
import sys
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
        self._switched_off: set[int] = set()

    def _require(self, port):
        if not port:
            raise RigError("this unit has no hub_port in rig.toml")

    def on(self, port, label):
        self._require(port)
        self.hub.switch([port], True)
        self._switched_off.discard(port)

    def off(self, port, label):
        self._require(port)
        # Only a port that was on is this run's to restore in close().
        if self.is_on(port):
            self._switched_off.add(port)
        self.hub.switch([port], False)

    def is_on(self, port):
        self._require(port)
        return bool(self.hub.states() & (1 << (port - 1)))

    def devices_on(self, port):
        """USB device nodes currently enumerated on a hub port."""
        return self.location.devices().get(port, [])

    def close(self):
        # Never leave a port this run switched off: a unit left undocked runs
        # its battery down and the next run starts from an unknown state.
        # Ports that were already off before the run are not ours to change.
        try:
            if self._switched_off:
                self.hub.switch(sorted(self._switched_off), True)
                self._switched_off.clear()
        finally:
            self.hub.close()


class ManualPower(Power):
    """For a bench without a hub: the operator is the switch."""

    def __init__(self, prompt=input, out=print):
        self.prompt, self.out = prompt, out

    def on(self, port, label):
        self.prompt(f"\n>>> Connect USB-C to {label} (dock it), then press Enter ")

    def off(self, port, label):
        self.prompt(f"\n>>> Disconnect USB-C from {label} (undock it), then press Enter ")


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
