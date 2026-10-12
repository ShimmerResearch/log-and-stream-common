"""The bench description: what hardware this machine has, and how to find it.

One rig.toml per bench (rig.example.toml is the template). Units are named by
something that survives a reboot - a MAC, a USB serial number, a hub port -
never by COM number or drive letter, which Windows reassigns.

Capabilities are derived from what the file declares, and tests ask for them
with @pytest.mark.hil(..., needs=[...]). A bench that lacks one reports the
affected items as not covered rather than failing or skipping them silently -
see conftest.py.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:  # pragma: no cover - 3.10 benches
    import tomli as tomllib

PLATFORMS = ("shimmer3", "shimmer3r")


class RigError(Exception):
    """rig.toml is missing, malformed, or names hardware that is not there."""


@dataclass
class Unit:
    name: str
    platform: str
    mac: str | None = None          # Bluetooth MAC, 12 hex digits, no separators
    usb_serial: str | None = None   # Shimmer3R USB iSerial (CDC + MSC)
    hub_port: int | None = None     # Exsys hub port that powers its USB-C
    radio: str | None = None        # e.g. "RN42 v6.15", "IF820 v1.4.16.16"
    board: str | None = None        # SR number / revision, free text
    notes: str = ""

    def capabilities(self) -> set[str]:
        caps = {self.platform}
        if self.mac:
            caps.add("bt")
        if self.usb_serial and self.platform == "shimmer3r":
            caps.add("usb")
        if self.hub_port:
            caps.add("hub_port")
        return caps


@dataclass
class Rig:
    path: Path
    bench: str
    units: list[Unit]
    hub_serial: str | None = None
    bench_tools: Path | None = None
    stm32_programmer: Path | None = None
    stlink_serial: str | None = None
    extra: dict = field(default_factory=dict)

    def capabilities(self, unit: Unit | None = None) -> set[str]:
        caps: set[str] = set()
        if self.hub_serial and self.bench_tools:
            caps.add("hub")
        if self.stm32_programmer and self.stm32_programmer.exists():
            caps.add("programmer:shimmer3r")
        if unit:
            caps |= unit.capabilities()
            # Power-switching a unit needs both a hub and a port to switch.
            if "hub" not in caps or "hub_port" not in caps:
                caps.discard("hub_port")
        return caps

    def units_for(self, platform: str) -> list[Unit]:
        return [u for u in self.units if u.platform == platform]


def _normalise_mac(mac: str) -> str:
    m = mac.replace(":", "").replace("-", "").upper()
    if len(m) != 12 or any(c not in "0123456789ABCDEF" for c in m):
        raise RigError(f"not a MAC address: {mac!r}")
    return m


def load(path: str | os.PathLike | None = None) -> Rig:
    """Read rig.toml. Default: $HIL_RIG, else rig.toml beside this package."""
    p = Path(path or os.environ.get("HIL_RIG") or Path(__file__).resolve().parent.parent / "rig.toml")
    if not p.exists():
        raise RigError(f"{p}: no rig description. Copy rig.example.toml to rig.toml and fill it in.")
    try:
        with open(p, "rb") as f:
            data = tomllib.load(f)
    except tomllib.TOMLDecodeError as e:
        raise RigError(f"{p}: {e}") from e

    bench = data.get("bench")
    if not isinstance(bench, str) or not bench:
        raise RigError(f"{p}: 'bench' must name this bench")

    units = []
    for i, u in enumerate(data.get("unit", [])):
        where = f"{p}: unit[{i}]"
        if u.get("platform") not in PLATFORMS:
            raise RigError(f"{where}: platform must be one of {', '.join(PLATFORMS)}")
        if not u.get("name"):
            raise RigError(f"{where}: missing name")
        units.append(Unit(
            name=u["name"], platform=u["platform"],
            mac=_normalise_mac(u["mac"]) if u.get("mac") else None,
            usb_serial=u.get("usb_serial") or None,
            hub_port=int(u["hub_port"]) if u.get("hub_port") else None,
            radio=u.get("radio"), board=u.get("board"), notes=u.get("notes", ""),
        ))
    names = [u.name for u in units]
    if len(set(names)) != len(names):
        raise RigError(f"{p}: unit names must be unique")

    hub = data.get("hub", {})
    tools = data.get("tools", {})
    return Rig(
        path=p, bench=bench, units=units,
        hub_serial=hub.get("serial") or None,
        bench_tools=Path(tools["bench_tools"]) if tools.get("bench_tools") else None,
        stm32_programmer=Path(tools["stm32_programmer"]) if tools.get("stm32_programmer") else None,
        stlink_serial=tools.get("stlink_serial") or None,
        extra={k: v for k, v in data.items() if k not in ("bench", "unit", "hub", "tools")},
    )
