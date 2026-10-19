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
# A unit's throughput baseline (rig/throughput.py): what the last release gave
# on this bench, in these units.
THROUGHPUT_KEYS = ("link_bytes_per_s", "stream_samples_per_s")


class RigError(Exception):
    """rig.toml is missing, malformed, or names hardware that is not there."""


@dataclass
class Unit:
    name: str
    platform: str
    mac: str | None = None          # Bluetooth MAC, 12 hex digits, no separators
    usb_serial: str | None = None   # Shimmer3R USB iSerial (CDC + MSC)
    hub_port: int | None = None     # Exsys hub port that powers its USB-C
    base6_slot: int | None = None   # Shimmer3: Base6 slot (1-6) it sits in
    radio: str | None = None        # e.g. "RN42 v6.15", "IF820 v1.4.16.16"
    board: str | None = None        # SR number / revision, free text
    notes: str = ""
    stlink_serial: str | None = None  # the ST-Link on this unit's SWD header, for flash method "swd"
    throughput: dict[str, float] = field(default_factory=dict)  # baseline figures, THROUGHPUT_KEYS

    def capabilities(self) -> set[str]:
        caps = {self.platform}
        if self.mac:
            caps.add("bt")
        if self.usb_serial and self.platform == "shimmer3r":
            caps.add("usb")
        if self.hub_port:
            caps.add("hub_port")
        if self.base6_slot:
            caps.add("dock")
        return caps


@dataclass
class Rig:
    path: Path
    bench: str
    units: list[Unit]
    hub_serial: str | None = None
    base6_serial: str | None = None  # FT4232 serial without the channel letter
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
            if not self.base6_serial:
                caps.discard("dock")
        return caps

    def units_for(self, platform: str) -> list[Unit]:
        return [u for u in self.units if u.platform == platform]

    def stlink_for(self, unit: Unit) -> str | None:
        """The ST-Link wired to this unit's SWD header: the unit's own, or else the
        bench's [tools] stlink_serial, which load() allows only when one Shimmer3R
        unit could be on it."""
        return unit.stlink_serial or self.stlink_serial


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
        hub_port = u.get("hub_port")
        if hub_port is not None and (isinstance(hub_port, bool) or not isinstance(hub_port, int) or hub_port < 1):
            raise RigError(f"{where}: hub_port must be a hub port number from 1, not {hub_port!r}")
        throughput = u.get("throughput", {})
        if not isinstance(throughput, dict):
            raise RigError(f"{where}: throughput must be a table, e.g. throughput = {{ link_bytes_per_s = 39315 }}")
        for k, v in throughput.items():
            if k not in THROUGHPUT_KEYS:
                raise RigError(f"{where}: throughput.{k} is not one of {', '.join(THROUGHPUT_KEYS)}")
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not v > 0:
                raise RigError(f"{where}: throughput.{k} must be a positive number, not {v!r}")
        units.append(Unit(
            name=u["name"], platform=u["platform"],
            mac=_normalise_mac(u["mac"]) if u.get("mac") else None,
            # Compared without regard to case wherever it is looked up (s3r_usb).
            usb_serial=str(u["usb_serial"]).upper() if u.get("usb_serial") else None,
            hub_port=hub_port,
            base6_slot=int(u["base6_slot"]) if u.get("base6_slot") else None,
            radio=u.get("radio"), board=u.get("board"), notes=u.get("notes", ""),
            stlink_serial=u.get("stlink_serial") or None,
            throughput={k: float(v) for k, v in throughput.items()},
        ))
    names = [u.name for u in units]
    if len(set(names)) != len(names):
        raise RigError(f"{p}: unit names must be unique")
    # Two units with one MAC, USB serial, hub port or Base6 slot would be one
    # device tested twice, or the wrong one switched off.
    for field_name in ("mac", "usb_serial", "hub_port", "base6_slot"):
        values = [getattr(u, field_name) for u in units if getattr(u, field_name)]
        dupes = sorted({str(v) for v in values if values.count(v) > 1})
        if dupes:
            raise RigError(f"{p}: two units share {field_name} {', '.join(dupes)}")

    hub = data.get("hub", {})
    tools = data.get("tools", {})
    # An ST-Link reaches one unit's SWD header. A bench-wide one used for
    # several units would program the same unit again and again, so each unit
    # names its own unless only one could be meant.
    bench_stlink = tools.get("stlink_serial") or None
    s3r = [u for u in units if u.platform == "shimmer3r"]
    if bench_stlink and sum(1 for u in s3r if not u.stlink_serial) > 1:
        raise RigError(f"{p}: [tools] stlink_serial would be used for every Shimmer3R unit that names "
                       "none, and an ST-Link reaches one unit: give each unit its own stlink_serial")
    used = [u.stlink_serial for u in s3r if u.stlink_serial]
    if bench_stlink and any(not u.stlink_serial for u in s3r):
        used.append(bench_stlink)
    if len(set(used)) != len(used):
        raise RigError(f"{p}: two Shimmer3R units name the same ST-Link")
    return Rig(
        path=p, bench=bench, units=units,
        hub_serial=hub.get("serial") or None,
        base6_serial=data.get("base6", {}).get("serial") or None,
        bench_tools=Path(tools["bench_tools"]) if tools.get("bench_tools") else None,
        stm32_programmer=Path(tools["stm32_programmer"]) if tools.get("stm32_programmer") else None,
        stlink_serial=tools.get("stlink_serial") or None,
        extra={k: v for k, v in data.items() if k not in ("bench", "unit", "hub", "tools", "base6")},
    )
