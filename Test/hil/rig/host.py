"""The PC a run is on: its name, OS and Bluetooth adapters, for the report's run.hardware.

A bench may be one of several PCs, and Bluetooth figures depend on the
adapter and its driver as much as on the unit, so every report names them.
Nothing here is fatal: a host whose adapters cannot be read reports none.
"""

from __future__ import annotations

import json
import os
import platform
import subprocess
import sys

# The radios themselves, not the paired devices and services Windows also files
# under the Bluetooth class: those have BTHENUM / BTHLE instance IDs.
_ADAPTERS_PS = (
    "$d = Get-PnpDevice -Class Bluetooth -PresentOnly | Where-Object { $_.InstanceId -like 'USB\\*' "
    "-or $_.InstanceId -like 'PCI\\*' }; "
    "$d | ForEach-Object { [pscustomobject]@{ name = $_.FriendlyName; "
    "driver = (Get-PnpDeviceProperty -InstanceId $_.InstanceId -KeyName DEVPKEY_Device_DriverVersion).Data } } "
    "| ConvertTo-Json -Compress"
)


def parse_adapters(text: str) -> list[dict]:
    """The adapters from the PowerShell query's JSON: one object, a list, or nothing."""
    text = (text or "").strip()
    if not text:
        return []
    try:
        data = json.loads(text)
    except ValueError:
        return []
    items = data if isinstance(data, list) else [data]
    return [{"name": str(i.get("name") or ""), "driver": str(i.get("driver") or "")}
            for i in items if isinstance(i, dict) and i.get("name")]


def bluetooth_adapters(timeout_s: float = 20.0) -> list[dict]:
    if sys.platform != "win32":
        return []
    try:
        out = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", _ADAPTERS_PS],
                             capture_output=True, text=True, timeout=timeout_s).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return parse_adapters(out)


def describe() -> dict:
    return {"name": os.environ.get("COMPUTERNAME") or platform.node(),
            "os": platform.platform(),
            "bluetooth": bluetooth_adapters()}
