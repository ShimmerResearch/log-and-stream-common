"""Find a Shimmer3R's USB functions - the CDC port and the SD card's drive -
by its USB serial number, which survives re-enumeration (COM numbers and drive
letters do not reliably).

The unit enumerates as 0483:5710, a composite of CDC-ACM (the dock protocol)
and MSC (the SD card). Both disappear while it is sensing and come back when it
stops; the MSC side only exists while USB power is present and the card is
assigned to USB (log_and_stream_common.c, LogAndStream_assignSdToUsb).

The disk lookup uses PowerShell's Get-Disk (storage cmdlets, not WMI
Win32_SerialPort, which pages every paired Bluetooth device and slows classic
links for most of a minute).
"""

from __future__ import annotations

import json
import subprocess
import time
from pathlib import Path

from serial.tools import list_ports

VID, PID = 0x0483, 0x5710
DFU_VID, DFU_PID = 0x0483, 0xDF11


def cdc_port(usb_serial: str) -> str | None:
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID and (p.serial_number or "").upper() == usb_serial.upper():
            return p.device
    return None


def usb_disks() -> list[dict]:
    """[{number, name, serial, letters}] for every USB-attached disk."""
    ps = ("Get-Disk | Where-Object BusType -eq 'USB' | ForEach-Object { "
          "$l = @($_ | Get-Partition -ErrorAction SilentlyContinue | "
          "Get-Volume -ErrorAction SilentlyContinue | Where-Object DriveLetter | "
          "ForEach-Object { [string]$_.DriveLetter }); "
          "[pscustomobject]@{number=$_.Number; name=$_.FriendlyName; "
          "serial=([string]$_.SerialNumber).Trim(); letters=$l} } | ConvertTo-Json -Compress")
    r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
                       capture_output=True, text=True, encoding="utf-8", timeout=60)
    out = r.stdout.strip()
    if not out:
        return []
    data = json.loads(out)
    return data if isinstance(data, list) else [data]


def drive(usb_serial: str) -> Path | None:
    """The root of the unit's SD card as seen over USB, e.g. Path('F:/')."""
    for d in usb_disks():
        if d["serial"].upper() == usb_serial.upper() and d.get("letters"):
            return Path(f"{d['letters'][0]}:/")
    return None


def dfu_present() -> bool:
    """ST's system bootloader in DFU mode. It has no COM port, so ask PnP."""
    ps = (f"@(Get-PnpDevice -PresentOnly | Where-Object InstanceId -like 'USB\\VID_{DFU_VID:04X}&PID_{DFU_PID:04X}*').Count")
    r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
                       capture_output=True, text=True, encoding="utf-8", timeout=60)
    return r.stdout.strip() not in ("", "0")


def wait_cdc(usb_serial: str, timeout_s: float = 30.0) -> str:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        p = cdc_port(usb_serial)
        if p:
            return p
        time.sleep(0.5)
    raise TimeoutError(f"USB CDC port for {usb_serial} did not appear within {timeout_s:.0f} s")


def wait_cdc_gone(usb_serial: str, timeout_s: float = 15.0) -> None:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if not cdc_port(usb_serial):
            return
        time.sleep(0.25)
    raise TimeoutError(f"USB CDC port for {usb_serial} still present after {timeout_s:.0f} s")


def wait_drive(usb_serial: str, timeout_s: float = 45.0) -> Path:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        d = drive(usb_serial)
        if d and d.exists():
            return d
        time.sleep(1.0)
    raise TimeoutError(f"SD card of {usb_serial} did not mount over USB within {timeout_s:.0f} s")
