"""Find a Shimmer3R's USB functions - the CDC port and the SD card's drive -
by its USB serial number, which survives re-enumeration (COM numbers and drive
letters do not reliably).

The unit enumerates as 0483:5710, a composite of CDC-ACM (the dock protocol)
and MSC (the SD card). Both disappear while it is sensing and come back when it
stops; the MSC side only exists while USB power is present and the card is
assigned to USB (log_and_stream_common.c, LogAndStream_assignSdToUsb).

The disk lookup asks PowerShell's Get-Disk (the Storage Management provider)
first, and the Win32_DiskDrive -> Win32_DiskPartition -> Win32_LogicalDisk
association when Get-Disk does not list the unit. Get-Disk has failed two ways
on a bench laptop while Win32_DiskDrive listed every unit:
- It listed no disks at all, not even the system disk, for minutes at a time.
  Each time, the provider's host service had started with "Provider: MISpace"
  failing to load (StorageManagement/Operational event 1000), and it recovered
  at the service's next clean start.
- A unit back from a long undocked log was listed 1-2.5 min after Win32 listed
  it, while the other units were listed throughout.
Neither query touches WMI Win32_SerialPort, which pages every paired Bluetooth
device and slows classic links for most of a minute.
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


GET_DISK = ("Get-Disk | Where-Object BusType -eq 'USB' | ForEach-Object { "
            "$l = @($_ | Get-Partition -ErrorAction SilentlyContinue | "
            "Get-Volume -ErrorAction SilentlyContinue | Where-Object DriveLetter | "
            "ForEach-Object { [string]$_.DriveLetter }); "
            "[pscustomobject]@{number=$_.Number; name=$_.FriendlyName; "
            "serial=([string]$_.SerialNumber).Trim(); letters=$l} } | ConvertTo-Json -Compress")

# The same shape from the Win32 classes, which answer when Get-Disk does not
# (module doc). name is the Model, which ends in " USB Device".
WIN32_DISKS = ("Get-CimInstance Win32_DiskDrive | Where-Object InterfaceType -eq 'USB' | ForEach-Object { "
               "$d = $_; $l = @(Get-CimAssociatedInstance -InputObject $d -ResultClassName Win32_DiskPartition | "
               "ForEach-Object { (Get-CimAssociatedInstance -InputObject $_ "
               "-ResultClassName Win32_LogicalDisk).DeviceID } | "
               "Where-Object { $_ } | ForEach-Object { $_.TrimEnd(':') }); "
               "[pscustomobject]@{number=$d.Index; name=$d.Model; "
               "serial=([string]$d.SerialNumber).Trim(); letters=$l} } | ConvertTo-Json -Compress")


def _disks(ps: str) -> list[dict]:
    r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
                       capture_output=True, text=True, encoding="utf-8", timeout=60)
    out = r.stdout.strip()
    if not out:
        return []
    data = json.loads(out)
    return data if isinstance(data, list) else [data]


def _root(disks: list[dict], usb_serial: str) -> Path | None:
    for d in disks:
        if d["serial"].upper() == usb_serial.upper() and d.get("letters"):
            return Path(f"{d['letters'][0]}:/")
    return None


def usb_disks() -> list[dict]:
    """[{number, name, serial, letters}] for every USB-attached disk: Get-Disk's,
    or Win32_DiskDrive's when Get-Disk lists none."""
    return _disks(GET_DISK) or _disks(WIN32_DISKS)


def drive(usb_serial: str) -> Path | None:
    """The root of the unit's SD card as seen over USB, e.g. Path('F:/').
    Win32_DiskDrive is asked whenever Get-Disk does not give this unit a
    letter, even when Get-Disk lists other disks: its lag after a long log
    is on the one disk (module doc)."""
    return _root(_disks(GET_DISK), usb_serial) or _root(_disks(WIN32_DISKS), usb_serial)


def dfu_devices() -> list[str]:
    """PnP instance ids of every ST system bootloader present in DFU mode. It has
    no COM port, so ask PnP. Any STM32 in its bootloader looks the same here, so
    a caller that wants one unit must be sure no other is in DFU (flash.flash_dfu)."""
    ps = ("Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -like "
          f"'USB\\VID_{DFU_VID:04X}&PID_{DFU_PID:04X}*' -and $_.InstanceId -notlike '*&MI_*' }} | "
          "ForEach-Object { $_.InstanceId }")
    r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
                       capture_output=True, text=True, encoding="utf-8", timeout=60)
    return [ln.strip() for ln in r.stdout.splitlines() if ln.strip()]


def dfu_present() -> bool:
    return bool(dfu_devices())


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
