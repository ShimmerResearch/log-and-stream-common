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
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path

from serial.tools import list_ports

from .dock import DockError

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


def wait_usb_settled(usb_serial: str, timeout_s: float = 90.0) -> str:
    """The unit's CDC port, once its whole USB device is back: the port and the
    SD card's drive both present. Wait for this before the first dock exchange
    after sensing or a re-dock.

    With USB-C in, ShimSens_startSensing() tears the USB device down and the
    stop brings it back with USB_init(), CDC and MSC together, so every docked
    stream is a removal and a fresh enumeration. wait_cdc alone returns as soon
    as the COM port exists, which can be part-way through it: on 28 Oct 2026,
    2 of 88 release-tier configure steps failed straight after a docked stream
    - no reply, or the port's I/O aborted - while those after a log row, which
    had waited for the drive and read the log off it, never did (DEV-1368).
    wait_drive also makes Windows mount the volume, since it checks that the
    root exists.

    No fixed settle after that. Both functions come up in one enumeration, so
    once both are present it has finished; an exchange that fails then is the
    unit's fault, which a host opening the port would meet too, and a delay
    here would hide it. For the same reason a port that goes away again while
    the drive mounts is raised, not waited out.
    """
    end = time.monotonic() + timeout_s
    port = wait_cdc(usb_serial, timeout_s)
    try:
        wait_drive(usb_serial, max(1.0, end - time.monotonic()))
    except TimeoutError:
        raise TimeoutError(f"SD card of {usb_serial} did not mount over USB within {timeout_s:.0f} s, "
                           f"though its CDC port {port} was back") from None
    if not cdc_port(usb_serial):
        raise OSError(f"USB CDC port {port} of {usb_serial} went away again while its drive mounted")
    return port


# --- host service stalls (DEV-1368) ---------------------------------------------

# A failed dock exchange this soon after the unit's USB arrival is checked for a
# host service stall (stall_named).
STALL_WINDOW_S = 35.0
# The Service Control Manager's wait for a service to answer a control. When it
# runs out it logs System event 7011, so a stall shows only this long after it began.
SCM_TIMEOUT_S = 30.0
# Time for that event to reach the log.
EVENT_SLACK_S = 2.0


class HostServiceStall(OSError):
    """A dock exchange that failed inside a host service stall (stall_named). An
    OSError, so code that catches a failed exchange's SerialException catches it."""


def _powershell(ps: str) -> str:
    r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
                       capture_output=True, text=True, encoding="utf-8", timeout=60)
    return r.stdout.strip()


def last_arrival(usb_serial: str) -> float | None:
    """When Windows last saw the unit's USB device arrive, as a time.time() value,
    or None if it has no record: PnP's LastArrivalDate on the composite device,
    which moves at every enumeration, a docked stream's included."""
    ps = (f"$p = Get-PnpDeviceProperty -InstanceId 'USB\\VID_{VID:04X}&PID_{PID:04X}\\{usb_serial.upper()}' "
          "-KeyName DEVPKEY_Device_LastArrivalDate -ErrorAction SilentlyContinue; "
          "if ($p.Data) { [DateTimeOffset]::new($p.Data).ToUnixTimeMilliseconds() }")
    out = _powershell(ps)
    return int(out) / 1000 if out else None


def service_timeouts(since: float) -> list[tuple[float, str]]:
    """(time, service) for each System event 7011 logged since then, oldest first:
    the Service Control Manager gave up after SCM_TIMEOUT_S waiting for that
    service to answer a control."""
    back = int(time.time() - since) + 60
    ps = ("Get-WinEvent -FilterHashtable @{LogName='System'; Id=7011; "
          f"StartTime=(Get-Date).AddSeconds(-{back})}} -ErrorAction SilentlyContinue | "
          "ForEach-Object { [pscustomobject]@{ms=[DateTimeOffset]::new($_.TimeCreated).ToUnixTimeMilliseconds(); "
          "service=[string]$_.Properties[1].Value} } | ConvertTo-Json -Compress")
    out = _powershell(ps)
    if not out:
        return []
    data = json.loads(out)
    rows = data if isinstance(data, list) else [data]
    return sorted((r["ms"] / 1000, r["service"]) for r in rows if r["ms"] / 1000 >= since)


def _clock(t: float) -> str:
    return datetime.fromtimestamp(t).strftime("%H:%M:%S")


def host_stall(usb_serial: str, failed_at: float) -> str | None:
    """The host service stall that a dock exchange failing at failed_at (a
    time.time() value) fell inside, as text for its error, or None.

    Each failed dock exchange after a re-enumeration on a bench laptop on 28-29
    Oct 2026 fell inside a 30 s stall of two Windows services that handle
    device events, an endpoint-security sensor and the search indexer. Each
    stall began as the unit re-enumerated; stalls also came with no failure,
    and with no Shimmer on USB at all. A stall shows only when it ends, as
    System event 7011. So only a failure within STALL_WINDOW_S of the unit's
    last USB arrival is looked into, and the log is read once a stall over the
    failure would be in it, about 32 s after the failure.
    """
    arrived = last_arrival(usb_serial)
    if arrived is None or not 0 <= failed_at - arrived <= STALL_WINDOW_S:
        return None
    until = failed_at + SCM_TIMEOUT_S + EVENT_SLACK_S
    while (left := until - time.time()) > 0:
        time.sleep(left)
    stalls: dict[float, list[str]] = {}
    for t, service in service_timeouts(arrived):
        if t <= until:
            stalls.setdefault(t, []).append(service)
    if not stalls:
        return None
    logged = "; ".join(f"{', '.join(sorted(s))} at {_clock(t)}" for t, s in sorted(stalls.items()))
    return f"System event 7011 for {logged}; the unit's USB arrived at {_clock(arrived)}"


@contextmanager
def stall_named(usb_serial: str):
    """Dock exchanges with the unit, with a failure inside a host service stall named.

    A DockError or OSError from the block goes to host_stall, which for one
    within STALL_WINDOW_S of the unit's USB arrival waits about 32 s from the
    failure. If a stall was logged, the failure is raised again as
    HostServiceStall naming it, chained from it; otherwise it propagates as it
    was. Either way it is a failure: nothing here retries. The bench's record of
    these stalls is bench-tools hil/known_issues.md.
    """
    try:
        yield
    except HostServiceStall:
        raise
    except (DockError, OSError) as e:
        failed_at = time.time()
        try:
            stall = host_stall(usb_serial, failed_at)
        except (OSError, ValueError, KeyError, subprocess.SubprocessError):
            stall = None                  # a failed lookup must not replace the failure itself
        if stall is None:
            raise
        raise HostServiceStall(f"{type(e).__name__}: {e} - inside a host service stall "
                               f"({stall}); see bench-tools hil/known_issues.md") from e
