"""Put a release candidate's image onto a Shimmer3R.

Two routes, both through STM32_Programmer_CLI (docs/SHIMMER3_BUILD_AND_PROGRAMMING.md §5.2):

  swd  An ST-Link wired to the unit's SWD header. Works on a unit whose
       firmware is broken, so it is the route of last resort and the default
       when rig.toml names an ST-Link.
  dfu  No extra hardware: the dock command ENTER_BOOTLOADER reboots the unit
       into ST's system bootloader, which enumerates as USB DFU on the same
       USB-C cable. Needs working firmware to start from.

Either way the image is verified after writing (-v), and the unit is then
waited for on USB and asked its version, so "flashed" means "running the
version we expected", not "the programmer exited 0".
"""

from __future__ import annotations

import subprocess
import time
from pathlib import Path

from . import dock, s3r_usb


class FlashError(Exception):
    pass


def _run(cli: Path, args: list[str], timeout_s: float = 300) -> str:
    r = subprocess.run([str(cli), *args], capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=timeout_s)
    out = r.stdout + r.stderr
    # The CLI's exit status is not reliable across versions; its own verdict is.
    if r.returncode != 0 or ("Error" in out and "Download verified successfully" not in out):
        tail = "\n".join(out.strip().splitlines()[-15:])
        raise FlashError(f"STM32_Programmer_CLI {' '.join(args)} failed:\n{tail}")
    return out


def flash_swd(cli: Path, image: Path, stlink_serial: str | None = None) -> str:
    conn = ["-c", "port=SWD", "mode=UR", "reset=HWrst"]
    if stlink_serial:
        conn.append(f"sn={stlink_serial}")
    return _run(cli, [*conn, "-w", str(image), "-v", "-rst"])


def flash_dfu(cli: Path, image: Path, usb_serial: str, timeout_s: float = 30) -> str:
    port = s3r_usb.wait_cdc(usb_serial, 15)
    with dock.open_when_ready(port) as d:
        d.enter_bootloader(1)
    s3r_usb.wait_cdc_gone(usb_serial, 15)
    end = time.monotonic() + timeout_s
    while not s3r_usb.dfu_present():
        if time.monotonic() > end:
            raise FlashError("unit did not enumerate as USB DFU after ENTER_BOOTLOADER")
        time.sleep(0.5)
    # -s with no address starts the application from the vector table the image set.
    return _run(cli, ["-c", "port=USB1", "-w", str(image), "-v", "-s"])


def flash(cli: Path, image: Path, usb_serial: str, method: str = "dfu",
          stlink_serial: str | None = None, expect_version: str | None = None,
          boot_timeout_s: float = 60) -> dock.Version:
    """Flash, wait for the unit to come back on USB, and return the version it reports."""
    image = Path(image)
    if not image.exists():
        raise FlashError(f"{image}: no such image")
    if method == "swd":
        flash_swd(cli, image, stlink_serial)
    elif method == "dfu":
        flash_dfu(cli, image, usb_serial)
    else:
        raise FlashError(f"unknown flash method {method!r}")

    port = s3r_usb.wait_cdc(usb_serial, boot_timeout_s)
    with dock.open_when_ready(port, attempts=20) as d:
        v = d.version()
    if expect_version and v.text != expect_version.lstrip("vV"):
        raise FlashError(f"flashed, but the unit reports v{v.text}, expected v{expect_version}")
    return v
