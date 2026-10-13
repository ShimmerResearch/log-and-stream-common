"""Flash a Shimmer3 in a Base6 slot without losing its configuration.

The MSP430 BSL's mass erase clears information memory B-D, which is where the
Shimmer keeps its InfoMem: configuration, device name, calibration. Consensys
re-writes InfoMem after flashing for the same reason. Here:

  1. read InfoMem over the dock and save it to disk (the copy that survives
     anything that goes wrong after this point);
  2. unmask the slot's BSL lines, program and CRC-verify the image, mask them;
  3. wait for the new firmware to answer on the dock;
  4. write InfoMem back and read it back - identical except for the bit the
     firmware sets on every write (byte 230 bit 0, flagWriteCfgToSd).
"""

from __future__ import annotations

import time
from pathlib import Path

from . import dock, msp430_bsl
from .base6 import Base6

SELF_SET_BITS = [(230, 0x01)]


class S3FlashError(Exception):
    pass


def _masked(im: bytes) -> bytes:
    b = bytearray(im)
    for off, mask in SELF_SET_BITS:
        b[off] &= ~mask & 0xFF
    return bytes(b)


def flash_slot(base6: Base6, slot: int, image: Path, backup_dir: Path,
               expect_version: str | None = None, log=print) -> dict:
    base6.select(slot)
    with dock.open_when_ready(base6.uart_port) as d:
        before = d.version()
        mac = d.mac()
        infomem = d.read_infomem()
    backup_dir.mkdir(parents=True, exist_ok=True)
    backup = backup_dir / f"infomem_{mac}_v{before.text}.bin"
    backup.write_bytes(infomem)
    log(f"slot {slot} ({mac}): v{before.text}, InfoMem saved to {backup}")

    base6.shimmer_bsl_lines(True)
    try:
        result = msp430_bsl.program(base6.bsl_port, image, log=log)
    finally:
        base6.shimmer_bsl_lines(False)
    log(f"slot {slot}: programmed and verified {result['bytes']} bytes in {result['seconds']} s")

    # The application boots straight after the BSL releases RST; give the
    # radio bring-up time before the first dock command.
    time.sleep(2)
    with dock.open_when_ready(base6.uart_port, attempts=30, delay_s=0.5) as d:
        after = d.version()
        if expect_version and after.text != expect_version.lstrip("vV"):
            raise S3FlashError(f"slot {slot}: flashed, but the unit reports v{after.text}, "
                               f"expected v{expect_version}")
        d.write_infomem(infomem)
        back = d.read_infomem()
    diff = [i for i, (a, b) in enumerate(zip(_masked(infomem), _masked(back))) if a != b]
    if diff:
        raise S3FlashError(f"slot {slot}: InfoMem restore differs at offsets {diff[:20]} "
                           f"- the original is in {backup}")
    log(f"slot {slot}: now v{after.text}, InfoMem restored")
    return {**result, "mac": mac, "before": before.text, "after": after.text, "backup": str(backup)}


def reset_slot(base6: Base6, slot: int) -> None:
    """Hardware-reset the Shimmer in `slot` through its RST line, into the application."""
    base6.select(slot)
    base6.shimmer_bsl_lines(True)
    try:
        with msp430_bsl.Bsl(base6.bsl_port) as b:
            b.exit_to_app()
    finally:
        base6.shimmer_bsl_lines(False)
