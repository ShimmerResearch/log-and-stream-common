"""Flash a Shimmer3 in a Base6 slot without losing its configuration.

The MSP430 BSL's mass erase clears information memory B-D, which is where the
Shimmer keeps its InfoMem: configuration, device name, calibration. Consensys
re-writes InfoMem after flashing for the same reason. Here:

  1. read InfoMem over the dock and save it to disk (the copy that survives
     anything that goes wrong after this point);
  2. unmask the slot's BSL lines, program and CRC-verify the image, mask them;
  3. wait for the new firmware to answer on the dock;
  4. write InfoMem back and read it back - identical except for the bit the
     firmware sets on every write (byte 230 bit 0, flagWriteCfgToSd), and the
     fields a newer firmware corrects (FIRMWARE_CORRECTED_BITS), which are
     logged and returned as corrections.
"""

from __future__ import annotations

import time
from pathlib import Path

from . import dock, msp430_bsl
from .base6 import Base6

SELF_SET_BITS = [(230, 0x01)]

# Fields that ShimConfig_checkAndCorrectConfig() rewrites in firmware newer than
# v1.01.009, which corrects none of them. Restoring an image saved under older
# firmware is a configuration write like any other, so the new firmware
# corrects these on the way in. A unit whose bytes 232 and 233 were never
# written by a current host holds 0xFF there, and reads back something else.
# Sections are in docs/SHIMMER3_CONFIGURATION_INFOMEM.md.
FIRMWARE_CORRECTED_BITS = [
    (217, 0x20),  # userButtonEnable: held at byte 232's nearest equivalent (8.1, 8.3)
    (218, 0x80),  # singleTouchStart: cleared when byte 232's mode can't start on a press (8.3)
    (232, 0xFF),  # button, logging and TTL input: only what the firmware acts on (8.3)
    (233, 0xFF),  # audio mode: 0 on Shimmer3, which has no microphone (8.5)
]


class S3FlashError(Exception):
    pass


def _masked(im: bytes) -> bytes:
    """im (InfoMem from offset 0, whole or only its start) with the bits the
    firmware sets itself cleared. A caller that restores only the sampling
    divider passes two bytes, so offsets beyond im are left alone."""
    b = bytearray(im)
    for off, mask in SELF_SET_BITS:
        if off < len(b):
            b[off] &= ~mask & 0xFF
    return bytes(b)


def _restore_differences(written: bytes, back: bytes) -> tuple[list[int], list[tuple[int, int, int]]]:
    """Compare a restored InfoMem with what was written. Returns the offsets that
    differ outside the fields the firmware may set or correct, and each
    corrected byte as (offset, written, read back)."""
    corrected_mask = dict(FIRMWARE_CORRECTED_BITS)
    unexplained, corrected = [], []
    for i, (a, b) in enumerate(zip(_masked(written), _masked(back))):
        if a == b:
            continue
        if (a ^ b) & ~corrected_mask.get(i, 0) & 0xFF:
            unexplained.append(i)
        else:
            corrected.append((i, written[i], back[i]))
    return unexplained, corrected


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
    diff, corrected = _restore_differences(infomem, back)
    if diff:
        raise S3FlashError(f"slot {slot}: InfoMem restore differs at offsets {diff[:20]} "
                           f"- the original is in {backup}")
    if corrected:
        log(f"slot {slot}: v{after.text} corrected InfoMem "
            + ", ".join(f"byte {i} 0x{a:02x} -> 0x{b:02x}" for i, a, b in corrected))
    log(f"slot {slot}: now v{after.text}, InfoMem restored")
    return {**result, "mac": mac, "before": before.text, "after": after.text, "backup": str(backup),
            "corrected": [{"offset": i, "written": a, "read_back": b} for i, a, b in corrected]}


RESTORE_ATTEMPTS = 3


def restore_infomem(base6: Base6, original: bytes, stream_error: BaseException | None,
                    backup=None) -> list[str]:
    """Write the selected unit's own InfoMem (from offset 0) back over the dock;
    return what each failed attempt saw.

    Runs in a finally block, so a failure here must not replace the test's
    own exception: when both fail, the DockError names the test's error and
    is chained from it.
    """
    seen = []
    for _ in range(RESTORE_ATTEMPTS):
        try:
            with dock.open_when_ready(base6.uart_port) as d:
                d.write_infomem(original, offset=0)
                if _masked(d.read_infomem(0, len(original))) != _masked(original):
                    raise dock.DockError("read back differs")
            return seen
        except (dock.DockError, OSError) as e:   # serial.SerialException is an OSError
            seen.append(str(e))
            time.sleep(1.0)
    msg = f"could not restore the configuration over the dock: {seen}"
    if backup is not None:
        msg += f"; the original is in {backup}"
    if stream_error is not None:
        raise dock.DockError(f"{msg} - after the stream itself failed with {stream_error!r}") from stream_error
    raise dock.DockError(msg)


def reset_slot(base6: Base6, slot: int) -> None:
    """Hardware-reset the Shimmer in `slot` through its RST line, into the application."""
    base6.select(slot)
    base6.shimmer_bsl_lines(True)
    try:
        with msp430_bsl.Bsl(base6.bsl_port) as b:
            b.exit_to_app()
    finally:
        base6.shimmer_bsl_lines(False)
