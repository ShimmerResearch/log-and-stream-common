"""Bluetooth on for the run, whatever the unit's configuration says.

A unit configured for SD sync (InfoMem byte 217 bit 2, syncEnable) or with
Bluetooth disabled (bit 3, bluetoothDisable) keeps its radio off until it is
needed for sync, or for ever, so every Bluetooth test against it times out -
and reads like a radio or link failure. Units are shipped and kept that way:
every Shimmer3 on our Base6 has sync on.

So for each unit whose tests need Bluetooth, the run clears those two bits
first and puts the unit's InfoMem back, byte for byte, when it ends. The
read-back is checked against the original everywhere but byte 230 bit 0
(flagWriteCfgToSd), which the firmware sets itself on every configuration
write - the same exception s3_flash makes. A dock
InfoMem write only stores the configuration (Comms/shimmer_dock_usart.c); the
firmware decides whether the radio is on when it boots
(ShimConfig_checkBtModeFromConfig), so each change is followed by a reset:

  Shimmer3   the Base6 slot's RST line (s3_flash.reset_slot)
  Shimmer3R  SWD reset when an ST-Link reaches the unit, else ENTER_BOOTLOADER
             over the dock and a start from the ST bootloader (flash.restart),
             which can miss the jump and leave the unit for a hand reset (DEV-1316)
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from .dock import INFOMEM_SIZE
from .s3_flash import _masked

SYNC_CFG_OFFSET = 217            # NV_SD_TRIAL_CONFIG0 (Configuration/shimmer_config.h)
SYNC_ENABLE, BT_DISABLE = 0x04, 0x08
BT_BLOCKING = SYNC_ENABLE | BT_DISABLE


class BtModeError(Exception):
    pass


def blocks_bt(infomem: bytes) -> bool:
    return bool(infomem[SYNC_CFG_OFFSET] & BT_BLOCKING)


def unblocked(infomem: bytes) -> bytes:
    """The configuration byte with SD sync and bluetoothDisable cleared."""
    return bytes([infomem[SYNC_CFG_OFFSET] & ~BT_BLOCKING & 0xFF])


@dataclass
class BtMode:
    """Clears the bits on the units given to enable() and restores them in restore().

    `access(unit)` is a context manager yielding a DockLink to the unit, and
    `reset(unit)` reboots it into its application; both are injected so the
    bookkeeping can be tested without hardware.
    """
    access: object
    reset: object
    log: object = print
    saved: dict = field(default_factory=dict)      # unit name -> (unit, original InfoMem)

    def enable(self, unit) -> None:
        if unit.name in self.saved:
            return
        with self.access(unit) as d:
            original = bytes(d.read_infomem())
            if len(original) != INFOMEM_SIZE:
                # A short read is not an original to restore from: written
                # back, it would leave the rest as the run left it, and a
                # read-back of the same short length would pass as restored.
                raise BtModeError(f"{unit.name}: InfoMem read gave {len(original)} bytes, "
                                  f"expected {INFOMEM_SIZE}; nothing was changed")
            if not blocks_bt(original):
                self.saved[unit.name] = (unit, None)
                return
            # Saved before the write: if the write, the reset or anything after
            # them fails, restore() still puts the original back.
            self.saved[unit.name] = (unit, original)
            d.write_infomem(unblocked(original), offset=SYNC_CFG_OFFSET)
        self.reset(unit)
        with self.access(unit) as d:
            now = d.read_infomem(SYNC_CFG_OFFSET, 1)[0]
        if now & BT_BLOCKING:
            # The firmware put it back: singleTouchStart forces syncEnable
            # (ShimConfig_checkAndCorrectConfig).
            raise BtModeError(f"{unit.name}: byte {SYNC_CFG_OFFSET} reads {now:#04x} after clearing "
                              f"SD sync and bluetoothDisable - is singleTouchStart set?")
        self.log(f"{unit.name}: SD sync / bluetoothDisable cleared for the run "
                 f"(byte {SYNC_CFG_OFFSET} {original[SYNC_CFG_OFFSET]:#04x} -> {now:#04x})")

    def restore(self) -> list[str]:
        """Put every changed unit back. Returns the problems, having tried them all."""
        problems = []
        for name, (unit, original) in list(self.saved.items()):
            if original is None:
                continue
            try:
                with self.access(unit) as d:
                    d.write_infomem(original)
                    back = bytes(d.read_infomem())
                self.reset(unit)
                # Byte for byte, as promised, but for the bit the firmware sets
                # itself on every config write (s3_flash.SELF_SET_BITS).
                diff = [i for i, (a, b) in enumerate(zip(_masked(original), _masked(back))) if a != b]
                if len(back) != len(original) or diff:
                    raise BtModeError(f"read back {len(back)} bytes, wrote {len(original)}; "
                                      f"differs at offsets {diff[:20]}")
                self.log(f"{name}: InfoMem restored (byte {SYNC_CFG_OFFSET} {original[SYNC_CFG_OFFSET]:#04x})")
            except Exception as e:                        # keep going: the other units still need theirs
                problems.append(f"{name}: InfoMem not restored: {e}")
        return problems


# --- the bench's access and reset routes ---------------------------------------

def bench_routes(rig):
    """(access, reset) for the units of this rig."""
    from contextlib import contextmanager

    from . import dock, flash, s3r_usb

    @contextmanager
    def access(unit):
        if unit.platform == "shimmer3":
            from .base6 import Base6
            with Base6(rig.base6_serial) as b:
                b.select(unit.base6_slot)
                with dock.open_when_ready(b.uart_port) as d:
                    yield d
        else:
            with dock.open_when_ready(s3r_usb.wait_cdc(unit.usb_serial, 60)) as d:
                yield d

    def reset(unit):
        if unit.platform == "shimmer3":
            from .base6 import Base6
            from .s3_flash import reset_slot
            with Base6(rig.base6_serial) as b:
                reset_slot(b, unit.base6_slot)
            time.sleep(4)                                # radio bring-up
        else:
            if not rig.stm32_programmer:
                raise BtModeError("resetting a Shimmer3R needs [tools] stm32_programmer")
            try:
                # A hardware reset through the unit's ST-Link if it has one, else a
                # round trip through ST's bootloader.
                flash.restart(rig.stm32_programmer, unit.usb_serial, rig.stlink_for(unit), 60)
            except flash.FlashError as e:
                # Both ways this ends with the unit in ST's ROM bootloader (PC in
                # system memory, 0x0BF9xxxx) and nothing over USB can start it:
                # it never enumerated as DFU (seen 18 Oct 2026), or the bootloader
                # missed the jump to the application (DEV-1316). Say so, rather
                # than leave a unit that looks dead.
                raise BtModeError(f"{unit.name}: left in ST's bootloader ({e}). Reset it over SWD "
                                  "or power-cycle it, and name its ST-Link (stlink_serial) in "
                                  "rig.toml so the run resets it that way.") from e
            time.sleep(3)

    return access, reset
