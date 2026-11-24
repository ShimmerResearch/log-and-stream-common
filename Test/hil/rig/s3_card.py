"""A docked Shimmer3's SD card, read through the Base6's card reader.

A Shimmer3 has no USB of its own. The Base6 switches a slot's card to its USB
card reader ("Shimmer HS-SD/MMC"), where it mounts as a drive:
Base6.card_to_pc(slot) sends SDC and Base6.select(slot) gives it back
(rig/base6.py).

Two things make that drive awkward:
- Its letter is not fixed, and a single-slot dock's reader has the same name.
  So it is found as a reader's drive that was not there before the SDC.
- Straight after a Shimmer3 is put back in the Base6 the mount comes and goes
  for a while (bench, 16 Oct 2026). So anything read from the card is read with
  retries (with_retries), each finding the drive again, and a session is copied
  off the card (copy_session) and checked from the copy.

On the card, a Shimmer3 with no real-world clock dates its files 1981, so a
session is found by its folder's name (sdfile.trial_sessions), never by date.
"""

from __future__ import annotations

import shutil
import time
from contextlib import contextmanager
from pathlib import Path

READER_NAME = "HS-SD"        # in the readers' disk name, "Shimmer HS-SD/MMC"
MOUNT_S = 40.0               # how long the card gets to mount after the SDC
GONE_S = 20.0                # how long a card given back gets for its drive to go
ATTEMPTS, RETRY_S = 5, 3.0


class CardError(OSError):
    """The card's drive went away, or a copy came off it short: worth another try."""


class CardNotMounted(Exception):
    """The card never mounted at all. Not an OSError, so with_retries does not
    spend its attempts waiting again for a reader that has shown nothing."""


def reader_disks(disks: list[dict] | None = None) -> list[dict]:
    """The Base6 readers among the USB disks (s3r_usb.usb_disks's shape)."""
    if disks is None:
        from .s3r_usb import usb_disks
        disks = usb_disks()
    return [d for d in disks if READER_NAME in str(d.get("name") or "").upper()]


def reader_roots(disks: list[dict] | None = None) -> set[Path]:
    """Roots of the readers' drives that have a card in now."""
    return {Path(f"{letter}:/") for d in reader_disks(disks) for letter in (d.get("letters") or [])}


class Card:
    """A slot's card while it is with the PC. root() finds its drive, waiting for
    it to mount: a reader's drive that was not there before the SDC."""

    def __init__(self, before: set[Path], roots=reader_roots, mount_s: float = MOUNT_S,
                 sleep=time.sleep, clock=time.monotonic):
        self.before, self.roots, self.mount_s = before, roots, mount_s
        self.sleep, self.clock = sleep, clock
        self.mounted = False            # the drive has been found at least once

    def root(self) -> Path:
        end = self.clock() + self.mount_s
        while True:
            new = sorted(r for r in self.roots() - self.before if r.exists())
            if new:
                self.mounted = True
                return new[0]
            if self.clock() >= end:
                what = (f"within {self.mount_s:.0f} s (readers with a card: "
                        f"{sorted(map(str, self.roots())) or 'none'})")
                if self.mounted:
                    raise CardError(f"the card's drive went away and did not come back {what}")
                raise CardNotMounted(f"no Base6 card reader drive appeared {what}")
            self.sleep(1.0)


@contextmanager
def on_pc(base6, slot: int, *, roots=reader_roots, mount_s: float = MOUNT_S, gone_s: float = GONE_S,
          sleep=time.sleep, clock=time.monotonic):
    """The slot's card on the PC for the with-block, then back with its Shimmer.

    No card should be with the PC beforehand, but the last one given back can
    leave its drive mounted for a moment. The next card may take the same
    reader and letter, and a drive already there before the SDC is never taken
    for the new card. So any reader drive still there is given up to `gone_s`
    to go before the SDC.
    """
    end = clock() + gone_s
    while True:
        before = {r for r in roots() if r.exists()}
        if not before or clock() >= end:
            break
        sleep(1.0)
    base6.card_to_pc(slot)
    try:
        yield Card(before, roots, mount_s, sleep, clock)
    finally:
        base6.select(slot)


def with_retries(card: Card, fn, attempts: int = ATTEMPTS, wait_s: float = RETRY_S, sleep=time.sleep):
    """fn(root) on the card's drive, again on an OSError (the mount went away),
    finding the drive afresh each time."""
    seen = []
    for _ in range(attempts):
        try:
            return fn(card.root())
        except OSError as e:
            seen.append(f"{type(e).__name__}: {str(e)[:120]}")
            sleep(wait_s)
    raise CardError(f"card access failed {attempts} times: {seen}")


def copy_session(root: Path, rel: Path, dest_root: Path) -> Path:
    """Copy the session directory root/rel to dest_root/rel and return the copy.
    Raises OSError unless every file arrived whole, so with_retries tries again."""
    src, dst = Path(root) / rel, Path(dest_root) / rel
    if dst.exists():
        shutil.rmtree(dst)          # a copy cut short by the mount going away
    shutil.copytree(src, dst)
    for f in src.iterdir():
        if f.is_file():
            got, want = (dst / f.name).stat().st_size, f.stat().st_size
            if got != want:
                raise CardError(f"{rel / f.name}: copied {got} of {want} bytes")
    return dst
