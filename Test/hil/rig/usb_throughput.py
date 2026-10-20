"""Shimmer3R SD card throughput over USB mass storage (smoke.usb_throughput).

A customer takes every SD recording off a Shimmer3R over its USB-C, as a mass
storage drive, so what that drive reads and writes at is a figure a release can
lose without any other item noticing. This writes a file of fixed size to the
unit's drive, reads it back and compares it, then removes it.

Both transfers go around the PC's file cache, or the read would come back from
RAM at gigabytes a second: on Windows the file is opened with
FILE_FLAG_NO_BUFFERING and FILE_FLAG_WRITE_THROUGH, which need transfers in
whole sectors from sector-aligned memory, so the buffer is a page from
VirtualAlloc and every chunk is a whole number of MiB. Elsewhere there is no
equivalent the suite relies on, and the item is not_covered.

Only the time spent in the transfers counts: generating the data and hashing
it are left out, so the figures are the drive's and the link's.

The figures are judged like the Bluetooth ones (rig/throughput.py compare),
against the unit's own `usb_throughput` in rig.toml. They are per unit, not
per build of anything, because the card dominates them: a different card in
the same unit gives different figures.
"""

from __future__ import annotations

import hashlib
import os
import shutil
import sys
import time
from dataclasses import dataclass
from pathlib import Path

USB_THROUGHPUT_KEYS = ("usb_write_bytes_per_s", "usb_read_bytes_per_s")
FILE_NAME = "HILUSBTP.BIN"     # 8.3, in the drive's root, away from the session directories
SIZE = 32 * 1024 * 1024        # long enough to see past the card's write cache; well under any card
CHUNK = 1024 * 1024            # a whole number of sectors for any card
# USB 2.0 Full Speed is 12 Mbit/s, about 1.2 MB/s at best in practice for mass
# storage. A read below this means the unit enumerated at Full Speed, not the
# High Speed DEV-737 made the default, whatever the card could do.
FULL_SPEED_CEILING = 1.3e6


class UsbThroughputError(Exception):
    pass


@dataclass
class Transfer:
    nbytes: int
    seconds: float

    @property
    def bytes_per_s(self) -> float:
        return self.nbytes / self.seconds if self.seconds > 0 else 0.0


@dataclass
class Result:
    write: Transfer
    read: Transfer

    def figures(self) -> dict[str, float]:
        return {"usb_write_bytes_per_s": round(self.write.bytes_per_s),
                "usb_read_bytes_per_s": round(self.read.bytes_per_s)}


def supported() -> bool:
    return sys.platform == "win32"


class _Unbuffered:
    """A file opened around the Windows cache. Transfers must be whole sectors,
    from sector-aligned memory; `buffer` is one page-aligned chunk."""

    GENERIC_READ, GENERIC_WRITE = 0x80000000, 0x40000000
    CREATE_ALWAYS, OPEN_EXISTING = 2, 3
    NO_BUFFERING, WRITE_THROUGH = 0x20000000, 0x80000000
    MEM_COMMIT_RESERVE, MEM_RELEASE, PAGE_READWRITE = 0x3000, 0x8000, 0x04

    def __init__(self, path: Path, write: bool, chunk: int):
        import ctypes
        from ctypes import wintypes as w
        self._c, self._w = ctypes, w
        k = ctypes.WinDLL("kernel32", use_last_error=True)
        k.CreateFileW.restype = w.HANDLE
        k.CreateFileW.argtypes = [w.LPCWSTR, w.DWORD, w.DWORD, w.LPVOID, w.DWORD, w.DWORD, w.HANDLE]
        k.VirtualAlloc.restype = w.LPVOID
        k.VirtualAlloc.argtypes = [w.LPVOID, ctypes.c_size_t, w.DWORD, w.DWORD]
        k.VirtualFree.argtypes = [w.LPVOID, ctypes.c_size_t, w.DWORD]
        k.ReadFile.argtypes = [w.HANDLE, w.LPVOID, w.DWORD, ctypes.POINTER(w.DWORD), w.LPVOID]
        k.WriteFile.argtypes = [w.HANDLE, w.LPVOID, w.DWORD, ctypes.POINTER(w.DWORD), w.LPVOID]
        k.FlushFileBuffers.argtypes = [w.HANDLE]
        k.CloseHandle.argtypes = [w.HANDLE]
        self._k, self.chunk = k, chunk
        flags = self.NO_BUFFERING | (self.WRITE_THROUGH if write else 0)
        self._h = k.CreateFileW(str(path), self.GENERIC_WRITE if write else self.GENERIC_READ, 0, None,
                                self.CREATE_ALWAYS if write else self.OPEN_EXISTING, flags, None)
        if self._h in (None, w.HANDLE(-1).value):
            raise UsbThroughputError(f"{path}: cannot open unbuffered ({ctypes.get_last_error()})")
        self._buf = k.VirtualAlloc(None, chunk, self.MEM_COMMIT_RESERVE, self.PAGE_READWRITE)
        if not self._buf:
            k.CloseHandle(self._h)
            raise UsbThroughputError(f"VirtualAlloc of {chunk} bytes failed ({ctypes.get_last_error()})")

    def write(self, data: bytes) -> None:
        self._c.memmove(self._buf, data, len(data))
        done = self._w.DWORD(0)
        if not self._k.WriteFile(self._h, self._buf, len(data), self._c.byref(done), None) or done.value != len(data):
            raise UsbThroughputError(f"write failed after {done.value} bytes ({self._c.get_last_error()})")

    def read(self, n: int) -> bytes:
        done = self._w.DWORD(0)
        if not self._k.ReadFile(self._h, self._buf, n, self._c.byref(done), None):
            raise UsbThroughputError(f"read failed ({self._c.get_last_error()})")
        return self._c.string_at(self._buf, done.value)

    def flush(self) -> None:
        if not self._k.FlushFileBuffers(self._h):
            raise UsbThroughputError(f"flush failed ({self._c.get_last_error()})")

    def close(self) -> None:
        self._k.CloseHandle(self._h)
        self._k.VirtualFree(self._buf, 0, self.MEM_RELEASE)


class _Buffered:
    """The same interface over an ordinary file. For the hardware-free tests only:
    its figures are the PC's cache, not a drive."""

    def __init__(self, path: Path, write: bool, chunk: int):
        self.chunk = chunk
        self._f = open(path, "wb" if write else "rb")

    def write(self, data: bytes) -> None:
        self._f.write(data)

    def read(self, n: int) -> bytes:
        return self._f.read(n)

    def flush(self) -> None:
        self._f.flush()
        os.fsync(self._f.fileno())

    def close(self) -> None:
        self._f.close()


def measure(root: Path, size: int = SIZE, chunk: int = CHUNK, opener=None, clock=time.perf_counter) -> Result:
    """Write `size` bytes to root/FILE_NAME, read them back and compare, then remove the file.

    The data is random, so neither the card nor anything between can shortcut
    it. Raises UsbThroughputError if the drive is short of space, a transfer
    fails, or what comes back is not what went in. The file is removed on every
    path out, so a failed run leaves nothing on a customer's card.
    """
    if size <= 0 or size % chunk:
        raise ValueError(f"size {size} must be a positive whole number of {chunk}-byte chunks")
    opener = opener or _Unbuffered
    free = shutil.disk_usage(root).free
    if free < 2 * size:
        raise UsbThroughputError(f"{root}: {free} bytes free, {size} needed with room to spare")
    path = Path(root) / FILE_NAME
    sent = hashlib.sha256()
    try:
        f = opener(path, True, chunk)
        spent = 0.0
        try:
            for _ in range(size // chunk):
                data = os.urandom(chunk)
                sent.update(data)
                t0 = clock()
                f.write(data)
                spent += clock() - t0
            t0 = clock()
            f.flush()
            spent += clock() - t0
        finally:
            f.close()
        write = Transfer(size, spent)

        got = hashlib.sha256()
        f = opener(path, False, chunk)
        spent, n = 0.0, 0
        try:
            while n < size:
                t0 = clock()
                data = f.read(chunk)
                spent += clock() - t0
                if not data:
                    break
                got.update(data)
                n += len(data)
        finally:
            f.close()
        if n != size:
            raise UsbThroughputError(f"read back {n} of {size} bytes")
        if got.digest() != sent.digest():
            raise UsbThroughputError(f"the {size} bytes read back differ from those written")
        return Result(write, Transfer(size, spent))
    finally:
        try:
            path.unlink()
        except FileNotFoundError:
            pass


def as_toml(measured: dict[str, float]) -> str:
    """The figures as a rig.toml line, to record them as the unit's baseline."""
    return "usb_throughput = { " + ", ".join(f"{k} = {measured[k]:g}" for k in USB_THROUGHPUT_KEYS) + " }"


def speed_note(measured: dict[str, float]) -> str | None:
    """A read too slow for High Speed points at the link, not the card."""
    if measured["usb_read_bytes_per_s"] < FULL_SPEED_CEILING:
        return (f"usb_read_bytes_per_s {measured['usb_read_bytes_per_s']:g} is within USB Full Speed: "
                "check that the unit enumerated at High Speed (480 Mbit/s)")
    return None
