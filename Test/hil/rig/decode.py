"""Turn packets or SD records into per-channel arrays, and kinematic channels
into physical units with the unit's own calibration.

Channel encodings: docs/SHIMMER3_STREAMING_DATA_FORMAT.md §3 and §5.
Calibration block and maths: docs/SHIMMER3_CALIBRATION.md §3 and §7.
"""

from __future__ import annotations

import struct

import numpy as np

from .bt import channel_width

# Kinematic triplets: name -> channel ids (x, y, z), and the 21-byte block for
# each in InfoMem (offset), and whether sensitivity carries the gyro x100.
TRIPLETS = {
    "ln_accel": ((0x00, 0x01, 0x02), 34, 1),
    "gyro": ((0x0A, 0x0B, 0x0C), 55, 100),
    "mag": ((0x07, 0x08, 0x09), 76, 1),
    "wr_accel": ((0x04, 0x05, 0x06), 97, 1),
    "alt_accel": ((0x14, 0x15, 0x16), 133, 1),
    "alt_mag": ((0x17, 0x18, 0x19), 154, 1),
}
VBATT = 0x03
SIGNED_I16 = {c for t in TRIPLETS.values() for c in t[0]}


def split_channels(rows: list[bytes], channels: list[int], hw: int) -> dict[int, np.ndarray]:
    """rows: channel-data bytes of each sample (no header, timestamp or CRC)."""
    out: dict[int, list[int]] = {c: [] for c in channels}
    widths = [channel_width(c, hw) for c in channels]
    for r in rows:
        pos = 0
        for c, w in zip(channels, widths):
            chunk = r[pos:pos + w]
            pos += w
            if w == 2:
                v = struct.unpack("<h" if c in SIGNED_I16 else "<H", chunk)[0]
            elif w == 3:
                v = int.from_bytes(chunk, "little")
            else:
                v = chunk[0]
            out[c].append(v)
    return {c: np.asarray(v, dtype=np.float64) for c, v in out.items()}


class Calib:
    """One 21-byte kinematic block. absent is True for an all-0xFF/0x00 block."""

    def __init__(self, block: bytes, scale: int):
        self.absent = block in (b"\xff" * 21, b"\x00" * 21)
        bias = struct.unpack(">3h", block[0:6])
        sens = struct.unpack(">3h", block[6:12])
        align = struct.unpack("9b", block[12:21])
        self.bias = np.asarray(bias, dtype=np.float64)
        self.sens = np.asarray(sens, dtype=np.float64) / scale
        R = np.asarray(align, dtype=np.float64).reshape(3, 3) / 100
        K = np.diag(self.sens)
        try:
            Rinv = np.linalg.inv(R)
        except np.linalg.LinAlgError:
            Rinv = np.eye(3)                     # §7.1: identity for a degenerate matrix
        try:
            Kinv = np.linalg.inv(K)
        except np.linalg.LinAlgError:
            Kinv = np.eye(3)
        self.M = Rinv @ Kinv

    def apply(self, xyz: np.ndarray) -> np.ndarray:
        """xyz: N x 3 raw counts -> N x 3 physical units."""
        return (self.M @ (xyz - self.bias).T).T


def calibs_from_infomem(image: bytes) -> dict[str, Calib]:
    return {name: Calib(image[off:off + 21], scale) for name, (_, off, scale) in TRIPLETS.items()}


def triplet(chans: dict[int, np.ndarray], name: str) -> np.ndarray | None:
    ids = TRIPLETS[name][0]
    if not all(i in chans for i in ids):
        return None
    return np.column_stack([chans[i] for i in ids])


# GET_ALL_CALIBRATION (0x2C) payload order: LN accel, gyro, mag, WR accel, then on
# Shimmer3R alt accel and alt mag (BT_COMMUNICATION_PROTOCOL §7.6).
BLOB_ORDER = ["ln_accel", "gyro", "mag", "wr_accel", "alt_accel", "alt_mag"]
# SD header offsets (SD_CARD_FORMAT §3.1): the blocks in force when the file was made.
SD_HEADER_OFFSETS = {"wr_accel": 76, "gyro": 97, "mag": 118, "ln_accel": 139}


def calibs_from_blob(blob: bytes) -> dict[str, Calib]:
    """Calibration as a streaming host gets it - for the sensors' current ranges."""
    out = {}
    for i, name in enumerate(BLOB_ORDER):
        block = blob[21 * i:21 * (i + 1)]
        if len(block) == 21:
            out[name] = Calib(block, TRIPLETS[name][2])
    return out


def calibs_from_sd_header(raw: bytes) -> dict[str, Calib]:
    """Calibration as an SD-file parser gets it, from the file's own header."""
    return {name: Calib(raw[off:off + 21], TRIPLETS[name][2]) for name, off in SD_HEADER_OFFSETS.items()}
