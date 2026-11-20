"""The one configuration every unit starts a run from (DEV-1463).

The smoke and soak tests change only what they measure (the sensors, the
sampling rate) and keep the rest of a unit's InfoMem as they found it. Without a
baseline, the rest is whatever the unit's last bench session left: a button or
log-trigger mode in byte 232, audio recording or the sound-level channel in byte
233, other ranges, an auto-stop. Each of those changes what a test measures, and
differently on each unit (seen on the v1.01.026 run: the sound level left on one
unit failed both throughput items before they measured anything, and three of
four units recorded audio whenever they logged).

So before its first test, every unit gets the baseline below written over its
dock (conftest.py `_bench_config`, through rig/btmode.py), and its own InfoMem
goes back when the run ends.

The baseline is the firmware's own default configuration
(ShimConfig_setDefaultConfig, sent over Bluetooth as RESET_TO_DEFAULT_CONFIGURATION,
0x5A), as LogAndStream_Shimmer3R v1.01.026 writes it: checked byte for byte against
what that release wrote on a bench unit. It is defined here rather than by sending
0x5A because a run must start from the same bytes whichever release it tests, and
0x5A also renames the unit and resets its Bluetooth baud byte.

Kept from the unit, because they describe the unit rather than configure it:
its calibration blocks, Shimmer name, MAC address, Bluetooth baud and PIN bytes,
the TCXO bit, and the padding that is never transferred.
"""

from __future__ import annotations

import hashlib

from .dock import INFOMEM_SIZE

CONFIG_SIZE = 384                    # bytes 384-511 are struct padding, never transferred (§2)

# (start, end exclusive) ranges copied from the unit. docs/SHIMMER3_CONFIGURATION_INFOMEM.md §2.
KEEP = [
    (30, 31),        # btCommsBaudRate: the radio's, and 0x5A leaves it 0
    (34, 118),       # lnAccel, gyro, mag, wrAccel calibration
    (133, 175),      # altAccel, altMag calibration
    (187, 199),      # shimmerName
    (224, 230),      # macAddr
    (231, 232),      # btSetPin
    (CONFIG_SIZE, INFOMEM_SIZE),
]
KEEP_BITS = [(218, 0x10)]            # tcxo: whether the board has one

# Calibration the firmware rewrites from its own calibration store for the ranges
# configured (ShimCalib_calibDumpToConfigBytesAndSdHeaderAll): not a difference
# when a written baseline reads back. Byte 230 bit 0 (flagWriteCfgToSd) is set by
# the firmware on every configuration write (s3_flash.SELF_SET_BITS).
FIRMWARE_OWNED = [(34, 118), (133, 175)]
SELF_SET_BITS = [(230, 0x01)]


def _shimmer3r_defaults() -> bytes:
    """ShimConfig_setDefaultConfig on Shimmer3R (Configuration/shimmer_config.c), as v1.01.026 writes it."""
    b = bytearray(CONFIG_SIZE)                                 # ShimConfig_createBlankConfigBytes: 0 ...
    b[234:256] = b"\xff" * 22                                  # ... reserved bytes after byte 233: 0xFF
    b[256:384] = b"\xff" * 128                                 # ... sync node table: 0xFF
    b[0:2] = (640).to_bytes(2, "little")                       # samplingRateTicks: 32768 / 640 = 51.2 Hz
    b[2] = 1                                                   # bufferSize
    b[3] = 0xE0                                                # LN accel, gyro, mag
    b[4] = 0x20                                                # VBatt
    b[6] = 0x51                                                # LIS2DW12 100 Hz, +/-2 g, high-performance mode
    b[7] = 0x05                                                # LSM6DSV gyro 60 Hz
    b[8] = 0x0E                                                # LIS2MDL mag 100 Hz; gyro +/-500 dps (LSB)
    b[9] = 0x08                                                # LSM6DSV accel +/-2 g; GSR auto-range; no exp. power
    exg = bytes([0x02, 0x80, 0x10, 0, 0, 0, 0, 0, 0, 0x02])    # ShimConfig_setExgConfigForEcg, both chips
    b[10:20] = exg
    b[20:30] = exg
    b[131] = 0x3E                                              # LIS3MDL mag 80 Hz, ultra-high performance
    b[199:211] = b"DefaultTrial"                               # expIdName; configTime (211-214) 0
    b[217] = 0x31                                              # userButtonEnable, rtcErrorEnable, sdErrorEnable
    b[219] = 54                                                # btIntervalSecs
    b[220:222] = (1).to_bytes(2, "big")                        # estimated experiment length 1 s
    # 222-223 auto-stop off; 232 button and log triggers legacy; 233 no audio, no sound level.
    return bytes(b)


TEMPLATES = {"shimmer3r": _shimmer3r_defaults()}


def name(platform: str) -> str | None:
    """What a report records for the baseline a run used: its platform and a hash of the bytes."""
    t = TEMPLATES.get(platform)
    if t is None:
        return None
    return f"{platform}-defaults-v1.01.026 sha256:{hashlib.sha256(t).hexdigest()[:16]}"


def image(platform: str, unit_image: bytes) -> bytes | None:
    """The baseline for this unit: the template, with the unit's own identity and calibration.
    None for a platform without one (its units keep their own configuration)."""
    t = TEMPLATES.get(platform)
    if t is None:
        return None
    if len(unit_image) != INFOMEM_SIZE:
        raise ValueError(f"InfoMem image is {len(unit_image)} bytes, expected {INFOMEM_SIZE}")
    out = bytearray(t) + bytearray(INFOMEM_SIZE - CONFIG_SIZE)
    for start, end in KEEP:
        out[start:end] = unit_image[start:end]
    for off, mask in KEEP_BITS:
        out[off] = (out[off] & ~mask & 0xFF) | (unit_image[off] & mask)
    return bytes(out)


def differences(written: bytes, back: bytes) -> list[int]:
    """Offsets where a read-back differs from the baseline written, but for what the firmware owns."""
    owned = {i for start, end in FIRMWARE_OWNED for i in range(start, end)}
    out = []
    for i, (a, c) in enumerate(zip(written, back)):
        if i in owned:
            continue
        mask = next((m for off, m in SELF_SET_BITS if off == i), 0)
        if (a & ~mask) != (c & ~mask):
            out.append(i)
    if len(written) != len(back):
        out.append(min(len(written), len(back)))
    return out
