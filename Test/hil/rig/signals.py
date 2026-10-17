"""Plausibility checks on data taken with the unit lying still.

These are physics, not specification: a resting unit sees 1 g, no rotation,
a steady field and a battery voltage in range. They catch the failures no
transport check can - a channel decoded at the wrong offset, a sensor left at
a stale ODR (staircase), a wrong range or calibration block, a saturated or
dead axis - because those packets are perfectly well formed.

Every check returns a problem string or nothing; check_rest() gathers them
with a short summary for the report note.
"""

from __future__ import annotations

import numpy as np

from .decode import VBATT, Calib, triplet

G = 9.80665
ACCEL_TOLERANCE = 0.10          # |a| within 10 % of 1 g on the calibrated accelerometers
GYRO_MAX_MEAN_DPS = 5.0         # resting mean per axis
MAX_STAIRCASE_RUN = 8           # identical consecutive raw values on a high-resolution IMU axis
MIN_NOISE_LSB = 2.0             # resting sample-to-sample spread needed before a run means anything
VBATT_MV = (3000, 4300)
I16_LIMITS = (-32768, 32767)


def longest_run(x: np.ndarray) -> int:
    if len(x) < 2:
        return len(x)
    change = np.flatnonzero(np.diff(x) != 0)
    edges = np.concatenate(([-1], change, [len(x) - 1]))
    return int(np.max(np.diff(edges)))


def check_rest(chans: dict[int, np.ndarray], calibs: dict[str, Calib], packet_rate_hz: float,
               mag_ceiling_hz: float = 100.0) -> tuple[list[str], str]:
    problems: list[str] = []
    notes: list[str] = []

    for name in ("ln_accel", "wr_accel", "alt_accel"):
        raw = triplet(chans, name)
        if raw is None:
            continue
        c = calibs.get(name)
        if name == "alt_accel" and (c is None or c.absent or np.all(np.abs(c.sens) <= 1)):
            # ADXL371 ships with a placeholder calibration (CALIBRATION §6.2):
            # its physical values mean nothing until a real one is written.
            notes.append("alt accel uncalibrated, magnitude not checked")
        elif c is None or c.absent:
            notes.append(f"{name}: no calibration stored, magnitude not checked")
        else:
            mag = float(np.median(np.linalg.norm(c.apply(raw), axis=1)))
            notes.append(f"{name} |a| {mag:.2f}")
            if abs(mag / G - 1) > ACCEL_TOLERANCE:
                problems.append(f"{name}: resting |a| {mag:.2f} m/s^2, expected {G:.2f} ±{ACCEL_TOLERANCE:.0%}")

    raw = triplet(chans, "gyro")
    if raw is not None:
        c = calibs.get("gyro")
        if c is None or c.absent:
            notes.append("gyro: no calibration stored, mean not checked")
        else:
            mean = np.mean(c.apply(raw), axis=0)
            worst = float(np.max(np.abs(mean)))
            notes.append(f"gyro max |mean| {worst:.2f} dps")
            if worst > GYRO_MAX_MEAN_DPS:
                problems.append(f"gyro: resting mean {np.round(mean, 2).tolist()} dps, limit ±{GYRO_MAX_MEAN_DPS}")

    # Staircase: a sensor running slower than the packet rate repeats readings
    # with fresh timestamps (INFOMEM §10.6). The LSM6DSV is fast and fine
    # enough that a resting axis changes every sample or nearly so.
    # Only on an axis whose resting noise spans a few counts: at a coarse range
    # (±16 g, ±1000 dps) a still axis legitimately repeats its value, and a run
    # there says nothing about the ODR.
    for name in ("ln_accel", "gyro"):
        raw = triplet(chans, name)
        if raw is None:
            continue
        runs = [longest_run(raw[:, i]) for i in range(3) if np.std(np.diff(raw[:, i])) >= MIN_NOISE_LSB]
        if runs and max(runs) > MAX_STAIRCASE_RUN:
            problems.append(f"{name}: {max(runs)} identical consecutive samples - ODR below the packet rate?")
    raw = triplet(chans, "mag")
    if raw is not None:
        # The LIS2MDL tops out at 100 Hz, so above that repeats are expected.
        allowed = MAX_STAIRCASE_RUN * max(1.0, packet_rate_hz / mag_ceiling_hz)
        run = max(longest_run(raw[:, i]) for i in range(3))
        if run > allowed:
            problems.append(f"mag: {run} identical consecutive samples (allowed {allowed:.0f})")

    # Dead or saturated axes on the signed 16-bit IMU channels.
    for name in ("ln_accel", "gyro", "mag", "wr_accel", "alt_accel", "alt_mag"):
        raw = triplet(chans, name)
        if raw is None:
            continue
        for i, axis in enumerate("xyz"):
            col = raw[:, i]
            if len(col) > 20 and np.all(col == col[0]):
                problems.append(f"{name} {axis}: constant {int(col[0])} for the whole capture")
            sat = np.mean((col <= I16_LIMITS[0]) | (col >= I16_LIMITS[1]))
            if sat > 0:
                problems.append(f"{name} {axis}: {sat:.1%} of samples at the int16 limit")

    if VBATT in chans:
        mv = float(np.median(chans[VBATT])) * 3000 / 4095 * 2
        notes.append(f"VBatt {mv:.0f} mV")
        if not VBATT_MV[0] <= mv <= VBATT_MV[1]:
            problems.append(f"VBatt {mv:.0f} mV outside {VBATT_MV[0]}-{VBATT_MV[1]} mV")

    for cid in (0x1B,):                          # pressure: a live sensor is never perfectly flat
        if cid in chans and len(chans[cid]) > 20 and np.all(chans[cid] == chans[cid][0]):
            problems.append("pressure: raw value constant for the whole capture")

    return problems, ", ".join(notes)
