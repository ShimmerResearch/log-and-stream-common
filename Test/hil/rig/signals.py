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
MAX_STAIRCASE_RUN = 8           # identical consecutive raw samples (all three axes) on a high-resolution IMU
MIN_NOISE_LSB = 2.0             # resting sample-to-sample spread needed before a run means anything
MIN_NOISY_AXES = 2              # axes that must carry that noise before a whole-sample run means anything
VBATT_MV = (3000, 4300)
I16_LIMITS = (-32768, 32767)
# Shimmer3R's alt accel decodes to 12-bit counts (decode.split_channels), so it
# saturates at the 12-bit limits. check_rest is used on Shimmer3R data only.
LIMITS = {"alt_accel": (-2048, 2047)}


def longest_run(x: np.ndarray) -> int:
    """Longest run of identical consecutive values; for a 2-D array, of
    identical consecutive rows."""
    if len(x) < 2:
        return len(x)
    d = np.diff(x, axis=0)
    change = np.flatnonzero(np.any(d != 0, axis=1) if d.ndim > 1 else d != 0)
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
    # A stale sensor repeats its whole sample, so the run is of whole samples,
    # all three axes at once. One axis alone is not evidence: at resting noise
    # of a couple of counts it repeats its last value about one sample in five,
    # so nine in a row turns up by chance about once in a five-minute 512 Hz
    # log - and did, failing a good build. Three axes repeating together eight
    # times over by chance is out of reach.
    # Only with at least two axes whose resting noise spans a few counts: at a
    # coarse range (±16 g, ±1000 dps) a still axis legitimately repeats its
    # value, and a run there says nothing about the ODR.
    for name in ("ln_accel", "gyro"):
        raw = triplet(chans, name)
        # Too short to have steps to judge: an empty or one-record capture is
        # the record-count check's to report, not this one's to crash on.
        if raw is None or len(raw) < 2:
            continue
        # The noise is measured between samples that changed: a staircase
        # repeats most samples, and the zero steps between them would make the
        # sensor look quiet - the very case this check is for.
        fresh = raw[np.concatenate(([True], np.any(np.diff(raw, axis=0) != 0, axis=1)))]
        if len(fresh) < 3:
            continue                                     # no steps between changes: nothing to judge
        noisy = sum(np.std(np.diff(fresh[:, i])) >= MIN_NOISE_LSB for i in range(3))
        if noisy < MIN_NOISY_AXES:
            continue
        run = longest_run(raw)
        if run > MAX_STAIRCASE_RUN:
            problems.append(f"{name}: {run} identical consecutive samples - ODR below the packet rate?")
    raw = triplet(chans, "mag")
    if raw is not None:
        # The LIS2MDL tops out at 100 Hz, so above that repeats are expected.
        # Whole samples, as above: a still axis repeats on its own.
        allowed = MAX_STAIRCASE_RUN * max(1.0, packet_rate_hz / mag_ceiling_hz)
        run = longest_run(raw)
        if run > allowed:
            problems.append(f"mag: {run} identical consecutive samples (allowed {allowed:.0f})")

    # Dead or saturated axes on the signed IMU channels.
    for name in ("ln_accel", "gyro", "mag", "wr_accel", "alt_accel", "alt_mag"):
        raw = triplet(chans, name)
        if raw is None:
            continue
        for i, axis in enumerate("xyz"):
            col = raw[:, i]
            if len(col) > 20 and np.all(col == col[0]):
                problems.append(f"{name} {axis}: constant {int(col[0])} for the whole capture")
            lo, hi = LIMITS.get(name, I16_LIMITS)
            sat = np.mean((col <= lo) | (col >= hi))
            if sat > 0:
                width = "int12" if name in LIMITS else "int16"
                problems.append(f"{name} {axis}: {sat:.1%} of samples at the {width} limit")

    if VBATT in chans:
        mv = float(np.median(chans[VBATT])) * 3000 / 4095 * 2
        notes.append(f"VBatt {mv:.0f} mV")
        if not VBATT_MV[0] <= mv <= VBATT_MV[1]:
            problems.append(f"VBatt {mv:.0f} mV outside {VBATT_MV[0]}-{VBATT_MV[1]} mV")

    for cid in (0x1B,):                          # pressure: a live sensor is never perfectly flat
        if cid in chans and len(chans[cid]) > 20 and np.all(chans[cid] == chans[cid][0]):
            problems.append("pressure: raw value constant for the whole capture")

    return problems, ", ".join(notes)
