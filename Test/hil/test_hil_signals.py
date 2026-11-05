#!/usr/bin/env python3
"""Unit tests for rig.signals: the stale-sensor (staircase) checks on resting
data, and rig.decode's byte order for the alt accel. Run: python Test/hil/test_hil_signals.py (no hardware).

The cases include the one that failed a good build: a resting gyro whose axes
each repeat their last value about one sample in five, over a five-minute log at
512 Hz. Judged one axis at a time, nine repeats in a row turn up there by chance;
a sensor that has really stopped updating repeats all three axes together.

Needs numpy, as rig.signals does; without it every case is skipped.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    import numpy as np
    from rig import bt, decode, signals
except ImportError:  # pragma: no cover - CI without numpy
    np = None

GYRO_IDS = (0x0A, 0x0B, 0x0C)


def resting(n, seed, spread=3):
    """n samples of a still IMU axis triplet: noise of +-spread counts."""
    rng = np.random.default_rng(seed)
    return rng.integers(-spread, spread + 1, size=(n, 3)) + np.array([12, -40, 7])


def chans_for(raw, ids=GYRO_IDS):
    return {cid: raw[:, i] for i, cid in enumerate(ids)}


@unittest.skipIf(np is None, "numpy not installed")
class Staircase(unittest.TestCase):
    def problems(self, raw, rate=512.0, ids=GYRO_IDS):
        return signals.check_rest(chans_for(raw, ids), {}, rate)[0]

    def test_one_axis_repeating_by_chance_is_not_a_staircase(self):
        raw = resting(154_000, seed=1)
        # Nine identical values on one axis only, the bench failure.
        raw[50_000:50_009, 0] = raw[50_000, 0]
        self.assertGreater(signals.longest_run(raw[:, 0]), signals.MAX_STAIRCASE_RUN)
        self.assertEqual([p for p in self.problems(raw) if "identical" in p], [])
        # The same data is judged, not skipped: freeze the whole sample there.
        raw[50_000:50_009] = raw[50_000]
        self.assertTrue(any("gyro: 9 identical" in p for p in self.problems(raw)))

    def test_five_minutes_of_rest_never_trips_it(self):
        for seed in range(5):
            with self.subTest(seed=seed):
                raw = resting(154_000, seed=seed)
                self.assertEqual([p for p in self.problems(raw) if "identical" in p], [])

    def test_a_sensor_that_stops_updating_is_caught(self):
        raw = resting(20_000, seed=2)
        raw[10_000:10_012] = raw[10_000]          # whole sample frozen for 12 packets
        self.assertTrue(any("gyro: 12 identical" in p for p in self.problems(raw)))

    def test_an_odr_far_below_the_packet_rate_is_caught(self):
        fresh = resting(2_000, seed=3)
        raw = np.repeat(fresh, 10, axis=0)          # a new sample every tenth packet
        runs = [p for p in self.problems(raw) if "gyro:" in p and "identical" in p]
        self.assertEqual(len(runs), 1)
        self.assertGreaterEqual(int(runs[0].split()[1]), 10)

    def test_a_coarse_range_with_one_noisy_axis_is_not_judged(self):
        raw = resting(20_000, seed=4)
        raw[:, 1] = 3                                # two still axes
        raw[:, 2] = -1
        raw[10_000:10_012] = raw[10_000]
        self.assertEqual([p for p in self.problems(raw) if "identical" in p], [])

    def test_mag_judges_whole_samples_too(self):
        mag_ids = (0x07, 0x08, 0x09)
        raw = resting(20_000, seed=5)
        raw[:, 2] = 0                                # one still axis repeats for ever
        self.assertEqual([p for p in self.problems(raw, rate=100.0, ids=mag_ids) if "mag:" in p], [])
        raw[5_000:5_020] = raw[5_000]                # a frozen magnetometer
        self.assertTrue(any("mag: 20 identical" in p
                            for p in self.problems(raw, rate=100.0, ids=mag_ids)))


    def test_an_empty_or_one_record_capture_does_not_crash(self):
        for n in (0, 1, 2):
            with self.subTest(records=n):
                raw = resting(n, seed=6) if n else np.zeros((0, 3), dtype=int)
                self.assertEqual([p for p in self.problems(raw) if "identical" in p], [])


@unittest.skipIf(np is None, "numpy not installed")
class LongestRun(unittest.TestCase):
    def test_one_dimensional(self):
        self.assertEqual(signals.longest_run(np.array([1, 1, 2, 2, 2, 3])), 3)

    def test_rows(self):
        x = np.array([[1, 2], [1, 2], [1, 3], [1, 3], [1, 3]])
        self.assertEqual(signals.longest_run(x), 3)
        self.assertEqual(signals.longest_run(x[:, 0]), 5)


ALT_IDS = (0x14, 0x15, 0x16)


def alt_row(counts12):
    """One sample of the three alt accel channels as the Shimmer3R sends it:
    each 12-bit count left-justified in a big-endian word."""
    return b"".join(((c & 0xFFF) << 4).to_bytes(2, "big") for c in counts12)


@unittest.skipIf(np is None, "numpy not installed")
class AltAccelByteOrder(unittest.TestCase):
    # The v1.01.023 release run: a resting ADXL371 (100 mg/LSB) read
    # little-endian landed on 24576 / -32768 / 8192 and failed every full_* row
    # as "at the int16 limit".
    REST = [(1, -5, -10), (0, -6, -10), (2, -5, -11), (-1, -4, -10)]

    def test_shimmer3r_decodes_12_bit_counts_with_sign(self):
        rows = [alt_row(r) for r in self.REST]
        chans = decode.split_channels(rows, list(ALT_IDS), bt.HW_SHIMMER3R)
        got = np.column_stack([chans[c] for c in ALT_IDS]).astype(int).tolist()
        self.assertEqual(got, [list(r) for r in self.REST])

    def test_shimmer3_decodes_a_big_endian_int16(self):
        rows = [(-300).to_bytes(2, "big", signed=True) * 3]
        chans = decode.split_channels(rows, list(ALT_IDS), bt.HW_SHIMMER3)
        self.assertEqual([chans[c][0] for c in ALT_IDS], [-300] * 3)

    def test_resting_alt_accel_is_not_saturated(self):
        rows = [alt_row(self.REST[i % 4]) for i in range(200)]
        chans = decode.split_channels(rows, list(ALT_IDS), bt.HW_SHIMMER3R)
        probs = signals.check_rest(chans, {}, 512.0)[0]
        self.assertEqual([p for p in probs if "limit" in p], [])

    def test_a_saturated_alt_accel_is_caught_at_12_bits(self):
        rows = [alt_row((2047, -2048, 0)), alt_row((2047, -2048, 1))] * 50
        chans = decode.split_channels(rows, list(ALT_IDS), bt.HW_SHIMMER3R)
        probs = signals.check_rest(chans, {}, 512.0)[0]
        self.assertIn("alt_accel x: 100.0% of samples at the int12 limit", probs)
        self.assertIn("alt_accel y: 100.0% of samples at the int12 limit", probs)

    def test_on_shimmer3_the_alt_accel_saturates_at_16_bits(self):
        # The 9-axis chip's int16: a reading past 2047 is a reading, not a limit.
        rows = [(3000).to_bytes(2, "big", signed=True) * 3] * 100
        chans = decode.split_channels(rows, list(ALT_IDS), bt.HW_SHIMMER3)
        probs = signals.check_rest(chans, {}, 512.0, hw=bt.HW_SHIMMER3)[0]
        self.assertEqual([p for p in probs if "limit" in p], [])


def le(*vals):
    return b"".join(v.to_bytes(2, "little", signed=True) for v in vals)


def be(*vals):
    return b"".join(v.to_bytes(2, "big", signed=True) for v in vals)


@unittest.skipIf(np is None, "numpy not installed")
class Shimmer3ByteOrder(unittest.TestCase):
    """STREAMING_DATA_FORMAT §5.2: a Shimmer3 streams each part's registers
    unswapped, so the gyro is big-endian on every board and the mag on an
    LSM303DLHC board, which also lists it X, Z, Y (§4.1)."""

    def test_the_gyro_is_big_endian(self):
        chans = decode.split_channels([be(-300, 5, 1200)], list(GYRO_IDS), bt.HW_SHIMMER3)
        self.assertEqual([chans[c][0] for c in GYRO_IDS], [-300, 5, 1200])

    def test_the_gyro_is_little_endian_on_shimmer3r(self):
        chans = decode.split_channels([le(-300, 5, 1200)], list(GYRO_IDS), bt.HW_SHIMMER3R)
        self.assertEqual([chans[c][0] for c in GYRO_IDS], [-300, 5, 1200])

    def test_an_lsm303dlhc_mag_is_big_endian_in_x_z_y_order(self):
        order = [0x07, 0x09, 0x08]
        chans = decode.split_channels([be(-250, 400, 90)], order, bt.HW_SHIMMER3)
        self.assertEqual((chans[0x07][0], chans[0x08][0], chans[0x09][0]), (-250, 90, 400))

    def test_an_lsm303ah_or_icm_mag_is_little_endian(self):
        order = [0x07, 0x08, 0x09]
        chans = decode.split_channels([le(-250, 90, 400)], order, bt.HW_SHIMMER3)
        self.assertEqual((chans[0x07][0], chans[0x08][0], chans[0x09][0]), (-250, 90, 400))

    def test_bmp_temperature_and_pressure_are_big_endian(self):
        row = (25000).to_bytes(2, "big") + (300000).to_bytes(3, "big")
        chans = decode.split_channels([row], [0x1A, 0x1B], bt.HW_SHIMMER3)
        self.assertEqual((chans[0x1A][0], chans[0x1B][0]), (25000, 300000))

    def test_the_wide_range_accel_stays_little_endian(self):
        ids = [0x04, 0x05, 0x06]
        chans = decode.split_channels([le(-16000, 30, 700)], ids, bt.HW_SHIMMER3)
        self.assertEqual([chans[c][0] for c in ids], [-16000, 30, 700])


@unittest.skipIf(np is None, "numpy not installed")
class DefaultCalibration(unittest.TestCase):
    """A firmware default seed carries no offset: a resting gyro read through one
    can sit well off zero, and it is not this unit's calibration to judge."""

    BLOCK = bytes(6) + (6550).to_bytes(2, "big") * 3 + bytes([100, 0, 0, 0, 100, 0, 0, 0, 100])

    def chans(self):
        raw = resting(500, seed=7) + np.array([1500, 0, 0])      # ~23 dps off on x at 65.5 LSB/dps
        return chans_for(raw)

    def test_a_gyro_offset_fails_with_a_calibration(self):
        calibs = {"gyro": decode.Calib(self.BLOCK, 100)}
        probs, _ = signals.check_rest(self.chans(), calibs, 51.2, hw=bt.HW_SHIMMER3)
        self.assertTrue(any(p.startswith("gyro: resting mean") for p in probs))

    def test_with_a_default_it_is_noted_not_judged(self):
        calibs = {"gyro": decode.Calib(self.BLOCK, 100)}
        probs, note = signals.check_rest(self.chans(), calibs, 51.2, hw=bt.HW_SHIMMER3, defaults={"gyro"})
        self.assertFalse(any(p.startswith("gyro: resting mean") for p in probs))
        self.assertIn("gyro: default calibration, mean not checked", note)


if __name__ == "__main__":
    unittest.main(verbosity=2)
