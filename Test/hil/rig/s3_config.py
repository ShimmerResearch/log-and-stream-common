"""Build and read Shimmer3 configurations, as a host writes them over Bluetooth.

The fields the release matrix varies: sampling rate, sensor enables, the
ranges, and the output rates (ODRs) that go with them. Everything else in the
image - trial config, names, sync table - is carried through from what the unit
already holds. Byte and bit positions are those of
docs/SHIMMER3_CONFIGURATION_INFOMEM.md §2-§4, Shimmer3 column.

Unlike rig/s3r_config.py, this module sets the ODRs itself. Shimmer3 firmware
does not raise an ODR to cover the packet rate (the correction in INFOMEM §10.6
is Shimmer3R-only, Configuration/shimmer_config.c), so keeping that invariant
is the host's job, as Consensys does. The codes here are chosen to cover the
rate whichever part the board carries, because the part is not something a
host can read (STREAMING_DATA_FORMAT §5.2): an LSM303DLHC on first-generation
boards, an LSM303AHTR on later ones, and the ICM-20948 standing in for both on
"no LSM" boards, where the firmware ignores these fields.

The calibration dump matters here too. A Bluetooth InfoMem write at offset 0
copies InfoMem's four calibration blocks into the calibration dump under the
ranges the image sets (BT_COMMUNICATION_PROTOCOL §7.5), so an image whose
ranges changed but whose blocks did not would overwrite the unit's calibration
for the new ranges with the old ranges' values. with_calibration() puts in the
dump's own block for each range first, as Consensys does.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

TICKS_PER_S = 32768

# name -> (byte, bit). Shimmer3 meanings (INFOMEM §2); Shimmer3R differs.
SENSOR_BITS = {
    "ln_accel": (3, 7), "gyro": (3, 6), "mag": (3, 5), "exg1_24": (3, 4), "exg2_24": (3, 3),
    "gsr": (3, 2), "ext_adc7": (3, 1), "ext_adc6": (3, 0),
    "bridge_amp": (4, 7), "vbatt": (4, 5), "wr_accel": (4, 4), "ext_adc15": (4, 3),
    "int_adc1": (4, 2), "int_adc12": (4, 1), "int_adc13": (4, 0),
    "int_adc14": (5, 7), "alt_accel": (5, 6), "alt_mag": (5, 5), "exg1_16": (5, 4), "exg2_16": (5, 3),
    "pressure": (5, 2),
}

# The channels each sensor adds to the packet, in the firmware's order
# (STREAMING_DATA_FORMAT §4.1): the MCU's ADC block first, then I2C. Only the
# internal sensors are here; the expansion-board channels depend on the board.
ADC_ORDER = [("ln_accel", (0x00, 0x01, 0x02)), ("vbatt", (0x03,))]
I2C_ORDER = [("gyro", (0x0A, 0x0B, 0x0C)), ("wr_accel", (0x04, 0x05, 0x06)), ("mag", (0x07, 0x08, 0x09)),
             ("alt_accel", (0x14, 0x15, 0x16)), ("alt_mag", (0x17, 0x18, 0x19)), ("pressure", (0x1A, 0x1B))]
STREAMABLE = {name for name, _ in ADC_ORDER + I2C_ORDER}
# On an LSM303DLHC board the mag goes X, Z, Y (§4.1); elsewhere X, Y, Z.
MAG_DLHC = (0x07, 0x09, 0x08)

# Calibration (CALIBRATION §2.3, §4.1): InfoMem offset of each 21-byte block,
# and its id in the calibration dump.
CALIB_BLOCKS = {"ln_accel": (34, 2), "gyro": (55, 30), "mag": (76, 32), "wr_accel": (97, 31)}
BLOCK_LEN = 21

# Wide-range accel ODR codes, normal mode (byte 6 bits 7-4): the rate each
# delivers on an LSM303DLHC and on an LSM303AHTR (Shimmer_Driver/LSM303*).
# Code 8 is low-power-only on the DLHC, so it is never chosen.
WR_ACCEL_CODES = [(5, 100, 200), (6, 200, 400), (7, 400, 800), (9, 1344, 3200)]
# Mag ODR codes (byte 8 bits 4-2): 6 is 75 Hz on an LSM303DLHC and 50 Hz on an
# LSM303AHTR (low power); 7 is 220 Hz and 100 Hz. SET_MAG_SAMPLING_RATE stores
# 7 as 6 (ShimConfig_configByteMagRateSet), so only an InfoMem write gives 7.
MAG_FAST, MAG_SLOW = 7, 6
MAG_FLOOR_HZ = 50.0             # the slowest of those: the rate a staircase check can assume
MAG_SLOW_FOR_HZ = 75.0          # code 6 covers this on a DLHC; above it, 7
GYRO_BASE_HZ = 8000.0           # MPU9x50 SMPLRT_DIV with the DLPF off; the ICM-20948 converts it


def divider_for(rate_hz: float) -> int:
    return max(1, round(TICKS_PER_S / rate_hz))


def odr_codes(rate_hz: float, *, via_commands: bool = False) -> dict[str, int]:
    """The ODR codes a host writes for a packet rate, on any Shimmer3 board.

    The wide-range accel gets the slowest normal-mode code that covers the rate
    on both LSM303 parts, the gyro the largest MPU9x50 divider that does, and
    the mag code 6 up to 75 Hz and 7 above. Through SET_MAG_SAMPLING_RATE the
    mag stays at 6, the most that command stores.
    """
    wr = next((code for code, dlhc, _ in WR_ACCEL_CODES if dlhc >= rate_hz), WR_ACCEL_CODES[-1][0])
    gyro = min(255, max(0, int(GYRO_BASE_HZ / rate_hz) - 1))
    mag = MAG_SLOW if rate_hz <= MAG_SLOW_FOR_HZ or via_commands else MAG_FAST
    return {"wr_accel_rate": wr, "gyro_rate": gyro, "mag_rate": mag, "wr_accel_lp_mode": 0}


def expected_channels(sensors: set[str], dlhc: bool) -> list[int]:
    out: list[int] = []
    for name, ids in ADC_ORDER + I2C_ORDER:
        if name in sensors:
            out += MAG_DLHC if (name == "mag" and dlhc) else ids
    return out


def packet_bytes(sensors: set[str], crc: int = 0) -> int:
    """Bytes per data packet: the 0x00, a 3-byte stamp, the channels, and the CRC."""
    from .bt import HW_SHIMMER3, channel_width
    return 4 + sum(channel_width(c, HW_SHIMMER3) for c in expected_channels(sensors, False)) + crc


# --- the calibration dump ------------------------------------------------------

@dataclass
class CalibRecord:
    ts: bytes
    data: bytes

    @property
    def is_default(self) -> bool:
        """An all-zero timestamp is a firmware default seed, never a calibration (CALIBRATION §2.2)."""
        return not any(self.ts)


def calib_records(blob: bytes) -> dict[tuple[int, int], CalibRecord]:
    """(sensor id, range) -> record, walked as the firmware walks it (CALIBRATION §2.2)."""
    end = min(len(blob), int.from_bytes(blob[:2], "little") + 2)
    out, at = {}, 10
    while at + 12 <= end:
        sid, rng, n = struct.unpack_from("<HBB", blob, at)
        out.setdefault((sid, rng), CalibRecord(bytes(blob[at + 4:at + 12]), bytes(blob[at + 12:at + 12 + n])))
        at += 12 + n
    return out


class S3Config:
    """A Shimmer3 configuration image: 384 bytes as read over Bluetooth, or 512 over the dock."""

    def __init__(self, image: bytes):
        if len(image) < 384:
            raise ValueError(f"InfoMem image is {len(image)} bytes, expected at least 384")
        self.b = bytearray(image)

    def bytes(self) -> bytes:
        return bytes(self.b)

    # --- rate and sensors ----------------------------------------------------

    @property
    def divider(self) -> int:
        return self.b[0] | (self.b[1] << 8)

    @divider.setter
    def divider(self, ticks: int):
        self.b[0], self.b[1] = ticks & 0xFF, (ticks >> 8) & 0xFF

    @property
    def rate_hz(self) -> float:
        return TICKS_PER_S / self.divider

    def enabled(self) -> set[str]:
        return {n for n, (byte, bit) in SENSOR_BITS.items() if self.b[byte] & (1 << bit)}

    def set_sensors(self, names: set[str]):
        unknown = set(names) - SENSOR_BITS.keys()
        if unknown:
            raise ValueError(f"unknown sensors: {sorted(unknown)}")
        for byte in (3, 4, 5):
            self.b[byte] = 0
        for n in names:
            byte, bit = SENSOR_BITS[n]
            self.b[byte] |= 1 << bit

    def sensor_bytes(self) -> bytes:
        """InfoMem 3-5, as SET_SENSORS carries them."""
        return bytes(self.b[3:6])

    # --- setup bytes 6-9 (INFOMEM §4) ----------------------------------------

    def _field(self, byte: int, shift: int, width: int) -> int:
        return (self.b[byte] >> shift) & ((1 << width) - 1)

    def _set(self, byte: int, shift: int, width: int, value: int):
        mask = ((1 << width) - 1) << shift
        self.b[byte] = (self.b[byte] & ~mask & 0xFF) | ((value << shift) & mask)

    # name -> (byte, shift, width)
    FIELDS = {
        "wr_accel_rate": (6, 4, 4), "wr_accel_range": (6, 2, 2), "wr_accel_lp_mode": (6, 1, 1),
        "wr_accel_hr_mode": (6, 0, 1),
        "gyro_rate": (7, 0, 8),
        "mag_range": (8, 5, 3), "mag_rate": (8, 2, 3), "gyro_range": (8, 0, 2),
        "alt_accel_range": (9, 6, 2),
    }

    def get(self, name: str) -> int:
        return self._field(*self.FIELDS[name])

    def put(self, name: str, value: int):
        self._set(*self.FIELDS[name], value)

    @property
    def setup_bytes(self) -> bytes:
        """InfoMem 6-9, which the Shimmer3 inquiry response carries (BT_COMMUNICATION_PROTOCOL §7.1)."""
        return bytes(self.b[6:10])

    # --- calibration ---------------------------------------------------------

    def calib_key(self, sensor: str) -> tuple[int, int]:
        """(dump id, range) of the calibration block in force for `sensor`."""
        rng = {"ln_accel": 0, "gyro": self.get("gyro_range"), "mag": self.get("mag_range"),
               "wr_accel": self.get("wr_accel_range")}[sensor]
        return CALIB_BLOCKS[sensor][1], rng

    def with_calibration(self, records: dict[tuple[int, int], CalibRecord]) -> list[str]:
        """Put the dump's block for each sensor's range into InfoMem; return the sensors it had none for.

        A range the dump holds no block for reads back as zeros through
        GET_ALL_CALIBRATION (CALIBRATION §8.2), so zeros go in: the write then
        stores what the firmware would report anyway.
        """
        missing = []
        for sensor, (offset, _) in CALIB_BLOCKS.items():
            rec = records.get(self.calib_key(sensor))
            if rec is None or len(rec.data) != BLOCK_LEN:
                missing.append(sensor)
                block = bytes(BLOCK_LEN)
            else:
                block = rec.data
            self.b[offset:offset + BLOCK_LEN] = block
        return missing

    def default_calibrations(self, records: dict[tuple[int, int], CalibRecord]) -> set[str]:
        """Sensors whose block for the current range is a firmware default, or absent."""
        out = set()
        for sensor in CALIB_BLOCKS:
            rec = records.get(self.calib_key(sensor))
            if rec is None or rec.is_default:
                out.add(sensor)
        return out
