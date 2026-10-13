"""Build and read Shimmer3R configurations in the 512-byte InfoMem image.

Only the fields the release matrix varies: sampling rate, sensor enables and
the ranges of the kinematic sensors. Everything else in the image - trial
config, names, calibration blocks - is carried through from what the unit
already holds. Byte and bit positions are those of
docs/SHIMMER3_CONFIGURATION_INFOMEM.md §2-§4, Shimmer3R column.

ODRs are deliberately left alone. The firmware's checkAndCorrectConfig raises
any enabled sensor's output rate to cover the packet rate (§10.6), and the
release tier checks it did - writing them here would hide that.
"""

from __future__ import annotations

# name -> (byte, bit). Shimmer3R meanings; Shimmer3 differs (byte map §2).
SENSOR_BITS = {
    "ln_accel": (3, 7), "gyro": (3, 6), "mag": (3, 5), "exg1_24": (3, 4), "exg2_24": (3, 3),
    "gsr": (3, 2), "ext_adc0": (3, 1), "ext_adc1": (3, 0),
    "bridge_amp": (4, 7), "mic": (4, 6), "vbatt": (4, 5), "wr_accel": (4, 4), "ext_adc2": (4, 3),
    "int_adc3": (4, 2), "int_adc0": (4, 1), "int_adc1": (4, 0),
    "int_adc2": (5, 7), "alt_accel": (5, 6), "alt_mag": (5, 5), "exg1_16": (5, 4), "exg2_16": (5, 3),
    "pressure": (5, 2),
}

# ODR field decoders and the rate each code delivers, Shimmer3R (§10.6 table).
GYRO_ODR_HZ = [0, 1.875, 7.5, 15, 30, 60, 120, 240, 480, 960, 1920, 3840, 7680]   # byte 7
WR_ACCEL_ODR_HZ = [0, 1.6, 12.5, 25, 50, 100, 200, 400, 800, 1600]                # byte 6 bits 7-4
MAG_ODR_HZ = [10, 20, 50, 100]                                                     # byte 8 bits 4-2
ALT_ACCEL_ODR_HZ = [320, 640, 1280, 2560]                                          # byte 130 bits 7-6

TICKS_PER_S = 32768


def divider_for(rate_hz: float) -> int:
    return max(1, round(TICKS_PER_S / rate_hz))


class S3rConfig:
    def __init__(self, image: bytes):
        if len(image) != 512:
            raise ValueError(f"InfoMem image is {len(image)} bytes, expected 512")
        self.b = bytearray(image)

    def bytes(self) -> bytes:
        return bytes(self.b)

    # --- rate ----------------------------------------------------------------

    @property
    def divider(self) -> int:
        return self.b[0] | (self.b[1] << 8)

    @divider.setter
    def divider(self, ticks: int):
        self.b[0], self.b[1] = ticks & 0xFF, (ticks >> 8) & 0xFF

    @property
    def rate_hz(self) -> float:
        return TICKS_PER_S / self.divider

    # --- sensors -------------------------------------------------------------

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

    # --- ranges --------------------------------------------------------------

    @property
    def ln_accel_range(self) -> int:            # byte 9 bits 7-6: 0..3 = ±2/4/8/16 g
        return (self.b[9] >> 6) & 0x3

    @ln_accel_range.setter
    def ln_accel_range(self, code: int):
        self.b[9] = (self.b[9] & 0x3F) | ((code & 0x3) << 6)

    @property
    def wr_accel_range(self) -> int:            # byte 6 bits 3-2
        return (self.b[6] >> 2) & 0x3

    @wr_accel_range.setter
    def wr_accel_range(self, code: int):
        self.b[6] = (self.b[6] & 0xF3) | ((code & 0x3) << 2)

    @property
    def gyro_range(self) -> int:                # byte 8 bits 1-0, MSB in byte 130 bit 2 (§4.1)
        return (self.b[8] & 0x3) | (((self.b[130] >> 2) & 1) << 2)

    @gyro_range.setter
    def gyro_range(self, code: int):
        self.b[8] = (self.b[8] & 0xFC) | (code & 0x3)
        self.b[130] = (self.b[130] & 0xFB) | (((code >> 2) & 1) << 2)

    # --- ODRs, for checking the firmware's correction ------------------------

    def odrs_hz(self) -> dict[str, float]:
        """Output rate of each enabled kinematic sensor whose ODR is configurable."""
        en = self.enabled()
        out = {}
        if en & {"ln_accel", "gyro"}:
            code = self.b[7]
            out["lsm6dsv"] = GYRO_ODR_HZ[code] if code < len(GYRO_ODR_HZ) else float("nan")
        if "wr_accel" in en:
            code = self.b[6] >> 4
            hz = WR_ACCEL_ODR_HZ[code] if code < len(WR_ACCEL_ODR_HZ) else float("nan")
            # Low-power mode (wrAccelHrMode, byte 6 bit 0, clear) delivers at most
            # 200 Hz whatever the code says (§10.6).
            out["lis2dw12"] = hz if self.wr_accel_hr_mode else min(hz, 200)
        if "mag" in en:
            out["lis2mdl"] = MAG_ODR_HZ[(self.b[8] >> 2) & 0x3] if ((self.b[8] >> 2) & 0x7) < 4 else float("nan")
        if "alt_accel" in en:
            out["adxl371"] = ALT_ACCEL_ODR_HZ[(self.b[130] >> 6) & 0x3]
        return out

    # The highest each part can go: a packet rate above it is served at the
    # ceiling, by design (§10.6 "Firmware enforcement").
    CEILING_HZ = {"lsm6dsv": 3840, "lis2dw12": 1600, "lis2mdl": 100, "adxl371": 2560}

    @property
    def wr_accel_hr_mode(self) -> bool:
        return bool(self.b[6] & 0x01)

    def ceiling_hz(self, part: str) -> float:
        if part == "lis2dw12" and not self.wr_accel_hr_mode:
            return 200
        return self.CEILING_HZ[part]

    def odr_shortfalls(self) -> list[str]:
        """Enabled sensors whose stored ODR is below min(packet rate, part ceiling)."""
        out = []
        for part, hz in self.odrs_hz().items():
            need = min(self.rate_hz, self.ceiling_hz(part))
            if not hz >= need:                   # also catches NaN
                out.append(f"{part} ODR {hz} Hz below {need:.1f} Hz")
        return out
