"""Flash a Shimmer3 (MSP430F5437A) through its ROM bootloader (BSL) over a dock.

A Python port of the parts of Consensys's MSPBSL library that flashing needs
(ConsensysApi .../bsl/msp430/: MSPBSL_PhysicalInterfaceSerialUart,
MSPBSL_PacketHandler5xxUart, MSPBSL_Connection5xx), checked against TI's
MSP430 BSL user's guide for the 5xx/6xx UART protocol.

Physical layer: the dock's bootloader channel (single dock: FT2232 A; Base6:
FT4232 C, with the slot selected and the Shimmer BSL lines unmasked) at 9600
8E1. DTR drives RST and RTS drives TEST, both active low. Entry is the 5xx
sequence: TEST pulses while RST is held low, then RST released.

Packet: 0x80 lenL lenH payload crcL crcH, CRC-CCITT seed 0xFFFF over the
payload. Every packet is answered by one byte, 0x00 = ACK; commands with a
reply then send 0x80 len payload crc, where payload[0] is 0x3B (message,
payload[1] = status, 0 = OK) or 0x3A (data).

Mass erase clears main flash and information memory B-D, which hold the
Shimmer's configuration and calibration. Callers back InfoMem up first and
put it back afterwards - see flash_shimmer3() in rig/s3_flash.py.

Every block written is verified with the BSL's own CRC_CHECK, so a
successful return means flash holds the image byte for byte.
"""

from __future__ import annotations

import struct
import time
from pathlib import Path

import serial

ACK = 0x00
HEADER = 0x80
RX_DATA_BLOCK, RX_PASSWORD, ERASE_SEGMENT, MASS_ERASE = 0x10, 0x11, 0x12, 0x15
CRC_CHECK, TX_BSL_VERSION, CHANGE_BAUD = 0x16, 0x19, 0x52
DATA_RESPONSE, MESSAGE_RESPONSE = 0x3A, 0x3B
BAUD_CODES = {9600: 0x02, 19200: 0x03, 38400: 0x04, 57600: 0x05, 115200: 0x06}
MAX_DATA = 240                  # bytes of data per RX_DATA_BLOCK, as Consensys uses
STATUS = {0x00: "ok", 0x01: "flash write check failed", 0x02: "flash fail bit set",
          0x03: "voltage change during program", 0x04: "BSL locked", 0x05: "password error",
          0x06: "byte write forbidden", 0x07: "unknown command", 0x08: "packet too long"}


class BslError(Exception):
    pass


def crc_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        x = ((crc >> 8) ^ b) & 0xFF
        x ^= x >> 4
        crc = ((crc << 8) ^ (x << 12) ^ (x << 5) ^ x) & 0xFFFF
    return crc


def read_ti_txt(path: Path) -> list[tuple[int, bytes]]:
    """[(address, data)] from a TI-TXT image."""
    segs: list[tuple[int, bytearray]] = []
    for line in Path(path).read_text(encoding="ascii").splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("@"):
            segs.append((int(line[1:], 16), bytearray()))
        elif line.lower() == "q":
            break
        else:
            if not segs:
                raise BslError(f"{path}: data before the first @address")
            segs[-1][1].extend(int(t, 16) for t in line.split())
    return [(a, bytes(d)) for a, d in segs if d]


class Bsl:
    def __init__(self, port: str, timeout_s: float = 2.0):
        self.ser = serial.Serial(port, 9600, bytesize=8, parity=serial.PARITY_EVEN,
                                 stopbits=1, timeout=timeout_s)

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # --- pins: RST on DTR, TEST on RTS, both asserted = low ------------------

    def _pins(self, rst_high: bool, test_high: bool, wait_ms: int):
        self.ser.dtr = not rst_high
        self.ser.rts = not test_high
        time.sleep(wait_ms / 1000)

    def invoke(self):
        """Reset into the BSL (5xx entry sequence)."""
        self._pins(False, False, 250)
        self._pins(False, False, 10)
        self._pins(False, True, 10)
        self._pins(False, False, 10)
        self._pins(False, True, 10)
        self._pins(False, False, 10)
        self._pins(True, False, 10)
        time.sleep(0.35)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.baudrate = 9600
        time.sleep(0.35)

    def exit_to_app(self):
        """Reset into the application: RST pulsed low with TEST held low."""
        self._pins(False, False, 250)
        self._pins(True, False, 360)

    # --- packets -------------------------------------------------------------

    def _tx(self, payload: bytes):
        frame = bytes([HEADER]) + struct.pack("<H", len(payload)) + payload \
            + struct.pack("<H", crc_ccitt(payload))
        self.ser.write(frame)
        a = self.ser.read(1)
        if not a:
            raise BslError(f"no ACK for command {payload[0]:#04x}")
        if a[0] != ACK:
            hint = " (0x90: BSL not entered, or the application answered)" if a[0] == 0x90 else ""
            raise BslError(f"command {payload[0]:#04x}: got {a[0]:#04x}, expected ACK{hint}")

    def _rx(self) -> bytes:
        head = self.ser.read(3)
        if len(head) != 3 or head[0] != HEADER:
            raise BslError(f"bad reply header {head.hex()}")
        n = struct.unpack("<H", head[1:3])[0]
        body = self.ser.read(n + 2)
        if len(body) != n + 2:
            raise BslError("reply timed out")
        payload, crc = body[:n], struct.unpack("<H", body[n:])[0]
        if crc != crc_ccitt(payload):
            raise BslError("reply CRC mismatch")
        return payload

    def _message(self, payload: bytes) -> None:
        self._tx(payload)
        r = self._rx()
        if r[0] != MESSAGE_RESPONSE:
            raise BslError(f"command {payload[0]:#04x}: expected a message, got {r.hex()}")
        if r[1] != 0:
            raise BslError(f"command {payload[0]:#04x}: {STATUS.get(r[1], hex(r[1]))}")

    def _data(self, payload: bytes) -> bytes:
        self._tx(payload)
        r = self._rx()
        if r[0] != DATA_RESPONSE:
            raise BslError(f"command {payload[0]:#04x}: expected data, got {r.hex()}")
        return r[1:]

    # --- commands ------------------------------------------------------------

    def mass_erase(self):
        self._message(bytes([MASS_ERASE]))

    def password(self, pw: bytes = b"\xff" * 32):
        self._message(bytes([RX_PASSWORD]) + pw)

    def change_baud(self, baud: int):
        self._tx(bytes([CHANGE_BAUD, BAUD_CODES[baud]]))     # ACK only
        time.sleep(0.05)
        self.ser.baudrate = baud

    def version(self) -> bytes:
        return self._data(bytes([TX_BSL_VERSION]))

    def write(self, addr: int, data: bytes):
        self._message(bytes([RX_DATA_BLOCK]) + addr.to_bytes(3, "little") + data)

    def crc(self, addr: int, length: int) -> int:
        r = self._data(bytes([CRC_CHECK]) + addr.to_bytes(3, "little") + struct.pack("<H", length))
        return struct.unpack("<H", r[:2])[0]


def program(port: str, image: Path, baud: int = 115200, log=print) -> dict:
    """Mass-erase, write and CRC-verify `image`, then reset into it."""
    segs = read_ti_txt(image)
    total = sum(len(d) for _, d in segs)
    t0 = time.monotonic()
    with Bsl(port) as b:
        b.invoke()
        b.mass_erase()          # before the password: after it the password is all 0xFF
        if baud != 9600:
            b.change_baud(baud)
        b.password()
        ver = b.version()
        log(f"BSL {ver.hex()}: writing {total} bytes in {len(segs)} segments")
        for addr, data in segs:
            for off in range(0, len(data), MAX_DATA):
                b.write(addr + off, data[off:off + MAX_DATA])
        for addr, data in segs:
            # CRC_CHECK takes a 16-bit length; check long segments piecewise.
            for off in range(0, len(data), 0x8000):
                chunk = data[off:off + 0x8000]
                got = b.crc(addr + off, len(chunk))
                if got != crc_ccitt(chunk):
                    raise BslError(f"verify failed at {addr + off:#07x}: flash CRC {got:#06x}, "
                                   f"image {crc_ccitt(chunk):#06x}")
        b.exit_to_app()
    return {"bytes": total, "segments": len(segs), "bsl_version": ver.hex(),
            "seconds": round(time.monotonic() - t0, 1)}
