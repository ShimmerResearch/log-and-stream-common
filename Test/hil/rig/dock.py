"""The dock protocol, as spoken by the Shimmer3R USB CDC port and the Shimmer3
dock UART (docs/SHIMMER3_DOCK_PROTOCOL.md).

    [$] [cmd] [len] [component] [property] [data ...] [crc lo] [crc hi]

A small, strict client rather than Extras' ShimmerUart: a test needs every
error code to raise, not print, and needs a NACK to be distinguishable from an
empty answer. The CRC itself is reused from the Extras host reference, which
crosscheck_host_constants.py already checks against the firmware.

`request` and `exchange` return the device's reply as it came, error codes
included, for the tests that send a wrong frame on purpose (rig/dock_protocol.py);
`get` and `set` raise on any error code.
"""

from __future__ import annotations

import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial

_EXTRAS = Path(__file__).resolve().parents[3] / "Extras" / "python_scripts" / "Shimmer_common"
sys.path.insert(0, str(_EXTRAS))
import shimmer_crc  # noqa: E402

START = 0x24
CMD_SET, CMD_RSP, CMD_GET = 0x01, 0x02, 0x03
ACK, BAD_CMD, BAD_ARG, BAD_CRC = 0xFF, 0xFC, 0xFD, 0xFE
ERRORS = {BAD_CMD: "bad command", BAD_ARG: "bad argument", BAD_CRC: "bad CRC"}
# The firmware's names for the reply codes (UART_BAD_CMD_RESPONSE and so on).
CODE_NAMES = {CMD_RSP: "RSP", ACK: "ACK", BAD_CMD: "BAD_CMD", BAD_ARG: "BAD_ARG", BAD_CRC: "BAD_CRC"}

COMP_SHIMMER, COMP_BAT, COMP_DAUGHTER_CARD, COMP_BLUETOOTH = 0x01, 0x02, 0x03, 0x0A
COMP_D_ACCEL, COMP_GSR, COMP_TEST = 0x04, 0x05, 0x0B     # declared; only TEST is handled
PROP_MAC, PROP_VER, PROP_RWC_CFG_TIME, PROP_CURR_TIME, PROP_INFOMEM = 0x02, 0x03, 0x04, 0x05, 0x06
PROP_LED0_STATE = 0x07          # ShimmerGQ only
PROP_ENTER_BOOTLOADER = 0x09    # Shimmer3R only
PROP_BAT_VALUE = 0x02
PROP_BT_VER = 0x03
PROP_CARD_ID = 0x02             # COMP_DAUGHTER_CARD: the 16-byte expansion-board ID page
PROP_CARD_MEM = 0x03            # COMP_DAUGHTER_CARD: EEPROM from offset + 16 (DOCK_PROTOCOL)
CARD_ID_SIZE = 16
CARD_MEM_BASE = 16
CARD_MEM_SIZE = 2032            # CARD_MEM reaches EEPROM 16-2047
CARD_MEM_CHUNK = 128
FACTORY_TEST_COUNT = 4          # Test/shimmer_test.h: suites 0-3; a higher property is BAD_CMD
# The EEPROM's radio settings page (SHIMMER3_EEPROM_MEMORY_MAP.md §4): byte 2
# bit 0 is bleEnabled, which the firmware brings BLE up by (§4.2).
EEPROM_RADIO_PAGE = 2032
RADIO_PAGE_SIZE = 16
RADIO_BLE_ENABLED = 0x01

INFOMEM_SIZE = 0x200
INFOMEM_CHUNK = 128
TICKS_PER_S = 32768             # the real-world clock's unit (SHIMMER3_TIMEKEEPING.md §1)


class DockError(Exception):
    pass


def crc16(buf: bytes) -> int:
    return shimmer_crc.calc_crc(len(buf), list(buf))


def frame(cmd: int, comp: int, prop: int, data: bytes = b"") -> bytes:
    body = bytes([START, cmd, 2 + len(data), comp, prop]) + data
    return body + struct.pack("<H", crc16(body))


@dataclass
class Reply:
    """One frame from the device: a response with its component, property and
    value, or a bare ACK or error code."""
    code: int
    comp: int = 0
    prop: int = 0
    data: bytes = b""

    @property
    def name(self) -> str:
        return CODE_NAMES.get(self.code, f"{self.code:#04x}")


@dataclass
class Version:
    hw: int
    fw_id: int
    major: int
    minor: int
    patch: int

    @property
    def text(self) -> str:
        return f"{self.major}.{self.minor:02d}.{self.patch:03d}"


class DockLink:
    def __init__(self, port: str, timeout_s: float = 1.0):
        self.ser = serial.Serial(port, 115200, timeout=timeout_s)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _read_exact(self, n: int) -> bytes:
        b = self.ser.read(n)
        if len(b) != n:
            raise DockError(f"timed out: wanted {n} bytes, got {len(b)} ({b.hex()})")
        return b

    def read_reply(self, timeout_s: float | None = None) -> Reply | None:
        """The next frame from the device, or None if nothing starts arriving in
        time. A frame that starts and is then cut short or fails its CRC raises.

        The port's timeout is changed, and put back, only for a timeout_s that
        differs from it. pyserial reconfigures the port at every assignment,
        which on the Shimmer3R's CDC port sends the unit line-coding and
        control-line requests; restoring it after every exchange did that each
        time, and it is the call that failed with WinError 995 inside a host
        service stall (DEV-1368, s3r_usb.stall_named)."""
        old = self.ser.timeout
        if timeout_s is None or timeout_s == old:
            first = self.ser.read(1)
        else:
            self.ser.timeout = timeout_s
            try:
                first = self.ser.read(1)
            finally:
                self.ser.timeout = old
        if not first:
            return None
        head = first + self._read_exact(1)
        if head[0] != START:
            raise DockError(f"expected '$', got {head.hex()}")
        code = head[1]
        if code in (ACK, BAD_CMD, BAD_ARG, BAD_CRC):
            crc = self._read_exact(2)
            if struct.unpack("<H", crc)[0] != crc16(head):
                raise DockError("CRC mismatch on ACK/NACK")
            return Reply(code)
        if code != CMD_RSP:
            raise DockError(f"unexpected response code {code:#x}")
        n = self._read_exact(1)[0]
        rest = self._read_exact(n + 2)
        body, crc = head + bytes([n]) + rest[:n], rest[n:]
        if struct.unpack("<H", crc)[0] != crc16(body):
            raise DockError("CRC mismatch on response")
        if n < 2:
            raise DockError(f"response LEN {n} leaves no component and property")
        return Reply(code, rest[0], rest[1], bytes(rest[2:n]))

    def exchange(self, raw: bytes, timeout_s: float | None = None) -> Reply:
        """Send these bytes as one write and return the one reply they get."""
        self.ser.reset_input_buffer()
        # One write per frame: the firmware's parser resets after a 100 ms gap.
        self.ser.write(raw)
        r = self.read_reply(timeout_s)
        if r is None:
            raise DockError(f"no reply to {raw.hex()}")
        return r

    def request(self, cmd, comp, prop, data=b"") -> Reply:
        return self.exchange(frame(cmd, comp, prop, data))

    def _transact(self, cmd, comp, prop, data=b"") -> bytes | None:
        r = self.request(cmd, comp, prop, data)
        if r.code in ERRORS:
            raise DockError(f"device answered {ERRORS[r.code]} to comp {comp:#x} prop {prop:#x}")
        if r.code == ACK:
            return None
        if (r.comp, r.prop) != (comp, prop):
            raise DockError(f"response for {r.comp:#x}/{r.prop:#x}, asked {comp:#x}/{prop:#x}")
        return r.data

    def get(self, comp, prop, data=b"") -> bytes:
        r = self._transact(CMD_GET, comp, prop, data)
        if r is None:
            raise DockError("GET answered with a bare ACK")
        return r

    def set(self, comp, prop, data=b"") -> None:
        r = self._transact(CMD_SET, comp, prop, data)
        if r is not None:
            raise DockError("SET answered with data")

    # --- typed helpers -------------------------------------------------------

    def mac(self) -> str:
        return self.get(COMP_SHIMMER, PROP_MAC).hex().upper()

    def version(self) -> Version:
        b = self.get(COMP_SHIMMER, PROP_VER)
        hw, fw_id, major, minor, patch = struct.unpack("<BHHBB", b[:7])
        return Version(hw, fw_id, major, minor, patch)

    def current_ticks(self) -> int:
        return struct.unpack("<Q", self.get(COMP_SHIMMER, PROP_CURR_TIME)[:8])[0]

    def rwc_config_ticks(self) -> int:
        """The tick value the real-world clock was last set to."""
        return struct.unpack("<Q", self.get(COMP_SHIMMER, PROP_RWC_CFG_TIME)[:8])[0]

    def set_rwc(self, ticks: int) -> None:
        """Set the real-world clock: 32768 Hz ticks since the Unix epoch, UTC."""
        self.set(COMP_SHIMMER, PROP_RWC_CFG_TIME, struct.pack("<Q", ticks))

    def battery(self) -> tuple[int, int]:
        b = self.get(COMP_BAT, PROP_BAT_VALUE)
        return b[0] | (b[1] << 8), b[2]

    def bt_version(self) -> str:
        return self.get(COMP_BLUETOOTH, PROP_BT_VER).decode("ascii", "replace").strip("\x00\r\n ")

    def card_id(self, length: int = CARD_ID_SIZE, offset: int = 0) -> bytes:
        """The expansion-board ID page, as the firmware holds it (read at boot)."""
        return self.get(COMP_DAUGHTER_CARD, PROP_CARD_ID, bytes([length, offset]))

    def write_card_id(self, data: bytes, offset: int = 0) -> None:
        self.set(COMP_DAUGHTER_CARD, PROP_CARD_ID, bytes([len(data), offset]) + data)

    def read_card_mem(self, offset: int, length: int) -> bytes:
        """EEPROM from `offset` + 16, in chunks the firmware accepts."""
        out = b""
        while length > 0:
            n = min(length, CARD_MEM_CHUNK)
            out += self.get(COMP_DAUGHTER_CARD, PROP_CARD_MEM, struct.pack("<BH", n, offset))
            offset, length = offset + n, length - n
        return out

    def write_card_mem(self, data: bytes, offset: int) -> None:
        while data:
            chunk, data = data[:CARD_MEM_CHUNK], data[CARD_MEM_CHUNK:]
            self.set(COMP_DAUGHTER_CARD, PROP_CARD_MEM, struct.pack("<BH", len(chunk), offset) + chunk)
            offset += len(chunk)

    def radio_settings(self) -> bytes:
        """The EEPROM's 16-byte radio settings page: radioHwVer, baud, the radio-enable bits.

        A reply of any other length is an error, not a page: its bits would be
        read as settings.
        """
        page = self.get(COMP_DAUGHTER_CARD, PROP_CARD_MEM,
                        struct.pack("<BH", RADIO_PAGE_SIZE, EEPROM_RADIO_PAGE - CARD_MEM_BASE))
        if len(page) != RADIO_PAGE_SIZE:
            raise DockError(f"radio settings read gave {len(page)} bytes, expected {RADIO_PAGE_SIZE}")
        return page

    def read_infomem(self, offset: int = 0, length: int = INFOMEM_SIZE, chunk: int = INFOMEM_CHUNK) -> bytes:
        out = b""
        while length > 0:
            n = min(length, chunk)
            out += self.get(COMP_SHIMMER, PROP_INFOMEM, struct.pack("<BH", n, offset))[:n]
            offset, length = offset + n, length - n
        return out

    def write_infomem(self, data: bytes, offset: int = 0) -> None:
        while data:
            chunk, data = data[:INFOMEM_CHUNK], data[INFOMEM_CHUNK:]
            self.set(COMP_SHIMMER, PROP_INFOMEM, struct.pack("<BH", len(chunk), offset) + chunk)
            offset += len(chunk)

    def enter_bootloader(self, seconds: int = 1) -> None:
        """Shimmer3R: reboot into ST's system bootloader (USB DFU) after this
        many seconds; 0 cancels a pending one. The CDC port vanishes when it fires."""
        self.set(COMP_SHIMMER, PROP_ENTER_BOOTLOADER, bytes([seconds]))


def ble_enabled(radio_page: bytes) -> bool:
    """Whether the radio settings page lets the firmware bring BLE up (bleEnabled)."""
    return len(radio_page) > 2 and bool(radio_page[2] & RADIO_BLE_ENABLED)


def open_when_ready(port: str, attempts: int = 10, delay_s: float = 0.5) -> DockLink:
    """Commands sent while the device boots are silently dropped; retry the first one."""
    last = None
    for _ in range(attempts):
        try:
            link = DockLink(port)
        except serial.SerialException as e:
            last = e
            time.sleep(delay_s)
            continue
        try:
            link.version()
            return link
        except (DockError, serial.SerialException) as e:
            # A unit still booting or re-enumerating can drop the port mid-read,
            # which is a reason to try again, not to fail.
            last = e
            try:
                link.close()
            except serial.SerialException:
                pass
            time.sleep(delay_s)
    raise DockError(f"{port}: no answer after {attempts} attempts ({last})")
