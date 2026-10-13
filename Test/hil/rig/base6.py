"""The Base6 dock: six Shimmer slots behind one FT4232.

  channel A  the dock's own MSP430 bootloader (not used here)
  channel B  slot multiplexer - ASCII commands ending '$', replies ending CRLF
  channel C  the selected Shimmer's bootloader (BSL) lines
  channel D  the selected Shimmer's dock UART (the '$' dock protocol, 115200)

Only one slot is connected to channels C and D at a time. SDQ$ reports which
slots are occupied ("Q,101100", slot 1 first); SDP,0n$ connects slot n and
needs ~1.5 s to settle; SDD$ disconnects all. The protocol is the one
Consensys uses (dockManager/SmartDockUart.java).

Ports are found by FTDI serial (<base>A..D), never by COM number. Consensys
must be closed: it holds these ports open.
"""

from __future__ import annotations

import time

import serial
from serial.tools import list_ports

from .config import RigError

SETTLE_S = 1.5


def port_for(serial_prefix: str) -> str | None:
    for p in list_ports.comports():
        if (p.serial_number or "").upper() == serial_prefix.upper():
            return p.device
    return None


class Base6:
    def __init__(self, ftdi_serial: str):
        """ftdi_serial is the FT4232's serial without the channel letter, e.g. 'SRYRNS21'."""
        self.serial = ftdi_serial
        ports = {ch: port_for(f"{ftdi_serial}{ch}") for ch in "ABCD"}
        missing = [ch for ch in "BCD" if not ports[ch]]
        if missing:
            raise RigError(f"Base6 {ftdi_serial}: channel(s) {', '.join(missing)} not present")
        self.ports = ports
        self.mux = serial.Serial(ports["B"], 115200, timeout=1.0)
        self.selected: int | None = None

    @property
    def uart_port(self) -> str:
        """COM port of the selected slot's dock UART (channel D)."""
        return self.ports["D"]

    @property
    def bsl_port(self) -> str:
        """COM port of the selected slot's bootloader lines (channel C)."""
        return self.ports["C"]

    def _cmd(self, text: str) -> str:
        self.mux.reset_input_buffer()
        self.mux.write(text.encode("ascii"))
        return self.mux.readline().decode("ascii", "replace").strip()

    def occupied(self) -> list[int]:
        r = self._cmd("SDQ$")
        if not r.startswith("Q,"):
            raise RigError(f"Base6 SDQ$ answered {r!r}")
        return [i + 1 for i, c in enumerate(r[2:]) if c == "1"]

    def select(self, slot: int) -> None:
        if not 1 <= slot <= 6:
            raise ValueError(slot)
        if self.selected == slot:
            return
        self._cmd(f"SDP,0{slot}$")
        time.sleep(SETTLE_S)
        self.selected = slot

    def shimmer_bsl_lines(self, connected: bool) -> None:
        """Connect (SDS,0$) or mask (SDS,1$) the selected slot's RST/TEST lines.

        Masked by default, so that opening channel C - which toggles DTR/RTS -
        cannot reset a docked Shimmer by accident. Unmask only to flash.
        """
        r = self._cmd("SDS,0$" if connected else "SDS,1$")
        if r.startswith("E"):
            raise RigError(f"Base6 refused to {'unmask' if connected else 'mask'} the BSL lines: {r!r}")

    def disconnect(self) -> None:
        self._cmd("SDD$")
        self.selected = None

    def close(self) -> None:
        try:
            self.disconnect()
        finally:
            self.mux.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
