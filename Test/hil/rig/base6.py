"""The Base6 dock: six Shimmer slots behind one FT4232.

  channel A  the dock's own MSP430 bootloader (not used here)
  channel B  slot multiplexer - ASCII commands ending '$', replies ending CRLF
  channel C  the selected Shimmer's bootloader (BSL) lines
  channel D  the selected Shimmer's dock UART (the '$' dock protocol, 115200)

Only one slot is connected to channels C and D at a time. SDQ$ reports which
slots are occupied ("Q,101100", slot 1 first); SDP,0n$ connects slot n and
needs ~1.5 s to settle; SDD$ disconnects all. The protocol is the one
Consensys uses (dockManager/SmartDockUart.java).

SDP leaves the slot's SD card with its Shimmer. SDC,0n$ connects the slot and
switches its card to the Base6's USB card reader ("Shimmer HS-SD/MMC"), where
it mounts as a drive (rig/s3_card.py); the next SDP gives it back. Consensys
waits 5 s on Windows after SDC (SLOT_CHANGEOVER_DELAY_WITH_SD_CARD_WIN).

A slot change answers only once it is done: SDC took 1.44-1.47 s and SDD
1.17 s on one Base6 (SDP about 0.4 s), past the 1 s the other commands get. So
SDP, SDC and SDD replies are read with a longer timeout, 10 s, as Consensys
reads them (SmartDock.SMARTDOCK_RESPONSE_TIMEOUT_SLOT_CHANGE).

A Base6 can also refuse a command at random. From one moment on 24 Nov 2026,
and through a power cycle, one answered most SDC, SDP and SDD commands with an
instant E (it never tried the switch) or not at all, and its queries now and
then. Sent again every 2 s, each got through within 6 tries. So every command
is sent up to TRIES times, RETRY_S apart, until it gets the answer it should.
A select that never takes raises: the dock UART would still be connected to
the slot selected before it, and that slot's unit would answer instead.

Ports are found by FTDI serial (<base>A..D), never by COM number. Consensys
must be closed: it holds these ports open.
"""

from __future__ import annotations

import time

import serial
from serial.tools import list_ports

from .config import RigError

SETTLE_S = 1.5
CARD_CHANGEOVER_S = 5.0     # after SDC, as Consensys waits on Windows
REPLY_S = 1.0               # SDQ, SDS
SLOT_REPLY_S = 10.0         # SDP, SDC, SDD: they answer once the slot has changed
TRIES = 8                   # a refused or unanswered command is sent this many times in all
RETRY_S = 2.0               # between tries


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
        self.mux = serial.Serial(ports["B"], 115200, timeout=REPLY_S)
        self.selected: int | None = None

    @property
    def uart_port(self) -> str:
        """COM port of the selected slot's dock UART (channel D)."""
        return self.ports["D"]

    @property
    def bsl_port(self) -> str:
        """COM port of the selected slot's bootloader lines (channel C)."""
        return self.ports["C"]

    def _cmd(self, text: str, reply_s: float = REPLY_S) -> str:
        """Send one command and read its reply line, waiting up to reply_s."""
        self.mux.reset_input_buffer()
        self.mux.write(text.encode("ascii"))
        was = self.mux.timeout
        if reply_s != was:
            self.mux.timeout = reply_s
        try:
            return self.mux.readline().decode("ascii", "replace").strip()
        finally:
            if reply_s != was:
                self.mux.timeout = was

    def _cmd_until(self, text: str, ok, reply_s: float = REPLY_S) -> tuple[bool, str]:
        """Send one command until ok(reply) holds, at most TRIES times, RETRY_S
        apart. Returns whether it did, and the last reply."""
        r = ""
        for attempt in range(TRIES):
            if attempt:
                time.sleep(RETRY_S)
            r = self._cmd(text, reply_s)
            if ok(r):
                return True, r
        return False, r

    def occupied(self) -> list[int]:
        done, r = self._cmd_until("SDQ$", lambda r: r.startswith("Q,"))
        if not done:
            raise RigError(f"Base6 SDQ$ answered {r!r} after {TRIES} tries")
        return [i + 1 for i, c in enumerate(r[2:]) if c == "1"]

    def select(self, slot: int) -> None:
        if not 1 <= slot <= 6:
            raise ValueError(slot)
        if self.selected == slot:
            return
        # Only this slot's answer will do: a late one for another slot is not it.
        done, r = self._cmd_until(f"SDP,0{slot}$", lambda r: r.startswith(f"P,0{slot}"), SLOT_REPLY_S)
        if not done:
            self.selected = None
            raise RigError(f"Base6 SDP,0{slot}$ answered {r!r} after {TRIES} tries")
        time.sleep(SETTLE_S)
        self.selected = slot

    def card_to_pc(self, slot: int) -> None:
        """Connect slot n with its SD card switched to the USB card reader (SDC,0n$),
        and wait out the changeover. select() gives the card back to the Shimmer."""
        if not 1 <= slot <= 6:
            raise ValueError(slot)
        done, r = self._cmd_until(f"SDC,0{slot}$", lambda r: r.startswith(f"C,0{slot}"), SLOT_REPLY_S)
        # Whatever the answer, the next select() must send SDP, which is what
        # hands the card back.
        self.selected = None
        if not done:
            raise RigError(f"Base6 SDC,0{slot}$ answered {r!r} after {TRIES} tries")
        time.sleep(CARD_CHANGEOVER_S)

    def shimmer_bsl_lines(self, connected: bool) -> None:
        """Connect (SDS,0$) or mask (SDS,1$) the selected slot's RST/TEST lines.

        Masked by default, so that opening channel C - which toggles DTR/RTS -
        cannot reset a docked Shimmer by accident. Unmask only to flash.
        """
        done, r = self._cmd_until("SDS,0$" if connected else "SDS,1$",
                                  lambda r: bool(r) and not r.startswith("E"))
        if not done:
            raise RigError(f"Base6 refused to {'unmask' if connected else 'mask'} the BSL lines: "
                           f"{r!r} after {TRIES} tries")

    def disconnect(self) -> bool:
        """Disconnect every slot (SDD$). Returns whether the Base6 said it did.
        A refusal does not raise: close() runs this on the way out of a failure."""
        done, _ = self._cmd_until("SDD$", lambda r: r.startswith("D"), SLOT_REPLY_S)
        self.selected = None
        return done

    def close(self) -> None:
        try:
            self.disconnect()
        finally:
            self.mux.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
