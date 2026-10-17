"""Classic Bluetooth (SPP) link to a Shimmer3 / Shimmer3R, and a stream checker.

Only what the tests need, written against
docs/SHIMMER3_BT_COMMUNICATION_PROTOCOL.md and
docs/SHIMMER3_STREAMING_DATA_FORMAT.md rather than against any host API, so a
host-side bug cannot hide a firmware one. The session runs with CRC off: the
first thing sent is SET_CRC 0, which §3.3 allows before the version reads.

The SPP port is found from the unit's MAC: Windows names the outgoing port's
device instance ...&0&<MAC>_C00000000.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field

import serial
from serial.tools import list_ports

ACK, NACK = 0xFF, 0xFE
INSTREAM = 0x8A

INQUIRY, INQUIRY_RSP = 0x01, 0x02
GET_DEVICE_VERSION, DEVICE_VERSION_RSP = 0x3F, 0x25
GET_FW_VERSION, FW_VERSION_RSP = 0x2E, 0x2F
GET_STATUS, STATUS_RSP = 0x72, 0x71
SET_CRC = 0x8B
START_STREAMING, STOP_STREAMING = 0x07, 0x20
START_LOGGING, STOP_LOGGING = 0x92, 0x93
START_SDBT, STOP_SDBT = 0x70, 0x97

STATUS_DOCKED, STATUS_SENSING, STATUS_LOGGING, STATUS_STREAMING = 0x01, 0x02, 0x08, 0x10
STATUS_SD_INSERTED, STATUS_SD_BAD_FILE = 0x20, 0x40

HW_SHIMMER3, HW_SHIMMER3R = 3, 10
# Shimmer3R LogAndStream sends two status bytes from v1.00.024, one before it
# (SHIMMER3_BT_COMMUNICATION_PROTOCOL.md §5.3). Shimmer3 always sends one.
S3R_TWO_STATUS_BYTES_FROM = (1, 0, 24)
TICKS_PER_S = 32768
TS_WRAP = 1 << 24

# Bytes per channel ID (STREAMING_DATA_FORMAT §3). Anything not listed is fatal:
# a parser that guesses a width produces plausible garbage (§5.6).
_WIDTH = {**{i: 2 for i in range(0x00, 0x1A)}, 0x1B: 3, 0x1C: 2,
          0x1D: 1, 0x1E: 3, 0x1F: 3, 0x20: 1, 0x21: 3, 0x22: 3,
          **{i: 2 for i in range(0x23, 0x29)}}


class BtError(Exception):
    pass


def channel_width(ch: int, hw: int) -> int:
    if ch == 0x1A:                       # BMP temperature: 2 on Shimmer3, 3 on Shimmer3R
        return 3 if hw == HW_SHIMMER3R else 2
    if ch not in _WIDTH:
        raise BtError(f"unknown channel id {ch:#04x}")
    return _WIDTH[ch]


def spp_port(mac: str) -> str | None:
    mac = mac.upper()
    for p in list_ports.comports():
        if f"&{mac}_C" in (p.hwid or "").upper():
            return p.device
    return None


@dataclass
class Inquiry:
    divider: int
    channels: list[int]
    buffer_size: int
    config: bytes

    @property
    def rate_hz(self) -> float:
        return TICKS_PER_S / self.divider


@dataclass
class StreamStats:
    packets: int = 0
    seconds: float = 0.0
    gaps: int = 0               # stamps that skip a slot of the sampling grid: samples lost
    missing: int = 0            # samples implied missing by those steps
    max_gap_ticks: int = 0
    dupes: int = 0              # stamps in a slot already taken, or before it: stamped twice
    jitter: int = 0             # stamps off their grid slot by more than a tick
    max_jitter_ticks: int = 0
    resyncs: int = 0            # bytes skipped to find a packet boundary
    status_pushes: int = 0
    first_ts: int | None = None
    deltas: dict = field(default_factory=dict)
    crc_errors: int = 0
    stop_ack: bool = False      # the stop command's ACK arrived at a frame boundary
    after_stop: int = 0         # packets that arrived after the stop was sent
    misframes: list = field(default_factory=list)   # hex of the first few frames that failed to confirm
    ts: list = field(default_factory=list)       # kept with stream(keep=True)
    rows: list = field(default_factory=list)

    def rate_hz(self) -> float:
        return self.packets / self.seconds if self.seconds else 0.0


def ts_step(ts: int, last: int) -> int:
    """The signed step from one 24-bit timestamp to the next.

    Across a wrap the step is forward and small. A stamp earlier than the last
    one is a step backwards, not a wrap: reduced modulo 2^24 alone it would look
    like a gap of nearly 512 s. No step in a stream or a file is half the range.
    """
    d = (ts - last) % TS_WRAP
    return d - TS_WRAP if d >= TS_WRAP // 2 else d


def classify_step(st, d: int, period: int) -> None:
    """Sort one timestamp step (signed, from ts_step) into lost samples,
    duplicates and jitter. Feed the steps in order.

    Steps are judged against the sampling grid, not one at a time. Each stamp
    is placed on the grid by its total distance from the first, so a sample
    stamped late - one long step then one short, summing to two periods - is
    one late sample, not a lost one followed by a duplicate. A stamp can be
    late (interrupt latency) but not early, so anything up to three quarters of
    a period late still belongs to its own slot: a stamp exactly half a period
    late, as seen on the bench, is jitter. The sampling timer keeps its phase
    through a stall, so the grid holds across gaps, and a lost sample whose
    successor is stamped on time or a little early is still a loss. A stamp
    that lands in a slot already taken, or before it, is a duplicate. Shared
    with sdfile.py.
    """
    cum = getattr(st, "_cum", 0) + d
    prev = getattr(st, "_slot", 0)
    slot = (cum + period // 4) // period
    st._cum, st._slot = cum, max(prev, slot)
    step = slot - prev
    if step <= 0:
        st.dupes += 1
        return
    if step >= 2:
        st.gaps += 1
        st.missing += step - 1
        st.max_gap_ticks = max(st.max_gap_ticks, d)
    off = abs(cum - slot * period)
    if off > 1:
        st.jitter += 1
        st.max_jitter_ticks = max(st.max_jitter_ticks, off)


class BtLink:
    def __init__(self, port: str, connect_timeout_s: float = 10.0):
        # Opening an SPP port is what makes Windows page the device; it can take
        # several seconds, and fails outright if the link cannot be made.
        self.ser = _open_spp(port, connect_timeout_s)
        self.hw: int | None = None
        self.crc_mode = 0
        self._nstatus: int | None = None
        # Until the constructor returns there is no link to close, so if the
        # first exchange fails the port is closed here: left open, it would
        # hold the COM port against every later connection.
        try:
            self.ser.timeout = 1.0
            time.sleep(0.2)
            self.ser.write(bytes([SET_CRC, 0]))
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            self.hw = self.device_version()
        except BaseException:
            self.ser.close()
            raise

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _read(self, n: int) -> bytes:
        b = self.ser.read(n)
        if len(b) != n:
            raise BtError(f"timed out: wanted {n} bytes, got {len(b)}")
        return b

    def command(self, op: int, args: bytes = b"", rsp: int | None = None, wrapped: bool = False) -> None:
        """Send op, consume the ACK and (if rsp) the response opcode. Payload is the caller's to read."""
        self.ser.reset_input_buffer()
        self.ser.write(bytes([op]) + args)
        a = self._read(1)[0]
        if a == NACK:
            raise BtError(f"NACK for {op:#04x}")
        if a != ACK:
            raise BtError(f"expected ACK for {op:#04x}, got {a:#04x}")
        if wrapped:
            if self._read(1)[0] != INSTREAM:
                raise BtError(f"expected in-stream wrapper for {op:#04x}")
        if rsp is not None:
            r = self._read(1)[0]
            if r != rsp:
                raise BtError(f"expected response {rsp:#04x} for {op:#04x}, got {r:#04x}")

    def device_version(self) -> int:
        self.command(GET_DEVICE_VERSION, rsp=DEVICE_VERSION_RSP)
        return self._read(1)[0]

    def fw_version(self) -> tuple[int, int, int, int]:
        self.command(GET_FW_VERSION, rsp=FW_VERSION_RSP)
        fw_id, major, minor, patch = struct.unpack("<HHBB", self._read(6))
        return fw_id, major, minor, patch

    def status_len(self) -> int:
        """Status bytes this unit sends, from its hardware and, on Shimmer3R, its
        firmware version (asked once). Never call it while streaming."""
        if self._nstatus is None:
            if self.hw != HW_SHIMMER3R:
                self._nstatus = 1
            else:
                _, major, minor, patch = self.fw_version()
                self._nstatus = 2 if (major, minor, patch) >= S3R_TWO_STATUS_BYTES_FROM else 1
        return self._nstatus

    def status(self, timeout_s: float = 3.0) -> int:
        """Status byte 0 (byte 1, Shimmer3R's usbPluggedIn, is read and dropped).

        The reply is found by its frame, ACK 0x8A 0x71, rather than assumed to
        be the next byte: after a stop at a high rate a stale data packet can
        still be in the device's transmit ring, and it goes out ahead of the
        reply (seen with STOP_SDBT at 256 Hz). Reading status therefore also
        flushes the link for the command after it.
        """
        n = self.status_len()
        pattern = bytes([ACK, INSTREAM, STATUS_RSP])
        self.ser.reset_input_buffer()
        self.ser.write(bytes([GET_STATUS]))
        buf = bytearray()
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            buf += self.ser.read(max(1, self.ser.in_waiting))
            i = buf.find(pattern)
            if i >= 0 and len(buf) >= i + 3 + n:
                time.sleep(0.05)
                self.ser.reset_input_buffer()     # its CRC, if a mode is on
                return buf[i + 3]
        raise BtError(f"no status reply within {timeout_s:.0f} s (got {bytes(buf[:24]).hex()})")

    def inquiry(self) -> Inquiry:
        self.command(INQUIRY, rsp=INQUIRY_RSP)
        fixed = 11 if self.hw == HW_SHIMMER3R else 8
        head = self._read(fixed)
        divider = struct.unpack("<H", head[:2])[0]
        nch, bufsize = head[fixed - 2], head[fixed - 1]
        chans = list(self._read(nch))
        return Inquiry(divider, chans, bufsize, head[2:fixed - 2])

    def all_calibration(self) -> bytes:
        """GET_ALL_CALIBRATION: 4 x 21 bytes on Shimmer3, 6 x 21 on Shimmer3R (§7.6)."""
        self.command(0x2C, rsp=0x2D)
        return self._read(126 if self.hw == HW_SHIMMER3R else 84)

    def wait_status(self, mask: int, want: bool, timeout_s: float = 10.0) -> int:
        end = time.monotonic() + timeout_s
        while True:
            s = self.status()
            if bool(s & mask) == want:
                return s
            if time.monotonic() > end:
                raise BtError(f"status {s:#04x}: bit mask {mask:#04x} not {'set' if want else 'clear'} "
                              f"after {timeout_s:.0f} s")
            time.sleep(0.5)

    def simple(self, op: int) -> None:
        self.command(op)

    # --- streaming -----------------------------------------------------------

    def set_crc(self, mode: int) -> None:
        """SET_CRC 0/1/2 (§3.3). The reply's own trailing bytes depend on which
        mode applies to it, so it is drained rather than parsed."""
        self.ser.reset_input_buffer()
        self.ser.write(bytes([SET_CRC, mode]))
        # Wait for the ACK itself: just after a stop the firmware is busy
        # closing the SD file and can take well over a second to answer, and a
        # late reply would otherwise sit in front of the next command's.
        old = self.ser.timeout
        self.ser.timeout = 5.0
        try:
            a = self.ser.read(1)
        finally:
            self.ser.timeout = old
        if a != bytes([ACK]):
            raise BtError(f"SET_CRC {mode}: expected ACK, got {a.hex() or 'nothing'}")
        self.quiesce(quiet_s=0.2, max_s=1.0)     # its CRC bytes, in whichever mode applied
        self.crc_mode = mode

    def stream(self, seconds: float, inq: Inquiry | None = None, *, keep: bool = False,
               start_op: int = START_STREAMING, stop_op: int = STOP_STREAMING) -> StreamStats:
        """Stream for `seconds`, checking framing, CRCs and timestamp continuity.

        With keep=True the timestamps and channel bytes of every packet are kept
        (st.ts, st.rows) for decoding. start_op/stop_op select plain streaming
        or START_SDBT/STOP_SDBT (stream and log together).
        """
        inq = inq or self.inquiry()
        crc_n = getattr(self, "crc_mode", 0)
        body = 4 + sum(channel_width(c, self.hw) for c in inq.channels)
        plen = body + crc_n
        push_len = 3 + self.status_len() + crc_n  # [ACK][0x8A][0x71][status...][CRC]; asked before the start
        st = StreamStats()
        self.command(start_op)
        if crc_n:
            self._read(crc_n)                    # the ACK's own CRC
        buf = bytearray()
        last_ts = None
        t0 = time.monotonic()
        stop_t = None               # set once the stop command has gone out
        old_timeout = self.ser.timeout
        try:
            while True:
                now = time.monotonic()
                if stop_t is None and now - t0 >= seconds:
                    st.seconds = now - t0
                    self.ser.write(bytes([stop_op]))
                    stop_t = now
                    self.ser.timeout = 0.3
                if stop_t is None:
                    buf += self.ser.read(max(1, self.ser.in_waiting))
                else:
                    # After the stop: keep parsing what is still in flight -
                    # those are real samples - and look for the stop's ACK,
                    # until the link has been quiet for 0.3 s (or 5 s pass).
                    chunk = self.ser.read(4096)
                    buf += chunk
                    final = not chunk or now - stop_t > 5.0
                while buf:
                    if buf[0] == 0x00:
                        need = plen + (0 if stop_t is not None and final else 1)
                        if len(buf) < need:              # +1: peek at the next frame's first byte
                            break
                        ts = buf[1] | (buf[2] << 8) | (buf[3] << 16)
                        # A 0x00 is not proof of a packet boundary. Confirm it: the
                        # next frame must start with 0x00 or an ACK; with a CRC on,
                        # the CRC must match; without one, the timestamp must step
                        # by a plausible number of periods. A frame that fails is
                        # re-searched a byte later and counted - it means bytes the
                        # parser did not expect arrived (and are kept for diagnosis).
                        if len(buf) > plen and buf[plen] not in (0x00, ACK):
                            ok = False
                        elif crc_n:
                            ok = _crc_ok(buf[:body], buf[body:plen])
                            if not ok:
                                st.crc_errors += 1
                        else:
                            ok = last_ts is None or _plausible_step((ts - last_ts) % TS_WRAP, inq.divider)
                        if not ok:
                            st.resyncs += 1
                            if len(st.misframes) < 3:
                                st.misframes.append(bytes(buf[:plen + 8]).hex())
                            del buf[:1]
                            continue
                        if last_ts is None:
                            st.first_ts = ts
                        else:
                            d = ts_step(ts, last_ts)
                            st.deltas[d] = st.deltas.get(d, 0) + 1
                            classify_step(st, d, inq.divider)
                        last_ts = ts
                        st.packets += 1
                        if stop_t is not None:
                            st.after_stop += 1
                        if keep:
                            st.ts.append(ts)
                            st.rows.append(bytes(buf[4:body]))
                        del buf[:plen]
                    elif buf[0] == ACK and len(buf) >= 2 and buf[1] == INSTREAM:
                        if len(buf) < push_len:
                            break
                        st.status_pushes += 1
                        del buf[:push_len]
                    elif buf[0] == ACK and stop_t is not None and (len(buf) >= 2 or final):
                        # A bare ACK at a frame boundary after the stop: the stop's.
                        st.stop_ack = True
                        del buf[:1 + crc_n]
                    elif buf[0] == ACK and len(buf) < 2:
                        break
                    else:
                        st.resyncs += 1
                        if len(st.misframes) < 3:
                            st.misframes.append(bytes(buf[:plen + 8]).hex())
                        del buf[:1]
                if stop_t is not None and final:
                    break
        finally:
            if stop_t is None:                   # left early on an exception
                st.seconds = time.monotonic() - t0
                self.ser.write(bytes([stop_op]))
            self.ser.timeout = old_timeout
            self.quiesce()
        return st

    def quiesce(self, quiet_s: float = 0.3, max_s: float = 5.0) -> None:
        """Drain until the link has been silent for quiet_s.

        After a stop, packets already queued in the device and the radio keep
        arriving for a while - longer at high rates - and would otherwise sit in
        front of the next command's reply.
        """
        end = time.monotonic() + max_s
        old = self.ser.timeout
        self.ser.timeout = quiet_s
        try:
            while time.monotonic() < end:
                if not self.ser.read(4096):
                    break
        finally:
            self.ser.timeout = old


def _plausible_step(d: int, period: int) -> bool:
    """A step of 1..(10 s) worth of ticks, within a quarter period of a whole number of periods."""
    k = round(d / period)
    return 1 <= k and d <= 10 * TICKS_PER_S and abs(d - k * period) <= period // 4


PORT_RELEASE_WAIT_S = 30.0


def _open_spp(port: str, timeout_s: float) -> serial.Serial:
    """Open an SPP COM port, waiting out Windows' release of the previous link.

    Closing an SPP port returns before Windows has torn the link down; until it
    has, opening that port - and on this bench sometimes another unit's - fails
    with "Access is denied". That is the PC, not the device, so it is waited
    for rather than reported. Any other error is raised at once.
    """
    end = time.monotonic() + PORT_RELEASE_WAIT_S
    paged = False
    while True:
        # Opened with DTR and RTS low, pyserial's default being high. On
        # Shimmer3R firmware without the module-frame guard (seen with IF820
        # v1.4.16.16), a port opened with them raised can have the rest of the
        # module's connect event run as a command, and on one host that changed
        # the stored configuration (SHIMMER3_BT_COMMUNICATION_PROTOCOL.md §2).
        ser = serial.Serial()
        ser.port, ser.baudrate = port, 115200
        ser.timeout = ser.write_timeout = timeout_s
        ser.dtr = ser.rts = False
        try:
            ser.open()
            return ser
        except serial.SerialException as e:
            if "semaphore timeout" in str(e) and not paged:
                # One page timeout is retried: seen right after the unit's USB
                # re-enumerates on re-dock. A second one is reported.
                paged = True
                time.sleep(5.0)
                continue
            if "Access is denied" not in str(e) or time.monotonic() > end:
                raise
            time.sleep(1.0)


def _crc_ok(frame: bytes, got: bytes) -> bool:
    """The link CRC (§3.3): Shimmer CRC-16 over the frame, low byte first; mode 1 sends only the low byte."""
    from .dock import crc16
    c = crc16(bytes(frame))
    want = bytes([c & 0xFF, c >> 8])[:len(got)]
    return bytes(got) == want
