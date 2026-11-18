#!/usr/bin/env python3
"""Unit tests for the stream parser's framing without a link CRC
(rig.bt.BtLink.stream): which bytes are packets. Run:
python Test/hil/test_hil_framing.py (no hardware).

Without a CRC a frame is confirmed by its timestamps. Shimmer3R v1.01.026
stamps one sample late each time its battery alarm fires (DEV-1464). The
parser used to reject that packet and search its bytes for the next boundary,
and the search could accept a false frame inside the data. Every real packet
then stepped off the false stamp and was rejected too: 193 KB and 380 KB of two
90 s streams "did not frame", where one late stamp every 2 s was the only
defect. The fixtures in testdata/ are cut from those streams, captured whole on
the bench (12 Nov 2026), so the cases here are the ones that happened:

- s3r_1024hz_late_stamp.bin: 110 packets of the 1024 Hz load (13 two-byte
  channels, 30-byte packets). Packet 10 is stamped 9 ticks late: steps of 41
  and 23 ticks where 32 belong. The search after it is the one that went wrong.
- s3r_4096hz_late_stamp.bin: 50 packets at 4096 Hz with VBatt alone (6-byte
  packets). Packet 10 is stamped 5 ticks late: steps of 13 and 3, not 8.

No hardware and no pyserial needed: rig.bt imports pyserial, so where it is not
installed (CI) a stand-in is put in its place, as test_rig_offline.py does.
"""

import sys
import types
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

try:
    import serial  # noqa: F401
    from serial.tools import list_ports  # noqa: F401
except ImportError:
    _serial = types.ModuleType("serial")
    _tools = types.ModuleType("serial.tools")
    _lp = types.ModuleType("serial.tools.list_ports")
    _lp.comports = lambda: []
    _tools.list_ports = _lp
    _serial.tools = _tools
    _serial.Serial = object
    _serial.SerialException = type("SerialException", (OSError,), {})
    sys.modules.update({"serial": _serial, "serial.tools": _tools, "serial.tools.list_ports": _lp})

from rig import bt, throughput  # noqa: E402

DATA = HERE / "testdata"
LOAD = bt.Inquiry(32, list(range(0x0D)), 0, b"")      # 1024 Hz, the load's 13 channels
VBATT_4096HZ = bt.Inquiry(8, [0x03], 0, b"")
PLEN_LOAD = 30


class Replay:
    """The serial side of a BtLink: answers START_STREAMING with an ACK and the
    captured bytes, STOP_STREAMING with an ACK. Reads return at most `chunk`
    bytes, so frames straddle reads as they do off the radio."""

    def __init__(self, stream: bytes, chunk: int):
        self.stream, self.chunk, self.buf, self.timeout = stream, chunk, bytearray(), 1.0

    @property
    def in_waiting(self):
        return min(len(self.buf), self.chunk)

    def reset_input_buffer(self):
        self.buf.clear()

    def write(self, data):
        if data == bytes([bt.START_STREAMING]):
            self.buf += bytes([bt.ACK]) + self.stream
        elif data == bytes([bt.STOP_STREAMING]):
            self.buf += bytes([bt.ACK])

    def read(self, n=1):
        n = min(n, self.chunk)
        out = bytes(self.buf[:n])
        del self.buf[:n]
        return out


def stream(data: bytes, inq: bt.Inquiry, chunk: int = 997) -> bt.StreamStats:
    link = bt.BtLink.__new__(bt.BtLink)
    link.hw, link._nstatus, link.crc_mode = bt.HW_SHIMMER3R, 2, 0
    link.ser = Replay(data, chunk)
    return link.stream(0.0, inq)


def fixture(name: str) -> bytes:
    return (DATA / name).read_bytes()


def steps(data: bytes, plen: int) -> list[int]:
    """Stamp steps of a capture framed by length alone, every frame whole."""
    ts = [bt._stamp(data, i) for i in range(0, len(data), plen)]
    return [b - a for a, b in zip(ts, ts[1:])]


class TheFixtures(unittest.TestCase):
    """Each fixture holds whole packets on the grid but for one late stamp, so
    the cases below test what they say they do."""

    def test_1024hz(self):
        data = fixture("s3r_1024hz_late_stamp.bin")
        self.assertEqual(len(data), 110 * PLEN_LOAD)
        s = steps(data, PLEN_LOAD)
        self.assertEqual(s[8:11], [32, 41, 23])
        self.assertEqual(s[:9] + s[11:], [32] * 107)

    def test_4096hz(self):
        data = fixture("s3r_4096hz_late_stamp.bin")
        self.assertEqual(len(data), 50 * 6)
        s = steps(data, 6)
        self.assertEqual(s[8:11], [8, 13, 3])
        self.assertEqual(s[:9] + s[11:], [8] * 47)


class LateStamp(unittest.TestCase):
    """A packet stamped off the grid, the packets after it back on it, is a
    packet with a late stamp: timing for the timing checks, not stray bytes."""

    def test_at_1024hz_one_late_stamp_is_jitter(self):
        st = stream(fixture("s3r_1024hz_late_stamp.bin"), LOAD)
        # Before: 14 packets, 2644 bytes that did not frame, 23611 samples "lost".
        self.assertEqual((st.packets, st.resyncs, st.misframes), (110, 0, []))
        self.assertEqual((st.gaps, st.missing, st.dupes), (0, 0, 0))
        self.assertEqual((st.jitter, st.max_jitter_ticks), (1, 9))

    def test_at_4096hz_one_late_stamp_is_jitter(self):
        st = stream(fixture("s3r_4096hz_late_stamp.bin"), VBATT_4096HZ)
        # Before: 49 packets, 6 bytes that did not frame, 1 sample "lost".
        self.assertEqual((st.packets, st.resyncs), (50, 0))
        self.assertEqual((st.gaps, st.missing, st.dupes), (0, 0, 0))
        self.assertEqual((st.jitter, st.max_jitter_ticks), (1, 5))

    def test_however_the_bytes_arrive(self):
        # The confirmation waits for the frames after the late one; it must
        # come out the same wherever a read ends.
        data = fixture("s3r_1024hz_late_stamp.bin")
        for chunk in (1, 7, 29, 30, 31, 61, 997, len(data) + 2):
            with self.subTest(chunk=chunk):
                st = stream(data, LOAD, chunk)
                self.assertEqual((st.packets, st.resyncs, st.jitter, st.max_jitter_ticks), (110, 0, 1, 9))

    def test_bt_throughput_fails_it_as_jitter(self):
        # smoke.bt_throughput judges the load stream with throughput.defects:
        # 9 ticks is over a quarter of the 32-tick period.
        st = stream(fixture("s3r_1024hz_late_stamp.bin"), LOAD)
        st.seconds = 110 / 1024
        rate = throughput.DataRate(bytes=5, seconds=1.0, packets=1, stopped=True, stop_ack=True)
        msg = "; ".join(throughput.defects(rate, st, LOAD))
        self.assertEqual(msg, "stream: stamp jitter up to 9 ticks (period 32), in 1 of 110 stamps")

    def test_within_a_quarter_period_bt_throughput_passes(self):
        st = bt.StreamStats(packets=1024, seconds=1.0, jitter=3, max_jitter_ticks=8)
        rate = throughput.DataRate(bytes=5, seconds=1.0, packets=1, stopped=True, stop_ack=True)
        self.assertEqual(throughput.defects(rate, st, LOAD), [])


class SearchAfterALostByte(unittest.TestCase):
    """After bytes that do not frame, the parser searches byte by byte, and a
    0x00 in the data with a plausible "stamp" after it is not enough: the two
    frames after a candidate must step on from it too."""

    def test_a_byte_lost_is_one_packet_of_stray_bytes(self):
        data = bytearray(fixture("s3r_1024hz_late_stamp.bin"))
        short = bytes(data[19 * PLEN_LOAD:20 * PLEN_LOAD])
        del data[19 * PLEN_LOAD + 1]      # the stamp's low byte, as DEV-1241 lost it on Shimmer3
        st = stream(bytes(data), LOAD)
        # Before: the search accepted a false frame in packet 25, then 2365
        # bytes did not frame and the stream "lost" thousands of samples.
        self.assertEqual(st.packets, 109)
        self.assertLess(st.resyncs, PLEN_LOAD)         # the short packet's 29 bytes, less any 0xFF read as an ACK
        self.assertTrue(st.misframes[0].startswith((short[:1] + short[2:]).hex()))
        self.assertEqual((st.gaps, st.missing, st.dupes), (1, 1, 0))
        self.assertEqual((st.jitter, st.max_jitter_ticks), (1, 9))   # the late stamp, still judged


def packet(ts: int) -> bytes:
    """A synthetic load packet: no 0x00 or 0xFF in its data."""
    return bytes([0]) + (ts % bt.TS_WRAP).to_bytes(3, "little") + bytes(range(0x10, 0x10 + 26))


class SyntheticStamps(unittest.TestCase):
    def test_a_repeated_stamp_is_a_duplicate(self):
        # Its own slot is then empty, so one sample counts as lost too. Before,
        # the packet was 30 bytes that did not frame.
        ts = [1000 + 32 * i for i in range(10)]
        ts[5] = ts[4]
        st = stream(b"".join(packet(t) for t in ts), LOAD)
        self.assertEqual((st.packets, st.resyncs, st.dupes, st.gaps, st.missing), (10, 0, 1, 1, 1))

    def test_a_late_stamp_across_the_counter_wrap(self):
        ts = [bt.TS_WRAP - 5 * 32 + 32 * i for i in range(10)]
        ts[5] += 9
        st = stream(b"".join(packet(t) for t in ts), LOAD)
        self.assertEqual((st.packets, st.resyncs, st.jitter, st.max_jitter_ticks), (10, 0, 1, 9))

    def test_a_late_stamp_with_no_frames_after_it_to_confirm_it(self):
        # The second-last packet of a stream: nothing can show the grid goes
        # on, so it is judged on its step alone, as before.
        ts = [1000 + 32 * i for i in range(10)]
        ts[8] += 9
        st = stream(b"".join(packet(t) for t in ts), LOAD)
        self.assertEqual((st.packets, st.resyncs), (9, PLEN_LOAD))

    def test_a_stamp_after_the_next_is_not_a_late_stamp(self):
        # Stamped after its successor: not a sample stamped late. Rejected.
        ts = [1000 + 32 * i for i in range(10)]
        ts[5] = ts[6] + 9
        st = stream(b"".join(packet(t) for t in ts), LOAD)
        self.assertEqual((st.packets, st.resyncs), (9, PLEN_LOAD))


if __name__ == "__main__":
    unittest.main()
