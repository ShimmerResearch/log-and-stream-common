#!/usr/bin/env python3
"""Unit tests for rig.midstream and the mid-stream commands of rig.bt's stream:
how the stamps around an SD-logging start are judged. Run:
python Test/hil/test_hil_midstream.py (no hardware).

The stamp sequences are shaped like the ones seen on hardware: a step ahead of
the grid followed by one well behind it, and a forward step that leaves the
grid moved. A miscount here passes a stream whose host would have read a
backward step as a 512 s counter wrap.

No hardware and no pyserial needed: rig.bt imports pyserial, so where it is not
installed (CI) a stand-in is put in its place, as test_hil_timing.py does.
"""

import sys
import time
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

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

from rig import bt, midstream  # noqa: E402
from rig.dock import crc16  # noqa: E402

P = 128         # 256 Hz


def stamps(steps, first=15025354):
    ts = [first]
    for d in steps:
        ts.append((ts[-1] + d) % bt.TS_WRAP)
    return ts


class Analyse(unittest.TestCase):
    def test_clean(self):
        g = midstream.analyse(stamps([P] * 500), P)
        self.assertTrue(g.clean)
        self.assertEqual((g.backward, g.shifts), ([], []))

    def test_clean_across_counter_wrap(self):
        g = midstream.analyse(stamps([P] * 100, first=bt.TS_WRAP - 50 * P), P)
        self.assertTrue(g.clean, g.issues)

    def test_ahead_then_behind(self):
        # A capture of a log start after a clock set: +644, then -2046, then
        # on a new grid. The backward step is reported where it happened.
        g = midstream.analyse(stamps([P] * 20 + [644, -2046] + [P] * 20), P)
        self.assertEqual(g.backward, [(22, -2046)])
        self.assertFalse(g.clean)
        text = dict(g.issues)[22]
        self.assertIn("2046 ticks behind", text)

    def test_zero_step_is_backward(self):
        g = midstream.analyse(stamps([P] * 5 + [0] + [P] * 5), P)
        self.assertEqual(g.backward, [(6, 0)])

    def test_forward_clock_step_moves_the_grid(self):
        # A clock set 600 ticks into the future at one start: by the stamps,
        # three samples skipped and the grid moved 88 ticks late. Judged on the
        # moved grid, the stamps after it are not jitter.
        g = midstream.analyse(stamps([P] * 20 + [600] + [P] * 40), P)
        self.assertEqual(g.shifts, [(21, 88)])
        self.assertEqual(g.backward, [])
        self.assertEqual(g.stats.missing, 3)
        self.assertEqual(g.stats.jitter, 0)          # judged on the moved grid, the step is whole periods
        self.assertEqual(len(g.issues), 1)
        self.assertIn("grid moved +88", g.issues[0][1])

    def test_small_move_without_loss(self):
        g = midstream.analyse(stamps([P] * 20 + [P + 6] + [P] * 20), P)
        self.assertEqual(g.shifts, [(21, 6)])
        self.assertEqual(g.stats.missing, 0)
        self.assertFalse(g.clean)

    def test_move_back_is_negative(self):
        g = midstream.analyse(stamps([P] * 20 + [P - 20] + [P] * 20), P)
        self.assertEqual(g.shifts, [(21, -20)])

    def test_one_late_stamp_is_jitter_not_a_move(self):
        # Late by a quarter period and back: within the smoke rule, so clean.
        g = midstream.analyse(stamps([P] * 10 + [P + 32, P - 32] + [P] * 10), P)
        self.assertEqual(g.shifts, [])
        self.assertTrue(g.clean, g.issues)
        # Later than that is an issue, but still not a move.
        g = midstream.analyse(stamps([P] * 10 + [P + 60, P - 60] + [P] * 10), P)
        self.assertEqual(g.shifts, [])
        self.assertEqual(g.issues, [(11, "stamp 60 ticks off its slot")])

    def test_a_stamp_three_quarters_late_is_jitter_not_loss(self):
        # Counted first in the next slot, the one before lost; the stamp after
        # it, in that same slot, shows it was late (classify_step).
        g = midstream.analyse(stamps([P] * 10 + [P + 96, P - 96] + [P] * 10), P)
        self.assertEqual(g.shifts, [])
        self.assertEqual((g.stats.missing, g.stats.dupes), (0, 0))
        self.assertEqual(g.issues, [(11, "stamp 96 ticks off its slot")])

    def test_one_tick_quantisation_is_clean(self):
        g = midstream.analyse(stamps([P, P + 1, P - 1] * 30), P)
        self.assertTrue(g.clean, g.issues)

    def test_move_near_the_end_needs_stamps_to_settle(self):
        # Off the grid on the last stamp: nothing after it, so not a move.
        g = midstream.analyse(stamps([P] * 10 + [P + 10]), P)
        self.assertEqual(g.shifts, [])
        # One or two stamps after it agreeing is still too few to call it a move.
        for tail in (1, 2):
            g = midstream.analyse(stamps([P] * 10 + [P + 10] + [P] * tail), P)
            self.assertEqual(g.shifts, [], tail)
        g = midstream.analyse(stamps([P] * 10 + [P + 10] + [P] * 3), P)
        self.assertEqual(g.shifts, [(11, 10)])

    def test_lost_sample(self):
        g = midstream.analyse(stamps([P] * 10 + [2 * P] + [P] * 10), P)
        self.assertEqual(g.stats.missing, 1)
        self.assertEqual(g.shifts, [])
        self.assertEqual(g.issues, [(11, f"1 samples lost (step {2 * P} ticks)")])


class ByCycle(unittest.TestCase):
    def test_issues_go_to_the_command_before_them(self):
        g = midstream.analyse(stamps([P] * 30 + [644, -2046] + [P] * 30 + [P + 6] + [P] * 30), P)
        events = [("start 1", 10, 0.0), ("status 1", 15, 0.0), ("stop 1", 20, 0.0),
                  ("set_rwc 2", 25, 0.0), ("start 2", 26, 0.0), ("stop 2", 60, 0.0)]
        groups = midstream.by_cycle(g, events)
        self.assertEqual(sorted(groups), ["cycle 2 start", "cycle 2 stop"])
        self.assertIn("2046 ticks behind", " ".join(groups["cycle 2 start"]))

    def test_before_any_command(self):
        g = midstream.analyse(stamps([P] * 3 + [2 * P] + [P] * 10), P)
        self.assertEqual(list(midstream.by_cycle(g, [("start 1", 8, 0.0)])), ["stream"])


class Schedule(unittest.TestCase):
    def test_cycles_alternate(self):
        plan = midstream.schedule(4, lead_s=3, on_s=2, off_s=2, rwc_gap_s=0.2, status_after_s=1, tail_s=2)
        self.assertEqual(plan.rwc_cycles, [2, 4])
        kinds = [(a.cycle, a.kind) for a in plan.actions]
        self.assertEqual(kinds[:7], [(1, "start"), (1, "status"), (1, "stop"),
                                     (2, "set_rwc"), (2, "start"), (2, "status"), (2, "stop")])
        times = [a.t for a in plan.actions]
        self.assertEqual(times, sorted(times))
        start2 = next(a for a in plan.actions if a.label == "start 2")
        self.assertAlmostEqual(start2.t, 7.2)
        self.assertAlmostEqual(plan.seconds, max(times) + 2)
        self.assertEqual(sum(a.acked for a in plan.actions), 4 * 2 + 2)

    def test_rwc_payload(self):
        b = midstream.rwc_payload(1.5)
        self.assertEqual(b[0], 0x8F)
        self.assertEqual(int.from_bytes(b[1:], "little"), int(1.5 * 32768))
        self.assertEqual(len(b), 9)

    def test_only_the_last_clock_set_is_the_probe(self):
        plan = midstream.schedule(6)
        self.assertEqual(plan.rwc_cycles, [2, 4, 6])
        self.assertEqual(plan.probe_s, midstream.PROBE_S)
        sets = [a for a in plan.actions if a.kind == "set_rwc"]
        now = time.time()
        sent = [int.from_bytes(a.bytes()[1:], "little") / 32768 - now for a in sets]
        self.assertLess(abs(sent[0]), 1.0)
        self.assertLess(abs(sent[1]), 1.0)
        self.assertAlmostEqual(sent[2], midstream.PROBE_S, delta=1.0)

    def test_no_probe(self):
        plan = midstream.schedule(4, probe=False)
        self.assertEqual(plan.probe_s, 0.0)
        now = time.time()
        for a in plan.actions:
            if a.kind == "set_rwc":
                self.assertLess(abs(int.from_bytes(a.bytes()[1:], "little") / 32768 - now), 1.0)

    def test_no_clock_set_no_probe(self):
        self.assertEqual(midstream.schedule(1).probe_s, 0.0)


class FakeClock:
    """A device clock read and set over Bluetooth: it reads the host's time plus
    offset_s, and a set takes at once unless ignore_sets."""

    def __init__(self, offset_s: float, ignore_sets: bool = False):
        self.offset_s = offset_s
        self.ignore_sets = ignore_sets
        self.sets = []

    def get_rwc(self) -> int:
        return int((time.time() + self.offset_s) * 32768)

    def set_rwc(self, ticks: int) -> None:
        self.sets.append(ticks)
        if not self.ignore_sets:
            self.offset_s = ticks / 32768 - time.time()


class ClockAfterStop(unittest.TestCase):
    def test_a_deferred_set_applied_at_the_stop(self):
        link = FakeClock(midstream.PROBE_S - 0.02)
        problems, note = midstream.clock_after_stop(link, midstream.PROBE_S)
        self.assertEqual(problems, [])
        self.assertEqual(len(link.sets), 1, "the clock is put back once")
        self.assertLess(abs(link.offset_s), 0.01)
        self.assertIn("after the stop", note)

    def test_a_set_lost_at_the_stop(self):
        problems, _ = midstream.clock_after_stop(FakeClock(-0.02), midstream.PROBE_S)
        self.assertEqual(len(problems), 1)
        self.assertIn("not applied at the stop", problems[0])

    def test_an_idle_set_that_does_not_take(self):
        problems, _ = midstream.clock_after_stop(FakeClock(midstream.PROBE_S, ignore_sets=True),
                                                 midstream.PROBE_S)
        self.assertEqual(len(problems), 1)
        self.assertIn("while idle", problems[0])

    def test_offset_is_against_the_middle_of_the_exchange(self):
        self.assertAlmostEqual(midstream.clock_offset(FakeClock(1.5)), 1.5, delta=0.01)


class FakeSerial:
    """Hands back a script of device bytes once the stream's start is written,
    and the stop's reply once the stop is."""

    def __init__(self, script: bytes, after_stop: bytes = b""):
        self.script = script
        self.after_stop = after_stop
        self.buf = bytearray()
        self.writes = []
        self.timeout = 1.0

    @property
    def in_waiting(self):
        return len(self.buf)

    def write(self, b):
        self.writes.append(bytes(b))
        if b[0] == bt.START_STREAMING:
            self.buf += self.script
        elif b[0] == bt.STOP_STREAMING:
            self.buf += self.after_stop

    def read(self, n):
        out = bytes(self.buf[:n])
        del self.buf[:n]
        return out

    def reset_input_buffer(self):
        self.buf.clear()

    def close(self):
        pass


def packet(ts: int) -> bytes:
    body = bytes([0x00]) + ts.to_bytes(3, "little") + b"\x34\x12"
    c = crc16(body)
    return body + bytes([c & 0xFF, c >> 8])


def with_crc(frame: bytes) -> bytes:
    c = crc16(frame)
    return frame + bytes([c & 0xFF, c >> 8])


ACK_CRC = with_crc(bytes([bt.ACK]))         # an ACK with its mode-2 CRC
NACK_CRC = with_crc(bytes([bt.NACK]))       # a NACK with its mode-2 CRC


def bare_packet(ts: int) -> bytes:
    """packet() without its link CRC, for a stream with the CRC off."""
    return bytes([0x00]) + ts.to_bytes(3, "little") + b"\x34\x12"


def crc_link(script: bytes, crc_mode: int = 2):
    link = bt.BtLink.__new__(bt.BtLink)
    after = ACK_CRC if crc_mode else bytes([bt.ACK])
    link.ser, link.hw, link.crc_mode, link._nstatus = FakeSerial(script, after), bt.HW_SHIMMER3R, crc_mode, 2
    return link


class StreamActions(unittest.TestCase):
    def test_mid_stream_replies_are_counted_not_framed(self):
        ts = stamps([P] * 9)
        logging_push = with_crc(bytes([bt.ACK, bt.INSTREAM, bt.STATUS_RSP, bt.STATUS_LOGGING | bt.STATUS_SENSING, 0]))
        script = (ACK_CRC + b"".join(packet(t) for t in ts[:4]) + ACK_CRC
                  + b"".join(packet(t) for t in ts[4:7]) + logging_push
                  + b"".join(packet(t) for t in ts[7:]))
        ser = FakeSerial(script, after_stop=ACK_CRC)
        link = bt.BtLink.__new__(bt.BtLink)
        link.ser, link.hw, link.crc_mode, link._nstatus = ser, bt.HW_SHIMMER3R, 2, 2
        inq = bt.Inquiry(P, [0x00], 1, b"")
        acts = [midstream.Action(0.0, "start", 1, bytes([bt.START_LOGGING])),
                midstream.Action(0.0, "status", 1, bytes([bt.GET_STATUS]))]
        st = link.stream(0.05, inq, keep=True, actions=acts)
        self.assertEqual(st.packets, len(ts))
        self.assertEqual(st.ts, ts)
        self.assertEqual((st.resyncs, st.crc_errors), (0, 0))
        self.assertEqual(st.acks, 1)
        self.assertEqual(st.pushed, [bt.STATUS_LOGGING | bt.STATUS_SENSING])
        self.assertEqual(midstream.logging_seen(st), 1)
        self.assertTrue(st.stop_ack)
        self.assertEqual([e[0] for e in st.events], ["start 1", "status 1"])
        self.assertIn(bytes([bt.START_LOGGING]), ser.writes)
        self.assertIn(bytes([bt.GET_STATUS]), ser.writes)

    def test_replies_with_a_bad_crc_are_not_believed(self):
        # A status frame whose CRC fails could carry a flipped logging bit, so
        # it is counted as a CRC error, not as a status; so is a bad ACK.
        ts = stamps([P] * 5)
        bad_push = bytes([bt.ACK, bt.INSTREAM, bt.STATUS_RSP, bt.STATUS_LOGGING, 0, 0, 0])
        bad_ack = bytes([bt.ACK, 0, 0])
        script = (ACK_CRC + b"".join(packet(t) for t in ts[:2]) + bad_ack
                  + b"".join(packet(t) for t in ts[2:4]) + bad_push + packet(ts[4]))
        link = bt.BtLink.__new__(bt.BtLink)
        link.ser, link.hw, link.crc_mode, link._nstatus = FakeSerial(script, ACK_CRC), bt.HW_SHIMMER3R, 2, 2
        st = link.stream(0.05, bt.Inquiry(P, [0x00], 1, b""))
        self.assertEqual(st.packets, 5)
        self.assertEqual((st.crc_errors, st.acks, st.pushed), (2, 0, []))
        self.assertTrue(st.stop_ack)

    def test_a_refused_start_is_a_nack_not_stray_bytes(self):
        # Firmware since DEV-1431 NACKs a log start it cannot carry out, here
        # between two packets. The packet before it still frames, the NACK and
        # its CRC are one reply, and the stream goes on behind them.
        ts = stamps([P] * 6)
        script = (ACK_CRC + b"".join(packet(t) for t in ts[:3]) + NACK_CRC
                  + b"".join(packet(t) for t in ts[3:]))
        link = crc_link(script)
        acts = [midstream.Action(0.0, "start", 1, bytes([bt.START_LOGGING]))]
        st = link.stream(0.05, bt.Inquiry(P, [0x00], 1, b""), keep=True, actions=acts)
        self.assertEqual(st.ts, ts)
        self.assertEqual((st.resyncs, st.crc_errors, st.acks, st.nacks), (0, 0, 0, 1))
        self.assertTrue(st.stop_ack)

    def test_a_nack_without_a_link_crc(self):
        ts = stamps([P] * 6)
        script = (bytes([bt.ACK]) + b"".join(bare_packet(t) for t in ts[:3]) + bytes([bt.NACK])
                  + b"".join(bare_packet(t) for t in ts[3:]))
        st = crc_link(script, crc_mode=0).stream(0.05, bt.Inquiry(P, [0x00], 1, b""), keep=True)
        self.assertEqual(st.ts, ts)
        self.assertEqual((st.resyncs, st.acks, st.nacks), (0, 0, 1))

    def test_a_nack_with_a_bad_crc_is_not_believed(self):
        ts = stamps([P] * 4)
        script = (ACK_CRC + b"".join(packet(t) for t in ts[:2]) + bytes([bt.NACK, 0, 0])
                  + b"".join(packet(t) for t in ts[2:]))
        st = crc_link(script).stream(0.05, bt.Inquiry(P, [0x00], 1, b""))
        self.assertEqual(st.packets, len(ts))
        self.assertEqual((st.crc_errors, st.nacks), (1, 0))

    def test_judge_takes_a_nack_as_a_refused_start(self):
        st = bt.StreamStats()
        st.ts = stamps([P] * 60)
        st.events = [("set_rwc 1", 10, 0.0), ("start 1", 10, 0.0), ("stop 1", 40, 0.0)]
        plan = midstream.Plan([midstream.Action(0, "set_rwc", 1, b""), midstream.Action(0, "start", 1, b""),
                               midstream.Action(0, "stop", 1, b"")], 60 * P / bt.TICKS_PER_S, [1])
        st.acks, st.nacks = 2, 1            # the clock set and the stop ACKed, the start refused
        problems, summary = midstream.judge(st, plan, P)
        self.assertEqual(problems, [])
        self.assertIn("1 log starts (1 refused with a NACK);", summary)
        st.acks, st.nacks = 1, 2            # only a log start may be refused
        problems, _ = midstream.judge(st, plan, P)
        self.assertEqual(problems, ["2 NACKs mid-stream, but only the 1 log starts may be refused"])
        st.acks, st.nacks = 1, 1            # a reply missing
        problems, _ = midstream.judge(st, plan, P)
        self.assertEqual(problems, ["1 ACKs and 1 NACKs for the 3 commands sent mid-stream"])

    def test_judge_names_the_cycle(self):
        st = bt.StreamStats()
        st.ts = stamps([P] * 30 + [644, -2046] + [P] * 30)
        st.events = [("set_rwc 2", 25, 0.0), ("start 2", 26, 0.0)]
        st.acks = 2
        plan = midstream.Plan([midstream.Action(0, "set_rwc", 2, b""), midstream.Action(0, "start", 2, b"")],
                              60 * P / bt.TICKS_PER_S, [2])
        problems, summary = midstream.judge(st, plan, P)
        self.assertEqual(len(problems), 1)
        self.assertTrue(problems[0].startswith("cycle 2 start: "))
        self.assertIn("1 of 1 starts after a clock set", summary)


    def test_too_few_packets_is_a_problem(self):
        for ts in ([], stamps([]), stamps([P] * 5)):
            st = bt.StreamStats()
            st.ts = ts
            plan = midstream.Plan([], 10.0, [])
            problems, _ = midstream.judge(st, plan, P)
            self.assertEqual(len(problems), 1, problems)
            self.assertIn(f"{len(ts)} packets in a 10 s stream", problems[0])

if __name__ == "__main__":
    unittest.main()
