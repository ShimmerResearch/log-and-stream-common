#!/usr/bin/env python3
"""Unit tests for rig.events: byte 232 as the firmware keeps it, the event
file checks the release rows make, and the Bluetooth frame scanner. Run:
python Test/hil/test_hil_events.py (no hardware).

The byte's rules are checked against docs/SHIMMER3_CONFIGURATION_INFOMEM.md
§8.3 itself, the table of every value the firmware keeps, so the rig and the
document cannot drift apart unnoticed.
"""

import re
import struct
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from rig import events  # noqa: E402

evf = events.evf
REPO = HERE.parents[1]
T = events.TICKS_PER_S
OPEN_TS = 0x0000_0123_4567_89AB
HW_S3R = 10


def kept_table():
    """§8.3's table: {value: userButtonEnable held at (None = unchanged)}."""
    doc = (REPO / "docs" / "SHIMMER3_CONFIGURATION_INFOMEM.md").read_text(encoding="utf-8")
    sec = doc.split("### 8.3 ", 1)[1].split("\n### ", 1)[0]
    out = {}
    for m in re.finditer(r"^\| `0x([0-9A-F]{2})` \|.*\| (Unchanged|0|1) \|$", sec, re.M):
        out[int(m.group(1), 16)] = None if m.group(2) == "Unchanged" else int(m.group(2))
    return out


def data_header(ts: int) -> bytes:
    """A Shimmer3R data file header with this initial timestamp (SD_CARD_FORMAT §3.3)."""
    b = bytearray(384)
    b[251], b[255], b[254], b[253], b[252] = (ts >> 32) & 0xFF, (ts >> 24) & 0xFF, (ts >> 16) & 0xFF, \
        (ts >> 8) & 0xFF, ts & 0xFF
    b[44], b[45], b[46] = (ts >> 56) & 0xFF, (ts >> 48) & 0xFF, (ts >> 40) & 0xFF
    return bytes(b)


def rec(rtype, seq, ts, *, flags=0, channel=0xFF, state=0, dropped=0) -> bytes:
    return struct.pack(evf.RECORD_FMT, rtype, flags, seq, ts, channel, state, dropped)


def event_file(records, *, enables=events.ENABLE_BUTTON | events.ENABLE_TTL, open_ts=OPEN_TS,
               reason=events.OPEN_SESSION_START, hw=HW_S3R) -> bytes:
    head = struct.pack(evf.HEADER_FMT, b"SEVT", 1, 64, 16, hw, open_ts, 0, 0, 3, 1, 1, 25, 0, reason, enables)
    return head.ljust(64, b"\0") + b"".join(records)


def quiet(seconds: float, beat_every: float = 60.0) -> list[bytes]:
    """A file nobody touched: the initial state, a heartbeat per minute, the final record."""
    out = [rec(events.TYPE_INIT, 0, OPEN_TS)]
    t = beat_every
    while t < seconds:
        out.append(rec(events.TYPE_HEARTBEAT, len(out), OPEN_TS + int(t * T)))
        t += beat_every
    out.append(rec(events.TYPE_HEARTBEAT, len(out), OPEN_TS + int(seconds * T), flags=evf.FLAG_FINAL))
    return out


class Byte232(unittest.TestCase):
    def test_the_documents_table_is_read(self):
        # 16 values, each also with the TTL bit: the table lists the 16
        self.assertEqual(len(kept_table()), 16)

    def test_every_value_in_the_table_is_kept_with_its_userbuttonenable(self):
        for v, held in kept_table().items():
            for ttl in (0, events.TTL):
                with self.subTest(value=hex(v | ttl)):
                    self.assertEqual(events.correct(v | ttl), v | ttl)
                    self.assertEqual(events.ube_held(v | ttl), held)

    def test_nothing_outside_the_table_is_kept(self):
        kept = {v | ttl for v in kept_table() for ttl in (0, events.TTL)}
        for v in range(256):
            with self.subTest(value=hex(v)):
                self.assertEqual(events.correct(v) == v, v in kept)
                self.assertIn(events.correct(v), kept)          # and it corrects to a kept value

    def test_the_documents_rules_for_what_is_not_kept(self):
        self.assertEqual(events.correct(0x80), 0)               # bit 7: the whole byte
        self.assertEqual(events.correct(0xFF), 0)               # erased memory
        self.assertEqual(events.correct(0xA4), 0)               # TTL too
        self.assertEqual(events.correct(0x26), 0x20)            # reserved mode: TTL stays
        self.assertEqual(events.correct(0x19), 0x00)            # reserved trigger
        self.assertEqual(events.correct(0x28), 0x20)            # triggers under legacy
        self.assertEqual(events.correct(0x41), 0x01)            # bit 6 cleared

    def test_sr48_6_0_has_no_presses_to_record_and_no_ttl_input(self):
        self.assertEqual(events.correct(0x24, sr48_6_0=True), 0x03)
        self.assertEqual(events.correct(0x05, sr48_6_0=True), 0x01)
        self.assertEqual(events.correct(0x2D, sr48_6_0=True), 0x09)
        self.assertEqual(events.correct(0x20, sr48_6_0=True), 0x00)
        self.assertEqual(events.correct(0x11, sr48_6_0=True), 0x11)

    def test_what_each_value_does(self):
        self.assertEqual(events.enables(0x24), events.ENABLE_BUTTON | events.ENABLE_TTL)
        self.assertEqual(events.enables(0x20), events.ENABLE_TTL)
        self.assertEqual(events.enables(0x03), 0)
        self.assertTrue(events.undock_starts(0x09, 0))
        self.assertTrue(events.undock_starts(0x00, 0))          # legacy: userButtonEnable off
        self.assertFalse(events.undock_starts(0x00, 1))
        self.assertFalse(events.undock_starts(0x24, 1))
        self.assertTrue(events.logging_off(0x11))
        self.assertFalse(events.logging_off(0x01))


class EventFile(unittest.TestCase):
    def check(self, blob: bytes, *, data_ts=OPEN_TS, span=None, enables=None, lead=0.0):
        """lead: how long before its open the data file's first sample is; span,
        the data file's from that sample (default: the event file's plus lead)."""
        h, recs = evf.parse(blob)
        span = (recs[-1]["ts"] - recs[0]["ts"]) / T + lead if span is None else span
        first = (data_ts - round(lead * T)) & 0xFFFFFF
        with tempfile.TemporaryDirectory() as d:
            p = Path(d, "000")
            p.write_bytes(blob)
            return events.check_file(p, data_header(data_ts), hw_id=HW_S3R, data_first_ts=first,
                                     data_span_s=span,
                                     want_enables=h["enables"] if enables is None else enables)

    def test_a_quiet_three_minute_file_passes(self):
        fc = self.check(event_file(quiet(180)))
        self.assertEqual(fc.problems, [])
        self.assertIn("2 heartbeats", fc.note)

    def test_heartbeats_two_syncs_apart_pass(self):
        # The first sync can come just short of 60 s, so the first heartbeat at the second
        self.assertEqual(self.check(event_file(quiet(180, beat_every=119))).problems, [])

    def test_no_file(self):
        with tempfile.TemporaryDirectory() as d:
            fc = events.check_file(Path(d, "000"), data_header(OPEN_TS), want_enables=1, hw_id=HW_S3R,
                                   data_first_ts=OPEN_TS & 0xFFFFFF, data_span_s=30)
        self.assertIn("no event file", fc.problems[0])

    def test_cut_short(self):
        recs = quiet(30)[:-1]
        self.assertTrue(any("final record" in p for p in self.check(event_file(recs)).problems))

    def test_an_edge_with_nothing_connected(self):
        recs = quiet(30)
        recs.insert(1, rec(events.TYPE_EDGE, 1, OPEN_TS + 5 * T, channel=1, state=2))
        recs[2:] = [r[:2] + struct.pack("<H", i + 2) + r[4:] for i, r in enumerate(recs[2:])]
        self.assertTrue(any("1 edge" in p for p in self.check(event_file(recs)).problems))

    def test_a_dropped_count(self):
        recs = quiet(30)
        recs[-1] = rec(events.TYPE_HEARTBEAT, 1, OPEN_TS + 30 * T, flags=evf.FLAG_FINAL, dropped=3)
        self.assertTrue(any("dropped" in p for p in self.check(event_file(recs)).problems))

    def test_too_long_without_a_record(self):
        probs = self.check(event_file(quiet(180, beat_every=130))).problems
        self.assertTrue(any("between records" in p for p in probs), probs)

    def test_a_long_file_without_a_heartbeat(self):
        recs = [rec(events.TYPE_INIT, 0, OPEN_TS),
                rec(events.TYPE_HEARTBEAT, 1, OPEN_TS + 140 * T, flags=evf.FLAG_FINAL)]
        probs = self.check(event_file(recs)).problems
        self.assertTrue(any("no heartbeat" in p for p in probs), probs)

    def test_the_inputs_byte_232_enables(self):
        probs = self.check(event_file(quiet(30)), enables=events.ENABLE_BUTTON).problems
        self.assertTrue(any("enables" in p for p in probs), probs)

    def test_another_data_files_events(self):
        probs = self.check(event_file(quiet(30)), data_ts=OPEN_TS + 1).problems
        self.assertTrue(any("not this data file" in p for p in probs), probs)

    def test_it_closes_with_the_data_file(self):
        probs = self.check(event_file(quiet(30)), span=25.0).problems
        self.assertTrue(any("spans" in p for p in probs), probs)

    def test_a_data_file_that_opened_after_its_first_sample(self):
        # Sampling starts at the start command; the file opens once its session
        # folder is made, 3.9 s later in a trial folder of 531 sessions (bench,
        # 2026-11-08). The event file starts at the open, so a 30 s event file
        # pairs with 33.9 s of samples.
        fc = self.check(event_file(quiet(30)), lead=3.9)
        self.assertEqual(fc.problems, [])
        self.assertIn("opened 3.9 s after its first sample", fc.note)
        probs = self.check(event_file(quiet(30)), lead=3.9, span=30.0).problems
        self.assertTrue(any("spans 30.0 s, the data file 26.1 s" in p for p in probs), probs)

    def test_a_data_file_whose_first_sample_is_after_its_open(self):
        probs = self.check(event_file(quiet(30)), lead=-1.0).problems
        self.assertTrue(any("before its open, or after it" in p for p in probs), probs)

    def test_the_initial_state_comes_first_with_nothing_pressed_or_high(self):
        recs = quiet(30)
        recs[0] = rec(events.TYPE_INIT, 0, OPEN_TS, state=events.STATE_TTL)
        probs = self.check(event_file(recs)).problems
        self.assertTrue(any("TTL input high" in p for p in probs), probs)

    def test_a_split_file_is_not_a_sessions_first(self):
        probs = self.check(event_file(quiet(30), reason=2)).problems
        self.assertTrue(any("reason 2" in p for p in probs), probs)

    def test_companion_path(self):
        root = Path("E:/")
        self.assertEqual(events.companion_path(root / "data/exp_1/Shim-003/000", root),
                         root / "events/exp_1/Shim-003/000")


# The worked frame in SHIMMER3_BT_COMMUNICATION_PROTOCOL.md §7.15, pinned by
# Test/host/test_event_record.c.
DOC_FRAME = bytes.fromhex("8ABB10 0100 0100 3353183772350000 00 01 0000 0920".replace(" ", ""))


class Frames(unittest.TestCase):
    def test_the_documents_frame(self):
        recs, bad, other = events.scan(DOC_FRAME)
        self.assertEqual((bad, other), (0, b""))
        self.assertEqual(len(recs), 1)
        r = recs[0]
        self.assertEqual((r["type"], r["seq"], r["ts"], r["channel"], r["state"]),
                         (events.TYPE_EDGE, 1, 0x357237185333, 0, 1))

    def test_found_among_other_traffic(self):
        status_push = bytes([0xFF, 0x8A, 0x71, 0x21, 0x00])
        lookalike = bytes([0x8A, 0xBB, 0x10]) + bytes(18)      # a CRC that fails
        recs, bad, other = events.scan(status_push + lookalike + DOC_FRAME + DOC_FRAME)
        self.assertEqual((len(recs), bad), (2, 1))
        self.assertTrue(other.startswith(status_push))

    def test_a_cut_frame_is_not_a_frame(self):
        recs, bad, other = events.scan(DOC_FRAME[:-1])
        self.assertEqual((recs, bad), ([], 0))
        self.assertEqual(other, DOC_FRAME[:-1])

    def test_status(self):
        s = events.Status.parse(bytes([3, 0x09, 0, 1, 0, 0, 0, 2, 0]))
        self.assertEqual((s.enables, s.next_seq, s.frames_lost), (3, 1, 2))
        self.assertTrue(s.flags & events.STATUS_TTL_ARMED)
        with self.assertRaises(ValueError):
            events.Status.parse(bytes(8))


try:
    import tomllib as _toml
except ModuleNotFoundError:     # pragma: no cover
    try:
        import tomli as _toml
    except ModuleNotFoundError:
        _toml = None


@unittest.skipIf(_toml is None, "needs tomllib (3.11+) or tomli")
class S3rMatrixEventRows(unittest.TestCase):
    def test_event_rows_log_and_record_an_input(self):
        with open(HERE / "matrix" / "s3r_release.toml", "rb") as f:
            rows = _toml.load(f)["row"]
        ev = [r for r in rows if "button_log_mode" in r]
        self.assertGreaterEqual(len(ev), 3)
        for r in ev:
            v = r["button_log_mode"]
            with self.subTest(row=r["id"]):
                self.assertIn(r["mode"], ("log", "both"))       # a stream writes no event file
                self.assertEqual(events.correct(v), v)          # kept as written
                self.assertTrue(events.enables(v))              # so an event file is due
                self.assertFalse(events.undock_starts(v, 1))    # the row starts its own log
                self.assertFalse(events.logging_off(v))
        self.assertEqual({events.enables(r["button_log_mode"]) for r in ev},
                         {events.ENABLE_BUTTON, events.ENABLE_TTL, events.ENABLE_BUTTON | events.ENABLE_TTL})
        # A heartbeat is due in one of them
        self.assertTrue(any(r["seconds"] >= events.HEARTBEAT_BY_S for r in ev))


if __name__ == "__main__":
    unittest.main(verbosity=2)
