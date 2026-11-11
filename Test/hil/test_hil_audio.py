#!/usr/bin/env python3
"""Unit tests for rig.s3r_audio: the release rows' checks on a session's audio
files, and rig.sound_level's on the sound-level channel. Run:
python Test/hil/test_hil_audio.py (no hardware, standard library only).

Each case builds a card's audio/ tree in a temporary directory, with files laid
out as docs/SHIMMER3R_AUDIO.md §3 describes, then breaks one thing.
"""

import random
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rig import s3r_audio as sa  # noqa: E402
from rig import sound_level as sl  # noqa: E402

TICKS = 32768
MAC = bytes.fromhex("D3B415BC0000")
FW = (3, 1, 1, 25)
CONFIG_TIME = 1792712946
INITIAL_TS = 50 * 60 * TICKS + 12345          # the data file's 40-bit initial timestamp
LOG_S = 30.0


def data_file(mode, initial=INITIAL_TS, log_s=LOG_S):
    """The data file as rig.s3r_audio sees it: its first record 0.85 s before
    the file opened (the packet ring), its last at the log's end."""
    first = initial - int(0.85 * TICKS)
    return sa.DataFile(initial_ts=initial, config_time=CONFIG_TIME, mac=MAC.hex().upper(), fw=FW,
                       audio_mode=mode, first_ts24=first % (1 << 24), span_ticks=int(log_s * TICKS))


def identity(mode, first_ticks, closed=1, fault=0, adf=(280, 280, 0)):
    h = bytearray(512)
    h[44:48] = b"SAUD"
    h[48], h[49], h[50], h[51] = 1, 10, mode, 0
    struct.pack_into("<H", h, 52, 0)
    struct.pack_into("<QI", h, 56, INITIAL_TS, CONFIG_TIME)
    struct.pack_into("<HHBB", h, 68, *FW)
    h[74:80] = MAC
    struct.pack_into("<QIi", h, 80, first_ticks, 24, 6)
    h[96], h[97] = closed, fault
    struct.pack_into("<III", h, 120, *adf)
    return h


def wav_bytes(mode, first, n, *, closed=1, silent=False, seed=1):
    h = identity(mode, first, closed)
    struct.pack_into("<4sI4s4sIHHIIHH4sI", h, 0, b"RIFF", 504 + 2 * n, b"WAVE", b"fmt ", 16, 1, 1,
                     16000, 32000, 2, 16, b"shim", 460)
    last = first + round((n - 1) * TICKS / 16000)
    struct.pack_into("<IQI", h, 100, n, last, 0)
    h[504:508] = b"data"
    struct.pack_into("<I", h, 508, 2 * n)
    rnd = random.Random(seed)
    body = bytes(2 * n) if silent else struct.pack(f"<{n}h", *(rnd.randint(-20, 20) for _ in range(n)))
    return bytes(h) + body


def afp_bytes(mode, first, chunks, *, numbers=None, jitter=None, records=3):
    h = identity(mode, first)
    struct.pack_into("<4sHHHHHHII", h, 0, b"SAFP", 1, 512, 16000, 256, 16, 4, 80000, 64000)
    numbers = list(range(chunks)) if numbers is None else numbers
    hop = 64000 * TICKS // 16000
    body = bytearray()
    ticks = []
    for i, k in enumerate(numbers):
        t = first + k * hop + (jitter[i] if jitter else 0)
        ticks.append(t)
        body += struct.pack("<HHIQ", 0x4843, records, k, t)
        for r in range(records):
            body += struct.pack("<I", (r * 7) | (5 << 9) | (3 << 15) | (40 << 21))
    struct.pack_into("<IQI", h, 100, len(numbers), ticks[-1] if ticks else 0, len(numbers) * records)
    return bytes(h) + bytes(body)


class Session:
    """A card with data/X/S/000 and whatever audio files a case writes."""

    def __init__(self, root: Path):
        self.root = root
        self.data = root / "data" / "DefaultTrial_1792712946" / "Shimmer_15BC-400" / "000"
        self.data.parent.mkdir(parents=True)
        self.data.write_bytes(b"")
        self.audio = root / "audio" / "DefaultTrial_1792712946" / "Shimmer_15BC-400"

    def write(self, name, raw):
        self.audio.mkdir(parents=True, exist_ok=True)
        (self.audio / name).write_bytes(raw)


def good_geometry():
    """The first sample 0.4 s after the file opened, the last 0.1 s before the
    log's end; the chunk the stop interrupted is dropped."""
    first = INITIAL_TS + int(0.4 * TICKS)
    end = data_file(2).last_ticks - int(0.1 * TICKS)
    n = (end - first) * 16000 // TICKS + 1
    chunks = (n - 80000) // 64000            # one fewer than complete: the last was being analysed
    return first, n, chunks


def status(mode, seconds, chunks, skipped=0):
    return sa.AudioStatus(1, mode, 3, 0, seconds, 0, 0, 0, 280, 280, chunks, skipped)


class AudioFiles(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.s = Session(Path(self.tmp.name))

    def tearDown(self):
        self.tmp.cleanup()

    def run_check(self, mode, st, data=None):
        return sa.check(self.s.data, data or data_file(mode), mode, st)

    def test_mode_2_clean(self):
        first, n, chunks = good_geometry()
        self.s.write("000.WAV", wav_bytes(2, first, n))
        self.s.write("000.AFP", afp_bytes(2, first, chunks))
        r = self.run_check(2, status(2, n // 16000, chunks))
        self.assertEqual(r.problems, [])
        self.assertIn("AFP 5 chunks", r.note)
        self.assertIn("WAV 28.", r.note)

    def test_full_clock_against_the_header_s_40_bits(self):
        """On a unit whose RTC holds the date, the Shimmer clock passes 2^40
        ticks: the audio headers keep all 64 bits, the data file's header the low 40."""
        high = 53 << 40
        first, n, chunks = good_geometry()
        raw = bytearray(afp_bytes(1, first + high, chunks))
        struct.pack_into("<Q", raw, 56, INITIAL_TS + high)
        self.s.write("000.AFP", bytes(raw))
        self.assertEqual(self.run_check(1, status(1, 20, chunks)).problems, [])
        struct.pack_into("<Q", raw, 56, INITIAL_TS + high + 1)
        self.s.write("000.AFP", bytes(raw))
        p = " | ".join(self.run_check(1, None).problems)
        self.assertIn("data file timestamp", p)

    def test_mode_1_and_3_clean(self):
        first, n, chunks = good_geometry()
        self.s.write("000.AFP", afp_bytes(1, first, chunks))
        self.assertEqual(self.run_check(1, status(1, 20, chunks)).problems, [])
        (self.s.audio / "000.AFP").unlink()
        self.s.write("000.WAV", wav_bytes(3, first, n))
        self.assertEqual(self.run_check(3, status(3, n // 16000, 0)).problems, [])

    def test_mode_0(self):
        self.assertEqual(self.run_check(0, None).problems, [])
        self.s.audio.mkdir(parents=True)
        self.assertIn("audio off", self.run_check(0, None).problems[0])

    def test_wrong_files_for_the_mode(self):
        first, n, _ = good_geometry()
        self.s.write("000.WAV", wav_bytes(2, first, n))
        p = self.run_check(2, None).problems
        self.assertTrue(any("expected ['000.AFP', '000.WAV']" in x for x in p), p)

    def test_not_closed_and_silent(self):
        first, n, _ = good_geometry()
        self.s.write("000.WAV", wav_bytes(3, first, n, closed=0, silent=True))
        p = " | ".join(self.run_check(3, None).problems)
        self.assertIn("not closed", p)
        self.assertIn("no sound", p)

    def test_chunk_gap_and_status_mismatch(self):
        first, n, chunks = good_geometry()
        self.s.write("000.AFP", afp_bytes(1, first, chunks, numbers=[0, 1, 3, 4]))
        p = " | ".join(self.run_check(1, status(1, 20, chunks)).problems)
        self.assertIn("without a gap", p)
        self.assertIn("audio status: 5 chunks written", p)

    def test_chunk_clock_off_its_line(self):
        first, n, chunks = good_geometry()
        self.s.write("000.AFP", afp_bytes(1, first, chunks, jitter=[0, 1, 30, 0, 2]))
        p = " | ".join(self.run_check(1, None).problems)
        self.assertIn("ticks off their line", p)

    def test_audio_starting_late(self):
        first, n, chunks = good_geometry()
        late = first + 2 * TICKS
        self.s.write("000.AFP", afp_bytes(1, late, chunks - 1))
        p = " | ".join(self.run_check(1, None).problems)
        self.assertIn("first sample +2.4", p)

    def test_identity_against_the_data_file(self):
        first, n, chunks = good_geometry()
        self.s.write("000.AFP", afp_bytes(1, first, chunks))
        other = data_file(1)
        other.config_time += 1
        p = " | ".join(self.run_check(1, None, data=other).problems)
        self.assertIn("config time", p)

    def test_header_byte_226(self):
        first, n, chunks = good_geometry()
        self.s.write("000.AFP", afp_bytes(1, first, chunks))
        p = self.run_check(1, None, data=data_file(0)).problems
        self.assertTrue(any("byte 226" in x for x in p), p)

    def test_wav_cut_short_at_stop(self):
        """A slowed card: the stop's WAV drain ran out of time minutes behind the
        capture, so the header has no last-sample time (0) and fault 4."""
        first, n, _ = good_geometry()
        raw = bytearray(wav_bytes(3, first, n // 2))
        raw[97] = 4
        struct.pack_into("<Q", raw, 104, 0)
        self.s.write("000.WAV", bytes(raw))
        p = " | ".join(self.run_check(3, None).problems)
        self.assertIn("no last-sample time", p)
        self.assertNotIn("data file's last record", p)
        self.assertNotIn("ppm", p)

    def test_remove_raw_keeps_the_fingerprint(self):
        first, n, chunks = good_geometry()
        self.s.write("000.WAV", wav_bytes(2, first, 1000))
        self.s.write("000.AFP", afp_bytes(2, first, 1))
        self.assertEqual(sa.remove_raw(self.s.data), ["000.WAV"])
        self.assertEqual(sorted(p.name for p in self.s.audio.iterdir()), ["000.AFP"])
        self.assertEqual(sa.remove_raw(self.s.data), [])


class DataHeader(unittest.TestCase):
    def test_from_file(self):
        head = bytearray(384)
        head[24:30] = MAC
        head[34:36], head[36:38], head[38], head[39] = (3).to_bytes(2, "big"), (1).to_bytes(2, "big"), 1, 25
        head[52:56] = CONFIG_TIME.to_bytes(4, "big")
        head[226] = 2
        ts = (0x12 << 32) | 0x3456789A
        head[251] = 0x12
        head[252:256] = (0x3456789A).to_bytes(4, "little")
        d = sa.DataFile.from_file(bytes(head), first_ts24=(ts - 100) % (1 << 24), span_ticks=1000)
        self.assertEqual((d.initial_ts, d.config_time, d.mac, d.fw, d.audio_mode),
                         (ts, CONFIG_TIME, MAC.hex().upper(), FW, 2))
        self.assertEqual((d.first_ticks, d.last_ticks), (ts - 100, ts + 900))

    def test_status_reply(self):
        raw = bytes([1, 2, 3, 0]) + struct.pack("<IIHHIIII", 28, 0, 0, 0, 280, 280, 6, 0)
        st = sa.audio_status(raw)
        self.assertEqual((st.mode, st.state, st.seconds, st.chunks), (2, 3, 28, 6))
        with self.assertRaises(ValueError):
            sa.audio_status(raw[:31])


def level_series(rate=128.0, seconds=10.0, none_s=0.3, db=(42.0, 47.5)):
    """The channel as a unit sends it: no level for none_s, then one value per
    128 ms block, held in every packet until the next"""
    out = []
    n = int(rate * seconds)
    for i in range(n):
        t = i / rate
        if t < none_s:
            out.append(sl.NONE)
        else:
            block = int(t / 0.128)
            out.append(int(round((db[0] + (db[1] - db[0]) * (block % 7) / 6) * 100)))
    return out


class SoundLevel(unittest.TestCase):
    def test_off_and_absent_is_fine(self):
        self.assertEqual(sl.check(None, 128.0, False), ([], ""))

    def test_on_needs_the_channel(self):
        probs, _ = sl.check(None, 128.0, True)
        self.assertEqual(len(probs), 1)

    def test_off_must_not_send_it(self):
        probs, _ = sl.check(level_series(), 128.0, False)
        self.assertEqual(len(probs), 1)

    def test_a_good_series(self):
        for rate in (51.2, 128.0, 1024.0):
            probs, note = sl.check(level_series(rate=rate), rate, True)
            self.assertEqual(probs, [], rate)
            self.assertIn("42.0-47.5 dB(A)", note)

    def test_late_first_level(self):
        probs, _ = sl.check(level_series(none_s=0.7), 128.0, True)
        self.assertTrue(any("first sound level" in p for p in probs), probs)

    def test_no_level_after_the_first(self):
        v = level_series()
        v[500] = sl.NONE
        probs, _ = sl.check(v, 128.0, True)
        self.assertTrue(any("after the first level" in p for p in probs), probs)

    def test_out_of_range(self):
        probs, _ = sl.check(level_series(db=(12.0, 15.0)), 128.0, True)
        self.assertTrue(any("outside" in p for p in probs), probs)

    def test_never_a_level(self):
        probs, _ = sl.check([sl.NONE] * 500, 128.0, True)
        self.assertTrue(any("no level" in p for p in probs), probs)

    def test_changes_faster_than_a_block(self):
        v = [4000 + (i % 2) for i in range(1280)]
        probs, _ = sl.check(v, 128.0, True)
        self.assertTrue(any("more than once a 128 ms block" in p for p in probs), probs)


if __name__ == "__main__":
    unittest.main(verbosity=2)
