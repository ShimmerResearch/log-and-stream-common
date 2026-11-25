#!/usr/bin/env python3
"""Unit tests for the Shimmer3 long log (soak.long_log on Shimmer3): its SD files,
the configuration that makes an undock start a log, the Base6 card reader, the
operator's requests, and the steps through rig/shared_log.py. Run:
python Test/hil/test_hil_s3_long_log.py

No hardware: the Shimmer3 SD files are synthetic, and the Base6, the dock,
Bluetooth and the operator are stand-ins that act as the firmware and a person
would.
"""

import contextlib
import importlib.util
import sys
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rig import base6, bt, config, dock, power, s3_card, s3_long_log, sdfile, shared_log  # noqa: E402
from rig.config import RigError  # noqa: E402
from rig.s3_config import S3Config, divider_for  # noqa: E402

IMU = {"ln_accel", "gyro", "mag", "vbatt"}
IMU_RECORD = 3 + 6 + 2 + 6 + 6        # stamp, LN accel, VBatt, gyro, mag


def s3_header(divider=128, sensors=IMU, sync=False, mac="E8EB1B9767A0", hw=sdfile.HW_SHIMMER3) -> bytes:
    """A Shimmer3 SD header: 256 bytes of 0xFF with the fields the reader uses (SD_CARD_FORMAT §3.1)."""
    h = bytearray(b"\xff" * sdfile.HEAD_S3)
    h[0:2] = divider.to_bytes(2, "little")
    cfg = S3Config(bytes(384))
    cfg.set_sensors(set(sensors))
    h[3:6] = cfg.b[3:6]
    h[sdfile.SDH_TRIAL_CONFIG0] = sdfile.SDH_TIME_SYNC if sync else 0
    h[24:30] = bytes.fromhex(mac)
    h[30:32] = hw.to_bytes(2, "big")
    h[34:40] = bytes([0, 3, 0, 1, 1, 10])
    return bytes(h)


def s3_body(stamps, record_len=IMU_RECORD, sync=False) -> bytes:
    """Records of `record_len` bytes, packed into blocks as the firmware writes them (§2.1)."""
    head = sdfile.SYNC_HEAD if sync else 0
    per_block = (sdfile.BLOCK - head) // record_len
    out = bytearray()
    for i, s in enumerate(stamps):
        if i % per_block == 0:
            out += bytes(head)
        out += (s & 0xFFFFFF).to_bytes(3, "little") + bytes(record_len - 3)
    return bytes(out)


def s3_file(folder, name, stamps, **kw) -> Path:
    p = Path(folder) / name
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_bytes(s3_header(**{k: v for k, v in kw.items() if k != "record_len"})
                  + s3_body(stamps, kw.get("record_len", IMU_RECORD), kw.get("sync", False)))
    return p


def grid(n, start=1, period=128):
    return [start + period * i for i in range(n)]


# --- the Shimmer3 SD file ---------------------------------------------------------

class S3SdFile(unittest.TestCase):
    def test_header_rebuilds_the_layout_from_the_sensor_bitmaps(self):
        h = sdfile.read_header(s3_header())
        self.assertEqual((h.hw, h.head_len, h.divider, h.mac), (3, 256, 128, "E8EB1B9767A0"))
        self.assertEqual(h.sensors, frozenset(IMU))
        # ADC block first (LN accel, VBatt), then I2C (gyro, mag): STREAMING_DATA_FORMAT §4.1.
        self.assertEqual(h.channels, [0x00, 0x01, 0x02, 0x03, 0x0A, 0x0B, 0x0C, 0x07, 0x08, 0x09])
        self.assertEqual(h.record_len, IMU_RECORD)

    def test_a_shimmer3_file_needs_only_its_256_byte_header(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "000"
            p.write_bytes(s3_header())
            fc = sdfile.check(p)
        self.assertEqual((fc.records, fc.trailing_bytes), (0, 0))

    def test_a_channel_it_cannot_lay_out_is_refused_by_name(self):
        with self.assertRaisesRegex(sdfile.SdFileError, "gsr"):
            sdfile.read_header(s3_header(sensors=IMU | {"gsr"}))

    def test_another_hardware_version_is_refused(self):
        with self.assertRaisesRegex(sdfile.SdFileError, "hardware 7"):
            sdfile.read_header(s3_header(hw=7))

    def test_short_header(self):
        with self.assertRaisesRegex(sdfile.SdFileError, "256-byte"):
            sdfile.read_header(s3_header()[:200])

    def test_records_without_sync_are_contiguous(self):
        with tempfile.TemporaryDirectory() as d:
            fc = sdfile.check(s3_file(d, "000", grid(100)), keep=True)
        self.assertEqual((fc.records, fc.gaps, fc.dupes, fc.trailing_bytes), (100, 0, 0, 0))
        self.assertEqual(fc.ts, grid(100))

    def test_records_with_sync_skip_each_blocks_offset(self):
        n = 3 * ((sdfile.BLOCK - sdfile.SYNC_HEAD) // IMU_RECORD) + 5      # three full blocks and a short one
        with tempfile.TemporaryDirectory() as d:
            fc = sdfile.check(s3_file(d, "000", grid(n), sync=True), keep=True)
        self.assertEqual((fc.records, fc.gaps, fc.trailing_bytes), (n, 0, 0))
        self.assertEqual(fc.ts, grid(n))

    def test_a_gap_is_counted_in_samples(self):
        stamps = grid(50)
        del stamps[20:23]
        with tempfile.TemporaryDirectory() as d:
            fc = sdfile.check(s3_file(d, "000", stamps))
        self.assertEqual((fc.gaps, fc.missing), (1, 3))


class S3SdSession(unittest.TestCase):
    def test_files_join_at_one_period_across_the_rollover(self):
        with tempfile.TemporaryDirectory() as d:
            s3_file(d, "000", grid(200))
            s3_file(d, "001", grid(200, start=1 + 128 * 200))
            sc = sdfile.check_session(Path(d))
        self.assertEqual((sc.files, sc.records, sc.gaps, sc.joins), (["000", "001"], 400, 0, [("001", 128)]))
        self.assertEqual(sc.span_ticks, 399 * 128)

    def test_a_loss_at_the_join_is_a_gap_and_shows_in_joins(self):
        with tempfile.TemporaryDirectory() as d:
            s3_file(d, "000", grid(200))
            s3_file(d, "001", grid(200, start=1 + 128 * 202))
            sc = sdfile.check_session(Path(d))
        self.assertEqual((sc.gaps, sc.missing, sc.joins), (1, 2, [("001", 384)]))

    def test_an_unstamped_record_is_a_zero_stamp(self):
        stamps = grid(50)
        stamps[30] = 0
        with tempfile.TemporaryDirectory() as d:
            s3_file(d, "000", stamps)
            sc = sdfile.check_session(Path(d))
        self.assertEqual(sc.zero_ts, 1)

    def test_a_file_with_another_configuration_is_a_header_mismatch(self):
        with tempfile.TemporaryDirectory() as d:
            s3_file(d, "000", grid(10))
            s3_file(d, "001", grid(10, start=1 + 128 * 10), sensors=IMU - {"mag"}, record_len=IMU_RECORD - 6)
            sc = sdfile.check_session(Path(d))
        self.assertEqual(sc.header_mismatches, ["001"])

    def test_trial_sessions_lists_one_trial_folder(self):
        with tempfile.TemporaryDirectory() as d:
            for p in ("data/HIL_longlog_5/Shimmer_67A0-000", "data/HIL_longlog_5/Shimmer_67A0-001",
                      "data/Other_5/Shimmer_67A0-000"):
                (Path(d) / p).mkdir(parents=True)
            got = sdfile.trial_sessions(Path(d), "HIL_longlog_5")
            self.assertEqual(sdfile.trial_sessions(Path(d), "Missing_1"), set())
        self.assertEqual(got, {Path("data/HIL_longlog_5/Shimmer_67A0-000"),
                               Path("data/HIL_longlog_5/Shimmer_67A0-001")})


# --- the configuration -------------------------------------------------------------

def infomem(**fields) -> bytearray:
    """A 512-byte Shimmer3 InfoMem with the trial bytes a unit off the shelf has."""
    b = bytearray(512)
    b[0:2] = (640).to_bytes(2, "little")                    # 51.2 Hz
    b[199:211] = b"Trial1".ljust(12, b"\0")
    b[211:215] = (1234).to_bytes(4, "big")
    b[217] = 0x20 | 0x01                                     # button on, SD error LED
    b[218] = 0x80 | 0x10 | 0x01                              # single touch, tcxo, low-battery auto-stop
    b[222:224] = (60).to_bytes(2, "big")                     # auto-stop after an hour
    b[232] = 0x02                                            # button mode: single press
    for k, v in fields.items():
        b[int(k[1:])] = v
    return b


class S3TrialFields(unittest.TestCase):
    def test_experiment_id_is_read_as_the_firmware_reads_it(self):
        cfg = S3Config(bytes(infomem()))
        self.assertEqual(cfg.experiment_id, "Trial1")
        cfg.b[199:211] = b"ABCDEFGHIJKL"                    # all 12 printable, no terminator
        self.assertEqual(cfg.experiment_id, "ABCDEFGHIJKL")
        cfg.b[199:211] = b"AB\x01CD".ljust(12, b"\0")        # stops at the first unprintable
        self.assertEqual(cfg.experiment_id, "AB")
        cfg.b[199:211] = b"\xff" * 12                        # none: the firmware's default
        self.assertEqual(cfg.experiment_id, "DefaultTrial")

    def test_experiment_id_set(self):
        cfg = S3Config(bytes(infomem()))
        cfg.experiment_id = "HIL_longlog"
        self.assertEqual(bytes(cfg.b[199:211]), b"HIL_longlog\0")
        for bad in ("", "x" * 13, "tab\there"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                cfg.experiment_id = bad

    def test_config_time_is_big_endian_and_names_the_trial_folder(self):
        cfg = S3Config(bytes(infomem()))
        self.assertEqual(cfg.config_time, 1234)
        cfg.config_time = 1795195659
        self.assertEqual(bytes(cfg.b[211:215]), (1795195659).to_bytes(4, "big"))
        self.assertEqual(cfg.trial_dir, "Trial1_1795195659")
        cfg.config_time = 0
        self.assertEqual(cfg.trial_dir, "Trial1_0")

    def test_undock_start_and_no_auto_stop(self):
        cfg = S3Config(bytes(infomem()))
        cfg.set_undock_start()
        cfg.clear_auto_stop()
        self.assertEqual(cfg.b[217], 0x01)                  # button off, the rest kept
        self.assertEqual(cfg.b[218], 0x10)                  # single touch and auto-stop off, tcxo kept
        self.assertEqual((cfg.b[232], bytes(cfg.b[222:224])), (0, b"\0\0"))

    def test_radio_blocked(self):
        self.assertFalse(S3Config(bytes(infomem())).radio_blocked)
        self.assertTrue(S3Config(bytes(infomem(b217=0x24))).radio_blocked)
        self.assertTrue(S3Config(bytes(infomem(b217=0x28))).radio_blocked)

    def test_long_log_image(self):
        cfg = s3_long_log.long_log_image(bytes(infomem()), 256, IMU, 1795195659)
        self.assertEqual((cfg.divider, cfg.enabled()), (128, IMU))
        self.assertEqual((cfg.get("mag_rate"), cfg.get("gyro_rate")), (7, 30))      # the ODRs cover 256 Hz
        self.assertEqual(cfg.trial_dir, "HIL_longlog_1795195659")
        self.assertEqual((cfg.b[217] & 0x20, cfg.b[218] & 0x81, cfg.b[232]), (0, 0, 0))


class S3SessionProblems(unittest.TestCase):
    def check(self, n, **kw):
        with tempfile.TemporaryDirectory() as d:
            s3_file(d, "000", grid(n, period=kw.get("divider", 128)), **kw)
            sc = sdfile.check_session(Path(d))
        return s3_long_log.session_problems(sc, 256, IMU, 10)

    def test_a_whole_log_at_the_configuration_is_clean(self):
        self.assertEqual(self.check(2561), [])

    def test_a_log_shorter_than_the_wait(self):
        self.assertRegex(" ".join(self.check(1000)), "spans 0.1 min of the 0.2 min")

    def test_the_wrong_rate_or_sensors(self):
        self.assertRegex(" ".join(self.check(2561 // 5, divider=640)), "divider 640, configured 128")
        self.assertRegex(" ".join(self.check(2561, sensors=IMU - {"mag"}, record_len=IMU_RECORD - 6)),
                         "configured")

    def test_no_data_file(self):
        with tempfile.TemporaryDirectory() as d:
            sc = sdfile.check_session(Path(d))
        self.assertEqual(s3_long_log.session_problems(sc, 256, IMU, 10), ["the session has no data file"])


# --- the operator ---------------------------------------------------------------------

def unit(name, slot, mac=None):
    return types.SimpleNamespace(name=name, platform="shimmer3", base6_slot=slot, hub_port=None,
                                 mac=mac or f"0006660000{slot:02X}", radio="RN42 v4.77")


class OperatorDockTests(unittest.TestCase):
    """The spoken part of each request is short; the printed detail names the units."""

    def recorder(self):
        asked = []
        return asked, lambda text, detail="": asked.append((text, detail))

    def test_one_request_each_way_for_the_whole_group(self):
        asked, ask = self.recorder()
        group = [unit("a", 1), unit("b", 2), unit("c", 3)]
        od = power.OperatorDock(ask, lambda: group)
        for u in group:
            od.off(None, u.name)
        for u in group:
            od.on(None, u.name)
        self.assertEqual(asked, [
            ("Undock the units", "take a, b and c out of the Base6"),
            ("Dock the units, each in its own slot",
             "put them back in the Base6: a in slot 1, b in slot 2 and c in slot 3")])

    def test_the_group_is_asked_for_at_the_first_off(self):
        group = []
        asked, ask = self.recorder()
        od = power.OperatorDock(ask, lambda: group)
        group += [unit("a", 1), unit("b", 2)]               # known only once configuring is over
        od.off(None, "a")
        self.assertEqual(asked, [("Undock the units", "take a and b out of the Base6")])

    def test_a_unit_outside_the_group_is_asked_for_alone(self):
        asked, ask = self.recorder()
        od = power.OperatorDock(ask, lambda: [unit("a", 1)])
        od.off(None, "z")
        od.on(None, "z")
        self.assertEqual(asked, [("Undock the unit", "take z out of the Base6"),
                                 ("Dock the unit", "put it back in the Base6: z")])

    def test_on_for_a_unit_never_taken_out_asks_nothing(self):
        asked, ask = self.recorder()
        power.OperatorDock(ask, lambda: [unit("a", 1)]).on(None, "a")
        self.assertEqual(asked, [])

    def test_the_other_units_wait_until_the_operator_has_answered(self):
        """On the bench (24 Nov 2026) the units after the first returned from
        off() at once, so each gave the operator only its own 30 s from the
        request, not from the answer, and failed while they were still in."""
        answered = threading.Event()
        returned = {}

        def ask(text, detail=""):
            time.sleep(0.3)                     # the operator, lifting the units out
            answered.set()
        od = power.OperatorDock(ask, lambda: [unit("a", 1), unit("b", 2), unit("c", 3)])

        def off(name):
            od.off(None, name)
            returned[name] = answered.is_set()
        threads = [threading.Thread(target=off, args=(n,)) for n in "abc"]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(returned, {"a": True, "b": True, "c": True})

    def test_a_request_cut_short_is_still_followed_by_the_one_to_put_them_back(self):
        asked = []

        def ask(text, detail=""):
            asked.append((text, detail))
            if text.startswith("Undock"):
                raise KeyboardInterrupt
        od = power.OperatorDock(ask, lambda: [unit("a", 1), unit("b", 2)])
        with self.assertRaises(KeyboardInterrupt):
            od.off(None, "a")
        od.on(None, "a")
        self.assertEqual(asked[-1], ("Dock the units, each in its own slot",
                                     "put them back in the Base6: a in slot 1 and b in slot 2"))

    def test_a_failed_put_back_is_asked_again(self):
        answers = [EOFError(), None]

        def ask(text, detail=""):
            if text.startswith("Dock"):
                a = answers.pop(0)
                if a:
                    raise a
        od = power.OperatorDock(ask, lambda: [unit("a", 1), unit("b", 2)])
        od.off(None, "a")
        with self.assertRaises(EOFError):
            od.on(None, "a")
        od.on(None, "b")
        self.assertEqual((answers, od.out), ([], {}))

    def test_manual_power_asks_through_the_same_prompt(self):
        asked, ask = self.recorder()
        mp = power.ManualPower(ask)
        mp.off(1, "s3r-x")
        mp.on(1, "s3r-x")
        self.assertEqual(asked, [("Undock s3r-x", "disconnect its USB-C"), ("Dock s3r-x", "connect its USB-C")])

    def test_listing(self):
        self.assertEqual([power.listing(x) for x in (["a"], ["a", "b"], ["a", "b", "c"])],
                         ["a", "a and b", "a, b and c"])


class OperatorAsk(unittest.TestCase):
    """conftest's hil_ask reads the answer from stdin, so stdin must be out of
    pytest's capture too: global_and_fixture_disabled() alone leaves it in,
    and input() failed at once on the bench (24 Nov 2026)."""

    @unittest.skipUnless(importlib.util.find_spec("pytest"), "conftest.py needs pytest")
    def test_stdin_is_released_for_the_answer_and_taken_back_after(self):
        import conftest
        calls = []

        class Capman:
            @contextlib.contextmanager
            def global_and_fixture_disabled(self):
                calls.append("out released")
                yield
                calls.append("all taken back")

            def suspend_global_capture(self, in_=False):
                calls.append(f"suspend in_={in_}")

        said = []
        ask = conftest._asker(said.append, Capman(), read=lambda: calls.append("read"))
        with mock.patch.object(conftest, "speak", lambda text: calls.append(f"spoke {text}")):
            ask("Undock the units", "take a and b out of the Base6")
        self.assertEqual(calls, ["spoke Undock the units", "out released", "suspend in_=True", "read",
                                 "all taken back"])
        self.assertEqual(said, [">>> Undock the units: take a and b out of the Base6, then press Enter"])

    @unittest.skipUnless(importlib.util.find_spec("pytest"), "conftest.py needs pytest")
    def test_a_progress_line_is_written_with_the_capture_suspended(self):
        """During a test pytest's fd capture has the terminal's descriptor too,
        so a line the reporter writes reached the screen only after the run,
        the operator's requests among them (bench, 24 Nov 2026)."""
        import conftest
        calls = []

        class Capman:
            @contextlib.contextmanager
            def global_and_fixture_disabled(self):
                calls.append("released")
                yield
                calls.append("taken back")

        class Tw:
            width_of_current_line = 0

            def flush(self):
                calls.append("flushed")

        class Reporter:
            _tw = Tw()

            def write_line(self, text):
                calls.append(f"wrote {text}")

        plugins = {"terminalreporter": Reporter(), "capturemanager": Capman()}
        config = types.SimpleNamespace(pluginmanager=types.SimpleNamespace(get_plugin=plugins.get))
        conftest._say(config)("Undock the units")
        self.assertEqual(calls, ["released", "wrote hil: Undock the units", "flushed", "taken back"])


class AttendedOnly(unittest.TestCase):
    @unittest.skipUnless(importlib.util.find_spec("pytest"), "conftest.py needs pytest")
    def test_the_shimmer3_long_log_runs_only_with_someone_at_the_bench(self):
        import conftest
        rig = config.Rig(path=Path("rig.toml"), bench="b", base6_serial="SRYRNS21",
                         units=[config.Unit("s3-a", "shimmer3", mac="000666000001", base6_slot=1)])
        needs = {"shimmer3", "dock", "bt", "attended"}
        for opts, missing in (({}, {"attended"}), ({"--attended": True}, set()),
                              ({"--manual-power": True}, set())):
            cfg = types.SimpleNamespace(_hil_rig=rig, getoption=lambda name, o=opts: o.get(name, False))
            with self.subTest(opts=opts):
                self.assertEqual(conftest._missing(cfg, rig.units[0], needs), missing)


# --- the Base6 card reader ----------------------------------------------------------------

class FakeMux:
    """Channel B: answers every command with `reply`, or each command with its
    own from a dict."""
    timeout = base6.REPLY_S

    def __init__(self, reply):
        self.reply, self.sent = reply, []

    def reset_input_buffer(self):
        pass

    def write(self, b):
        self.sent.append(b.decode())

    def readline(self):
        return self.reply[self.sent[-1]] if isinstance(self.reply, dict) else self.reply


class Base6Card(unittest.TestCase):
    def b6(self, reply):
        b = base6.Base6.__new__(base6.Base6)
        b.mux, b.selected = FakeMux(reply), 3
        return b

    def test_sdc_gives_the_card_to_the_reader_and_select_takes_it_back(self):
        b = self.b6({"SDC,03$": b"C,03\r\n", "SDP,03$": b"P,03\r\n"})
        with mock.patch.object(base6.time, "sleep") as slept:
            b.card_to_pc(3)
            self.assertEqual((b.mux.sent, b.selected), (["SDC,03$"], None))
            self.assertEqual(slept.call_args[0][0], base6.CARD_CHANGEOVER_S)
            b.select(3)                                    # not skipped: SDP is what hands the card back
        self.assertEqual(b.mux.sent, ["SDC,03$", "SDP,03$"])

    def test_a_refusal_raises_and_still_forgets_the_selection(self):
        b = self.b6(b"E\r\n")
        with mock.patch.object(base6.time, "sleep"), self.assertRaisesRegex(RigError, "SDC,02"):
            b.card_to_pc(2)
        self.assertEqual(b.mux.sent, ["SDC,02$"] * base6.TRIES)
        self.assertIsNone(b.selected)

    def test_reader_roots_come_from_the_readers_disks_only(self):
        disks = [{"name": "Shimmer HS-SD/MMC", "letters": ["E"]},
                 {"name": "Shimmer HS-SD/MMC", "letters": []},
                 {"name": "STM32 Shimmer3R", "letters": ["F"]}]
        self.assertEqual(s3_card.reader_roots(disks), {Path("E:/")})


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


class CardTests(unittest.TestCase):
    def test_root_is_the_reader_drive_that_appeared(self):
        with tempfile.TemporaryDirectory() as d:
            old, new = Path(d) / "old", Path(d) / "new"
            old.mkdir()
            new.mkdir()
            card = s3_card.Card({old}, roots=lambda: {old, new})
            self.assertEqual(card.root(), new)

    def test_waits_for_the_mount(self):
        clock = FakeClock()
        with tempfile.TemporaryDirectory() as d:
            drive = Path(d) / "e"
            calls = []

            def roots():
                calls.append(1)
                if len(calls) == 3:
                    drive.mkdir()
                return {drive}
            card = s3_card.Card(set(), roots, mount_s=10, sleep=clock.sleep, clock=clock)
            self.assertEqual(card.root(), drive)

    def test_a_card_never_mounted_is_not_retried(self):
        clock = FakeClock()
        card = s3_card.Card(set(), lambda: set(), mount_s=5, sleep=clock.sleep, clock=clock)
        with self.assertRaises(s3_card.CardNotMounted):
            s3_card.with_retries(card, lambda root: root, sleep=clock.sleep)
        self.assertLessEqual(clock.t, 6)

    def test_with_retries_finds_the_drive_again_after_it_went_away(self):
        clock = FakeClock()
        with tempfile.TemporaryDirectory() as d:
            e, f = Path(d) / "e", Path(d) / "f"
            e.mkdir()
            current = [e]
            card = s3_card.Card(set(), lambda: set(current), mount_s=5, sleep=clock.sleep, clock=clock)
            seen = []

            def fn(root):
                seen.append(root)
                if len(seen) == 1:
                    e.rmdir()
                    f.mkdir()
                    current[0] = f                         # it came back under another letter
                    raise OSError("the device is not ready")
                return root
            self.assertEqual(s3_card.with_retries(card, fn, sleep=clock.sleep), f)
        self.assertEqual(seen, [e, f])

    def test_with_retries_gives_up(self):
        clock = FakeClock()
        with tempfile.TemporaryDirectory() as d:
            card = s3_card.Card(set(), lambda: {Path(d)}, sleep=clock.sleep, clock=clock)

            def fn(root):
                raise OSError("gone")
            with self.assertRaisesRegex(s3_card.CardError, "failed 5 times"):
                s3_card.with_retries(card, fn, sleep=clock.sleep)

    def test_copy_session(self):
        with tempfile.TemporaryDirectory() as card, tempfile.TemporaryDirectory() as out:
            rel = Path("data/T_1/S-000")
            s3_file(Path(card) / rel, "000", grid(10))
            s3_file(Path(card) / rel, "001", grid(10))
            (Path(out) / rel).mkdir(parents=True)
            (Path(out) / rel / "000").write_bytes(b"half")  # an earlier attempt's
            dst = s3_card.copy_session(Path(card), rel, Path(out))
            self.assertEqual(sorted(p.name for p in dst.iterdir()), ["000", "001"])
            self.assertEqual((dst / "000").read_bytes(), (Path(card) / rel / "000").read_bytes())

    def test_a_short_copy_is_an_error_worth_retrying(self):
        with tempfile.TemporaryDirectory() as card, tempfile.TemporaryDirectory() as out:
            rel = Path("data/T_1/S-000")
            s3_file(Path(card) / rel, "000", grid(10))

            def short(src, dst):
                Path(dst).mkdir(parents=True)
                (Path(dst) / "000").write_bytes(b"x")
            with mock.patch.object(s3_card.shutil, "copytree", short), \
                    self.assertRaisesRegex(s3_card.CardError, "copied 1 of"):
                s3_card.copy_session(Path(card), rel, Path(out))

    def test_on_pc_gives_the_card_back_whatever_happens(self):
        b6 = mock.Mock()
        with self.assertRaises(ValueError), s3_card.on_pc(b6, 4, roots=lambda: set()):
            raise ValueError
        self.assertEqual([c[0] for c in b6.method_calls], ["card_to_pc", "select"])

    def test_on_pc_lets_the_last_cards_drive_go_before_the_sdc(self):
        clock = FakeClock()
        with tempfile.TemporaryDirectory() as d:
            e = Path(d) / "e"
            e.mkdir()                                       # the card given back a moment ago, still mounted
            b6 = mock.Mock()
            polls = []

            def roots():
                polls.append(b6.card_to_pc.called)
                if len(polls) == 3 and not b6.card_to_pc.called:
                    e.rmdir()                              # it goes
                if b6.card_to_pc.called and not e.exists():
                    e.mkdir()                              # and the new card mounts as the same letter
                return {e}
            with s3_card.on_pc(b6, 2, roots=roots, sleep=clock.sleep, clock=clock) as card:
                self.assertEqual(card.root(), e)
        self.assertEqual(polls[:3], [False, False, False])


# --- the steps, through shared_log, against a simulated bench ----------------------------

class Shimmer:
    """One simulated Shimmer3: its InfoMem, its card, and what its firmware does."""

    def __init__(self, bench, u):
        self.bench, self.u = bench, u
        self.infomem = infomem()
        self.infomem[187:199] = f"Shimmer_{u.mac[-4:]}".encode().ljust(12, b"\0")
        self.card = bench.tmp / f"card_{u.name}"
        (self.card / "data" / "Trial1_1234" / f"Shimmer_{u.mac[-4:]}-000").mkdir(parents=True)
        self.logging = False
        self.ignores_undock = False
        self.corrects = None            # a function the firmware applies to every written image
        self.silent_opens = 0           # dock opens it leaves unanswered, as while it boots

    def undocked(self):
        cfg = S3Config(bytes(self.infomem))
        if not (cfg.b[217] & 0x20 or cfg.b[218] & 0x80 or cfg.b[232] or self.ignores_undock):
            self.logging = True

    def stop(self):
        """Close the log: one session in its trial folder, spanning the wait."""
        self.logging = False
        cfg = S3Config(bytes(self.infomem))
        folder = self.card / "data" / cfg.trial_dir
        folder.mkdir(parents=True, exist_ok=True)
        n = len([p for p in folder.iterdir()])
        records = int(self.bench.seconds * 32768 / cfg.divider) + 1
        s3_file(folder / f"Shimmer_{self.u.mac[-4:]}-{n:03d}", "000",
                grid(records, period=cfg.divider), divider=cfg.divider, sensors=cfg.enabled(),
                mac=self.u.mac, record_len=3 + 20 if cfg.enabled() == IMU else 3)


    def moved(self):
        """Taken out of one slot and put in another moments later, the log's
        undock-start configuration still in it: a session with no records."""
        self.undocked()
        if not self.logging:
            return
        self.logging = False
        cfg = S3Config(bytes(self.infomem))
        folder = self.card / "data" / cfg.trial_dir
        n = len([p for p in folder.iterdir()])
        s3_file(folder / f"Shimmer_{self.u.mac[-4:]}-{n:03d}", "000", [], divider=cfg.divider,
                sensors=cfg.enabled(), mac=self.u.mac, record_len=3 + 20 if cfg.enabled() == IMU else 3)


class FakeBase6:
    uart_port = "COM14"

    def __init__(self, bench):
        self.bench, self.slots, self.selected, self.card_slot = bench, {}, None, None

    def occupied(self):
        return sorted(self.slots)

    def select(self, slot):
        self.selected, self.card_slot = slot, None

    def card_to_pc(self, slot):
        self.selected, self.card_slot = None, slot


class FakeDockLink:
    def __init__(self, sh):
        self.sh = sh

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass

    def mac(self):
        return self.sh.u.mac

    def read_infomem(self, offset=0, length=512):
        return bytes(self.sh.infomem[offset:offset + length])

    def write_infomem(self, data, offset=0):
        self.sh.infomem[offset:offset + len(data)] = data
        self.sh.infomem[230] |= 0x01                         # flagWriteCfgToSd
        if self.sh.corrects:
            self.sh.corrects(self.sh.infomem)


class FakeBtLink:
    def __init__(self, sh):
        self.sh = sh

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass

    def wait_status(self, mask, want, timeout_s=10):
        if self.sh.logging != want:
            raise bt.BtError(f"status: bit mask {mask:#04x} not {'set' if want else 'clear'}")

    def simple(self, op):
        if op == bt.STOP_LOGGING and self.sh.logging:
            self.sh.stop()


class Bench:
    """Shimmer3s in a Base6, and an operator who does what is asked, or what a test says."""

    def __init__(self, tmp, units, seconds=2.0):
        self.tmp, self.seconds = Path(tmp), seconds
        self.base6 = FakeBase6(self)
        self.units = units
        self.shimmers = {u.name: Shimmer(self, u) for u in units}
        for u in units:
            self.base6.slots[u.base6_slot] = self.shimmers[u.name]
        self.asks = []
        self.left_in = set()           # units the operator forgets to take out
        self.put_back = {}             # unit name -> the slot the operator puts it back in (None: not at all)
        self.moves_done = True         # whether the operator does a "Move" request
        self.moves_log = False         # whether a move starts a log, as an undock with undock-start set does
        self.boot_silence = {}         # unit name -> dock opens it leaves unanswered once put back
        self.ports = {f"COM{40 + i}": self.shimmers[u.name] for i, u in enumerate(units)}

    def ask(self, text, detail=""):
        self.asks.append(f"{text}: {detail}" if detail else text)
        named = [u for u in self.units if u.name in f"{text} {detail}"]
        if text.startswith("Undock"):
            for u in named:
                if u.name not in self.left_in:
                    slot = next(s for s, sh in self.base6.slots.items() if sh.u is u)
                    del self.base6.slots[slot]
                    self.shimmers[u.name].undocked()
        elif text.startswith("Dock"):
            for u in named:
                slot = self.put_back.get(u.name, u.base6_slot)
                if slot is not None:
                    self._dock(u, slot)
                    self.shimmers[u.name].silent_opens = self.boot_silence.get(u.name, 0)
        elif text.startswith("Move") and self.moves_done:
            for u in named:
                if self.moves_log:
                    self.shimmers[u.name].moved()
                self._dock(u, u.base6_slot)

    def _dock(self, u, slot):
        for s, sh in list(self.base6.slots.items()):
            if sh.u is u:
                del self.base6.slots[s]
        self.base6.slots[slot] = self.shimmers[u.name]

    def where(self, name):
        return next((s for s, sh in self.base6.slots.items() if sh.u.name == name), None)

    @contextlib.contextmanager
    def patched(self):
        def open_dock(port, attempts=10, delay_s=0.5):
            sh = self.base6.slots.get(self.base6.selected)
            if sh is None or sh.silent_opens:
                if sh:
                    sh.silent_opens -= 1
                raise dock.DockError(f"{port}: no answer")
            return FakeDockLink(sh)

        def spp_port(mac):
            return next((p for p, sh in self.ports.items() if sh.u.mac == mac), None)

        @contextlib.contextmanager
        def on_pc(b6, slot, **kw):
            b6.card_to_pc(slot)
            try:
                yield s3_card.Card(set(), lambda: {b6.slots[slot].card}, mount_s=0)
            finally:
                b6.select(slot)

        with mock.patch.object(dock, "open_when_ready", open_dock), \
                mock.patch.object(bt, "spp_port", spp_port), \
                mock.patch.object(bt, "BtLink", lambda port, **kw: FakeBtLink(self.ports[port])), \
                mock.patch.object(s3_card, "on_pc", on_pc):
            yield

    def run(self):
        clock = FakeClock()
        steps = s3_long_log.S3LogSteps(self.base6, self.ask, self.tmp / "out", 256, IMU,
                                       now=lambda: 1795195659, clock=clock, sleep=clock.sleep)
        with self.patched():
            logs = shared_log.log_together(self.units, power.OperatorDock(self.ask, lambda: steps.ready),
                                           self.seconds, steps, sleep=lambda s: None)
        return logs, steps


class StepsThroughSharedLog(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.units = [unit("s3-a", 1, "000666AAAAA1"), unit("s3-b", 2, "000666BBBBB2"),
                      unit("s3-c", 3, "000666CCCCC3")]
        self.bench = Bench(self.tmp, self.units)
        self.own = {n: bytes(sh.infomem) for n, sh in self.bench.shimmers.items()}

    def tearDown(self):
        self._tmp.cleanup()

    def assertRestored(self, *names):
        for n in names or self.own:
            got = bytearray(self.bench.shimmers[n].infomem)
            got[230] &= ~0x01
            self.assertEqual(bytes(got), self.own[n], f"{n}'s InfoMem")

    def test_every_unit_logs_and_is_checked_from_a_copy_of_its_card(self):
        logs, steps = self.bench.run()
        self.assertEqual({n: lg.errors for n, lg in logs.items()}, {u.name: [] for u in self.units})
        self.assertEqual(self.bench.asks, [
            "Undock the units: take s3-a, s3-b and s3-c out of the Base6",
            "Dock the units, each in its own slot: put them back in the Base6: s3-a in slot 1, s3-b in slot 2 "
            "and s3-c in slot 3"])
        for u in self.units:
            lg = logs[u.name]
            self.assertEqual(lg.session, self.tmp / "out" / u.name / "data" / "HIL_longlog_1795195659"
                             / f"Shimmer_{u.mac[-4:]}-000")
            self.assertEqual((lg.check.records, lg.check.gaps, lg.check.header.sensors),
                             (513, 0, frozenset(IMU)))
            self.assertEqual(s3_long_log.session_problems(lg.check, 256, IMU, self.bench.seconds), [])
            self.assertTrue((self.tmp / "out" / f"infomem_{u.name}.bin").exists())
        self.assertRestored()
        self.assertIsNone(self.bench.base6.card_slot, "a card was left with the reader")

    def test_units_put_back_in_each_others_slots_are_found_and_moved(self):
        self.bench.put_back = {"s3-a": 2, "s3-b": 1}
        logs, steps = self.bench.run()
        self.assertEqual({n: lg.errors for n, lg in logs.items()}, {u.name: [] for u in self.units})
        self.assertEqual(self.bench.asks[2], "Move s3-a to slot 1 (now in slot 2) and s3-b to slot 2 (now in slot 1)")
        self.assertEqual((self.bench.where("s3-a"), self.bench.where("s3-b")), (1, 2))
        self.assertRestored()

    def test_the_session_a_requested_move_starts_is_noted_not_failed(self):
        self.bench.put_back = {"s3-a": 2, "s3-b": 1}
        self.bench.moves_log = True
        logs, steps = self.bench.run()
        self.assertEqual({n: lg.errors for n, lg in logs.items()}, {u.name: [] for u in self.units})
        for u in self.units[:2]:
            lg = logs[u.name]
            self.assertEqual(lg.session.name, f"Shimmer_{u.mac[-4:]}-000")
            self.assertEqual(lg.check.records, 513)
            self.assertRegex(" ".join(lg.notes),
                             rf"1 more session\(s\) .*\(Shimmer_{u.mac[-4:]}-001\).*moved between slots")
        self.assertEqual(logs["s3-c"].notes, [])
        self.assertRestored()

    def test_a_unit_not_asked_to_move_has_no_reason_for_a_second_session(self):
        logs, steps = self.bench.run()
        self.assertEqual([steps.extra_sessions_reason(u) for u in self.units], [None, None, None])

    def test_a_unit_still_booting_is_waited_for_not_moved(self):
        self.bench.boot_silence = {"s3-b": 2}
        logs, steps = self.bench.run()
        self.assertEqual({n: lg.errors for n, lg in logs.items()}, {u.name: [] for u in self.units})
        self.assertEqual(len(self.bench.asks), 2, self.bench.asks)

    def test_a_unit_not_put_back_is_asked_for(self):
        self.bench.put_back = {"s3-c": None}
        logs, steps = self.bench.run()
        self.assertEqual(self.bench.asks[2], "Move s3-c to slot 3 (not found)")
        self.assertEqual({n: lg.errors for n, lg in logs.items()}, {u.name: [] for u in self.units})
        self.assertRestored()

    def test_a_unit_left_in_the_wrong_slot_fails_but_keeps_its_log_and_configuration(self):
        self.bench.put_back = {"s3-a": 5}
        self.bench.moves_done = False
        logs, steps = self.bench.run()
        self.assertEqual(sum(t.startswith("Move") for t in self.bench.asks), s3_long_log.SLOT_REQUESTS)
        self.assertRegex(" ".join(logs["s3-a"].errors), r"restore: .*slot 5, not its own slot 1")
        self.assertEqual(logs["s3-a"].check.records, 513)
        self.assertEqual((logs["s3-b"].errors, logs["s3-c"].errors), ([], []))
        self.assertRestored()

    def test_a_unit_whose_radio_is_off_stays_docked_and_out_of_the_request(self):
        self.bench.shimmers["s3-b"].infomem[217] |= 0x04                  # SD sync
        self.own["s3-b"] = bytes(self.bench.shimmers["s3-b"].infomem)
        logs, steps = self.bench.run()
        self.assertRegex(" ".join(logs["s3-b"].errors), "configure: .*SD sync")
        self.assertEqual(self.bench.asks[0], "Undock the units: take s3-a and s3-c out of the Base6")
        self.assertEqual(self.bench.where("s3-b"), 2)
        self.assertEqual((logs["s3-a"].errors, logs["s3-c"].errors), ([], []))
        self.assertRestored()

    def test_a_correction_by_the_firmware_fails_configure_and_puts_the_unit_back(self):
        def keep_button(im):                                              # as if byte 232 held the button on
            im[217] |= 0x20
        self.bench.shimmers["s3-c"].corrects = keep_button
        logs, steps = self.bench.run()
        self.assertRegex(" ".join(logs["s3-c"].errors), "configure: .*offsets \\[217\\]")
        self.assertNotIn("s3-c", self.bench.asks[0])
        self.bench.shimmers["s3-c"].corrects = None
        self.assertRestored("s3-c")

    def test_a_unit_the_undock_did_not_start_fails_and_is_still_put_back(self):
        self.bench.shimmers["s3-a"].ignores_undock = True
        logs, steps = self.bench.run()
        self.assertRegex(" ".join(logs["s3-a"].errors), "start: .*not set")
        self.assertIsNone(logs["s3-a"].check)
        self.assertEqual(self.bench.where("s3-a"), 1)
        self.assertEqual((logs["s3-b"].errors, logs["s3-c"].errors), ([], []))
        self.assertRestored()

    def test_a_unit_left_in_the_base6_fails_its_undock(self):
        self.bench.left_in = {"s3-b"}
        logs, steps = self.bench.run()
        self.assertRegex(" ".join(logs["s3-b"].errors), "undock: .*still in slot 2")
        self.assertEqual((logs["s3-a"].errors, logs["s3-c"].errors), ([], []))
        self.assertRestored()

    def test_no_session_in_the_trial_folder_names_what_the_card_has(self):
        def no_file(sh=self.bench.shimmers["s3-a"]):
            sh.logging = False
        self.bench.shimmers["s3-a"].stop = no_file
        logs, steps = self.bench.run()
        self.assertRegex(" ".join(logs["s3-a"].errors), r"no session in data/HIL_longlog_1795195659.*Trial1_1234")
        self.assertRestored()

    def test_an_unpaired_unit_is_not_configured(self):
        del self.bench.ports["COM41"]                                       # s3-b
        logs, steps = self.bench.run()
        self.assertRegex(" ".join(logs["s3-b"].errors), "configure: .*no paired SPP port")
        self.assertEqual(self.bench.shimmers["s3-b"].infomem, bytearray(self.own["s3-b"]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
