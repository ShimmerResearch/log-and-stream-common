#!/usr/bin/env python3
"""Unit tests for rig.shared_log: the soak tier's long log on every unit at once.

What is checked is the order and the guarantees: one unit's failure leaves the
others logging, every unit taken off the dock goes back on it, every unit's
configuration goes back, and the log is waited for once. Run:
python Test/hil/test_hil_shared_log.py

No hardware and nothing beyond the standard library: the platform's steps and
the hub are stand-ins.
"""

import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rig import shared_log  # noqa: E402

ROOT = Path("E:/")
OLD = Path("data/exp/old-000")


def unit(name, port):
    return types.SimpleNamespace(name=name, hub_port=port)


class FakePower:
    def __init__(self, events, fail=()):
        self.events, self.fail = events, set(fail)
        self.off_ports = set()

    def off(self, port, label):
        self.events.append(("off", label))
        self.off_ports.add(port)
        if ("off", label) in self.fail:
            raise OSError("hub refused")

    def on(self, port, label):
        self.events.append(("on", label))
        self.off_ports.discard(port)
        if ("on", label) in self.fail:
            raise OSError("hub refused")


class FakeSteps:
    """Every call goes into `events`. `fail` maps (step, unit name) to the
    exception that step raises - "sessions-after" for the look at the card after
    the re-dock; `new` maps a unit name to the sessions its log writes (one by
    default)."""

    def __init__(self, events, fail=None, new=None):
        self.events, self.fail, self.new = events, fail or {}, new or {}
        self.logged = set()
        self.restored = {}

    def _call(self, step, name, fail_as=None):
        self.events.append((step, name))
        e = self.fail.get((fail_as or step, name))
        if e is not None:
            raise e

    def configure(self, u):
        self._call("configure", u.name)
        return f"infomem-{u.name}"

    def sessions(self, u, timeout_s):
        after = timeout_s == shared_log.REDOCK_DRIVE_S
        self._call("sessions", u.name, "sessions-after" if after else None)
        dirs = {OLD}
        if u.name in self.logged:
            dirs |= set(self.new.get(u.name, [Path(f"data/exp/{u.name}-001")]))
        return ROOT, dirs

    def undocked(self, u):
        self._call("undocked", u.name)

    def start(self, u):
        self._call("start", u.name)
        self.logged.add(u.name)

    def stop(self, u):
        self._call("stop", u.name)

    def restore(self, u, saved):
        self._call("restore", u.name)
        self.restored[u.name] = saved

    def check(self, session):
        self.events.append(("check", session.name))
        return f"checked {session.name}"


class ExplainingSteps(FakeSteps):
    """FakeSteps with shared_log.Steps' optional extra_sessions_reason: `reasons`
    maps a unit name to why it may have more than one new session."""

    def __init__(self, events, fail=None, new=None, reasons=None):
        super().__init__(events, fail, new)
        self.reasons = reasons or {}

    def extra_sessions_reason(self, u):
        self.events.append(("reason", u.name))
        return self.reasons.get(u.name)


UNITS = [unit("A", 1), unit("B", 2), unit("C", 3)]


def run(fail=None, power_fail=(), new=None, sleep=None, units=UNITS, reasons=None):
    events = []
    steps = FakeSteps(events, fail, new) if reasons is None else ExplainingSteps(events, fail, new, reasons)
    power = FakePower(events, power_fail)
    sleeps = []

    def default_sleep(s):
        events.append(("sleep", s))
        sleeps.append(s)
    logs = shared_log.log_together(units, power, 600, steps, sleep=sleep or default_sleep)
    return logs, events, steps, power, sleeps


def names(events, step):
    return [n for s, n in events if s == step]


class AllWell(unittest.TestCase):
    def setUp(self):
        self.logs, self.events, self.steps, self.power, self.sleeps = run()

    def test_every_unit_logs_and_each_session_is_checked(self):
        for name, lg in self.logs.items():
            self.assertTrue(lg.ok, lg.errors)
            self.assertEqual(lg.session, ROOT / f"data/exp/{name}-001")
            self.assertEqual(lg.check, f"checked {name}-001")

    def test_the_log_is_waited_for_once(self):
        self.assertEqual(self.sleeps, [600])

    def test_every_start_comes_before_the_wait_and_every_stop_after(self):
        wait = self.events.index(("sleep", 600))
        self.assertTrue(all(self.events.index(("start", n)) < wait < self.events.index(("stop", n))
                            for n in "ABC"))

    def test_each_unit_is_undocked_before_its_log_starts(self):
        for n in "ABC":
            self.assertLess(self.events.index(("undocked", n)), self.events.index(("start", n)))

    def test_re_docks_go_one_at_a_time_each_waiting_for_its_drive(self):
        after = self.events[self.events.index(("stop", "C")) + 1:]
        redock = [e for e in after if e[0] in ("on", "sessions")]
        self.assertEqual(redock, [("on", "A"), ("sessions", "A"), ("on", "B"), ("sessions", "B"),
                                  ("on", "C"), ("sessions", "C")])

    def test_every_configuration_goes_back_before_the_sessions_are_checked(self):
        self.assertEqual(self.steps.restored, {n: f"infomem-{n}" for n in "ABC"})
        last_restore = max(self.events.index(("restore", n)) for n in "ABC")
        first_check = min(i for i, e in enumerate(self.events) if e[0] == "check")
        self.assertLess(last_restore, first_check)
        self.assertFalse(self.power.off_ports)


class OneUnitFails(unittest.TestCase):
    def test_a_start_that_fails_leaves_the_others_logging(self):
        logs, events, steps, power, sleeps = run(fail={("start", "B"): TimeoutError("no status")})
        self.assertEqual(logs["B"].errors, ["start: TimeoutError: no status"])
        self.assertIsNone(logs["B"].check)
        self.assertNotIn("B", names(events, "stop"))
        self.assertTrue(logs["A"].ok and logs["C"].ok)
        self.assertEqual(sleeps, [600])
        # B was undocked, so it goes back on the dock and gets its configuration back.
        self.assertIn(("on", "B"), events)
        self.assertIn("B", steps.restored)
        self.assertFalse(power.off_ports)

    def test_a_unit_whose_configuration_fails_is_never_undocked_or_restored(self):
        logs, events, steps, power, _ = run(fail={("configure", "A"): OSError("dock silent")})
        self.assertEqual(logs["A"].errors, ["configure: OSError: dock silent"])
        self.assertNotIn("A", names(events, "off") + names(events, "start"))
        self.assertNotIn("A", steps.restored)
        self.assertTrue(logs["B"].ok and logs["C"].ok)

    def test_a_unit_that_does_not_leave_usb_is_put_back_and_not_started(self):
        logs, events, steps, power, _ = run(fail={("undocked", "C"): TimeoutError("CDC still present")})
        self.assertTrue(logs["C"].errors[0].startswith("undock:"))
        self.assertNotIn("C", names(events, "start"))
        self.assertIn(("on", "C"), events)
        self.assertIn("C", steps.restored)

    def test_a_hub_switch_that_fails_off_is_still_switched_back_on(self):
        logs, events, steps, power, _ = run(power_fail={("off", "B")})
        self.assertTrue(logs["B"].errors[0].startswith("undock:"))
        self.assertIn(("on", "B"), events)
        self.assertFalse(power.off_ports)

    def test_a_stop_that_fails_is_reported_and_its_session_not_judged(self):
        logs, events, steps, power, _ = run(fail={("stop", "A"): TimeoutError("still logging")})
        self.assertEqual(logs["A"].errors, ["stop: TimeoutError: still logging"])
        self.assertIsNone(logs["A"].session)
        self.assertIn(("on", "A"), events)
        self.assertEqual(names(events, "stop"), ["A", "B", "C"])

    def test_a_restore_that_fails_does_not_stop_the_others_or_the_check(self):
        logs, events, steps, power, _ = run(fail={("restore", "A"): OSError("port gone")})
        self.assertEqual(logs["A"].errors, ["restore: OSError: port gone"])
        self.assertEqual(logs["A"].check, "checked A-001")
        self.assertEqual(set(steps.restored), {"B", "C"})

    def test_no_new_session_or_two_are_each_a_failure(self):
        logs, *_ = run(new={"A": [], "B": [Path("data/exp/B-001"), Path("data/exp/B-002")]})
        self.assertEqual(logs["A"].errors, ["session: expected one new session, found 0"])
        self.assertEqual(logs["B"].errors, ["session: expected one new session, found 2"])
        self.assertTrue(logs["C"].ok)

    def test_a_drive_that_does_not_come_back_is_a_re_dock_failure(self):
        logs, events, steps, power, _ = run(fail={("sessions-after", "B"): TimeoutError("SD card did not mount")})
        self.assertEqual(logs["B"].errors, ["re-dock: TimeoutError: SD card did not mount"])
        self.assertTrue(logs["A"].ok and logs["C"].ok)
        self.assertEqual(names(events, "on"), ["A", "B", "C"])
        self.assertIn("B", steps.restored)

    def test_when_no_log_starts_there_is_no_wait(self):
        fail = {("start", n): TimeoutError("no status") for n in "ABC"}
        logs, events, steps, power, sleeps = run(fail=fail)
        self.assertEqual(sleeps, [])
        self.assertFalse(power.off_ports)
        self.assertEqual(set(steps.restored), set("ABC"))


class ExtraSessions(unittest.TestCase):
    """A unit the steps themselves undocked again after the re-dock has written a
    session after its log's own. Only the steps can say so."""

    TWO = [Path("data/exp/B-002"), Path("data/exp/B-001")]

    def test_with_a_reason_the_lowest_numbered_session_is_judged_and_the_rest_noted(self):
        logs, *_ = run(new={"B": self.TWO}, reasons={"B": "moved between slots"})
        self.assertTrue(logs["B"].ok, logs["B"].errors)
        self.assertEqual(logs["B"].session, ROOT / "data/exp/B-001")
        self.assertEqual(logs["B"].check, "checked B-001")
        self.assertEqual(logs["B"].notes,
                         ["1 more session(s) after the log's own (B-002), not judged: moved between slots"])
        self.assertEqual((logs["A"].notes, logs["C"].notes), ([], []))

    def test_without_a_reason_two_sessions_are_still_a_failure(self):
        logs, *_ = run(new={"B": self.TWO}, reasons={})
        self.assertEqual(logs["B"].errors, ["session: expected one new session, found 2"])
        self.assertIsNone(logs["B"].session)

    def test_steps_without_the_method_fail_two_sessions_as_before(self):
        logs, *_ = run(new={"B": self.TWO})
        self.assertEqual(logs["B"].errors, ["session: expected one new session, found 2"])

    def test_a_reason_is_asked_for_only_when_there_is_more_than_one(self):
        logs, events, *_ = run(new={"B": self.TWO}, reasons={"B": "moved"})
        self.assertEqual(names(events, "reason"), ["B"])

    def test_a_reason_does_not_excuse_a_log_with_no_session(self):
        logs, *_ = run(new={"A": []}, reasons={"A": "moved"})
        self.assertEqual(logs["A"].errors, ["session: expected one new session, found 0"])


class Interrupted(unittest.TestCase):
    def interrupted(self, **kw):
        events = []
        steps = FakeSteps(events, kw.pop("fail", None))
        power = FakePower(events)
        with self.assertRaises(KeyboardInterrupt):
            shared_log.log_together(UNITS, power, 600, steps, **kw)
        return events, steps, power

    def test_an_interrupt_during_the_log_still_re_docks_and_restores_every_unit(self):
        def sleep(s):
            raise KeyboardInterrupt
        events, steps, power = self.interrupted(sleep=sleep)
        self.assertFalse(power.off_ports)
        self.assertEqual(set(steps.restored), set("ABC"))
        self.assertEqual(names(events, "stop"), [])      # nothing more is asked of the units

    def test_an_interrupt_mid_re_dock_puts_back_the_rest(self):
        events, steps, power = self.interrupted(fail={("sessions-after", "A"): KeyboardInterrupt()},
                                                sleep=lambda s: None)
        self.assertFalse(power.off_ports)
        self.assertEqual(names(events, "on"), ["A", "B", "C"])
        self.assertEqual(set(steps.restored), set("ABC"))
        self.assertEqual(names(events, "check"), [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
