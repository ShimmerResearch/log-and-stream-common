#!/usr/bin/env python3
"""Unit tests for rig.s3_slots: each Shimmer3 found in the Base6 by the MAC it
reports, so a unit docked in another's slot is not addressed as that other
unit. Run:
python Test/hil/test_hil_s3_slots.py

No hardware: the Base6 and the dock are stand-ins.
"""

import contextlib
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rig import dock, s3_slots  # noqa: E402


class FakeBase6:
    """`slots` maps a slot to the MAC its unit reports, or None for a unit that does not answer."""

    uart_port = "COM14"

    def __init__(self, slots):
        self.slots, self.selected = dict(slots), None

    def occupied(self):
        return sorted(self.slots)

    def select(self, slot):
        self.selected = slot


class FakeDock:
    def __init__(self, mac):
        self._mac = mac

    def mac(self):
        return self._mac


def unit(name, slot, mac, platform="shimmer3"):
    return types.SimpleNamespace(name=name, platform=platform, base6_slot=slot, mac=mac)


A, B, C = "000666AAAAA1", "000666BBBBB2", "000666CCCCC3"


class Locate(unittest.TestCase):
    def locate(self, slots, units):
        b6 = FakeBase6(slots)

        @contextlib.contextmanager
        def open_dock(port, attempts=10, delay_s=0.5):
            mac = b6.slots.get(b6.selected)
            if mac is None:
                raise dock.DockError(f"{port}: no answer")
            yield FakeDock(mac.lower())             # case is not the unit's to decide

        with mock.patch.object(dock, "open_when_ready", open_dock):
            return s3_slots.locate(b6, units)

    def test_units_in_their_own_slots_are_left_alone(self):
        units = [unit("a", 1, A), unit("b", 2, B)]
        self.assertEqual(self.locate({1: A, 2: B}, units), [])
        self.assertEqual([u.base6_slot for u in units], [1, 2])

    def test_two_units_in_each_others_slots_are_each_pointed_at_where_they_are(self):
        units = [unit("a", 1, A), unit("b", 3, B), unit("c", 2, C)]
        lines = self.locate({1: B, 2: C, 3: A}, units)
        self.assertEqual([u.base6_slot for u in units], [3, 1, 2])
        self.assertEqual(lines, ["a answers in Base6 slot 3, not its rig.toml slot 1: the run uses slot 3",
                                 "b answers in Base6 slot 1, not its rig.toml slot 3: the run uses slot 1"])

    def test_a_unit_found_nowhere_keeps_an_empty_or_silent_slot(self):
        units = [unit("a", 1, A), unit("b", 2, B)]
        lines = self.locate({1: A, 2: None}, units)
        self.assertEqual([u.base6_slot for u in units], [1, 2])
        self.assertEqual(lines, ["b answers in no Base6 slot (occupied: [1, 2]): "
                                 "the run looks for it in its rig.toml slot 2"])

    def test_a_unit_found_nowhere_loses_a_slot_another_unit_answers_in(self):
        stranger = "000666DDDDD4"                   # in no rig unit
        units = [unit("a", 1, A), unit("b", 2, B)]
        lines = self.locate({1: A, 2: stranger}, units)
        self.assertEqual([u.base6_slot for u in units], [1, None])
        self.assertEqual(lines, [f"b answers in no Base6 slot, and {stranger} is in its slot 2: "
                                 "its dock tests are not covered"])

    def test_only_shimmer3s_with_a_mac_and_a_slot_are_looked_for(self):
        units = [unit("r", None, A, platform="shimmer3r"), unit("n", 2, None), unit("s", None, B)]
        b6 = FakeBase6({1: A})
        with mock.patch.object(dock, "open_when_ready", side_effect=AssertionError("no unit to look for")):
            self.assertEqual(s3_slots.locate(b6, units), [])
        self.assertEqual([u.base6_slot for u in units], [None, 2, None])

    def test_scan_names_each_answering_slot_in_upper_case(self):
        b6 = FakeBase6({1: A, 4: None, 6: C})

        @contextlib.contextmanager
        def open_dock(port, attempts=10, delay_s=0.5):
            mac = b6.slots.get(b6.selected)
            if mac is None:
                raise dock.DockError("no answer")
            yield FakeDock(mac.lower())

        with mock.patch.object(dock, "open_when_ready", open_dock):
            self.assertEqual(s3_slots.scan(b6), {1: A, 6: C})


if __name__ == "__main__":
    unittest.main(verbosity=2)
