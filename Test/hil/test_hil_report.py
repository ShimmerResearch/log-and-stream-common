#!/usr/bin/env python3
"""Unit tests for hil_report.py: how test outcomes become report items.
Run: python Test/hil/test_hil_report.py  (no hardware)
"""

import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hil_gate  # noqa: E402
import hil_report  # noqa: E402

SHA = "0123456789abcdef0123456789abcdef01234567"


def make(col):
    return col.report(platform="shimmer3r", commit=SHA, firmware_version="1.01.022",
                      bench="b", operator="o")


class Aggregation(unittest.TestCase):
    def status(self, *outcomes):
        c = hil_report.Collector()
        for o in outcomes:
            c.record("x", o)
        return make(c)["results"][0]["status"]

    def test_all_pass(self):
        self.assertEqual(self.status("pass", "pass"), "pass")

    def test_one_failure_wins(self):
        self.assertEqual(self.status("pass", "fail", "not_covered"), "fail")

    def test_partial_coverage_is_not_a_pass(self):
        # One unit ran, another could not: the item is not fully covered.
        self.assertEqual(self.status("pass", "not_covered"), "not_covered")

    def test_order_does_not_matter(self):
        self.assertEqual(self.status("fail", "pass"), "fail")

    def test_unknown_status_rejected(self):
        with self.assertRaises(ValueError):
            hil_report.Collector().record("x", "ok")


class Report(unittest.TestCase):
    def test_report_is_what_the_gate_accepts(self):
        c = hil_report.Collector()
        c.record("smoke.a", "pass", "u1: fine")
        c.record("smoke.b", "not_covered", "u1: bench lacks: hub")
        rep = make(c)
        self.assertEqual(hil_gate.validate_report("r", rep), [])
        self.assertEqual(rep["kind"], "automated")

    def test_notes_are_bounded(self):
        c = hil_report.Collector()
        for i in range(10):
            c.record("x", "pass", f"unit{i}: " + "y" * 200)
        note = make(c)["results"][0]["note"]
        self.assertLessEqual(len(note), 500)

    def test_bad_commit_refused_at_write_time(self):
        c = hil_report.Collector()
        c.record("x", "pass")
        with self.assertRaises(ValueError):
            c.report(platform="shimmer3r", commit="abc", firmware_version="1", bench="b", operator="o")


class Waivers(unittest.TestCase):
    def report(self, waivers):
        c = hil_report.Collector()
        c.record("a.fails", "fail", "u1: lost 2 samples")
        c.record("a.gap", "not_covered")
        c.record("a.passes", "pass")
        return c.report(platform="shimmer3r", commit=SHA, firmware_version="1.01.022",
                        bench="b", operator="o", waivers=waivers)

    def test_a_waived_failure_or_gap_carries_its_issue_key(self):
        r = {x["id"]: x for x in self.report({"a.fails": "DEV-1098", "a.gap": "DEV-1", "a.passes": "DEV-2"})["results"]}
        self.assertEqual((r["a.fails"]["waiver"], r["a.gap"]["waiver"]), ("DEV-1098", "DEV-1"))
        self.assertNotIn("waiver", r["a.passes"], "a pass needs no waiver")

    def test_the_gate_lets_a_waived_failure_through_and_blocks_an_unwaived_one(self):
        required = [{"id": "a.passes"}]
        rep = self.report({"a.fails": "DEV-1098"})
        v = hil_gate.evaluate("shimmer3r", SHA, "1.01.022", {"r.json": rep}, required)
        self.assertTrue(v.ok, v.problems)
        rep = self.report({})
        v = hil_gate.evaluate("shimmer3r", SHA, "1.01.022", {"r.json": rep}, required)
        self.assertFalse(v.ok)

    def test_a_waiver_that_is_not_an_issue_key_is_refused(self):
        with self.assertRaises(ValueError):
            self.report({"a.fails": "accepted"})


class Hardware(unittest.TestCase):
    """run.hardware: the host, the kit and the units a session tested."""

    def _unit(self, **kw):
        from rig import config
        return config.Unit(**{"name": "u", "platform": "shimmer3r", **kw})

    def test_only_the_tested_units_of_the_platform_once_each(self):
        a = self._unit(name="a", mac="CF9B9E3615BC", radio="IF820 v1.4.18.18 test_254", hub_port=1)
        b = self._unit(name="b", platform="shimmer3", base6_slot=3, radio="RN42 Ver 6.15")
        items = [types.SimpleNamespace(callspec=types.SimpleNamespace(params={"unit": u})) for u in (a, b, a)]
        items.append(types.SimpleNamespace())           # an item with no unit
        self.assertEqual([u.name for u in hil_report.tested_units(items, "shimmer3r")], ["a"])
        self.assertEqual(hil_report.tested_units(items, "shimmer3r", only="x"), [])
        hw = hil_report.hardware({"name": "PC"}, {"hub_serial": "H"}, hil_report.tested_units(items, "shimmer3"))
        self.assertEqual(hw["units"], [{"name": "b", "platform": "shimmer3", "radio": "RN42 Ver 6.15", "base6_slot": 3}])
        self.assertEqual(hw["host"], {"name": "PC"})

    def test_the_kit_names_each_st_link_once(self):
        from rig import config
        rig = config.Rig(path=Path("rig.toml"), bench="b", hub_serial="H", base6_serial="B6", stlink_serial="S1",
                         units=[self._unit(name="a", stlink_serial="S1"), self._unit(name="b", stlink_serial="S2")])
        self.assertEqual(rig.kit(), {"hub_serial": "H", "base6_serial": "B6", "stlink_serials": ["S1", "S2"]})

    def test_adapters_parse_from_one_object_a_list_or_nothing(self):
        from rig import host
        one = '{"name":"Intel(R) Wireless Bluetooth(R)","driver":"24.40.0.3"}'
        self.assertEqual(host.parse_adapters(one), [{"name": "Intel(R) Wireless Bluetooth(R)", "driver": "24.40.0.3"}])
        self.assertEqual(len(host.parse_adapters("[" + one + "," + one + "]")), 2)
        self.assertEqual(host.parse_adapters(""), [])
        self.assertEqual(host.parse_adapters("not json"), [])

    def test_a_report_with_hardware_still_passes_the_gate_check(self):
        col = hil_report.Collector()
        col.record("smoke.boot_identity", "pass")
        rep = col.report(platform="shimmer3r", commit="a" * 40, firmware_version="1.01.022", bench="b",
                         operator="o", extra={"hardware": hil_report.hardware({"name": "PC"}, {}, [])})
        self.assertEqual(rep["run"]["hardware"]["units"], [])


# What the Shimmer3R dock reported on 20 Nov 2026 (COMP_BLUETOOTH / PROP_BT_VER, v1.01.027).
TEST_254 = "CYW20820 app=v01.04.18.18, stack=0x00000000, protocol=0x0104, hardware=0xf1, build=test_254"
TEST_251 = "CYW20820 app=v01.04.18.18, stack=0x00000000, protocol=0x0104, hardware=0xf1, build=test_251"
V1_4_16_16 = "CYW20820 app=v01.04.16.16, stack=0x00000000, protocol=0x0104, hardware=0xf1"


class RadioReported(unittest.TestCase):
    """run.hardware.units[].radio_reported: each unit's module firmware, read over its dock."""

    def _unit(self, **kw):
        from rig import config
        return config.Unit(**{"name": "u", "platform": "shimmer3r", **kw})

    def test_each_unit_gets_its_string_or_the_reason_it_has_none(self):
        units = [self._unit(name=n) for n in ("a", "b", "c", "d", "e")]
        replies = {"a": TEST_254, "c": OSError("could not open port 'COM9'"), "d": "", "e": V1_4_16_16}

        def read(unit):
            r = replies[unit.name]
            if isinstance(r, Exception):
                raise r
            return r

        got = hil_report.read_radios(units, read, lambda u: "bench lacks: usb" if u.name == "b" else None)
        self.assertEqual(got, {
            "a": {"radio_reported": TEST_254},
            "b": {"radio_unread": "bench lacks: usb"},
            "c": {"radio_unread": "OSError: could not open port 'COM9'"},
            "d": {"radio_unread": "the dock replied with an empty string"},
            "e": {"radio_reported": V1_4_16_16},
        })

    def test_a_unit_the_bench_cannot_reach_is_not_read(self):
        def read(unit):
            raise AssertionError("read a unit the bench lacks the dock for")
        got = hil_report.read_radios([self._unit(name="a")], read, lambda u: "bench lacks: dock")
        self.assertEqual(got, {"a": {"radio_unread": "bench lacks: dock"}})

    def test_the_reported_string_sits_beside_the_rigs_radio(self):
        a = self._unit(name="a", radio="IF820 v1.4.18.18 test_254", hub_port=1)
        b = self._unit(name="b", radio="IF820", hub_port=4)
        c = self._unit(name="c", radio="IF820 v1.4.16.16", hub_port=3)
        radios = {"a": {"radio_reported": TEST_254}, "b": {"radio_reported": TEST_251}}
        units = hil_report.hardware({}, {}, [a, b, c], radios)["units"]
        self.assertEqual(units[0], {"name": "a", "platform": "shimmer3r", "radio": "IF820 v1.4.18.18 test_254",
                                    "hub_port": 1, "radio_reported": TEST_254})
        self.assertEqual(units[1]["radio_reported"], TEST_251)
        self.assertNotIn("radio_reported", units[2])     # not read: no entry, as in a report from before

    def test_a_report_with_and_without_it_passes_the_gate_check(self):
        a = self._unit(name="a", radio="IF820 v1.4.16.16")
        for radios in ({"a": {"radio_reported": V1_4_16_16}}, {"a": {"radio_unread": "bench lacks: usb"}}, None):
            col = hil_report.Collector()
            col.record("smoke.boot_identity", "pass")
            rep = col.report(platform="shimmer3r", commit=SHA, firmware_version="1.01.027", bench="b",
                             operator="o", extra={"hardware": hil_report.hardware({}, {}, [a], radios)})
            self.assertEqual(hil_gate.validate_report("r.json", rep), [])
            v = hil_gate.evaluate("shimmer3r", SHA, "1.01.027", {"r.json": rep}, [{"id": "smoke.boot_identity"}])
            self.assertTrue(v.ok, v.problems)


if __name__ == "__main__":
    unittest.main(verbosity=2)
