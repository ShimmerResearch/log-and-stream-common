"""pytest plugin for the hardware-in-the-loop suite.

    python -m pytest Test/hil -m smoke --rig my-rig.toml \
        --candidate build-info.json --hil-report out/hil-report-<bench>.json

Every hardware test is marked with the report item it provides evidence for,
and what it needs from the bench:

    @pytest.mark.hil("smoke.boot", needs=["usb"])

Before a test runs, its needs are checked against rig.toml (rig/config.py). If
the bench cannot satisfy them the test is skipped AND its item is recorded as
not_covered - so a bench without a hub produces a report that says exactly
which items another bench must cover, rather than one that looks complete.

--candidate names the build-info.json published with the release candidate.
It binds the report to that build (commit and version) and is what the tests
compare the unit's reported version against. Without it, tests still run but
no report is written: a report that cannot name its build is not evidence.
"""

from __future__ import annotations

import getpass
import json
import os
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import hil_gate  # noqa: E402
import hil_report  # noqa: E402
from rig import config as rig_config  # noqa: E402
from rig.power import HubPower, ManualPower  # noqa: E402


def pytest_addoption(parser):
    g = parser.getgroup("hil", "hardware-in-the-loop")
    g.addoption("--rig", default=None, help="rig.toml (default: $HIL_RIG, else Test/hil/rig.toml)")
    g.addoption("--candidate", default=None, help="build-info.json of the release candidate under test")
    g.addoption("--hil-report", default=None, help="where to write the report (needs --candidate)")
    g.addoption("--operator", default=getpass.getuser())
    g.addoption("--unit", default=None,
                help="run against this rig unit only; the others are reported not_covered, so the "
                     "report never passes an item for a unit it did not test (default: every unit)")
    g.addoption("--manual-power", action="store_true",
                help="prompt the operator to dock/undock even if the rig has a hub")
    g.addoption("--waive", action="append", default=[], metavar="ITEM=ISSUE",
                help="accept a known failure or gap of ITEM under issue ISSUE (e.g. "
                     "smoke.sd_log=ABC-123): the key goes into the item's result if it does not "
                     "pass, as a sign-off's waiver does. Repeatable.")


def _waivers(config) -> dict[str, str]:
    out = {}
    for w in config.getoption("--waive"):
        item, sep, key = w.partition("=")
        item, key = item.strip(), key.strip().upper()
        if not sep or not item or not hil_gate.WAIVER_RE.fullmatch(key):
            raise pytest.UsageError(f"--waive {w}: expected ITEM=ISSUE, e.g. smoke.sd_log=ABC-123")
        out[item] = key
    return out


def pytest_configure(config):
    config.addinivalue_line("markers", "hil(item_id, needs=[]): report item and required bench capabilities")
    config.addinivalue_line("markers", "smoke: the smoke tier - required for every release candidate")
    config.addinivalue_line("markers", "release: the release tier")
    config.addinivalue_line("markers", "fleet: one unit per shipped radio firmware (Shimmer3)")
    config.addinivalue_line("markers", "soak: long-running, advisory")
    config._hil_collector = hil_report.Collector()
    config._hil_waivers = _waivers(config)


# Processes known to distort a run on this kind of bench. STM32CubeIDE scans
# every COM port on USB arrival, paging absent Bluetooth devices and cutting
# classic throughput for most of a minute; Consensys holds dock ports open.
INTERFERING = {"stm32cubeide.exe": "STM32CubeIDE (pages Bluetooth ports on USB arrival)",
               "consensys.exe": "Consensys (holds dock ports)"}


def _running(names):
    if sys.platform != "win32":
        return []
    import subprocess
    out = subprocess.run(["tasklist", "/FO", "CSV", "/NH"], capture_output=True,
                         text=True, encoding="utf-8", errors="replace").stdout.lower()
    return [desc for exe, desc in names.items() if f'"{exe}"' in out]


def pytest_sessionstart(session):
    session.config._hil_interference = _running(INTERFERING)


def pytest_report_header(config):
    found = getattr(config, "_hil_interference", None) or _running(INTERFERING)
    if found:
        return [f"hil: WARNING - running now: {', '.join(found)}. "
                "Bluetooth timing results may not be representative; close them for a release run."]
    return []


# --- session fixtures --------------------------------------------------------

@pytest.fixture(scope="session")
def rig(pytestconfig):
    try:
        return rig_config.load(pytestconfig.getoption("--rig"))
    except rig_config.RigError as e:
        pytest.exit(f"rig: {e}", returncode=3)


@pytest.fixture(scope="session")
def candidate(pytestconfig):
    """build-info.json of the build under test, or None for an unbound run."""
    p = pytestconfig.getoption("--candidate")
    if not p:
        return None
    with open(p, encoding="utf-8") as f:
        info = json.load(f)
    for key in ("platform", "commit", "firmware_version"):
        if not info.get(key):
            pytest.exit(f"{p}: missing {key}", returncode=3)
    return info


@pytest.fixture(scope="session")
def power(rig, pytestconfig):
    if pytestconfig.getoption("--manual-power") or "hub" not in rig.capabilities():
        p = ManualPower()
    else:
        p = HubPower(rig.bench_tools, rig.hub_serial)
    yield p
    p.close()


# What rig/btmode.py needs to change a unit's InfoMem and then reset it: a
# Shimmer3R's USB dock and STM32_Programmer_CLI (every Shimmer3R reset route,
# SWD or bootloader, goes through it); a Shimmer3's Base6 slot, which does both.
DOCK_ROUTE = {"shimmer3r": {"usb", "programmer:shimmer3r"}, "shimmer3": {"dock"}}


@pytest.fixture(scope="session", autouse=True)
def _bt_on(request):
    """Bluetooth on for every unit this run tests over Bluetooth (rig/btmode.py).

    Session-scoped and autouse, so it runs before any test module opens a dock
    or Base6 port and finishes after they have all closed theirs. A unit's
    InfoMem goes back at the end even when the run failed, and a unit that
    could not be put back fails the run loudly - left changed, it would no
    longer sync in the field.
    """
    from rig.btmode import BtMode, bench_routes

    # Only units a Bluetooth test will actually run against: one the bench
    # cannot serve is reported not_covered by pytest_runtest_setup, and
    # reaching for its dock here would fail the run before that. The fixture
    # also needs its own route to change InfoMem and reset the unit
    # (DOCK_ROUTE), which a Bluetooth-only test does not declare
    # (smoke.bt_stream needs only "bt"). A unit without it is left as it is,
    # and its Bluetooth tests run against whatever it is set to.
    units = {}
    only = request.config.getoption("--unit")
    for item in request.session.items:
        item_id, needs = _item(item)
        unit = item.callspec.params.get("unit") if hasattr(item, "callspec") else None
        if not (item_id and unit is not None and "bt" in needs and (not only or unit.name == only)):
            continue
        route = DOCK_ROUTE.get(unit.platform, set())
        if not _missing(request.config, unit, needs | route):
            units[unit.name] = unit
    if not units:
        yield
        return
    tr = request.config.pluginmanager.get_plugin("terminalreporter")
    def log(m):
        if tr is None:
            print(m)
            return
        # At teardown the progress dots are still on the line; without a fresh
        # line the restore confirmation is glued to them ("..hil: ..."). The
        # reporter's ensure_newline() tracks file paths, not what is on the
        # line, so ask the terminal writer.
        if getattr(tr._tw, "width_of_current_line", 0):
            tr.write("\n")
        tr.write_line(f"hil: {m}")
    mode = BtMode(*bench_routes(request.getfixturevalue("rig")), log=log)
    try:
        for unit in units.values():
            mode.enable(unit)
        yield
    finally:
        problems = mode.restore()
        if problems:
            pytest.fail("; ".join(problems), pytrace=False)


# --- per-unit parametrisation ------------------------------------------------

def _rig_for_collection(config):
    if not hasattr(config, "_hil_rig"):
        try:
            config._hil_rig = rig_config.load(config.getoption("--rig"))
        except rig_config.RigError:
            config._hil_rig = None
    return config._hil_rig


def pytest_generate_tests(metafunc):
    """A test that takes `unit` runs once per rig unit of the platform it targets.

    The platform comes from the test's needs (shimmer3 / shimmer3r). With no
    matching unit the test still appears once, with unit=None, so that it is
    reported as not covered instead of vanishing from the report.

    --unit does not drop the other units: they stay in the run and are skipped
    as not_covered (pytest_runtest_setup). An item is "pass" only when every
    test behind it passed, so a report from a --unit run cannot pass an item
    for a unit it never touched.
    """
    if "unit" not in metafunc.fixturenames:
        return
    rig = _rig_for_collection(metafunc.config)
    m = metafunc.definition.get_closest_marker("hil")
    needs = set(m.kwargs.get("needs", [])) if m else set()
    platforms = needs & set(rig_config.PLATFORMS)
    only = metafunc.config.getoption("--unit")
    if only and rig and only not in [u.name for u in rig.units]:
        raise pytest.UsageError(f"--unit {only}: no such unit in {rig.path}")
    units = []
    if rig:
        units = [u for u in rig.units if not platforms or u.platform in platforms]
    if units:
        metafunc.parametrize("unit", units, ids=[u.name for u in units], scope="module")
    else:
        metafunc.parametrize("unit", [None], ids=["no-unit"], scope="module")


def pytest_collection_modifyitems(config, items):
    """With a candidate named, run only the tests for its platform.

    A Shimmer3 test has nothing to say about a Shimmer3R build, and recording
    it - even as not_covered - would put the wrong platform's items in the
    report.
    """
    cand_path = config.getoption("--candidate")
    if not cand_path:
        return
    with open(cand_path, encoding="utf-8") as f:
        plat = json.load(f).get("platform")
    # --unit naming another platform's unit would skip every test of this
    # candidate as not_covered, and a run that tested nothing would look like
    # a run.
    only, rig = config.getoption("--unit"), _rig_for_collection(config)
    if only and rig:
        unit = next((u for u in rig.units if u.name == only), None)
        if unit is None:
            raise pytest.UsageError(f"--unit {only}: no such unit in {rig.path}")
        if unit.platform != plat:
            raise pytest.UsageError(f"--unit {only} is a {unit.platform} unit, but the candidate is {plat}")
    keep, drop = [], []
    for item in items:
        _, needs = _item(item)
        other = (needs & set(rig_config.PLATFORMS)) - {plat}
        (drop if other else keep).append(item)
    if drop:
        config.hook.pytest_deselected(items=drop)
        items[:] = keep


# --- capability check and outcome recording ----------------------------------

def _item(item):
    """(report item id, needs). The id may name test parameters - "release.{row}" -
    which are filled from the test's parameters; a dict parameter contributes its "id"."""
    m = item.get_closest_marker("hil")
    if not (m and m.args):
        return None, set()
    item_id = m.args[0]
    if "{" in item_id and hasattr(item, "callspec"):
        params = {k: (v.get("id") if isinstance(v, dict) else v) for k, v in item.callspec.params.items()}
        item_id = item_id.format(**params)
    return item_id, set(m.kwargs.get("needs", []))


def _missing(config, unit, needs: set[str]) -> set[str]:
    """What the bench lacks of `needs` for this unit: the tests it makes skip."""
    rig = _rig_for_collection(config)
    if rig is None:
        return needs or {"rig.toml"}
    have = rig.capabilities(unit)
    if config.getoption("--manual-power"):
        # Without the hub the operator is the switch, so docking is possible.
        have.add("hub_port")
    missing = needs - have
    if unit is None and needs & set(rig_config.PLATFORMS):
        missing |= needs & set(rig_config.PLATFORMS)
    return missing


def pytest_runtest_setup(item):
    item_id, needs = _item(item)
    if not item_id:
        return
    unit = item.callspec.params.get("unit") if hasattr(item, "callspec") else None
    only = item.config.getoption("--unit")
    if only and unit is not None and unit.name != only:
        item._hil_not_covered = f"not run: --unit {only}"
        pytest.skip(item._hil_not_covered)
    missing = _missing(item.config, unit, needs)
    if missing:
        item._hil_not_covered = f"bench lacks: {', '.join(sorted(missing))}"
        pytest.skip(item._hil_not_covered)


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    rep = outcome.get_result()
    item_id, _ = _item(item)
    if not item_id:
        return
    col = item.config._hil_collector
    unit = item.callspec.params.get("unit") if hasattr(item, "callspec") else None
    who = unit.name if unit else "-"
    if rep.when == "setup" and rep.skipped:
        col.record(item_id, "not_covered", f"{who}: {getattr(item, '_hil_not_covered', 'skipped')}")
    elif rep.failed:
        msg = str(rep.longrepr.reprcrash.message) if hasattr(rep.longrepr, "reprcrash") else str(rep.longrepr)
        col.record(item_id, "fail", f"{who}: {msg.splitlines()[0][:200] if msg else 'failed'}")
    elif rep.when == "call" and rep.skipped:
        # A test that decides at run time it cannot check something has not
        # checked it.
        reason = rep.longrepr[2] if isinstance(rep.longrepr, tuple) else "skipped"
        col.record(item_id, "not_covered", f"{who}: {reason}")
    elif rep.when == "call" and rep.passed:
        note = getattr(item, "_hil_note", None)
        col.record(item_id, "pass", f"{who}: {note}" if note else None)


@pytest.fixture
def hil_note(request):
    """Attach a short observation (a version string, a rate) to this test's item."""
    def note(text: str):
        prev = getattr(request.node, "_hil_note", None)
        request.node._hil_note = f"{prev}; {text}" if prev else text
    return note


def pytest_sessionfinish(session, exitstatus):
    cfg = session.config
    out = cfg.getoption("--hil-report")
    cand_path = cfg.getoption("--candidate")
    col = cfg._hil_collector
    if not col.items:
        return
    tr = cfg.pluginmanager.get_plugin("terminalreporter")
    if not (out and cand_path):
        if tr:
            tr.write_line("hil: no --candidate/--hil-report, so no report written "
                          "(an unbound run is not release evidence)")
        return
    if os.environ.get("HIL_MATRIX"):
        if tr:
            tr.write_line("hil: HIL_MATRIX is set - an experiment's rows are not release evidence, "
                          "so no report written")
        return
    with open(cand_path, encoding="utf-8") as f:
        cand = json.load(f)
    rig = _rig_for_collection(cfg)
    rep = col.report(
        platform=cand["platform"], commit=cand["commit"],
        firmware_version=cand["firmware_version"],
        bench=rig.bench if rig else "unknown", operator=cfg.getoption("--operator"),
        extra={"markers": cfg.getoption("-m") or "",
               "units": [u.name for u in rig.units if not cfg.getoption("--unit") or u.name == cfg.getoption("--unit")]
               if rig else [],
               "unit_filter": cfg.getoption("--unit") or "",
               "interference": getattr(cfg, "_hil_interference", []),
               "image_sha256": cand.get("image_sha256"), "host": os.environ.get("COMPUTERNAME", "")},
        waivers=cfg._hil_waivers,
    )
    path = hil_report.write(rep, Path(out))
    used = {r["id"] for r in rep["results"] if r.get("waiver")}
    for item, key in sorted(cfg._hil_waivers.items()):
        if tr and item not in used:
            tr.write_line(f"hil: --waive {item}={key} not used: the item passed, or was not run")
    if tr:
        counts = {}
        for r in rep["results"]:
            counts[r["status"]] = counts.get(r["status"], 0) + 1
        tr.write_line(f"hil: report written to {path}  {counts}")
