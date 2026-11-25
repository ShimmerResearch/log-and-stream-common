"""Which Base6 slot each Shimmer3 is really in, found by the MAC it reports over the dock.

rig.toml gives each Shimmer3 a base6_slot, and the harness addresses a unit
through that slot: its InfoMem writes and restores, its resets, its card. Units
come out of the Base6 and go back by hand, and on the bench two went back into
each other's slots: every test would then have configured, reset and judged
one unit as the other.

So a run asks every occupied slot which unit it holds before addressing any,
and points each Shimmer3 at the slot it answers in (locate()). A unit found
nowhere keeps its rig.toml slot, so its tests fail on a unit that does not
answer, unless another unit answers there. Then its slot is cleared: the bench
lacks its dock for this run, and its dock tests are not_covered rather than run
against the other unit.
"""

from __future__ import annotations

from . import dock

DOCK_ATTEMPTS = 6       # dock opens per slot: a unit just docked may still be booting


def scan(base6, attempts: int = DOCK_ATTEMPTS) -> dict[int, str]:
    """{slot: MAC} for every occupied slot whose unit answers on the dock."""
    found = {}
    for slot in base6.occupied():
        base6.select(slot)
        try:
            with dock.open_when_ready(base6.uart_port, attempts=attempts) as d:
                found[slot] = d.mac().upper()
        except (dock.DockError, OSError):
            continue                        # a unit that does not answer cannot be named
    return found


def locate(base6, units, attempts: int = DOCK_ATTEMPTS) -> list[str]:
    """Point each Shimmer3 of `units` that has a MAC and a base6_slot at the slot it
    answers in, changing its base6_slot. Returns one line per unit not in its
    rig.toml slot, saying what the run does about it. Other units are untouched."""
    s3 = [u for u in units if u.platform == "shimmer3" and u.mac and u.base6_slot]
    if not s3:
        return []
    answered = scan(base6, attempts)
    slot_of = {mac: slot for slot, mac in answered.items()}
    lines = []
    for u in s3:
        own, mac = u.base6_slot, u.mac.upper()
        now = slot_of.get(mac)
        if now == own:
            continue
        if now is not None:
            lines.append(f"{u.name} answers in Base6 slot {now}, not its rig.toml slot {own}: "
                         f"the run uses slot {now}")
            u.base6_slot = now
        elif answered.get(own) is not None:
            lines.append(f"{u.name} answers in no Base6 slot, and {answered[own]} is in its slot {own}: "
                         "its dock tests are not covered")
            u.base6_slot = None
        else:
            lines.append(f"{u.name} answers in no Base6 slot (occupied: {sorted(base6.occupied()) or 'none'}): "
                         f"the run looks for it in its rig.toml slot {own}")
    return lines
