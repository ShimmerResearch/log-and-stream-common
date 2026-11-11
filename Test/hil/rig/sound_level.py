"""The sound-level channel's checks (docs/SHIMMER3R_AUDIO.md section 8, DEV-1413).

Channel 0x38: int16, hundredths of a dB(A) SPL, -32768 while there is no
level. Configuration byte 233 bit 4 turns it on. A row checks that the channel
is there exactly when the bit is set, that the level arrives within the
capture's start-up, that it never drops back to none while sensing, that it is
a level a room can have, and that it changes no more often than once per
128 ms block.
"""
from __future__ import annotations

CHANNEL = 0x38
NONE = -32768
BLOCK_S = 0.128
# The first level comes with the third block: 256 to 384 ms after sensing
# starts, plus the task's wait
FIRST_BY_S = 0.5
# A quiet bench room reads about 40-51 dB(A); the microphones' self-noise
# about 26. The loudest an int16 capture can show is 111.1 dB(A).
LOW_DB, HIGH_DB = 20.0, 110.0


def check(values, rate_hz: float, enabled: bool) -> tuple[list[str], str]:
    """values: the channel's samples in order, or None when the packets have no
    0x38. Returns (problems, note)."""
    if values is None:
        return (["sound level on (byte 233 bit 4), but no channel 0x38 in the packets"] if enabled else [], "")
    if not enabled:
        return (["channel 0x38 in the packets with byte 233 bit 4 clear"], "")
    v = [int(x) for x in values]
    if not v:
        return (["no samples to check the sound level on"], "")
    first = next((i for i, x in enumerate(v) if x != NONE), None)
    if first is None:
        return ([f"all {len(v)} samples say no level (-32768)"], "")
    probs = []
    if first / rate_hz > FIRST_BY_S:
        probs.append(f"first sound level {first / rate_hz:.2f} s in, expected within {FIRST_BY_S} s")
    gaps = sum(1 for x in v[first:] if x == NONE)
    if gaps:
        probs.append(f"{gaps} samples say no level (-32768) after the first level")
    db = [x / 100 for x in v[first:] if x != NONE]
    outside = [x for x in db if not LOW_DB <= x <= HIGH_DB]
    if outside:
        probs.append(f"{len(outside)} levels outside {LOW_DB:.0f}-{HIGH_DB:.0f} dB(A), e.g. {outside[0]:.2f}")
    changes = sum(1 for a, b in zip(v[first:], v[first + 1:]) if a != b)
    span_s = (len(v) - first) / rate_hz
    most = span_s / BLOCK_S * 1.1 + 2
    if changes > most:
        probs.append(f"the level changed {changes} times in {span_s:.1f} s: more than once a 128 ms block")
    ordered = sorted(db)
    note = (f"sound level {ordered[0]:.1f}-{ordered[-1]:.1f} dB(A), median {ordered[len(ordered) // 2]:.1f}, "
            f"first after {first / rate_hz * 1000:.0f} ms") if db else ""
    return probs, note
