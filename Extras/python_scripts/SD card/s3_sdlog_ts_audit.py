#!/usr/bin/env python3
"""Report gaps and invalid records in a Shimmer3 LogAndStream SD data file.

A recording can be short of samples for two unrelated reasons, and this tells
them apart by WHERE in a block the gap lands:

  * a gap whose first record sits at block position 0 is the SD write buffers.
    Every buffer was queued when a record was offered, so the record was
    refused. The firmware counts these in sdWrBuf.diag.putsRefusedFull.
  * a gap starting mid-block is the sample ring: a tick refused because the
    ring was full or a gather was still outstanding, or the stall fail-safe
    restarting a packet. Counted in sensing.ring.diag.

A gap from either cause can land at position 0, so the split is evidence about
the run as a whole rather than proof about any one gap. The report prints how
many would land there by chance for comparison.

It also flags any record whose timestamp reads exactly zero. Records are
stamped when the sample tick starts them and are not written out otherwise, so
zero marks an invalid record rather than the counter's origin. A host that
unwraps naively reads it as a rollover and adds 512 s to every later sample.

Block layout, which is what makes the position test possible, and which is
specified in docs/SHIMMER3_SD_CARD_FORMAT.md sections 2.1 and 4.1:

    recordsPerBlock = floor((512 - syncHead) / recordLength)
    blockBytes      = syncHead + recordsPerBlock * recordLength

syncHead is 9 when the trial has SD sync enabled - the node's published clock
offset, a sign flag plus a 64-bit magnitude - and 0 otherwise. The header's sync
bit is cross-checked against the file length, because a layout that does not
tile the file means the parse is wrong, and the symptom of getting it wrong is
silent drift rather than an error.

Blocks are NOT 512 bytes. Only the used part of the buffer is written, so a
block is 512 rounded down to a whole number of records, and the final block of
a file is shorter again because it is flushed when logging stops.

Usage:
    python s3_sdlog_ts_audit.py <file-or-session-directory> [...] [options]

    --row N        override the record length in bytes (skip header derivation)
    --period N     override the sample period in ticks
    --sync-head N  force the per-block sync offset width (0 or 9)
    --no-sync      same as --sync-head 0
    --quiet        counts only, no per-gap listing

Give a session directory and every numerically-named file in it is read in
order (000, 001, 002 ...). Files given together are treated as one session and
summarised at the end.
"""
import os
import sys
from collections import Counter

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Shimmer_common"))

from util_shimmer import byte_array_to_int

RTC = 32768.0
SD_WRITE_BUF_SIZE = 512
HEADER_LEN = 256
TS_BYTES = 3
TS_MAX = 1 << (8 * TS_BYTES)
# The node's published clock offset: a sign flag plus a 64-bit magnitude.
SYNC_HEAD = 9
# SrHwVer.SHIMMER3 in Shimmer_common/shimmer_device.py. Not imported from
# there: that module pulls in pyserial, and this tool reads files offline.
HW_VER_SHIMMER3 = 3

# SDLogHeader bit -> bytes per record (ShimmerObject.SDLogHeader / ShimmerSDLog)
CHANNELS = [
    ("ACCEL_LN", 1 << 7, 6), ("BATTERY", 1 << 13, 2), ("EXT_EXP_A7", 1 << 1, 2),
    ("EXT_EXP_A6", 1 << 0, 2), ("EXT_EXP_A15", 1 << 11, 2), ("INT_EXP_A12", 1 << 9, 2),
    ("INT_EXP_A13", 1 << 8, 2), ("INT_EXP_A14", 1 << 23, 2), ("BRIDGE_AMP", 1 << 15, 4),
    ("GSR", 1 << 2, 2), ("INT_EXP_A1", 1 << 10, 2), ("GYRO", 1 << 6, 6),
    ("ACCEL_WR", 1 << 12, 6), ("MAG", 1 << 5, 6), ("ACCEL_MPU", 1 << 22, 6),
    ("MAG_MPU", 1 << 21, 6), ("BMPX80", 1 << 18, 5), ("EXG1_24BIT", 1 << 4, 7),
    ("EXG2_24BIT", 1 << 3, 7), ("EXG1_16BIT", 1 << 20, 5), ("EXG2_16BIT", 1 << 19, 5),
]


def duration_str(seconds):
    """A duration, not a wall-clock time. Shimmer_common's seconds_to_time_str
    formats an absolute date from an epoch, which is a different question."""
    return "%dm%02ds" % divmod(int(seconds), 60)


def fit_error(body_len, row, sync_head):
    """How badly a block layout fails to tile the file. Zero is a clean fit;
    one short final block is expected, so only bytes that cannot belong to a
    legal short block count against the layout."""
    per_block = (SD_WRITE_BUF_SIZE - sync_head) // row
    if per_block < 1:
        return None
    block_bytes = sync_head + per_block * row
    remainder = body_len % block_bytes
    if remainder == 0:
        return 0
    tail = remainder - sync_head
    if tail < row:
        return remainder
    return tail % row


def choose_sync_head(body_len, row, header_says_sync, forced):
    """Confirm the header's sync bit against the file length. A layout that
    does not tile the file means the parse is wrong, and the failure mode is
    silent drift, so it is worth catching before reading any timestamps."""
    if forced is not None:
        return forced, "forced"
    expected = SYNC_HEAD if header_says_sync else 0
    best, best_error = expected, None
    for width in (expected, SYNC_HEAD if not header_says_sync else 0):
        error = fit_error(body_len, row, width)
        if error is None:
            continue
        if best_error is None or error < best_error:
            best, best_error = width, error
        if error == 0:
            break
    if best == expected:
        note = "as the header says"
    else:
        note = "header implies %d, but %d tiles the file" % (expected, best)
    return best, note


def unwrap(raw):
    """Modulo-aware unwrap. A step backwards of more than half the range is a
    rollover; anything else is a record arriving out of order, which must not
    advance the counter. Zero timestamps are excluded by the caller, because
    reading one as a rollover is exactly the failure this reports."""
    out, total, prev = [], 0, raw[0]
    for value in raw:
        delta = (value - prev) % TS_MAX
        if delta > TS_MAX // 2:
            delta = 0  # out of order: hold the timeline
        total += delta
        out.append(total)
        prev = value
    return out


def decode_sync_offsets(heads):
    """The node's clock offset from the centre, published at the head of every
    block while SD sync is on.

    Byte 0 is a sign flag, bytes 1-8 a 64-bit magnitude in 32768 Hz ticks, LSB
    first. The firmware refills the head with 0xFF after each block consumes
    it, so an all-0xFF head means no new offset was published for that block
    rather than an offset of zero. See docs/SHIMMER3_SD_SYNC.md section 5.
    """
    offsets = []  # (blockIndex, signedTicks)
    for block, head in enumerate(heads):
        if head[0] == 0xFF:
            continue
        magnitude = byte_array_to_int(head[1:9])
        offsets.append((block, -magnitude if head[0] == 1 else magnitude))
    return offsets


def report_sync_offsets(heads, per_block, period):
    if not heads:
        return
    offsets = decode_sync_offsets(heads)
    print("  sync offsets: %d of %d blocks carry one" % (len(offsets), len(heads)))
    if not offsets:
        print("        every head is 0xFF - the node never completed a sync "
              "round, so this file cannot be aligned to the centre")
        return
    values = [v for _, v in offsets]
    first, last = values[0], values[-1]
    seconds_per_block = per_block * period / RTC
    print("        offset from centre: first %+.3f ms, last %+.3f ms, "
          "range %+.3f to %+.3f ms"
          % (first * 1000.0 / RTC, last * 1000.0 / RTC,
             min(values) * 1000.0 / RTC, max(values) * 1000.0 / RTC))
    span_blocks = offsets[-1][0] - offsets[0][0]
    if span_blocks > 0:
        drift_ms = (last - first) * 1000.0 / RTC
        span_s = span_blocks * seconds_per_block
        print("        drift %+.3f ms over %.1f s (%+.2f ms/min); a positive "
              "offset means the node is ahead"
              % (drift_ms, span_s, drift_ms / (span_s / 60.0) if span_s else 0.0))


def read_samples(body, row, sync_head):
    """Every record in the file, with where it sat in its block. Includes the
    short final block, which is written when logging stops."""
    per_block = (SD_WRITE_BUF_SIZE - sync_head) // row
    block_bytes = sync_head + per_block * row

    samples = []  # (timestamp, blockIndex, positionInBlock)
    heads = []    # the raw sync-offset bytes leading each block
    warning = None
    offset, block = 0, 0
    while offset < len(body):
        remaining = len(body) - offset
        if remaining <= sync_head:
            break
        count = (min(block_bytes, remaining) - sync_head) // row
        if sync_head:
            heads.append(body[offset:offset + sync_head])
        base = offset + sync_head
        for position in range(count):
            start = base + position * row
            samples.append(
                (byte_array_to_int(body[start:start + TS_BYTES]), block, position))
        if count < per_block:
            offset += sync_head + count * row
            block += 1
            if offset < len(body):
                warning = ("a short block at block %d is not the last one - "
                           "the record length or sync head is wrong" % block)
            break
        offset += block_bytes
        block += 1
    return samples, heads, per_block, block_bytes, warning


def report_gap_profile(boundary_gaps, mid_gaps, period, minutes):
    """Gap sizes, split by what caused them.

    Pooling the two populations hides the most useful thing in them. Firmware
    that gives up on a stalled packet after a fixed number of sample periods
    puts a hard ceiling on its mid-block gaps; firmware that waits a wall-clock
    timeout instead has no ceiling and a long tail. Those two look identical in
    a combined histogram and obviously different side by side, so the largest
    gap in each population is always reported, even when the histogram below it
    is truncated.
    """
    populations = [("block boundary", boundary_gaps), ("mid-block", mid_gaps)]
    if not any(group for _, group in populations):
        return

    print("  gap profile by cause:")
    print("      %-15s %6s %8s %7s %7s %6s %9s"
          % ("", "gaps", "samples", "mean", "median", "max", "per min"))
    for name, group in populations:
        if not group:
            print("      %-15s %6d" % (name, 0))
            continue
        sizes = sorted(g[5] for g in group)
        middle = len(sizes) // 2
        median = (sizes[middle] if len(sizes) % 2
                  else (sizes[middle - 1] + sizes[middle]) / 2.0)
        print("      %-15s %6d %8d %7.1f %7.1f %6d %9.2f"
              % (name, len(sizes), sum(sizes), sum(sizes) / float(len(sizes)),
                 median, sizes[-1], len(sizes) / minutes if minutes else 0.0))

    for name, group in populations:
        if not group:
            continue
        histogram = sorted(Counter(g[5] for g in group).items())
        print("  samples lost per gap, %s:" % name)
        for lost, count in histogram[:12]:
            print("      %6d samples (%8.3f s) x %d"
                  % (lost, lost * period / RTC, count))
        if len(histogram) > 12:
            rest = sum(count for _, count in histogram[12:])
            print("      ... and %d more, in sizes up to %d samples"
                  % (rest, histogram[-1][0]))


def report_loss_over_time(gaps, total_ticks, period, buckets=10):
    """Where in the recording the loss happened. Steady loss and a burst have
    very different causes, and a total hides the difference."""
    if not gaps or total_ticks <= 0:
        return
    lost_per_bucket = [0] * buckets
    for _, _, _, _, at_ticks, lost in gaps:
        index = min(buckets - 1, int(buckets * at_ticks / float(total_ticks)))
        lost_per_bucket[index] += lost
    peak = max(lost_per_bucket)
    if peak == 0:
        return
    print("  samples lost across the recording, in tenths:")
    span_s = total_ticks / RTC / buckets
    for i, lost in enumerate(lost_per_bucket):
        bar = "#" * int(round(20.0 * lost / peak)) if lost else ""
        print("      %6.0f-%6.0f s  %5d %s" % (i * span_s, (i + 1) * span_s, lost, bar))


def audit(path, options, session):
    print("=== %s ===" % path)
    try:
        data = open(path, "rb").read()
    except (IOError, OSError) as exc:
        print("  cannot read: %s\n" % exc)
        return
    if len(data) <= HEADER_LEN:
        print("  file is header-only or truncated\n")
        return

    header = data[:HEADER_LEN]

    # Shimmer3 only. A Shimmer3R file has a 384-byte header and records its own
    # channel order in it, so none of the arithmetic below applies and the
    # output would be confident nonsense.
    hardware_version = byte_array_to_int(header[30:32], lsb_order=False)
    if hardware_version != HW_VER_SHIMMER3:
        print("  header reports hardware version %d, not %d (Shimmer3)."
              % (hardware_version, HW_VER_SHIMMER3))
        print("  This tool reads Shimmer3 files only: Shimmer3R uses a "
              "384-byte header and a different")
        print("  channel order. Refusing to guess.\n")
        return

    divider = byte_array_to_int(header[0:2])
    period = options["period"] or divider
    rate = RTC / period if period else 0.0
    enabled = byte_array_to_int(header[3:8])
    header_sync = (header[16] >> 2) & 1
    firmware = byte_array_to_int(header[34:36], lsb_order=False)
    version = (byte_array_to_int(header[36:38], lsb_order=False), header[38], header[39])
    exp_board = list(header[214:217])
    initial_ticks = byte_array_to_int(header[252:256]) | (header[251] << 32)

    channels = [(name, width) for name, bit, width in CHANNELS if enabled & bit]
    row = options["row"] or (TS_BYTES + sum(width for _, width in channels))

    body = data[HEADER_LEN:]
    sync_head, sync_note = choose_sync_head(
        len(body), row, header_sync, options["sync_head"])
    samples, heads, per_block, block_bytes, warning = read_samples(
        body, row, sync_head)

    print("  HW %d  FW id %d  v%d.%02d.%03d   exp brd %d.%d.%d%s"
          % (hardware_version, firmware, version[0], version[1], version[2],
             exp_board[0], exp_board[1], exp_board[2],
             "   <-- unset (EEPROM not read)"
             if exp_board in ([0, 0, 0], [255, 255, 255]) else ""))
    print("  rate %.3f Hz (%d ticks/sample)   timestamp rolls over every %.1f s"
          % (rate, period, TS_MAX / RTC))
    print("  channels: %s" % ", ".join(name for name, _ in channels))
    print("  record %d B   block %d B = %dB sync head + %d records   (%s)"
          % (row, block_bytes, sync_head, per_block, sync_note))
    if warning:
        print("  NOTE: %s" % warning)

    if not samples:
        print("  no samples\n")
        return

    raw = [t for t, _, _ in samples]
    zeros = [i for i, t in enumerate(raw) if t == 0]
    zero_set = set(zeros)

    valid = [(i, t) for i, t in enumerate(raw) if i not in zero_set]
    unwrapped = unwrap([t for _, t in valid])

    true_seconds = len(raw) / rate if rate else 0.0
    naive_seconds = true_seconds + len(zeros) * (TS_MAX + period) / RTC

    print("  samples %d   sampling time %.2f s (%s)   file opened at %.2f s"
          % (len(raw), true_seconds, duration_str(true_seconds),
             initial_ticks / RTC))
    print("  zero timestamps: %d" % len(zeros))
    if zeros:
        print("  >>> a host that unwraps naively will report %.2f s (%s), "
              "%.2fx too long"
              % (naive_seconds, duration_str(naive_seconds),
                 naive_seconds / true_seconds if true_seconds else 0))

    # A gap is a step that skipped at least one whole sample. The step is
    # rounded to the nearest number of periods first, because the sample timer
    # jitters by a tick or so and a 66-tick step at a 65-tick period is not a
    # gap. Attributed by where the record AFTER the gap sits in its block.
    gaps = []
    for n in range(1, len(unwrapped)):
        step = unwrapped[n] - unwrapped[n - 1]
        lost = int(round(float(step) / period)) - 1
        if lost >= 1:
            index = valid[n][0]
            _, block, position = samples[index]
            gaps.append((index, step, block, position, unwrapped[n - 1], lost))

    boundary_gaps = [g for g in gaps if g[3] == 0]
    mid_gaps = [g for g in gaps if g[3] != 0]
    lost_at_boundary = sum(g[5] for g in boundary_gaps)
    lost_mid_block = sum(g[5] for g in mid_gaps)
    lost_total = lost_at_boundary + lost_mid_block
    expected_total = len(raw) + lost_total

    print("  gaps: %d at a block boundary (%d samples), "
          "%d mid-block (%d samples)"
          % (len(boundary_gaps), lost_at_boundary, len(mid_gaps),
             lost_mid_block))
    if gaps:
        print("        %.1f of the %d would land at a boundary by chance"
              % (len(gaps) / float(per_block), len(gaps)))
    if boundary_gaps:
        print("        a boundary excess means the SD write buffers refused "
              "records; compare with sdWrBuf.diag.putsRefusedFull")
    if mid_gaps:
        print("        mid-block gaps are the sample ring refusing ticks or "
              "restarting a stalled packet; compare with sensing.ring.diag")
    print("  samples lost %d of %d expected (%.3f%%)"
          % (lost_total, expected_total,
             100.0 * lost_total / expected_total if expected_total else 0.0))

    report_sync_offsets(heads, per_block, period)

    if gaps:
        report_loss_over_time(gaps, unwrapped[-1] - unwrapped[0], period)
        report_gap_profile(boundary_gaps, mid_gaps, period,
                           true_seconds / 60.0 if true_seconds else 0.0)

    if not options["quiet"]:
        for index, step, block, position, previous, lost in gaps[:40]:
            print("      t=%8.1fs  gap %8.3f s (%d samples)  "
                  "sample %d, block %d pos %d%s"
                  % (previous / RTC, step / RTC, lost, index, block, position,
                     "  <-- block boundary" if position == 0 else ""))
        if len(gaps) > 40:
            print("      ... %d more" % (len(gaps) - 40))
        for index in zeros[:20]:
            _, block, position = samples[index]
            print("      zero timestamp at sample %d, block %d pos %d"
                  % (index, block, position))
        if len(zeros) > 20:
            print("      ... %d more" % (len(zeros) - 20))

    if not gaps and not zeros:
        print("  >>> clean - no gaps, no invalid records")
    print()

    session["files"] += 1
    session["samples"] += len(raw)
    session["lost_boundary"] += lost_at_boundary
    session["lost_mid"] += lost_mid_block
    session["zeros"] += len(zeros)
    session["seconds"] += true_seconds
    session["opened"].append(initial_ticks / RTC)


def print_session(session):
    if session["files"] < 2:
        return
    lost = session["lost_boundary"] + session["lost_mid"]
    expected = session["samples"] + lost
    print("=== session totals, %d files ===" % session["files"])
    print("  samples %d   sampling time %.2f s (%s)"
          % (session["samples"], session["seconds"],
             duration_str(session["seconds"])))
    print("  samples lost %d of %d expected (%.3f%%): %d at block boundaries, "
          "%d mid-block"
          % (lost, expected, 100.0 * lost / expected if expected else 0.0,
             session["lost_boundary"], session["lost_mid"]))
    print("  zero timestamps: %d" % session["zeros"])

    # Each file carries its own initial timestamp, and a host that restarts its
    # unwrap per file loses the offset between them.
    opened = session["opened"]
    steps = [opened[i + 1] - opened[i] for i in range(len(opened) - 1)]
    if steps and min(steps) < 0:
        print("  NOTE: a later file reports an earlier open time than the one "
              "before it, so the")
        print("        files are not in chronological order, or the clock was "
              "reset between them")
    print()


def expand_paths(paths):
    """A directory is a session directory: take its numerically-named data
    files in order, which is how the firmware writes them (000, 001, 002...).
    Mirrors what the MATLAB reference tooling did by incrementing a name."""
    out = []
    for path in paths:
        if os.path.isdir(path):
            names = [n for n in os.listdir(path)
                     if os.path.isfile(os.path.join(path, n)) and n.isdigit()]
            if not names:
                print("=== %s ===" % path)
                print("  no numerically-named data files in this directory\n")
                continue
            out.extend(os.path.join(path, n) for n in sorted(names, key=int))
        else:
            out.append(path)
    return out


def main(argv):
    options = {"row": 0, "period": 0, "sync_head": None, "quiet": False}
    files = []
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == "--row":
            i += 1
            options["row"] = int(argv[i])
        elif arg == "--period":
            i += 1
            options["period"] = int(argv[i])
        elif arg == "--sync-head":
            i += 1
            options["sync_head"] = int(argv[i])
        elif arg == "--no-sync":
            options["sync_head"] = 0
        elif arg == "--quiet":
            options["quiet"] = True
        elif arg.startswith("-"):
            sys.exit("unknown option %s" % arg)
        else:
            files.append(arg)
        i += 1

    if not files:
        print(__doc__)
        return 1

    session = {"files": 0, "samples": 0, "lost_boundary": 0, "lost_mid": 0,
               "zeros": 0, "seconds": 0.0, "opened": []}
    for path in expand_paths(files):
        audit(path, options, session)
    print_session(session)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
