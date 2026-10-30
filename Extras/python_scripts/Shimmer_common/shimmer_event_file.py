"""Read and check a Shimmer3/Shimmer3R event file (docs/SHIMMER3_EVENT_CHANNEL.md).

    python shimmer_event_file.py events/<experiment>/<session>/000
    python shimmer_event_file.py events/.../000 --data data/.../000

Prints the header and every record, then checks the file: the header's magic
and lengths, a whole number of records, known record types, sequence numbers
that run on from the header without gaps, times that never go backwards, and
a final record at the end. With --data it also checks that the event file is
the one recorded beside that data file. Exits 1 if any check fails.

Needs nothing beyond the standard library.
"""

import argparse
import struct
import sys

TICKS_PER_S = 32768
HEADER_FMT = "<4sBBBBQQIHHBBHBB"
RECORD_FMT = "<BBHQBBH"
TYPES = {1: "edge", 2: "init", 3: "hbeat", 4: "overflow"}
CHANNELS = {0: "btn", 1: "ttl", 0xFF: "-"}
FLAG_FINAL = 0x80
OPEN_REASONS = {1: "session start", 2: "split"}
HW_ID_SHIMMER3R = 10


def decode_record(blob, off=0):
    """One record, laid out as in the file and in a Bluetooth event frame."""
    rtype, flags, seq, ts, channel, state, dropped = struct.unpack_from(RECORD_FMT, blob, off)
    return dict(type=rtype, flags=flags, seq=seq, ts=ts, channel=channel, state=state,
                dropped=dropped)


def parse(blob):
    """Returns (header dict, list of record dicts). Raises ValueError if the
    file is not an event file."""
    if len(blob) < 8 or blob[0:4] != b"SEVT":
        raise ValueError("not an event file: no SEVT magic")
    header_len, record_len = blob[5], blob[6]
    if header_len < struct.calcsize(HEADER_FMT) or record_len < struct.calcsize(RECORD_FMT):
        raise ValueError(f"header length {header_len} / record length {record_len} too short")
    fields = struct.unpack_from(HEADER_FMT, blob, 0)
    header = dict(zip(
        ("magic", "version", "header_len", "record_len", "hw_id", "file_open_ts",
         "rwc_offset", "config_time", "fw_id", "fw_major", "fw_minor", "fw_internal",
         "first_seq", "open_reason", "enables"), fields))
    records = [decode_record(blob, off)
               for off in range(header_len, len(blob) - record_len + 1, record_len)]
    header["trailing_bytes"] = (len(blob) - header_len) % record_len
    return header, records


def data_file_initial_ts(blob, hw_id):
    """The data file header's initial timestamp (SD_CARD_FORMAT section 3.3):
    five bytes at 251-255, plus the top three at 44-46 on Shimmer3R."""
    ts = (blob[251] << 32) | (blob[255] << 24) | (blob[254] << 16) | (blob[253] << 8) | blob[252]
    if hw_id == HW_ID_SHIMMER3R:
        ts |= (blob[44] << 56) | (blob[45] << 48) | (blob[46] << 40)
    return ts


def check(header, records, data_blob=None):
    """Returns a list of problems, empty if the file is good."""
    problems = []
    if header["version"] != 1:
        problems.append(f"format version {header['version']}, this reader knows 1")
    if header["trailing_bytes"]:
        problems.append(f"{header['trailing_bytes']} bytes after the last whole record")
    expected_seq = header["first_seq"]
    last_ts = None
    for i, r in enumerate(records):
        if r["type"] not in TYPES:
            problems.append(f"record {i}: unknown type 0x{r['type']:02X}")
        if r["seq"] != expected_seq:
            problems.append(f"record {i}: seq {r['seq']}, expected {expected_seq}")
        expected_seq = (r["seq"] + 1) & 0xFFFF
        if last_ts is not None and r["ts"] < last_ts:
            problems.append(f"record {i}: time goes backwards")
        last_ts = r["ts"]
    if not records or not (records[-1]["flags"] & FLAG_FINAL):
        problems.append("no final record: the file was not closed cleanly")
    if not any(r["type"] == 2 for r in records):
        problems.append("no initial-state record")
    if data_blob is not None:
        if len(data_blob) < 256:
            problems.append("data file shorter than its header")
        else:
            data_ts = data_file_initial_ts(data_blob, header["hw_id"])
            event_ts = header["file_open_ts"]
            if header["hw_id"] != HW_ID_SHIMMER3R:
                event_ts &= (1 << 40) - 1
            if data_ts != event_ts:
                problems.append(f"not this data file's events: data starts at {data_ts}, "
                                f"event file at {event_ts}")
    return problems


def print_file(header, records):
    open_ts = header["file_open_ts"]
    print(f"hw {header['hw_id']}, fw {header['fw_id']} v{header['fw_major']}."
          f"{header['fw_minor']:02d}.{header['fw_internal']:03d}, "
          f"opened at {open_ts} ticks ({OPEN_REASONS.get(header['open_reason'], '?')}), "
          f"config time {header['config_time']}, enables 0x{header['enables']:02X}")
    print("  seq | type     | flags | ch  | state | t from open (s) | dropped")
    for r in records:
        t = (r["ts"] - open_ts) / TICKS_PER_S
        print(f"{r['seq']:5d} | {TYPES.get(r['type'], '?'):8s} | 0x{r['flags']:02X}  | "
              f"{CHANNELS.get(r['channel'], '?'):3s} | b{r['state']:02b}   | {t:+15.6f} | "
              f"{r['dropped']}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("event_file")
    ap.add_argument("--data", help="the data file it was recorded beside")
    args = ap.parse_args(argv)

    with open(args.event_file, "rb") as f:
        blob = f.read()
    data_blob = None
    if args.data:
        with open(args.data, "rb") as f:
            data_blob = f.read(256)

    try:
        header, records = parse(blob)
    except ValueError as e:
        print(f"{args.event_file}: {e}")
        return 1
    print_file(header, records)
    problems = check(header, records, data_blob)
    for p in problems:
        print(f"FAIL: {p}")
    if not problems:
        print(f"OK: {len(records)} records")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
