"""Bench test for the event channel's Bluetooth stream
(docs/SHIMMER3_BT_COMMUNICATION_PROTOCOL.md section 7.15).

Opts in with SET_FEATURE / FEATURE_EVENT_STREAM, then prints each event record
the device sends while you press the button or pulse the TTL input. It checks
every frame's CRC, the link's sequence numbers, its first record and the
status counters, then opts out and checks that nothing more arrives.

    python test_event_stream.py [COMx] [--seconds 60] [--stream] [--csv run.csv]
                                [--reconnect-check]
    python test_event_stream.py --compare run.csv <event file>

--stream           Also streams sensor data, at whatever rate and with whatever
                   sensors the device is configured for, to see how late frames
                   run behind a busy transmit ring.
--csv              Saves the records received, for --compare.
--reconnect-check  Afterwards, drops the link, reconnects without opting in,
                   and checks that no frame arrives and that the TTL input is
                   not left armed for a host.
--compare          Offline: matches a saved run against the event file the
                   device wrote meanwhile. Copy it off the card, or fetch it
                   with test_sd_transfer.py on Shimmer3R. Every record received
                   while logging should be in the file at exactly the same time.

Enable the inputs first: configuration byte 218, bit 1 for button markers and
bit 5 for the TTL input. Opting in enables neither. Exits 1 if a check fails.

"Late" is how long a frame took to arrive, measured against the quickest frame
of the run, since the device's clock and the PC's share no zero.
"""

import argparse
import csv
import statistics
import sys
import time

sys.path.append("..")

from Shimmer_common import shimmer_crc, shimmer_device, shimmer_event_file
from Shimmer_common.shimmer_comms_bluetooth import BtCmds, BtFeatures

NACK = 0xFE
FRAME_HEAD = bytes([BtCmds.INSTREAM_CMD_RESPONSE, BtCmds.EVENT_RECORD_RESPONSE])
RECORD_LEN = 16
# A longer record from later firmware is allowed; anything past this is not a frame
MAX_RECORD_LEN = 64
TYPE_EDGE = 1
TYPE_INITIAL_STATE = 2
TYPE_OVERFLOW = 4
STATUS_LEN = 9
STATUS_FLAGS = ("stream on", "file open", "file error", "TTL armed", "TTL throttled")
CSV_FIELDS = ("arrived_s", "seq", "type", "flags", "ts", "channel", "state", "dropped")
TICKS_PER_S = shimmer_event_file.TICKS_PER_S


class FrameScanner:
    """Finds event frames in whatever else the link carries: ACKs, replies,
    status pushes and, with --stream, data packets. A frame is 0x8A 0xBB, a
    record length, the record, and a CRC-16 over everything before it. A
    candidate whose CRC fails is passed over a byte at a time, since a data
    packet can hold 0x8A 0xBB by chance. Everything that is not a frame is kept
    in `other`, where replies are looked for."""

    def __init__(self):
        self.buf = bytearray()
        self.other = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        self.buf += data
        records = []
        while True:
            i = self.buf.find(FRAME_HEAD)
            if i < 0:
                # A trailing 0x8A may be the start of the next frame
                keep = 1 if self.buf.endswith(FRAME_HEAD[:1]) else 0
                self._skip(len(self.buf) - keep)
                return records
            self._skip(i)
            if len(self.buf) < 3:
                return records
            frame_len = 3 + self.buf[2] + 2
            if not RECORD_LEN <= self.buf[2] <= MAX_RECORD_LEN:
                self._skip(1)
                continue
            if len(self.buf) < frame_len:
                return records
            frame = self.buf[:frame_len]
            crc = frame[-2] | (frame[-1] << 8)
            if shimmer_crc.calc_crc(frame_len - 2, frame) != crc:
                self.bad_crc += 1
                self._skip(1)
                continue
            records.append(shimmer_event_file.decode_record(bytes(frame), 3))
            del self.buf[:frame_len]

    def _skip(self, n):
        self.other += self.buf[:n]
        del self.buf[:n]

    def seen(self):
        """Everything but frames, including a trailing byte held back in case a
        frame starts there"""
        return self.other + self.buf


class Link:
    """What the frames say about the link: its first record, gaps in its
    sequence, and how late each frame arrived."""

    def __init__(self):
        self.received = []  # (seconds after the opt-in, record)
        self.lost = 0
        self.problems = []
        self.warnings = []

    def add(self, arrived_s, rec):
        if not self.received:
            if rec["type"] != TYPE_INITIAL_STATE or rec["seq"] != 0:
                self.problems.append(f"the first frame is {type_name(rec)} seq {rec['seq']}, "
                                     "not the initial state at seq 0")
        else:
            prev = self.received[-1][1]
            expected = (prev["seq"] + 1) & 0xFFFF
            if rec["seq"] != expected:
                lost = (rec["seq"] - expected) & 0xFFFF
                self.lost += lost
                self.warnings.append(f"seq {expected} to {(rec['seq'] - 1) & 0xFFFF}: "
                                     f"{lost} record(s) lost by the link")
            if rec["ts"] < prev["ts"]:
                self.warnings.append(f"seq {rec['seq']}: time goes backwards")
        self.received.append((arrived_s, rec))

    def lateness_ms(self):
        offsets = [arrived - rec["ts"] / TICKS_PER_S for arrived, rec in self.received]
        if not offsets:
            return []
        quickest = min(offsets)
        return [(o - quickest) * 1000 for o in offsets]


def type_name(rec):
    return shimmer_event_file.TYPES.get(rec["type"], f"type 0x{rec['type']:02X}")


def print_header():
    print("  seq | type     | flags | ch  | state | t from opt-in (s) | dropped | arrived (s)")


def print_record(rec, t0_ticks, arrived_s):
    t = (rec["ts"] - t0_ticks) / TICKS_PER_S
    print(f"{rec['seq']:5d} | {type_name(rec):8s} | 0x{rec['flags']:02X}  | "
          f"{shimmer_event_file.CHANNELS.get(rec['channel'], '?'):3s} | b{rec['state']:02b}   | "
          f"{t:+17.6f} | {rec['dropped']:7d} | {arrived_s:11.3f}")


def read_for(bt, scanner, seconds, on_record=None, until=None):
    """Reads the link for up to `seconds`, handing each frame's record to
    on_record. Stops early once until() is true, or on Ctrl+C."""
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            waiting = bt.ser.inWaiting()
            data = bt.ser.read(waiting if waiting else 1)
            if data:
                for rec in scanner.feed(data):
                    if on_record:
                        on_record(rec)
            if until and until():
                return True
    except KeyboardInterrupt:
        print("(stopped early)")
    return False


def command(bt, scanner, cmd, on_record=None, timeout_s=1.5):
    """Sends a command on an idle link and returns the bytes that were not part
    of a frame, from the reply on. Frames that arrive meanwhile still go to
    on_record."""
    mark = len(scanner.other)
    bt.send_bluetooth(cmd)

    def replied():
        rest = scanner.other[mark:]
        return BtCmds.ACK_COMMAND_PROCESSED in rest or NACK in rest

    read_for(bt, scanner, timeout_s, on_record, replied)
    rest = bytes(scanner.other[mark:])
    for i, b in enumerate(rest):
        if b in (BtCmds.ACK_COMMAND_PROCESSED, NACK):
            return rest[i:]
    return b""


def query_status(bt, scanner, on_record=None):
    """GET_EVENT_CHANNEL_STATUS_COMMAND, decoded, or None without an answer."""
    mark = len(scanner.other)
    bt.send_bluetooth([BtCmds.GET_EVENT_CHANNEL_STATUS_COMMAND])
    head = bytes([BtCmds.ACK_COMMAND_PROCESSED, BtCmds.EVENT_CHANNEL_STATUS_RESPONSE])

    def answered():
        seen = scanner.seen()
        i = seen.find(head, mark)
        return i >= 0 and len(seen) >= i + 2 + STATUS_LEN

    if not read_for(bt, scanner, 1.5, on_record, answered):
        return None
    seen = scanner.seen()
    i = seen.find(head, mark) + 2
    b = seen[i:i + STATUS_LEN]
    return dict(enables=b[0], flags=b[1], state=b[2], next_seq=b[3] | (b[4] << 8),
                dropped=b[5] | (b[6] << 8), frames_lost=b[7] | (b[8] << 8))


def flag(status, name):
    return bool(status["flags"] & (1 << STATUS_FLAGS.index(name)))


def print_status(label, status):
    if status is None:
        print(f"{label}: no answer to GET_EVENT_CHANNEL_STATUS_COMMAND")
        return
    flags = [name for name in STATUS_FLAGS if flag(status, name)]
    print(f"{label}: inputs enabled: button markers "
          f"{'on' if status['enables'] & 1 else 'off'}, TTL {'on' if status['enables'] & 2 else 'off'}; "
          f"{', '.join(flags) if flags else 'nothing on'}; "
          f"button {'pressed' if status['state'] & 1 else 'released'}, "
          f"TTL {'high' if status['state'] & 2 else 'low'}; next seq {status['next_seq']}, "
          f"dropped this session {status['dropped']}, frames lost {status['frames_lost']}")


def reset_crc(bt):
    """The session CRC can outlive a link the device never saw drop
    (protocol section 3.3); with it on, every reply carries CRC bytes"""
    bt.send_bluetooth([BtCmds.SET_CRC_COMMAND, 0])
    time.sleep(0.3)
    bt.clear_serial_buffer()


def pick_port():
    options = shimmer_device.serial_ports_bluetooth()
    if not options:
        return ""
    for i, item in enumerate(options):
        print(f"{i + 1}) {item.name}, {item.description}, {item.hwid}")
    choice = ""
    while not choice.isdigit() or not 1 <= int(choice) <= len(options):
        choice = input("Your choice: ")
    return options[int(choice) - 1].name


def check_ttl_not_left_armed(status, when, problems):
    if status is not None and flag(status, "TTL armed") and not flag(status, "file open"):
        problems.append(f"{when}: the TTL input is armed with no host opted in and no file open")


def run(args):
    port = args.port or pick_port()
    if not port:
        print("No Bluetooth COM port found")
        return 1
    shimmer = shimmer_device.Shimmer3()
    if not shimmer.setup_bluetooth_com_port(port):
        return 1
    bt = shimmer.bluetooth_port
    scanner = FrameScanner()
    link = Link()
    problems = []
    t_opt_in = None
    t0_ticks = None

    def on_record(rec):
        nonlocal t0_ticks
        arrived = time.monotonic() - t_opt_in
        if t0_ticks is None:
            t0_ticks = rec["ts"]
        link.add(arrived, rec)
        print_record(rec, t0_ticks, arrived)

    reset_crc(bt)
    status = query_status(bt, scanner)
    print_status("Before", status)
    if status is None:
        print("This firmware has no event channel, or the link is not up.")
        bt.close_port()
        return 1
    if not status["enables"] & 3:
        print("WARNING: neither input is enabled (configuration byte 218 bits 1 and 5), "
              "so only the initial state will arrive.")
    if flag(status, "stream on"):
        # Opting in again would change nothing, so no initial state would come
        problems.append("the stream was on before opting in: the reset on connect did not happen")
        command(bt, scanner, [BtCmds.SET_FEATURE, BtFeatures.EVENT_STREAM, 0])
        read_for(bt, scanner, 0.5)

    t_opt_in = time.monotonic()
    print_header()
    reply = command(bt, scanner, [BtCmds.SET_FEATURE, BtFeatures.EVENT_STREAM, 1], on_record)
    if not reply or reply[0] != BtCmds.ACK_COMMAND_PROCESSED:
        print("SET_FEATURE was " + ("NACKed: no event stream in this firmware" if reply
                                    else "not answered"))
        bt.close_port()
        return 1

    if args.stream:
        bt.send_bluetooth([BtCmds.START_STREAMING_COMMAND])
    print(f"Opted in. Listening for {args.seconds} s: press the button and pulse the TTL "
          "input. Ctrl+C stops early.")
    read_for(bt, scanner, args.seconds, on_record)
    if args.stream:
        bt.send_bluetooth([BtCmds.STOP_STREAMING_COMMAND])
    print("Hands off the button and the TTL input for the checks...")
    read_for(bt, scanner, 1.5, on_record)

    # The counters should agree with the frames, before opting out resets them
    status = query_status(bt, scanner, on_record)
    print_status("Opted in", status)
    if status is None:
        problems.append("no status while opted in (a frame may have crossed the reply)")
    else:
        if not flag(status, "stream on"):
            problems.append("status says the stream is off while opted in")
        if link.received:
            last_seq = link.received[-1][1]["seq"]
            if status["next_seq"] != (last_seq + 1) & 0xFFFF:
                problems.append(f"status next seq {status['next_seq']}, but the last frame "
                                f"was seq {last_seq}")
        if status["frames_lost"] != link.lost:
            problems.append(f"status counts {status['frames_lost']} frames lost, the sequence "
                            f"numbers show {link.lost}")

    count = len(link.received)
    reply = command(bt, scanner, [BtCmds.SET_FEATURE, BtFeatures.EVENT_STREAM, 0], on_record)
    if not reply or reply[0] != BtCmds.ACK_COMMAND_PROCESSED:
        problems.append("opting out was not ACKed")
    read_for(bt, scanner, 1.5, on_record)
    status = query_status(bt, scanner, on_record)
    print_status("Opted out", status)
    if len(link.received) != count:
        problems.append(f"{len(link.received) - count} frame(s) arrived after opting out")
    if status is not None and flag(status, "stream on"):
        problems.append("status says the stream is still on after opting out")
    check_ttl_not_left_armed(status, "after opting out", problems)

    if args.reconnect_check:
        # Dropped while opted in, which is the case that matters
        print("\nOpting in again, then dropping the link and reconnecting without opting in...")
        again = []
        command(bt, scanner, [BtCmds.SET_FEATURE, BtFeatures.EVENT_STREAM, 1], again.append)
        read_for(bt, scanner, 0.5, again.append)
        if not again or again[0]["type"] != TYPE_INITIAL_STATE or again[0]["seq"] != 0:
            problems.append("opting in again did not start a fresh link with the initial state")
        bt.close_port()
        time.sleep(2)
        if not shimmer.setup_bluetooth_com_port(port):
            problems.append("could not reconnect")
        else:
            reset_crc(bt)
            stray = []
            status = query_status(bt, scanner, stray.append)
            print_status("Reconnected", status)
            if status is not None and flag(status, "stream on"):
                problems.append("the stream is on after reconnecting")
            check_ttl_not_left_armed(status, "after reconnecting", problems)
            print(f"Listening for {args.reconnect_seconds} s: press the button and pulse the "
                  "TTL input. Nothing should arrive.")
            read_for(bt, scanner, args.reconnect_seconds, stray.append)
            if stray:
                problems.append(f"{len(stray)} frame(s) arrived without opting in")
    bt.close_port()

    print(f"\n{len(link.received)} frame(s) received, {link.lost} lost by the link, "
          f"{scanner.bad_crc} candidate(s) with a bad CRC")
    if scanner.bad_crc and not args.stream:
        problems.append(f"{scanner.bad_crc} frame(s) with a bad CRC on an idle link")
    late = sorted(link.lateness_ms())
    if len(late) > 1:
        print(f"Late: median {statistics.median(late):.1f} ms, "
              f"95th percentile {late[int(0.95 * (len(late) - 1))]:.1f} ms, max {late[-1]:.1f} ms")
    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(CSV_FIELDS)
            for arrived, rec in link.received:
                w.writerow([f"{arrived:.6f}"] + [rec[k] for k in CSV_FIELDS[1:]])
        print(f"Saved to {args.csv}")
    for w in link.warnings:
        print(f"WARNING: {w}")
    problems = link.problems + problems
    for p in problems:
        print(f"FAIL: {p}")
    if not problems:
        print("OK")
    return 1 if problems else 0


def compare(csv_path, event_path):
    def key(r):
        return r["type"], r["flags"], r["ts"], r["channel"], r["state"]

    with open(csv_path, newline="") as f:
        received = [{k: int(row[k]) for k in CSV_FIELDS[1:]} for row in csv.DictReader(f)]
    with open(event_path, "rb") as f:
        header, records = shimmer_event_file.parse(f.read())
    edges = [r for r in received if r["type"] in (TYPE_EDGE, TYPE_OVERFLOW)]
    file_edges = [r for r in records if r["type"] in (TYPE_EDGE, TYPE_OVERFLOW)]
    if not edges or not records:
        print("Nothing to compare: no edges received, or an empty event file")
        return 1

    # Received while the file was recording: each should be in it, unchanged
    first, last = records[0]["ts"], records[-1]["ts"]
    in_file = {key(r) for r in records}
    during = [r for r in edges if first <= r["ts"] <= last]
    missing = [r for r in during if key(r) not in in_file]
    print(f"{len(edges)} edge or overflow record(s) received, {len(during)} while the file "
          f"was recording; {len(during) - len(missing)} of those are in it at exactly the "
          "same time")
    for r in missing:
        print(f"FAIL: seq {r['seq']} ({type_name(r)} at {r['ts']} ticks) is not in the file")

    # Written while the link was up but never received: lost by the link
    span = (received[0]["ts"], received[-1]["ts"])
    got = {key(r) for r in edges}
    not_received = [r for r in file_edges if span[0] <= r["ts"] <= span[1] and key(r) not in got]
    print(f"{len(not_received)} of the file's records from while the link was up never "
          "arrived; the link's sequence gaps should account for them")
    return 1 if missing else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port", nargs="?", help="the Bluetooth COM port; asks if left out")
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--stream", action="store_true")
    ap.add_argument("--csv")
    ap.add_argument("--reconnect-check", action="store_true")
    ap.add_argument("--reconnect-seconds", type=float, default=15)
    ap.add_argument("--compare", nargs=2, metavar=("CSV", "EVENT_FILE"))
    args = ap.parse_args(argv)
    if args.compare:
        return compare(*args.compare)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
