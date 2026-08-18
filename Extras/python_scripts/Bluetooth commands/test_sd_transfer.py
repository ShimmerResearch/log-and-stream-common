"""Bench test for the SD-card file transfer commands (opcodes 0xC1-0xCC,
LogAndStream_Shimmer3R >= v1.01.009).

Walks the card's data/ tree over the BT serial link, prints it, downloads the
largest file (SHA-256 + throughput), and exercises the gating/error paths.
Run with the device idle (not logging/streaming), undocked, USB unplugged.

Usage: python test_sd_transfer.py            (interactive COM-port pick)
"""

import hashlib
import sys
import time

sys.path.append("..")

from Shimmer_common import shimmer_app_common, shimmer_device


def fmt_bytes(n):
    if n < 1024:
        return "%d B" % n
    if n < 1024 * 1024:
        return "%.1f KB" % (n / 1024)
    return "%.2f MB" % (n / 1024 / 1024)


def walk(bt, path, depth=0, files=None):
    if files is None:
        files = []
    status, entries = bt.sd_list_dir(path)
    if status != 0:
        print("%s<list failed, status=%s>" % ("  " * depth, status))
        return files
    for e in entries:
        line = "  " * depth + ("[D] " if e["is_dir"] else "    ") + e["name"]
        if not e["is_dir"]:
            line += "  (%s)" % fmt_bytes(e["size"])
        print(line)
        child = path + "/" + e["name"]
        if e["is_dir"]:
            walk(bt, child, depth + 1, files)
        else:
            files.append({"path": child, "size": e["size"]})
    return files


def main():
    com_port = shimmer_app_common.get_selected_com_port(dock_ports=False)
    if not com_port:
        print("Supported COM port not found, exiting")
        return

    shimmer = shimmer_device.Shimmer3()
    if not shimmer.setup_bluetooth_com_port(com_port, debug_txrx_packets=False):
        return
    bt = shimmer.bluetooth_port

    print("\n--- Free space ---")
    t0 = time.time()
    status, free_kb, total_kb = bt.sd_free_space()
    print("status=%s free=%s total=%s (query took %.1fs)"
          % (status, fmt_bytes(free_kb * 1024), fmt_bytes(total_kb * 1024), time.time() - t0))

    print("\n--- Card tree (data/) ---")
    files = walk(bt, "data")
    total = sum(f["size"] for f in files)
    print("%d file(s), %s total" % (len(files), fmt_bytes(total)))

    print("\n--- Error paths ---")
    status, _ = bt.sd_stat("data/does_not_exist_12345")
    print("stat(non-existent): status=%s (expect FatFs no-file, 4)" % status)
    status = bt.sd_delete("sdlog.cfg")
    print("delete(sdlog.cfg): status=%s (expect bad-args 0xF2 = 242)" % status)

    if not files:
        print("\nNo data files on the card - logging a short session first would "
              "give the download test something to chew on.")
        bt.close_port()
        return

    target = max(files, key=lambda f: f["size"])
    print("\n--- Download %s (%s) ---" % (target["path"], fmt_bytes(target["size"])))
    progress = {"last": 0}

    def on_progress(done, size):
        if done - progress["last"] >= 64 * 1024 or done == size:
            progress["last"] = done
            print("  %s / %s" % (fmt_bytes(done), fmt_bytes(size)))

    t0 = time.time()
    data = bt.sd_download_file(target["path"], size=target["size"], on_progress=on_progress)
    elapsed = time.time() - t0
    if data is None:
        print("DOWNLOAD FAILED")
    else:
        print("Downloaded %s in %.1fs -> %.1f KB/s" % (fmt_bytes(len(data)), elapsed,
                                                       len(data) / 1024 / elapsed))
        print("SHA-256: %s" % hashlib.sha256(data).hexdigest())
        out_name = target["path"].replace("/", "_") + ".bin"
        with open(out_name, "wb") as f:
            f.write(data)
        print("Saved as %s - compare the hash against a direct card read of the "
              "same file." % out_name)

    bt.close_port()
    print("All done")


if __name__ == "__main__":
    main()
