#!/usr/bin/env python3
"""Active Bluetooth scan loop: 15 s scan on / 5 s off, provoking answers.

`bluetoothctl scan on` sends BR/EDR inquiry and BLE scan requests;
nearby devices answer and become visible. Emits the same bt/ble JSON
lines as monitor_scan.py (imports its sighting parser; no second
writer implementation). Empty window: no line. Ctrl-C: exit 0.
"""
import argparse
import json
import sys
import time

from monitor_scan import bt_sightings, emit


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--adapter", default="hci0")
    ap.add_argument("--scan-seconds", type=float, default=15)
    ap.add_argument("--gap-seconds", type=float, default=5)
    ap.add_argument("--out", default="monitor_hits.jsonl")
    ap.add_argument("--limit", type=int, default=0,
                    help="stop after N windows (0 = forever)")
    args = ap.parse_args()

    import threading
    out = open(args.out, "a")
    lock = threading.Lock()
    last = {}
    windows = 0
    try:
        while True:
            try:
                for rec in bt_sightings(args.adapter, args.scan_seconds):
                    if time.time() - last.get(rec["mac"], 0) < 10.0:
                        continue
                    last[rec["mac"]] = time.time()
                    emit(out, lock, rec)
            except RuntimeError as e:
                print(f"ERROR: bt unavailable ({e})", file=sys.stderr)
                return 1
            windows += 1
            if args.limit and windows >= args.limit:
                return 0
            time.sleep(args.gap_seconds)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
