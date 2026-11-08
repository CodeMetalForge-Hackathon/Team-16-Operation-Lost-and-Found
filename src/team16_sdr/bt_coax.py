#!/usr/bin/env python3
"""Pi onboard Bluetooth as the coaxer (TX): inquiry/scan bursts.

Loop: `discoverable on` + scan for --scan-seconds, then scan off +
`discoverable off` for --gap-seconds. The discoverability toggle makes
phones in a locked state answer a BR/EDR inquiry; the scan half sends
BLE scan requests. Sighting parsing is reused from monitor_scan
(bt_sightings) — no second parser.

Emits one summary line per window:
  {"t":…, "kind": "coax-window", "adapter": "hci0", "sightings": N}
so the demo can show "we poked, N answered". Individual bt/ble sighting
lines come from monitor_scan.py's feeds, not from here.
"""
import argparse
import subprocess
import sys
import threading
import time

from monitor_scan import bt_sightings, emit, resolve_controller


def set_discoverable(adapter, on):
    """Toggle discoverability on one controller via bluetoothctl."""
    ctrl = resolve_controller(adapter)
    if ctrl is None or ctrl == "00:00:00:00:00:00":
        raise RuntimeError(f"{adapter}: no controller")
    try:
        proc = subprocess.Popen(["bluetoothctl"],
                                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, text=True)
    except FileNotFoundError:
        raise RuntimeError("bluetoothctl missing (sudo apt-get install -y bluez)")
    proc.stdin.write(f"select {ctrl}\ndiscoverable {'on' if on else 'off'}\nexit\n")
    proc.stdin.flush()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--adapter", default="hci0",
                    help="TX/coaxing adapter (the Alfa hci1 stays passive)")
    ap.add_argument("--scan-seconds", type=float, default=15)
    ap.add_argument("--gap-seconds", type=float, default=5)
    ap.add_argument("--out", default="monitor_hits.jsonl")
    ap.add_argument("--limit", type=int, default=0,
                    help="stop after N windows (0 = forever)")
    args = ap.parse_args()

    out = open(args.out, "a")
    lock = threading.Lock()
    windows = 0
    try:
        while True:
            set_discoverable(args.adapter, True)
            n = 0
            try:
                for _rec in bt_sightings(args.adapter, args.scan_seconds):
                    n += 1
            finally:
                set_discoverable(args.adapter, False)
            emit(out, lock, {"t": round(time.time(), 3), "kind": "coax-window",
                             "adapter": args.adapter, "sightings": n})
            windows += 1
            if args.limit and windows >= args.limit:
                return 0
            time.sleep(args.gap_seconds)
    except KeyboardInterrupt:
        return 0
    except RuntimeError as e:
        print(f"ERROR: bt unavailable ({e})", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
