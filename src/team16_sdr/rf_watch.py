#!/usr/bin/env python3
"""rfhits viewer: watch all four hub ports from a laptop, one line/hit.

Connects to the rfhits hub (rf_hub.py on the Pi) on all four category
ports at once and prints each hit as it arrives:

  00:21:07  ble   6e:4c:d8:ae:57:2b  rssi=0    name=-
  00:21:07  wifi  probe 1e:5b:92:9b:bc:75  rssi=-39  ssid=Low Quality Bait
  00:21:09  cell  1732561035 Hz  power_db=-67.87

Streams are live only (no history): you see hits from connect onward.
Reconnects automatically if the hub restarts. Ctrl-C exits.

  python rf_watch.py                     # default host 192.168.8.163
  python rf_watch.py --host 127.0.0.1    # e.g. hub on this machine
"""
import argparse
import json
import socket
import sys
import threading
import time

PORTS = [(8091, "bt"), (8092, "ble"), (8093, "cell"), (8094, "wifi")]
print_lock = threading.Lock()


def describe(rec):
    """One-line summary of a hit, per category."""
    cat = rec.get("cat")
    if cat == "cell":
        return f"{rec.get('hz', 0):.0f} Hz  power_db={rec.get('power_db')}"
    kind = rec.get("kind", "")
    bits = [kind.replace("wifi-", "") if cat == "wifi" else "",
            rec.get("mac") or "-",
            f"rssi={rec.get('rssi')}",
            f"name={rec.get('name') or '-'}"]
    if rec.get("ssid"):
        bits.append(f"ssid={rec['ssid']}")
    return "  ".join(b for b in bits if b)


def watch(host, port, cat, stop):
    """Reader thread: keep one port connected, print hits."""
    while not stop.is_set():
        try:
            with socket.create_connection((host, port), timeout=5) as s:
                s.settimeout(None)  # connect timeout only; recv blocks idle
                buf = b""
                while not stop.is_set():
                    chunk = s.recv(4096)
                    if not chunk:
                        break  # hub closed us
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if not line.strip():
                            continue
                        try:
                            rec = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        stamp = time.strftime("%H:%M:%S",
                                              time.localtime(rec.get("t", 0)))
                        with print_lock:
                            print(f"{stamp}  {cat:<4}  {describe(rec)}",
                                  flush=True)
        except (ConnectionRefusedError, TimeoutError, OSError) as e:
            if not stop.is_set():
                with print_lock:
                    print(f"[{cat}] {host}:{port} down ({e}); retry 3 s",
                          flush=True)
                time.sleep(3)


def main():
    ap = argparse.ArgumentParser(description="Watch rfhits hub ports live.")
    ap.add_argument("--host", default="192.168.8.163")
    ap.add_argument("--cats", default="bt,ble,cell,wifi",
                    help="comma list of categories to watch "
                         "(bt, ble, cell, wifi)")
    args = ap.parse_args()
    wanted = [c.strip() for c in args.cats.split(",") if c.strip()]
    bad = [c for c in wanted if c not in {c for _, c in PORTS}]
    if bad:
        ap.error(f"unknown categor{'y' if len(bad) == 1 else 'ies'}: "
                 f"{', '.join(bad)} (choose from bt, ble, cell, wifi)")
    ports = [(p, c) for p, c in PORTS if c in wanted]
    stop = threading.Event()
    threads = [threading.Thread(target=watch,
                                args=(args.host, port, cat, stop),
                                daemon=True)
               for port, cat in ports]
    for t in threads:
        t.start()
    print(f"watching {args.host}: " + ", ".join(f"{p}={c}" for p, c in ports)
          + "  (Ctrl-C to quit)", flush=True)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop.set()
    return 0


if __name__ == "__main__":
    sys.exit(main())
