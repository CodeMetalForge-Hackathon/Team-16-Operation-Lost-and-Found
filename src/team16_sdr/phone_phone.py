#!/usr/bin/env python3
"""Correlator: merge WiFi/BT monitor sightings with cellular rf hits.

Reads monitor_hits.jsonl (monitor_scan.py) and rf_hits.jsonl
(phone_scan.py); appends phone-contact JSON lines to --out:
  {"t", "kind", "mac", "name", "rssi", "count", "adapters",
   "hz", "power_db", "pose": null}
Unused fields are null: cell contacts have no mac/rssi; wifi/bt/ble
contacts have no hz/power_db. `pose` stays null (no fused-pose source).

Contact rules inside --window seconds:
  wifi/bt/ble mac: >=1 wifi-probe/wifi-assoc whose ssid contains the
  bait SSID, OR >= --min-hits sightings.
  cell: rf_hit whose hz falls in a configured uplink range (UPLINK_HZ)
  -> kind "cell", mac null.

Tail mode polls both files every 1 s (byte offsets remembered) and
emits a contact when its aggregate changes; --once reads each file
once and exits; --report prints --out strongest-signal-first and
labels the top one `closest` (walk_log.py convention).
"""
import argparse
import json
import os
import sys
import threading
import time

from monitor_scan import emit

BAIT_SSID = "Low Quality Bait"
# B12, B5, B4, B2, B7 uplinks; matches phone_scan.py --channels defaults.
UPLINK_HZ = [(707e6, 716e6), (837e6, 846e6), (1730e6, 1755e6),
             (1850e6, 1915e6), (2535e6, 2585e6)]


def read_new(path, offset):
    """(records, new_offset): complete JSONL lines appended after offset."""
    if not os.path.exists(path):
        return [], offset
    with open(path, "rb") as f:
        f.seek(offset)
        data = f.read()
    if not data:
        return [], offset
    if not data.endswith(b"\n"):
        cut = data.rfind(b"\n")
        if cut == -1:
            return [], offset  # partial line: re-read next poll
        data = data[:cut + 1]
    recs = []
    for line in data.decode("utf-8", "replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            recs.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return recs, offset + len(data)


def is_uplink(hz):
    return any(lo <= hz <= hi for lo, hi in UPLINK_HZ)


def wifi_contacts(records, window, min_hits, ref_t):
    """mac -> contact dict for monitor records inside the window."""
    by_mac = {}
    for r in records:
        kind = r.get("kind")
        if kind not in ("wifi-probe", "wifi-assoc", "bt", "ble"):
            continue
        t = r.get("t", 0)
        if ref_t - t > window:
            continue
        mac = r.get("mac")
        if not mac:
            continue
        d = by_mac.setdefault(mac, {"kind": kind, "name": None, "rssi": None,
                                    "count": 0, "adapters": set(), "t": t,
                                    "bait": False})
        d["count"] += 1
        if t >= d["t"]:
            d["t"] = t
            d["kind"] = kind
        if r.get("name"):
            d["name"] = r["name"]
        if r.get("rssi") is not None:
            d["rssi"] = r["rssi"] if d["rssi"] is None else max(d["rssi"],
                                                                r["rssi"])
        if r.get("adapter"):
            d["adapters"].add(r["adapter"])
        if kind in ("wifi-probe", "wifi-assoc") and \
                BAIT_SSID in (r.get("ssid") or ""):
            d["bait"] = True
    contacts = {}
    for mac, d in by_mac.items():
        if not (d["bait"] or d["count"] >= min_hits):
            continue
        contacts[mac] = {"t": d["t"], "kind": d["kind"], "mac": mac,
                         "name": d["name"], "rssi": d["rssi"],
                         "count": d["count"], "adapters": sorted(d["adapters"]),
                         "hz": None, "power_db": None, "pose": None}
    return contacts


def cell_contacts(records):
    """rf_hit records in an uplink range -> cell contact dicts."""
    out = []
    for r in records:
        hz = r.get("hz")
        if hz is None or not is_uplink(hz):
            continue
        out.append({"t": r.get("t"), "kind": "cell", "mac": None, "name": None,
                    "rssi": None, "count": 1, "adapters": [], "hz": hz,
                    "power_db": r.get("power_db"), "pose": None})
    return out


def once(args):
    mon, _ = read_new(args.monitor, 0)
    rf, _ = read_new(args.rf, 0)
    stamps = [r.get("t", 0) for r in mon + rf]
    if not stamps:
        return 0
    ref = max(stamps)  # window relative to newest record, not wall clock
    out = open(args.out, "a")
    lock = threading.Lock()
    for rec in wifi_contacts(mon, args.window, args.min_hits, ref).values():
        emit(out, lock, rec)
    rf_win = [r for r in rf if ref - r.get("t", 0) <= args.window]
    for rec in cell_contacts(rf_win):
        emit(out, lock, rec)
    return 0


def tail(args):
    off_m = off_r = 0
    mon_buf, rf_buf = [], []
    emitted = {}  # mac -> (count, rssi) signature at last emit
    out = open(args.out, "a")
    lock = threading.Lock()
    try:
        while True:
            new_m, off_m = read_new(args.monitor, off_m)
            new_r, off_r = read_new(args.rf, off_r)
            mon_buf.extend(new_m)
            rf_buf.extend(new_r)
            ref = time.time()
            mon_buf[:] = [r for r in mon_buf if ref - r.get("t", 0) <= args.window]
            rf_buf[:] = [r for r in rf_buf if ref - r.get("t", 0) <= args.window]
            for rec in wifi_contacts(mon_buf, args.window,
                                     args.min_hits, ref).values():
                sig = (rec["count"], rec["rssi"])
                if emitted.get(rec["mac"]) != sig:
                    emitted[rec["mac"]] = sig
                    emit(out, lock, rec)
            for rec in cell_contacts(new_r):  # each rf line emitted once
                emit(out, lock, rec)
            time.sleep(1.0)
    except KeyboardInterrupt:
        return 0


def report(args):
    rows = []
    if os.path.exists(args.out):
        with open(args.out) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    if not rows:
        print("no phone contacts")
        return 0
    best = {}   # mac -> row with best rssi (wifi/bt/ble)
    cells = {}  # hz -> row with best power_db
    for r in rows:
        if r.get("kind") == "cell":
            cur = cells.get(r.get("hz"))
            if cur is None or (r.get("power_db") or -1e9) > \
                    (cur.get("power_db") or -1e9):
                cells[r.get("hz")] = r
            continue
        mac = r.get("mac")
        cur = best.get(mac)
        if cur is None or (r.get("rssi") or -1e9) > (cur.get("rssi") or -1e9):
            best[mac] = r
    contacts = list(best.values()) + list(cells.values())

    def strength(r):
        if r.get("rssi") is not None:
            return r["rssi"]
        return r["power_db"] if r.get("power_db") is not None else -1e9

    ranked = sorted(contacts, key=strength, reverse=True)
    print("contacts by signal (strongest first):")
    for i, r in enumerate(ranked):
        tag = "  closest" if i == 0 else ""
        if r.get("kind") == "cell":
            label = f"cell {r['hz'] / 1e6:.1f} MHz"
            v, unit = r.get("power_db"), "dBFS"
        else:
            label = r.get("name") or r.get("mac")
            v, unit = r.get("rssi"), "dBm"
        sig = f"{v} {unit}" if v is not None else "no signal"
        print(f"  {label}: {sig} ({r.get('kind')}, x{r.get('count')}){tag}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--monitor", default="monitor_hits.jsonl")
    ap.add_argument("--rf", default="rf_hits.jsonl")
    ap.add_argument("--out", default="phone_contacts.jsonl")
    ap.add_argument("--window", type=float, default=60)
    ap.add_argument("--min-hits", type=int, default=2)
    ap.add_argument("--once", action="store_true",
                    help="read each input file once and exit")
    ap.add_argument("--report", action="store_true",
                    help="print --out ranked strongest-first, exit")
    args = ap.parse_args()
    if args.report:
        return report(args)
    if args.once:
        return once(args)
    return tail(args)


if __name__ == "__main__":
    sys.exit(main())
