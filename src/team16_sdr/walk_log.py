#!/usr/bin/env python3
"""Walk-log proximity inference: RSSI trend while the receiver moves.

No directional antenna: strongest uplink = nearest phone (free-space
path loss). The approach direction is the longest streak of strictly
increasing power along log order.

  log:    press Enter -> one 0.5 s dwell -> type position -> logged
  report: sort positions by power_db desc, label top `closest`,
          report the increasing streak.
"""
import argparse
import json
import os
import sys
import time

import phone_scan


def do_log(args):
    phy, lpc = phone_scan.open_device(args.uri, args.freq, args.samplerate,
                                      args.samplerate, args.gain)
    phy.find_channel("altvoltage0", True).attrs["frequency"].value = str(int(args.freq))
    print(f"dwelling on {args.freq/1e6:.0f} MHz; Enter to sample, Ctrl-C to stop")
    try:
        with open(args.out, "a") as out:
            while True:
                input()
                iq = phone_scan.dwell(lpc, args.samplerate, 0.5)
                res = phone_scan.analyze(iq, args.samplerate, args.margin_db)
                power = round(res[2], 2) if res else None
                pos = input("position? ").strip()
                rec = {"t": round(time.time(), 3), "pos": pos, "power_db": power}
                out.write(json.dumps(rec) + "\n")
                out.flush()
                print(f"logged: {rec}")
    except KeyboardInterrupt:
        return 0


def do_report(args):
    if not os.path.exists(args.out):
        print("no walk data")
        return 0
    rows = []
    with open(args.out) as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    if not rows:
        print("no walk data")
        return 0
    ranked = sorted(rows, key=lambda r: (r["power_db"] is not None,
                                         r["power_db"] or -1e9), reverse=True)
    print("positions by uplink power (strongest first):")
    for i, r in enumerate(ranked):
        tag = "  closest" if i == 0 else ""
        p = r["power_db"]
        print(f"  {r['pos']}: {p if p is not None else 'no signal'}{tag}")
    # longest strictly-increasing run of power_db along log order
    best_len, best_start, cur_len, cur_start = 0, 0, 1, 0
    for i in range(1, len(rows)):
        a, b = rows[i - 1]["power_db"], rows[i]["power_db"]
        if a is not None and b is not None and b > a:
            cur_len += 1
        else:
            if cur_len > best_len:
                best_len, best_start = cur_len, cur_start
            cur_len, cur_start = 1, i
    if cur_len > best_len:
        best_len, best_start = cur_len, cur_start
    if best_len >= 2:
        seq = rows[best_start:best_start + best_len]
        print(f"approach direction: {best_len}-step rising streak "
              f"'{seq[0]['pos']}' -> '{seq[-1]['pos']}' "
              f"({seq[0]['power_db']} -> {seq[-1]['power_db']} dBFS)")
    else:
        print("no rising streak (walked away or stood still)")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--role", choices=["log", "report"], default="report")
    ap.add_argument("--uri", default="usb:")
    ap.add_argument("--freq", type=int, default=2_620_000_000)
    ap.add_argument("--samplerate", type=float, default=20e6)
    ap.add_argument("--gain", type=float, default=30)
    ap.add_argument("--margin-db", type=float, default=8.0)
    ap.add_argument("--out", default="walk.jsonl")
    args = ap.parse_args()
    return do_log(args) if args.role == "log" else do_report(args)


if __name__ == "__main__":
    sys.exit(main())
