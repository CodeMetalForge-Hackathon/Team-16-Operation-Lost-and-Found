#!/usr/bin/env python3
"""SDR / phone-detect feed for the dashboard (stdlib only, read-only: it never opens the Pluto or any radio).

The rf hub (src/team16_sdr/rf_hub.py) streams hits as JSON lines on localhost-only TCP ports:
    8091 bt (Bluetooth classic)   8092 ble   8093 cell (Pluto uplink energy)   8094 wifi
A browser cannot open raw TCP and those ports are bound to 127.0.0.1, so this small bridge runs on the Pi,
reads the four streams, keeps the recent history in memory and serves it as JSON with CORS on :8085.

  GET /sdr     -> state for the dashboard card
  GET /health  -> {"ok": true}

Options: --port 8085, --hub-host 127.0.0.1, --dir ~/rfhits (preload the last few minutes from the hub's
category files so a freshly opened dashboard is not empty), --demo (simulated hits, no hub needed).
"""
import argparse
import json
import os
import random
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CATS = {"bt": (8091, "bt_classic.jsonl"), "ble": (8092, "ble.jsonl"),
        "cell": (8093, "cellular.jsonl"), "wifi": (8094, "wifi.jsonl")}
HIST_SECS = 300
ACTIVE_SECS = 300
CLOSE_SECS = 30
BANDS = [(2535e6, "B7"), (1730e6, "B4"), (707e6, "B12")]

lock = threading.Lock()
hub_up = {c: False for c in CATS}
events = deque(maxlen=400)           # recent hits of every category, newest last
devices = {}                         # (cat, id) -> dict
cell_hist = deque(maxlen=4000)       # (t, hz, power_db)
cell_chan = {}                       # hz -> dict
counts = {c: 0 for c in CATS}
recent = {c: deque(maxlen=5000) for c in CATS}   # arrival times, for hits per minute


def band_of(hz):
    for centre, name in BANDS:
        if abs(hz - centre) < 25e6:
            return name
    return None


def ingest(rec, cat, t=None):
    now = t if t is not None else time.time()
    with lock:
        counts[cat] += 1
        recent[cat].append(now)
        ev = {"t": now, "cat": cat}
        if cat == "cell":
            hz, p = rec.get("hz"), rec.get("power_db")
            if hz is None or p is None:
                return
            ev.update(hz=hz, power_db=p)
            cell_hist.append((now, hz, p))
            ch = cell_chan.setdefault(hz, {"hz": hz, "hits": 0, "best": None})
            ch.update(last=now, power_db=p, hits=ch["hits"] + 1)
            ch["best"] = p if ch["best"] is None else max(ch["best"], p)
        else:
            mac = rec.get("mac") or rec.get("addr") or "unknown"
            rssi = rec.get("rssi")
            if rssi in (0, None):
                rssi = None  # hci0 reports 0 for "unknown"
            name = rec.get("name") or rec.get("ssid") or None
            ev.update(mac=mac, rssi=rssi, name=name, kind=rec.get("kind"))
            d = devices.setdefault((cat, mac), {"cat": cat, "mac": mac, "first": now, "hits": 0,
                                                "rssi": None, "best": None, "name": None, "kind": None})
            d["last"] = now
            d["hits"] += 1
            d["kind"] = rec.get("kind") or d["kind"]
            if name:
                d["name"] = name
            if rssi is not None:
                d["rssi"] = rssi
                d["best"] = rssi if d["best"] is None else max(d["best"], rssi)
        events.append(ev)


def reader(cat, host):
    port = CATS[cat][0]
    while True:
        try:
            with socket.create_connection((host, port), timeout=5) as s:
                s.settimeout(None)
                hub_up[cat] = True
                buf = b""
                while True:
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try:
                            ingest(json.loads(line), cat)
                        except ValueError:
                            pass
        except OSError:
            pass
        hub_up[cat] = False
        time.sleep(2)


def preload(directory):
    """Fill the last few minutes from the hub's files so the card is not empty on first load."""
    cutoff = time.time() - HIST_SECS
    for cat, (_, fname) in CATS.items():
        path = os.path.join(directory, fname)
        try:
            with open(path, "rb") as f:
                f.seek(0, os.SEEK_END)
                f.seek(max(0, f.tell() - 200_000))
                lines = f.read().splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            t = rec.get("t_ns", 0) / 1e9 or rec.get("t") or 0
            if t >= cutoff:
                ingest(rec, cat, t)


def demo():
    rnd = random.Random(3)
    macs = ["a4:83:e7:10:2b:%02x" % i for i in range(5)]
    names = ["Pixel 8", "iPhone", None, "Low Quality Bait", "Galaxy S23"]
    for c in hub_up:
        hub_up[c] = True
    while True:
        time.sleep(rnd.uniform(0.3, 1.2))
        k = rnd.choice(["wifi", "wifi", "ble", "bt", "cell"])
        if k == "cell":
            hz = rnd.choice([2535e6, 1730e6, 707e6])
            ingest({"hz": hz, "power_db": round(-92 + rnd.gauss(0, 3) + (8 if hz == 1730e6 else 0), 1)}, "cell")
        else:
            i = rnd.randrange(5)
            ingest({"mac": macs[i], "name": names[i], "rssi": rnd.randint(-85, -45), "kind": "wifi-probe" if k == "wifi" else k}, k)


def snapshot():
    now = time.time()
    with lock:
        act = [d for d in devices.values() if now - d["last"] <= ACTIVE_SECS]
        act.sort(key=lambda d: -d["last"])
        out_dev = [{"cat": d["cat"], "mac": d["mac"], "name": d["name"], "kind": d["kind"], "rssi": d["rssi"],
                    "best": d["best"], "hits": d["hits"], "age_s": round(now - d["last"], 1)} for d in act[:40]]
        close = [d for d in act if d["rssi"] is not None and now - d["last"] <= CLOSE_SECS]
        closest = max(close, key=lambda d: d["rssi"]) if close else None
        chans = []
        for hz, ch in sorted(cell_chan.items()):
            chans.append({"hz": hz, "mhz": round(hz / 1e6, 1), "band": band_of(hz), "power_db": ch["power_db"],
                          "best": ch["best"], "hits": ch["hits"], "age_s": round(now - ch["last"], 1)})
        hist = [{"t": round(t, 2), "hz": hz, "v": p} for t, hz, p in cell_hist if now - t <= HIST_SECS]
        per_min = {c: sum(1 for t in recent[c] if now - t <= 60) for c in CATS}
        ev = [{**e, "age_s": round(now - e["t"], 1)} for e in list(events)[-30:]]
        n_act = {c: sum(1 for d in act if d["cat"] == c) for c in ("wifi", "ble", "bt")}
    return {"now": now, "hub": dict(hub_up), "hub_all_up": all(hub_up.values()), "hub_any_up": any(hub_up.values()),
            "counts": dict(counts), "per_min": per_min, "active": n_act, "devices": out_dev,
            "closest": ({"cat": closest["cat"], "mac": closest["mac"], "name": closest["name"], "rssi": closest["rssi"],
                         "age_s": round(now - closest["last"], 1)} if closest else None),
            "cell": {"channels": chans, "history": hist}, "events": ev}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/sdr":
            self._send(200, snapshot())
        elif path == "/health":
            self._send(200, {"ok": True})
        else:
            self._send(404, {"error": "not found"})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8085)
    ap.add_argument("--hub-host", default="127.0.0.1")
    ap.add_argument("--dir", default="~/rfhits")
    ap.add_argument("--demo", action="store_true")
    a = ap.parse_args()
    if a.demo:
        threading.Thread(target=demo, daemon=True).start()
    else:
        preload(os.path.expanduser(a.dir))
        for cat in CATS:
            threading.Thread(target=reader, args=(cat, a.hub_host), daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), Handler)
    print("sdr feed on :%d (%s)" % (a.port, "demo" if a.demo else "hub " + a.hub_host), flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
