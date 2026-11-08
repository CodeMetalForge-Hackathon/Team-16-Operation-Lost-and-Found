#!/usr/bin/env python3
"""Beacon telemetry feed for the dashboard (stdlib only, receive-only: nothing here transmits).

Collects what the receivers hear from the DF beacon and serves it as JSON on :8083.

Sources (use any combination):
  --serial /dev/ttyUSB0   LoRa32 receiver board (esp32-pico-rx). It prints lines like
                          "RX rssi=-92 dBm snr=7.5 dB len=24  T16DF,BCA14,12,345[,3987]"
                          (id, packet counter, beacon uptime s, optional battery mV)
  --sdr-file PATH         JSON lines appended by an SDR script, one per power reading:
                          {"t": 1699999999.1, "freq_hz": 915000000, "power_dbm": -97.2}
  --demo                  simulated beacon + SDR, for building/testing the dashboard
  POST /ingest            same two shapes as JSON: {"type":"packet", ...} or {"type":"sdr", ...}

GET /beacon  -> state (CORS open).   GET /health -> {"ok": true}
"""
import argparse
import json
import os
import random
import re
import statistics
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HIST_SECS = 300
LOST_AFTER = 10.0  # seconds without a packet before the link counts as lost
PKT_RE = re.compile(r"RX rssi=(-?\d+)\s*dBm\s+snr=(-?[\d.]+)\s*dB\s+len=\d+\s+T16DF,([0-9A-Fa-f]+),(\d+),(\d+)(?:,(\d+))?")

lock = threading.Lock()
packets = deque(maxlen=1200)   # dicts: t id seq uptime rssi snr mv lost_before
sdr = deque(maxlen=3000)       # (t, freq_hz, power_dbm)
last_seq = {}                  # id -> last seq seen
totals = {"received": 0, "lost": 0}
sources = set()
sdr_unit = ["dBm"]               # unit of the SDR readings (the Pluto script reports relative dBFS)


def add_packet(rssi, snr, dev_id, seq, uptime, mv=None, t=None):
    t = t or time.time()
    with lock:
        prev = last_seq.get(dev_id)
        lost = 0
        if prev is not None and seq > prev and seq - prev <= 1000:
            lost = seq - prev - 1
        last_seq[dev_id] = seq  # a lower seq means the beacon rebooted: restart the count, no loss
        totals["received"] += 1
        totals["lost"] += lost
        packets.append({"t": t, "id": dev_id, "seq": seq, "uptime": uptime, "rssi": rssi, "snr": snr,
                        "mv": mv, "lost_before": lost})


def add_sdr(freq_hz, power_dbm, t=None, unit=None):
    with lock:
        if unit:
            sdr_unit[0] = str(unit)[:8]
        sdr.append((t or time.time(), float(freq_hz), float(power_dbm)))


# --------------------------------------------------------------------------- sources
def serial_reader(path):
    import termios
    import tty
    sources.add("lora-serial")
    while True:
        try:
            fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
            tty.setraw(fd)
            a = termios.tcgetattr(fd)
            a[4] = a[5] = termios.B115200
            termios.tcsetattr(fd, termios.TCSANOW, a)
            buf = b""
            while True:
                chunk = os.read(fd, 512)
                if not chunk:
                    raise OSError("serial closed")
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    m = PKT_RE.search(line.decode("ascii", "replace"))
                    if m:
                        add_packet(int(m[1]), float(m[2]), m[3].upper(), int(m[4]), int(m[5]),
                                   int(m[6]) if m[6] else None)
        except Exception as exc:
            print(f"serial {path}: {exc}; retrying in 3 s", flush=True)
            time.sleep(3)


def sdr_tail(path):
    sources.add("sdr")
    pos = 0
    while True:
        try:
            if os.path.getsize(path) < pos:
                pos = 0  # file was rotated
            with open(path, "r") as f:
                f.seek(pos)
                for line in f:
                    if not line.endswith("\n"):
                        break  # partial line, finish it next time
                    pos += len(line.encode())
                    try:
                        d = json.loads(line)
                        add_sdr(d["freq_hz"], d["power_dbm"], d.get("t"), d.get("unit"))
                    except (ValueError, KeyError):
                        pass
        except OSError:
            pass
        time.sleep(0.5)


def demo():
    sources.update(["demo", "sdr"])
    seq, start, level, t0 = 0, time.time(), -104.0, time.time()
    next_pkt = 0.0
    while True:
        now = time.time()
        outage = (now - t0) % 90 > 75  # 15 s of "no signal" every 90 s so the lost state can be seen
        level = max(-128, min(-85, level + random.gauss(0, 1.2)))
        if now >= next_pkt:
            next_pkt = now + 1.0
            seq += 1
            if not outage and random.random() > 0.08:
                add_packet(int(level + random.gauss(0, 2)), round(random.gauss(5 + (level + 105) / 6, 1), 1),
                           "BCA14", seq, int(now - start), 3950 - int((now - start) / 40))
        pulse = (not outage) and (now % 1.0) < 0.25
        add_sdr(915e6, -112 + random.gauss(0, 1.0) + (level + 128 + 12 if pulse else 0) * (0.6 if pulse else 0))
        time.sleep(0.25)


# --------------------------------------------------------------------------- state
def link_state(age, rssi_avg):
    if age is None:
        return "none"
    if age > LOST_AFTER:
        return "lost"
    if rssi_avg is None:
        return "ok"
    return "strong" if rssi_avg >= -100 else "ok" if rssi_avg >= -115 else "weak"


def snapshot():
    now = time.time()
    with lock:
        pk = [p for p in packets if now - p["t"] <= HIST_SECS]
        sd = [s for s in sdr if now - s[0] <= HIST_SECS]
        recv_tot, lost_tot = totals["received"], totals["lost"]
        latest_all = packets[-1] if packets else None
    last60 = [p for p in pk if now - p["t"] <= 60]
    got = len(last60)
    lost60 = sum(p["lost_before"] for p in last60)
    age = (now - latest_all["t"]) if latest_all else None
    avg5 = statistics.mean(p["rssi"] for p in pk[-5:]) if pk else None
    out = {
        "t": now,
        "sources": sorted(sources),
        "link": link_state(age, avg5),
        "age_s": None if age is None else round(age, 1),
        "latest": latest_all,
        "stats": {
            "received_total": recv_tot, "lost_total": lost_tot,
            "rate_per_min": got,
            "loss_pct_60s": round(100.0 * lost60 / (got + lost60), 1) if got + lost60 else None,
            "rssi_avg5": None if avg5 is None else round(avg5, 1),
            "rssi_best_5min": max((p["rssi"] for p in pk), default=None),
        },
        "packets": [{k: p[k] for k in ("t", "seq", "rssi", "snr", "lost_before")} for p in pk[-10:]][::-1],
        "history": [{"t": p["t"], "rssi": p["rssi"], "snr": p["snr"]} for p in pk],
        "sdr": None,
    }
    if sd:
        floor = statistics.median(s[2] for s in sd)
        cur = max(s[2] for s in sd[-8:])  # peak of the last ~2 s: LoRa chirps are short bursts
        step = max(1, len(sd) // 300)
        out["sdr"] = {"unit": sdr_unit[0], "freq_mhz": round(sd[-1][1] / 1e6, 3), "power_dbm": round(cur, 1),
                      "noise_floor_dbm": round(floor, 1), "over_floor_db": round(cur - floor, 1),
                      "detected": cur - floor >= 6.0, "age_s": round(now - sd[-1][0], 1),
                      "history": [{"t": s[0], "p": round(s[2], 1)} for s in sd[::step]]}
        seen = next((s[0] for s in reversed(sd) if s[2] - floor >= 6.0), None)  # last reading clearly above the floor
        out["sdr"]["last_detect_age_s"] = None if seen is None else round(now - seen, 1)
        # No LoRa receiver yet (or it lost the beacon) but the SDR hears it: report the link as SDR-only.
        if out["link"] in ("none", "lost") and seen is not None and now - seen <= LOST_AFTER and len(sd) >= 20:
            out["link"] = "sdr"
    return out


class Handler(BaseHTTPRequestHandler):
    timeout = 10

    def _send(self, code, body):
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_OPTIONS(self):
        self.send_response(204)
        for k, v in (("Access-Control-Allow-Origin", "*"), ("Access-Control-Allow-Headers", "content-type")):
            self.send_header(k, v)
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/beacon":
            self._send(200, snapshot())
        elif path == "/health":
            self._send(200, {"ok": True})
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        if self.path.split("?")[0] != "/ingest":
            return self._send(404, {"error": "not found"})
        try:
            d = json.loads(self.rfile.read(min(int(self.headers.get("Content-Length", 0)), 4096)))
            if d["type"] == "packet":
                add_packet(int(d["rssi"]), float(d.get("snr", 0)), str(d["id"]).upper(), int(d["seq"]),
                           int(d.get("uptime", 0)), d.get("mv"))
            elif d["type"] == "sdr":
                add_sdr(d["freq_hz"], d["power_dbm"], d.get("t"), d.get("unit"))
            else:
                raise ValueError("type must be packet or sdr")
            sources.add("ingest")
            self._send(200, {"ok": True})
        except (ValueError, KeyError, TypeError) as exc:
            self._send(400, {"error": str(exc)})

    def log_message(self, *a):
        pass


class Server(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 32


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8083)
    ap.add_argument("--serial", help="LoRa receiver board, e.g. /dev/ttyUSB0")
    ap.add_argument("--sdr-file", help="JSON-lines file written by the SDR script")
    ap.add_argument("--demo", action="store_true")
    a = ap.parse_args()
    if a.serial:
        threading.Thread(target=serial_reader, args=(a.serial,), daemon=True).start()
    if a.sdr_file:
        threading.Thread(target=sdr_tail, args=(a.sdr_file,), daemon=True).start()
    if a.demo:
        threading.Thread(target=demo, daemon=True).start()
    print(f"beacon feed on :{a.port}  sources: {a.serial or ''} {a.sdr_file or ''} {'demo' if a.demo else ''}", flush=True)
    try:
        Server(("0.0.0.0", a.port), Handler).serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
