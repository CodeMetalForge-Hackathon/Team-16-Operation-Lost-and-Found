#!/usr/bin/env python3
"""Pi health feed for the dashboard (stdlib only, read-only). GET /stats returns JSON on :8087 (CORS open).

Reads /proc, /sys, ps, vcgencmd and the detector's own status and log files. It never opens the Hailo or
any other device, so watching it cannot disturb the detector.
"""
import argparse
import json
import os
import re
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HOME = os.path.expanduser("~")
STATUS = "/dev/shm/team16_fusion/status.json"
FUSION_LOG = os.path.join(HOME, "team16_thermal", "fusion.log")
HAILO_PCI = "/sys/bus/pci/devices/0000:01:00.0"
HIST_SECS = 300
FRIENDLY = {"fusion_pi.py": "Detector (Hailo)", "fusion_web.py": "Camera web server", "phone_scan.py": "Cellular scan (Pluto)",
            "rf_hub.py": "RF hub", "sdr_feed.py": "SDR bridge", "dr_run.py": "INS", "serve.py": "Dashboard server",
            "map_feed.py": "Lidar map feed", "pi_stats.py": "This stats feed"}

lock = threading.Lock()
state = {"cpu": {"total": None, "cores": []}, "hist": deque(maxlen=400), "net": {}}
cache = {}


def run(cmd, timeout=4):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except Exception:
        return ""


def cached(key, ttl, fn):
    now = time.time()
    if key in cache and now - cache[key][0] < ttl:
        return cache[key][1]
    val = fn()
    cache[key] = (now, val)
    return val


def read(path, default=None):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def cpu_times():
    out = []
    for line in (read("/proc/stat") or "").splitlines():
        if line.startswith("cpu"):
            f = line.split()
            v = list(map(int, f[1:9]))
            idle = v[3] + v[4]
            out.append((f[0], sum(v), idle))
    return out


def net_bytes():
    out = {}
    for line in (read("/proc/net/dev") or "").splitlines()[2:]:
        name, _, rest = line.partition(":")
        name = name.strip()
        if name == "lo":
            continue
        f = rest.split()
        out[name] = (int(f[0]), int(f[8]))
    return out


def meminfo():
    m = {}
    for line in (read("/proc/meminfo") or "").splitlines():
        k, _, v = line.partition(":")
        m[k] = int(v.split()[0]) * 1024 if v.split() else 0
    total, avail = m.get("MemTotal", 0), m.get("MemAvailable", 0)
    return {"total_mb": round(total / 1e6), "used_mb": round((total - avail) / 1e6),
            "used_pct": round(100 * (total - avail) / total, 1) if total else None}


def sampler():
    prev, prev_net, prev_t = cpu_times(), net_bytes(), time.time()
    while True:
        time.sleep(2)
        cur, cur_net, now = cpu_times(), net_bytes(), time.time()
        pct = {}
        for (n, t0, i0), (_, t1, i1) in zip(prev, cur):
            dt = t1 - t0
            pct[n] = round(100.0 * (1 - (i1 - i0) / dt), 1) if dt > 0 else 0.0
        dts = max(now - prev_t, 0.001)
        net = {k: {"rx_kbps": round((cur_net[k][0] - prev_net.get(k, cur_net[k])[0]) * 8 / 1000 / dts, 1),
                   "tx_kbps": round((cur_net[k][1] - prev_net.get(k, cur_net[k])[1]) * 8 / 1000 / dts, 1)}
               for k in cur_net}
        temp = read("/sys/class/thermal/thermal_zone0/temp")
        temp = round(int(temp) / 1000, 1) if temp else None
        with lock:
            state["cpu"] = {"total": pct.get("cpu"), "cores": [pct[k] for k in sorted(pct) if k != "cpu"]}
            state["net"] = net
            state["hist"].append({"t": round(now, 1), "cpu": pct.get("cpu"), "temp": temp, "mem": meminfo()["used_pct"]})
        prev, prev_net, prev_t = cur, cur_net, now


def throttle():
    out = run(["vcgencmd", "get_throttled"]).strip()
    m = re.search(r"0x([0-9a-fA-F]+)", out)
    if not m:
        return None
    v = int(m.group(1), 16)
    names_now = ((0, "undervoltage"), (1, "freq capped"), (2, "throttled"), (3, "soft temp limit"))
    names_ever = ((16, "undervoltage"), (17, "freq capped"), (18, "throttled"), (19, "soft temp limit"))
    return {"raw": "0x%x" % v, "now": [n for b, n in names_now if v >> b & 1],
            "since_boot": [n for b, n in names_ever if v >> b & 1]}


def top_procs():
    rows = []
    for line in run(["ps", "-eo", "pid,pcpu,pmem,comm,args", "--sort=-pcpu"]).splitlines()[1:9]:
        f = line.split(None, 4)
        if len(f) < 5 or f[3] == "ps":
            continue
        script = next((os.path.basename(a) for a in f[4].split() if a.endswith(".py")), f[3])
        rows.append({"pid": int(f[0]), "cpu": float(f[1]), "mem": float(f[2]), "name": FRIENDLY.get(script, script)})
    return rows[:6]


def procs_running():
    names = set()
    for line in run(["ps", "-eo", "args"]).splitlines():
        for a in line.split():
            if a.endswith(".py"):
                names.add(os.path.basename(a))
    return names


def fusion_log_stats():
    try:
        with open(FUSION_LOG, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 400_000))
            text = f.read().decode("utf-8", "replace")
    except OSError:
        return {"starts_15min": None, "watchdogs_15min": None, "last_watchdog_age_s": None, "hailo_unavailable": None}
    now = time.time()
    starts, dogs, last_dog = 0, 0, None
    for m in re.finditer(r"\[supervisor (\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)\] (started|WATCHDOG)", text):
        try:
            t = time.mktime(time.strptime(m.group(1), "%Y-%m-%d %H:%M:%S"))
        except ValueError:
            continue
        if m.group(2) == "WATCHDOG":
            last_dog = t
        if now - t <= 900:
            if m.group(2) == "started":
                starts += 1
            else:
                dogs += 1
    return {"starts_15min": starts, "watchdogs_15min": dogs,
            "last_watchdog_age_s": round(now - last_dog) if last_dog else None,
            "hailo_unavailable": "HAILO UNAVAILABLE" in text[-6000:]}


def hailo_errors():
    out = run(["sudo", "-n", "dmesg"], 6)
    if not out:
        return None
    return sum(1 for l in out.splitlines() if "hailo" in l.lower() and ("disconnected" in l.lower() or "timeout" in l.lower()))


def hailo_info():
    present = os.path.isdir(HAILO_PCI) and os.path.exists("/dev/hailo0")
    try:
        age = time.time() - os.stat(STATUS).st_mtime
        st = json.loads(read(STATUS, "{}") or "{}")
        det = {"age_s": round(age, 1), "fps": st.get("fps"), "ok": age < 8, "ai": st.get("ai"),
               "ai_stalls_15min": st.get("ai_stalls_15min")}
    except (OSError, ValueError):
        det = {"age_s": None, "fps": None, "ok": False}
    link = None
    if present:
        link = (read(HAILO_PCI + "/current_link_speed", "") + " x" + read(HAILO_PCI + "/current_link_width", "?")).strip()
    return {"on_bus": present, "link": link, "driver_errors": cached("herr", 20, hailo_errors),
            "detector": det, "supervisor": cached("flog", 5, fusion_log_stats)}


def snapshot():
    with lock:
        cpu = dict(state["cpu"])
        hist = list(state["hist"])
        net = dict(state["net"])
    now = time.time()
    hist = [h for h in hist if now - h["t"] <= HIST_SECS]
    st = os.statvfs("/")
    la = (read("/proc/loadavg") or "0 0 0").split()[:3]
    freq = read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
    running = cached("procs", 3, procs_running)
    return {"now": now, "uptime_s": round(float((read("/proc/uptime") or "0").split()[0])),
            "cpu": cpu, "cpu_mhz": round(int(freq) / 1000) if freq else None,
            "temp_c": hist[-1]["temp"] if hist else None, "load": [float(x) for x in la], "cores_n": os.cpu_count(),
            "mem": meminfo(),
            "disk": {"used_pct": round(100 * (1 - st.f_bavail / st.f_blocks), 1),
                     "free_gb": round(st.f_bavail * st.f_frsize / 1e9, 1)},
            "throttle": cached("thr", 5, throttle), "net": net, "top": cached("top", 2, top_procs),
            "hailo": hailo_info(), "cell_scan_running": "phone_scan.py" in running, "history": hist}


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
        if path == "/stats":
            self._send(200, snapshot())
        elif path == "/health":
            self._send(200, {"ok": True})
        else:
            self._send(404, {"error": "not found"})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8087)
    a = ap.parse_args()
    threading.Thread(target=sampler, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), Handler)
    print("pi stats on :%d" % a.port, flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
