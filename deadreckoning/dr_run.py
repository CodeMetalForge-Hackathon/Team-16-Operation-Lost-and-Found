#!/usr/bin/env python3
"""INS coast service: MPU9250 + the drone's orientation + lidar pose fixes -> position, served to the dashboard.

Runs on this PC for the proof of concept; the identical file runs on the Pi once the IMU is wired to it.
The dashboard sets the start coordinate (decimal degrees) and reads the position from :8084.

Sources (pick one):
  --sim                     simulated 100 m flight (loops), with 1 Hz lidar fixes and flight-controller attitude
  --serial COM5 [--baud N]  MPU9250 board streaming lines (see below)
  --replay FILE             a recorded stream in the same line format, played back in real time
Line format (one per line, comma separated; units g, deg/s, microtesla; t in ms):
  I,t_ms,ax,ay,az,gx,gy,gz,mx,my,mz     IMU sample, about 100 Hz
  Q,t_ms,w,x,y,z                        orientation from the flight controller (NED/FRD, as MAVLink/PX4/ArduPilot)
  L,t_ms,agl_m                          lidar height above ground
  F,t_ms,x_m,y_m,z_m,sigma_m            pose_fix from the lidar terrain match, map-frame metres
HTTP (CORS open), default port 8084:
  GET  /ins                              state: position (DD + metres), uncertainty, mode, path
  GET|POST /start?lat=..&lon=..[&alt=..&x=..&y=..&heading=..]   set the start (decimal degrees or DMS text)
  POST /fix        {"x","y","z","sigma"[,"heading","heading_sigma"]} or {"lat","lon","alt","sigma"}    a pose_fix
  POST /attitude   {"q":[w,x,y,z],"convention":"ned_frd"}
  POST /agl        {"agl": metres}
  GET  /mode?mode=auto|flying|grounded|walking   (walking = step counting, for a hand-carried demo)
  GET  /magnetometer?on=0|1               use the compass for heading (turn off indoors; heading then comes from the gyro)
Logs ins_coast (10 Hz) and pose_fix messages to --log as JSON lines.
"""
import argparse
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from drcore import DeadReckoner, load_mag_cal, meters_per_degree, parse_coord

LOCK = threading.Lock()
DR = None
LOGF = None
START_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "start.json")
SOURCE = {"name": "none", "last_input": 0.0}


def log(msg):
    if LOGF:
        LOGF.write(json.dumps(msg) + "\n")


def save_start():
    try:
        with open(START_FILE, "w") as f:
            json.dump({"lat": DR.start_lat, "lon": DR.start_lon, "alt": DR.start_alt,
                       "x": DR.start_xy[0], "y": DR.start_xy[1], "heading": DR.init_heading_deg,
                       "mode": DR.mode, "use_mag": DR.use_mag}, f)
    except OSError:
        pass


def latlon_to_xy(lat, lon):
    mlat, mlon = meters_per_degree(DR.start_lat)
    return (DR.start_xy[0] + (lon - DR.start_lon) * mlon, DR.start_xy[1] + (lat - DR.start_lat) * mlat)


def apply_fix(d, source="lidar"):
    """A pose_fix from the lidar module, either in map metres or as decimal-degree lat/lon."""
    if "lat" in d and "lon" in d:
        x, y = latlon_to_xy(parse_coord(d["lat"], "lat"), parse_coord(d["lon"], "lon"))
        z = d.get("alt")
    else:
        x, y, z = float(d["x"]), float(d["y"]), d.get("z")
    sigma = float(d.get("sigma", 5.0))
    heading = d.get("heading")
    with LOCK:
        ok = DR.apply_fix(x, y, None if z is None else float(z), sigma=sigma,
                          heading=None if heading in (None, "") else float(heading),
                          heading_sigma=float(d.get("heading_sigma", 5.0)))
        log({"msg": "pose_fix", "t": DR.t_prev, "x": round(x, 3), "y": round(y, 3),
             "z": None if z is None else round(float(z), 3), "source": source, "used": ok})
    return ok


def set_start(q):
    lat = parse_coord(q["lat"], "lat")
    lon = parse_coord(q["lon"], "lon")
    alt = float(q["alt"]) if q.get("alt") not in (None, "") else DR.start_alt
    xy = (float(q["x"]), float(q["y"])) if q.get("x") not in (None, "") and q.get("y") not in (None, "") else (0.0, 0.0)
    hd = float(q["heading"]) if q.get("heading") not in (None, "") else DR.init_heading_deg
    with LOCK:
        was_flying = DR.t_prev is not None and not DR.grounded
        DR.reset(lat, lon, alt, xy, hd, keep_bias=True, airborne=was_flying)   # mid-flight: fixes pull it in
        save_start()
    return DR.state()


# ------------------------------------------------------------------ input sources
def handle_line(line):
    """One line of the serial/replay format."""
    p = line.strip().split(",")
    if len(p) < 3:
        return
    kind, t = p[0], float(p[1]) / 1000.0
    v = [float(x) for x in p[2:]]
    SOURCE["last_input"] = time.time()
    with LOCK:
        if kind == "I" and len(v) >= 9:
            DR.feed(t, v[0:3], v[3:6], v[6:9], agl=DR.agl_in)
        elif kind == "Q" and len(v) >= 4:
            DR.feed_attitude(t, v[0:4], "ned_frd")
        elif kind == "L" and v:
            DR.agl_in = v[0]
    if kind == "F" and len(v) >= 3:
        apply_fix({"x": v[0], "y": v[1], "z": v[2], "sigma": v[3] if len(v) > 3 else 5.0})


def serial_source(port, baud):
    SOURCE["name"] = f"serial {port}"
    try:
        import serial
        ser = serial.Serial(port, baud, timeout=1)
        while True:
            line = ser.readline().decode("ascii", "replace")
            if line:
                try:
                    handle_line(line)
                except ValueError:
                    pass
    except ImportError:
        raise SystemExit("pyserial is needed for --serial:  pip install pyserial")


def replay_source(path):
    SOURCE["name"] = f"replay {os.path.basename(path)}"
    with open(path) as f:
        lines = f.readlines()
    t0_data = t0_wall = None
    for line in lines:
        try:
            t = float(line.split(",")[1]) / 1000.0
        except (IndexError, ValueError):
            continue
        t0_data = t if t0_data is None else t0_data
        t0_wall = time.time() if t0_wall is None else t0_wall
        wait = (t - t0_data) - (time.time() - t0_wall)
        if wait > 0:
            time.sleep(wait)
        handle_line(line)


def sim_source(speed, fix_hz, fix_sigma):
    import random
    from sim import simulate
    SOURCE["name"] = "simulated flight"
    rnd = random.Random(3)
    route = [(0, 0, 100), (400, 0, 100), (400, 400, 100), (0, 400, 100), (0, 0, 100)]
    samples, truth, quats = simulate(route, vmax=8.0, return_quat=True, att_err_deg=1.0)
    while True:
        with LOCK:
            sx, sy = DR.start_xy
            DR.reset(keep_bias=False)
        wall0, next_fix = time.time(), 0.0
        for (t, a, g, m), tr, (_, q) in zip(samples, truth, quats):
            wait = t / speed - (time.time() - wall0)
            if wait > 0.002:
                time.sleep(wait)
            SOURCE["last_input"] = time.time()
            with LOCK:
                DR.feed_attitude(t, q, "ned_frd")
                DR.feed(t, a, g, m, agl=tr[2] + rnd.gauss(0, 0.3))
                flying = not DR.grounded
            if fix_hz and flying and t >= next_fix:
                next_fix = t + 1.0 / fix_hz
                apply_fix({"x": sx + tr[0] + rnd.gauss(0, fix_sigma), "y": sy + tr[1] + rnd.gauss(0, fix_sigma),
                           "z": tr[2] + DR.start_alt + rnd.gauss(0, 2.0), "sigma": fix_sigma})
        time.sleep(2.0)


def logger():
    while True:
        time.sleep(0.1)
        with LOCK:
            if DR.t_prev is not None:
                log(DR.ins_coast())
        if LOGF:
            LOGF.flush()


# ------------------------------------------------------------------ HTTP
class Handler(BaseHTTPRequestHandler):
    timeout = 10

    def _send(self, code, body):
        data = json.dumps(body).encode()
        self.send_response(code)
        for k, v in (("Content-Type", "application/json"), ("Content-Length", str(len(data))),
                     ("Access-Control-Allow-Origin", "*"), ("Cache-Control", "no-store")):
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "content-type")
        self.end_headers()

    def _body(self):
        n = min(int(self.headers.get("Content-Length") or 0), 4096)
        return json.loads(self.rfile.read(n)) if n else {}

    def _handle(self, method):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        try:
            if method == "POST":
                q.update(self._body())
            if u.path == "/ins":
                with LOCK:
                    s = DR.state()
                s["source"] = SOURCE["name"]
                s["input_age_s"] = round(time.time() - SOURCE["last_input"], 1) if SOURCE["last_input"] else None
                return self._send(200, s)
            if u.path == "/start":
                return self._send(200, set_start(q))
            if u.path == "/fix":
                return self._send(200, {"used": apply_fix(q)})
            if u.path == "/attitude":
                with LOCK:
                    DR.feed_attitude(DR.t_prev or 0.0, q["q"], q.get("convention", "ned_frd"))
                return self._send(200, {"ok": True})
            if u.path == "/agl":
                with LOCK:
                    DR.agl_in = float(q["agl"])
                return self._send(200, {"ok": True})
            if u.path == "/mode":
                with LOCK:
                    DR.set_mode(q["mode"])
                    save_start()
                return self._send(200, {"mode": q["mode"]})
            if u.path == "/magnetometer":
                with LOCK:
                    DR.use_mag = str(q.get("on", "1")).lower() in ("1", "true", "on", "yes")
                    DR.att.mag_allowed = DR.use_mag
                    save_start()
                return self._send(200, {"use_mag": DR.use_mag})
            if u.path == "/health":
                return self._send(200, {"ok": True})
            self._send(404, {"error": "not found"})
        except (ValueError, KeyError, TypeError) as exc:
            self._send(400, {"error": str(exc)})

    def do_GET(self):
        self._handle("GET")

    def do_POST(self):
        self._handle("POST")

    def log_message(self, *a):
        pass


class Server(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 32


def main():
    global DR, LOGF
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8084)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--speed", type=float, default=1.0, help="simulation speed multiplier")
    ap.add_argument("--sim-fixes", type=float, default=1.0, help="simulated lidar fix rate, Hz (0 = coast only)")
    ap.add_argument("--serial")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--replay")
    ap.add_argument("--lat", default=None, help="start latitude, decimal degrees")
    ap.add_argument("--lon", default=None, help="start longitude, decimal degrees")
    ap.add_argument("--alt", type=float, default=None)
    ap.add_argument("--heading", type=float, default=None, help="start heading deg (if no orientation input)")
    ap.add_argument("--log", default="ins.jsonl")
    a = ap.parse_args()

    saved = {}
    try:
        with open(START_FILE) as f:
            saved = json.load(f)
    except (OSError, ValueError):
        pass
    lat = parse_coord(a.lat, "lat") if a.lat is not None else saved.get("lat", 38.8977)
    lon = parse_coord(a.lon, "lon") if a.lon is not None else saved.get("lon", -77.0365)
    DR = DeadReckoner(lat=lat, lon=lon, alt=a.alt if a.alt is not None else saved.get("alt", 0.0),
                      start_xy=(saved.get("x", 0.0), saved.get("y", 0.0)),
                      init_heading_deg=a.heading if a.heading is not None else saved.get("heading", 0.0),
                      mag_cal=load_mag_cal(os.path.join(os.path.dirname(os.path.abspath(__file__)), "mag_cal.json")),
                      mag_aligned=False)
    DR.agl_in = None
    if saved.get("mode") in ("auto", "flying", "grounded", "walking"):
        DR.set_mode(saved["mode"])          # the demo mode survives a restart or a Pi reboot
    if "use_mag" in saved:
        DR.use_mag = bool(saved["use_mag"])
        DR.att.mag_allowed = DR.use_mag
    LOGF = open(a.log, "a") if a.log else None

    if a.sim:
        DR.mag_aligned = True
        threading.Thread(target=sim_source, args=(a.speed, a.sim_fixes, 5.0), daemon=True).start()
    elif a.serial:
        threading.Thread(target=serial_source, args=(a.serial, a.baud), daemon=True).start()
    elif a.replay:
        threading.Thread(target=replay_source, args=(a.replay,), daemon=True).start()
    threading.Thread(target=logger, daemon=True).start()
    print(f"INS service on :{a.port}  source: {SOURCE['name'] if SOURCE['name'] != 'none' else 'waiting for input'}"
          f"  start {lat:.6f}, {lon:.6f}", flush=True)
    try:
        Server(("0.0.0.0", a.port), Handler).serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
