#!/usr/bin/env python3
"""
Ground-station map: one browser page that shows the search probability map, the pod's track,
detections, beacon drops and RF hits on the height map, next to the Pi's live camera feed.

  1. python ground_station/server.py --camera http://<pi>:8080/
  2. open http://localhost:8000   (teammates on the same Wi-Fi: http://<laptop-ip>:8000)
  3. start a search from the page (pick a trail or click the map)

It reads JSON-lines logs every refresh (files are re-read, so anything appended shows up):
  pose_fix / ins_coast : {"type","t", "lat","lon" | "x","y" (UTM 19N m), "z", "source"}
  detection            : {"type","t","cls","confidence",...}  pose optional -> matched to the track by time
  drop_event           : {"type","t","pose"|lat/lon,"beacon_id"}
  rf_hit               : {"type","t","hz","power_db", pose optional}
Default logs: ground_station/logs/*.jsonl and ./detections.jsonl (written by picam_color_anomaly.py on the PC).
Only Python's standard library + numpy + matplotlib. Works offline (no map tiles).
"""
import argparse, glob, io, json, os, random, re, subprocess, sys, threading, time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from geo import to_latlon, dist_m, latlon_to_utm, utm_to_latlon
from planner import Planner

POSE_TYPES = ("pose_fix", "ins_coast")


class Run:
    """The search area: probability grid + elevation saved by hotspots.py."""
    def __init__(self, run_dir):
        self.dir = run_dir
        g = json.load(open(os.path.join(run_dir, "grid.json")))
        self.g = g
        self.prob = np.load(os.path.join(run_dir, "probability.npy"))
        self.z = np.load(os.path.join(run_dir, "elevation.npy"))
        self.streams = np.load(os.path.join(run_dir, "streams.npy")).astype(bool)
        rp = os.path.join(run_dir, "route.npy")
        self.route = np.load(rp).astype(bool) if os.path.exists(rp) else np.zeros_like(self.streams)
        self.hotspots = [json.loads(l) for l in open(os.path.join(run_dir, "hotspots.jsonl")) if l.strip()]
        self.bounds = dict(north=g["lat0"], west=g["lon0"],
                           south=g["lat0"] + g["dlat"] * g["rows"], east=g["lon0"] + g["dlon"] * g["cols"])
        self.cell_m = (g["dx"] + g["dy"]) / 2
        self.title = g.get("title", "")

    def rc(self, lat, lon):
        return int((lat - self.g["lat0"]) / self.g["dlat"]), int((lon - self.g["lon0"]) / self.g["dlon"])

    def latlon(self, r, c):
        return self.g["lat0"] + (r + 0.5) * self.g["dlat"], self.g["lon0"] + (c + 0.5) * self.g["dlon"]


def load_region(png_path):
    """A georeferenced overview map (PNG + .pgw world file, EPSG:26919), e.g. data/wmnf_demo_map.png."""
    pgw = os.path.splitext(png_path)[0] + ".pgw"
    if not (os.path.exists(png_path) and os.path.exists(pgw)):
        return None, None
    A, D, B, E, C, F = [float(v) for v in open(pgw).read().split()[:6]]
    from PIL import Image
    w, h = Image.open(png_path).size
    left, top = C - A / 2, F - E / 2                        # world file gives the centre of the top-left pixel
    meta = dict(left=left, top=top, right=left + A * w, bottom=top + E * h, w=w, h=h)
    return open(png_path, "rb").read(), meta


def png(rgba):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    buf = io.BytesIO(); plt.imsave(buf, rgba, format="png"); return buf.getvalue()


def feather(shape, frac=0.10):
    """0 at the grid edge -> 1 inside, with a smooth ramp, so overlays blend into the map (no hard square)."""
    rows, cols = shape
    r = np.minimum(np.arange(rows), np.arange(rows)[::-1])[:, None]
    c = np.minimum(np.arange(cols), np.arange(cols)[::-1])[None, :]
    t = np.clip(np.minimum(r, c) / max(1.0, frac * min(rows, cols)), 0, 1)
    return t * t * (3 - 2 * t)                               # smoothstep


def render_basemap(run):
    """Relief shading drawn as a transparent overlay: shadows darken and sunlit slopes brighten the
    team's map underneath, so its colours, roads and labels show through. Streams and the planned
    route are drawn as thin lines. The edges fade out into the map."""
    from matplotlib.colors import LightSource
    from scipy import ndimage
    hs = LightSource(315, 45).hillshade(run.z, dx=run.g["dx"], dy=run.g["dy"], vert_exag=2.0)
    f = feather(run.z.shape)
    rgba = np.zeros(run.z.shape + (4,))
    shade = np.clip(0.62 - hs, 0, 1) / 0.62                 # 0 on lit slopes, 1 in deep shadow
    light = np.clip(hs - 0.75, 0, 1) / 0.25
    rgba[..., :3] = np.where(shade[..., None] > 0, [0.10, 0.12, 0.10], [1.0, 1.0, 0.95])
    rgba[..., 3] = np.maximum(shade * 0.55, light * 0.25) * f
    streams = run.streams
    rgba[streams] = [0.20, 0.47, 0.85, 0.85]
    rgba[streams, 3] *= f[streams]
    route = run.route
    rgba[route] = [0.55, 0.12, 0.10, 0.95]                  # trail in a dark red, like topo-map trails
    return png(np.clip(rgba, 0, 1))


def render_heat(prob):
    """Search probability as a soft warm glow (transparent where unlikely), feathered at the edges."""
    from matplotlib.colors import LinearSegmentedColormap
    from scipy import ndimage
    lp = np.log10(prob + 1e-12)
    lo, hi = np.quantile(lp, 0.75), np.quantile(lp, 0.999)
    v = np.clip((lp - lo) / max(hi - lo, 1e-9), 0, 1)
    v = ndimage.gaussian_filter(v, 1.2)
    cmap = LinearSegmentedColormap.from_list("glow", ["#ffe680", "#ffae3b", "#ff6a2b", "#d7263d"])
    rgba = cmap(v)
    rgba[..., 3] = np.clip(v ** 1.3 * 0.62, 0, 0.62) * feather(prob.shape, 0.06)
    return png(rgba)


def read_logs(patterns):
    recs = []
    for pat in patterns:
        for path in glob.glob(pat):
            try:
                with open(path, encoding="utf-8", errors="ignore") as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith("{"):
                            try:
                                recs.append(json.loads(line))
                            except json.JSONDecodeError:
                                pass                         # a half-written last line
            except OSError:
                pass
    recs.sort(key=lambda r: float(r.get("t", 0)))
    return recs


def with_xy(d):
    if d.get("lat") is not None and d.get("lon") is not None:
        d["x"], d["y"] = latlon_to_utm(d["lat"], d["lon"])
    return d


class State:
    def __init__(self, run, logs, footprint_m, pod, match_s, beacons=10):
        self.run, self.logs, self.footprint_m, self.pod, self.match_s = run, logs, footprint_m, pod, match_s
        self.beacons = beacons
        self.basemap = render_basemap(run) if run else None      # no search area until the user starts one
        self._cov_key, self._heat, self._cache = None, (render_heat(run.prob) if run else None), {}
        self._planner_next = None
        self.heat_version = 0
        self.run_id = 1 if run else 0
        self.lock = threading.Lock()

    def switch(self, run):
        """Show a different search area (after a new simulation is set up)."""
        basemap, heat = render_basemap(run), render_heat(run.prob)
        with self.lock:
            self.run, self.basemap, self._heat = run, basemap, heat
            self._cov_key, self._cache, self._planner_next = None, {}, None
            self.heat_version += 1
            self.run_id += 1

    def clear(self):
        """Back to the empty start screen: no search area."""
        with self.lock:
            self.run, self.basemap, self._heat = None, None, None
            self._cov_key, self._cache, self._planner_next = None, {}, None
            self.run_id += 1

    def build(self):
        with self.lock:
            return self._build()

    def _build(self):
        run = self.run
        if run is None:                                            # waiting for the user to start a search
            return dict(active=False, run_id=0, hotspots=[], track=[], detections=[], drops=[], rf_hits=[],
                        events=[], next=None, status="idle", heat_version=0, now=time.time(),
                        title="No active search")
        recs = read_logs(self.logs)
        track = []
        for r in recs:
            if r.get("type") in POSE_TYPES:
                ll = to_latlon(r)
                if ll:
                    track.append(with_xy(dict(t=float(r.get("t", 0)), lat=ll[0], lon=ll[1], sim=r.get("sim"),
                                              z=r.get("z"), source=r.get("source", r.get("type")))))
        tt = np.array([p["t"] for p in track]) if track else np.array([])

        def place(r):
            ll = to_latlon(r.get("pose") or {}) or to_latlon(r)
            if ll:
                return ll, "logged"
            if len(tt):
                i = int(np.argmin(np.abs(tt - float(r.get("t", 0)))))
                if abs(tt[i] - float(r.get("t", 0))) <= self.match_s:
                    return (track[i]["lat"], track[i]["lon"]), "matched-by-time"
            return None, "no-position"

        dets, drops, rfs, events = [], [], [], []
        for r in recs:
            ty = r.get("type")
            if ty == "detection_note":
                events.append(dict(r, placed="note")); continue
            if ty not in ("detection", "drop_event", "rf_hit"):
                continue
            ll, how = place(r)
            item = with_xy(dict(r, lat=ll[0] if ll else None, lon=ll[1] if ll else None, placed=how))
            if ty == "drop_event":
                if len(drops) >= self.beacons:
                    continue                                       # the pod can't drop more beacons than it carries
                item["kind"] = r.get("kind") or "confirmed"       # drops without a kind count as a confirmed find
            {"detection": dets, "drop_event": drops, "rf_hit": rfs}[ty].append(item)
            events.append(item)

        # Coverage -> Bayesian update, done exactly like the pod's planner (planner.py): every `plan`
        # message starts a new pass; each pass multiplies what the camera saw by (1 - POD).
        plans = [r for r in recs if r.get("type") == "plan"]
        key = (len(track), len(plans))
        if key != self._cov_key:
            pl = Planner(run.dir, self.footprint_m, self.pod)
            ti = 0
            for pr in plans + [None]:
                t_end = float(pr["t"]) if pr else float("inf")
                while ti < len(track) and track[ti]["t"] < t_end:
                    pl.see(track[ti]["lat"], track[ti]["lon"]); ti += 1
                pl.end_pass()
            nlat, nlon, mass = pl.next_box()
            self._planner_next = with_xy(dict(lat=nlat, lon=nlon, size_m=pl.box_m, poa=mass, label="SEARCH NEXT"))
            seen = pl.passes > 0
            self._cache = dict(searched=pl.searched_share(),
                               covered_km2=float(seen.sum() * run.g["dx"] * run.g["dy"] / 1e6))
            self._heat = render_heat(pl.posterior())
            self._cov_key = key
            self.heat_version += 1

        # What the yellow box shows: the box the pod announced (plan), else the planner's suggestion.
        status, nxt = "searching" if track else "idle", self._planner_next
        if plans:
            lp = plans[-1]
            if lp.get("status") in ("located", "return"):
                status, nxt = lp["status"], None
            else:
                nxt = with_xy(dict(lat=lp["lat"], lon=lp["lon"], size_m=lp.get("size_m", 500), poa=lp.get("poa"),
                                   box=lp.get("box"), label="SEARCH NEXT"))
                if track and abs(track[-1]["lat"] - lp["lat"]) * 111320 < nxt["size_m"] / 2 + 60 \
                        and abs(track[-1]["lon"] - lp["lon"]) * 111320 * 0.72 < nxt["size_m"] / 2 + 60:
                    nxt["label"] = "SEARCHING NOW"
        if any(d.get("kind") == "confirmed" for d in drops):
            status, nxt = "located", None
        truth = next((with_xy(dict(r)) for r in reversed(recs) if r.get("type") == "sim_truth"), None)
        if status not in ("located", "return"):
            truth = None                                           # never reveal the answer mid-search
        for pr in plans:
            if pr.get("box"):
                events.append(dict(pr, placed="logged"))

        # hot spots still waiting for the pod (none of the track within the footprint)
        hs = []
        for h in run.hotspots:
            visited = any(dist_m((h["lat"], h["lon"]), (p["lat"], p["lon"])) <= self.footprint_m for p in track[::3])
            hs.append(with_xy(dict(rank=h["rank"], zone=h.get("zone"), lat=h["lat"], lon=h["lon"],
                                   poa=h.get("poa_250m"), reasons=h.get("reasons", []), visited=visited)))
        b = run.bounds
        corners = dict(nw=latlon_to_utm(b["north"], b["west"]), ne=latlon_to_utm(b["north"], b["east"]),
                       sw=latlon_to_utm(b["south"], b["west"]))
        return dict(active=True, title=run.title, bounds=run.bounds, corners=corners, size=list(run.prob.shape), hotspots=hs,
                    track=track[-5000:], detections=dets, drops=drops, rf_hits=rfs,
                    events=[dict(t=e.get("t"), type=e.get("type"), cls=e.get("cls"), color=e.get("color"),
                                 confidence=e.get("confidence"), beacon_id=e.get("beacon_id"), hz=e.get("hz"),
                                 power_db=e.get("power_db"), note=e.get("note"), placed=e["placed"],
                                 kind=e.get("kind"), reason=e.get("reason"), beacons_left=e.get("beacons_left"),
                                 box=e.get("box"), poa=e.get("poa"))
                            for e in sorted([e for e in events if e.get("type") != "rf_hit"] + rfs[-8:],
                                            key=lambda e: float(e.get("t", 0)))[-40:]],
                    heat_version=self.heat_version, run_id=self.run_id, now=time.time(), next=nxt, status=status, truth=truth,
                    beacons_total=self.beacons, beacons_left=max(0, self.beacons - len(drops)),
                    **self._cache)


class SimJob:
    """Sets up a new search area with hotspots.py, then flies simulate_flight.py over it, in the background."""
    def __init__(self, state, excel):
        self.state, self.excel = state, excel
        self.stage, self.message, self.label = "idle", "", ""
        self.proc, self.busy = None, False

    def info(self):
        if self.proc and self.stage == "flying" and self.proc.poll() is not None:
            self.stage, self.message = "done", "Simulation finished. Pick another place to run again."
        return dict(stage=self.stage, message=self.message, label=self.label)

    def reset(self):
        """Stop the pod and wipe the simulated flight, so the map shows a fresh, unsearched area."""
        self.cancelled = True                                  # a search map still being built is thrown away
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except Exception:
                pass
        log = os.path.join(HERE, "logs", "sim_flight.jsonl")
        if os.path.exists(log):
            os.remove(log)
        self.state.clear()
        self.stage, self.message = "idle", ""
        return True, "ok"

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
        if self.stage == "flying":
            self.stage, self.message = "done", "Simulation stopped."

    def start(self, trail, lat, lon, speed, seed, hiker="realistic"):
        self.hiker = hiker
        if self.busy:
            return False, "Still setting up the previous simulation, wait a moment."
        self.busy, self.cancelled = True, False
        threading.Thread(target=self._run, args=(trail, lat, lon, speed, seed), daemon=True).start()
        return True, "ok"

    def _run(self, trail, lat, lon, speed, seed):
        try:
            self.stop()
            self.label = trail or f"last known point {lat:.4f}, {lon:.4f}"
            self.stage, self.message = "building", f"Building the search map for {self.label}... (reads the height map, ~10-30 s)"
            slug = re.sub(r"[^a-z0-9]+", "_", (trail or f"pt_{lat:.4f}_{lon:.4f}").lower()).strip("_")[:40]
            out_dir = os.path.join(ROOT, "out", "sim_" + slug)
            cmd = [sys.executable, os.path.join(ROOT, "search_hotspots", "hotspots.py"), "--out", out_dir,
                   "--trail", trail or "none"]
            if lat is not None:
                cmd += ["--ipp", f"{lat},{lon}"]
            if self.excel:
                cmd += ["--excel", self.excel]
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
            if r.returncode != 0 or not os.path.exists(os.path.join(out_dir, "grid.json")):
                self.stage = "error"
                self.message = "Could not build the search map: " + (r.stderr or r.stdout).strip().splitlines()[-1:][0][:300]
                return
            if getattr(self, "cancelled", False):          # the search was reset while the map was building
                self.stage, self.message = "idle", ""
                return
            warn = "Warning: height map not found, so this uses synthetic terrain. " if "synthetic" in r.stdout else ""
            self.state.switch(Run(out_dir))
            log = os.path.join(HERE, "logs", "sim_flight.jsonl")
            if os.path.exists(log):
                os.remove(log)
            self.proc = subprocess.Popen([sys.executable, os.path.join(HERE, "simulate_flight.py"), "--run", out_dir,
                                          "--live", str(speed), "--seed", str(seed), "--out", log,
                                          "--hiker", getattr(self, "hiker", "realistic"),
                                          "--beacons", str(self.state.beacons)],
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.stage = "flying"
            self.message = warn + f"Pod is searching near {self.label} ({speed}x speed)."
        except Exception as e:
            self.stage, self.message = "error", f"Simulation failed: {e}"
        finally:
            self.busy = False


def trail_list():
    """Trails the search tool knows (name + start point) for the page's dropdown."""
    sys.path.insert(0, os.path.join(ROOT, "search_hotspots"))
    import hotspots as H
    gaz = {g["name"]: g for g in H.load_gazetteer()}
    out = []
    for name, wps in H.TRAIL_ROUTES.items():
        p = gaz.get(wps[0])
        if p:
            x, y = latlon_to_utm(p["lat"], p["lon"])
            out.append(dict(name=name, lat=p["lat"], lon=p["lon"], x=x, y=y))
    return sorted(out, key=lambda d: d["name"])


def places_list():
    """Peaks, trailheads and landmarks from the gazetteer, for map labels."""
    sys.path.insert(0, os.path.join(ROOT, "search_hotspots"))
    import hotspots as H
    out = []
    for g in H.load_gazetteer():
        x, y = latlon_to_utm(g["lat"], g["lon"])
        out.append(dict(name=g["name"], kind=g["kind"], lat=g["lat"], lon=g["lon"], x=x, y=y))
    return out


def make_handler(state, camera, region_png=None, region_meta=None, lock=None, job=None):
    page = open(os.path.join(HERE, "index.html"), "rb").read()
    try:
        trails = trail_list()
    except Exception as e:
        print("trail list unavailable:", e); trails = []
    names = {t["name"] for t in trails}
    try:
        places = places_list()
    except Exception as e:
        print("places unavailable:", e); places = []

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send(self, body, ctype, code=200):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            try:
                if path == "/":
                    self.send(page, "text/html; charset=utf-8")
                elif path == "/api/config":
                    self.send(json.dumps(dict(camera=camera, region=region_meta, lock=lock)).encode(), "application/json")
                elif path == "/region.png" and region_png:
                    self.send(region_png, "image/png")
                elif path == "/api/state":
                    st = state.build()
                    st["job"] = job.info() if job else None
                    self.send(json.dumps(st).encode(), "application/json")
                elif path == "/api/places":
                    self.send(json.dumps(places).encode(), "application/json")
                elif path == "/api/trails":
                    self.send(json.dumps(trails).encode(), "application/json")
                elif path == "/basemap.png" and state.basemap:
                    self.send(state.basemap, "image/png")
                elif path == "/heat.png" and state._heat:
                    self.send(state._heat, "image/png")
                else:
                    self.send(b"not found", "text/plain", 404)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as e:                          # keep the server alive during the demo
                self.send(json.dumps(dict(error=str(e))).encode(), "application/json", 500)
        def do_POST(self):
            path = self.path.split("?")[0]
            try:
                n = int(self.headers.get("Content-Length", 0) or 0)
                body = json.loads(self.rfile.read(n) or b"{}") if n else {}
                if path == "/api/stop" and job:
                    job.stop(); self.send(b'{"ok":true}', "application/json")
                elif path == "/api/reset" and job:
                    ok, msg = job.reset()
                    self.send(json.dumps(dict(ok=ok, error=None if ok else msg)).encode(), "application/json",
                              200 if ok else 409)
                elif path == "/api/simulate" and job:
                    trail = body.get("trail") or None
                    if trail and trail not in names:
                        return self.send(json.dumps(dict(ok=False, error="unknown trail")).encode(), "application/json", 400)
                    lat = lon = None
                    if body.get("x") is not None and body.get("y") is not None:
                        x, y = float(body["x"]), float(body["y"])
                        if lock and not (lock["left"] <= x <= lock["right"] and lock["bottom"] <= y <= lock["top"]):
                            return self.send(json.dumps(dict(ok=False, error="point is outside the mapped area")).encode(),
                                             "application/json", 400)
                        lat, lon = utm_to_latlon(x, y)
                    if not trail and lat is None:
                        return self.send(json.dumps(dict(ok=False, error="pick a trail or click the map")).encode(),
                                         "application/json", 400)
                    speed = max(1, min(200, int(body.get("speed", 20))))
                    seed = int(body["seed"]) if str(body.get("seed", "")).strip().isdigit() else random.randint(1, 9999)
                    hiker = body.get("hiker") if body.get("hiker") in ("realistic", "model") else "realistic"
                    ok, msg = job.start(trail, lat, lon, speed, seed, hiker)
                    self.send(json.dumps(dict(ok=ok, error=None if ok else msg, seed=seed)).encode(), "application/json",
                              200 if ok else 409)
                else:
                    self.send(b"not found", "text/plain", 404)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as e:
                self.send(json.dumps(dict(ok=False, error=str(e))).encode(), "application/json", 500)
    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", default=None,
                    help="optional: open with this hotspots.py output folder already loaded "
                         "(default: start empty and wait for the user to start a search on the page)")
    ap.add_argument("--logs", nargs="*", default=[os.path.join(HERE, "logs", "*.jsonl"),
                                                   os.path.join(ROOT, "detections.jsonl")])
    ap.add_argument("--camera", default="", help="Pi MJPEG feed, e.g. http://<pi>:8080/")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--lock", default="map",
                    help="box the map view is locked to: 'map' = the whole overview image (default), "
                         "'terrain' = height_map.tif extent (tighter), 'left,bottom,right,top' in UTM 19N m, or 'none'")
    ap.add_argument("--basemap", default=os.path.join(ROOT, "data", "wmnf_demo_map.png"),
                    help="overview map PNG with a .pgw world file (EPSG:26919)")
    ap.add_argument("--footprint-m", type=float, default=60, help="camera ground footprint radius")
    ap.add_argument("--pod", type=float, default=0.7, help="probability of detection inside the footprint")
    ap.add_argument("--beacons", type=int, default=10, help="beacons the pod carries")
    ap.add_argument("--match-s", type=float, default=10, help="max time gap to pin a detection to the track")
    a = ap.parse_args()
    if a.run and not os.path.exists(os.path.join(a.run, "grid.json")):
        sys.exit(f"{a.run}/grid.json not found - re-run search_hotspots/hotspots.py (current version) first.")
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    old_sim = os.path.join(HERE, "logs", "sim_flight.jsonl")
    if os.path.exists(old_sim):                                    # every session starts clean
        os.remove(old_sim)
    state = State(Run(a.run) if a.run else None, a.logs, a.footprint_m, a.pod, a.match_s, a.beacons)
    region_png, region_meta = load_region(a.basemap)
    lock = None
    spec = (a.lock or "none").lower()
    if spec == "map" and region_meta:
        spec = f"{region_meta['left']},{region_meta['bottom']},{region_meta['right']},{region_meta['top']}"
    elif spec == "terrain":
        spec = "232705,4745304,439538,5000060"
    if spec not in ("none", "map"):
        l, b, r, t = [float(v) for v in spec.split(",")]
        if region_meta:                                     # never lock outside the overview image
            l, b = max(l, region_meta["left"]), max(b, region_meta["bottom"])
            r, t = min(r, region_meta["right"]), min(t, region_meta["top"])
        lock = dict(left=l, bottom=b, right=r, top=t)
    print("Overview map:", a.basemap if region_png else "none (pass --basemap PNG with .pgw)")
    excel = os.path.join(ROOT, "data", "white_mountains_trails.xlsx")
    job = SimJob(state, excel if os.path.exists(excel) else None)
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), make_handler(state, a.camera, region_png, region_meta, lock, job))
    print(f"Ground station: http://localhost:{a.port}   (reading {', '.join(a.logs)})")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
