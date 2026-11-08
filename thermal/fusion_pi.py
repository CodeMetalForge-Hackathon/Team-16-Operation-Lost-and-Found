"""Fused thermal + visible human detection on the Pi: ONE process, ONE Hailo-8, two cameras.

  visible (Pi Cam 3, rpicam-vid)  -> YOLOv8s on Hailo (color)   -> person boxes
  thermal (HT-203U, GStreamer)    -> YOLOv8s on Hailo (gray) + RandomForest on CPU + hotspot boxes
  fusion: each visible person box is checked for a hotspot inside it (via a thermal->visible
  homography). Heat + shape = HUMAN; shape without heat = NO HEAT (decoy?); heat without a visible
  person = THERMAL ONLY (maybe hidden under cover).

Run ON the Pi from ~/team16_thermal/thermal (see start_fusion.sh):
  python3 fusion_pi.py                       (and, in another process, python3 fusion_web.py -> :8081 :8082)
  python3 fusion_pi.py --record ~/team16_thermal/pairs   # also save paired frames
Calibrate the camera alignment with calibrate_fusion.py (on a PC) using saved pairs; without
fusion_calib.json the overlay is marked UNCALIBRATED and uses a rough centered guess.
"""
import argparse
import faulthandler
import json
import signal
import sys
import math
import subprocess
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from thermal_pi import CpuBackend, HailoBackend, HEF_DEFAULT, SIZE, find_camera, open_camera, thermal_crop

HERE = Path(__file__).resolve().parent
CALIB = HERE / "fusion_calib.json"
WEIGHTS = HERE / "fusion_weights.json"
# Hand-set logistic weights (no paired training data yet). Override with fusion_weights.json.
W = {"b": -3.0, "visible": 3.0, "thermal": 1.5, "match": 2.0}
THERMAL_SHAPE = (192, 256)  # rows, cols of the cropped thermal frame
THERMAL_FPS = 18  # the camera delivers 25; frames beyond this rate are dropped to save CPU
THM_SCALE_YOLO = 0.25       # thermal YOLO scores run low; this score counts as "full"


def sigmoid(z):
    return 1.0 / (1.0 + math.exp(-z))


# ------------------------------------------------------------------ sources
class Latest(threading.Thread):
    """Background grabber that keeps only the newest frame."""

    def __init__(self):
        super().__init__(daemon=True)
        self.frame, self.n, self.alive, self.lock = None, 0, True, threading.Lock()

    def push(self, f):
        with self.lock:
            self.frame, self.n = f, self.n + 1

    def get(self):
        with self.lock:
            return self.frame, self.n


class ThermalSource(Latest):
    """Reads the HT-203U; survives USB dropouts by reopening, and reports frame age."""

    def __init__(self, dev=None):
        super().__init__()
        self.dev, self.cap, self.t_frame = dev, None, 0.0

    def age(self):
        return time.time() - self.t_frame if self.t_frame else 1e9

    def _open(self):
        dev = self.dev or find_camera()
        pipe = (f"v4l2src device={dev} ! video/x-raw,format=YUY2,width=256,height=392,framerate=25/1 "
                "! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false")
        cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
        return cap if cap.isOpened() else None

    def run(self):
        fails = 0
        while self.alive:
            if self.cap is None:
                self.cap = self._open()
                if self.cap is None:
                    time.sleep(2.0)
                    continue
                print("thermal camera connected", flush=True)
            ok, f = self.cap.read()
            if ok:
                fails = 0
                now = time.time()
                if now - self.t_frame >= 1.0 / THERMAL_FPS - 0.002:
                    self.t_frame = now
                    self.push(thermal_crop(f))
            else:
                fails += 1
                time.sleep(0.05)
                if fails >= 20:  # ~1 s of nothing: drop the pipeline and look for the camera again
                    print("thermal camera lost, retrying", flush=True)
                    self.cap.release()
                    self.cap, fails = None, 0

    def close(self):
        self.alive = False
        if self.cap is not None:
            self.cap.release()


class VisibleSource(Latest):
    """rpicam-vid MJPEG on stdout, split into JPEGs."""

    def __init__(self, w, h, fps, rotation=180):
        super().__init__()
        # --mode 2304:1296 is the full-field-of-view binned mode; the default pick for 1280x720
        # (1536x864) is a centre crop that clips the edges of the Pi Cam 3 view.
        self.p = subprocess.Popen(
            ["rpicam-vid", "-n", "-t", "0", "--codec", "mjpeg", "--width", str(w), "--height", str(h),
             "--mode", "2304:1296", "--rotation", str(rotation),
             "--framerate", str(fps), "--autofocus-mode", "continuous", "-o", "-"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)

    def run(self):
        buf = b""
        while self.alive:
            chunk = self.p.stdout.read(65536)
            if not chunk:
                break
            buf += chunk
            last = None
            while True:
                a = buf.find(b"\xff\xd8")
                b = buf.find(b"\xff\xd9", a + 2) if a >= 0 else -1
                if a < 0 or b < 0:
                    break
                last, buf = buf[a:b + 2], buf[b + 2:]
            if last is not None:
                f = cv2.imdecode(np.frombuffer(last, np.uint8), cv2.IMREAD_COLOR)
                if f is not None:
                    self.push(f)
        self.alive = False

    def close(self):
        self.alive = False
        self.p.terminate()


# ------------------------------------------------------------------- Hailo
class FusionHailo(HailoBackend):
    """One Hailo device/network shared by both cameras (color for visible, gray for thermal)."""
    color = False

    def prep(self, img):
        if not self.color:
            return super().prep(img)
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        s = min(SIZE / w, SIZE / h)
        nw, nh = int(round(w * s)), int(round(h * s))
        canvas = np.full((SIZE, SIZE, 3), 114, np.uint8)
        top, left = (SIZE - nh) // 2, (SIZE - nw) // 2
        canvas[top:top + nh, left:left + nw] = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
        self._lb = (s, top, left, h, w)
        return canvas

    def detect(self, img, color):
        self.color = color
        return self.run(img)


class NoHailo:
    """Stand-in when the Hailo chip is missing/wedged: cameras and heat hotspots still work, no AI person boxes."""
    name = "none"

    def detect(self, img, color):
        return 0.0, {}, []

    def close(self):
        pass


# ----------------------------------------------------------------- thermal
def hotspot_boxes(img):
    """Same hotspot logic as thermal_viewer.draw_hotspots, but returns (x, y, w, h) boxes."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    smooth = cv2.GaussianBlur(gray, (5, 5), 0)
    cutoff = max(float(np.median(smooth)) + 15, float(np.percentile(smooth, 90)) - 8)
    mask = (smooth >= cutoff).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [cv2.boundingRect(c) for c in cnts if cv2.contourArea(c) >= 30]


def load_homography(vis_w, vis_h):
    """3x3 matrix mapping thermal-crop pixels -> visible pixels. Returns (H, calibrated)."""
    if CALIB.exists():
        d = json.loads(CALIB.read_text())
        H = np.array(d["H"], np.float64)
        sx, sy = vis_w / d.get("vis_w", vis_w), vis_h / d.get("vis_h", vis_h)
        return np.diag([sx, sy, 1.0]) @ H, True
    th, tw = THERMAL_SHAPE
    scale = 0.7 * vis_h / th  # guess: thermal view covers the central 70% of visible height
    ox, oy = vis_w / 2 - tw * scale / 2, vis_h / 2 - th * scale / 2
    return np.array([[scale, 0, ox], [0, scale, oy], [0, 0, 1]], np.float64), False


def map_box(H, box):
    x, y, w, h = box
    pts = np.array([[[x, y], [x + w, y], [x + w, y + h], [x, y + h]]], np.float32)
    q = cv2.perspectiveTransform(pts, H)[0]
    x0, y0 = q.min(axis=0)
    x1, y1 = q.max(axis=0)
    return int(x0), int(y0), int(x1), int(y1)


# ------------------------------------------------------------------ fusion
def fuse(vis_boxes, hot_boxes, H, rf_p, thm_yolo):
    """Returns (targets, overall). target = dict(kind, p, vbox (visible px) or None, tbox (thermal px) or None)."""
    te = 0.5 * rf_p + 0.5 * min(1.0, thm_yolo / THM_SCALE_YOLO)  # global thermal evidence 0..1
    mapped = [(hb, map_box(H, hb)) for hb in hot_boxes]
    used, targets = set(), []
    for sv, x1, y1, x2, y2 in vis_boxes:
        if sv < 0.3:
            continue
        mx, my = 0.15 * (x2 - x1), 0.15 * (y2 - y1)
        match = None
        for i, (hb, (a, b, c, d)) in enumerate(mapped):
            cx, cy = (a + c) / 2, (b + d) / 2
            if x1 - mx <= cx <= x2 + mx and y1 - my <= cy <= y2 + my:
                match = i
                used.add(i)
                break
        z = W["b"] + W["visible"] * sv + W["thermal"] * te + W["match"] * (match is not None)
        p = sigmoid(z)
        if match is not None and p >= 0.6:
            kind = "HUMAN"
        elif match is None and sv >= 0.5:
            kind = "NO HEAT (decoy?)"
        else:
            kind = "UNCONFIRMED"
        targets.append({"kind": kind, "p": p, "vbox": (x1, y1, x2, y2), "vscore": sv,
                        "tbox": mapped[match][0] if match is not None else None})
    for i, (hb, mb) in enumerate(mapped):
        if i in used or te < 0.5:
            continue
        p = sigmoid(-2.0 + 3.0 * te)
        targets.append({"kind": "THERMAL ONLY", "p": p, "vbox": mb, "vscore": 0.0, "tbox": hb})
    overall = max([t["p"] for t in targets if t["kind"] in ("HUMAN", "UNCONFIRMED", "THERMAL ONLY")] + [0.3 * te])
    return targets, overall, te


COLORS = {"HUMAN": (0, 220, 0), "UNCONFIRMED": (0, 190, 255), "NO HEAT (decoy?)": (160, 160, 160),
          "THERMAL ONLY": (255, 120, 0)}


def footprint(H, vis_shape):
    """Region of the visible frame that the thermal camera sees: (x0, y0, x1, y1), clamped."""
    th, tw = THERMAL_SHAPE
    x0, y0, x1, y1 = map_box(H, (0, 0, tw, th))
    vh, vw = vis_shape[:2]
    x0, y0, x1, y1 = max(0, x0), max(0, y0), min(vw, x1), min(vh, y1)
    if x1 - x0 < 64 or y1 - y0 < 64:  # degenerate calibration: fall back to the whole frame
        return 0, 0, vw, vh
    return x0, y0, x1, y1


def label(img, text, x, y, col, scale=0.5):
    (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    y = y if y - h - 4 >= 0 else y + h + 8
    x = min(max(x, 0), max(img.shape[1] - w - 2, 0))
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, col, 1, cv2.LINE_AA)


def status_bar(width, line1, col1, line2, col2):
    bar = np.zeros((64, width, 3), np.uint8)
    cv2.putText(bar, line1, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, col1, 2, cv2.LINE_AA)
    cv2.putText(bar, line2, (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col2, 1, cv2.LINE_AA)
    return bar


def render(vis, thm, targets, overall, te, calibrated, stats, rect, rf_p, thm_score, thermal_ok=True, view_h=480):
    """Returns (fused_view, thermal_view). Fused = visible cropped to the thermal footprint + boxes."""
    x0, y0, x1, y1 = rect
    vs = view_h / (y1 - y0)
    v = cv2.resize(vis[y0:y1, x0:x1], None, fx=vs, fy=vs)
    ts = view_h / thm.shape[0]
    t = cv2.resize(thm, None, fx=ts, fy=ts, interpolation=cv2.INTER_NEAREST)
    for tg in targets:
        col = COLORS[tg["kind"]]
        bx1, by1, bx2, by2 = tg["vbox"]
        if (bx1 + bx2) / 2 < x0 or (bx1 + bx2) / 2 > x1 or (by1 + by2) / 2 < y0 or (by1 + by2) / 2 > y1:
            continue  # target is outside what the thermal camera covers
        p1 = (int((bx1 - x0) * vs), int((by1 - y0) * vs))
        p2 = (int((bx2 - x0) * vs), int((by2 - y0) * vs))
        cv2.rectangle(v, p1, p2, col, 2)
        label(v, f"{tg['kind']} {tg['p']:.0%}", p1[0], p1[1] - 5, col)
        if tg["tbox"] is not None:
            x, y, w, h = [int(c * ts) for c in tg["tbox"]]
            cv2.rectangle(t, (x, y), (x + w, y + h), col, 2)
    t = cv2.rotate(t, cv2.ROTATE_180)  # camera is mounted upside down: show it upright (processing uses the raw orientation)
    col = (0, 220, 0) if overall >= 0.6 else (0, 190, 255) if overall >= 0.3 else (200, 200, 200)
    note = "" if calibrated else "UNCALIBRATED  "
    fused = np.vstack([v, status_bar(v.shape[1], f"FUSED human likelihood {overall:.0%}", col,
                                     note + stats, (0, 160, 255) if note else (200, 200, 200))])
    tcol = (0, 220, 0) if te >= 0.5 else (200, 200, 200)
    if thermal_ok:
        thermal_view = np.vstack([t, status_bar(t.shape[1], f"THERMAL evidence {te:.0%}", tcol,
                                                f"RF {rf_p:.0%}   thermal-YOLO {thm_score:.2f}", (200, 200, 200))])
    else:
        t = (t * 0.25).astype(np.uint8)
        cv2.putText(t, "NO THERMAL SIGNAL", (t.shape[1] // 2 - 150, t.shape[0] // 2), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (0, 0, 255), 2, cv2.LINE_AA)
        thermal_view = np.vstack([t, status_bar(t.shape[1], "THERMAL OFFLINE", (0, 0, 255),
                                                "camera not delivering frames - replug USB (it reconnects itself)", (200, 200, 200))])
        fused = np.vstack([v, status_bar(v.shape[1], f"VISIBLE ONLY {overall:.0%} (no thermal)", (0, 0, 255), "thermal offline", (200, 200, 200))])
    return fused, thermal_view


# ------------------------------------------------------------- auto-calibration
class AutoCalib:
    """Fit thermal->visible alignment from a lone person seen by both cameras.

    Each sample pairs the thermal-YOLO person box with the visible-YOLO person box. Their edges give
    x = sx*tx + ox and y = sy*ty + oy (scale + offset, no rotation), solved by least squares.
    """
    NEED, MAX, SECS = 12, 60, 30

    def __init__(self):
        self.lock = threading.Lock()
        self.state, self.samples, self.deadline, self.msg = "idle", [], 0.0, ""
        self.frames = self.one_v = self.one_t = 0

    def start(self):
        with self.lock:
            self.state, self.samples, self.deadline = "collecting", [], time.time() + self.SECS
            self.frames = self.one_v = self.one_t = 0
            self.msg = "Stand ALONE in view of both cameras and move slowly side to side"

    def add(self, vboxes, tboxes):
        v = [b for b in vboxes if b[0] >= 0.5]
        t = [b for b in tboxes if b[0] >= 0.3]
        with self.lock:
            if self.state == "collecting":
                self.frames += 1
                self.one_v += len(v) == 1
                self.one_t += len(t) == 1
            if self.state == "collecting" and len(v) == 1 and len(t) == 1:
                self.samples.append((t[0][1:], v[0][1:]))  # (x1,y1,x2,y2) each

    def due(self):
        with self.lock:
            return self.state == "collecting" and (len(self.samples) >= self.MAX or time.time() > self.deadline)

    @staticmethod
    def _fit(pairs):
        A = np.array([[a, 1.0] for a, _ in pairs])
        b = np.array([c for _, c in pairs])
        for _ in range(2):  # refit once without the worst 20%
            sol = np.linalg.lstsq(A, b, rcond=None)[0]
            res = np.abs(A @ sol - b)
            keep = res <= np.percentile(res, 80)
            A, b = A[keep], b[keep]
        sol = np.linalg.lstsq(A, b, rcond=None)[0]
        return sol[0], sol[1], float(np.abs(A @ sol - b).mean())

    def finish(self, vis_shape):
        with self.lock:
            s, self.state = list(self.samples), "idle"
        if len(s) < self.NEED:
            self.msg = (f"Failed: only {len(s)} usable of {self.frames} frames (need {self.NEED}). "
                        f"One person seen in visible camera in {self.one_v}, in thermal in {self.one_t}. "
                        "Stand alone, fully in BOTH views, close enough to be detected.")
            print("calibration:", self.msg, flush=True)
            return False
        xs = [(tb[0], vb[0]) for tb, vb in s] + [(tb[2], vb[2]) for tb, vb in s]
        ys = [(tb[1], vb[1]) for tb, vb in s] + [(tb[3], vb[3]) for tb, vb in s]
        sx, ox, ex = self._fit(xs)
        sy, oy, ey = self._fit(ys)
        spread_x = max(a for a, _ in xs) - min(a for a, _ in xs)  # thermal px the person actually covered
        spread_y = max(a for a, _ in ys) - min(a for a, _ in ys)
        print(f"calibration fit: sx={sx:.2f} sy={sy:.2f} thermal spread x={spread_x:.0f}px y={spread_y:.0f}px samples={len(s)}", flush=True)
        if spread_x < 40:
            self.msg = (f"Failed: you only covered {spread_x:.0f} px of the thermal image left-to-right (need 40+). "
                        "Walk slowly from one side of the thermal view to the other, fully in view, alone.")
            print("calibration:", self.msg, flush=True)
            return False
        if not (sx > 0 and sy > 0 and 0.6 < sy / sx < 2.0):
            self.msg = f"Failed: implausible fit (scale {sx:.2f}/{sy:.2f}, x-range {spread_x:.0f}px). Try again, standing alone and moving."
            print("calibration:", self.msg, flush=True)
            return False
        CALIB.write_text(json.dumps({"H": [[sx, 0, ox], [0, sy, oy], [0, 0, 1]], "vis_w": vis_shape[1],
                                     "vis_h": vis_shape[0], "samples": len(s), "err_px": round((ex + ey) / 2, 1)}, indent=2))
        RELOAD.set()
        self.msg = f"Calibrated from {len(s)} frames, error ~{(ex + ey) / 2:.0f} px"
        print("calibration:", self.msg, flush=True)
        return True

    def status(self):
        with self.lock:
            return {"state": self.state, "samples": len(self.samples), "needed": self.NEED,
                    "secs_left": max(0, int(self.deadline - time.time())) if self.state == "collecting" else 0,
                    "message": self.msg, "calibrated": CALIB.exists(),
                    "thermal_ok": THERMAL.age() < 1.5 if hasattr(THERMAL, "t_frame") else False}


CAL = AutoCalib()
THERMAL = ThermalSource.__new__(ThermalSource)  # replaced in main(); lets the handler report camera health
RELOAD = threading.Event()


# ------------------------------------------------- shared-memory hand-off to fusion_web.py
# The web server runs in its OWN process (fusion_web.py) so it never competes with the Hailo loop
# for the GIL. We only write the latest frames/status to RAM and read control commands from it.
SHM = Path("/dev/shm/team16_fusion")


def _atomic(path, data):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_bytes(data)
    tmp.replace(path)


def publish(name, img):
    ok, j = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if ok:
        _atomic(SHM / f"{name}.jpg", j.tobytes())


def publish_status(extra):
    _atomic(SHM / "status.json", json.dumps({**CAL.status(), **extra}).encode())


def poll_commands():
    cmd = SHM / "cmd"
    try:
        text = cmd.read_text().strip()
        cmd.unlink()
    except FileNotFoundError:
        return
    if text == "calibrate":
        print("calibration requested", flush=True)
        CAL.start()
    elif text == "reset":
        CALIB.unlink(missing_ok=True)
        RELOAD.set()
        CAL.state, CAL.msg = "idle", "Reset to rough guess"


# -------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hef", default=str(HEF_DEFAULT))
    ap.add_argument("--device", help="thermal camera node (default: auto-detect)")
    ap.add_argument("--vis-size", default="1280x720")
    ap.add_argument("--vis-fps", type=int, default=15)
    ap.add_argument("--vis-rotation", type=int, choices=[0, 180], default=180, help="Pi Cam 3 is mounted upside-down")
    ap.add_argument("--record", metavar="DIR", help="also save paired raw frames here")
    ap.add_argument("--record-every", type=float, default=2.0, help="seconds between saved pairs")
    ap.add_argument("--log", default="fusion_detections.jsonl")
    ap.add_argument("--no-hailo", action="store_true", help="skip the Hailo chip (cameras + thermal classifier only)")
    a = ap.parse_args()
    def _term(*_):
        raise KeyboardInterrupt  # lets the finally block release the Hailo and cameras cleanly

    signal.signal(signal.SIGTERM, _term)
    faulthandler.register(signal.SIGUSR1, all_threads=True)  # kill -USR1 <pid> dumps every thread's stack to the log
    vw, vh = (int(x) for x in a.vis_size.split("x"))
    if WEIGHTS.exists():
        W.update(json.loads(WEIGHTS.read_text()))

    cpu = CpuBackend()
    try:
        if a.no_hailo:
            raise RuntimeError("--no-hailo")
        hailo = FusionHailo(a.hef)
    except Exception as e:  # chip gone: keep the screens alive without AI boxes
        print(f"HAILO UNAVAILABLE ({type(e).__name__}) - running cameras + heat hotspots only", flush=True)
        hailo = NoHailo()
    global THERMAL
    thermal, visible = ThermalSource(a.device), VisibleSource(vw, vh, a.vis_fps, a.vis_rotation)
    THERMAL = thermal
    thermal.start()
    visible.start()
    SHM.mkdir(parents=True, exist_ok=True)
    for f in SHM.glob("*"):
        f.unlink()  # drop stale frames/commands from a previous run
    t_status = 0.0
    rec = Path(a.record).expanduser() if a.record else None
    if rec:
        rec.mkdir(parents=True, exist_ok=True)
    H, calibrated = None, False
    smooth, last_vn, last_rec, fps, t_last = None, -1, 0.0, 0.0, time.time()
    frames_done, t_beat = 0, time.time()
    logf = open(a.log, "a", buffering=1)
    print("running (Ctrl+C to stop)", flush=True)
    try:
        while visible.alive:
            vis, vn = visible.get()
            thm, _ = thermal.get()
            thermal_ok = thermal.age() < 1.5 and thm is not None
            if thm is None:
                thm = np.zeros((*THERMAL_SHAPE, 3), np.uint8)
            if vis is None or vn == last_vn:
                time.sleep(0.005)
                continue
            last_vn = vn
            if H is None:
                H, calibrated = load_homography(vis.shape[1], vis.shape[0])
            t0 = time.perf_counter()
            _, _, vboxes = hailo.detect(vis, color=True)
            if thermal_ok:
                thm_score, _, tboxes = hailo.detect(thm, color=False)
                rf_p, _, _ = cpu.run(thm)
                hot = hotspot_boxes(thm)
                CAL.add(vboxes, tboxes)
            else:  # never fuse a stale/frozen thermal frame
                thm_score, tboxes, rf_p, hot = 0.0, [], 0.0, []
            if CAL.due():
                CAL.finish(vis.shape)
            if RELOAD.is_set():
                RELOAD.clear()
                H, calibrated = load_homography(vis.shape[1], vis.shape[0])
            targets, overall, te = fuse(vboxes, hot, H, rf_p, thm_score)
            if not thermal_ok:
                targets = [x for x in targets if x["kind"] != "NO HEAT (decoy?)"]  # absence of heat means nothing now
            smooth = overall if smooth is None else 0.6 * smooth + 0.4 * overall
            ms = (time.perf_counter() - t0) * 1e3
            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - t_last, 1e-6)
            t_last = now
            st = CAL.status()
            stats = (f"CALIBRATING {st['samples']}/{st['needed']} - stand alone, move slowly ({st['secs_left']}s)"
                     if st["state"] == "collecting" and thermal_ok else f"{fps:.0f} fps  {ms:.0f} ms")
            fused_img, thermal_img = render(vis, thm, targets, smooth, te, calibrated, stats,
                                            footprint(H, vis.shape), rf_p, thm_score, thermal_ok)
            frames_done += 1
            if now - t_beat >= 5:
                print(f"heartbeat: {frames_done} frames, thermal_ok={thermal_ok}, vis {vis.shape[1]}x{vis.shape[0]}", flush=True)
                t_beat = now
            poll_commands()
            if now - t_status >= 0.3:
                t_status = now
                publish_status({"fps": round(fps, 1), "t": now})
            publish("fused", fused_img)
            publish("thermal", thermal_img)
            if any(t["kind"] in ("HUMAN", "THERMAL ONLY") for t in targets):
                logf.write(json.dumps({"t": now, "overall": round(smooth, 3), "targets": [
                    {"kind": t["kind"], "p": round(t["p"], 3), "vbox": list(map(int, t["vbox"]))} for t in targets]}) + "\n")
            if rec and now - last_rec >= a.record_every:
                last_rec = now
                stamp = time.strftime("%H%M%S") + f"_{int(now * 1000) % 1000:03d}"
                cv2.imwrite(str(rec / f"{stamp}_vis.jpg"), vis)
                cv2.imwrite(str(rec / f"{stamp}_thm.png"), thm)
    except KeyboardInterrupt:
        pass
    finally:
        thermal.close()
        visible.close()
        hailo.close()


if __name__ == "__main__":
    main()
