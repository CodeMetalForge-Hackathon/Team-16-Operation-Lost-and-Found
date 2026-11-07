"""Unnatural-color hotspot detector with a learned baseline (office-friendly).

1. LEARN: for a few seconds, pan the camera around the room. Every saturated pixel
   adds to a HSV histogram of "colors that normally exist here". Grays/whites/blacks
   (low saturation or dark) are ignored, so a beige office does not matter.
2. WATCH: saturated pixels whose color is rare in that baseline become candidates.
   Candidates that persist for several frames become HOTSPOTS: circled on the live
   feed, a `detection` JSON line is appended to detections.jsonl, and it pings.

Run on the PC (starts the camera stream on the Pi over SSH):
    $env:PI_HOST="<pi address>" ; python picam_color_anomaly.py
Run on the Pi (reads the camera directly, serves the annotated feed over HTTP):
    python3 picam_color_anomaly.py --source local --headless --serve 8080
    then open http://<pi>:8080/ in a browser on the PC
Keys (window): b relearn baseline, m toggle mask view, s save frame, q quit.
Headless/serve control: touch ~/relearn on the Pi to relearn the baseline.
Self-test without a camera:  python picam_color_anomaly.py --selftest
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------- parameters
SMALL = (320, 180)      # detection runs at this size (fast on the Pi)
S_MIN, V_MIN = 70, 60   # "chromatic" pixel gate (ignores gray/white/black)
HB, SB, VB = 30, 8, 8   # histogram bins for H (0-179), S, V
COMMON_P = 0.002        # a color bin is "normal" if >= this share of baseline pixels
NV = 16                 # brightness levels for neutral (gray/white/black) pixels
NEUTRAL_P = 0.01        # a neutral shade is "normal" if >= this share of neutral pixels
NCH = HB * SB * VB      # chromatic bins; neutral bins follow
MIN_AREA = 40           # blob area in SMALL pixels
MIN_AREA_NEUTRAL = 150  # neutral blobs must be bigger (less noise from shadows)
CONFIRM_HITS = 4        # frames a blob must persist before it pings
MATCH_DIST = 30         # px (SMALL) to match a blob to an existing track

HUES = [(10, "red"), (22, "orange"), (35, "yellow"), (85, "green"), (100, "cyan"),
        (130, "blue"), (150, "purple"), (170, "pink"), (180, "red")]


def hue_name(h):
    for top, name in HUES:
        if h < top:
            return name
    return "red"


def bin_index(hsv):
    """Chromatic pixels -> (H,S,V) bin; neutral pixels -> brightness-only bin (hue is noise there)."""
    h = hsv[..., 0].astype(np.int32) * HB // 180
    s = hsv[..., 1].astype(np.int32) * SB // 256
    v = hsv[..., 2].astype(np.int32) * VB // 256
    chroma = (h * SB + s) * VB + v
    neutral = NCH + hsv[..., 2].astype(np.int32) * NV // 256
    return np.where(chroma_mask(hsv), chroma, neutral)


def chroma_mask(hsv):
    return (hsv[..., 1] >= S_MIN) & (hsv[..., 2] >= V_MIN)


def smooth_hist(hist):
    """Blur the baseline histogram so near-identical shades count as common."""
    h = hist.reshape(HB, SB, VB).astype(np.float64)
    p = np.pad(h, ((1, 1), (0, 0), (0, 0)), mode="wrap")
    p = np.pad(p, ((0, 0), (1, 1), (1, 1)), mode="constant")
    out = np.zeros_like(h)
    for dh in range(3):
        for ds in range(3):
            for dv in range(3):
                out += p[dh:dh + HB, ds:ds + SB, dv:dv + VB]
    return out.reshape(-1)


class Baseline:
    def __init__(self, use_neutral=True):
        self.use_neutral = use_neutral
        self.reset()

    def reset(self):
        self.counts = np.zeros(NCH + NV, np.float64)
        self.common = None

    def learn(self, hsv):
        self.counts += np.bincount(bin_index(hsv).ravel(), minlength=self.counts.size)

    def finish(self):
        chroma, neutral = self.counts[:NCH], self.counts[NCH:]
        self.common = np.zeros(NCH + NV, bool)
        # chromatic: share of saturated baseline pixels (27 = smoothing kernel size)
        if chroma.sum() >= 500:
            self.common[:NCH] = smooth_hist(chroma) / (27.0 * chroma.sum()) >= COMMON_P
        # neutral: share of gray/white/black pixels, smoothed over neighbouring brightness
        if self.use_neutral and neutral.sum() > 0:
            p = np.pad(neutral, 1, mode="edge")
            sm = (p[:-2] + p[1:-1] + p[2:]) / 3.0
            self.common[NCH:] = sm / neutral.sum() >= NEUTRAL_P
        elif not self.use_neutral:
            self.common[NCH:] = True

    def anomaly_masks(self, hsv):
        rare = ~self.common[bin_index(hsv)]
        ch = chroma_mask(hsv)
        return ((ch & rare).astype(np.uint8) * 255, (~ch & rare).astype(np.uint8) * 255)


class Tracker:
    def __init__(self):
        self.tracks = []  # dict(c, hits, pinged, info)

    def update(self, blobs):
        for t in self.tracks:
            t["seen"] = False
        for b in blobs:
            best, bd = None, MATCH_DIST
            for t in self.tracks:
                d = np.hypot(t["c"][0] - b["c"][0], t["c"][1] - b["c"][1])
                if not t["seen"] and d < bd:
                    best, bd = t, d
            if best is None:
                best = dict(hits=0, pinged=False)
                self.tracks.append(best)
            best.update(c=b["c"], info=b, seen=True, hits=min(best["hits"] + 1, 12))
        new = []
        for t in list(self.tracks):
            if not t["seen"]:
                t["hits"] -= 1
                if t["hits"] <= 0:
                    self.tracks.remove(t)
                    continue
            if t["hits"] >= CONFIRM_HITS and not t["pinged"]:
                t["pinged"] = True
                new.append(t)
        return new

    def confirmed(self):
        return [t for t in self.tracks if t["hits"] >= CONFIRM_HITS and t.get("seen")]

    def candidates(self):
        return [t for t in self.tracks if t["hits"] < CONFIRM_HITS and t.get("seen")]


def neutral_name(v):
    return "black" if v < 60 else "white" if v > 190 else "gray"


def find_blobs(mask, hsv, min_area=MIN_AREA, neutral=False):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
    blobs = []
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_area:
            continue
        sel = lab == i
        h = int(np.median(hsv[..., 0][sel]))
        name = neutral_name(int(np.median(hsv[..., 2][sel]))) if neutral else hue_name(h)
        s = float(hsv[..., 1][sel].mean())
        x, y, w, hh = (int(stats[i, k]) for k in (0, 1, 2, 3))
        blobs.append(dict(
            c=(float(cent[i][0]), float(cent[i][1])), area=int(area), bbox=(x, y, w, hh),
            color=name,
            conf=round(min(1.0, 0.5 * (0.5 if neutral else s / 255) + 0.5 * min(1.0, area / (0.01 * SMALL[0] * SMALL[1]))), 2),
        ))
    return mask, blobs


# ------------------------------------------------------------------- sources
class Grabber(threading.Thread):
    """Reads a source in a thread and always keeps only the newest frame."""

    def __init__(self):
        super().__init__(daemon=True)
        self.frame, self.lock, self.alive = None, threading.Lock(), True

    def latest(self):
        with self.lock:
            f, self.frame = self.frame, None
        return f

    def push(self, f):
        with self.lock:
            self.frame = f


class TcpSource(Grabber):
    def __init__(self, host, user, port, w, h, fps, start_remote=True):
        super().__init__()
        self.key = Path.home() / ".ssh" / "codemetal_ed25519"
        self.remote = None
        self.target = (user, host)
        if start_remote:
            cmd = ("pkill rpicam-vid; exec rpicam-vid -n -t 0 --codec mjpeg "
                   f"--width {w} --height {h} --framerate {fps} --autofocus-mode continuous "
                   f"--listen -o tcp://0.0.0.0:{port}")
            self.remote = subprocess.Popen(
                ["ssh", "-i", str(self.key), "-o", "BatchMode=yes", f"{user}@{host}", cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.cap = None
        for _ in range(20):
            time.sleep(1)
            self.cap = cv2.VideoCapture(f"tcp://{host}:{port}", cv2.CAP_FFMPEG)
            if self.cap.isOpened():
                break
            self.cap.release()
        else:
            self.close()
            sys.exit("Could not connect to the Pi stream")

    def run(self):
        while self.alive:
            ok, f = self.cap.read()
            if not ok:
                break
            self.push(f)
        self.alive = False

    def close(self):
        self.alive = False
        if self.cap:
            self.cap.release()
        if self.remote:
            self.remote.terminate()
            subprocess.run(["ssh", "-i", str(self.key), "-o", "BatchMode=yes",
                            f"{self.target[0]}@{self.target[1]}", "pkill rpicam-vid"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class LocalSource(Grabber):
    """On the Pi: rpicam-vid MJPEG on stdout, split into JPEGs."""

    def __init__(self, w, h, fps):
        super().__init__()
        self.p = subprocess.Popen(
            ["rpicam-vid", "-n", "-t", "0", "--codec", "mjpeg", "--width", str(w),
             "--height", str(h), "--framerate", str(fps), "--autofocus-mode", "continuous",
             "-o", "-"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)

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


# ------------------------------------------------------------- live HTTP feed
class Feed:
    jpg = None
    lock = threading.Lock()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with Feed.lock:
                    j = Feed.jpg
                if j is not None:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + j + b"\r\n")
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


def serve(port):
    srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()


def ping():
    try:
        import winsound
        winsound.Beep(1200, 150)
    except Exception:
        sys.stdout.write("\a")
        sys.stdout.flush()


# ---------------------------------------------------------------- main logic
class Detector:
    def __init__(self, learn_secs, log_path, use_neutral=True):
        self.learn_secs, self.log_path = learn_secs, log_path
        self.base, self.tracker = Baseline(use_neutral), Tracker()
        self.start_learning()

    def start_learning(self):
        self.base.reset()
        self.tracker = Tracker()
        self.learning_until = time.time() + self.learn_secs
        self.learning = True

    def process(self, frame):
        """Returns (annotated frame, mask or None, list of new hotspot events)."""
        small = cv2.resize(frame, SMALL, interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2HSV)
        out, mask, events = frame.copy(), None, []
        sx, sy = frame.shape[1] / SMALL[0], frame.shape[0] / SMALL[1]
        if self.learning:
            self.base.learn(hsv)
            left = self.learning_until - time.time()
            if left <= 0:
                self.base.finish()
                self.learning = False
            else:
                self.banner(out, f"LEARNING BASELINE {left:.0f}s - pan the camera around the room", (0, 200, 255))
                return out, None, events
        cm, nm = self.base.anomaly_masks(hsv)
        mask, blobs = find_blobs(cm, hsv)
        if self.base.use_neutral:
            nmask, nblobs = find_blobs(nm, hsv, MIN_AREA_NEUTRAL, neutral=True)
            mask, blobs = cv2.bitwise_or(mask, nmask), blobs + nblobs
        for t in self.tracker.update(blobs):
            i = t["info"]
            x, y, w, h = i["bbox"]
            ev = dict(t=round(time.time(), 3), cls="unnatural-color", color=i["color"],
                      confidence=i["conf"], bbox=[int(x * sx), int(y * sy), int(w * sx), int(h * sy)])
            events.append(ev)
            with open(self.log_path, "a") as f:
                f.write(json.dumps({"type": "detection", **ev}) + "\n")
        for t in self.tracker.candidates():
            cx, cy = int(t["c"][0] * sx), int(t["c"][1] * sy)
            cv2.circle(out, (cx, cy), 14, (0, 255, 255), 1)
        for t in self.tracker.confirmed():
            i = t["info"]
            x, y, w, h = i["bbox"]
            cx, cy = int((x + w / 2) * sx), int((y + h / 2) * sy)
            r = int(max(w * sx, h * sy) / 2 + 14)
            cv2.circle(out, (cx, cy), r, (0, 0, 255), 3)
            cv2.putText(out, f"HOTSPOT {i['color']} {i['conf']:.2f}", (max(cx - r, 5), max(cy - r - 8, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        n = len(self.tracker.confirmed())
        self.banner(out, f"WATCHING  hotspots: {n}", (0, 0, 255) if n else (0, 200, 0))
        return out, mask, events

    @staticmethod
    def banner(img, text, color):
        cv2.rectangle(img, (0, 0), (img.shape[1], 32), (0, 0, 0), -1)
        cv2.putText(img, text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)


def selftest():
    """Beige office with wood + blue-ish carpet baseline, then an orange card appears."""
    rng = np.random.default_rng(1)

    def scene(card, black=False):
        img = np.full((720, 1280, 3), (190, 200, 205), np.uint8)       # beige wall (BGR)
        img[400:, :] = (60, 80, 120)                                    # brownish floor
        img[100:300, 800:1100] = (150, 110, 70)                         # blueish monitor
        img = np.clip(img + rng.normal(0, 6, img.shape), 0, 255).astype(np.uint8)
        if card:
            img[200:260, 300:380] = (0, 120, 255)                       # orange card (BGR)
        if black:
            img[150:290, 600:720] = (12, 12, 12)                        # black object
        return img

    d = Detector(learn_secs=0.0, log_path=os.devnull)
    for _ in range(15):
        d.base.learn(cv2.cvtColor(cv2.resize(scene(False), SMALL), cv2.COLOR_BGR2HSV))
    d.base.finish()
    d.learning = False
    fp = sum(len(d.process(scene(False))[2]) for _ in range(20))
    ev = [e for _ in range(10) for e in d.process(scene(True))[2]]
    print("false pings on normal scene:", fp)
    print("pings with orange card:", len(ev), ev[:1])
    d.tracker = Tracker()
    nv = [e for _ in range(10) for e in d.process(scene(False, black=True))[2]]
    print("pings with black object:", len(nv), nv[:1])
    ok = (fp == 0 and len(ev) == 1 and ev[0]["color"] == "orange"
          and len(nv) == 1 and nv[0]["color"] == "black")
    print("SELFTEST", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=["tcp", "local"], default="tcp")
    ap.add_argument("--host", default=os.environ.get("PI_HOST"))
    ap.add_argument("--user", default="hacker")
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--rotate", type=int, default=90, choices=[0, 90, 180, 270])
    ap.add_argument("--learn", type=float, default=10.0, help="baseline seconds")
    ap.add_argument("--no-ssh", action="store_true", help="tcp: Pi stream already running")
    ap.add_argument("--no-neutral", action="store_true",
                    help="ignore gray/white/black (default: they are learned as baseline too)")
    ap.add_argument("--headless", action="store_true", help="no window")
    ap.add_argument("--serve", type=int, default=0, help="serve annotated MJPEG on this port")
    ap.add_argument("--log", default="detections.jsonl")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest()

    if a.source == "local":
        src = LocalSource(a.width, a.height, a.fps)
    else:
        if not a.host:
            sys.exit("Set PI_HOST or pass --host")
        src = TcpSource(a.host, a.user, a.port, a.width, a.height, a.fps, not a.no_ssh)
    src.start()
    rot = {0: None, 90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
           270: cv2.ROTATE_90_COUNTERCLOCKWISE}[a.rotate]
    if a.serve:
        serve(a.serve)
        print(f"Live feed: http://<this host>:{a.serve}/")

    det = Detector(a.learn, a.log, not a.no_neutral)
    relearn_flag = Path.home() / "relearn"
    show_mask, n, t0 = False, 0, time.time()
    try:
        while src.alive:
            f = src.latest()
            if f is None:
                time.sleep(0.005)
                continue
            if rot is not None:
                f = cv2.rotate(f, rot)
            if relearn_flag.exists():
                relearn_flag.unlink()
                det.start_learning()
            out, mask, events = det.process(f)
            for e in events:
                print(f"HOTSPOT {e['color']} conf={e['confidence']} bbox={e['bbox']}")
                ping()
            n += 1
            cv2.putText(out, f"{n / (time.time() - t0):.0f} fps", (out.shape[1] - 90, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            if a.serve:
                ok, j = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    with Feed.lock:
                        Feed.jpg = j.tobytes()
            if not a.headless:
                cv2.imshow("Color anomaly (b relearn, m mask, s save, q quit)", out)
                if show_mask and mask is not None:
                    cv2.imshow("mask", cv2.resize(mask, (640, 360), interpolation=cv2.INTER_NEAREST))
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                elif k == ord("b"):
                    det.start_learning()
                elif k == ord("m"):
                    show_mask = not show_mask
                    if not show_mask:
                        cv2.destroyWindow("mask")
                elif k == ord("s"):
                    Path("data/picam").mkdir(parents=True, exist_ok=True)
                    p = f"data/picam/anom_{datetime.now():%H%M%S}.jpg"
                    cv2.imwrite(p, out)
                    print("Saved", p)
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
