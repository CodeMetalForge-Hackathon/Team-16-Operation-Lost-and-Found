"""Thermal person classifier on the Raspberry Pi: CPU (RandomForest) vs Hailo-8, with live feed.

Backends (both give a 0..1 "human likelihood" per frame):
  cpu    your RandomForest on 6 hand-built scene features (thermal/data/thermal_model.joblib)
  hailo  pretrained YOLOv8s person detector (.hef) on the Hailo-8; score = best 'person' box

Run ON the Pi (camera = USB thermal HT-203U). From the folder that contains data/ and models/:
  python3 thermal_pi.py --bench-images                 speed + accuracy on the 74 labeled images
  python3 thermal_pi.py --bench-camera 200             speed on live camera frames
  python3 thermal_pi.py --serve 8081 --backend both    live annotated feed, open http://<pi>:8081/

Hailo needs PYTHONPATH to include the built hailo_platform (see setup notes in README).
"""
import argparse
import csv
import glob
import json
import os
import statistics
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

from scene_features import DATA, FEATURES, extract_scene_features
from thermal_viewer import draw_hotspots

HEF_DEFAULT = Path.home() / "team16_thermal" / "models" / "yolov8s.hef"
SIZE = 640  # YOLOv8s input


def find_camera():
    hits = sorted(glob.glob("/dev/v4l/by-id/usb-HIK_HikCamera_*-video-index0"))
    return os.path.realpath(hits[0]) if hits else "/dev/video8"


def open_camera(dev):
    pipe = (f"v4l2src device={dev} ! video/x-raw,format=YUY2,width=256,height=392,framerate=25/1 "
            "! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false")
    cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        sys.exit(f"Could not open thermal camera {dev}")
    return cap


def thermal_crop(frame):
    return frame[196:388]  # same crop as thermal_viewer.py / collect_data.py


# ----------------------------------------------------------------- backends
class CpuBackend:
    name = "cpu"

    def __init__(self):
        import joblib
        import pandas as pd
        art = joblib.load(DATA / "thermal_model.joblib")
        self.model, self.pd = art["model"], pd
        self.human = list(self.model.classes_).index(1)
        self.info = {k: art.get(k) for k in ("held_out_accuracy", "held_out_balanced_accuracy", "held_out_roc_auc")}

    def run(self, img):
        t0 = time.perf_counter()
        row = self.pd.DataFrame([extract_scene_features(img)], columns=FEATURES)
        t1 = time.perf_counter()
        p = float(self.model.predict_proba(row)[0][self.human])
        t2 = time.perf_counter()
        return p, {"pre_ms": (t1 - t0) * 1e3, "infer_ms": (t2 - t1) * 1e3, "post_ms": 0.0}, []

    def close(self):
        pass


class HailoBackend:
    name = "hailo"

    def __init__(self, hef_path=HEF_DEFAULT, stretch=False):
        import hailo_platform as hp
        self.hp, self.stretch = hp, stretch
        self.hef = hp.HEF(str(hef_path))
        self.vdev = hp.VDevice()
        cfg = hp.ConfigureParams.create_from_hef(self.hef, interface=hp.HailoStreamInterface.PCIe)
        self.ng = self.vdev.configure(self.hef, cfg)[0]
        self.ng_params = self.ng.create_params()
        self.in_name = self.hef.get_input_vstream_infos()[0].name
        self.out_name = self.hef.get_output_vstream_infos()[0].name
        try:
            in_p = hp.InputVStreamParams.make_from_network_group(self.ng, quantized=False, format_type=hp.FormatType.UINT8)
            out_p = hp.OutputVStreamParams.make_from_network_group(self.ng, quantized=False, format_type=hp.FormatType.FLOAT32)
        except TypeError:
            in_p = hp.InputVStreamParams.make_from_network_group(self.ng, format_type=hp.FormatType.UINT8)
            out_p = hp.OutputVStreamParams.make_from_network_group(self.ng, format_type=hp.FormatType.FLOAT32)
        self.pipe = hp.InferVStreams(self.ng, in_p, out_p)
        self.pipe.__enter__()
        self.act = self.ng.activate(self.ng_params)
        self.act.__enter__()
        self.info = {"hef": Path(hef_path).name}

    def prep(self, img):
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self.stretch:  # raw thermal is low contrast; stretch so a person is bright against the scene
            lo, hi = np.percentile(g, (1, 99))
            g = np.clip((g.astype(np.float32) - lo) * 255.0 / max(hi - lo, 1.0), 0, 255).astype(np.uint8)
        h, w = g.shape
        s = min(SIZE / w, SIZE / h)
        nw, nh = int(round(w * s)), int(round(h * s))
        small = cv2.resize(g, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((SIZE, SIZE), 114, np.uint8)
        top, left = (SIZE - nh) // 2, (SIZE - nw) // 2
        canvas[top:top + nh, left:left + nw] = small
        self._lb = (s, top, left, h, w)
        return np.repeat(canvas[:, :, None], 3, axis=2)

    def run(self, img):
        t0 = time.perf_counter()
        x = self.prep(img)
        t1 = time.perf_counter()
        res = self.pipe.infer({self.in_name: x[None]})
        t2 = time.perf_counter()
        out = res[self.out_name]
        persons = np.asarray(out[0][0]) if len(out[0]) else np.zeros((0, 5))  # class 0 = person
        s, top, left, h, w = self._lb
        boxes, score = [], 0.0
        for ymin, xmin, ymax, xmax, sc in persons:
            if sc <= 0:
                continue
            score = max(score, float(sc))
            boxes.append((float(sc),
                          int(np.clip((xmin * SIZE - left) / s, 0, w)), int(np.clip((ymin * SIZE - top) / s, 0, h)),
                          int(np.clip((xmax * SIZE - left) / s, 0, w)), int(np.clip((ymax * SIZE - top) / s, 0, h))))
        t3 = time.perf_counter()
        return score, {"pre_ms": (t1 - t0) * 1e3, "infer_ms": (t2 - t1) * 1e3, "post_ms": (t3 - t2) * 1e3}, boxes

    def close(self):
        for c in (self.act, self.pipe):
            try:
                c.__exit__(None, None, None)
            except Exception:
                pass


def make_backends(which, args):
    out = []
    if which in ("cpu", "both"):
        out.append(CpuBackend())
    if which in ("hailo", "both"):
        out.append(HailoBackend(args.hef, stretch=args.stretch))
    return out


# ---------------------------------------------------------------- benchmarks
def summarize(name, rows):
    tot = [r["pre_ms"] + r["infer_ms"] + r["post_ms"] for r in rows]
    srt = sorted(tot)
    p95 = srt[min(len(srt) - 1, int(0.95 * len(srt)))]
    return {"backend": name, "frames": len(rows),
            "pre_ms": round(statistics.mean(r["pre_ms"] for r in rows), 2),
            "infer_ms": round(statistics.mean(r["infer_ms"] for r in rows), 2),
            "post_ms": round(statistics.mean(r["post_ms"] for r in rows), 2),
            "total_ms_mean": round(statistics.mean(tot), 2), "total_ms_p95": round(p95, 2),
            "max_fps": round(1000.0 / statistics.mean(tot), 1)}


def bench_images(backends, args):
    from sklearn.metrics import accuracy_score, balanced_accuracy_score, roc_auc_score
    imgs, labels = [], []
    with open(DATA / "metadata.csv", newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            p = DATA.parent / Path(r["image_path"].replace("\\", "/"))
            im = cv2.imread(str(p))
            if im is not None and r["subject_type"] in ("human", "nonhuman", "empty"):
                imgs.append(im)
                labels.append(1 if r["subject_type"] == "human" else 0)
    print(f"{len(imgs)} labeled images ({sum(labels)} human)")
    y, results = np.array(labels), []
    for b in backends:
        for im in imgs[:5]:
            b.run(im)  # warm-up
        probs, rows = [], []
        for im in imgs:
            p, t, _ = b.run(im)
            probs.append(p)
            rows.append(t)
        probs = np.array(probs)
        res = summarize(b.name, rows)
        res["auc"] = round(float(roc_auc_score(y, probs)), 3)
        best = max(((accuracy_score(y, probs >= th), th) for th in np.linspace(0.05, 0.95, 19)), key=lambda x: x[0])
        res["acc_at_0.5"] = round(float(accuracy_score(y, probs >= 0.5)), 3)
        res["bal_acc_at_0.5"] = round(float(balanced_accuracy_score(y, probs >= 0.5)), 3)
        res["best_acc"] = f"{best[0]:.3f}@{best[1]:.2f}"
        if b.name == "cpu":
            res["note"] = (f"model was trained on these images, so its numbers here are optimistic; "
                           f"held-out-session accuracy = {b.info.get('held_out_accuracy'):.3f}, "
                           f"AUC = {b.info.get('held_out_roc_auc'):.3f}")
        else:
            res["note"] = "pretrained, never saw these images, so all 74 count as held-out"
        results.append(res)
    return results


def bench_camera(backends, n, args):
    cap = open_camera(args.device or find_camera())
    frames = []
    while len(frames) < n + 5:
        ok, f = cap.read()
        if ok:
            frames.append(thermal_crop(f))
    cap.release()
    results = []
    for b in backends:
        for im in frames[:5]:
            b.run(im)
        rows = [b.run(im)[1] for im in frames[5:]]
        results.append(summarize(b.name, rows))
    return results


def print_table(results):
    print()
    for r in results:
        print(json.dumps(r, indent=2))
    print("\nbackend  total_ms(mean)  p95_ms   max_fps   infer_ms   pre_ms")
    for r in results:
        print(f"{r['backend']:7s}  {r['total_ms_mean']:12.2f}  {r['total_ms_p95']:7.2f}  {r['max_fps']:8.1f}  "
              f"{r['infer_ms']:9.2f}  {r['pre_ms']:7.2f}")


# ------------------------------------------------------------------ live feed
class Feed:
    jpg, lock = None, threading.Lock()


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


def serve(backends, args):
    srv = ThreadingHTTPServer(("0.0.0.0", args.serve), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"Live feed: http://<pi address>:{args.serve}/   (Ctrl+C to stop)")
    cap = open_camera(args.device or find_camera())
    smooth = {b.name: None for b in backends}
    t_last, fps = time.time(), 0.0
    try:
        while True:
            ok, f = cap.read()
            if not ok:
                break
            img = thermal_crop(f)
            shown, count = draw_hotspots(img)
            panel = cv2.copyMakeBorder(shown, 0, 28 * (len(backends) + 1) + 6, 0, 0, cv2.BORDER_CONSTANT)
            y = shown.shape[0] + 22
            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - t_last, 1e-6)
            t_last = now
            cv2.putText(panel, f"hotspots: {count}   feed {fps:.0f} fps", (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            for b in backends:
                p, t, boxes = b.run(img)
                smooth[b.name] = p if smooth[b.name] is None else 0.7 * smooth[b.name] + 0.3 * p
                y += 28
                ms = t["pre_ms"] + t["infer_ms"] + t["post_ms"]
                col = (0, 220, 0) if smooth[b.name] >= 0.5 else (0, 190, 255)
                cv2.putText(panel, f"{b.name.upper():5s} human {smooth[b.name]:.0%}  {ms:.1f} ms", (8, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.62, col, 2)
                for sc, x1, y1, x2, y2 in boxes:
                    if sc >= 0.3:
                        cv2.rectangle(panel, (x1 * 3, y1 * 3), (x2 * 3, y2 * 3), (0, 255, 0), 2)
                        cv2.putText(panel, f"person {sc:.2f}", (x1 * 3, max(y1 * 3 - 5, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            ok2, j = cv2.imencode(".jpg", panel, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok2:
                with Feed.lock:
                    Feed.jpg = j.tobytes()
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["cpu", "hailo", "both"], default="both")
    ap.add_argument("--hef", default=str(HEF_DEFAULT))
    ap.add_argument("--stretch", action="store_true", help="hailo: contrast-stretch the input (measured worse on our images)")
    ap.add_argument("--device", help="thermal camera node (default: auto-detect)")
    ap.add_argument("--bench-images", action="store_true")
    ap.add_argument("--bench-camera", type=int, metavar="N")
    ap.add_argument("--serve", type=int, metavar="PORT")
    ap.add_argument("--out", default="bench_results.json")
    args = ap.parse_args()
    if not (args.bench_images or args.bench_camera or args.serve):
        ap.print_help()
        return
    backends = make_backends(args.backend, args)
    try:
        if args.bench_images:
            res = bench_images(backends, args)
            print_table(res)
            Path(args.out).write_text(json.dumps(res, indent=2))
        if args.bench_camera:
            res = bench_camera(backends, args.bench_camera, args)
            print_table(res)
        if args.serve:
            serve(backends, args)
    finally:
        for b in backends:
            b.close()


if __name__ == "__main__":
    main()
