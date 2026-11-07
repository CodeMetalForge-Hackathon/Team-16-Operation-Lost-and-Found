"""View the Pi Camera 3 live on this PC.

Starts rpicam-vid on the Pi over SSH (MJPEG over TCP), then reads it with OpenCV.

  set PI_HOST=<pi address>         (PowerShell: $env:PI_HOST="...")
  python picam_pc_view.py                live window; q quits, s saves a frame to data/picam/
  python picam_pc_view.py --no-ssh       Pi stream already running, just connect

Uses the pod key ~/.ssh/codemetal_ed25519. Frames arrive as BGR numpy arrays in
`frame`, so the HSV unnatural-color detector can be dropped in where marked.
"""
import argparse
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2

ap = argparse.ArgumentParser()
ap.add_argument("--host", default=os.environ.get("PI_HOST"))
ap.add_argument("--user", default="hacker")
ap.add_argument("--port", type=int, default=8888)
ap.add_argument("--width", type=int, default=1280)
ap.add_argument("--height", type=int, default=720)
ap.add_argument("--fps", type=int, default=30)
ap.add_argument("--no-ssh", action="store_true", help="don't start the Pi side")
ap.add_argument("--rotate", type=int, default=90, choices=[0, 90, 180, 270],
                help="clockwise degrees to fix the camera mounting")
args = ap.parse_args()
ROT = {0: None, 90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
       270: cv2.ROTATE_90_COUNTERCLOCKWISE}[args.rotate]
if not args.host:
    sys.exit("Set PI_HOST or pass --host")

key = Path.home() / ".ssh" / "codemetal_ed25519"
remote = None
if not args.no_ssh:
    cmd = (
        "pkill rpicam-vid; "
        f"exec rpicam-vid -n -t 0 --codec mjpeg --width {args.width} --height {args.height} "
        f"--framerate {args.fps} --autofocus-mode continuous --listen -o tcp://0.0.0.0:{args.port}"
    )
    remote = subprocess.Popen(
        ["ssh", "-i", str(key), "-o", "BatchMode=yes", f"{args.user}@{args.host}", cmd],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

url = f"tcp://{args.host}:{args.port}"
cap = None
for _ in range(20):  # wait for the Pi to start listening
    time.sleep(1)
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if cap.isOpened():
        break
    cap.release()
else:
    if remote:
        remote.terminate()
    sys.exit("Could not connect to the Pi stream")

out_dir = Path("data/picam")
n, t0 = 0, time.time()
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Stream ended")
            break
        n += 1
        if ROT is not None:
            frame = cv2.rotate(frame, ROT)
        # --- unnatural-color / detection code goes here, working on `frame` ---
        fps = n / (time.time() - t0)
        shown = frame.copy()
        cv2.putText(shown, f"{fps:.1f} fps", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Pi Camera 3  (q quit, s save)", shown)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("s"):
            out_dir.mkdir(parents=True, exist_ok=True)
            p = out_dir / f"{datetime.now():%H%M%S_%f}.jpg"
            cv2.imwrite(str(p), frame)
            print("Saved", p)
finally:
    cap.release()
    cv2.destroyAllWindows()
    if remote:
        remote.terminate()
        subprocess.run(
            ["ssh", "-i", str(key), "-o", "BatchMode=yes", f"{args.user}@{args.host}", "pkill rpicam-vid"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
