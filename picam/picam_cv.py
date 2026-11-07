"""Pi Camera 3 through cv2.VideoCapture (GStreamer + libcamera). Run on the Pi.

Needs: sudo apt install -y python3-opencv gstreamer1.0-libcamera gstreamer1.0-plugins-base gstreamer1.0-tools
Press q in the window to quit; with no display (SSH) it saves picam_cv.jpg and exits.
"""
import os
import time

import cv2

W, H, FPS = 1280, 720, 30

PIPELINE = (
    f"libcamerasrc ! video/x-raw,width={W},height={H},framerate={FPS}/1 "
    "! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1"
)

cap = cv2.VideoCapture(PIPELINE, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("GStreamer pipeline failed. OpenCV GStreamer support:")
    print([l for l in cv2.getBuildInformation().splitlines() if "GStreamer" in l])
    raise SystemExit(
        "Install gstreamer1.0-libcamera, or check `rpicam-hello --list-cameras`"
    )

headless = not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY")
n, t0 = 0, time.time()
while True:
    ok, frame = cap.read()
    if not ok:
        print("Read failed")
        break
    n += 1
    if headless:
        if n >= 30:
            cv2.imwrite("picam_cv.jpg", frame)
            print("Frame:", frame.shape, f"~{n / (time.time() - t0):.1f} fps, saved picam_cv.jpg")
            break
    else:
        cv2.imshow("Pi Camera 3", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cap.release()
cv2.destroyAllWindows()
