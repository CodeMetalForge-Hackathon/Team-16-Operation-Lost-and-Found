"""Smoke test for Raspberry Pi Camera Module 3 using Picamera2. Run on the Pi."""
import time

import cv2
from picamera2 import Picamera2

cams = Picamera2.global_camera_info()
print("Cameras found:", cams)
if not cams:
    raise SystemExit("No camera detected. Check ribbon cable and run: rpicam-hello --list-cameras")

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (1280, 720), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

# Camera Module 3 has autofocus; run one AF cycle before capturing.
try:
    from libcamera import controls
    picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
except Exception as e:
    print("Autofocus control unavailable:", e)

time.sleep(2)

frame = picam2.capture_array()  # RGB888 is BGR-ordered in memory, OpenCV-ready
print("Frame shape:", frame.shape, "dtype:", frame.dtype)
cv2.imwrite("picam_test.jpg", frame)
print("Saved picam_test.jpg")

# Quick FPS check
n, t0 = 30, time.time()
for _ in range(n):
    picam2.capture_array()
print(f"~{n / (time.time() - t0):.1f} fps")

picam2.stop()
picam2.close()
