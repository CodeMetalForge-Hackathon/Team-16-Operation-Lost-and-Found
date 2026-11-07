
import cv2
import csv
import time
import uuid
from pathlib import Path
from datetime import datetime

# Dataset directories
DATA = Path("data")
IMAGES = DATA / "images"
IMAGES.mkdir(parents=True, exist_ok=True)

CSV = DATA / "metadata.csv"

# Recording identifiers
session_id = uuid.uuid4().hex[:8]
scene_id = input("Scene ID (e.g. office01): ").strip()

if not scene_id:
    scene_id = "test_scene"

subject = "unknown"
occlusion = "none"
view_id = 1
burst_remaining = 0
next_burst_time = 0.0
BURST_SIZE = 10
BURST_INTERVAL_SECONDS = 0.35

# Create metadata file
fields = [
    "image_path",
    "session_id",
    "scene_id",
    "subject_type",
    "occlusion_level",
    "view_id",
    "image_type",
    "timestamp"
]

if not CSV.exists():
    with open(CSV, "w", newline="") as f:
        csv.writer(f).writerow(fields)

# HT-203U setup
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

cap.set(
    cv2.CAP_PROP_FOURCC,
    cv2.VideoWriter_fourcc(*"YUY2")
)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 392)
cap.set(cv2.CAP_PROP_FPS, 25)

if not cap.isOpened():
    raise RuntimeError("Camera unavailable")

print("1 = Human | 2 = Nonhuman | 3 = Empty")
print("N = No occlusion | P = Partial | H = Heavy")
print("V = Next viewpoint | S = Save one | B = Save 10-frame burst | Q = Quit")


def save_sample(thermal):
    timestamp = datetime.now()
    filename = f"{session_id}_{timestamp:%H%M%S_%f}.png"
    path = IMAGES / filename

    if not cv2.imwrite(str(path), thermal):
        raise RuntimeError(f"Could not save image: {path}")

    with open(CSV, "a", newline="") as f:
        csv.writer(f).writerow([
            str(path),
            session_id,
            scene_id,
            subject,
            occlusion,
            view_id,
            "original",
            timestamp.isoformat()
        ])

    print("Saved:", path)

try:
    while True:
        ret, frame = cap.read()

        if not ret:
            break

        thermal = frame[196:388].copy()

        now = time.monotonic()
        if burst_remaining and now >= next_burst_time:
            save_sample(thermal)
            burst_remaining -= 1
            next_burst_time = now + BURST_INTERVAL_SECONDS

        # Display status without altering saved frame
        display = cv2.resize(
            thermal, (768, 576),
            interpolation=cv2.INTER_NEAREST
        )

        status = (
            f"{subject} | {occlusion} | "
            f"View {view_id} | Burst remaining {burst_remaining}"
        )

        cv2.putText(
            display, status, (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (0, 255, 0), 2
        )

        cv2.imshow("Thermal Dataset Collector", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("1"):
            subject = "human"
        elif key == ord("2"):
            subject = "nonhuman"
        elif key == ord("3"):
            subject = "empty"
        elif key == ord("n"):
            occlusion = "none"
        elif key == ord("p"):
            occlusion = "partial"
        elif key == ord("h"):
            occlusion = "heavy"
        elif key == ord("v"):
            view_id += 1

        elif key == ord("b"):
            if subject == "unknown":
                print("Select a subject label first")
                continue
            burst_remaining = BURST_SIZE
            next_burst_time = 0.0
            print(f"Capturing {BURST_SIZE} {subject} frames. Move slightly during the burst.")

        elif key == ord("s"):
            if subject == "unknown":
                print("Select a subject label first")
                continue
            save_sample(thermal)

        elif key == ord("q"):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
