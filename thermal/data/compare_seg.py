
import cv2
import csv
import random
import numpy as np
from pathlib import Path
from collections import defaultdict

DATA = Path("data")
OUTPUT = DATA / "segmentation_comparison.jpg"

# Load existing metadata
with open(DATA / "metadata.csv", newline="") as f:
    rows = list(csv.DictReader(f))

# Select 3 images per label combination
groups = defaultdict(list)

for row in rows:
    key = (row["subject_type"], row["occlusion_level"])
    groups[key].append(row)

random.seed(42)
selected = []

for key, images in sorted(groups.items()):
    selected.extend(
        random.sample(images, min(3, len(images)))
    )

panels = []

for row in selected:
    image = cv2.imread(row["image_path"])

    if image is None:
        print("Missing:", row["image_path"])
        continue

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Method 1: Otsu
    _, otsu = cv2.threshold(
        blurred, 0, 255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    # Method 2: Adaptive threshold
    adaptive = cv2.adaptiveThreshold(
        blurred,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        31,
        -5
    )

    # Method 3: Otsu + morphology
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (3, 3)
    )

    cleaned = cv2.morphologyEx(
        otsu, cv2.MORPH_OPEN, kernel
    )

    cleaned = cv2.morphologyEx(
        cleaned, cv2.MORPH_CLOSE, kernel
    )

    images = [
        image,
        cv2.cvtColor(otsu, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(adaptive, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
    ]

    # Resize for contact sheet
    images = [
        cv2.resize(im, (256, 192))
        for im in images
    ]

    panel = cv2.hconcat(images)

    panel = cv2.copyMakeBorder(
        panel, 50, 0, 0, 0,
        cv2.BORDER_CONSTANT,
        value=(35, 35, 35)
    )

    label = (
        f'{row["subject_type"]} | '
        f'{row["occlusion_level"]} | '
        f'{row["session_id"]}'
    )

    cv2.putText(
        panel, label, (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (255, 255, 255), 1
    )

    for x, title in enumerate(
        ["Original", "Otsu", "Adaptive", "Morphology"]
    ):
        cv2.putText(
            panel, title, (x * 256 + 10, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45, (255, 255, 255), 1
        )

    panels.append(panel)

if not panels:
    raise RuntimeError("No images loaded")

comparison = cv2.vconcat(panels)

cv2.imwrite(str(OUTPUT), comparison)

print(f"Compared {len(panels)} images")
print(f"Saved: {OUTPUT}")
