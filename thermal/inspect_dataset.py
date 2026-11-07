
import cv2
import csv
import random
from pathlib import Path
from collections import defaultdict

DATA = Path("data")
METADATA = DATA / "metadata.csv"
OUTPUT = DATA / "segmentation_review.jpg"

SAMPLES_PER_GROUP = 3
CELL_W = 256
CELL_H = 240

# Read metadata
with open(METADATA, newline="", encoding="utf-8") as f:
    rows = list(csv.DictReader(f))

# Group by subject and occlusion
groups = defaultdict(list)

for row in rows:
    key = (
        row["subject_type"],
        row["occlusion_level"]
    )
    groups[key].append(row)

# Select representative examples
random.seed(42)
selected = []

for key, images in sorted(groups.items()):
    chosen = random.sample(
        images,
        min(SAMPLES_PER_GROUP, len(images))
    )
    selected.extend(chosen)

print(f"Reviewing {len(selected)} of {len(rows)} images")

# Make a side-by-side review:
# Original | Otsu Mask
panels = []

for row in selected:
    path = Path(row["image_path"])

    if not path.exists():
        path = Path(row["image_path"].replace("\\", "/"))

    image = cv2.imread(str(path))

    if image is None:
        print("Could not read:", path)
        continue

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(
        gray, (5, 5), 0
    )

    threshold, mask = cv2.threshold(
        blurred,
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    mask_color = cv2.cvtColor(
        mask,
        cv2.COLOR_GRAY2BGR
    )

    original = cv2.resize(image, (CELL_W, 192))
    segmented = cv2.resize(mask_color, (CELL_W, 192))

    # Add label space above both images
    pair = cv2.hconcat([original, segmented])
    panel = cv2.copyMakeBorder(
        pair, 48, 0, 0, 0,
        cv2.BORDER_CONSTANT,
        value=(35, 35, 35)
    )

    label = (
        f'{row["subject_type"]} | '
        f'{row["occlusion_level"]} | '
        f'Session {row["session_id"]}'
    )

    cv2.putText(
        panel,
        label,
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 255, 255),
        1
    )

    cv2.putText(
        panel,
        f"Otsu: {threshold:.0f}",
        (CELL_W + 8, 38),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 255, 255),
        1
    )

    panels.append(panel)

if not panels:
    raise RuntimeError("No valid images found")

contact_sheet = cv2.vconcat(panels)

cv2.imwrite(str(OUTPUT), contact_sheet)

print("Saved contact sheet:", OUTPUT)
print("Rows:", len(panels))
