 
import cv2
import csv
import numpy as np
from pathlib import Path

DATA = Path(__file__).resolve().parent / "data"
FEATURES = [
    "mean", "std", "foreground_coverage", "component_count",
    "largest_component_fraction", "edge_density",
]
INPUT = DATA / "metadata.csv"
OUTPUT = DATA / "scene_features.csv"

def extract_scene_features(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    _, mask = cv2.threshold(
        blurred, 0, 255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (3, 3)
    )

    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, kernel
    )
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE, kernel
    )

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    significant = [
        c for c in contours
        if cv2.contourArea(c) >= 20
    ]

    coverage = cv2.countNonZero(mask) / mask.size

    largest_area = (
        max(cv2.contourArea(c) for c in significant)
        if significant else 0
    )

    edges = cv2.Canny(gray, 50, 150)
    edge_density = cv2.countNonZero(edges) / edges.size

    return {
        "mean": float(gray.mean()),
        "std": float(gray.std()),
        "foreground_coverage": float(coverage),
        "component_count": len(significant),
        "largest_component_fraction": largest_area / mask.size,
        "edge_density": float(edge_density)
    }

def main():
    with open(INPUT, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    output_rows = []

    for row in rows:
        path = DATA.parent / Path(row["image_path"].replace(chr(92), "/"))
        image = cv2.imread(str(path))

        if image is None:
            print("Skipping missing image:", path)
            continue

        features = extract_scene_features(image)

        output_rows.append({
            **row,
            **features
        })

    if not output_rows:
        raise RuntimeError("No valid images processed")

    with open(OUTPUT, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f, fieldnames=output_rows[0].keys()
        )
        writer.writeheader()
        writer.writerows(output_rows)

    print("Processed images:", len(output_rows))
    print("Saved:", OUTPUT)


if __name__ == "__main__":
    main()
