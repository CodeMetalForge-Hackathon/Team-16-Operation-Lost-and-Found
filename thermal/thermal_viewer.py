"""HT-203U hotspot viewer. Press Q or Escape to quit."""
import argparse
from pathlib import Path

import cv2
import numpy as np

from scene_features import DATA, FEATURES, extract_scene_features


def thermal_preview(frame):
    # Match the BGR preview saved by collect_data.py.
    if frame.shape != (392, 256, 3) or frame.dtype != np.uint8:
        raise ValueError(f"Expected 392x256 BGR from HT-203U, got {frame.shape} {frame.dtype}")
    return frame[196:388]


def draw_hotspots(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    smooth = cv2.GaussianBlur(gray, (5, 5), 0)
    background = float(np.median(smooth))
    # Require contrast above the scene median as well as a bright region.
    cutoff = max(background + 15, float(np.percentile(smooth, 90)) - 8)
    mask = (smooth >= cutoff).astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    shown = cv2.resize(image, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
    count = 0
    for contour in contours:
        if cv2.contourArea(contour) < 30:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        color = (0, 255, 255)
        cv2.rectangle(shown, (3*x, 3*y), (3*(x+w)-1, 3*(y+h)-1), color, 2)
        cv2.putText(shown, "Hotspot", (3*x, max(16, 3*y-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        count += 1
    return shown, count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--camera", type=int, default=1)
    source.add_argument("--image", type=Path, help="Replay a saved thermal preview")
    parser.add_argument(
        "--no-classify",
        action="store_true",
        help="Hide the experimental human-likelihood percentage",
    )
    parser.add_argument("--classify", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()

    classifier = None
    if not args.no_classify:
        try:
            import joblib
            import pandas as pd

            artifact = joblib.load(DATA / "thermal_model.joblib")
            if artifact["features"] != FEATURES:
                raise ValueError("Model features differ. Run train_model.py again.")
            if artifact.get("target") != "human_present":
                raise ValueError("Old three-class model found. Run train_model.py again.")
            classifier = artifact["model"]
        except Exception as exc:
            print(f"Experimental model unavailable. Hotspots still enabled. {exc}")

    cap = None
    smoothed_human_probability = None
    try:
        if args.image is not None:
            image = cv2.imread(str(args.image))
            if image is None:
                raise ValueError(f"Could not read {args.image}")
        else:
            cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
            if not cap.isOpened():
                raise ValueError(f"Could not open HT-203U at camera index {args.camera}")
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUY2"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 392)
            cap.set(cv2.CAP_PROP_FPS, 25)

        while True:
            if cap is not None:
                ok, frame = cap.read()
                if not ok:
                    raise ValueError("Thermal camera stopped delivering frames")
                image = thermal_preview(frame)

            shown, count = draw_hotspots(image)
            status = f"Hotspots: {count} | Relative thermal brightness"
            human_status = "Human likelihood: model unavailable"
            human_color = (160, 160, 160)
            if classifier is not None:
                try:
                    row = pd.DataFrame([extract_scene_features(image)], columns=FEATURES)
                    probabilities = classifier.predict_proba(row)[0]
                    human_index = list(classifier.classes_).index(1)
                    human_probability = float(probabilities[human_index])
                    if smoothed_human_probability is None or cap is None:
                        smoothed_human_probability = human_probability
                    else:
                        smoothed_human_probability = (
                            0.85 * smoothed_human_probability
                            + 0.15 * human_probability
                        )
                    human_status = (
                        "EXPERIMENTAL HUMAN LIKELIHOOD: "
                        f"{smoothed_human_probability:.0%}"
                    )
                    human_color = (
                        (0, 220, 0)
                        if smoothed_human_probability >= 0.50
                        else (0, 190, 255)
                    )
                except Exception as exc:
                    print(f"Experimental prediction disabled. {exc}")
                    classifier = None

            panel = cv2.copyMakeBorder(shown, 0, 62, 0, 0, cv2.BORDER_CONSTANT)
            cv2.putText(panel, status, (8, shown.shape[0] + 21),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
            if not args.no_classify:
                cv2.putText(panel, human_status, (8, shown.shape[0] + 49),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.66, human_color, 2, cv2.LINE_AA)
            cv2.imshow("Thermal hotspots - Q to quit", panel)
            if cv2.waitKey(0 if cap is None else 1) & 0xFF in (ord("q"), 27):
                break
            if cap is None:
                break
    except ValueError as exc:
        parser.exit(1, f"{exc}\n")
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
