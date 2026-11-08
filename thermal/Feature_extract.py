
import cv2
import numpy as np


def extract_features(contour):
    """
    Extract 10 geometric features from a detected contour.

    Features:
        0: Aspect ratio (height / width)
        1: Extent
        2: Solidity
        3-9: Seven Hu moments

    Returns:
        NumPy array of shape (10,), or None if invalid.
    """

    # Validate contour
    if contour is None or len(contour) < 3:
        return None

    # Calculate contour properties
    area = cv2.contourArea(contour)
    x, y, w, h = cv2.boundingRect(contour)

    if area <= 0 or w <= 0 or h <= 0:
        return None

    # Geometric features
    aspect_ratio = h / w
    extent = area / (w * h)

    # Convex hull and solidity
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)

    solidity = (
        area / hull_area
        if hull_area > 0 else 0.0
    )

    # Hu moments: shape descriptors
    moments = cv2.moments(contour)
    hu = cv2.HuMoments(moments).flatten()

    # Log transformation for numerical stability
    hu = -np.sign(hu) * np.log10(
        np.maximum(np.abs(hu), 1e-12)
    )

    # Combine features
    features = np.array(
        [
            aspect_ratio,
            extent,
            solidity,
            *hu
        ],
        dtype=np.float32
    )

    return features
