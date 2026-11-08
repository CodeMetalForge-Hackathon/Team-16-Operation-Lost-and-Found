"""A fixed flight line across the synthetic hill. Heading follows the line."""

from __future__ import annotations

import math

import numpy as np

# Kept inside the map so a 250 m lidar footprint still hits terrain.
_CONTROLS = np.array(
    [
        [400.0, 500.0],
        [700.0, 1100.0],
        [1100.0, 1500.0],
        [1500.0, 1150.0],
        [1650.0, 700.0],
        [1150.0, 450.0],
    ],
    dtype=np.float64,
)


def demo_poses(samples: int = 8) -> list[tuple[float, float, float]]:
    if samples < 2:
        raise ValueError("need at least two samples")
    segment = np.linalg.norm(np.diff(_CONTROLS, axis=0), axis=1)
    length = np.concatenate(([0.0], np.cumsum(segment)))
    targets = np.linspace(0.0, length[-1], samples)
    poses: list[tuple[float, float, float]] = []
    for distance in targets:
        index = int(np.searchsorted(length, distance, side="right") - 1)
        index = min(max(index, 0), len(segment) - 1)
        span = segment[index]
        fraction = 0.0 if span == 0.0 else (distance - length[index]) / span
        start = _CONTROLS[index]
        end = _CONTROLS[index + 1]
        point = start + fraction * (end - start)
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        poses.append((float(point[0]), float(point[1]), yaw))
    return poses
