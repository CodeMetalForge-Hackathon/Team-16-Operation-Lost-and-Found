"""Pitch one RPLidar scan into the base frame.

Positive tilt is a right-handed pitch of the laser frame about Y.
A forward ray moves toward -Z as the angle increases. tilt_sign flips that.
"""

from __future__ import annotations

import math

import numpy as np


def in_dead_sector(angle_deg: np.ndarray, dead_min_deg: float, dead_max_deg: float) -> np.ndarray:
    """True for scan angles that hit the mount.

    When dead_min_deg <= dead_max_deg the sector is that closed interval.
    When dead_min_deg is greater, the sector wraps past ±180°.
    The default pair (180, -180) drops nothing inside a normal scan.
    """
    if dead_min_deg <= dead_max_deg:
        return (angle_deg >= dead_min_deg) & (angle_deg <= dead_max_deg)
    return (angle_deg >= dead_min_deg) | (angle_deg <= dead_max_deg)


def scan_to_points(
    ranges,
    angle_min: float,
    angle_increment: float,
    tilt_deg: float,
    tilt_sign: float = 1.0,
    range_min: float = 0.15,
    range_max: float = 12.0,
    dead_min_deg: float = 180.0,
    dead_max_deg: float = -180.0,
) -> np.ndarray:
    """Return Nx3 points in base_link for one tilted scan."""
    distance = np.asarray(ranges, dtype=np.float64)
    if distance.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    alpha = angle_min + np.arange(distance.size, dtype=np.float64) * angle_increment
    usable = np.isfinite(distance) & (distance >= range_min) & (distance <= range_max)
    usable &= ~in_dead_sector(np.degrees(alpha), dead_min_deg, dead_max_deg)
    distance = distance[usable]
    alpha = alpha[usable]
    beta = math.radians(tilt_deg * tilt_sign)
    forward = np.cos(alpha)
    points = np.empty((distance.size, 3), dtype=np.float64)
    points[:, 0] = distance * forward * math.cos(beta)
    points[:, 1] = distance * np.sin(alpha)
    points[:, 2] = -distance * forward * math.sin(beta)
    return points
