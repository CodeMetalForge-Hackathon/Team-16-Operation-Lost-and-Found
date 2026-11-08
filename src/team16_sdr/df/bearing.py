"""Heading marks from a directional-antenna sweep.

Power is relative. The peak heading is the bearing. Two marks only say
which of those two headings is stronger.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass


def angdiff(a_deg: float, b_deg: float) -> float:
    """Signed shortest difference a - b, in degrees, wrapped to [-180, 180)."""
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0


def wrap360(deg: float) -> float:
    return deg % 360.0


@dataclass(frozen=True)
class Bearing:
    deg: float
    method: str
    contrast_db: float


class RecentPeak:
    """Highest power seen inside a short window.

    A LoRa packet is a burst. The mark button stores this peak, so the
    operator can hold the antenna still for a moment and still catch one packet.
    """

    def __init__(self, window_s: float = 1.4) -> None:
        self.window_s = window_s
        self._hist: deque[tuple[float, float]] = deque()

    def push(self, t: float, power_db: float) -> float:
        self._hist.append((t, power_db))
        cutoff = t - self.window_s
        while self._hist and self._hist[0][0] < cutoff:
            self._hist.popleft()
        return self.value

    @property
    def value(self) -> float | None:
        if not self._hist:
            return None
        return max(power for _, power in self._hist)

    def clear(self) -> None:
        self._hist.clear()


def add_sample(samples: list[tuple[float, float]], deg: float, power_db: float) -> None:
    """Record one aim. A second mark near the same heading replaces the first."""
    deg = wrap360(deg)
    for i, (have, _) in enumerate(samples):
        if abs(angdiff(have, deg)) < 10.0:
            samples[i] = (deg, power_db)
            return
    samples.append((deg, power_db))


def _parabola_vertex(
    x0: float, y0: float, x1: float, y1: float, x2: float, y2: float
) -> float | None:
    denom = (x0 - x1) * (x0 - x2) * (x1 - x2)
    if denom == 0.0:
        return None
    a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / denom
    b = (x2 * x2 * (y0 - y1) + x1 * x1 * (y2 - y0) + x0 * x0 * (y1 - y2)) / denom
    if a == 0.0:
        return None
    vertex = -b / (2.0 * a)
    lo, hi = min(x0, x2), max(x0, x2)
    if vertex < lo or vertex > hi or a >= 0.0:
        return None
    return vertex


def refine_peak(samples: list[tuple[float, float]]) -> float:
    """Heading of the strongest mark, nudged between neighbors when it is safe."""
    origin, _ = max(samples, key=lambda sample: sample[1])
    left = None
    right = None
    for deg, power in samples:
        delta = angdiff(deg, origin)
        if -50.0 <= delta < -1.0 and (left is None or delta > left[0]):
            left = (delta, power)
        elif 1.0 < delta <= 50.0 and (right is None or delta < right[0]):
            right = (delta, power)
    if left is None or right is None:
        return wrap360(origin)
    peak_power = max(power for deg, power in samples if abs(angdiff(deg, origin)) < 1.0)
    vertex = _parabola_vertex(left[0], left[1], 0.0, peak_power, right[0], right[1])
    if vertex is None:
        return wrap360(origin)
    return wrap360(origin + vertex)


def bearing_report(samples: list[tuple[float, float]]) -> Bearing | None:
    if len(samples) < 2:
        return None
    powers = [power for _, power in samples]
    contrast = max(powers) - min(powers)
    if contrast < 1.0:
        return None
    if len(samples) == 2:
        (d1, p1), (d2, p2) = samples
        deg = d1 if p1 >= p2 else d2
        return Bearing(wrap360(deg), "two-sample", contrast)
    return Bearing(refine_peak(samples), "peak", contrast)
