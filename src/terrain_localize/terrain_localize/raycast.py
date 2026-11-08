"""Downward 3D lidar rays against a height grid. Points are in the sensor frame."""

from __future__ import annotations

import numpy as np

from terrain_localize.heightmap import HeightMap


def lidar_directions(vertical_samples: int = 16, horizontal_samples: int = 180) -> np.ndarray:
    """Level sensor. Beams look down so they hit terrain from the air."""
    elevation = np.linspace(-1.2, -0.4, vertical_samples)
    azimuth = np.linspace(-np.pi, np.pi, horizontal_samples, endpoint=False)
    elev, az = np.meshgrid(elevation, azimuth, indexing="ij")
    cos_e = np.cos(elev)
    directions = np.stack(
        (cos_e * np.cos(az), cos_e * np.sin(az), np.sin(elev)),
        axis=-1,
    )
    return directions.reshape(-1, 3)


def raycast_cloud(
    heightmap: HeightMap,
    x: float,
    y: float,
    z: float,
    yaw: float,
    directions: np.ndarray | None = None,
    max_range: float = 250.0,
) -> np.ndarray:
    if directions is None:
        directions = lidar_directions()
    cos_y = float(np.cos(yaw))
    sin_y = float(np.sin(yaw))
    rotation = np.array([[cos_y, -sin_y, 0.0], [sin_y, cos_y, 0.0], [0.0, 0.0, 1.0]])
    world_dirs = directions @ rotation.T
    step = heightmap.resolution * 0.5
    times = np.arange(step, max_range + step * 0.5, step)
    origin = np.array([x, y, z], dtype=np.float64)
    pos = origin + times[:, None, None] * world_dirs[None, :, :]
    ix, iy = heightmap.index(pos[:, :, 0], pos[:, :, 1])
    inside = heightmap.contains(ix, iy)
    terrain = np.full(ix.shape, np.inf, dtype=np.float64)
    terrain[inside] = heightmap.z[iy[inside], ix[inside]]
    hit = inside & (pos[:, :, 2] <= terrain)
    any_hit = hit.any(axis=0)
    first = hit.argmax(axis=0)
    rays = np.nonzero(any_hit)[0]
    if rays.size == 0:
        return np.zeros((0, 3), dtype=np.float64)

    t_hi = times[first[rays]]
    t_lo = np.maximum(t_hi - step, 0.0)
    dirs = world_dirs[rays]
    for _ in range(8):
        mid = 0.5 * (t_lo + t_hi)
        px = x + dirs[:, 0] * mid
        py = y + dirs[:, 1] * mid
        pz = z + dirs[:, 2] * mid
        sampled, ok = heightmap.sample(px, py)
        below = ok & (pz <= sampled)
        t_hi = np.where(below, mid, t_hi)
        t_lo = np.where(below, t_lo, mid)
    hit_t = t_hi
    world = origin + dirs * hit_t[:, None]
    sensor = (world - origin) @ rotation
    return sensor.astype(np.float64)
