"""Global terrain match. No initial pose. Level vehicle, yaw unknown."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from terrain_localize.heightmap import HeightMap


@dataclass
class Fix:
    x: float
    y: float
    z: float
    yaw: float
    score: float
    ix: int
    iy: int
    rms: float


def _correlate_valid(image: np.ndarray, kernel: np.ndarray) -> np.ndarray:
    """Correlation. Output[j, i] aligns kernel[0, 0] with image[j, i]."""
    height, width = image.shape
    kh, kw = kernel.shape
    fft_shape = (height + kh - 1, width + kw - 1)
    full = np.fft.irfft2(
        np.fft.rfft2(image, fft_shape) * np.fft.rfft2(kernel[::-1, ::-1], fft_shape),
        fft_shape,
    )
    return full[kh - 1 : kh - 1 + (height - kh + 1), kw - 1 : kw - 1 + (width - kw + 1)]


def _ncc(image: np.ndarray, template: np.ndarray, mask: np.ndarray) -> np.ndarray:
    count = float(mask.sum())
    centered = (template - (template * mask).sum() / count) * mask
    std_t = np.sqrt((centered * centered).sum() / count)
    if std_t < 1e-6:
        return np.full((image.shape[0] - template.shape[0] + 1, image.shape[1] - template.shape[1] + 1), -np.inf)
    numerator = _correlate_valid(image, centered)
    sum_m = _correlate_valid(image, mask)
    sum_m2 = _correlate_valid(image * image, mask)
    variance = np.maximum(sum_m2 / count - (sum_m / count) ** 2, 1e-8)
    return numerator / (std_t * np.sqrt(variance) * count)


def _bin_template(xs: np.ndarray, ys: np.ndarray, zs: np.ndarray, resolution: float):
    u = np.rint(xs / resolution).astype(np.int32)
    v = np.rint(ys / resolution).astype(np.int32)
    u0, u1 = int(u.min()), int(u.max())
    v0, v1 = int(v.min()), int(v.max())
    width = u1 - u0 + 1
    height = v1 - v0 + 1
    flat = (v - v0) * width + (u - u0)
    order = np.argsort(flat, kind="mergesort")
    flat = flat[order]
    zs = zs[order]
    template = np.zeros((height, width), dtype=np.float64)
    mask = np.zeros((height, width), dtype=np.float64)
    start = 0
    n = flat.size
    while start < n:
        stop = start + 1
        while stop < n and flat[stop] == flat[start]:
            stop += 1
        cell = int(flat[start])
        template[cell // width, cell % width] = float(np.median(zs[start:stop]))
        mask[cell // width, cell % width] = 1.0
        start = stop
    return template, mask, -u0, -v0


def _ground_points(points_sensor, min_cells, max_radius):
    points = np.asarray(points_sensor, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < min_cells:
        return None
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    horizontal = np.hypot(points[:, 0], points[:, 1])
    ground = (points[:, 2] < -2.0) & (horizontal > 5.0) & (horizontal < max_radius)
    points = points[ground]
    if len(points) < min_cells:
        return None
    return points


def _rms_grid(heightmap, xs, ys, zs, origins, yaw):
    """Height agreement for many sensor origins at one heading. Lower is better."""
    cos_y = float(np.cos(yaw))
    sin_y = float(np.sin(yaw))
    xr = cos_y * xs - sin_y * ys
    yr = sin_y * xs + cos_y * ys
    world_x = origins[:, 0:1] + xr[None, :]
    world_y = origins[:, 1:2] + yr[None, :]
    terrain, ok = heightmap.sample(world_x, world_y)
    implied = np.where(ok, terrain - zs[None, :], np.nan)
    enough = np.sum(ok, axis=1) >= max(40, xs.size // 5)
    sensor_z = np.nanmedian(implied, axis=1)
    error = implied - sensor_z[:, None]
    rms = np.sqrt(np.nanmean(error * error, axis=1))
    rms = np.where(enough, rms, np.inf)
    return rms, sensor_z


def localize(
    heightmap: HeightMap,
    points_sensor: np.ndarray,
    yaw_step_deg: float = 5.0,
    min_cells: int = 40,
    max_radius: float = 280.0,
) -> Fix | None:
    points = _ground_points(points_sensor, min_cells, max_radius)
    if points is None:
        return None
    xs = points[:, 0]
    ys = points[:, 1]
    zs = points[:, 2]

    # Correlation is reliable for the cell and weak for heading.
    rough_ix = rough_iy = None
    best_score = -np.inf
    for yaw_deg in np.arange(0.0, 360.0, yaw_step_deg):
        yaw = float(np.deg2rad(yaw_deg))
        cos_y = float(np.cos(yaw))
        sin_y = float(np.sin(yaw))
        template, mask, cx, cy = _bin_template(
            cos_y * xs - sin_y * ys,
            sin_y * xs + cos_y * ys,
            zs,
            heightmap.resolution,
        )
        if mask.sum() < min_cells:
            continue
        if template.shape[0] >= heightmap.height or template.shape[1] >= heightmap.width:
            continue
        score = _ncc(heightmap.z, template, mask)
        peak = np.unravel_index(int(np.argmax(score)), score.shape)
        peak_score = float(score[peak])
        if not np.isfinite(peak_score) or peak_score <= best_score:
            continue
        ix = int(peak[1] + cx)
        iy = int(peak[0] + cy)
        if not bool(heightmap.contains(np.int32(ix), np.int32(iy))):
            continue
        best_score = peak_score
        rough_ix, rough_iy = ix, iy
    if rough_ix is None:
        return None

    x0, y0 = heightmap.cell_center(rough_ix, rough_iy)
    stride = max(1, xs.size // 400)
    found = _search_neighborhood(
        heightmap,
        xs[::stride],
        ys[::stride],
        zs[::stride],
        x0,
        y0,
        yaw_step_deg,
        xy_step=5.0,
        xy_radius=30.0,
    )
    if found is None:
        return None
    rms, x, y, sensor_z, yaw = found
    refined = _search_neighborhood(
        heightmap,
        xs,
        ys,
        zs,
        x,
        y,
        yaw_step_deg=1.0,
        xy_step=1.0,
        xy_radius=4.0,
        yaw_center=yaw,
        yaw_radius_deg=4.0,
    )
    if refined is not None:
        rms, x, y, sensor_z, yaw = refined
    ix, iy = heightmap.index(x, y)
    return Fix(float(x), float(y), float(sensor_z), float(yaw), float(best_score), int(ix), int(iy), float(rms))


def _search_neighborhood(
    heightmap,
    xs,
    ys,
    zs,
    x0,
    y0,
    yaw_step_deg,
    xy_step,
    xy_radius,
    yaw_center=None,
    yaw_radius_deg=180.0,
):
    offsets = np.arange(-xy_radius, xy_radius + xy_step * 0.5, xy_step)
    ox, oy = np.meshgrid(offsets, offsets, indexing="xy")
    origins = np.column_stack((x0 + ox.ravel(), y0 + oy.ravel()))
    if yaw_center is None:
        yaws = np.deg2rad(np.arange(0.0, 360.0, yaw_step_deg))
    else:
        deltas = np.arange(-yaw_radius_deg, yaw_radius_deg + yaw_step_deg * 0.5, yaw_step_deg)
        yaws = yaw_center + np.deg2rad(deltas)
    best = None
    for yaw in yaws:
        yaw = float((yaw + np.pi) % (2.0 * np.pi) - np.pi)
        rms, sensor_z = _rms_grid(heightmap, xs, ys, zs, origins, yaw)
        choice = int(np.argmin(rms))
        if not np.isfinite(rms[choice]):
            continue
        if best is None or rms[choice] < best[0]:
            best = (float(rms[choice]), float(origins[choice, 0]), float(origins[choice, 1]), float(sensor_z[choice]), yaw)
    return best
