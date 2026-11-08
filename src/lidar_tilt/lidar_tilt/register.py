"""3D occupancy grid and upright pose match for one tilted lidar.

The base stays upright. A pose is x, y, z, and yaw. Roll and base pitch stay zero.
No ROS imports: the offline check and the node share this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree

HIT_ODDS = 0.85
MISS_ODDS = -0.4
ODDS_MIN = -2.0
ODDS_MAX = 3.5
OCCUPIED_ODDS = 0.0
INLIER_GATE_M = 0.4
INLIER_FRACTION = 0.4
INLIER_RMS_M = 0.25
MIN_POINTS = 30


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    def copy(self) -> "Pose":
        return Pose(self.x, self.y, self.z, self.yaw)

    def xy_error(self, other: "Pose") -> float:
        return float(
            math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2)
        )

    def yaw_error_deg(self, other: "Pose") -> float:
        delta = (self.yaw - other.yaw + math.pi) % (2.0 * math.pi) - math.pi
        return abs(math.degrees(delta))


def _wrap(yaw: float) -> float:
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def rotate_points(points: np.ndarray, yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    out = np.empty_like(points, dtype=np.float64)
    out[:, 0] = c * points[:, 0] - s * points[:, 1]
    out[:, 1] = s * points[:, 0] + c * points[:, 1]
    out[:, 2] = points[:, 2]
    return out


def apply_pose(points: np.ndarray, pose: Pose) -> np.ndarray:
    world = rotate_points(np.asarray(points, dtype=np.float64), pose.yaw)
    world[:, 0] += pose.x
    world[:, 1] += pose.y
    world[:, 2] += pose.z
    return world


def inverse_apply(points: np.ndarray, pose: Pose) -> np.ndarray:
    shifted = np.asarray(points, dtype=np.float64).copy()
    shifted[:, 0] -= pose.x
    shifted[:, 1] -= pose.y
    shifted[:, 2] -= pose.z
    return rotate_points(shifted, -pose.yaw)


class OccupancyGrid:
    def __init__(self, voxel: float = 0.05):
        self.voxel = float(voxel)
        self.log_odds: dict[tuple[int, int, int], float] = {}

    def _key(self, point: np.ndarray) -> tuple[int, int, int]:
        idx = np.floor(np.asarray(point, dtype=np.float64) / self.voxel).astype(np.int32)
        return int(idx[0]), int(idx[1]), int(idx[2])

    def _add(self, key: tuple[int, int, int], delta: float) -> None:
        value = self.log_odds.get(key, 0.0) + delta
        if value < ODDS_MIN:
            value = ODDS_MIN
        elif value > ODDS_MAX:
            value = ODDS_MAX
        self.log_odds[key] = value

    def integrate(self, origin: np.ndarray, endpoints: np.ndarray) -> None:
        """Mark hits occupied and the voxels along each ray, short of the hit, free."""
        origin = np.asarray(origin, dtype=np.float64).reshape(3)
        endpoints = np.asarray(endpoints, dtype=np.float64).reshape(-1, 3)
        if endpoints.size == 0:
            return
        for endpoint in endpoints:
            for key in _free_keys(origin, endpoint, self.voxel):
                self._add(key, MISS_ODDS)
            self._add(self._key(endpoint), HIT_ODDS)

    def occupy_points(self, points: np.ndarray, odds: float = 2.0) -> None:
        """Load a cloud that has no stored free space. Each point is a hit."""
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        for point in points:
            key = self._key(point)
            current = self.log_odds.get(key, 0.0)
            if odds > current:
                self.log_odds[key] = min(odds, ODDS_MAX)

    def occupied_centers(self) -> np.ndarray:
        keys = [key for key, odds in self.log_odds.items() if odds > OCCUPIED_ODDS]
        if not keys:
            return np.zeros((0, 3), dtype=np.float64)
        idx = np.asarray(keys, dtype=np.float64)
        return (idx + 0.5) * self.voxel

    def save_npz(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if self.log_odds:
            keys = np.asarray(list(self.log_odds.keys()), dtype=np.int32)
            values = np.asarray([self.log_odds[tuple(key)] for key in keys], dtype=np.float32)
        else:
            keys = np.zeros((0, 3), dtype=np.int32)
            values = np.zeros((0,), dtype=np.float32)
        np.savez_compressed(path, voxel=np.float64(self.voxel), keys=keys, log_odds=values)

    @classmethod
    def load_npz(cls, path: Path) -> "OccupancyGrid":
        data = np.load(path)
        grid = cls(float(data["voxel"]))
        keys = np.asarray(data["keys"], dtype=np.int32).reshape(-1, 3)
        values = np.asarray(data["log_odds"], dtype=np.float64).reshape(-1)
        for key, value in zip(keys, values):
            grid.log_odds[(int(key[0]), int(key[1]), int(key[2]))] = float(value)
        return grid


def _free_keys(origin: np.ndarray, end: np.ndarray, voxel: float):
    """Voxel keys the segment crosses, excluding the voxel that contains the endpoint."""
    direction = end - origin
    length = float(np.linalg.norm(direction))
    if length < voxel * 0.5:
        return
    step = np.sign(direction).astype(np.int32)
    key = np.floor(origin / voxel).astype(np.int32)
    key_end = np.floor(end / voxel).astype(np.int32)
    if np.array_equal(key, key_end):
        return
    t_max = np.full(3, np.inf)
    t_delta = np.full(3, np.inf)
    for axis in range(3):
        if step[axis] == 0:
            continue
        if step[axis] > 0:
            boundary = (key[axis] + 1) * voxel
        else:
            boundary = key[axis] * voxel
        t_max[axis] = max(0.0, (boundary - origin[axis]) / direction[axis])
        t_delta[axis] = voxel / abs(float(direction[axis]))
    yield tuple(int(v) for v in key)
    for _ in range(512):
        axis = int(np.argmin(t_max))
        if not math.isfinite(t_max[axis]) or t_max[axis] >= 1.0:
            return
        key[axis] += step[axis]
        t_max[axis] += t_delta[axis]
        if np.array_equal(key, key_end):
            return
        yield tuple(int(v) for v in key)


class MapIndex:
    def __init__(self, centers: np.ndarray):
        self.centers = np.asarray(centers, dtype=np.float64).reshape(-1, 3)
        self.tree = cKDTree(self.centers) if self.centers.shape[0] else None

    def __len__(self) -> int:
        return int(self.centers.shape[0])


def accepted(fraction: float, rms: float, count: int) -> bool:
    return count >= MIN_POINTS and fraction >= INLIER_FRACTION and rms <= INLIER_RMS_M


def score_pose(index: MapIndex, points: np.ndarray, pose: Pose, gate: float = INLIER_GATE_M):
    """Inlier fraction, rms, and count of a slice at a fixed pose."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if index.tree is None or points.shape[0] == 0:
        return 0.0, 1.0e9, 0
    distance, _ = index.tree.query(apply_pose(points, pose))
    keep = distance <= gate
    count = int(np.count_nonzero(keep))
    if count == 0:
        return 0.0, 1.0e9, 0
    rms = float(np.sqrt(np.mean(distance[keep] ** 2)))
    return count / points.shape[0], rms, count


def icp(
    index: MapIndex,
    points: np.ndarray,
    pose: Pose,
    gate: float = INLIER_GATE_M,
    iterations: int = 15,
) -> tuple[Pose, float, float, int]:
    """Point-to-point ICP in x, y, z, and yaw. Returns pose, inlier fraction, rms, count."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    current = pose.copy()
    if index.tree is None or points.shape[0] < MIN_POINTS:
        return current, 0.0, 1.0e9, 0
    fraction = 0.0
    rms = 1.0e9
    count = 0
    for _ in range(iterations):
        world = apply_pose(points, current)
        distance, neighbors = index.tree.query(world)
        keep = distance <= gate
        count = int(np.count_nonzero(keep))
        fraction = count / points.shape[0]
        if count < MIN_POINTS:
            rms = 1.0e9
            break
        rms = float(np.sqrt(np.mean(distance[keep] ** 2)))
        source = world[keep]
        rotated = source.copy()
        rotated[:, 0] -= current.x
        rotated[:, 1] -= current.y
        rotated[:, 2] -= current.z
        residual = index.centers[neighbors[keep]] - source
        py = rotated[:, 1]
        px = rotated[:, 0]
        normal = np.zeros((4, 4))
        gradient = np.zeros(4)
        normal[0, 0] = normal[1, 1] = normal[2, 2] = count
        gradient[0] = float(residual[:, 0].sum())
        gradient[1] = float(residual[:, 1].sum())
        gradient[2] = float(residual[:, 2].sum())
        yaw_x = -py
        yaw_y = px
        normal[0, 3] = float(yaw_x.sum())
        normal[1, 3] = float(yaw_y.sum())
        normal[3, 0] = normal[0, 3]
        normal[3, 1] = normal[1, 3]
        normal[3, 3] = float(np.dot(yaw_x, yaw_x) + np.dot(yaw_y, yaw_y))
        gradient[3] = float(np.dot(yaw_x, residual[:, 0]) + np.dot(yaw_y, residual[:, 1]))
        normal[np.diag_indices(4)] += 1.0e-4
        delta = np.linalg.solve(normal, gradient)
        current.x += float(delta[0])
        current.y += float(delta[1])
        current.z += float(delta[2])
        current.yaw = _wrap(current.yaw + float(delta[3]))
        if float(np.linalg.norm(delta[:3])) < 1.0e-4 and abs(float(delta[3])) < 1.0e-4:
            break
    return current, fraction, rms, count


def _sample(points: np.ndarray, limit: int) -> np.ndarray:
    if points.shape[0] <= limit:
        return points
    choice = np.linspace(0, points.shape[0] - 1, limit).astype(np.int64)
    return points[choice]


def _voxel_centers(points: np.ndarray, voxel: float) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if points.shape[0] == 0:
        return points
    idx = np.floor(points / voxel).astype(np.int64)
    view = np.ascontiguousarray(idx).view(np.dtype((np.void, idx.dtype.itemsize * 3)))
    _, keep = np.unique(view, return_index=True)
    return points[keep]


def _wall_points(points: np.ndarray, radius_m: float = 1.0) -> np.ndarray:
    """Drop the nearby floor. It stays close after a turn and hides the shift."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if points.shape[0] == 0:
        return points
    walls = points[np.hypot(points[:, 0], points[:, 1]) > radius_m]
    if walls.shape[0] >= 80:
        return walls
    return points



def _pull_in(index: MapIndex, dense: np.ndarray, seeds: list[Pose]) -> tuple[Pose, float, float, int]:
    best_pose = Pose()
    best_fraction = -1.0
    best_rms = 1.0e9
    best_count = 0
    for seed in seeds:
        wide, _, _, _ = icp(index, dense, seed, gate=0.6, iterations=8)
        pose, fraction, rms, count = icp(index, dense, wide, gate=0.15, iterations=10)
        if fraction > best_fraction or (abs(fraction - best_fraction) < 1.0e-6 and rms < best_rms):
            best_pose, best_fraction, best_rms, best_count = pose, fraction, rms, count
    return best_pose, best_fraction, best_rms, best_count



class WallField:
    """Sensor-height walls flattened to a 2D distance image, in meters.

    A full 3D neighbor search scores wrong poses as perfect because the room
    cloud is dense. The horizontal band does not.
    """

    def __init__(self, centers: np.ndarray, resolution: float = 0.10):
        centers = np.asarray(centers, dtype=np.float64).reshape(-1, 3)
        self.resolution = float(resolution)
        self.ok = False
        self.origin = np.zeros(2, dtype=np.float64)
        self.nx = 0
        self.ny = 0
        self.pad = 0
        self.image = np.zeros((1, 1), dtype=np.float32)
        if centers.shape[0] < MIN_POINTS:
            return
        band = centers[(centers[:, 2] > -0.5) & (centers[:, 2] < 0.8)]
        if band.shape[0] < MIN_POINTS:
            return
        flat = np.column_stack((band[:, 0], band[:, 1], np.zeros(band.shape[0])))
        xy = _voxel_centers(flat, self.resolution)[:, :2]
        self.origin = xy.min(axis=0) - 0.5
        maxs = xy.max(axis=0) + 0.5
        self.nx = int(math.ceil((maxs[0] - self.origin[0]) / self.resolution)) + 1
        self.ny = int(math.ceil((maxs[1] - self.origin[1]) / self.resolution)) + 1
        pix = np.floor((xy - self.origin) / self.resolution).astype(np.int32)
        inside = (
            (pix[:, 0] >= 0)
            & (pix[:, 0] < self.nx)
            & (pix[:, 1] >= 0)
            & (pix[:, 1] < self.ny)
        )
        pix = pix[inside]
        occupied = np.zeros((self.ny, self.nx), dtype=bool)
        occupied[pix[:, 1], pix[:, 0]] = True
        distance = np.minimum(distance_transform_edt(~occupied) * self.resolution, 1.0)
        self.pad = int(math.ceil(12.0 / self.resolution)) + 3
        self.image = np.full(
            (self.ny + 2 * self.pad, self.nx + 2 * self.pad), 1.0, dtype=np.float32
        )
        self.image[self.pad : self.pad + self.ny, self.pad : self.pad + self.nx] = distance
        self.ok = True

    def best_poses(self, sample_xy: np.ndarray, yaw_step: int = 5) -> list[Pose]:
        """Best upright poses. One distance-image pass per yaw, then a few seeds."""
        if not self.ok or sample_xy.shape[0] < 80:
            return []
        peaks: list[tuple[float, Pose]] = []
        step = max(int(yaw_step), 1)
        for yaw_deg in range(-180, 180, step):
            yaw = math.radians(float(yaw_deg))
            c = math.cos(yaw)
            s = math.sin(yaw)
            rx = c * sample_xy[:, 0] - s * sample_xy[:, 1]
            ry = s * sample_xy[:, 0] + c * sample_xy[:, 1]
            pix = np.floor(np.column_stack((rx, ry)) / self.resolution).astype(np.int32)
            acc = np.zeros((self.ny, self.nx), dtype=np.float32)
            used = 0
            for px, py in pix:
                y0 = self.pad + int(py)
                x0 = self.pad + int(px)
                view = self.image[y0 : y0 + self.ny, x0 : x0 + self.nx]
                if view.shape != acc.shape:
                    continue
                acc += view
                used += 1
            if used < 80:
                continue
            flat = int(np.argmin(acc))
            ty = flat // self.nx
            tx = flat % self.nx
            pose = Pose(
                float(self.origin[0] + (tx + 0.5) * self.resolution),
                float(self.origin[1] + (ty + 0.5) * self.resolution),
                0.0,
                yaw,
            )
            peaks.append((float(acc[ty, tx]) / used, pose))
        peaks.sort(key=lambda item: item[0])
        chosen: list[Pose] = []
        for _score, pose in peaks:
            if any(
                pose.xy_error(other) < 1.2 and pose.yaw_error_deg(other) < 25.0 for other in chosen
            ):
                continue
            chosen.append(pose)
            if len(chosen) == 3:
                break
        return chosen


def _sensor_ring(points: np.ndarray) -> np.ndarray:
    """Horizontal hits around the sensor. A steep tilt has too few of these to trust."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if points.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.float64)
    keep = (np.abs(points[:, 2]) < 0.45) & (np.hypot(points[:, 0], points[:, 1]) > 1.0)
    xy = points[keep][:, :2]
    if xy.shape[0] < 80:
        return np.zeros((0, 2), dtype=np.float64)
    bins = np.floor((np.arctan2(xy[:, 1], xy[:, 0]) + math.pi) * (6.0 / math.pi)).astype(np.int32)
    if np.unique(np.clip(bins, 0, 11)).size < 6:
        return np.zeros((0, 2), dtype=np.float64)
    return _sample(xy, 160)


def recover_pose(
    index: MapIndex,
    points: np.ndarray,
    field: WallField | None = None,
) -> tuple[Pose, float, float, int]:
    """Find x, y, z, and yaw of an upright cloud against a room map.

    The yaw/xy search is a lookup in the sensor-height distance image, then
    ICP on the full 3D map pulls in height. Returns an empty count when the
    cloud has no horizontal ring yet, so the caller keeps collecting slices.
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    empty = Pose(), 0.0, 1.0e9, 0
    if index.tree is None or len(index) < MIN_POINTS or points.shape[0] < MIN_POINTS:
        return empty
    sample = _sensor_ring(points)
    if sample.shape[0] < 80:
        return empty
    if field is None or not field.ok:
        field = WallField(index.centers)
    seeds = field.best_poses(sample)
    if not seeds:
        return empty
    return _pull_in(index, _sample(_wall_points(points), 900), seeds)


def global_search(index: MapIndex, points: np.ndarray) -> tuple[Pose, float, float, int]:
    """Coarse yaw and xy search, then a wide ICP pull-in and a tight ICP."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    empty = Pose(), 0.0, 1.0e9, 0
    if index.tree is None or len(index) < MIN_POINTS or points.shape[0] < MIN_POINTS:
        return empty
    sample = _sample(points, 500)
    z_pose = float(np.median(index.centers[:, 2]) - np.median(sample[:, 2]))
    mins = index.centers.min(axis=0)
    maxs = index.centers.max(axis=0)
    xs = np.arange(float(mins[0]), float(maxs[0]) + 0.5, 1.0)
    ys = np.arange(float(mins[1]), float(maxs[1]) + 0.5, 1.0)
    best_score = -1.0
    best = Pose(z=z_pose)
    for yaw_deg in range(-180, 180, 30):
        yaw = math.radians(yaw_deg)
        rotated = rotate_points(sample, yaw)
        for x_m in xs:
            shifted = rotated.copy()
            shifted[:, 0] += x_m
            shifted[:, 2] += z_pose
            for y_m in ys:
                query = shifted.copy()
                query[:, 1] += y_m
                distance, _ = index.tree.query(query)
                score = float(np.mean(distance <= 0.3))
                if score > best_score:
                    best_score = score
                    best = Pose(float(x_m), float(y_m), z_pose, yaw)
    if best_score < 0.15:
        return empty
    wide, _, _, _ = icp(index, points, best, gate=1.0, iterations=12)
    return icp(index, points, wide, gate=INLIER_GATE_M, iterations=15)


def load_ply(path: Path) -> np.ndarray:
    path = Path(path)
    with path.open("r", encoding="ascii", errors="replace") as handle:
        count = 0
        for line in handle:
            if line.startswith("element vertex"):
                count = int(line.split()[-1])
            if line.strip() == "end_header":
                break
        if count == 0:
            return np.zeros((0, 3), dtype=np.float64)
        data = np.loadtxt(handle, max_rows=count)
    data = np.atleast_2d(np.asarray(data, dtype=np.float64))
    return data[:, :3]


def save_ply(path: Path, points: np.ndarray) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    xyz = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    with path.open("w", encoding="ascii") as handle:
        handle.write("ply\nformat ascii 1.0\n")
        handle.write(f"element vertex {xyz.shape[0]}\n")
        handle.write("property float x\nproperty float y\nproperty float z\nend_header\n")
        for x_m, y_m, z_m in xyz:
            handle.write(f"{x_m:.5f} {y_m:.5f} {z_m:.5f}\n")
