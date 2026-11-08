"""Offline pose checks for the occupancy matcher. No lidar and no servo."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

from lidar_tilt.register import (
    MapIndex,
    OccupancyGrid,
    Pose,
    apply_pose,
    global_search,
    icp,
    inverse_apply,
    load_ply,
    save_ply,
    score_pose,
    accepted,
)


def _scene() -> tuple[np.ndarray, np.ndarray]:
    """Near walls, plus a far wall the first pose cannot claim as its own slice."""
    near = []
    for y_m in np.linspace(-1.5, 1.5, 40):
        for z_m in np.linspace(0.2, 1.8, 16):
            near.append((2.2, y_m, z_m))
    for x_m in np.linspace(0.4, 2.0, 20):
        for z_m in np.linspace(0.2, 1.6, 12):
            near.append((x_m, -1.4, z_m))
    near.append((1.2, 0.8, 0.4))
    near.append((1.2, 0.8, 1.0))
    near.append((1.2, 0.8, 1.5))
    far = []
    for y_m in np.linspace(-0.4, 1.2, 24):
        for z_m in np.linspace(0.3, 1.7, 12):
            far.append((4.5, y_m, z_m))
    return np.asarray(near, dtype=np.float64), np.asarray(far, dtype=np.float64)


def _grid_from_points(points: np.ndarray, voxel: float = 0.05) -> OccupancyGrid:
    grid = OccupancyGrid(voxel)
    grid.integrate(np.zeros(3), points)
    return grid


def _recover(points: np.ndarray, truth: Pose, voxel: float = 0.08) -> Pose:
    grid = _grid_from_points(points, voxel)
    index = MapIndex(grid.occupied_centers())
    seen = inverse_apply(points, truth)
    pose, fraction, rms, count = global_search(index, seen)
    if fraction < 0.4:
        raise RuntimeError(f"global search rejected fraction={fraction:.2f} rms={rms:.3f} n={count}")
    return pose


def _check_hold() -> None:
    near, _far = _scene()
    grid = _grid_from_points(near, 0.08)
    index = MapIndex(grid.occupied_centers())
    nudged = near + np.array([0.01, 0.0, 0.01])
    fraction, rms, count = score_pose(index, nudged, Pose())
    print(f"hold still inliers={fraction:.2f} rms={rms:.3f}")
    if not accepted(fraction, rms, count):
        raise RuntimeError("a still slice would be allowed to walk the pose")
    shifted = inverse_apply(near, Pose(0.45, 0.0, 0.0, 0.0))
    fraction, rms, count = score_pose(index, shifted, Pose())
    print(f"hold moved inliers={fraction:.2f} rms={rms:.3f}")
    if accepted(fraction, rms, count):
        raise RuntimeError("a 0.45 m move was treated as standing still")


def _check_known_pose(points: np.ndarray, label: str) -> None:
    truth = Pose(1.0, -0.4, 0.1, math.radians(20.0))
    found = _recover(points, truth)
    xy = found.xy_error(truth)
    yaw = found.yaw_error_deg(truth)
    print(f"{label} pose error {xy:.3f} m, yaw {yaw:.2f} deg")
    if xy > 0.3 or yaw > 10.0:
        raise RuntimeError(f"{label} missed the known pose")


def _check_path() -> None:
    near, _far = _scene()
    extra = []
    for z_m in np.linspace(0.2, 1.5, 10):
        extra.append((3.4, 0.2, z_m))
        extra.append((3.4, 0.6, z_m))
        extra.append((3.6, 0.2, z_m))
    extra_points = np.asarray(extra, dtype=np.float64)
    grid = OccupancyGrid(0.08)
    grid.integrate(np.zeros(3), near)
    before = len(grid.occupied_centers())
    index = MapIndex(grid.occupied_centers())
    path = [
        Pose(0.3, 0.1, 0.0, math.radians(8.0)),
        Pose(0.7, -0.2, 0.05, math.radians(18.0)),
    ]
    current = Pose()
    combined = np.vstack((near, extra_points))
    for truth in path:
        seen = inverse_apply(combined, truth)
        current, fraction, rms, count = icp(index, seen, current)
        if fraction < 0.4 or rms > 0.25:
            current, fraction, rms, count = global_search(index, seen)
        if fraction < 0.4:
            raise RuntimeError(f"path step rejected fraction={fraction:.2f} rms={rms:.3f} n={count}")
        world = apply_pose(seen, current)
        origin = np.array([current.x, current.y, current.z])
        grid.integrate(origin, world)
        index = MapIndex(grid.occupied_centers())
    after = len(grid.occupied_centers())
    xy = current.xy_error(path[-1])
    yaw = current.yaw_error_deg(path[-1])
    print(f"path error {xy:.3f} m, yaw {yaw:.2f} deg, occupied {before} -> {after}")
    if xy > 0.3 or yaw > 10.0:
        raise RuntimeError("path tracking missed the last pose")
    if after <= before:
        raise RuntimeError("the second pose added no occupied cells")
    saved = Path("/tmp/lidar_tilt_grid_check.npz")
    ply_path = Path("/tmp/lidar_tilt_grid_check.ply")
    grid.save_npz(saved)
    loaded = OccupancyGrid.load_npz(saved)
    if len(loaded.occupied_centers()) != after:
        raise RuntimeError("npz round trip changed the occupied set")
    save_ply(ply_path, loaded.occupied_centers())
    reloaded = load_ply(ply_path)
    if reloaded.shape[0] != after:
        raise RuntimeError("ply round trip dropped occupied cells")


def _room_cloud() -> np.ndarray | None:
    candidates = [
        Path(__file__).resolve().parents[1] / "maps" / "room_cloud.ply",
        Path("/home/alex/Desktop/Hackathon_Code/src/lidar_tilt/maps/room_cloud.ply"),
    ]
    for path in candidates:
        if path.is_file() and path.stat().st_size > 200:
            cloud = load_ply(path)
            if cloud.shape[0] > 500:
                print(f"room cloud {cloud.shape[0]} points from {path}")
                return cloud
    print("no room cloud, synthetic scene only")
    return None


def main() -> None:
    near, far = _scene()
    _check_hold()
    _check_known_pose(np.vstack((near, far)), "synthetic")
    room = _room_cloud()
    if room is not None:
        step = max(1, room.shape[0] // 2500)
        _check_known_pose(room[::step], "room")
    _check_path()
    print("REGISTER_OK")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"REGISTER_FAIL {exc}", file=sys.stderr)
        sys.exit(1)
