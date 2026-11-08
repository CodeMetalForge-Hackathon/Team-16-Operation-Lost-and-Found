"""Recover three known poses from ray-cast clouds. No initial guess."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

from terrain_localize.heightmap import load_heightmap, save_heightmap, synthetic_hill
from terrain_localize.match import localize
from terrain_localize.raycast import lidar_directions, raycast_cloud

POSES = (
    (400.0, 1500.0, math.radians(20.0)),
    (1200.0, 800.0, math.radians(110.0)),
    (1500.0, 400.0, math.radians(250.0)),
)
AGL = 80.0
MAX_XY = 30.0
MAX_YAW = math.radians(15.0)


def map_paths() -> tuple[Path, Path]:
    source_dir = Path(__file__).resolve().parents[1] / "maps"
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("terrain_localize")) / "maps" / "synthetic_hill.yaml"
        if share.exists():
            return share, source_dir
    except Exception:
        pass
    return source_dir / "synthetic_hill.yaml", source_dir


def angle_diff(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def run_once(heightmap, directions) -> bool:
    ok = True
    for x, y, yaw in POSES:
        terrain, on_map = heightmap.sample(x, y)
        if not on_map:
            print(f"pose ({x:.0f}, {y:.0f}) is off the map")
            return False
        z = float(terrain) + AGL
        cloud = raycast_cloud(heightmap, x, y, z, yaw, directions)
        fix = localize(heightmap, cloud)
        if fix is None:
            print(f"no fix for ({x:.0f}, {y:.0f}, yaw {math.degrees(yaw):.0f})")
            ok = False
            continue
        xy_error = math.hypot(fix.x - x, fix.y - y)
        yaw_error = abs(angle_diff(fix.yaw, yaw))
        passed = xy_error <= MAX_XY and yaw_error <= MAX_YAW
        ok = ok and passed
        print(
            f"truth ({x:.1f}, {y:.1f}, z {z:.1f}, yaw {math.degrees(yaw):.1f}) "
            f"fix ({fix.x:.1f}, {fix.y:.1f}, z {fix.z:.1f}, yaw {math.degrees(fix.yaw):.1f}) "
            f"xy {xy_error:.1f} m  yaw {math.degrees(yaw_error):.1f} deg  rms {fix.rms:.2f} m "
            f"{'ok' if passed else 'FAIL'}"
        )
    return ok


def ensure_map(source_dir: Path):
    yaml_path = source_dir / "synthetic_hill.yaml"
    npy_path = source_dir / "synthetic_hill.npy"
    if yaml_path.exists() and npy_path.exists():
        return load_heightmap(yaml_path)
    heightmap = synthetic_hill()
    save_heightmap(heightmap, source_dir)
    return heightmap


def main() -> None:
    yaml_path, source_dir = map_paths()
    if yaml_path.exists():
        heightmap = load_heightmap(yaml_path)
    else:
        heightmap = ensure_map(source_dir)
    directions = lidar_directions()
    passed = True
    for trial in range(1, 4):
        print(f"--- trial {trial} ---")
        passed = run_once(heightmap, directions) and passed
    if not passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
