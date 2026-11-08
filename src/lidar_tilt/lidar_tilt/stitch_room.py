"""Stitch station clouds into the first cloud's frame.

The first ply is the origin. Later clouds are matched with the same search
the live localizer uses, then merged. Heading in the notes is clockwise.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

from lidar_tilt.register import (
    MapIndex,
    OccupancyGrid,
    Pose,
    WallField,
    _voxel_centers,
    apply_pose,
    load_ply,
    recover_pose,
    save_ply,
)


def _occupy(grid: OccupancyGrid, points: np.ndarray) -> None:
    grid.occupy_points(_voxel_centers(points, grid.voxel), 2.0)


def _yaw_cw(pose: Pose) -> float:
    return -math.degrees(pose.yaw)


def stitch(paths: list[Path]) -> tuple[np.ndarray, OccupancyGrid, list[tuple[str, Pose]]]:
    if len(paths) < 2:
        raise RuntimeError("need at least two clouds")
    clouds = [load_ply(path) for path in paths]
    grid = OccupancyGrid(0.05)
    _occupy(grid, clouds[0])
    index = MapIndex(grid.occupied_centers())
    field = WallField(index.centers)
    placed = [clouds[0]]
    poses = [(paths[0].stem, Pose())]
    for path, cloud in zip(paths[1:], clouds[1:]):
        pose, fraction, rms, count = recover_pose(index, cloud, field)
        # A new station still sees walls the map does not have yet, so this is
        # looser than a live fix against a finished map.
        if not (count >= 40 and fraction >= 0.55 and rms <= 0.10):
            raise RuntimeError(
                f"{path.name} did not lock "
                f"inliers={fraction:.2f} rms={rms:.3f} count={count}"
            )
        world = apply_pose(cloud, pose)
        placed.append(world)
        poses.append((path.stem, pose))
        _occupy(grid, world)
        index = MapIndex(grid.occupied_centers())
        field = WallField(index.centers)
        print(
            f"{path.stem} x={pose.x:.3f} y={pose.y:.3f} z={pose.z:.3f} "
            f"yaw_cw={_yaw_cw(pose):.1f} inliers={fraction:.2f}",
            flush=True,
        )
    return np.vstack(placed), grid, poses


def _write_notes(path: Path, room: str, poses: list[tuple[str, Pose]]) -> None:
    lines = [
        room,
        "origin is the first cloud",
        "+X is that lidar forward, +Y is its left, +Z is up",
        "positive heading below is clockwise from the first cloud",
    ]
    for name, pose in poses:
        lines.append(
            f"{name} x_m {pose.x:.3f} y_m {pose.y:.3f} z_m {pose.z:.3f} "
            f"yaw_cw_deg {_yaw_cw(pose):.1f}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Stitch station clouds into the first cloud's frame.")
    parser.add_argument("clouds", nargs="+", type=Path, help="ply files, first one is the origin")
    parser.add_argument("--out", required=True, type=Path, help="output prefix, without an extension")
    parser.add_argument("--room", default="", help="name written at the top of the notes")
    args = parser.parse_args()
    paths = [path.expanduser().resolve() for path in args.clouds]
    for path in paths:
        if not path.is_file():
            raise SystemExit(f"missing cloud {path}")
    merged, grid, poses = stitch(paths)
    prefix = args.out.expanduser().resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    save_ply(prefix.with_suffix(".ply"), merged)
    grid.save_npz(prefix.with_suffix(".npz"))
    room = args.room.strip() or prefix.stem
    _write_notes(prefix.with_suffix(".txt"), room, poses)
    print(f"STITCH_OK {merged.shape[0]} points -> {prefix}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"STITCH_FAIL {exc}", file=sys.stderr)
        sys.exit(1)
