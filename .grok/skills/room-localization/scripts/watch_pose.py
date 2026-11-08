"""Print when a held localization pose moves. Quiet otherwise."""

from __future__ import annotations

import math
import re
import sys
import time

LINE = re.compile(
    r"(HOLD|RECOVER) x=([-0-9.]+) y=([-0-9.]+) z=([-0-9.]+) yaw=([-0-9.]+)"
)


def yaw_delta(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def main() -> None:
    path = sys.argv[1]
    reported = None
    pending = None
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        handle.seek(0, 2)
        while True:
            line = handle.readline()
            if not line:
                time.sleep(0.4)
                continue
            match = LINE.search(line)
            if match is None:
                if "pose lost" in line or "recover rejected" in line or "pose slipping" in line:
                    pending = None
                continue
            kind, x_m, y_m, z_m, yaw = match.groups()
            pose = (float(x_m), float(y_m), float(z_m), float(yaw))
            if pending is None:
                pending = pose
                continue
            near = (
                math.hypot(pose[0] - pending[0], pose[1] - pending[1], pose[2] - pending[2]) < 0.35
                and yaw_delta(pose[3], pending[3]) < 12.0
            )
            pending = pose
            if not near:
                continue
            moved = reported is None or (
                math.hypot(pose[0] - reported[0], pose[1] - reported[1], pose[2] - reported[2]) > 0.4
                or yaw_delta(pose[3], reported[3]) > 15.0
            )
            if not moved:
                continue
            reported = pose
            print(
                f"ACTION_REQUIRED: {kind} x={pose[0]:.2f} y={pose[1]:.2f} z={pose[2]:.2f} "
                f"yaw_cw={-pose[3]:.1f}",
                flush=True,
            )


if __name__ == "__main__":
    main()
