---
name: room-localization
description: >
  Run continuous localization against a saved room map and report x, y, z,
  and clockwise heading only while the pose holds. Use when the user wants
  to localize, track the rig, read current coordinates, or runs /room-localization.
---

# Room localization

Read `src/lidar_tilt/OPERATIONS.md` and run the commands from that file. Do not change the matcher while a session is running.

## Start

Use the map they name. If they say the presentation room, that file is `src/lidar_tilt/maps/room2/presentation_room.npz`. Launch with `extend_map:=false` so the map is not rewritten.

The origin is the first cloud in that map. +X is that lidar's forward, +Y is its left, +Z is up. Heading reported to the user is clockwise from that first cloud.

## Monitor

Run `scripts/watch_pose.py` from this skill on the launch log. On each `ACTION_REQUIRED` line, read the launch log before telling them a position.

Report x, y, z in meters and the clockwise heading only when the same pose is still in `HOLD` lines. A `RECOVER` or `HOLD` followed by `pose lost`, `pose slipping`, or `recover rejected` is not a position. Say there is no lock yet.

Give the axis reminder with the first fix only.

## Stop

When they say to stop, park the head at level using the stop section of the operations guide, then stop the launch and any pose watcher. Do not start it again unless they ask.
