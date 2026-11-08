---
name: room-mapping
description: >
  Capture a room with the tilting lidar. Asks how many positions to record
  before any sweep, saves one cloud per station, then stitches them.
  Use when the user wants to map a room, capture stations, stitch clouds,
  or runs /room-mapping.
---

# Room mapping

Read `src/lidar_tilt/OPERATIONS.md` and run the commands from that file. The ports, launch lines, and stitch command live only there.

## Before any motion

Ask how many positions to capture before the stitch. Wait for a whole number. Do not open a serial port, flash, or start a launch before that answer.

Ask for the room name. Save clouds in `src/lidar_tilt/maps/<room>/`. Do not write into another room's directory, and do not overwrite `presentation_room` or `room_grid` unless they named that room.

## Each station

Wait until they describe this station and say to start. Their heading is clockwise-positive: positive is clockwise from the first cloud, negative is counterclockwise. Record that note with the cloud.

Then run one parked sweep from the operations guide: two passes, start at the current zero, save one ply, park at 0, and stop. Tell them the cloud path and wait. Do not start the next station.

The base stays upright. Only the lidar tilts.

## After the last station

Stitch in the order captured. The first cloud is the origin. +X is that lidar's forward, +Y is its left, +Z is up. Use the stitch command in the operations guide. Show the pose table it writes. Do not start localization unless they ask.
