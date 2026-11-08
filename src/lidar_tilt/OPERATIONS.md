# Lidar tilt: map a room, then localize

The rig stays upright. A stepper tilts the RPLidar C1 from -60° to 60°. Mapping saves one cloud per station and stitches them. Localization matches live slices to that map and reports x, y, z, and a clockwise heading.

The lidar is `/dev/ttyUSB0`. The XIAO ESP32-C5 is `/dev/ttyACM0`. Flash and serial commands go to the XIAO only.

## Shell

Run this in every new terminal. `pio` is not on `PATH`.

```bash
source /opt/ros/jazzy/setup.bash
source /home/alex/Desktop/install/setup.bash
source /home/alex/Desktop/Hackathon_Code/install/setup.bash
export DISPLAY=:0
```

Build after a Python or launch change:

```bash
cd /home/alex/Desktop/Hackathon_Code
colcon build --symlink-install --packages-select lidar_tilt
```

Existing Python files are hardlinked into the install, so a new process picks them up without a rebuild. A new module needs the build above.

## Rules

- Level the lidar before a reset. The pose at reset is 0°. Opening `/dev/ttyACM0` often resets the board, and a second open can reset it again. `ZERO` marks the current shaft angle as 0° and does not move. `GOTO 0` moves back to that mark. Do not open the port while a sweep already has it.
- One launch owns both serial ports. Stop it before starting another.
- `tilt_sign` is `-1` on this stepper, so a forward ray goes toward +Z. Leave it there unless a cloud bends the wrong way.
- Station headings in these notes are clockwise from the first cloud. The log yaw is the opposite sign: clockwise degrees = `-yaw` from a `HOLD` line.
- The map origin is the first cloud. +X is that lidar's forward, +Y is its left, +Z is up. The base does not roll.
- `extend_map:=false` localizes without writing the map. A mapping launch with no `map_path` paints a new grid and rewrites `src/lidar_tilt/maps/room_grid.npz` and `room_cloud.ply`.
- Clouds and grids under `src/lidar_tilt/maps/` are gitignored.

## Flash

Only when the firmware must change, and only to the XIAO.

```bash
cd /home/alex/Desktop/Hackathon_Code/firmware/lidar_tilt
/home/alex/.platformio/penv/bin/pio run -t upload --upload-port /dev/ttyACM0
```

If the loader does not start, hold BOOT, tap RESET, and release BOOT. Level the head before this reset.

## Mark level as zero

Do this only when nothing else has `/dev/ttyACM0` open, after the head is where 0° should be.

```bash
python3 - << 'PY'
import serial
port = serial.Serial()
port.port = "/dev/ttyACM0"
port.baudrate = 115200
port.timeout = 0.2
port.dtr = False
port.rts = False
port.open()
port.write(b"ZERO\n")
port.flush()
for _ in range(30):
    line = port.readline().decode("ascii", errors="replace").strip()
    if line:
        print(line)
    if line.startswith("AT"):
        break
port.close()
PY
```

Expect `AT 0.0`. Then start the sweep without opening the port again yourself.

## Map one station

Stop any launch that still has the ports. Set the shaft, mark zero if this station's zero is new, then:

```bash
ros2 launch lidar_tilt sweep.launch.py \
  continuous:=false start_at_zero:=true park_at_zero:=true passes:=2 \
  output_ply:=/home/alex/Desktop/Hackathon_Code/src/lidar_tilt/maps/ROOM/NAME.ply
```

That sweeps 0° to +60° to -60°, does it twice, writes one cloud, and parks at 0°. Wait until the log says `Parked at` and the process exits. Repeat for the next station only after the rig has been moved and you have the new heading.

`passes:=2` is the two-sweep capture. `rviz:=false` skips the viewer.

## Stitch

List the clouds in capture order. The first file is the origin. This refuses a cloud that does not lock, and it does not start localization.

```bash
cd /home/alex/Desktop/Hackathon_Code
PYTHONPATH=src/lidar_tilt python3 -m lidar_tilt.stitch_room \
  src/lidar_tilt/maps/ROOM/first.ply \
  src/lidar_tilt/maps/ROOM/second.ply \
  --room ROOM \
  --out src/lidar_tilt/maps/ROOM/ROOM
```

Add every later ply before `--room`. The command writes `ROOM.ply`, `ROOM.npz`, and `ROOM.txt`. The text file is the station table: x, y, z in meters and clockwise heading.

The presentation room is already stitched at `src/lidar_tilt/maps/room2/presentation_room.npz`.

## Localize continuously

```bash
ros2 launch lidar_tilt localize.launch.py \
  map_path:=/home/alex/Desktop/Hackathon_Code/src/lidar_tilt/maps/ROOM/ROOM.npz \
  extend_map:=false
```

RViz starts with the launch. The fixed frame is `map`. The pose topic is `/pose_fix`. The map cloud is `/map_cloud`.

A fix in the log looks like:

```text
HOLD x=-0.28 y=-1.13 z=0.04 yaw=-12.5 inliers=0.99 rms=0.045
```

That example is x = -0.28 m, y = -1.13 m, z = 0.04 m, heading = 12.5° clockwise. Report a pose only while later lines stay on `HOLD` for those same numbers. `pose lost` and `recover rejected` mean there is no lock.

Watch a launch log for the next held pose (the script prints nothing until the pose moves):

```bash
python3 /home/alex/Desktop/Hackathon_Code/.grok/skills/room-localization/scripts/watch_pose.py LAUNCH.log
```

## Stop with the head level

A one-station sweep already parks at 0°.

During a continuous launch, the head crosses level twice each pass. Stop the launch when the log prints `AT 0.0 deg` or `AT 0.5 deg`. That leaves the shaft on the center step. Closing the port can reset the board; after that, the stopped angle is the new software zero, so do not send `GOTO 0` unless you still want the old mark.

Ctrl-C in the launch terminal stops the lidar node, the sweep, localization, and RViz together.
