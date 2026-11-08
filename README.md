# Team 16

## Live sensor dashboard

Start the dashboard from the repository root:

    python dashboard/serve.py

Open the local address printed by the server, enter the Pi hostname or IP
address, and select **Connect screens**. The dashboard shows the fused detection screen on port 8081 and
the thermal screen on port 8082.

## Windows thermal demo

Thermal scripts live in `thermal/` (data in `thermal/data/`) and are run from inside that folder. Start the HT-203U hotspot viewer with:

```powershell
cd thermal
python thermal_viewer.py
```

The viewer opens camera index 1 and displays hotspot boxes plus the experimental
human-likelihood percentage. Press `Q` or Escape to quit. Replay a saved frame
without a camera with:

```powershell
python thermal_viewer.py --image data/images/41f70a7f_124109_720005.png
```

Rebuild the scene features and model after collecting more labeled images:

```powershell
python scene_features.py
python train_model.py
```

## Thermal on the Raspberry Pi: CPU vs Hailo-8

`thermal/thermal_pi.py` runs the thermal person classifier on the Pi with the USB HT-203U and
serves an annotated live feed to any browser on the network. Two backends, same frames:

- `cpu`: the RandomForest in `thermal/data/thermal_model.joblib` (6 hand-built scene features).
- `hailo`: pretrained YOLOv8s person detector (`.hef`) on the Hailo-8. The RandomForest cannot run
  on the Hailo (it only runs compiled neural networks), so this is a different, pretrained model.
  It also returns person boxes. It was trained on color photos, not thermal.

Run on the Pi from `~/team16_thermal/thermal` (copy `scene_features.py`, `thermal_viewer.py`,
`thermal_pi.py` and `data/` there; `thermal/setup_hailo_py.sh` builds the Hailo Python bindings and
downloads the model, no sudo):

```bash
export PYTHONPATH=$HOME/team16_thermal/hailort/hailort/libhailort/bindings/python/platform
python3 thermal_pi.py --bench-images               # speed + accuracy on the 74 labeled images
python3 thermal_pi.py --bench-camera 150           # speed on live camera frames
python3 thermal_pi.py --serve 8081 --backend both  # live feed: http://<pi address>:8081/
```

Measured on the Pi 5 (74 labeled images, 27 human):

| | CPU RandomForest | Hailo-8 YOLOv8s |
|---|---|---|
| Time per frame (mean) | 12.5-13.1 ms (~78 fps max) | 14.9-15.0 ms (~67 fps max) |
| of which model inference | 11.2 ms | 11.7 ms |
| of which preprocessing | 1.3 ms | 3.2 ms |
| Ranking quality (AUC) | 0.73 on held-out sessions (0.98 on its own training data) | 0.80, never trained on these images |
| Accuracy | 66% on held-out sessions | 72% at the usual 0.5 cutoff, 85% at a 0.05 cutoff |

Takeaways: the Hailo is not faster than this tiny RandomForest; what it adds is a real neural
detector that locates people, needs no training here, and leaves the CPU free. Its person scores
run low on thermal frames, so use a low cutoff (about 0.05) and expect false alarms. Feeding it the
raw grayscale frame scored better than contrast-stretching it. Only 74 images, so treat these
numbers as indicative.

Camera notes: read the thermal camera with GStreamer (`v4l2src ... YUY2 256x392`), not OpenCV's
V4L2 path, and crop rows 196:388 as on Windows. On one USB port the camera failed the USB
handshake (`Failed to query (130) UVC probe control`); on another port it worked. If it will not
open, try a different port first.

## Fused thermal + visible detection (Pi)

`thermal/fusion_pi.py` runs both cameras and one shared Hailo-8 in a single process: YOLOv8s finds
people in the Pi Cam 3 (color) and the HT-203U (gray), the RandomForest scores the thermal frame, and
each visible person box is checked for a hotspot inside it. Labels: HUMAN (shape + heat), UNCONFIRMED,
NO HEAT (decoy?), THERMAL ONLY (heat, no visible person). `thermal/fusion_web.py` is a separate
process that serves the results so the Hailo loop can never starve the web server.
`thermal/fusion_supervisor.sh` (started by `start_fusion.sh`) is a watchdog: if the detector exits, or
stops writing frames for 15 s (the Hailo `infer()` call has hung after about an hour), it is killed and
restarted automatically. Restarts are logged in `fusion.log`.

On the Pi (stop other feeds first; only one process can own the Hailo and the Pi camera):

```bash
~/team16_thermal/thermal/start_fusion.sh            # add --record to save paired frames
~/team16_thermal/thermal/stop_fusion.sh
```

Fused view (visible cropped to the thermal field of view) on `:8081`, thermal view on `:8082`, and the
dashboard on `:8090`. The visible camera is rotated 180 degrees and uses the full-field 2304x1296 sensor
mode (the default 1280x720 pick is a centre crop). Align the two cameras with the dashboard's
**Recalibrate** button: stand alone in view of both cameras and move slowly for up to 30 s. It writes
`thermal/fusion_calib.json` (not committed; it depends on how the cameras are mounted). Manual
alternative: `thermal/calibrate_fusion.py` with a pair saved by `--record`. Without calibration the
overlay says UNCALIBRATED and the alignment is only a rough guess. The fusion weights are hand-set,
not trained, and no accuracy has been measured yet.

## Thermal canopy fusion (see through foliage)

`thermal/thermal_stitch.py` projects many drone frames onto a common ground grid using each
frame's pose, then fuses them with a high percentile so the warm target stays and the canopy
(which sits above the ground and smears across frames) is rejected. Run from `thermal/`:

```powershell
python thermal_stitch.py --demo                    # simulated drone sweep -> data/stitch_demo.png
python thermal_stitch.py --demo --pose-noise 0.5   # see how pose error ruins it
python thermal_stitch.py --selftest
python thermal_stitch.py --frames-dir DIR --poses poses.csv   # real frames; CSV: file,x,y,z,yaw
```

Simulation result: pose error up to ~0.3 m is fine at 20 m altitude; 0.5 m or worse loses the
target. A narrow flight path can fail on some canopies (target never visible), so fly wide.
Assumes a nadir-looking camera over flat ground; HFOV defaults to 50 deg (`--hfov`).

## Pi Camera 3 color detector

Scripts live in `picam/`. Run from the repository root:

```powershell
$env:PI_HOST="<pi address>"
python picam/picam_color_anomaly.py   # learn office colors, then circle rare ones
python picam/picam_pc_view.py         # plain live view
```

The baseline learns every color it sees, including grays and whites, so pan the camera across the
whole scene while it learns. The learning time is the `--learn` default in
`picam/picam_color_anomaly.py` (seconds; search for `--learn`), or pass `--learn <seconds>`.
The baseline mostly ignores brightness, so a shadowed or lit version of a known color is not flagged.

To run it on the Pi and watch from your PC, copy `picam_color_anomaly.py` to the Pi and run
`python3 picam_color_anomaly.py --source local --headless --serve 8080`, then open
`http://<pi address>:8080/`. Relearn there with `touch ~/relearn`.
`picam/setup_picam.sh` builds libcamera for the Pi 5 on Ubuntu 24.04.

## RPLidar C1 tilt map

The C1 stays a 2D lidar on `/dev/ttyUSB0`. A Seeed XIAO ESP32-C5 on `/dev/ttyACM0` drives one bipolar stepper through an Adafruit DC & Stepper Motor HAT on I2C (`D4` SDA, `D5` SCL, `3V3`, `GND` to the Pi header, coils on M1 and M2). Motor power is the HAT screw terminal, not the XIAO. `tilt_sweep` sends `GOTO <degrees>`, waits for `AT <degrees>`, then keeps the next `/scan` and pitches it into `base_link`. The motor steps from -60 to 60 degrees in 0.5 degree increments and travels back until you stop the launch. The pose at reset is 0 degrees, so level the lidar before reset. Each settled scan is published on `/tilt_slice`. `/tilt_cloud` keeps the latest coverage from the current spot. `cloud_localize` turns those slices into a 3D occupancy grid and a pose. Carry the upright rig to a new spot and the grid grows. Each pass rewrites `src/lidar_tilt/maps/room_grid.npz` and `room_cloud.ply`.

Station capture, stitching, and live localization commands are in `src/lidar_tilt/OPERATIONS.md`.

Flash the board before the first sweep. The upload port is the XIAO, not the lidar.

```bash
cd firmware/lidar_tilt
pio run -t upload --upload-port /dev/ttyACM0
```

Source ROS 2 Jazzy, the C1 driver workspace, and this repo:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/sllidar/install/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select lidar_tilt
ros2 launch lidar_tilt localize.launch.py
```

An empty `map_path` starts a new grid at the origin. To localize against a saved map without painting more cells:

```bash
ros2 launch lidar_tilt localize.launch.py map_path:=src/lidar_tilt/maps/room_grid.npz extend_map:=false
```

`map_path` may also be an ascii xyz PLY. The pose is `/pose_fix` in `map`, and the occupied cells are `/map_cloud`. The base stays upright: the fix is x, y, z, and yaw. Positive tilt pitches the laser frame about Y. `tilt_sign` is `-1` so a forward ray moves toward +Z on this stepper. Set it back to `1.0`, or flip `kStepSign` in the firmware, if the cloud bends the wrong way. Ranges closer than 0.15 m or past 12 m are dropped. `dead_min_deg` and `dead_max_deg` mask the mount. When the first number is greater than the second, the mask wraps past ±180°. Steps per revolution, gear ratio, the angle clamp, settle time, and the sweep rate are at the top of `firmware/lidar_tilt/src/main.cpp`. The motor eases between angles at about 24 degrees per second. Coil PWM is scaled down because the hat does not limit current on a 12 V supply.

## Terrain localization

`terrain_localize` matches a 3D lidar `PointCloud2` on `/lidar/points` to a prior heightmap. There is no GPS, no initial pose, and no command output. The map in the repo is a synthetic 2 km hill (`src/terrain_localize/maps`). A real GeoTIFF can replace `synthetic_hill.npy` and the yaml beside it.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select terrain_localize terrain_gazebo
ros2 run terrain_localize check_poses
ros2 launch terrain_localize demo.launch.py
```

The check ray-casts three known poses and must recover each within 30 m and 15 degrees. The path demo opens RViz, steps eight stops, and writes `/tmp/terrain_path_demo.png`. Fixes are published on `/pose_fix`. `ros2 launch terrain_gazebo hill.launch.py` starts a headless Gazebo world for the same topic. On the Intel GPU in this laptop the heightmap shader does not compile, so use `ros2 run terrain_localize raycast_cloud` for the repeatable demo.
