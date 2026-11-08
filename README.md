# Team 16: Operation Lost and Found

Picture this: someone is lost in the White Mountains. No GPS, no cell service, no radio. That was our problem at Code Metal Forge 2026, and over one weekend we built a small sensor pod to deal with it.

The pod works out where it is without GPS, looks for people with a normal camera and a thermal one, listens for stray radio signals from phones, and carries a beacon a ground team can walk toward. Everything lands on one dashboard in a browser.

The block diagram is in [`media/pod-blocks.html`](media/pod-blocks.html). Open it in a browser. Photos and screenshots of the build go in [`media/`](media/) too.

## What we built

Four lines of effort. [`LOE.md`](LOE.md) has the short status of each.

**1. Find itself without GPS.** The pod coasts on an IMU (`deadreckoning/`) and corrects the drift with a lidar-versus-terrain match (`src/terrain_localize/`). For indoor work there's a rig that tilts an RPLidar C1 back and forth to build a 3D map of a room, then localizes against it (`src/lidar_tilt/`, `firmware/lidar_tilt/`).

**2. Find a person.** A Pi Camera and an HT-203U thermal camera share one Hailo-8 AI chip on a Raspberry Pi 5. We fuse the two views, so every contact gets a label: **HUMAN** (shape and heat agree), **NO HEAT** (looks like a person, feels like a mannequin), **THERMAL ONLY** (warm, but nothing visible), or **UNCONFIRMED**. Code is in `thermal/` and `picam/`.

**3. Decide where to look.** `search_hotspots/` builds a probability map for any of 35 trails, using the terrain and 547 real incident records (80 of them lost or missing). It ranks the spots a search team should check first.

**4. Hand the find to people on foot.** A LoRa beacon on a LilyGO board (`df_beacon/`) sends a packet a second at 915 MHz, plus Wi-Fi and Bluetooth signals a phone can walk toward. A receive-only software radio setup (`src/team16_sdr/`) logs phones and other transmitters near the pod.

The dashboard (`dashboard/`) pulls it all together: fused camera view, thermal view, Pi health, radio contacts, the lidar room map, and the pod's position.

## How we built it

- **Hardware:** Raspberry Pi 5 with a Hailo-8, HT-203U thermal camera, Pi Camera 3, RPLidar C1, a stepper motor on an Adafruit motor hat driven by a XIAO ESP32 board, an MPU9250 IMU, two ADALM-Pluto radios, an Alfa Wi-Fi card, and a LilyGO LoRa32 beacon.
- **Software:** mostly Python, with ROS 2 Jazzy for the lidar and terrain pieces, PlatformIO firmware for the microcontrollers, and a plain HTML/JS dashboard with no framework.
- **Detection:** YOLOv8s on the Hailo finds people in both cameras. A small random forest scores the thermal frame on six hand-built features. The fusion step checks whether each visible person has a hotspot inside their box.
- **Search map:** probability in each cell is distance from the planned route, times whether the slope can be walked, boosted by water, trail, alpine ground, terrain traps, and past incidents. The weights came from the incident spreadsheet.
- **Radios:** each tool writes hits to its own log, and a hub (`rf_hub.py`) sorts them by category and serves them on live ports. The dashboard reads from that.
- **Small services:** the dashboard talks to little feeds (map, Pi stats, radio, inertial) that each run on their own port, so one can die without taking the rest down.

## What worked

- **Fusion labels are useful.** Seeing HUMAN versus NO HEAT on a live feed made the demo click right away.
- **The Hailo beat our own classifier on ranking.** YOLOv8s scored 0.80 AUC on our 74 labeled thermal frames and never trained on them. The random forest scored 0.73 on held-out sessions. At a 0.05 cutoff the Hailo reaches 85% accuracy, though it also raises false alarms.
- **Lidar room mapping.** The tilt rig stacks scans into a 3D occupancy grid. We can carry it around, stitch rooms together, and get a live pose against the saved map.
- **Dead reckoning (in simulation).** With a lidar fix every second the position holds at about 5 m RMS. A 60-second dropout drifts to about 120 m, under our 250 m budget. Walking mode counted all 80 steps of a simulated 56 m square and finished 0.5 m off.
- **Phone detection.** The radios pick up Wi-Fi probes, Bluetooth, and cellular uplink bursts, and rank contacts strongest first.
- **Canopy fusion (in simulation).** Stacking many drone frames keeps a warm target while foliage smears away, as long as pose error stays under about 0.3 m at 20 m altitude.

## What we fixed

- **The Hailo stalling froze everything.** The detector, cameras, and web page all lived in one process, so one hung `infer()` call took them all down. Now the Hailo runs in its own worker process that gets killed and restarted when it stops answering, the web server is a separate process, and a supervisor script restarts the whole detector if frames stop for 15 seconds. If the chip can't open at all, the cameras and hotspots still run.
- **Upside-down camera.** The thermal camera is mounted upside down. We first rotated it in the dashboard, then moved the rotation to the source so every view agrees.
- **Cellular scan bands.** We got these wrong a few times. A 20 MHz dwell window also catches tower downlink, which isn't a phone. We kept only US uplink ranges, swapped out downlink-only dwells, and made the hub drop anything that isn't an uplink.
- **Pluto buffer bugs.** The Pi 5's USB controller sometimes handed back a dead buffer. We recreate it until it's live, require a real signal before accepting it, and use per-device locks so two radios can coexist.
- **Bluetooth.** The Alfa card's Bluetooth chip wedges the kernel on this Pi, so we moved Bluetooth scanning to a different adapter and made boot wait for it.
- **Dashboard feeds.** The map feed now remembers every pose and survives restarts. Camera tiles recover on their own. The thermal feed is capped at 18 fps since the camera delivers 25 and we didn't need them all.
- **Visible camera framing.** The default 1280x720 mode is a centre crop. We switched to the full 2304x1296 sensor mode so the two cameras line up.

## What changed along the way

- **Servo to stepper.** The lidar tilt started with a hobby servo sweeping -30 to 30 degrees. We moved to a stepper on a motor hat, sweeping -60 to 60 in half-degree steps, and added station capture, stitching, and fast localization against a stitched room map.
- **The ROS workspace went out and came back.** We added a ROS 2 workspace early, removed it, then re-added the pieces we actually used.
- **The beacon went from one signal to three.** It started as a Wi-Fi and Bluetooth beacon a phone could see. We added LoRa at 915 MHz, because sub-GHz goes through trees far better than 2.4 GHz.
- **The dashboard changed a lot.** It started as a debug page and became a real panel layout. We added Pi health (CPU, temperature, throttling, an AI-stall counter), the lidar map with a rotate button, a Reset fusion button, and one-click calibration. We removed the beacon panel, restyled it a few times (cream background, pastel colors, softer corners), and ended up with something we'd actually show a judge.
- **Fusion got a restart button.** When the detector is being difficult you can reset it from the browser instead of opening a terminal.
- **Radio data got organized.** What began as a few scripts writing log files turned into a hub with per-category logs, live ports, a laptop viewer (`rf_watch.py`), and services that start on boot.

## What was hard

- **The Hailo.** It stalled most often while the cellular scan was running. We never proved why. When it hangs, only a full power-off brings it back, so there were a lot of restarts and a lot of staring at logs.
- **Sharing hardware.** Only one process can own the Hailo and the Pi camera, and only one buffer can be open per Pluto. A lot of "why did it stop working" turned out to be two things grabbing the same device.
- **Calibration.** Lining up a thermal camera with a visible camera depends on exactly how they're bolted down. We ended up with a calibration button where you walk across the view.
- **Radio rules.** The Pluto can transmit, so it could have done things we chose not to do. Faking a cell tower, collecting phone identifiers, and pushing messages to someone's handset are illegal for us, and an unlicensed band doesn't change that. The only thing we transmit is the beacon.
- **Phones are quiet.** A handset puts out about a fifth of a watt, and a phone with no tower it can decode may say nothing at all. What we catch is a phone that still has a tower, or just lost one, and only nearby.
- **Real terrain.** The Gazebo heightmap shader doesn't compile on one team laptop's GPU, so the repeatable terrain demo uses a ray-cast instead.

## What's not proven yet

We'd rather say it plainly than oversell it.

- The dead-reckoning error numbers come from simulation, not a flight.
- The terrain match uses a synthetic 2 km hill. It has never flown or run over a real mountain.
- Fusion weights are hand-picked, and the fused label's accuracy is unmeasured.
- The thermal results come from only 74 labeled images (27 with a person), so treat them as indicative.
- The Franconia Ridge heatmap uses a real elevation tile, but other trails use synthetic terrain, and the map has not been scored against real find locations.
- The beacon has no release servo and no parachute in the repo. For now you carry it and set it down.
- The canopy fusion is a simulation.

## Where things live

| Folder | What's in it |
|---|---|
| `deadreckoning/` | IMU dead reckoning (drone and walking modes), simulator, evaluation |
| `src/terrain_localize/`, `src/terrain_gazebo/` | Lidar-to-heightmap terrain match and Gazebo world |
| `src/lidar_tilt/`, `firmware/lidar_tilt/` | Tilting lidar room mapper and its stepper firmware |
| `thermal/` | Thermal camera, fusion detector, Hailo worker, supervisor, canopy stitching |
| `picam/` | Pi Camera color-anomaly detector |
| `search_hotspots/`, `data/`, `dem/` | Search probability map, trail and incident data, elevation tiles |
| `df_beacon/` | LoRa, Wi-Fi, and Bluetooth homing beacon firmware |
| `src/team16_sdr/` | Receive-only SDR scanners, radio hub, phone correlator |
| `dashboard/` | The browser dashboard and its feeds |
| `media/` | Diagrams, photos, screenshots |

## Team

Roman: dead reckoning, thermal and fusion detection, dashboard, beacon. Alexander MacConnell: lidar and terrain match. Lauren Berning: search map. Patrick McAuliffe: radios.

---

# Run notes

Details for running each piece. Other folders have their own READMEs: [`deadreckoning`](deadreckoning/README.md), [`search_hotspots`](search_hotspots/README.md), [`df_beacon`](df_beacon/README.md), [`dashboard`](dashboard/README.md), [`src/team16_sdr`](src/team16_sdr/README.md). Clipped elevation tiles go in `dem/` and are not committed (`dem/README.txt`).

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
The Hailo itself runs in a worker process (`thermal/hailo_proxy.py`), not in the detector. If the chip stops answering (the Hailo `infer()` call hangs), the cameras and the dashboard keep running, the status line says AI STALLED, and after a few seconds the worker is killed and restarted by itself. `thermal/fusion_supervisor.sh` (started by `start_fusion.sh`) is the outer watchdog: if the detector exits, or stops writing frames for 15 s, it is restarted, with a longer wait after repeated quick failures. If the chip cannot be opened at all the detector still runs on the cameras and the heat hotspots. Restarts are logged in `fusion.log`. The thermal feed is capped at 18 fps (the camera delivers 25). The Hailo has stalled most often while the Pluto cellular scan (`src/team16_sdr/`) was running; the cause is not proven, and a hung chip only recovers after a full power-off.

On the Pi (stop other feeds first; only one process can own the Hailo and the Pi camera):

```bash
~/team16_thermal/thermal/start_fusion.sh            # add --record to save paired frames
~/team16_thermal/thermal/stop_fusion.sh
```

Fused view (visible cropped to the thermal field of view) on `:8081`, thermal view on `:8082`, and the
dashboard on `:8090`. The visible camera is rotated 180 degrees and uses the full-field 2304x1296 sensor
mode (the default 1280x720 pick is a centre crop). Align the two cameras with the dashboard's
**Recalibrate** button: stand alone in view of both cameras and walk slowly from one side of the thermal view to the other for up to 30 s (it needs a wide left-to-right spread). It writes
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
