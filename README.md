# Team 16 ROS 2 workspace

Colcon workspace for ROS 2 Jazzy. Clone this repo and build it on any machine that has Jazzy Desktop installed. Build products (`build/`, `install/`, `log/`) stay local and are not committed.

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

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy Desktop (`/opt/ros/jazzy`)
- `colcon` and `rosdep` (`sudo apt install ros-dev-tools`)

## Build

```bash
cd /path/to/Team-16
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Run

In a new terminal:

```bash
cd /path/to/Team-16
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run team16_bringup hello
```

The `hello` node logs once a second and publishes `ready` on `/team16/status`. Stop it with Ctrl+C.

Add new packages under `src/`, then build again.
