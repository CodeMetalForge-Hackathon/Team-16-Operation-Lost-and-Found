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

## Pi Camera 3 color detector

Scripts live in `picam/`. Run from the repository root:

```powershell
$env:PI_HOST="<pi address>"
python picam/picam_color_anomaly.py   # learn office colors, then circle rare ones
python picam/picam_pc_view.py         # plain live view
```

The baseline learning time is the `--learn` default on line 418 of
`picam/picam_color_anomaly.py` (seconds), or pass `--learn <seconds>`.
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
