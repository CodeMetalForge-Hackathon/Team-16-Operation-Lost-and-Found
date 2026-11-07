# Team 16 ROS 2 workspace

Colcon workspace for ROS 2 Jazzy. Clone this repo and build it on any machine that has Jazzy Desktop installed. Build products (`build/`, `install/`, `log/`) stay local and are not committed.

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
