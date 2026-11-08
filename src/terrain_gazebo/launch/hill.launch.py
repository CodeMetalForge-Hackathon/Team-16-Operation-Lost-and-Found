"""Launch a headless hill world, bridge the lidar, and run the localizer.

The sensor is not driven. Set its pose with:

  gz service -s /world/hill/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \\
    --req 'name: "pod" position {x: 400 y: 1500 z: 520}'
"""

import os
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _world(context, *args, **kwargs):
    from terrain_localize.heightmap import _write_png, load_heightmap

    map_yaml = Path(LaunchConfiguration("map_yaml").perform(context))

    heightmap = load_heightmap(map_yaml)
    width = heightmap.width * heightmap.resolution
    depth = heightmap.height * heightmap.resolution
    z_min = float(heightmap.z.min())
    z_max = float(heightmap.z.max())
    z_span = max(z_max - z_min, 1.0)
    # Ogre2 requires the heightmap width times sampling to be a power of two.
    side = 256
    ys = np.linspace(0, heightmap.height - 1, side)
    xs = np.linspace(0, heightmap.width - 1, side)
    y0 = np.floor(ys).astype(int)
    x0 = np.floor(xs).astype(int)
    y1 = np.clip(y0 + 1, 0, heightmap.height - 1)
    x1 = np.clip(x0 + 1, 0, heightmap.width - 1)
    wy = (ys - y0)[:, None]
    wx = (xs - x0)[None, :]
    grid = heightmap.z
    sampled = (
        grid[y0][:, x0] * (1.0 - wy) * (1.0 - wx)
        + grid[y0][:, x1] * (1.0 - wy) * wx
        + grid[y1][:, x0] * wy * (1.0 - wx)
        + grid[y1][:, x1] * wy * wx
    )
    image = np.round((sampled - z_min) / z_span * 255.0).astype(np.uint8)
    png = Path("/tmp/terrain_hill.png")
    _write_png(png, image)
    # Gazebo centers the heightmap on the model. Black is the bottom of the box.
    center_x = width * 0.5
    center_y = depth * 0.5
    center_z = z_min + z_span * 0.5
    world_path = Path("/tmp/terrain_hill.sdf")
    world_path.write_text(
        f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="hill">
    <physics name="static" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 800 0 0.6 0</pose>
      <diffuse>1 1 1 1</diffuse>
    </light>
    <model name="terrain">
      <static>true</static>
      <pose>{center_x} {center_y} {center_z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>{png}</uri>
              <size>{width} {depth} {z_span}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <uri>{png}</uri>
              <size>{width} {depth} {z_span}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <material>
            <ambient>0.45 0.38 0.28 1</ambient>
            <diffuse>0.45 0.38 0.28 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="pod">
      <pose>400 1500 520 0 0 0</pose>
      <link name="lidar_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.01</iyy><iyz>0</iyz>
            <izz>0.01</izz>
          </inertia>
        </inertial>
        <sensor name="lidar" type="gpu_lidar">
          <always_on>1</always_on>
          <update_rate>2</update_rate>
          <topic>lidar</topic>
          <gz_frame_id>lidar_link</gz_frame_id>
          <lidar>
            <scan>
              <horizontal>
                <samples>180</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
              </horizontal>
              <vertical>
                <samples>16</samples>
                <resolution>1</resolution>
                <min_angle>-1.2</min_angle>
                <max_angle>-0.4</max_angle>
              </vertical>
            </scan>
            <range>
              <min>0.5</min>
              <max>250</max>
              <resolution>0.1</resolution>
            </range>
          </lidar>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
"""
    )
    return [
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "-r", "--headless-rendering", str(world_path)],
            output="screen",
            additional_env={"LIBGL_ALWAYS_SOFTWARE": os.environ.get("LIBGL_ALWAYS_SOFTWARE", "0")},
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
            output="screen",
        ),
        Node(
            package="terrain_localize",
            executable="localizer",
            name="terrain_localizer",
            output="screen",
            parameters=[{"map_yaml": str(map_yaml)}],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("terrain_localize"))
    default_map = str(share / "maps" / "synthetic_hill.yaml")
    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share / "maps")),
            DeclareLaunchArgument("map_yaml", default_value=default_map),
            OpaqueFunction(function=_world),
        ]
    )
