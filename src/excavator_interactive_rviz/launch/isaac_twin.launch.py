#!/usr/bin/env python3
"""
isaac_twin.launch.py — AUWO Isaac Sim twin, sensor configuration driven by YAML.

One launch file for every lidar setup. The configuration lives in
  config/sensors/<sensor_config>.yaml
which defines each lidar's frame, parent, topic and mount pose, plus the
mapping parameters. Add a new setup by adding a YAML file - no launch edits.

Usage:
  ros2 launch excavator_interactive_rviz isaac_twin.launch.py
  ros2 launch excavator_interactive_rviz isaac_twin.launch.py \
      sensor_config:=front_corners run_name:=front_corners__tilt15
  ros2 launch excavator_interactive_rviz isaac_twin.launch.py \
      sensor_config:=cab_only mapping:=false        # teleop only, no mapping

The accumulated map is written to
  ~/Desktop/AUWO-testbed/maps/<run_name>.pcd
when you Ctrl+C the launch.

ISAAC SIDE: open the USD named in the config's `usd_scene` field, confirm
  - SubscribeJointState topicName = /joint_command
  - a ROS2 Publish Clock node exists
  - each RTX Lidar Helper publishes the topic/frame named in the YAML
then press PLAY.

IMPORTANT: the mount poses in the YAML must match the USD prims. After moving
a sensor in Isaac, re-read its pose (XformCache, relative to `parent`) and
update the YAML - a stale extrinsic silently smears the map.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MAP_DIR = os.path.expanduser("~/Desktop/AUWO-testbed/maps")


def build(context, *args, **kwargs):
    pkg = get_package_share_directory("excavator_interactive_rviz")
    desc = get_package_share_directory("excavator_description")

    cfg_name = LaunchConfiguration("sensor_config").perform(context)
    run_name = LaunchConfiguration("run_name").perform(context)
    do_map = LaunchConfiguration("mapping").perform(context).lower() == "true"

    cfg_path = os.path.join(pkg, "config", "sensors", cfg_name + ".yaml")
    if not os.path.exists(cfg_path):
        raise RuntimeError("sensor config not found: " + cfg_path)
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    print("\n[isaac_twin] sensor config : %s" % cfg.get("name", cfg_name))
    print("[isaac_twin] usd scene     : %s" % cfg.get("usd_scene", "?"))
    for l in cfg["lidars"]:
        print("[isaac_twin]   lidar %-10s parent=%-18s topic=%s"
              % (l["frame"], l["parent"], l["topic"]))
    if do_map:
        print("[isaac_twin] map output    : %s.pcd\n"
              % os.path.join(MAP_DIR, run_name))
    else:
        print("[isaac_twin] mapping       : disabled\n")

    use_sim_time = {"use_sim_time": True}
    nodes = []

    xacro_path = os.path.join(desc, "urdf", "excavator.urdf.xacro")
    nodes.append(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", xacro_path]), value_type=str),
            "publish_frequency": 50.0,
            "ignore_timestamp": True,
            **use_sim_time,
        }],
    ))

    # one static TF per lidar in the config
    for l in cfg["lidars"]:
        x, y, z = [str(v) for v in l["xyz"]]
        qx, qy, qz, qw = [str(v) for v in l["quat_xyzw"]]
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=l["frame"] + "_tf", output="screen",
            arguments=[
                "--x", x, "--y", y, "--z", z,
                "--qx", qx, "--qy", qy, "--qz", qz, "--qw", qw,
                "--frame-id", l["parent"], "--child-frame-id", l["frame"],
            ],
            parameters=[use_sim_time],
        ))

    nodes.append(Node(
        package="excavator_interactive_rviz",
        executable="isaac_command_bridge.py",
        name="isaac_command_bridge", output="screen",
        parameters=[use_sim_time],
    ))

    nodes.append(Node(
        package="excavator_interactive_rviz", executable="joint_imarkers.py",
        name="excavator_joint_imarkers", output="screen",
        parameters=[use_sim_time],
    ))

    if do_map:
        m = cfg.get("mapping", {})
        nodes.append(Node(
            package="excavator_interactive_rviz", executable="lidar_mapper.py",
            name="lidar_mapper", output="screen",
            parameters=[{
                "input_topics": [l["topic"] for l in cfg["lidars"]],
                "fixed_frame": "base_link",
                "min_range": float(m.get("min_range", 2.0)),
                "inflate": float(m.get("inflate", 0.15)),
                "voxel_size": float(m.get("voxel_size", 0.10)),
                "save_path": os.path.join(MAP_DIR, run_name + ".pcd"),
                **use_sim_time,
            }],
        ))

    rviz_cfg = os.path.join(pkg, "rviz", "auwo_twin.rviz")
    nodes.append(Node(
        package="rviz2", executable="rviz2", name="auwo_digital_twin_rviz",
        output="screen",
        arguments=(["-d", rviz_cfg] if os.path.exists(rviz_cfg) else []),
        parameters=[use_sim_time],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "sensor_config", default_value="cab_only",
            description="name of config/sensors/<name>.yaml"),
        DeclareLaunchArgument(
            "run_name", default_value="run",
            description="output map filename stem (written under %s)" % MAP_DIR),
        DeclareLaunchArgument(
            "mapping", default_value="true",
            description="run the lidar mapper"),
        OpaqueFunction(function=build),
    ])
