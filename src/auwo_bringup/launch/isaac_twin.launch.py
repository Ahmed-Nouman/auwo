#!/usr/bin/env python3
"""
isaac_twin.launch.py — AUWO Isaac Sim twin, sensor configuration driven by YAML.

Lives in auwo_bringup. One launch file for every lidar setup; the configuration
is in config/sensors/<sensor_config>.yaml, which names each lidar's frame,
parent, topic and mount pose plus the mapping parameters. Add a setup by adding
a YAML file - no launch edits.

Usage
  ros2 launch auwo_bringup isaac_twin.launch.py
  ros2 launch auwo_bringup isaac_twin.launch.py \
      sensor_config:=roof_os0_plus_boom run_name:=roof_plus_boom__os0_level
  ros2 launch auwo_bringup isaac_twin.launch.py mapping:=false   # no survey map

The survey map is written to ~/Desktop/AUWO-testbed/maps/<run_name>.pcd when
you Ctrl+C the launch.

ISAAC SIDE: open the USD named in the config's `usd_scene` field, confirm
  - SubscribeJointState topicName = /joint_command
  - a ROS2 Publish Clock node exists
  - each RTX Lidar Helper publishes the topic/frame named in the YAML
then press PLAY.

MARKER REMAPPING - the reason it is here
joint_imarkers.py has /arm_position_controller/commands hardcoded. That topic
must have exactly ONE writer: the arbiter. Rather than patch the node, the
Node() below remaps its output to /auwo/cmd/markers, which the arbiter accepts
while in teleop mode. Same for any other legacy source added later.

Without this remap the markers write straight to the machine, bypassing the
arbiter's mode, e-stop and freeze logic - which defeats the whole point of
having a single writer.

IMPORTANT: the mount poses in the YAML must match the USD prims. After moving a
sensor in Isaac, read its pose back and update the YAML - a stale extrinsic
silently smears the map with no error at all.
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

# the one topic the arbiter owns; every legacy source is remapped off it
MACHINE_CMD = "/arm_position_controller/commands"


def _first_existing(*paths):
    for p in paths:
        if p and os.path.exists(p):
            return p
    return None


def build(context, *args, **kwargs):
    bringup = get_package_share_directory("auwo_bringup")
    desc = get_package_share_directory("excavator_description")

    cfg_name = LaunchConfiguration("sensor_config").perform(context)
    run_name = LaunchConfiguration("run_name").perform(context)
    do_map = LaunchConfiguration("mapping").perform(context).lower() == "true"

    # sensor configs live in auwo_bringup; fall back to the old package while
    # the migration is in progress
    cfg_path = _first_existing(
        os.path.join(bringup, "config", "sensors", cfg_name + ".yaml"),
        os.path.join(get_package_share_directory("excavator_interactive_rviz"),
                     "config", "sensors", cfg_name + ".yaml"),
    )
    if cfg_path is None:
        raise RuntimeError("sensor config not found: %s.yaml" % cfg_name)
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    print("\n[isaac_twin] sensor config : %s" % cfg.get("name", cfg_name))
    print("[isaac_twin] config file   : %s" % cfg_path)
    print("[isaac_twin] usd scene     : %s" % cfg.get("usd_scene", "?"))
    for l in cfg["lidars"]:
        print("[isaac_twin]   lidar %-11s parent=%-18s topic=%s"
              % (l["frame"], l["parent"], l["topic"]))
    print("[isaac_twin] survey map    : %s"
          % (os.path.join(MAP_DIR, run_name + ".pcd") if do_map else "disabled"))
    print()

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

    # one static TF per lidar named in the config
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

    # Float64MultiArray -> JointState, into Isaac                [auwo_control]
    nodes.append(Node(
        package="auwo_control", executable="isaac_command_bridge.py",
        name="isaac_command_bridge", output="screen",
        parameters=[use_sim_time],
    ))

    # RViz joint sliders. REMAPPED off the machine topic - see the note above.
    nodes.append(Node(
        package="excavator_interactive_rviz", executable="joint_imarkers.py",
        name="excavator_joint_imarkers", output="screen",
        remappings=[(MACHINE_CMD, "/auwo/cmd/markers")],
        parameters=[use_sim_time],
    ))

    # survey map: union-forever, for the sensor placement study [auwo_perception]
    if do_map:
        m = cfg.get("mapping", {})
        nodes.append(Node(
            package="auwo_perception", executable="lidar_mapper.py",
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

    rviz_cfg = _first_existing(
        os.path.join(bringup, "config", "rviz", "auwo_twin.rviz"),
        os.path.join(get_package_share_directory("excavator_interactive_rviz"),
                     "rviz", "auwo_twin.rviz"),
    )
    nodes.append(Node(
        package="rviz2", executable="rviz2", name="auwo_digital_twin_rviz",
        output="screen",
        arguments=(["-d", rviz_cfg] if rviz_cfg else []),
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
            description="survey map filename stem, written under %s" % MAP_DIR),
        DeclareLaunchArgument(
            "mapping", default_value="true",
            description="run the survey mapper"),
        OpaqueFunction(function=build),
    ])