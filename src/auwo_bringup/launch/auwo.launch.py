#!/usr/bin/env python3
"""
auwo.launch.py — the whole stack in one command.

  ros2 launch auwo_bringup auwo.launch.py

Starts the twin (robot_state_publisher, sensor TFs, Isaac bridge, RViz), the
control chain (arbiter + joystick teleop), the terrain map, and the dig-region
adapter. Replaces isaac_twin.launch.py, which can be deleted.

ONE THING STAYS SEPARATE
keyboard_to_joy reads the keyboard directly, so it needs its own terminal with
focus. Launch cannot give a node a TTY. Run it alongside:

    ros2 run auwo_control keyboard_to_joy.py

With a real gamepad, pass joy:=true instead and nothing separate is needed.

COMMON INVOCATIONS

  # everything, default sensor rig
  ros2 launch auwo_bringup auwo.launch.py

  # a different sensor configuration
  ros2 launch auwo_bringup auwo.launch.py sensor_config:=boom_only

  # sensor placement study: survey map on, control off
  ros2 launch auwo_bringup auwo.launch.py \
      survey_map:=true terrain_map:=false teleop:=false operator:=false \
      run_name:=roof_plus_boom__level

  # watch only, no chance of commanding the machine
  ros2 launch auwo_bringup auwo.launch.py mode:=idle

  # real gamepad instead of the keyboard
  ros2 launch auwo_bringup auwo.launch.py joy:=true

ISAAC SIDE, before launching: open the USD named in the sensor config, check
the SubscribeJointState node is on /joint_command, that a ROS2 Publish Clock
node exists, and that each RTX Lidar Helper publishes the topic and frame named
in the YAML. Then press PLAY.

TWO MAPS, DELIBERATELY
  survey_map   lidar_mapper      union-forever, for the coverage study
  terrain_map  terrain_map_node  latest-wins, for the operator
They answer different questions and both are correct. See the build guide.
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

# the one topic the arbiter owns; every other source is remapped off it
MACHINE_CMD = "/arm_position_controller/commands"


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


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
    mode = LaunchConfiguration("mode").perform(context)

    do_survey = _flag(context, "survey_map")
    do_terrain = _flag(context, "terrain_map")
    do_teleop = _flag(context, "teleop")
    do_operator = _flag(context, "operator")
    do_rviz = _flag(context, "rviz")
    do_joy = _flag(context, "joy")
    do_foxglove = _flag(context, "foxglove")

    cfg_path = _first_existing(
        os.path.join(bringup, "config", "sensors", cfg_name + ".yaml"),
        os.path.join(get_package_share_directory("excavator_interactive_rviz"),
                     "config", "sensors", cfg_name + ".yaml"))
    if cfg_path is None:
        raise RuntimeError("sensor config not found: %s.yaml" % cfg_name)
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    lidar_topics = [l["topic"] for l in cfg["lidars"]]
    m = cfg.get("mapping", {})

    print("\n" + "=" * 66)
    print("AUWO  |  sensor rig: %s" % cfg.get("name", cfg_name))
    print("=" * 66)
    print("  usd scene    %s" % cfg.get("usd_scene", "?"))
    for l in cfg["lidars"]:
        print("  lidar        %-11s parent=%-18s %s"
              % (l["frame"], l["parent"], l["topic"]))
    print("  control      %s" % ("arbiter + teleop, mode=%s" % mode
                                 if do_teleop else "OFF"))
    print("  terrain map  %s" % ("on" if do_terrain else "off"))
    print("  survey map   %s" % (os.path.join(MAP_DIR, run_name + ".pcd")
                                 if do_survey else "off"))
    print("  dig region   %s" % ("on" if do_operator else "off"))
    if do_foxglove:
        print("  foxglove     ws://localhost:8765")
    if do_teleop and not do_joy:
        print("\n  RUN SEPARATELY (it needs a focused terminal):")
        print("      ros2 run auwo_control keyboard_to_joy.py")
    print("=" * 66 + "\n")

    use_sim_time = {"use_sim_time": True}
    nodes = []

    # ---------------------------------------------------------------- twin
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
        }]))

    for l in cfg["lidars"]:
        x, y, z = [str(v) for v in l["xyz"]]
        qx, qy, qz, qw = [str(v) for v in l["quat_xyzw"]]
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=l["frame"] + "_tf", output="screen",
            arguments=["--x", x, "--y", y, "--z", z,
                       "--qx", qx, "--qy", qy, "--qz", qz, "--qw", qw,
                       "--frame-id", l["parent"], "--child-frame-id", l["frame"]],
            parameters=[use_sim_time]))

    nodes.append(Node(
        package="auwo_control", executable="isaac_command_bridge.py",
        name="isaac_command_bridge", output="screen",
        parameters=[use_sim_time]))

    # RViz joint sliders, remapped OFF the machine topic so the arbiter stays
    # the only writer
    nodes.append(Node(
        package="excavator_interactive_rviz", executable="joint_imarkers.py",
        name="excavator_joint_imarkers", output="screen",
        remappings=[(MACHINE_CMD, "/auwo/cmd/markers")],
        parameters=[use_sim_time]))

    # ------------------------------------------------------------- control
    # No use_sim_time on these two: they run wall-clock loops on purpose, and
    # a safety timeout must not depend on a clock the simulator can pause.
    if do_teleop:
        nodes.append(Node(
            package="auwo_control", executable="auwo_arbiter.py",
            name="auwo_arbiter", output="screen",
            parameters=[{"start_mode": mode}]))
        nodes.append(Node(
            package="auwo_control", executable="auwo_joy_teleop.py",
            name="auwo_joy_teleop", output="screen"))
        if do_joy:
            nodes.append(Node(package="joy", executable="joy_node",
                              name="joy_node", output="screen"))

    # ------------------------------------------------------------ perception
    if do_terrain:
        nodes.append(Node(
            package="auwo_perception", executable="terrain_map_node.py",
            name="terrain_map_node", output="screen",
            parameters=[{
                "input_topics": lidar_topics,      # straight from the rig config
                "fixed_frame": "base_link",
                "min_range": float(m.get("min_range", 0.5)),
                "inflate": float(m.get("inflate", 0.15)),
                "resolution": 0.15,
                **use_sim_time,
            }]))

    if do_survey:
        nodes.append(Node(
            package="auwo_perception", executable="lidar_mapper.py",
            name="lidar_mapper", output="screen",
            parameters=[{
                "input_topics": lidar_topics,
                "fixed_frame": "base_link",
                "min_range": float(m.get("min_range", 0.5)),
                "inflate": float(m.get("inflate", 0.15)),
                "voxel_size": float(m.get("voxel_size", 0.10)),
                "save_path": os.path.join(MAP_DIR, run_name + ".pcd"),
                **use_sim_time,
            }]))

    # -------------------------------------------------------------- operator
    if do_operator:
        nodes.append(Node(
            package="auwo_operator", executable="clicked_point_adapter.py",
            name="clicked_point_adapter", output="screen",
            parameters=[{"fixed_frame": "base_link"}]))

    # -------------------------------------------------------------- foxglove
    # The dashboard connects over websocket, so it needs nothing installed and
    # can run on another machine. foxglove_bridge rather than rosbridge: the
    # latter struggles with high-rate topics and large messages, which is
    # exactly the point clouds and camera feeds.
    if do_foxglove:
        nodes.append(Node(
            package="foxglove_bridge", executable="foxglove_bridge",
            name="foxglove_bridge", output="screen",
            parameters=[{
                "port": 8765,
                "address": "0.0.0.0",          # reachable from another machine
                "send_buffer_limit": 100000000,
                "use_compression": True,
                "max_qos_depth": 10,
                **use_sim_time,
            }]))

    # ------------------------------------------------------------------ rviz
    if do_rviz:
        rviz_cfg = _first_existing(
            os.path.join(bringup, "config", "rviz", "auwo.rviz"),
            os.path.join(bringup, "config", "rviz", "auwo_twin.rviz"),
            os.path.join(get_package_share_directory("excavator_interactive_rviz"),
                         "rviz", "auwo_twin.rviz"))
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="auwo_rviz",
            output="screen",
            arguments=(["-d", rviz_cfg] if rviz_cfg else []),
            parameters=[use_sim_time]))

    return nodes


def generate_launch_description():
    a = DeclareLaunchArgument
    return LaunchDescription([
        a("sensor_config", default_value="roof_plus_boom",
          description="name of config/sensors/<name>.yaml"),
        a("run_name", default_value="run",
          description="survey map filename stem, written under %s" % MAP_DIR),
        a("mode", default_value="teleop",
          description="arbiter start mode: idle | teleop | auto"),
        a("teleop", default_value="true", description="arbiter + joystick teleop"),
        a("terrain_map", default_value="true", description="operator map"),
        a("survey_map", default_value="false", description="coverage-study map"),
        a("operator", default_value="true", description="dig region adapter"),
        a("rviz", default_value="true"),
        a("joy", default_value="false", description="start joy_node for a gamepad"),
        a("foxglove", default_value="true",
          description="foxglove_bridge websocket on port 8765"),
        OpaqueFunction(function=build),
    ])