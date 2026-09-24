#!/usr/bin/env python3
"""AUWO Isaac Sim digital twin (no Gazebo) — RSP + lidar TF + bridge + markers + RViz."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from excavator_models import (
    apply_sim_patches,
    default_model,
    load_profile,
    process_xacro,
    resolve_package_uris,
)

LIDAR_PARENT_FRAME = "sensor_lidar_link"
LIDAR_CHILD_FRAME = "lidar_cab"
LIDAR_XYZ = ("0.189769", "0.027852", "1.425159")
LIDAR_QUAT_XYZW = ("0.325568", "0.0", "0.945519", "-0.0")


def generate_launch_description():
    desc_share = get_package_share_directory("excavator_description")
    rviz_share = get_package_share_directory("excavator_interactive_rviz")

    profile = load_profile(os.environ.get("AUWO_EXCAVATOR_MODEL", ""))
    robot_description = resolve_package_uris(
        apply_sim_patches(
            process_xacro(profile, {"use_mock_hardware": "true", "use_sim": "false"}),
            profile),
        profile, scheme="file")

    use_sim_time = {"use_sim_time": True}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "publish_frequency": 50.0,
            "ignore_timestamp": True,
            **use_sim_time,
        }],
    )

    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_cab_tf",
        output="screen",
        arguments=[
            "--x", LIDAR_XYZ[0], "--y", LIDAR_XYZ[1], "--z", LIDAR_XYZ[2],
            "--qx", LIDAR_QUAT_XYZW[0], "--qy", LIDAR_QUAT_XYZW[1],
            "--qz", LIDAR_QUAT_XYZW[2], "--qw", LIDAR_QUAT_XYZW[3],
            "--frame-id", LIDAR_PARENT_FRAME,
            "--child-frame-id", LIDAR_CHILD_FRAME,
        ],
        parameters=[use_sim_time],
    )

    bridge = Node(
        package="excavator_interactive_rviz",
        executable="isaac_command_bridge.py",
        name="isaac_command_bridge",
        output="screen",
        parameters=[use_sim_time],
    )

    markers = Node(
        package="excavator_interactive_rviz",
        executable="joint_imarkers.py",
        name="excavator_joint_imarkers",
        output="screen",
        parameters=[use_sim_time],
    )

    rviz_cfg = os.path.join(rviz_share, "rviz", "auwo_twin.rviz")
    rviz_args = ["-d", rviz_cfg] if os.path.exists(rviz_cfg) else []
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="auwo_digital_twin_rviz",
        output="screen",
        arguments=rviz_args,
        parameters=[use_sim_time],
    )

    return LaunchDescription([rsp, lidar_tf, bridge, markers, rviz])
