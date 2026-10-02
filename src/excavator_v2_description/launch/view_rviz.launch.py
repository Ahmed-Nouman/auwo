#!/usr/bin/env python3
"""View the v2 excavator in RViz with sliders for the actuated joints.

The hydraulic cylinders and bucket linkage follow the sliders
(excavator_models/linkage_state_publisher, display mode).

    ros2 launch excavator_v2_description view_rviz.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from excavator_models import apply_sim_patches, load_profile, process_xacro, resolve_package_uris


def generate_launch_description():
    share = get_package_share_directory('excavator_v2_description')
    prof = load_profile('v2')
    urdf = process_xacro(prof, {'use_mock_hardware': 'true'})
    urdf = resolve_package_uris(apply_sim_patches(urdf, prof), prof, scheme='file')

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             arguments=[prof['control_urdf_path']],
             remappings=[('joint_states', 'joint_commands'),
                         ('robot_description', 'control_robot_description')]),
        Node(package='excavator_models', executable='linkage_state_publisher',
             parameters=[{'excavator_model': 'v2', 'mode': 'display'}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'view.rviz')]),
    ])
