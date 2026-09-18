"""
MoveIt + RViz only: mock ros2_control (no Gazebo).

Use this to verify Plan and Execute against a local FollowJointTrajectory server
and consistent /joint_states.

  ros2 launch excavator_moveit_config demo_moveit_rviz.launch.py
  ros2 launch excavator_moveit_config demo_moveit_rviz.launch.py excavator_model:=v1

This launch defaults to ROS_DOMAIN_ID=0 (standard/default domain).
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from excavator_models import default_model
from excavator_models.moveit import moveit_configs


def _setup(context, *args, **kwargs):
    pkg_moveit = get_package_share_directory("excavator_moveit_config")

    use_sim = LaunchConfiguration("use_sim_time")

    moveit_config, profile = moveit_configs(
        context.launch_configurations.get("excavator_model", ""),
        use_mock_hardware=True,
        pipelines=["ompl"],
    )
    model = profile["model"]
    controllers_yaml = profile["controllers_path"]
    robot_desc_dict = moveit_config.robot_description

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_desc_dict,
            controllers_yaml,
            {"use_sim_time": ParameterValue(use_sim, value_type=bool)},
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_desc_dict,
            {"use_sim_time": ParameterValue(use_sim, value_type=bool)},
        ],
    )

    # SRDF virtual_joint uses parent_frame "world" -> base_link. Without this, "world" is missing
    # from /tf and the MoveIt RViz planning scene often stays frozen while Plan/Execute still work.
    world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
        ],
        parameters=[{"use_sim_time": ParameterValue(use_sim, value_type=bool)}],
    )

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    wait_script = os.path.join(
        get_package_prefix("excavator_moveit_config"),
        "lib",
        "excavator_moveit_config",
        "wait_for_arm_trajectory_action.py",
    )
    wait_trajectory_action = ExecuteProcess(
        cmd=[
            "python3",
            wait_script,
            "--timeout",
            LaunchConfiguration("wait_move_group_timeout_sec"),
            "--controller-manager",
            "/controller_manager",
            "--action",
            "/arm_trajectory_controller/follow_joint_trajectory",
        ],
        output="screen",
    )

    move_group_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "move_group.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim,
            "use_mock_hardware": "true",
            "excavator_model": model,
            "trajectory_action": "/arm_trajectory_controller/follow_joint_trajectory",
            "joint_states_topic": "/joint_states",
        }.items(),
    )

    moveit_rviz_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "moveit_rviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim,
            "use_mock_hardware": "true",
            "excavator_model": model,
            "joint_states_topic": "/joint_states",
        }.items(),
    )

    def _on_wait_exit(event, context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "[demo_moveit_rviz] wait_for_arm_trajectory_action failed (exit %d)."
                        % event.returncode
                    )
                ),
            ]
        actions = [move_group_ld]
        try:
            rviz_on = context.perform_substitution(LaunchConfiguration("launch_rviz"))
        except Exception:
            rviz_on = "true"
        if str(rviz_on).lower() in ("true", "1"):
            actions.append(moveit_rviz_ld)
        return actions

    when_wait_done = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_trajectory_action,
            on_exit=_on_wait_exit,
        )
    )

    after_jsb_spawn_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=spawner_jsb,
            on_exit=[spawner_arm],
        )
    )

    # Model extras (v2: blade_controller, linkage_controller), spawned one after another
    # once arm_trajectory_controller is up, in parallel with the trajectory-action wait.
    extra_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )
        for name in profile["extra_controllers"]
    ]
    after_arm_spawn_wait = RegisterEventHandler(
        OnProcessExit(
            target_action=spawner_arm,
            on_exit=[wait_trajectory_action] + extra_spawners[:1],
        )
    )
    extra_chain = [
        RegisterEventHandler(OnProcessExit(target_action=a, on_exit=[b]))
        for a, b in zip(extra_spawners[:-1], extra_spawners[1:])
    ]

    spawn_controllers = TimerAction(
        period=3.0,
        actions=[spawner_jsb, after_jsb_spawn_arm, after_arm_spawn_wait, *extra_chain],
    )

    model_nodes = []
    if profile["linkage"].get("enabled"):
        model_nodes.append(Node(
            package="excavator_models",
            executable="linkage_state_publisher",
            name="linkage_state_publisher",
            output="screen",
            parameters=[{
                "excavator_model": model,
                "mode": "controller",
                "use_sim_time": ParameterValue(use_sim, value_type=bool),
            }],
        ))

    return [
        when_wait_done,
        ros2_control_node,
        robot_state_publisher,
        world_tf,
        spawn_controllers,
        *model_nodes,
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "excavator_model",
                default_value=default_model(),
                description=(
                    "Excavator model: v1 (excavator_description) or v2 "
                    "(excavator_v2_description). Default: $AUWO_EXCAVATOR_MODEL or v2."
                ),
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="0",
                description="ROS domain for mock MoveIt demo.",
            ),
            SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Wall clock for mock hardware (set true only if driving /clock).",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start RViz with MotionPlanning after controllers are ready.",
            ),
            DeclareLaunchArgument(
                "wait_move_group_timeout_sec",
                default_value="120.0",
                description="Seconds to wait for arm trajectory action + active controllers.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
