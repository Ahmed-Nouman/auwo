"""
MoveIt + RViz with Gazebo Harmonic by default (excavator in sim + Plan/Execute).

  # Default: Gazebo + use_sim_time (excavation site world unless overridden).
  # Gazebo starts paused; press Play in the sim GUI to run physics.
  ros2 launch excavator_moveit_config bucket_moveit.launch.py
  # Original excavator model instead of the new one:
  ros2 launch excavator_moveit_config bucket_moveit.launch.py excavator_model:=v1

  # Mock ros2_control only (no Gazebo): local /controller_manager + mock hardware
  ros2 launch excavator_moveit_config bucket_moveit.launch.py \\
    include_gazebo:=false use_sim_time:=false

  include_gazebo:=false does not attach to an already-running Gazebo. For
  auwo_twin + MoveIt, use a separate launch or domain that connects only to the
  existing sim (avoid two /controller_manager).

Defaults to ROS_DOMAIN_ID=0 (standard/default domain).
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from excavator_models import default_model
from excavator_models.moveit import moveit_configs


def _setup(context, *args, **kwargs):
    pkg_moveit = get_package_share_directory("excavator_moveit_config")
    pkg_gazebo = get_package_share_directory("excavator_gazebo")

    cfg = context.launch_configurations
    use_sim = LaunchConfiguration("use_sim_time")
    include_gz = LaunchConfiguration("include_gazebo")

    # ---- Mock hardware (no Gazebo): same builder pattern as demo_moveit_rviz ----
    moveit_mock, profile = moveit_configs(
        cfg.get("excavator_model", ""), use_mock_hardware=True, pipelines=["ompl"])
    model = profile["model"]
    scene = profile.get("scene", {})
    controllers_yaml = profile["controllers_path"]
    robot_desc_mock = moveit_mock.robot_description

    def _scene(arg, key):
        """Launch argument if given, otherwise the model profile value."""
        value = cfg.get(arg, "").strip()
        return value if value else str(scene.get(key, 0.0))

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_desc_mock,
            controllers_yaml,
            {"use_sim_time": ParameterValue(use_sim, value_type=bool)},
        ],
        condition=UnlessCondition(include_gz),
    )

    robot_state_publisher_mock = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_desc_mock,
            {"use_sim_time": ParameterValue(use_sim, value_type=bool)},
        ],
        condition=UnlessCondition(include_gz),
    )

    # Same SRDF virtual_joint (world -> base_link) as demo_moveit_rviz; mock stack had no Gazebo excavator_world_tf.
    world_tf_mock = Node(
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
        condition=UnlessCondition(include_gz),
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
        condition=UnlessCondition(include_gz),
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
        condition=UnlessCondition(include_gz),
    )

    # ---- Gazebo (excavator in sim); ros2_control lives inside gz ----
    use_excavation_site = LaunchConfiguration("use_excavation_site")
    world_cfg = LaunchConfiguration("world")

    gazebo_excavation = GroupAction(
        condition=IfCondition(include_gz),
        actions=[
            GroupAction(
                condition=IfCondition(use_excavation_site),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py"),
                        ),
                        launch_arguments={
                            "world": world_cfg,
                            "use_sim_time": use_sim,
                            "headless": LaunchConfiguration("headless"),
                            "gazebo_unified_gui": LaunchConfiguration(
                                "gazebo_unified_gui"
                            ),
                            "gazebo_verbose": LaunchConfiguration("gazebo_verbose"),
                            "controller_spawn_delay_sec": LaunchConfiguration(
                                "gazebo_controller_spawn_delay_sec"
                            ),
                            "spawn_x": "0.0",
                            "spawn_y": "0.0",
                            "spawn_z": "1.5",
                            "spawn_dumper": "true",
                            # dumper pose: from the excavator model profile (scene section)
                            "excavator_model": model,
                        }.items(),
                    ),
                ],
            ),
        ],
    )

    gazebo_default = GroupAction(
        condition=IfCondition(include_gz),
        actions=[
            GroupAction(
                condition=UnlessCondition(use_excavation_site),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py"),
                        ),
                        launch_arguments={
                            "world": world_cfg,
                            "use_sim_time": use_sim,
                            "headless": LaunchConfiguration("headless"),
                            "gazebo_unified_gui": LaunchConfiguration(
                                "gazebo_unified_gui"
                            ),
                            "gazebo_verbose": LaunchConfiguration("gazebo_verbose"),
                            "controller_spawn_delay_sec": LaunchConfiguration(
                                "gazebo_controller_spawn_delay_sec"
                            ),
                            "excavator_model": model,
                        }.items(),
                    ),
                ],
            ),
        ],
    )

    pub_truck = LaunchConfiguration("publish_truck_obstacle")
    truck_collision = GroupAction(
        condition=IfCondition(include_gz),
        actions=[
            GroupAction(
                condition=IfCondition(pub_truck),
                actions=[
                    Node(
                        package="excavator_moveit_config",
                        executable="publish_truck_collision_object.py",
                        name="truck_collision_object_publisher",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(
                                    use_sim, value_type=bool
                                ),
                                "frame_id": "world",
                                "object_id": "dump_truck_box",
                                "position_x": float(_scene("truck_box_x", "truck_box_x")),
                                "position_y": float(_scene("truck_box_y", "truck_box_y")),
                                "position_z": float(_scene("truck_box_z", "truck_box_z")),
                                "yaw": float(_scene("truck_box_yaw", "truck_box_yaw")),
                                "box_length_x": float(_scene("truck_box_lx", "truck_box_lx")),
                                "box_length_y": float(_scene("truck_box_ly", "truck_box_ly")),
                                "box_length_z": float(_scene("truck_box_lz", "truck_box_lz")),
                            }
                        ],
                    ),
                ],
            ),
        ],
    )

    move_group_common = {
        "use_sim_time": use_sim,
        "excavator_model": model,
        "trajectory_action": "/arm_trajectory_controller/follow_joint_trajectory",
        "joint_states_topic": "/joint_states",
        "body_rotation_planning_min": LaunchConfiguration("body_rotation_planning_min"),
        "body_rotation_planning_max": LaunchConfiguration("body_rotation_planning_max"),
    }

    move_group_mock_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "move_group.launch.py")
        ),
        launch_arguments={**move_group_common, "use_mock_hardware": "true"}.items(),
    )

    move_group_sim_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "move_group.launch.py")
        ),
        launch_arguments={**move_group_common, "use_mock_hardware": "false"}.items(),
    )

    moveit_rviz_mock_ld = IncludeLaunchDescription(
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

    moveit_rviz_sim_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "moveit_rviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim,
            "use_mock_hardware": "false",
            "excavator_model": model,
            "joint_states_topic": "/joint_states",
        }.items(),
    )

    wait_script = os.path.join(
        get_package_prefix("excavator_moveit_config"),
        "lib",
        "excavator_moveit_config",
        "wait_for_arm_trajectory_action.py",
    )

    wait_mock = ExecuteProcess(
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

    wait_gazebo = ExecuteProcess(
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

    def _on_wait_mock_exit(event, context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "[bucket_moveit] wait_for_arm_trajectory_action failed (exit %d)."
                        % event.returncode
                    )
                ),
            ]
        actions = [move_group_mock_ld]
        try:
            rviz_on = context.perform_substitution(LaunchConfiguration("launch_rviz"))
        except Exception:
            rviz_on = "true"
        if str(rviz_on).lower() in ("true", "1"):
            actions.append(moveit_rviz_mock_ld)
        return actions

    def _on_wait_gazebo_exit(event, context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "[bucket_moveit] wait_for_arm_trajectory_action failed (exit %d)."
                        % event.returncode
                    )
                ),
            ]
        actions = [move_group_sim_ld]
        try:
            rviz_on = context.perform_substitution(LaunchConfiguration("launch_rviz"))
        except Exception:
            rviz_on = "true"
        if str(rviz_on).lower() in ("true", "1"):
            actions.append(moveit_rviz_sim_ld)
        return actions

    when_wait_mock_done = RegisterEventHandler(
        OnProcessExit(target_action=wait_mock, on_exit=_on_wait_mock_exit),
        condition=UnlessCondition(include_gz),
    )

    after_jsb_spawn_arm = RegisterEventHandler(
        OnProcessExit(target_action=spawner_jsb, on_exit=[spawner_arm]),
        condition=UnlessCondition(include_gz),
    )

    # Model extras for the mock path (v2: blade_controller, linkage_controller).
    # In the Gazebo path excavator_gazebo spawns them.
    extra_spawners_mock = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
            condition=UnlessCondition(include_gz),
        )
        for name in profile["extra_controllers"]
    ]
    after_arm_spawn_wait_mock = RegisterEventHandler(
        OnProcessExit(target_action=spawner_arm,
                      on_exit=[wait_mock] + extra_spawners_mock[:1]),
        condition=UnlessCondition(include_gz),
    )
    extra_chain_mock = [
        RegisterEventHandler(OnProcessExit(target_action=a, on_exit=[b]),
                             condition=UnlessCondition(include_gz))
        for a, b in zip(extra_spawners_mock[:-1], extra_spawners_mock[1:])
    ]

    spawn_controllers_mock = TimerAction(
        period=3.0,
        actions=[spawner_jsb, after_jsb_spawn_arm, after_arm_spawn_wait_mock,
                 *extra_chain_mock],
        condition=UnlessCondition(include_gz),
    )

    # v2 linkage node for the mock path (Gazebo path: started by excavator_gazebo)
    model_nodes_mock = []
    if profile["linkage"].get("enabled"):
        model_nodes_mock.append(Node(
            package="excavator_models",
            executable="linkage_state_publisher",
            name="linkage_state_publisher",
            output="screen",
            parameters=[{
                "excavator_model": model,
                "mode": "controller",
                "use_sim_time": ParameterValue(use_sim, value_type=bool),
            }],
            condition=UnlessCondition(include_gz),
        ))

    when_wait_gazebo_done = RegisterEventHandler(
        OnProcessExit(target_action=wait_gazebo, on_exit=_on_wait_gazebo_exit),
        condition=IfCondition(include_gz),
    )

    delayed_wait_gazebo = TimerAction(
        period=LaunchConfiguration("gazebo_controller_ready_delay_sec"),
        actions=[wait_gazebo],
        condition=IfCondition(include_gz),
    )

    return [
        # Mock path (demo-style)
        when_wait_mock_done,
        ros2_control_node,
        robot_state_publisher_mock,
        world_tf_mock,
        spawn_controllers_mock,
        *model_nodes_mock,
        # Gazebo path (move_group + RViz only after wait_for_arm_trajectory_action succeeds)
        when_wait_gazebo_done,
        gazebo_excavation,
        gazebo_default,
        delayed_wait_gazebo,
        truck_collision,
    ]


def generate_launch_description():
    pkg_desc = get_package_share_directory("excavator_description")
    excavation_site_world = os.path.join(pkg_desc, "worlds", "excavation_site_local.sdf")
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
                description="ROS domain for this stack.",
            ),
            SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
            DeclareLaunchArgument(
                "include_gazebo",
                default_value="true",
                description=(
                    "If true: start excavator_gazebo (gz + controllers in sim). "
                    "If false: local ros2_control mock only (not external Gazebo attach)."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo /clock when include_gazebo is true; set false for mock-only.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start MoveIt RViz after the arm trajectory wait succeeds.",
            ),
            DeclareLaunchArgument(
                "wait_move_group_timeout_sec",
                default_value="120.0",
                description="Seconds to wait for active controllers + FollowJointTrajectory action.",
            ),
            DeclareLaunchArgument(
                "use_excavation_site",
                default_value="true",
                description="When include_gazebo: excavation world + dump truck spawn.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=excavation_site_world,
                description="World SDF when include_gazebo true",
            ),
            DeclareLaunchArgument(
                "publish_truck_obstacle",
                default_value="true",
                description="Publish coarse truck box on /collision_object (Gazebo path).",
            ),
            DeclareLaunchArgument(
                "gazebo_controller_ready_delay_sec",
                default_value="35.0",
                description=(
                    "include_gazebo: seconds before wait script runs (after gz spawn + spawners)."
                ),
            ),
            DeclareLaunchArgument(
                "gazebo_controller_spawn_delay_sec",
                default_value="12.0",
                description="Forwarded to excavator_gazebo before loading controllers.",
            ),
            DeclareLaunchArgument(
                "body_rotation_planning_min",
                default_value="-3.141592653589793",
                description="Planning-only lower bound for body_rotation (rad)",
            ),
            DeclareLaunchArgument(
                "body_rotation_planning_max",
                default_value="3.141592653589793",
                description="Planning-only upper bound for body_rotation (rad)",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Forwarded to excavator_gazebo.",
            ),
            DeclareLaunchArgument(
                "gazebo_unified_gui",
                default_value="false",
                description="Forwarded to excavator_gazebo.",
            ),
            DeclareLaunchArgument(
                "gazebo_verbose",
                default_value="1",
                description="Forwarded to excavator_gazebo.",
            ),
            DeclareLaunchArgument("truck_box_x", default_value="",
                                  description="empty: from the excavator model profile"),
            DeclareLaunchArgument("truck_box_y", default_value="",
                                  description="empty: from the excavator model profile"),
            DeclareLaunchArgument("truck_box_z", default_value="",
                                  description="empty: from the excavator model profile"),
            DeclareLaunchArgument("truck_box_yaw", default_value="",
                                  description="empty: from the excavator model profile"),
            DeclareLaunchArgument("truck_box_lx", default_value="",
                                  description="empty: from the excavator model profile"),
            DeclareLaunchArgument("truck_box_ly", default_value="",
                                  description="empty: from the excavator model profile"),
            DeclareLaunchArgument("truck_box_lz", default_value="",
                                  description="empty: from the excavator model profile"),
            OpaqueFunction(function=_setup),
        ]
    )
