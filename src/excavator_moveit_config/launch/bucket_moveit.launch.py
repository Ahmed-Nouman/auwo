"""
MoveIt + RViz for the excavator: mock ros2_control by default, Gazebo Harmonic on request.

  # Default: mock ros2_control (no Gazebo), local /controller_manager, wall clock.
  ros2 launch excavator_moveit_config bucket_moveit.launch.py

  # Gazebo: excavator in sim + Plan/Execute, use_sim_time switches on automatically.
  # Gazebo starts paused; press Play in the sim GUI to run physics.
  ros2 launch excavator_moveit_config bucket_moveit.launch.py use_gazebo:=true

  # Original excavator model instead of the new one (works in either mode):
  ros2 launch excavator_moveit_config bucket_moveit.launch.py excavator_model:=v1

  use_sim_time is left empty by default and then follows use_gazebo
  (true with Gazebo, false with mock). Set it explicitly only to override that,
  e.g. mock hardware driven by an external /clock.

  use_gazebo:=false does not attach to an already-running Gazebo. For
  auwo_twin + MoveIt, use a separate launch or domain that connects only to the
  existing sim (avoid two /controller_manager).

  The dump-truck collision box (block in RViz) is off by default; add
  publish_truck_obstacle:=true to have MoveIt plan around the truck.

  Gazebo-only arguments (world, use_excavation_site, publish_truck_obstacle,
  gazebo_*, headless, truck_box_*) are ignored in mock mode.

Defaults to ROS_DOMAIN_ID=0 (standard/default domain).
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

from excavator_models import default_model
from excavator_models.moveit import moveit_configs

TRAJECTORY_ACTION = "/arm_trajectory_controller/follow_joint_trajectory"
CONTROLLER_MANAGER = "/controller_manager"


def _truthy(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def _setup(context, *args, **kwargs):
    pkg_moveit = get_package_share_directory("excavator_moveit_config")
    cfg = context.launch_configurations

    include_gz = _truthy(cfg.get("use_gazebo", "false"))
    sim_time_arg = cfg.get("use_sim_time", "").strip()
    use_sim = _truthy(sim_time_arg) if sim_time_arg else include_gz
    use_sim_str = "true" if use_sim else "false"
    mock_str = "false" if include_gz else "true"
    sim_param = {"use_sim_time": use_sim}

    # Profile (model, controllers, scene, linkage) is the same for both modes; the mock
    # robot_description is only used by the local ros2_control/RSP nodes in mock mode.
    # In Gazebo mode move_group.launch.py builds its own config with use_mock_hardware=false.
    moveit_mock, profile = moveit_configs(
        cfg.get("excavator_model", ""), use_mock_hardware=True, pipelines=["ompl"])
    model = profile["model"]
    scene = profile.get("scene", {})
    robot_desc_mock = moveit_mock.robot_description

    def _scene(arg, key):
        """Launch argument if given, otherwise the model profile value."""
        value = cfg.get(arg, "").strip()
        return float(value) if value else float(scene.get(key, 0.0))

    def _spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", CONTROLLER_MANAGER],
            output="screen",
        )

    # ---- Wait for the arm trajectory action, then move_group (+ RViz) ----
    wait_script = os.path.join(
        get_package_prefix("excavator_moveit_config"),
        "lib",
        "excavator_moveit_config",
        "wait_for_arm_trajectory_action.py",
    )
    wait_action = ExecuteProcess(
        cmd=[
            "python3",
            wait_script,
            "--timeout",
            cfg.get("wait_move_group_timeout_sec", "120.0"),
            "--controller-manager",
            CONTROLLER_MANAGER,
            "--action",
            TRAJECTORY_ACTION,
        ],
        output="screen",
    )

    move_group_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "move_group.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_str,
            "use_mock_hardware": mock_str,
            "excavator_model": model,
            "trajectory_action": TRAJECTORY_ACTION,
            "joint_states_topic": "/joint_states",
            "body_rotation_planning_min": cfg["body_rotation_planning_min"],
            "body_rotation_planning_max": cfg["body_rotation_planning_max"],
        }.items(),
    )

    moveit_rviz_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "moveit_rviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_str,
            "use_mock_hardware": mock_str,
            "excavator_model": model,
            "joint_states_topic": "/joint_states",
        }.items(),
    )

    launch_rviz = _truthy(cfg.get("launch_rviz", "true"))

    def _on_wait_exit(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "[bucket_moveit] wait_for_arm_trajectory_action failed (exit %d)."
                        % event.returncode
                    )
                ),
            ]
        return [move_group_ld] + ([moveit_rviz_ld] if launch_rviz else [])

    actions = [
        LogInfo(msg="[bucket_moveit] mode=%s model=%s use_sim_time=%s"
                % ("gazebo" if include_gz else "mock", model, use_sim_str)),
        RegisterEventHandler(OnProcessExit(target_action=wait_action, on_exit=_on_wait_exit)),
    ]

    if include_gz:
        # ---- Gazebo: ros2_control, controllers, linkage and world TF live in excavator_gazebo ----
        pkg_gazebo = get_package_share_directory("excavator_gazebo")
        gz_args = {
            "world": cfg["world"],
            "use_sim_time": use_sim_str,
            "headless": cfg["headless"],
            "gazebo_unified_gui": cfg["gazebo_unified_gui"],
            "gazebo_verbose": cfg["gazebo_verbose"],
            "controller_spawn_delay_sec": cfg["gazebo_controller_spawn_delay_sec"],
            "excavator_model": model,
        }
        if _truthy(cfg.get("use_excavation_site", "true")):
            # dumper pose: from the excavator model profile (scene section)
            gz_args.update({
                "spawn_x": "0.0",
                "spawn_y": "0.0",
                "spawn_z": "1.5",
                "spawn_dumper": "true",
            })
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, "launch", "gazebo.launch.py"),
            ),
            launch_arguments=gz_args.items(),
        ))

        if _truthy(cfg.get("publish_truck_obstacle", "false")):
            actions.append(Node(
                package="excavator_moveit_config",
                executable="publish_truck_collision_object.py",
                name="truck_collision_object_publisher",
                output="screen",
                parameters=[{
                    **sim_param,
                    "frame_id": "world",
                    "object_id": "dump_truck_box",
                    "position_x": _scene("truck_box_x", "truck_box_x"),
                    "position_y": _scene("truck_box_y", "truck_box_y"),
                    "position_z": _scene("truck_box_z", "truck_box_z"),
                    "yaw": _scene("truck_box_yaw", "truck_box_yaw"),
                    "box_length_x": _scene("truck_box_lx", "truck_box_lx"),
                    "box_length_y": _scene("truck_box_ly", "truck_box_ly"),
                    "box_length_z": _scene("truck_box_lz", "truck_box_lz"),
                }],
            ))

        actions.append(TimerAction(
            period=float(cfg["gazebo_controller_ready_delay_sec"]),
            actions=[wait_action],
        ))

    else:
        # ---- Mock hardware: local ros2_control + spawner chain ----
        actions.append(Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[robot_desc_mock, profile["controllers_path"], sim_param],
        ))
        actions.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_desc_mock, sim_param],
        ))
        # SRDF virtual_joint uses parent_frame "world" -> base_link. Without this, "world" is
        # missing from /tf and the MoveIt RViz planning scene often stays frozen while
        # Plan/Execute still work. (In Gazebo mode excavator_gazebo publishes it.)
        actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--frame-id", "world", "--child-frame-id", "base_link",
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            ],
            parameters=[sim_param],
        ))

        # joint_state_broadcaster -> arm_trajectory_controller -> (wait + model extras in chain)
        # Model extras: v2 blade_controller, linkage_controller.
        spawner_jsb = _spawner("joint_state_broadcaster")
        spawner_arm = _spawner("arm_trajectory_controller")
        extra_spawners = [_spawner(name) for name in profile["extra_controllers"]]
        chain = [
            RegisterEventHandler(OnProcessExit(target_action=spawner_jsb, on_exit=[spawner_arm])),
            RegisterEventHandler(OnProcessExit(
                target_action=spawner_arm, on_exit=[wait_action] + extra_spawners[:1])),
        ]
        chain += [
            RegisterEventHandler(OnProcessExit(target_action=a, on_exit=[b]))
            for a, b in zip(extra_spawners[:-1], extra_spawners[1:])
        ]
        actions.append(TimerAction(period=3.0, actions=[spawner_jsb, *chain]))

        if profile["linkage"].get("enabled"):
            actions.append(Node(
                package="excavator_models",
                executable="linkage_state_publisher",
                name="linkage_state_publisher",
                output="screen",
                parameters=[{
                    "excavator_model": model,
                    "mode": "controller",
                    **sim_param,
                }],
            ))

    return actions


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
                "use_gazebo",
                default_value="false",
                description=(
                    "false (default): local ros2_control mock only. "
                    "true: start excavator_gazebo (gz + controllers in sim). "
                    "Never attaches to an external Gazebo."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="",
                description="Empty: follows use_gazebo. Set true/false to override.",
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
                "body_rotation_planning_min",
                default_value="-3.141592653589793",
                description="Planning-only lower bound for body_rotation (rad)",
            ),
            DeclareLaunchArgument(
                "body_rotation_planning_max",
                default_value="3.141592653589793",
                description="Planning-only upper bound for body_rotation (rad)",
            ),
            # ---- Gazebo-only arguments ----
            DeclareLaunchArgument(
                "use_excavation_site",
                default_value="true",
                description="Gazebo: excavation world spawn pose + dump truck.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=excavation_site_world,
                description="Gazebo: world SDF.",
            ),
            DeclareLaunchArgument(
                "publish_truck_obstacle",
                default_value="false",
                description=(
                    "Gazebo: publish coarse truck box on /collision_object "
                    "(shown as a block in RViz; MoveIt plans around it)."
                ),
            ),
            DeclareLaunchArgument(
                "gazebo_controller_ready_delay_sec",
                default_value="35.0",
                description="Gazebo: seconds before wait script runs (after gz spawn + spawners).",
            ),
            DeclareLaunchArgument(
                "gazebo_controller_spawn_delay_sec",
                default_value="12.0",
                description="Gazebo: forwarded to excavator_gazebo before loading controllers.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Gazebo: forwarded to excavator_gazebo.",
            ),
            DeclareLaunchArgument(
                "gazebo_unified_gui",
                default_value="false",
                description="Gazebo: forwarded to excavator_gazebo.",
            ),
            DeclareLaunchArgument(
                "gazebo_verbose",
                default_value="1",
                description="Gazebo: forwarded to excavator_gazebo.",
            ),
            DeclareLaunchArgument("truck_box_x", default_value="",
                                  description="Gazebo, empty: from the excavator model profile"),
            DeclareLaunchArgument("truck_box_y", default_value="",
                                  description="Gazebo, empty: from the excavator model profile"),
            DeclareLaunchArgument("truck_box_z", default_value="",
                                  description="Gazebo, empty: from the excavator model profile"),
            DeclareLaunchArgument("truck_box_yaw", default_value="",
                                  description="Gazebo, empty: from the excavator model profile"),
            DeclareLaunchArgument("truck_box_lx", default_value="",
                                  description="Gazebo, empty: from the excavator model profile"),
            DeclareLaunchArgument("truck_box_ly", default_value="",
                                  description="Gazebo, empty: from the excavator model profile"),
            DeclareLaunchArgument("truck_box_lz", default_value="",
                                  description="Gazebo, empty: from the excavator model profile"),
            OpaqueFunction(function=_setup),
        ]
    )
