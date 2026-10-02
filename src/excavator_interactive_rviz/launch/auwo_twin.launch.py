"""AUWO digital twin.

  use_gazebo:=false (default) RViz only, ros2_control on mock hardware — for use with
                              Isaac Sim (Isaac subscribes to /joint_states) or on its own
  use_gazebo:=true            Gazebo + gz_ros2_control + RViz

Both modes publish /joint_states and accept the same commands, so the interactive
markers, twin router and trajectory adapter work identically.
"""
import os
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from excavator_models import (
    apply_sim_patches,
    default_model,
    load_profile,
    process_xacro,
    resolve_package_uris,
)


def _write_urdf(profile):
    """URDF of the selected model for RViz / robot_state_publisher (no Gazebo).

    Mock-hardware ros2_control, the model's simulation patches, and file:// mesh
    paths for v2 (same treatment gazebo.launch.py gives its RViz URDF).
    """
    urdf = process_xacro(profile, {"use_mock_hardware": "true", "use_sim": "false"})
    urdf = resolve_package_uris(apply_sim_patches(urdf, profile), profile, scheme="file")
    path = os.path.join(tempfile.gettempdir(),
                        f"excavator_{profile['model']}_standalone.urdf")
    with open(path, "w") as f:
        f.write(urdf)
    return path, urdf


def _setup(context, *args, **kwargs):
    profile = load_profile(context.launch_configurations.get("excavator_model", ""))
    model = profile["model"]
    scene = profile.get("scene", {})
    pkg_gazebo = get_package_share_directory("excavator_gazebo")

    use_excavation_site = LaunchConfiguration("use_excavation_site")
    world_cfg = LaunchConfiguration("world")
    # Gazebo on/off decides sim time and which startup delays are needed
    use_gazebo = str(context.launch_configurations.get("use_gazebo", "false")).lower() in ("true", "1")
    sim_time = use_gazebo

    # -------------------------------------------------------------------------
    # Gazebo: excavation site (world + dump truck)
    #
    # FIX 1: spawn_z lowered from 1.5 -> 0.1 so the robot barely drops before
    #         physics settles it. Dropping from 1.5m caused tumbling/inversion.
    #
    # FIX 2: Explicit spawn_R/P/Y passed so gazebo.launch.py never inherits a
    #         wrong default orientation.
    #
    # FIX 3: controller_spawn_delay_sec forwarded explicitly (was missing —
    #         gazebo.launch.py used its own default but twin launch nodes tried
    #         to connect before controllers finished spawning).
    # -------------------------------------------------------------------------
    gazebo_excavation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_gazebo, "/launch/gazebo.launch.py"],
        ),
        launch_arguments={
            "world": world_cfg,
            "spawn_x": "0.0",
            "spawn_y": "0.0",
            "spawn_z": "0.1",           # FIX 1: was 1.5
            "spawn_R": "0.0",
            "spawn_P": "0.0",
            # spawn_Y=0.0: desired world-facing direction for the robot.
            # gazebo.launch.py applies the model's sim patches (v1: zeroes the
            # base_to_base_link 180° yaw), so RViz and Gazebo agree.
            "spawn_Y": "0.0",
            "excavator_model": model,
            "controller_spawn_delay_sec": "25.0"  # FIX: was 12.0 — /controller_manager needs more time after plugin loads,  # FIX 3: forwarded explicitly
        }.items(),
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("use_gazebo"), "'.lower() in ('true','1') and '",
             use_excavation_site, "'.lower() in ('true','1')"])),
    )

    # --- Gazebo: default empty world (no truck) ---
    gazebo_default = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_gazebo, "/launch/gazebo.launch.py"],
        ),
        launch_arguments={
            "world": world_cfg,
            "spawn_z": "0.1",           # FIX 1: consistent with above
            "spawn_R": "0.0",
            "spawn_P": "0.0",
            "spawn_Y": "0.0",
            "excavator_model": model,
            "controller_spawn_delay_sec": "25.0"  # FIX: was 12.0 — /controller_manager needs more time after plugin loads,  # FIX 3
        }.items(),
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("use_gazebo"), "'.lower() in ('true','1') and '",
             use_excavation_site, "'.lower() not in ('true','1')"])),
    )

    # -------------------------------------------------------------------------
    # RViz — delayed to allow Gazebo, robot_state_publisher, and controllers
    # to be fully active before RViz tries to resolve TF and robot description.
    # FIX: was 4.0s — not enough. Now 15s (controllers finish ~12s + buffer).
    # -------------------------------------------------------------------------
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("excavator_description"), "config", "auwo_twin.rviz"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="auwo_digital_twin_rviz",
        arguments=["-d", rviz_config],
        output="screen",
    )
    # Gazebo needs ~25 s before controllers are up; mock hardware is ready in a few seconds.
    t_controllers = 5.0 if not use_gazebo else None
    t_adapter = 28.0 if use_gazebo else 8.0
    t_interactive = 32.0 if use_gazebo else 10.0
    t_rviz = 30.0 if use_gazebo else 6.0
    rviz_delayed = TimerAction(period=t_rviz, actions=[rviz])

    # NOTE: Gazebo auto-pause removed.
    # Pausing Gazebo during startup causes a deadlock: the controller_manager
    # load_controller service handler needs the sim update loop to tick in order
    # to complete pluginlib initialization. With the sim paused, the service call
    # hangs for 10s and times out, so joint_state_broadcaster and
    # arm_trajectory_controller never load.
    # Use the Gazebo GUI pause button manually after launch if needed.

    # -------------------------------------------------------------------------
    # Interactive markers
    #
    # FIX: Was launched immediately — joint_states not available yet.
    # Delay to 16s: controllers finish at ~12s, joint_state_broadcaster
    # starts publishing /joint_states shortly after. 16s provides safe margin.
    # -------------------------------------------------------------------------
    joint_imarkers = Node(
        package="excavator_interactive_rviz",
        executable="joint_imarkers.py",
        name="excavator_joint_imarkers",
        output="screen",
        parameters=[{"use_sim_time": sim_time, "excavator_model": model}],
    )
    joint_imarkers_delayed = TimerAction(period=t_interactive, actions=[joint_imarkers])

    # -------------------------------------------------------------------------
    # Twin router
    #
    # FIX 1: Was launched immediately — same timing problem as joint_imarkers.
    # FIX 2: sim_command_topic corrected. gazebo.launch.py spawns
    #         arm_trajectory_controller, not arm_position_controller.
    #         The trajectory_command_adapter bridges position cmds ->
    #         JointTrajectory on /arm_trajectory_controller/joint_trajectory.
    #         Router must send to the adapter's INPUT topic, not the controller
    #         directly — keep as /arm_position_controller/commands only if that
    #         topic is what trajectory_command_adapter.py subscribes to.
    #         Verify with: ros2 topic list | grep arm
    # -------------------------------------------------------------------------
    twin_router_node = Node(
        package="excavator_interactive_rviz",
        executable="twin_router_node.py",
        name="excavator_twin_router",
        output="screen",
        parameters=[
            {"use_sim_time": sim_time},
            {"excavator_model": model},
            {"default_mode": "simulation"},
            # This must match trajectory_command_adapter.py's subscribed input topic.
            # If the adapter subscribes to /arm_position_controller/commands, keep as-is.
            # If it subscribes to something else, update here.
            {"sim_command_topic": "/arm_position_controller/commands"},
            {"sim_state_topic": "/joint_states"},
            {"physical_command_topic": "/physical_twin/commands"},
            {"physical_state_topic": "/physical_twin/state"},
        ],
    )
    twin_router_delayed = TimerAction(period=t_interactive, actions=[twin_router_node])

    # -------------------------------------------------------------------------
    # Trajectory adapter
    # Delayed to 14s — needs arm_trajectory_controller active to forward cmds.
    # FIX: was launched immediately.
    # -------------------------------------------------------------------------
    trajectory_adapter = Node(
        package="excavator_teleop",
        executable="trajectory_command_adapter.py",
        name="trajectory_command_adapter",
        output="screen",
        parameters=[{"use_sim_time": sim_time, "excavator_model": model}],
    )
    trajectory_adapter_delayed = TimerAction(period=t_adapter, actions=[trajectory_adapter])

    # -------------------------------------------------------------------------
    # IMU -> Pose for RViz
    #
    # FIX: position_z was 1.5 (matched old spawn_z). With spawn_z now 0.1,
    # this should be 0.1 + the IMU link's z offset from base_link in your URDF.
    # Adjust imu_z_offset to match: `ros2 run tf2_ros tf2_echo base_link imu_link`
    # -------------------------------------------------------------------------
    # Height from the model profile (v1: 0.1 spawn + 0.5 offset = 0.6, v2: roof IMU 2.27)
    imu_pose_z = float(scene.get("imu_pose_z", {}).get("twin", 0.6))
    imu_to_pose = Node(
        package="excavator_gazebo",
        executable="imu_to_pose.py",
        name="imu_to_pose",
        output="screen",
        parameters=[
            {"use_sim_time": sim_time},
            {"pose_frame_id": "world"},
            {"position_x": 0.5},
            {"position_y": 0.0},
            {"position_z": imu_pose_z},
        ],
    )

    # -------------------------------------------------------------------------
    # Point cloud frame remap: /points -> /points_viz as sensor_lidar_link
    # No timing change needed — this is a pure topic relay, stateless.
    # -------------------------------------------------------------------------
    points_frame_remap = Node(
        package="excavator_gazebo",
        executable="points_frame_remap.py",
        name="points_frame_remap",
        output="screen",
        parameters=[
            {"use_sim_time": sim_time},
            {"target_frame_id": "sensor_lidar_link"},
            {"input_topic": "/points"},
            {"output_topic": "/points_viz"},
        ],
    )

    # -------------------------------------------------------------------------
    # No Gazebo (use_gazebo:=false): ros2_control on mock hardware.
    #
    # Gives the same /joint_states and the same command topics as the Gazebo run,
    # so Isaac Sim (subscribing to /joint_states) follows the markers, the twin
    # router or MoveIt without Gazebo running. The Gazebo-only sensor relays
    # (imu_to_pose, points_frame_remap) are skipped: /imu and /points do not exist.
    # -------------------------------------------------------------------------
    standalone = []
    if not use_gazebo:
        urdf_path, urdf_xml = _write_urdf(profile)

        standalone.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": urdf_xml,
                "use_sim_time": False,
                "publish_frequency": 50.0,
            }],
        ))

        # RViz reads /excavator_robot_description (and /truck_robot_description)
        standalone.append(Node(
            package="excavator_gazebo",
            executable="publish_robot_descriptions.py",
            name="publish_robot_descriptions",
            output="screen",
            parameters=[{"excavator_description_file": urdf_path}],
        ))

        # world -> base_link, same frame layout as the Gazebo run
        standalone.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="excavator_world_tf",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "world", "--child-frame-id", "base_link",
            ],
            parameters=[{"use_sim_time": False}],
        ))

        controllers_yaml = profile["controllers_path"]
        standalone.append(Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="both",
            parameters=[controllers_yaml, {"use_sim_time": False}],
            remappings=[("~/robot_description", "/robot_description")],
        ))

        def _spawner(controller):
            return Node(
                package="controller_manager",
                executable="spawner",
                name=f"spawner_{controller}",
                output="screen",
                arguments=[controller, "--controller-manager", "/controller_manager",
                           "--param-file", controllers_yaml],
                parameters=[{"use_sim_time": False}],
            )

        # Chain the spawners (jsb -> arm -> model extras): parallel spawners race
        # the controller_manager, same as in gazebo.launch.py.
        chain = [_spawner("joint_state_broadcaster"), _spawner("arm_trajectory_controller")]
        chain += [_spawner(c) for c in profile["extra_controllers"]]
        chain_handlers = [
            RegisterEventHandler(OnProcessExit(target_action=a, on_exit=[b]))
            for a, b in zip(chain[:-1], chain[1:])
        ]
        standalone.append(TimerAction(period=t_controllers,
                                      actions=[chain[0], *chain_handlers]))

        if profile["linkage"].get("enabled"):
            # v2: hydraulic cylinders + bucket linkage follow the arm joints
            standalone.append(TimerAction(period=t_controllers, actions=[Node(
                package="excavator_models",
                executable="linkage_state_publisher",
                name="linkage_state_publisher",
                output="screen",
                parameters=[{
                    "excavator_model": model,
                    "mode": "controller",
                    "use_sim_time": False,
                }],
            )]))

    return [
        # Gazebo worlds (conditional, and only when use_gazebo:=true)
        gazebo_excavation,
        gazebo_default,
        # Gazebo sensor relays (stateless — start early, no dependency on controllers)
        *([imu_to_pose, points_frame_remap] if use_gazebo else []),
        # ros2_control on mock hardware when Gazebo is off
        *standalone,
        # Trajectory adapter needs an active arm_trajectory_controller
        trajectory_adapter_delayed,
        # RViz — after controllers + TF tree are stable
        rviz_delayed,
        # Interactive nodes — need /joint_states live from joint_state_broadcaster
        joint_imarkers_delayed,
        twin_router_delayed,
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
                "use_gazebo",
                default_value="false",
                description=(
                    "false (default): RViz + ros2_control on mock hardware, e.g. when the "
                    "physics runs in Isaac Sim. true: run Gazebo."
                ),
            ),
            DeclareLaunchArgument(
                "use_excavation_site",
                default_value="true",
                description="Use excavation_site world and spawn dump truck",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=excavation_site_world,
                description="Path to world SDF. Default: excavation_site_local.sdf. Use empty.sdf for plain ground.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
