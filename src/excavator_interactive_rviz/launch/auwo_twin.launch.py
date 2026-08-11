import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_desc = get_package_share_directory("excavator_description")
    pkg_gazebo = get_package_share_directory("excavator_gazebo")
    empty_world = os.path.join(pkg_desc, "worlds", "empty.sdf")
    excavation_site_world = os.path.join(pkg_desc, "worlds", "excavation_site_local.sdf")

    use_excavation_site = LaunchConfiguration("use_excavation_site")
    world_cfg = LaunchConfiguration("world")

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
            # gazebo.launch.py internally adds +3.14159 to the Gazebo spawner
            # to compensate for the URDF base_to_base_link 180° fixed joint,
            # while publishing the static TF at spawn_Y so RViz renders correctly.
            "spawn_Y": "0.0",
            "spawn_dumper": "true",
            "dumper_x": "4.0",
            "dumper_y": "3.0",
            "dumper_z": "0.5",
            "dumper_yaw": "0.0",
            "controller_spawn_delay_sec": "25.0"  # FIX: was 12.0 — /controller_manager needs more time after plugin loads,  # FIX 3: forwarded explicitly
        }.items(),
        condition=IfCondition(use_excavation_site),
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
            "spawn_Y": "0.0",  # gazebo.launch.py handles the +3.14159 offset internally
            "controller_spawn_delay_sec": "25.0"  # FIX: was 12.0 — /controller_manager needs more time after plugin loads,  # FIX 3
        }.items(),
        condition=UnlessCondition(use_excavation_site),
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
    rviz_delayed = TimerAction(period=30.0, actions=[rviz])  # FIX: was 4.0s

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
        parameters=[{"use_sim_time": True}],
    )
    joint_imarkers_delayed = TimerAction(period=32.0, actions=[joint_imarkers])

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
            {"use_sim_time": True},
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
    twin_router_delayed = TimerAction(period=32.0, actions=[twin_router_node])

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
        parameters=[{"use_sim_time": True}],
    )
    trajectory_adapter_delayed = TimerAction(period=28.0, actions=[trajectory_adapter])

    # -------------------------------------------------------------------------
    # IMU -> Pose for RViz
    #
    # FIX: position_z was 1.5 (matched old spawn_z). With spawn_z now 0.1,
    # this should be 0.1 + the IMU link's z offset from base_link in your URDF.
    # Adjust imu_z_offset to match: `ros2 run tf2_ros tf2_echo base_link imu_link`
    # -------------------------------------------------------------------------
    imu_z_offset = 0.5  # <- tune this: z distance from ground to your IMU link
    imu_to_pose = Node(
        package="excavator_gazebo",
        executable="imu_to_pose.py",
        name="imu_to_pose",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"pose_frame_id": "world"},
            {"position_x": 0.5},
            {"position_y": 0.0},
            {"position_z": 0.1 + imu_z_offset},  # FIX: was hardcoded 1.5
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
            {"use_sim_time": True},
            {"target_frame_id": "sensor_lidar_link"},
            {"input_topic": "/points"},
            {"output_topic": "/points_viz"},
        ],
    )

    return LaunchDescription(
        [
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
            # Gazebo worlds (conditional)
            # gazebo_excavation,
            # gazebo_default,
            # Sensor relay nodes (stateless — start early, no dependency on controllers)
            imu_to_pose,
            points_frame_remap,
            # Trajectory adapter needs active arm_trajectory_controller
            trajectory_adapter_delayed,
            # RViz — after controllers + TF tree are stable
            rviz_delayed,
            # Interactive nodes — need /joint_states live from joint_state_broadcaster
            joint_imarkers_delayed,
            twin_router_delayed,
        ]
    )
