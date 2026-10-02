#!/usr/bin/env python3
# Gazebo Harmonic launcher for the AUWO excavator (ROS 2 Jazzy).
# The excavator is the only model spawned (no dump truck, no pete_environment).
# excavator_model:=v1 (excavator_description) or v2 (excavator_v2_description).
import os
import shutil
import subprocess
import tempfile
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    TimerAction,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    TextSubstitution,
    Command,
    PathJoinSubstitution,
    FindExecutable,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

from excavator_models import (
    apply_sim_patches,
    default_model,
    load_profile,
    process_xacro,
    resolve_package_uris,
)

# Same transport partition for gz sim, bridges, and ros_gz_sim create (must match).
_GZ_TRANSPORT_ENV = {
    'GZ_PARTITION': 'auwo_sim',
    'IGN_PARTITION': 'auwo_sim',
    # FIX: GZ_SIM_SYSTEM_PLUGIN_PATH was missing from the gz sim server environment.
    # Without it, Gazebo cannot find libgz_ros2_control-system.so even though it
    # exists in /opt/ros/jazzy/lib — Gazebo uses this var, not LD_LIBRARY_PATH,
    # to locate system plugins. This caused gz_ros2_control to silently not load,
    # leaving /controller_manager unreachable and /joint_states unpublished.
    'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib',
}

MINIMAL_WORLD_SDF = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="auwo_empty">
    <gravity>0 0 -9.81</gravity>
    <physics name="ode" type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1.0</ambient>
      <background>0.7 0.7 0.7 1.0</background>
    </scene>
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
"""

def _ensure_local_world(path: str) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        with open(path, "w") as f:
            f.write(MINIMAL_WORLD_SDF)
    return path

def _register_local_model(pkg_share: str):
    """Create ~/.gz/models/excavator_description so model:// URIs can resolve locally."""
    home = os.path.expanduser("~")
    models_root = os.path.join(home, ".gz", "models")
    model_root = os.path.join(models_root, "excavator_description")
    meshes_src = os.path.join(pkg_share, "meshes")
    meshes_dst = os.path.join(model_root, "meshes")

    os.makedirs(models_root, exist_ok=True)
    os.makedirs(model_root, exist_ok=True)

    config_path = os.path.join(model_root, "model.config")
    if not os.path.exists(config_path):
        with open(config_path, "w") as f:
            f.write("""<?xml version="1.0"?>
<model>
  <name>excavator_description</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>local</name><email>none@example.com</email></author>
  <description>Local shim model for ROS package meshes</description>
</model>
""")

    sdf_path = os.path.join(model_root, "model.sdf")
    if not os.path.exists(sdf_path):
        with open(sdf_path, "w") as f:
            f.write("""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="excavator_description">
    <static>true</static>
    <link name="dummy"/>
  </model>
</sdf>
""")

    # Ensure meshes symlink points to package meshes
    if os.path.lexists(meshes_dst):
        if os.path.islink(meshes_dst):
            target = os.readlink(meshes_dst)
            if target != meshes_src:
                os.unlink(meshes_dst)
        else:
            shutil.rmtree(meshes_dst)
    if not os.path.exists(meshes_dst):
        try:
            os.symlink(meshes_src, meshes_dst)
        except FileExistsError:
            pass


def _excavator_urdf_files(profile, model_override: str = '') -> tuple[str, str]:
    """Write the RViz/RSP URDF and the Gazebo URDF of the selected excavator model.

    RViz/RSP URDF : package:// kept (v1) or rewritten to file:// (v2, profile flag).
    Gazebo URDF   : package:// replaced by absolute paths.
    Both get the model's simulation-only patches (v1: base_to_base_link yaw and bucket
    visual pitch zeroed, exactly as this launch file did before; v2: none).
    Returns (rviz_path, gazebo_path).
    """
    if model_override:
        profile = dict(profile, xacro_path=model_override)
    if not os.path.isfile(profile['xacro_path']):
        raise FileNotFoundError(f"Excavator xacro not found: {profile['xacro_path']}")
    urdf = apply_sim_patches(process_xacro(profile, {'use_sim': 'true'}), profile)

    suffix = '' if profile['model'] == 'v1' else '_' + profile['model']
    tmp = tempfile.gettempdir()
    rviz_path = os.path.join(tmp, f'excavator{suffix}_rviz.urdf')
    gz_path = os.path.join(tmp, f'excavator{suffix}_resolved.urdf')
    with open(rviz_path, 'w') as f:
        f.write(resolve_package_uris(urdf, profile, scheme='file'))
    with open(gz_path, 'w') as f:
        f.write(resolve_package_uris(urdf, profile, scheme='path', force=True))
    return rviz_path, gz_path


def _launch_setup(context, *args, **kwargs):
    """Everything that depends on the selected excavator model."""
    pkg_name = 'excavator_description'   # shared worlds / models / Gazebo model shim
    pkg_share = get_package_share_directory(pkg_name)

    cfg = context.launch_configurations
    profile = load_profile(cfg.get('excavator_model', ''))

    _register_local_model(pkg_share)

    # Excavator URDFs for RViz/RSP and for Gazebo (model-specific sim patches applied)
    excavator_urdf_path = None
    excavator_rviz_path = os.path.join(tempfile.gettempdir(), 'excavator_rviz.urdf')
    try:
        excavator_rviz_path, excavator_urdf_path = _excavator_urdf_files(
            profile, cfg.get('model', '').strip())
    except Exception as e:  # noqa: BLE001 - keep launching, report clearly
        print(f'[excavator_gazebo] excavator URDF generation failed: {e}')
    print(f'[excavator_gazebo] excavator model {profile["model"]} ({profile["package"]})')

    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_R = LaunchConfiguration('spawn_R')
    spawn_P = LaunchConfiguration('spawn_P')
    spawn_Y = LaunchConfiguration('spawn_Y')
    controller_spawn_delay_sec = LaunchConfiguration('controller_spawn_delay_sec', default='25.0')

    # Resource paths for Gazebo (package://<description>/... needs the share parent)
    user_models = os.path.join(os.path.expanduser("~"), ".gz", "models")
    share_parent = os.path.dirname(pkg_share)  # so package://excavator_description/meshes resolves
    resource_parts = [
        EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
        TextSubstitution(text=':' + share_parent),
        TextSubstitution(text=':' + pkg_share),
        TextSubstitution(text=':' + os.path.join(pkg_share, 'meshes')),
        TextSubstitution(text=':' + os.path.join(pkg_share, 'models')),
        TextSubstitution(text=':' + os.path.join(pkg_share, 'worlds')),
        TextSubstitution(text=':' + user_models),
    ]
    if profile['share'] != pkg_share:
        resource_parts += [
            TextSubstitution(text=':' + os.path.dirname(profile['share'])),
            TextSubstitution(text=':' + profile['share']),
        ]
    set_gz_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_parts)

    # RSP uses the patched RViz URDF (v1: base_to_base_link rpy="0 0 0") so TF
    # chain is consistent with Gazebo physics — eliminates -180° roll on bucket.
    with open(excavator_rviz_path, 'r') as _f:
        _rsp_urdf = _f.read()

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': _rsp_urdf,
            'use_sim_time': False,
            'publish_frequency': 50.0,
            'ignore_timestamp': True,
        }]
    )

    # world -> base_link static TF at spawn pose. z=0.0: spawn_z is the drop height;
    # the robot settles to the ground. No yaw offset: v1's base_to_base_link 180° yaw
    # is zeroed in the sim URDFs, v2 has none.
    excavator_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='excavator_world_tf',
        arguments=[
            '--x', spawn_x, '--y', spawn_y, '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0',
            '--yaw', spawn_Y,
            '--frame-id', 'world', '--child-frame-id', 'base_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    spawn_Y_gz = spawn_Y

    # Publish the excavator URDF on /excavator_robot_description for RViz (the node also
    # publishes a minimal /truck_robot_description so the RViz truck display stays valid)
    desc_publisher = Node(
        package='excavator_gazebo',
        executable='publish_robot_descriptions.py',
        name='publish_robot_descriptions',
        output='screen',
        parameters=[{
            'excavator_description_file': excavator_rviz_path,
        }],
    )

    # Spawn excavator from the pre-resolved URDF file (package:// already replaced).
    # Using -file avoids gz transport service-discovery issues that -topic can hit.
    if excavator_urdf_path is not None:
        spawn_source = ['-file', excavator_urdf_path]
    else:
        spawn_source = ['-topic', 'robot_description']
    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='create',
        output='screen',
        arguments=[
            '-world', 'default',
            *spawn_source,
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-R', spawn_R, '-P', spawn_P, '-Y', spawn_Y_gz,
        ],
        additional_env=_GZ_TRANSPORT_ENV,
    )

    # ---- Auto-spawn controllers after spawn has run ----
    controllers_yaml = profile['controllers_path']

    def _spawner(controller):
        return ExecuteProcess(
            cmd=[
                'ros2', 'run', 'controller_manager', 'spawner',
                controller,
                '--controller-manager', '/controller_manager',
                '--param-file', controllers_yaml,
                '--controller-manager-timeout', '30',
                '--switch-timeout', '15',
            ],
            output='screen'
        )

    spawner_jsb = _spawner('joint_state_broadcaster')
    spawner_arm = _spawner('arm_trajectory_controller')

    # NOTE: arm_position_controller is declared in controllers.yaml but intentionally
    # NOT spawned here. It is not a hardware controller — /arm_position_controller/commands
    # is just a ROS topic that trajectory_command_adapter subscribes to. Spawning it as a
    # ros2_control controller would conflict with arm_trajectory_controller over the same
    # joints (both claim position command interfaces on the same 4 joints).
    #
    # The command flow is:
    #   twin_router → /arm_position_controller/commands (Float64MultiArray, topic only)
    #   → trajectory_command_adapter → /arm_trajectory_controller/joint_trajectory
    #   → arm_trajectory_controller (the actual active hardware controller)

    # Chain spawners: JSB first, then trajectory controller, then model extras
    # (v2: blade_controller, linkage_controller). Parallel spawners race controller_manager.
    chain = [spawner_jsb, spawner_arm] + [_spawner(c) for c in profile['extra_controllers']]
    chain_handlers = [
        RegisterEventHandler(OnProcessExit(target_action=a, on_exit=[b]))
        for a, b in zip(chain[:-1], chain[1:])
    ]

    # Controllers after model is in world and /clock bridge is up.
    spawn_after_gz = TimerAction(
        period=controller_spawn_delay_sec,
        actions=[spawner_jsb, *chain_handlers],
    )

    model_actions = []
    if profile['linkage'].get('enabled'):
        # v2: hydraulic cylinders + bucket linkage follow the arm joints
        model_actions.append(Node(
            package='excavator_models',
            executable='linkage_state_publisher',
            name='linkage_state_publisher',
            output='screen',
            parameters=[{
                'excavator_model': profile['model'],
                'mode': 'controller',
                'use_sim_time': use_sim_time,
            }],
        ))

    actions = [
        set_gz_resource_path,
        rsp,
        excavator_world_tf,
        desc_publisher,
        spawner,
        spawn_after_gz,
        *model_actions,
    ]
    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('excavator_description')
    pkg_gazebo_share = get_package_share_directory('excavator_gazebo')

    # Local world (avoid Fuel/network)
    default_world = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    world_file = _ensure_local_world(default_world)

    # ---- Args ----
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    unified_gui = LaunchConfiguration('gazebo_unified_gui')
    gazebo_verbose = LaunchConfiguration('gazebo_verbose')
    physics_engine = LaunchConfiguration('physics_engine')  # selector

    excavator_model_arg = DeclareLaunchArgument(
        'excavator_model',
        default_value=default_model(),
        description=(
            'Excavator model: v1 (excavator_description, original) or v2 '
            '(excavator_v2_description, new). Default from AUWO_EXCAVATOR_MODEL, else v2.'
        ),
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to local .sdf/.world'
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='',
        description='Optional path to a top-level Xacro/URDF (overrides the excavator_model xacro)'
    )
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='excavator')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')

    gazebo_unified_gui_arg = DeclareLaunchArgument(
        'gazebo_unified_gui',
        default_value='false',
        description=(
            'If true (and headless:=false), run one gz sim process (GUI+server). '
            'If false, use gz sim -s then gz sim -g (default; reliable ros_gz_sim create).'
        ),
    )
    gazebo_verbose_arg = DeclareLaunchArgument(
        'gazebo_verbose',
        default_value='1',
        description='Gazebo log verbosity for gz sim -v (0–4). Lower = less console spam.',
    )

    physics_engine_arg = DeclareLaunchArgument(
        'physics_engine',
        default_value='gz-physics-bullet-featherstone-plugin',
        description='Physics engine plugin (e.g., gz-physics-bullet-featherstone-plugin or gz-physics-dartsim-plugin)'
    )

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='1.5')
    spawn_R_arg = DeclareLaunchArgument('spawn_R', default_value='0.0')
    spawn_P_arg = DeclareLaunchArgument('spawn_P', default_value='0.0')
    spawn_Y_arg = DeclareLaunchArgument('spawn_Y', default_value='0.0')

    # The dump truck is no longer spawned. These arguments are kept so existing
    # launch files and scripts that still pass them keep working; they do nothing.
    spawn_dumper_arg = DeclareLaunchArgument(
        'spawn_dumper', default_value='false', description='Deprecated: no truck is spawned.')
    dumper_x_arg = DeclareLaunchArgument('dumper_x', default_value='', description='Deprecated.')
    dumper_y_arg = DeclareLaunchArgument('dumper_y', default_value='', description='Deprecated.')
    dumper_z_arg = DeclareLaunchArgument('dumper_z', default_value='', description='Deprecated.')
    dumper_yaw_arg = DeclareLaunchArgument('dumper_yaw', default_value='', description='Deprecated.')

    controller_spawn_delay_sec_arg = DeclareLaunchArgument(
        'controller_spawn_delay_sec',
        default_value='25.0',  # FIX: was 12.0 — plugin now loads correctly but needs more time to initialize /controller_manager
        description=(
            'Seconds after Gazebo starts before spawning joint_state_broadcaster and '
            'arm_trajectory_controller (allow model + /clock bridge to be ready).'
        ),
    )

    # Same Gazebo Transport partition for gz sim + bridge so /excavator/imu and /excavator/points are visible to the bridge
    set_gz_partition = SetEnvironmentVariable(name='GZ_PARTITION', value='auwo_sim')

    # FIX: Set GZ_SIM_SYSTEM_PLUGIN_PATH so Gazebo can find libgz_ros2_control-system.so.
    # Gazebo Harmonic resolves system plugins via this env var, not LD_LIBRARY_PATH.
    # Without it the gz_ros2_control plugin silently fails to load → no /controller_manager.
    set_gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value=''),
            TextSubstitution(text=':/opt/ros/jazzy/lib'),
        ]
    )

    # Split: server (-s) + delayed GUI (-g). Unified: single process (some GPUs/desktops are smoother).
    # No -r: start paused (use Play in the Gazebo GUI to run); -r would auto-run on start.
    gz_server = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-s', '-v', gazebo_verbose,
            '--physics-engine', physics_engine, world,
        ],
        output='screen',
        additional_env=_GZ_TRANSPORT_ENV,
    )
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        additional_env=_GZ_TRANSPORT_ENV,
        condition=UnlessCondition(headless),
    )
    gz_gui_delayed = TimerAction(period=3.0, actions=[gz_gui])
    gz_unified = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-v', gazebo_verbose,
            '--physics-engine', physics_engine, world,
        ],
        output='screen',
        additional_env=_GZ_TRANSPORT_ENV,
    )

    use_unified_gui = PythonExpression(
        [
            "'",
            headless,
            "'.lower() not in ('true', '1') and '",
            unified_gui,
            "'.lower() in ('true', '1')",
        ]
    )
    use_split_gui = PythonExpression(
        [
            "'",
            headless,
            "'.lower() in ('true', '1') or '",
            unified_gui,
            "'.lower() not in ('true', '1')",
        ]
    )
    gz_unified_group = GroupAction(
        actions=[gz_unified],
        condition=IfCondition(use_unified_gui),
    )
    gz_split_group = GroupAction(
        actions=[gz_server, gz_gui_delayed],
        condition=IfCondition(use_split_gui),
    )

    # Bridge nodes must use use_sim_time:=false so they do not block waiting for /clock while publishing it.
    clock_bridge_config = os.path.join(pkg_gazebo_share, 'config', 'clock_bridge.yaml')
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='bridge_node',
        name='auwo_clock_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': False, 'config_file': clock_bridge_config},
        ],
        additional_env=_GZ_TRANSPORT_ENV,
        arguments=['--ros-args', '--log-level', 'info'],
    )
    sensors_bridge_config = os.path.join(pkg_gazebo_share, 'config', 'sensors_bridge.yaml')
    sensors_bridge = Node(
        package='ros_gz_bridge',
        executable='bridge_node',
        name='auwo_sensor_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': False, 'config_file': sensors_bridge_config},
        ],
        additional_env=_GZ_TRANSPORT_ENV,
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # Spawn immediately — gz transport service discovery requires the create node
    # to join the network alongside the server to catch multicast announcements.
    # The create node retries internally until the service appears.
    bridge_clock_early = TimerAction(period=1.5, actions=[clock_bridge])
    bridge_sensors_delayed = TimerAction(period=5.0, actions=[sensors_bridge])

    return LaunchDescription([
        excavator_model_arg,
        world_arg, model_arg, robot_name_arg, use_sim_time_arg, headless_arg,
        gazebo_unified_gui_arg,
        gazebo_verbose_arg,
        physics_engine_arg,
        spawn_x_arg, spawn_y_arg, spawn_z_arg, spawn_R_arg, spawn_P_arg, spawn_Y_arg,
        spawn_dumper_arg, dumper_x_arg, dumper_y_arg, dumper_z_arg, dumper_yaw_arg,
        controller_spawn_delay_sec_arg,
        set_gz_partition,
        set_gz_plugin_path,
        # Model-dependent: resource path, RSP, TF, description topics, spawn, controllers
        OpaqueFunction(function=_launch_setup),
        gz_unified_group,
        gz_split_group,
        bridge_clock_early,
        bridge_sensors_delayed,
    ])
