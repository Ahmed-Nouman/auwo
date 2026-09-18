"""MoveIt move_group for AUWO excavator (run after Gazebo + controllers are up)."""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

from excavator_models import default_model
from excavator_models.moveit import moveit_configs


def _ompl_extra_env():
    extra_env = {"DISPLAY": os.environ.get("DISPLAY", "")}
    try:
        ompl_prefix = get_package_prefix("moveit_planners_ompl")
        ament = os.environ.get("AMENT_PREFIX_PATH", "")
        extra_env["AMENT_PREFIX_PATH"] = (
            f"{ompl_prefix}:{ament}" if ament else ompl_prefix
        )
    except PackageNotFoundError:
        pass
    return extra_env


def _bootstrap_move_group_capabilities():
    builder = MoveItConfigsBuilder("excavator", package_name="excavator_moveit_config")
    builder.planning_pipelines(pipelines=["ompl"])
    caps = builder.to_moveit_configs().move_group_capabilities
    return caps["capabilities"], caps["disable_capabilities"]


def _opaque_move_group(context, *args, **kwargs):
    use_mock = context.launch_configurations.get("use_mock_hardware", "false").lower() in (
        "true",
        "1",
    )
    # Model-specific URDF / SRDF / joint limits (v1 identical to the previous builder calls)
    moveit_config, _profile = moveit_configs(
        context.launch_configurations.get("excavator_model", ""),
        use_mock_hardware=use_mock,
        pipelines=["ompl"],
    )
    body_min = float(
        context.launch_configurations.get(
            "body_rotation_planning_min", "-3.141592653589793"
        )
    )
    body_max = float(
        context.launch_configurations.get(
            "body_rotation_planning_max", "3.141592653589793"
        )
    )
    # Keep planning on a near yaw branch (avoid full-turn detours).
    planning = moveit_config.joint_limits.setdefault("robot_description_planning", {})
    joint_limits = planning.setdefault("joint_limits", {})
    body = joint_limits.setdefault("body_rotation", {})
    body.update(
        {
            "has_position_limits": True,
            "min_position": body_min,
            "max_position": body_max,
        }
    )

    use_sim = context.launch_configurations.get("use_sim_time", "true").lower() in (
        "true",
        "1",
    )
    allow_exec = context.launch_configurations.get(
        "allow_trajectory_execution", "true"
    ).lower() in ("true", "1")
    pub_scene = context.launch_configurations.get(
        "publish_monitored_planning_scene", "true"
    ).lower() in ("true", "1")
    capabilities = context.launch_configurations.get("capabilities", "")
    disable_cap = context.launch_configurations.get("disable_capabilities", "")

    move_group_configuration = {
        "use_sim_time": use_sim,
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": allow_exec,
        "capabilities": capabilities,
        "disable_capabilities": disable_cap,
        "publish_planning_scene": pub_scene,
        "publish_geometry_updates": pub_scene,
        "publish_state_updates": pub_scene,
        "publish_transforms_updates": pub_scene,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    trajectory_action = context.launch_configurations.get(
        "trajectory_action",
        "/arm_trajectory_controller/follow_joint_trajectory",
    )
    joint_states_topic = context.launch_configurations.get(
        "joint_states_topic",
        "/joint_states",
    )
    move_group_remappings = [
        (
            "arm_trajectory_controller/follow_joint_trajectory",
            trajectory_action,
        ),
        (
            "joint_states",
            joint_states_topic,
        ),
    ]

    gdb = str(moveit_config.package_path / "launch" / "gdb_settings.gdb")
    extra_env = _ompl_extra_env()

    standard_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        remappings=move_group_remappings,
        additional_env=extra_env,
        condition=UnlessCondition(LaunchConfiguration("debug")),
    )
    debug_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        remappings=move_group_remappings,
        additional_env=extra_env,
        prefix=[f"gdb -x {gdb} --ex run --args"],
        arguments=["--debug"],
        condition=IfCondition(LaunchConfiguration("debug")),
    )
    return [standard_node, debug_node]


def generate_launch_description():
    cap_default, disable_cap_default = _bootstrap_move_group_capabilities()

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "excavator_model",
            default_value=default_model(),
            description=(
                "Excavator model: v1 (excavator_description) or v2 "
                "(excavator_v2_description). Default: $AUWO_EXCAVATOR_MODEL or v2."
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo /clock",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description=(
                "If true, load URDF with mock_components/GenericSystem (RViz-only demo; "
                "no Gazebo)."
            ),
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=cap_default,
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=disable_cap_default,
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "trajectory_action",
            default_value="/arm_trajectory_controller/follow_joint_trajectory",
            description="Absolute FollowJointTrajectory action path used by MoveIt",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/joint_states",
            description="Joint state topic consumed by move_group current state monitor",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "body_rotation_planning_min",
            default_value="-3.141592653589793",
            description="Planning-only lower bound for body_rotation (rad)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "body_rotation_planning_max",
            default_value="3.141592653589793",
            description="Planning-only upper bound for body_rotation (rad)",
        )
    )

    ld.add_action(OpaqueFunction(function=_opaque_move_group))
    return ld
