"""RViz2 + MotionPlanning plugin (AUWO excavator)."""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

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


def _opaque_rviz(context, *args, **kwargs):
    use_mock = context.launch_configurations.get("use_mock_hardware", "false").lower() in (
        "true",
        "1",
    )
    # Model-specific URDF / SRDF / joint limits (v1 identical to the previous builder calls)
    moveit_config, _profile = moveit_configs(
        context.launch_configurations.get("excavator_model", ""),
        use_mock_hardware=use_mock,
        pipelines=["ompl"],
        load_all=False,
    )

    use_sim = context.launch_configurations.get("use_sim_time", "true").lower() in (
        "true",
        "1",
    )
    rviz_config = context.launch_configurations.get(
        "rviz_config",
        str(moveit_config.package_path / "config/moveit.rviz"),
    )
    joint_states_topic = context.launch_configurations.get(
        "joint_states_topic",
        "/joint_states",
    )

    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": use_sim},
    ]

    extra_env = _ompl_extra_env()

    standard_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", rviz_config],
        parameters=rviz_parameters,
        remappings=[("joint_states", joint_states_topic)],
        additional_env=extra_env,
        condition=UnlessCondition(LaunchConfiguration("debug")),
    )
    debug_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", rviz_config],
        parameters=rviz_parameters,
        remappings=[("joint_states", joint_states_topic)],
        additional_env=extra_env,
        prefix=["gdb --ex run --args"],
        condition=IfCondition(LaunchConfiguration("debug")),
    )
    return [standard_node, debug_node]


def generate_launch_description():
    builder = MoveItConfigsBuilder("excavator", package_name="excavator_moveit_config")
    builder.planning_pipelines(pipelines=["ompl"], load_all=False)
    default_rviz = str(builder.to_moveit_configs().package_path / "config/moveit.rviz")

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
            description="Match move_group: mock URDF for RViz-only demos.",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz,
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/joint_states",
            description="Joint state topic for RViz robot model / MotionPlanning plugin",
        )
    )

    ld.add_action(OpaqueFunction(function=_opaque_rviz))
    return ld
