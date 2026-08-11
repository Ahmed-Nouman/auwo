"""One-view pipeline: TF tree + depth->cloud per camera + merge.

Run (after colcon build + source):
  ros2 launch excavator_perception combine_view.launch.py
Optional override:
  ros2 launch excavator_perception combine_view.launch.py urdf:=/abs/path/excavator.urdf

Requires: sim publishing /clock, /joint_states, and the rgb/depth/camera_info
topics named /rgb/camera_{left,right}, /depth/camera_{left,right},
/camera_info/camera_{left,right}.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

USE_SIM_TIME = {'use_sim_time': True}

# sensor_imu_link -> optical-frame extrinsics measured from the USD.
# Re-derive these if the cameras move; do not hand-edit.
LEFT_TF = ['--x', '0.2676', '--y', '0.3833', '--z', '1.5049',
           '--qx', '0.770361', '--qy', '-0.502969',
           '--qz', '0.215171', '--qw', '-0.327517',
           '--frame-id', 'sensor_imu_link', '--child-frame-id', 'sim_camera_left']

RIGHT_TF = ['--x', '0.2592', '--y', '-0.4047', '--z', '1.5049',
            '--qx', '-0.402949', '--qy', '0.834063',
            '--qz', '-0.342735', '--qw', '0.156537',
            '--frame-id', 'sensor_imu_link', '--child-frame-id', 'sim_camera_right']


def cloud_node(side):
    return ComposableNode(
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyzrgbNode',
        name=f'cloud_{side}',
        parameters=[USE_SIM_TIME],
        remappings=[
            ('rgb/image_rect_color', f'/rgb/camera_{side}'),
            ('rgb/camera_info', f'/camera_info/camera_{side}'),
            ('depth_registered/image_rect', f'/depth/camera_{side}'),
            ('points', f'/points/camera_{side}'),
        ])


def generate_launch_description():
    default_urdf = PathJoinSubstitution(
        [FindPackageShare('excavator_description'), 'urdf', 'excavator.urdf.xacro'])
    urdf_arg = DeclareLaunchArgument('urdf', default_value=default_urdf,
                                     description='path to excavator.urdf')
    urdf = LaunchConfiguration('urdf')

    # ParameterValue(value_type=str) prevents launch from YAML-parsing the URDF
    robot_description = ParameterValue(Command(['cat ', urdf]), value_type=str)

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[USE_SIM_TIME, {'robot_description': robot_description}])

    static_l = Node(package='tf2_ros', executable='static_transform_publisher',
                    name='cam_left_tf', arguments=LEFT_TF)
    static_r = Node(package='tf2_ros', executable='static_transform_publisher',
                    name='cam_right_tf', arguments=RIGHT_TF)

    clouds = ComposableNodeContainer(
        name='rgbd_container', namespace='', package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[cloud_node('left'), cloud_node('right')])

    merger = Node(package='excavator_perception', executable='merge_clouds',
                  output='screen',
                  parameters=[USE_SIM_TIME,
                              {'target_frame': 'body', 'stride': 4,
                               'max_range': 20.0}])

    return LaunchDescription([urdf_arg, rsp, static_l, static_r, clouds, merger])
