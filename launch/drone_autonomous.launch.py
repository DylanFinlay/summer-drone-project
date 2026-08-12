"""Launch the complete DIY autonomous drone software stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create a launch description for all autonomous drone nodes."""
    package_name = 'diy_autonomous_drone'
    package_share = get_package_share_directory(package_name)
    default_parameter_file = os.path.join(
        package_share, 'config', 'params.yaml')
    mavros_overrides = os.path.join(
        package_share, 'config', 'mavros_overrides.yaml')
    parameter_file = LaunchConfiguration('parameter_file')
    autonomy_mode = LaunchConfiguration('autonomy_mode')
    mavros_share = FindPackageShare('mavros')

    launch_arguments = [
        DeclareLaunchArgument(
            'parameter_file',
            default_value=default_parameter_file,
            description='ROS parameter YAML file for the drone stack.',
        ),
        DeclareLaunchArgument(
            'autonomy_mode',
            default_value='hover',
            description=(
                'Motion mode: hover, active_track, or gesture_control.'),
        ),
        DeclareLaunchArgument(
            'enable_gesture_control',
            default_value='false',
            description='Allow the experimental gesture-control mode.',
        ),
        DeclareLaunchArgument(
            'enable_vision',
            default_value='false',
            description='Start the camera/perception node.',
        ),
        DeclareLaunchArgument(
            'enable_object_detection',
            default_value='true',
            description='Run YOLO person detection in the vision node.',
        ),
        DeclareLaunchArgument(
            'enable_tracking',
            default_value='true',
            description='Start the autonomous command generator.',
        ),
        DeclareLaunchArgument(
            'enable_fc_interface',
            default_value='true',
            description='Start MAVROS and the command safety adapter.',
        ),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='serial:///dev/ttyAMA0:57600',
            description='MAVROS flight-controller connection URL.',
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='Optional MAVROS ground-station forwarding URL.',
        ),
        DeclareLaunchArgument(
            'target_system_id',
            default_value='1',
            description='MAVLink flight-controller system ID.',
        ),
        DeclareLaunchArgument(
            'target_component_id',
            default_value='1',
            description='MAVLink flight-controller component ID.',
        ),
    ]

    nodes = [
        Node(
            package=package_name,
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[
                parameter_file,
                {
                    'enable_object_detection': ParameterValue(
                        LaunchConfiguration('enable_object_detection'),
                        value_type=bool,
                    ),
                },
            ],
            condition=IfCondition(LaunchConfiguration('enable_vision')),
        ),
        Node(
            package=package_name,
            executable='tracking_bridge_node',
            name='tracking_bridge_node',
            output='screen',
            parameters=[
                parameter_file,
                {
                    'autonomy_mode': ParameterValue(
                        autonomy_mode,
                        value_type=str,
                    ),
                    'enable_gesture_control': ParameterValue(
                        LaunchConfiguration('enable_gesture_control'),
                        value_type=bool,
                    ),
                },
            ],
            condition=IfCondition(LaunchConfiguration('enable_tracking')),
        ),
        Node(
            package=package_name,
            executable='safety_supervisor_node',
            name='safety_supervisor_node',
            output='screen',
            parameters=[parameter_file],
        ),
        Node(
            package=package_name,
            executable='status_node',
            name='status_node',
            output='screen',
            parameters=[
                parameter_file,
                {
                    'expect_fc_interface': ParameterValue(
                        LaunchConfiguration('enable_fc_interface'),
                        value_type=bool,
                    ),
                    'expect_tracking': ParameterValue(
                        LaunchConfiguration('enable_tracking'),
                        value_type=bool,
                    ),
                },
            ],
        ),
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='mavros',
            name='mavros_node',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [mavros_share, 'launch', 'apm_pluginlists.yaml']),
                PathJoinSubstitution(
                    [mavros_share, 'launch', 'apm_config.yaml']),
                mavros_overrides,
                {
                    'fcu_url': ParameterValue(
                        LaunchConfiguration('fcu_url'), value_type=str),
                    'gcs_url': ParameterValue(
                        LaunchConfiguration('gcs_url'), value_type=str),
                    'tgt_system': ParameterValue(
                        LaunchConfiguration('target_system_id'),
                        value_type=int,
                    ),
                    'tgt_component': ParameterValue(
                        LaunchConfiguration('target_component_id'),
                        value_type=int,
                    ),
                    'fcu_protocol': 'v2.0',
                },
            ],
            condition=IfCondition(
                LaunchConfiguration('enable_fc_interface')),
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package=package_name,
            executable='fc_interface_node',
            name='fc_interface_node',
            output='screen',
            parameters=[parameter_file],
            condition=IfCondition(
                LaunchConfiguration('enable_fc_interface')),
            respawn=True,
            respawn_delay=2.0,
        ),
    ]

    return LaunchDescription(launch_arguments + nodes)
