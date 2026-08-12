"""Start ArduCopter SITL and the complete drone ROS stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from diy_autonomous_drone.sitl_support import sim_vehicle_arguments


def generate_launch_description():
    """Launch one simulated quadcopter and connect MAVROS over TCP."""
    package_share = get_package_share_directory('diy_autonomous_drone')
    main_launch_file = os.path.join(
        package_share, 'launch', 'drone_autonomous.launch.py')
    default_parameter_file = os.path.join(
        package_share, 'config', 'params.yaml')

    sim_vehicle_command = LaunchConfiguration('sim_vehicle_command')
    sitl_state_directory = LaunchConfiguration('sitl_state_directory')
    sitl_speedup = LaunchConfiguration('sitl_speedup')
    start_sitl = LaunchConfiguration('start_sitl')

    launch_arguments = [
        DeclareLaunchArgument(
            'sim_vehicle_command',
            default_value='sim_vehicle.py',
            description=(
                'sim_vehicle.py command or absolute path in an ArduPilot '
                'checkout.'),
        ),
        DeclareLaunchArgument(
            'sitl_state_directory',
            default_value='/tmp/diy_autonomous_drone_sitl',
            description='Directory for simulated EEPROM, logs, and state.',
        ),
        DeclareLaunchArgument(
            'sitl_speedup',
            default_value='1',
            description='ArduPilot simulation speed multiplier.',
        ),
        DeclareLaunchArgument(
            'start_sitl',
            default_value='true',
            description=(
                'Start sim_vehicle.py; disable to use an existing SITL.'),
        ),
        DeclareLaunchArgument(
            'stack_start_delay_sec',
            default_value='2.0',
            description='Delay before starting ROS nodes and MAVROS.',
        ),
        DeclareLaunchArgument(
            'parameter_file',
            default_value=default_parameter_file,
            description='ROS parameter YAML file for the simulated stack.',
        ),
        DeclareLaunchArgument(
            'autonomy_mode',
            default_value='hover',
            description='Initial motion mode; hover is the safe default.',
        ),
        DeclareLaunchArgument(
            'enable_vision',
            default_value='false',
            description='Start camera perception inside the VM.',
        ),
        DeclareLaunchArgument(
            'enable_object_detection',
            default_value='false',
            description='Run YOLO when simulated vision is enabled.',
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='Optional MAVROS forwarding URL for a GCS.',
        ),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='tcp://127.0.0.1:5760',
            description='MAVROS URL for the local ArduPilot SITL instance.',
        ),
    ]

    sitl_process = ExecuteProcess(
        cmd=[sim_vehicle_command] + sim_vehicle_arguments(
            vehicle='ArduCopter',
            frame='quad',
            instance='0',
            speedup=sitl_speedup,
            state_directory=sitl_state_directory,
        ),
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_sitl),
    )

    drone_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main_launch_file),
        launch_arguments={
            'parameter_file': LaunchConfiguration('parameter_file'),
            'autonomy_mode': LaunchConfiguration('autonomy_mode'),
            'enable_gesture_control': 'false',
            'enable_vision': LaunchConfiguration('enable_vision'),
            'enable_object_detection': LaunchConfiguration(
                'enable_object_detection'),
            'enable_tracking': 'true',
            'enable_fc_interface': 'true',
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'target_system_id': '1',
            'target_component_id': '1',
        }.items(),
    )

    delayed_stack = TimerAction(
        period=LaunchConfiguration('stack_start_delay_sec'),
        actions=[drone_stack],
    )

    return LaunchDescription(
        launch_arguments + [sitl_process, delayed_stack])
