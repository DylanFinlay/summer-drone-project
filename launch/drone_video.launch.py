"""Run active-tracking perception against a local video with no FC link."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from diy_autonomous_drone.recorded_video import (
    recorded_video_stack_arguments,
)


def generate_launch_description():
    """Start the safe perception pipeline for one required video file."""
    package_share = get_package_share_directory('diy_autonomous_drone')
    main_launch_file = os.path.join(
        package_share, 'launch', 'drone_autonomous.launch.py')

    launch_arguments = [
        DeclareLaunchArgument(
            'video_file',
            description='Absolute or working-directory-relative video path.',
        ),
        DeclareLaunchArgument(
            'loop_video',
            default_value='false',
            description='Loop video after clearing target identity state.',
        ),
        DeclareLaunchArgument(
            'enable_object_detection',
            default_value='true',
            description='Run the configured YOLO detector on video frames.',
        ),
        DeclareLaunchArgument(
            'autonomy_mode',
            default_value='active_track',
            description='Use active_track or gesture_control for playback.',
        ),
        DeclareLaunchArgument(
            'enable_gesture_control',
            default_value='false',
            description='Unlock experimental gesture command generation.',
        ),
        DeclareLaunchArgument(
            'enable_gesture_recognition',
            default_value='false',
            description='Run YOLO pose gesture recognition on the video.',
        ),
        DeclareLaunchArgument(
            'enable_flight_logging',
            default_value='false',
            description='Record perception and command decisions to rosbag.',
        ),
    ]

    perception_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main_launch_file),
        launch_arguments=recorded_video_stack_arguments(
            video_file=LaunchConfiguration('video_file'),
            loop_video=LaunchConfiguration('loop_video'),
            enable_object_detection=LaunchConfiguration(
                'enable_object_detection'),
            enable_flight_logging=LaunchConfiguration(
                'enable_flight_logging'),
            autonomy_mode=LaunchConfiguration('autonomy_mode'),
            enable_gesture_control=LaunchConfiguration(
                'enable_gesture_control'),
            enable_gesture_recognition=LaunchConfiguration(
                'enable_gesture_recognition'),
        ).items(),
    )

    return LaunchDescription(launch_arguments + [perception_stack])
