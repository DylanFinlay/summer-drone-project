"""Deterministic rosbag configuration for flight-data capture."""

from datetime import datetime
import os


FLIGHT_LOG_TOPICS = (
    '/drone/active_gesture',
    '/drone/autonomy_mode',
    '/drone/cmd_vel_raw',
    '/drone/cmd_vel_safe',
    '/drone/fc_gate_reason',
    '/drone/rc_aux_state',
    '/drone/safety_stop_reason',
    '/drone/status',
    '/drone/target_tracking_box',
    '/drone/target_visible',
    '/drone/tracking_state',
    '/diagnostics',
    '/mavros/setpoint_velocity/cmd_vel',
    '/mavros/state',
)


def default_flight_log_directory(now=None, home_directory=None):
    """Return a unique, user-owned default output path for one launch."""
    timestamp = now or datetime.now()
    home = home_directory or os.path.expanduser('~')
    return os.path.join(
        home,
        '.ros',
        'diy_autonomous_drone',
        'flight_%s' % timestamp.strftime('%Y%m%d_%H%M%S_%f'),
    )


def rosbag_record_arguments(output_directory):
    """Build the explicit-topic rosbag command used by launch files."""
    if output_directory is None or (
        isinstance(output_directory, str) and not output_directory.strip()
    ):
        raise ValueError('flight log output directory cannot be empty')
    return [
        'ros2',
        'bag',
        'record',
        '--output',
        output_directory,
        *FLIGHT_LOG_TOPICS,
    ]
