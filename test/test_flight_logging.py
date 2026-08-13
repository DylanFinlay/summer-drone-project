"""Tests for deterministic, bounded flight-data logging configuration."""

from datetime import datetime
import unittest

from diy_autonomous_drone.flight_logging import (
    FLIGHT_LOG_TOPICS,
    default_flight_log_directory,
    rosbag_record_arguments,
)


class TestFlightLogging(unittest.TestCase):
    """Verify unique paths and deliberate topic selection."""

    def test_default_directory_is_timestamped_under_ros_home(self):
        """Default recordings have a predictable user-owned unique path."""
        output = default_flight_log_directory(
            now=datetime(2026, 8, 11, 14, 5, 9),
            home_directory='/home/student',
        )
        self.assertEqual(
            output,
            '/home/student/.ros/diy_autonomous_drone/'
            'flight_20260811_140509_000000',
        )

    def test_record_command_contains_only_explicit_topics(self):
        """The recorder cannot silently expand to high-bandwidth topics."""
        arguments = rosbag_record_arguments('/tmp/flight-test')
        self.assertEqual(arguments[:5], [
            'ros2', 'bag', 'record', '--output', '/tmp/flight-test'])
        self.assertEqual(tuple(arguments[5:]), FLIGHT_LOG_TOPICS)
        self.assertNotIn('-a', arguments)
        self.assertNotIn('/camera/image_raw', arguments)

    def test_critical_decision_topics_are_recorded(self):
        """A bag includes commands, target state, gates, and FC state."""
        required = {
            '/drone/autonomy_mode',
            '/drone/cmd_vel_raw',
            '/drone/cmd_vel_safe',
            '/drone/safety_stop_reason',
            '/drone/fc_gate_reason',
            '/drone/rc_aux_state',
            '/drone/tracking_state',
            '/mavros/state',
        }
        self.assertTrue(required.issubset(set(FLIGHT_LOG_TOPICS)))

    def test_empty_output_directory_is_rejected(self):
        """An invalid log destination fails before starting rosbag."""
        with self.assertRaises(ValueError):
            rosbag_record_arguments('')


if __name__ == '__main__':
    unittest.main()
