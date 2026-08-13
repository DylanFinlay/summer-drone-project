"""Tests for the isolated recorded-video launch configuration."""

import unittest

from diy_autonomous_drone.recorded_video import (
    recorded_video_stack_arguments,
)


class TestRecordedVideoConfiguration(unittest.TestCase):
    """Verify video testing cannot accidentally open the FC command path."""

    def test_video_launch_is_active_tracking_but_fc_is_disabled(self):
        """Perception and controls run while MAVROS remains disconnected."""
        arguments = recorded_video_stack_arguments(
            video_file='/tmp/walk.mp4',
            loop_video='true',
            enable_object_detection='true',
            enable_flight_logging='false',
        )
        self.assertEqual(arguments['configuration_profile'], 'bench')
        self.assertEqual(arguments['autonomy_mode'], 'active_track')
        self.assertEqual(arguments['enable_vision'], 'true')
        self.assertEqual(arguments['enable_tracking'], 'true')
        self.assertEqual(arguments['enable_fc_interface'], 'false')
        self.assertEqual(arguments['video_file'], '/tmp/walk.mp4')

    def test_missing_video_launch_value_is_rejected(self):
        """The helper cannot construct an unspecified video source."""
        with self.assertRaises(ValueError):
            recorded_video_stack_arguments(None, False, True, False)


if __name__ == '__main__':
    unittest.main()
