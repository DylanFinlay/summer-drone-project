"""ROS integration tests for live tracking-bridge mode changes."""

import unittest


try:
    import rclpy
    from geometry_msgs.msg import Pose2D
    from rclpy.parameter import Parameter

    from diy_autonomous_drone.tracking_bridge_node import TrackingBridgeNode
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class CapturingPublisher:
    """Capture messages sent by the node during a mode transition."""

    def __init__(self):
        """Start with no captured messages."""
        self.messages = []

    def publish(self, message):
        """Record one published message."""
        self.messages.append(message)


@unittest.skipUnless(ROS_AVAILABLE, 'ROS 2 Python packages are unavailable')
class TestTrackingBridgeModeChanges(unittest.TestCase):
    """Exercise the ROS parameter API when ROS 2 is installed."""

    def test_live_mode_transitions_stop_and_clear_inputs(self):
        """Accepted transitions stop; unsafe requests are rejected."""
        owns_context = not rclpy.ok()
        if owns_context:
            rclpy.init()

        node = TrackingBridgeNode()
        publisher = CapturingPublisher()
        node._command_publisher = publisher
        try:
            node._latest_target = Pose2D()
            node._latest_target_time = node.get_clock().now()
            node._tracking_filter.update((0.5, 0.0, 0.2))
            node._velocity_limiter.limit(
                (1.0, 0.0, 0.0, 0.0), 0.0)
            node._velocity_limiter.limit(
                (1.0, 0.0, 0.0, 0.0), 0.1)

            result = node.set_parameters([
                Parameter('autonomy_mode', value='active_track'),
            ])[0]
            self.assertTrue(result.successful)
            self.assertEqual(node._mode, node.MODE_ACTIVE_TRACK)
            self.assertIsNone(node._latest_target)
            self.assertIsNone(node._tracking_filter.current)
            self.assertEqual(
                node._velocity_limiter.current,
                (0.0, 0.0, 0.0, 0.0),
            )
            self.assertEqual(len(publisher.messages), 1)

            result = node.set_parameters([
                Parameter('autonomy_mode', value='unknown'),
            ])[0]
            self.assertFalse(result.successful)
            self.assertEqual(node._mode, node.MODE_ACTIVE_TRACK)
            self.assertEqual(len(publisher.messages), 1)

            result = node.set_parameters([
                Parameter('autonomy_mode', value='gesture_control'),
            ])[0]
            self.assertFalse(result.successful)

            result = node.set_parameters([
                Parameter('enable_gesture_control', value=True),
            ])[0]
            self.assertTrue(result.successful)
            result = node.set_parameters([
                Parameter('autonomy_mode', value='gesture_control'),
            ])[0]
            self.assertTrue(result.successful)
            self.assertEqual(node._mode, node.MODE_GESTURE_CONTROL)
            self.assertEqual(len(publisher.messages), 2)

            result = node.set_parameters([
                Parameter('enable_gesture_control', value=False),
            ])[0]
            self.assertFalse(result.successful)
            self.assertTrue(node._gesture_enabled)
        finally:
            node.destroy_node()
            if owns_context:
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
