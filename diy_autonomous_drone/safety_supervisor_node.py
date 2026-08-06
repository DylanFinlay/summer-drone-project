"""Gate raw velocity commands behind target and command watchdogs."""

from typing import Optional

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node


class SafetySupervisorNode(Node):
    """Publish only commands that pass timeout and proximity checks."""

    def __init__(self) -> None:
        """Declare safety parameters and establish ROS interfaces."""
        super().__init__('safety_supervisor_node')

        self.declare_parameter('watchdog_timeout_sec', 0.5)
        self.declare_parameter('min_safety_box_height', 0.85)

        self._watchdog_timeout = max(
            0.05,
            float(self.get_parameter('watchdog_timeout_sec').value),
        )
        self._min_safety_box_height = float(
            self.get_parameter('min_safety_box_height').value)

        self._last_command: Optional[Twist] = None
        self._last_command_time = None
        self._last_target_time = None
        self._target_too_close = False
        self._fault_reason: Optional[str] = None

        self._safe_command_publisher = self.create_publisher(
            Twist, '/drone/cmd_vel_safe', 10)
        self._command_subscription = self.create_subscription(
            Twist, '/drone/cmd_vel_raw', self._command_callback, 10)
        self._tracking_subscription = self.create_subscription(
            Pose2D,
            '/drone/target_tracking_box',
            self._tracking_callback,
            10,
        )
        timer_period = min(0.1, self._watchdog_timeout / 2.0)
        self._watchdog_timer = self.create_timer(
            timer_period, self._watchdog_callback)

        self.get_logger().info(
            'Safety supervisor started with %.2fs watchdog.'
            % self._watchdog_timeout)

    def _command_callback(self, message: Twist) -> None:
        """Store a raw command and immediately publish its safe equivalent."""
        self._last_command = message
        self._last_command_time = self.get_clock().now()
        self._publish_evaluated_command()

    def _tracking_callback(self, message: Pose2D) -> None:
        """Refresh target health and evaluate the proximity limit."""
        self._last_target_time = self.get_clock().now()
        self._target_too_close = (
            message.theta >= self._min_safety_box_height)
        self._publish_evaluated_command()

    def _watchdog_callback(self) -> None:
        """Continuously re-evaluate safety, publishing a stop on any fault."""
        self._publish_evaluated_command()

    def _publish_evaluated_command(self) -> None:
        """Publish the most recent command only when every guard passes."""
        fault_reason = self._current_fault_reason()
        if fault_reason is None and self._last_command is not None:
            output = self._last_command
        else:
            output = Twist()

        if fault_reason != self._fault_reason:
            if fault_reason is None:
                self.get_logger().info('Safety fault cleared.')
            else:
                self.get_logger().warning(
                    'Safety stop active: %s.' % fault_reason)
            self._fault_reason = fault_reason

        self._safe_command_publisher.publish(output)

    def _current_fault_reason(self) -> Optional[str]:
        """Return a human-readable active fault, or ``None`` when safe."""
        now = self.get_clock().now()
        if self._last_command_time is None:
            return 'waiting for first velocity command'
        if self._seconds_since(self._last_command_time, now) > \
                self._watchdog_timeout:
            return 'velocity command timeout'
        if self._command_requests_motion(self._last_command):
            if self._last_target_time is None:
                return 'motion requested without a target observation'
            if self._seconds_since(self._last_target_time, now) > \
                    self._watchdog_timeout:
                return 'target tracking timeout'
            if self._target_too_close:
                return 'target inside minimum safety distance'
        return None

    @staticmethod
    def _command_requests_motion(command: Optional[Twist]) -> bool:
        """Return whether any supported velocity axis is nonzero."""
        if command is None:
            return False
        values = (
            command.linear.x,
            command.linear.y,
            command.linear.z,
            command.angular.z,
        )
        return any(abs(value) > 1e-6 for value in values)

    @staticmethod
    def _seconds_since(then, now) -> float:
        """Return elapsed ROS-clock seconds between two time objects."""
        return (now - then).nanoseconds / 1_000_000_000.0


def main(args=None) -> None:
    """Run the safety supervisor node until ROS shuts down."""
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Safety supervisor interrupted by user.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
