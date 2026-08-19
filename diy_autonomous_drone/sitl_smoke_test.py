"""Non-flight smoke test for the ROS-to-ArduPilot SITL connection."""

import time
from typing import Optional

from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.msg import State
import rclpy
from rclpy.node import Node


class SITLSmokeTestNode(Node):
    """Verify the connected, zero-command path without granting authority."""

    REQUIRED_MESSAGES = 3

    def __init__(self) -> None:
        """Create subscriptions for every stage of the command pipeline."""
        super().__init__('sitl_smoke_test')
        self.declare_parameter('timeout_sec', 30.0)
        self._timeout_sec = max(
            1.0, float(self.get_parameter('timeout_sec').value))

        self._connected = False
        self._last_mode: Optional[str] = None
        self._raw_count = 0
        self._safe_count = 0
        self._mavros_count = 0
        self._nonzero_stage: Optional[str] = None

        self.create_subscription(
            State, '/mavros/state', self._state_callback, 10)
        self.create_subscription(
            Twist, '/drone/cmd_vel_raw', self._raw_callback, 10)
        self.create_subscription(
            Twist, '/drone/cmd_vel_safe', self._safe_callback, 10)
        self.create_subscription(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            self._mavros_callback,
            10,
        )

    def _state_callback(self, message: State) -> None:
        """Record whether MAVROS has a live flight-controller link."""
        self._connected = bool(message.connected)
        self._last_mode = message.mode or 'UNKNOWN'

    def _raw_callback(self, message: Twist) -> None:
        """Check the tracking bridge's power-on hover command."""
        self._raw_count += 1
        self._check_zero('tracking bridge', message)

    def _safe_callback(self, message: Twist) -> None:
        """Check the safety supervisor's output."""
        self._safe_count += 1
        self._check_zero('safety supervisor', message)

    def _mavros_callback(self, message: TwistStamped) -> None:
        """Check the final command sent toward MAVROS."""
        self._mavros_count += 1
        self._check_zero('MAVROS adapter', message.twist)

    def _check_zero(self, stage: str, message: Twist) -> None:
        """Remember the first command stage that requests any motion."""
        if self._nonzero_stage is None and not self._is_zero(message):
            self._nonzero_stage = stage

    def passed(self) -> bool:
        """Return whether every expected signal has been observed safely."""
        enough_commands = all(count >= self.REQUIRED_MESSAGES for count in (
            self._raw_count,
            self._safe_count,
            self._mavros_count,
        ))
        return (
            self._connected
            and enough_commands
            and self._nonzero_stage is None
        )

    def failure_reason(self) -> str:
        """Describe missing or unsafe observations for operator diagnostics."""
        if self._nonzero_stage is not None:
            return '%s produced a nonzero command' % self._nonzero_stage

        missing = []
        if not self._connected:
            missing.append('connected /mavros/state')
        if self._raw_count < self.REQUIRED_MESSAGES:
            missing.append('/drone/cmd_vel_raw')
        if self._safe_count < self.REQUIRED_MESSAGES:
            missing.append('/drone/cmd_vel_safe')
        if self._mavros_count < self.REQUIRED_MESSAGES:
            missing.append('/mavros/setpoint_velocity/cmd_vel')
        return 'timed out waiting for ' + ', '.join(missing)

    @staticmethod
    def _is_zero(message: Twist) -> bool:
        """Return whether all linear and angular components are near zero."""
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        return all(abs(value) <= 1e-6 for value in values)


def main(args=None) -> int:
    """Run the bounded SITL smoke test and return a shell-friendly status."""
    rclpy.init(args=args)
    node = SITLSmokeTestNode()
    deadline = time.monotonic() + node._timeout_sec
    exit_code = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._nonzero_stage is not None:
                break
            if node.passed():
                node.get_logger().info(
                    'SITL smoke test passed: MAVROS connected in %s mode '
                    'and all command stages remained at zero.'
                    % (node._last_mode or 'UNKNOWN')
                )
                exit_code = 0
                break

        if exit_code != 0:
            node.get_logger().error(
                'SITL smoke test failed: %s.' % node.failure_reason())
    except KeyboardInterrupt:
        node.get_logger().warning('SITL smoke test interrupted.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
