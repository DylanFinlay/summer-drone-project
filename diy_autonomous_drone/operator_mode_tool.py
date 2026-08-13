"""Operator-friendly one-shot control for the ROS autonomy mode."""

import argparse
import math
import sys
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.utilities import remove_ros_args

from diy_autonomous_drone.operator_control import command_plan
from diy_autonomous_drone.shutdown_safety import publish_zero_burst


class OperatorModeTool(Node):
    """Read or update tracking mode without touching ArduPilot authority."""

    def __init__(self, target_node: str, timeout_sec: float) -> None:
        """Create the remote parameter client and zero-command publisher."""
        super().__init__('operator_mode_tool')
        self._timeout_sec = float(timeout_sec)
        self._client = AsyncParameterClient(self, target_node)
        self._zero_publisher = self.create_publisher(
            Twist, '/drone/cmd_vel_raw', 10)

    def wait_for_target(self) -> bool:
        """Wait a bounded time for the tracking node's parameter services."""
        return self._client.wait_for_service(
            timeout_sec=self._timeout_sec)

    def read_status(self):
        """Return the current mode and gesture feature-lock setting."""
        future = self._client.get_parameters([
            'autonomy_mode',
            'enable_gesture_control',
        ])
        response = self._wait_for_future(future)
        if response is None or len(response.values) != 2:
            raise RuntimeError('tracking node did not return mode parameters')
        return response.values[0].string_value, response.values[1].bool_value

    def apply_command(self, command: str) -> None:
        """Apply validated ordered changes and an optional shutdown burst."""
        plan = command_plan(command)
        if plan is not None:
            for change in plan.changes:
                self._set_parameter(change.name, change.value)
            if plan.zero_burst:
                publish_zero_burst(
                    self._zero_publisher,
                    Twist,
                    count=10,
                    interval_sec=0.05,
                    sleeper=self._sleep_with_ros,
                )

    def _set_parameter(self, name: str, value) -> None:
        """Set one parameter and fail if the tracking node rejects it."""
        future = self._client.set_parameters([
            Parameter(name, value=value),
        ])
        response = self._wait_for_future(future)
        if response is None or len(response.results) != 1:
            raise RuntimeError('tracking node did not confirm %s' % name)
        result = response.results[0]
        if not result.successful:
            raise RuntimeError(
                '%s rejected: %s' % (name, result.reason or 'no reason'))

    def _wait_for_future(self, future):
        """Spin for a bounded time and return a completed service response."""
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self._timeout_sec)
        if not future.done():
            raise TimeoutError('tracking node parameter request timed out')
        error = future.exception()
        if error is not None:
            raise RuntimeError(str(error))
        return future.result()

    def _sleep_with_ros(self, duration: float) -> None:
        """Give DDS time to send each zero without blocking ROS entirely."""
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self, timeout_sec=min(remaining, 0.01))


def _argument_parser() -> argparse.ArgumentParser:
    """Build the non-interactive operator command parser."""
    parser = argparse.ArgumentParser(
        description='Safely inspect or change the drone autonomy mode.')
    parser.add_argument(
        'command',
        choices=(
            'status',
            'hover',
            'track',
            'gesture',
            'lock-gesture',
            'prepare-shutdown',
        ),
    )
    parser.add_argument(
        '--target-node',
        default='/tracking_bridge_node',
        help='Tracking node name (default: /tracking_bridge_node).',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=5.0,
        help='Parameter-service timeout in seconds.',
    )
    return parser


def main(args=None) -> None:
    """Execute one operator command and report the confirmed final state."""
    if args is None:
        ros_args = None
        cli_args = remove_ros_args(sys.argv)[1:]
    else:
        ros_args = args
        cli_args = remove_ros_args(['operator_mode_tool', *args])[1:]
    parsed = _argument_parser().parse_args(cli_args)
    if not math.isfinite(parsed.timeout) or parsed.timeout <= 0.0:
        raise SystemExit('--timeout must be finite and positive')

    rclpy.init(args=ros_args)
    node = OperatorModeTool(parsed.target_node, parsed.timeout)
    exit_code = 0
    try:
        if not node.wait_for_target():
            raise TimeoutError(
                'tracking node parameter services are unavailable')
        node.apply_command(parsed.command)
        mode, gesture_enabled = node.read_status()
        print(
            'mode=%s gesture_control=%s'
            % (mode, str(gesture_enabled).lower())
        )
        if parsed.command == 'prepare-shutdown':
            print('Zero burst sent; stop the launch now with Ctrl+C.')
    except (RuntimeError, TimeoutError, ValueError) as error:
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
