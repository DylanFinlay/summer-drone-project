"""Convert vision observations or gestures into raw velocity commands."""

from typing import Optional

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Int32

from diy_autonomous_drone.autonomy_modes import (
    MODE_ACTIVE_TRACK,
    MODE_GESTURE_CONTROL,
    MODE_HOVER,
    mode_rejection_reason,
    normalized_mode,
)


class TrackingBridgeNode(Node):
    """Generate bounded commands for one operator-selected demo mode."""

    MODE_HOVER = MODE_HOVER
    MODE_ACTIVE_TRACK = MODE_ACTIVE_TRACK
    MODE_GESTURE_CONTROL = MODE_GESTURE_CONTROL

    GESTURE_NONE = 0
    GESTURE_UP = 2
    GESTURE_DOWN = 3
    GESTURE_LEFT = 4
    GESTURE_RIGHT = 5

    def __init__(self) -> None:
        """Declare control parameters and establish ROS interfaces."""
        super().__init__('tracking_bridge_node')

        self.declare_parameter('autonomy_mode', self.MODE_HOVER)
        self.declare_parameter('enable_gesture_control', False)
        self.declare_parameter('target_box_height', 0.35)
        self.declare_parameter('k_p_yaw', 1.2)
        self.declare_parameter('k_p_forward', 1.5)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 0.4)
        self.declare_parameter('gesture_speed', 0.25)
        self.declare_parameter('target_timeout_sec', 0.5)
        self.declare_parameter('gesture_timeout_sec', 0.3)
        self.declare_parameter('command_rate_hz', 20)

        requested_mode = normalized_mode(
            self.get_parameter('autonomy_mode').value)
        self._gesture_enabled = bool(
            self.get_parameter('enable_gesture_control').value)
        self._mode = self._startup_mode(
            requested_mode, self._gesture_enabled)
        if self._mode != requested_mode:
            self.set_parameters([
                Parameter('autonomy_mode', value=self._mode),
            ])

        self._target_box_height = float(
            self.get_parameter('target_box_height').value)
        self._k_p_yaw = float(self.get_parameter('k_p_yaw').value)
        self._k_p_forward = float(
            self.get_parameter('k_p_forward').value)
        self._max_linear_speed = abs(float(
            self.get_parameter('max_linear_speed').value))
        self._max_angular_speed = abs(float(
            self.get_parameter('max_angular_speed').value))
        self._gesture_speed = abs(float(
            self.get_parameter('gesture_speed').value))
        self._target_timeout = max(
            0.05, float(self.get_parameter('target_timeout_sec').value))
        self._gesture_timeout = max(
            0.05, float(self.get_parameter('gesture_timeout_sec').value))
        command_rate = max(
            1, int(self.get_parameter('command_rate_hz').value))

        self._latest_target: Optional[Pose2D] = None
        self._latest_target_time = None
        self._latest_gesture = self.GESTURE_NONE
        self._latest_gesture_time = None

        self._command_publisher = self.create_publisher(
            Twist, '/drone/cmd_vel_raw', 10)
        self._tracking_subscription = self.create_subscription(
            Pose2D,
            '/drone/target_tracking_box',
            self._tracking_callback,
            10,
        )
        self._gesture_subscription = self.create_subscription(
            Int32, '/drone/active_gesture', self._gesture_callback, 10)
        self._command_timer = self.create_timer(
            1.0 / float(command_rate), self._publish_command)
        self.add_on_set_parameters_callback(
            self._validate_parameter_update_callback)
        self.add_post_set_parameters_callback(
            self._apply_parameter_update_callback)

        self.get_logger().info(
            'Tracking bridge started in %s mode.' % self._mode)

    def _startup_mode(self, requested: str, gesture_enabled: bool) -> str:
        """Fail closed to hover for an unsafe launch-time mode request."""
        rejection_reason = mode_rejection_reason(
            requested, gesture_enabled)
        if rejection_reason is None:
            return requested
        self.get_logger().error(
            '%s; falling back to hover.' % rejection_reason)
        return self.MODE_HOVER

    def _validate_parameter_update_callback(
        self, parameters
    ) -> SetParametersResult:
        """Reject invalid or unsafe live feature-mode changes."""
        requested_mode = self._mode
        gesture_enabled = self._gesture_enabled

        for parameter in parameters:
            if parameter.name == 'autonomy_mode':
                if parameter.type_ != Parameter.Type.STRING:
                    return SetParametersResult(
                        successful=False,
                        reason='autonomy_mode must be a string',
                    )
                requested_mode = normalized_mode(parameter.value)
                if parameter.value != requested_mode:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            'use the canonical autonomy_mode name %r'
                            % requested_mode
                        ),
                    )
            elif parameter.name == 'enable_gesture_control':
                if parameter.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False,
                        reason='enable_gesture_control must be a boolean',
                    )
                gesture_enabled = bool(parameter.value)

        rejection_reason = mode_rejection_reason(
            requested_mode, gesture_enabled)
        if rejection_reason is not None:
            return SetParametersResult(
                successful=False, reason=rejection_reason)

        return SetParametersResult(successful=True)

    def _apply_parameter_update_callback(self, parameters) -> None:
        """Apply a committed mode change and invalidate earlier input."""
        previous_mode = self._mode
        for parameter in parameters:
            if parameter.name == 'autonomy_mode':
                self._mode = normalized_mode(parameter.value)
            elif parameter.name == 'enable_gesture_control':
                self._gesture_enabled = bool(parameter.value)

        if self._mode != previous_mode:
            self._stop_and_clear_inputs()
            self.get_logger().warning(
                'Autonomy mode changed from %s to %s; published stop and '
                'waiting for fresh input.'
                % (previous_mode, self._mode)
            )

    def _stop_and_clear_inputs(self) -> None:
        """Stop immediately and invalidate data from the previous mode."""
        self._latest_target = None
        self._latest_target_time = None
        self._latest_gesture = self.GESTURE_NONE
        self._latest_gesture_time = None
        self._command_publisher.publish(Twist())

    def _tracking_callback(self, message: Pose2D) -> None:
        """Store the newest target observation for the control timer."""
        self._latest_target = message
        self._latest_target_time = self.get_clock().now()

    def _gesture_callback(self, message: Int32) -> None:
        """Store the newest gesture for the control timer."""
        self._latest_gesture = int(message.data)
        self._latest_gesture_time = self.get_clock().now()

    def _publish_command(self) -> None:
        """Publish a fresh command, defaulting to zero on stale input."""
        command = Twist()
        if self._mode == self.MODE_ACTIVE_TRACK:
            command = self._active_tracking_command()
        elif self._mode == self.MODE_GESTURE_CONTROL:
            command = self._gesture_command()
        self._command_publisher.publish(command)

    def _active_tracking_command(self) -> Twist:
        """Return a bounded P-controller command for a fresh target."""
        if self._latest_target is None or not self._is_fresh(
            self._latest_target_time, self._target_timeout
        ):
            return Twist()

        command = Twist()
        height_error = (
            self._target_box_height - self._latest_target.theta)
        command.linear.x = self._clamp(
            self._k_p_forward * height_error,
            -self._max_linear_speed,
            self._max_linear_speed,
        )
        command.angular.z = self._clamp(
            -self._k_p_yaw * self._latest_target.x,
            -self._max_angular_speed,
            self._max_angular_speed,
        )

        # TODO: Add a deadband, target filtering, and acceleration limiting
        # after measuring detector noise on recorded flight video.
        return command

    def _gesture_command(self) -> Twist:
        """Return a low-speed command only while a gesture is fresh."""
        if not self._is_fresh(
            self._latest_gesture_time, self._gesture_timeout
        ):
            return Twist()

        command = Twist()
        if self._latest_gesture == self.GESTURE_UP:
            command.linear.z = self._gesture_speed
        elif self._latest_gesture == self.GESTURE_DOWN:
            command.linear.z = -self._gesture_speed
        elif self._latest_gesture == self.GESTURE_LEFT:
            command.linear.y = self._gesture_speed
        elif self._latest_gesture == self.GESTURE_RIGHT:
            command.linear.y = -self._gesture_speed
        return command

    def _is_fresh(self, timestamp, timeout: float) -> bool:
        """Return whether a ROS-clock timestamp is within its timeout."""
        if timestamp is None:
            return False
        age = self.get_clock().now() - timestamp
        return age.nanoseconds / 1_000_000_000.0 <= timeout

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        """Clamp a number to an inclusive range."""
        return max(lower, min(upper, float(value)))


def main(args=None) -> None:
    """Run the tracking bridge node until ROS shuts down."""
    rclpy.init(args=args)
    node = TrackingBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Tracking bridge interrupted by user.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
