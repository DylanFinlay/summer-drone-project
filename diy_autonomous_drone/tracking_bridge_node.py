"""Convert vision observations or gestures into raw velocity commands."""

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Int32, String

from diy_autonomous_drone.autonomy_modes import (
    MODE_ACTIVE_TRACK,
    MODE_GESTURE_CONTROL,
    MODE_HOVER,
    mode_rejection_reason,
    normalized_mode,
)
from diy_autonomous_drone.shutdown_safety import publish_zero_burst
from diy_autonomous_drone.tracking_filter import (
    TargetObservationFilter,
    apply_continuous_deadband,
)
from diy_autonomous_drone.target_loss_state import (
    TargetLossStateMachine,
    TargetTrackingState,
)
from diy_autonomous_drone.velocity_limiter import VelocityLimiter


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
        self.declare_parameter('tracking_filter_alpha', 0.35)
        self.declare_parameter('yaw_error_deadband', 0.04)
        self.declare_parameter('forward_error_deadband', 0.025)
        self.declare_parameter('k_p_yaw', 1.2)
        self.declare_parameter('k_p_forward', 1.5)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 0.4)
        self.declare_parameter('max_linear_acceleration', 0.5)
        self.declare_parameter('max_yaw_acceleration', 0.8)
        self.declare_parameter('acceleration_limiter_max_dt_sec', 0.1)
        self.declare_parameter('gesture_speed', 0.25)
        self.declare_parameter('target_timeout_sec', 0.5)
        self.declare_parameter('target_loss_grace_sec', 0.75)
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
        filter_alpha = self._bounded_parameter(
            'tracking_filter_alpha', lower=0.0, upper=1.0,
            allow_lower=False, allow_upper=True)
        self._tracking_filter = TargetObservationFilter(filter_alpha)
        self._yaw_error_deadband = self._bounded_parameter(
            'yaw_error_deadband', lower=0.0, upper=1.0,
            allow_lower=True, allow_upper=False)
        self._forward_error_deadband = self._bounded_parameter(
            'forward_error_deadband', lower=0.0, upper=1.0,
            allow_lower=True, allow_upper=False)
        self._k_p_yaw = float(self.get_parameter('k_p_yaw').value)
        self._k_p_forward = float(
            self.get_parameter('k_p_forward').value)
        self._max_linear_speed = abs(float(
            self.get_parameter('max_linear_speed').value))
        self._max_angular_speed = abs(float(
            self.get_parameter('max_angular_speed').value))
        max_linear_acceleration = self._positive_parameter(
            'max_linear_acceleration')
        max_yaw_acceleration = self._positive_parameter(
            'max_yaw_acceleration')
        limiter_max_dt = self._positive_parameter(
            'acceleration_limiter_max_dt_sec')
        self._velocity_limiter = VelocityLimiter(
            max_linear_acceleration=max_linear_acceleration,
            max_yaw_acceleration=max_yaw_acceleration,
            max_dt=limiter_max_dt,
        )
        self._gesture_speed = abs(float(
            self.get_parameter('gesture_speed').value))
        self._target_timeout = max(
            0.05, float(self.get_parameter('target_timeout_sec').value))
        self._target_loss_state = TargetLossStateMachine(
            self._positive_parameter('target_loss_grace_sec'))
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
        self._tracking_state_publisher = self.create_publisher(
            String, '/drone/tracking_state', 10)
        self._autonomy_mode_publisher = self.create_publisher(
            String, '/drone/autonomy_mode', 10)
        self._tracking_subscription = self.create_subscription(
            Pose2D,
            '/drone/target_tracking_box',
            self._tracking_callback,
            10,
        )
        self._target_visibility_subscription = self.create_subscription(
            Bool,
            '/drone/target_visible',
            self._target_visibility_callback,
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
        self._publish_tracking_state()
        self._publish_autonomy_mode()

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
        self._publish_autonomy_mode()

    def _stop_and_clear_inputs(self, publish_stop: bool = True) -> None:
        """Stop immediately and invalidate data from the previous mode."""
        self._latest_target = None
        self._latest_target_time = None
        self._latest_gesture = self.GESTURE_NONE
        self._latest_gesture_time = None
        self._tracking_filter.reset()
        previous_state = self._target_loss_state.state
        self._target_loss_state.reset()
        self._velocity_limiter.reset()
        if publish_stop:
            self._command_publisher.publish(Twist())
        self._report_tracking_state_change(previous_state)

    def _tracking_callback(self, message: Pose2D) -> None:
        """Filter and store the newest target for the control timer."""
        if self._mode != self.MODE_ACTIVE_TRACK:
            return
        filtered = self._tracking_filter.update(
            (message.x, message.y, message.theta))
        filtered_message = Pose2D()
        filtered_message.x = filtered[0]
        filtered_message.y = filtered[1]
        filtered_message.theta = filtered[2]
        self._latest_target = filtered_message
        self._latest_target_time = self.get_clock().now()
        previous_state = self._target_loss_state.state
        self._target_loss_state.target_seen()
        self._report_tracking_state_change(previous_state)

    def _target_visibility_callback(self, message: Bool) -> None:
        """Stop immediately when a processed frame has no safe target."""
        if self._mode != self.MODE_ACTIVE_TRACK or message.data:
            return
        self._enter_temporary_target_loss(time.monotonic())

    def _gesture_callback(self, message: Int32) -> None:
        """Store the newest gesture for the control timer."""
        self._latest_gesture = int(message.data)
        self._latest_gesture_time = self.get_clock().now()

    def _publish_command(self) -> None:
        """Publish a limited command or bypass the limiter for a safe stop."""
        # These component heartbeats must continue even when a mode has no
        # fresh perception input and therefore returns early with a stop.
        self._publish_autonomy_mode()
        self._publish_tracking_state()
        if self._mode == self.MODE_ACTIVE_TRACK:
            now = time.monotonic()
            tracking_state = self._target_loss_state.state
            target_is_stale = (
                self._latest_target is None
                or not self._is_fresh(
                    self._latest_target_time,
                    self._target_timeout,
                )
            )
            if (
                tracking_state == TargetTrackingState.TRACKING
                and target_is_stale
            ):
                self._enter_temporary_target_loss(now)
                return

            if tracking_state == TargetTrackingState.TEMPORARILY_LOST:
                previous_state = self._target_loss_state.state
                self._target_loss_state.update(now)
                if (
                    self._target_loss_state.state
                    == TargetTrackingState.HOVER
                ):
                    self._publish_immediate_stop(clear_target=True)
                else:
                    self._publish_immediate_stop(clear_target=False)
                self._report_tracking_state_change(previous_state)
                return

            if tracking_state != TargetTrackingState.TRACKING:
                self._publish_immediate_stop(clear_target=True)
                self._publish_tracking_state()
                return
            desired_command = self._active_tracking_command()
        elif self._mode == self.MODE_GESTURE_CONTROL:
            if not self._is_fresh(
                self._latest_gesture_time, self._gesture_timeout
            ):
                self._publish_immediate_stop(clear_target=True)
                return
            desired_command = self._gesture_command()
        else:
            self._publish_immediate_stop(clear_target=True)
            self._publish_tracking_state()
            return

        command = self._limited_command(desired_command)
        self._command_publisher.publish(command)
        self._publish_tracking_state()

    def _enter_temporary_target_loss(self, timestamp: float) -> None:
        """Transition from tracking and publish an immediate zero command."""
        previous_state = self._target_loss_state.state
        self._target_loss_state.target_missed(timestamp)
        if (
            self._target_loss_state.state
            == TargetTrackingState.TEMPORARILY_LOST
        ):
            self._publish_immediate_stop(clear_target=True)
        self._report_tracking_state_change(previous_state)

    def _publish_immediate_stop(self, clear_target: bool) -> None:
        """Bypass ramping for a safety stop and optionally clear perception."""
        if clear_target:
            self._tracking_filter.reset()
            self._latest_target = None
            self._latest_target_time = None
        self._velocity_limiter.reset()
        self._command_publisher.publish(Twist())

    def _report_tracking_state_change(
        self, previous_state: TargetTrackingState
    ) -> None:
        """Log and publish one explicit target state transition."""
        current_state = self._target_loss_state.state
        if current_state != previous_state:
            if current_state == TargetTrackingState.TRACKING:
                self.get_logger().info(
                    'Tracking state: tracking; fresh target acquired.')
            elif current_state == TargetTrackingState.TEMPORARILY_LOST:
                self.get_logger().warning(
                    'Tracking state: temporarily_lost; immediate hover and '
                    'bounded reacquisition window active.')
            else:
                self.get_logger().warning(
                    'Tracking state: hover; target-loss grace period '
                    'expired or tracking was disabled.')
        self._publish_tracking_state()

    def _publish_tracking_state(self) -> None:
        """Publish the current explicit target-tracking state."""
        message = String()
        message.data = self._target_loss_state.state.value
        self._tracking_state_publisher.publish(message)
        self._publish_autonomy_mode()

    def _publish_autonomy_mode(self) -> None:
        """Publish the currently active command-generator mode."""
        message = String()
        message.data = self._mode
        self._autonomy_mode_publisher.publish(message)

    def _limited_command(self, desired: Twist) -> Twist:
        """Convert a desired command through the time-based limiter."""
        limited = self._velocity_limiter.limit(
            (
                desired.linear.x,
                desired.linear.y,
                desired.linear.z,
                desired.angular.z,
            ),
            time.monotonic(),
        )
        command = Twist()
        command.linear.x = limited[0]
        command.linear.y = limited[1]
        command.linear.z = limited[2]
        command.angular.z = limited[3]
        return command

    def _active_tracking_command(self) -> Twist:
        """Return a bounded P-controller command for a fresh target."""
        if self._latest_target is None or not self._is_fresh(
            self._latest_target_time, self._target_timeout
        ):
            return Twist()

        command = Twist()
        height_error = apply_continuous_deadband(
            self._target_box_height - self._latest_target.theta,
            self._forward_error_deadband,
        )
        horizontal_error = apply_continuous_deadband(
            self._latest_target.x,
            self._yaw_error_deadband,
        )
        command.linear.x = self._clamp(
            self._k_p_forward * height_error,
            -self._max_linear_speed,
            self._max_linear_speed,
        )
        command.angular.z = self._clamp(
            -self._k_p_yaw * horizontal_error,
            -self._max_angular_speed,
            self._max_angular_speed,
        )
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

    def _positive_parameter(self, name: str) -> float:
        """Read one required finite positive safety parameter."""
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError('%s must be finite and positive' % name)
        return value

    def _bounded_parameter(
        self,
        name: str,
        lower: float,
        upper: float,
        allow_lower: bool,
        allow_upper: bool,
    ) -> float:
        """Read one finite parameter within configured bounds."""
        value = float(self.get_parameter(name).value)
        lower_valid = value >= lower if allow_lower else value > lower
        upper_valid = value <= upper if allow_upper else value < upper
        if not math.isfinite(value) or not lower_valid or not upper_valid:
            left_bracket = '[' if allow_lower else '('
            right_bracket = ']' if allow_upper else ')'
            raise ValueError(
                '%s must be finite and in %s%s, %s%s'
                % (
                    name,
                    left_bracket,
                    lower,
                    upper,
                    right_bracket,
                )
            )
        return value

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        """Clamp a number to an inclusive range."""
        return max(lower, min(upper, float(value)))

    def publish_shutdown_stop(self) -> int:
        """Best-effort repeated zero output before orderly node teardown."""
        self._stop_and_clear_inputs(publish_stop=False)
        return publish_zero_burst(
            self._command_publisher,
            Twist,
            count=5,
            interval_sec=0.025,
        )


def main(args=None) -> None:
    """Run the tracking bridge node until ROS shuts down."""
    rclpy.init(args=args)
    node = TrackingBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Tracking bridge interrupted by user.')
    finally:
        if rclpy.ok():
            node.publish_shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
