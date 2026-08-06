"""Gate safe commands using MAVROS flight-controller state."""

import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node


class FlightControllerInterfaceNode(Node):
    """Publish fresh commands to MAVROS only with explicit FC authority."""

    REQUIRED_FLIGHT_MODE = 'GUIDED'

    def __init__(self) -> None:
        """Declare safety parameters and establish MAVROS interfaces."""
        super().__init__('fc_interface_node')

        self.declare_parameter('command_rate_hz', 20)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('state_timeout_sec', 2.0)
        self.declare_parameter('require_guided_mode', True)
        self.declare_parameter('require_armed', True)

        command_rate = max(
            1, int(self.get_parameter('command_rate_hz').value))
        self._command_timeout = max(
            0.05,
            float(self.get_parameter('command_timeout_sec').value),
        )
        self._state_timeout = max(
            0.25,
            float(self.get_parameter('state_timeout_sec').value),
        )
        self._require_guided = bool(
            self.get_parameter('require_guided_mode').value)
        self._require_armed = bool(
            self.get_parameter('require_armed').value)

        self._latest_command = Twist()
        self._last_command_time: Optional[float] = None
        self._latest_state: Optional[State] = None
        self._last_state_time: Optional[float] = None
        self._authority_active = False
        self._gate_reason: Optional[str] = None

        self._velocity_publisher = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self._command_subscription = self.create_subscription(
            Twist, '/drone/cmd_vel_safe', self._command_callback, 10)
        self._state_subscription = self.create_subscription(
            State, '/mavros/state', self._state_callback, 10)
        self._command_timer = self.create_timer(
            1.0 / float(command_rate), self._publish_gated_command)

        self.get_logger().info(
            'MAVROS safety adapter started; movement requires connected '
            'MAVROS state, Guided mode, an armed vehicle, and fresh safety '
            'commands.')

    def _command_callback(self, message: Twist) -> None:
        """Store a safety-approved command and its monotonic receipt time."""
        self._latest_command = message
        self._last_command_time = time.monotonic()

    def _state_callback(self, message: State) -> None:
        """Store the newest MAVROS connection, mode, and arming state."""
        self._latest_state = message
        self._last_state_time = time.monotonic()

    def _publish_gated_command(self) -> None:
        """Publish a fresh command or an explicit zero through MAVROS."""
        authority_reason = self._authority_block_reason()
        authority_active = authority_reason is None

        # Re-entering Guided/armed authority must begin at zero. A new safety
        # command received after the RC mode transition is required before the
        # adapter will publish nonzero motion.
        if authority_active and not self._authority_active:
            self._latest_command = Twist()
            self._last_command_time = None
        self._authority_active = authority_active

        command, gate_reason = self._gated_command(authority_reason)
        self._log_gate_transition(gate_reason)

        output = TwistStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'base_link'
        output.twist = command
        self._velocity_publisher.publish(output)

    def _authority_block_reason(self) -> Optional[str]:
        """Return why MAVROS/RC state does not currently grant authority."""
        if self._latest_state is None or self._last_state_time is None:
            return 'waiting for MAVROS state'
        if time.monotonic() - self._last_state_time > self._state_timeout:
            return 'MAVROS state timeout'
        if not self._latest_state.connected:
            return 'MAVROS is disconnected from the flight controller'
        flight_mode = self._latest_state.mode.strip().upper()
        if self._require_guided and flight_mode != self.REQUIRED_FLIGHT_MODE:
            return 'flight mode is %s, not Guided' % (flight_mode or 'UNKNOWN')
        if self._require_armed and not self._latest_state.armed:
            return 'vehicle is disarmed'
        return None

    def _gated_command(
        self, authority_reason: Optional[str]
    ) -> Tuple[Twist, Optional[str]]:
        """Return motion only when authority and command freshness pass."""
        if authority_reason is not None:
            return Twist(), authority_reason
        if self._last_command_time is None:
            return Twist(), 'waiting for a new safety command'
        if time.monotonic() - self._last_command_time > \
                self._command_timeout:
            return Twist(), 'safety command timeout'
        return self._latest_command, None

    def _log_gate_transition(self, reason: Optional[str]) -> None:
        """Log safety-gate changes without repeating at command frequency."""
        if reason == self._gate_reason:
            return
        if reason is None:
            self.get_logger().info(
                'MAVROS command gate open: Guided, armed, connected, and '
                'receiving fresh safety commands.')
        else:
            self.get_logger().warning(
                'MAVROS command gate closed: %s.' % reason)
        self._gate_reason = reason


def main(args=None) -> None:
    """Run the MAVROS safety adapter until ROS shuts down."""
    rclpy.init(args=args)
    node = FlightControllerInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MAVROS safety adapter interrupted by user.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
