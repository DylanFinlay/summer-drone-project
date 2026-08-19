"""Publish a compact summary and standard ROS diagnostics for the drone."""

import json
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from diy_autonomous_drone.status_summary import DroneStatusModel
from mavros_msgs.msg import State
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusNode(Node):
    """Aggregate authoritative component reports without controlling flight."""

    def __init__(self) -> None:
        """Declare freshness settings and establish diagnostic interfaces."""
        super().__init__('status_node')

        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('input_timeout_sec', 2.0)
        self.declare_parameter('expect_fc_interface', True)
        self.declare_parameter('expect_tracking', True)
        self.declare_parameter('expect_rc_aux', False)

        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate_hz must be finite and positive')
        self._model = DroneStatusModel(
            input_timeout_sec=float(
                self.get_parameter('input_timeout_sec').value),
            expect_fc_interface=bool(
                self.get_parameter('expect_fc_interface').value),
            expect_tracking=bool(
                self.get_parameter('expect_tracking').value),
            expect_rc_aux=bool(
                self.get_parameter('expect_rc_aux').value),
        )

        self._status_publisher = self.create_publisher(
            String, '/drone/status', 10)
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self._mode_subscription = self.create_subscription(
            String,
            '/drone/autonomy_mode',
            self._mode_callback,
            10,
        )
        self._tracking_subscription = self.create_subscription(
            String,
            '/drone/tracking_state',
            self._tracking_callback,
            10,
        )
        self._fc_state_subscription = self.create_subscription(
            State, '/mavros/state', self._fc_state_callback, 10)
        self._safety_reason_subscription = self.create_subscription(
            String,
            '/drone/safety_stop_reason',
            self._safety_reason_callback,
            10,
        )
        self._fc_gate_reason_subscription = self.create_subscription(
            String,
            '/drone/fc_gate_reason',
            self._fc_gate_reason_callback,
            10,
        )
        self._rc_aux_subscription = self.create_subscription(
            String,
            '/drone/rc_aux_state',
            self._rc_aux_callback,
            10,
        )
        self._timer = self.create_timer(
            1.0 / publish_rate, self._publish_status)

        self.get_logger().info(
            'Status aggregation started on /drone/status and /diagnostics.')

    def _mode_callback(self, message: String) -> None:
        """Store the active command-generator mode."""
        self._model.set_autonomy_mode(message.data, time.monotonic())

    def _tracking_callback(self, message: String) -> None:
        """Store the explicit target-lock state."""
        self._model.set_tracking_state(message.data, time.monotonic())

    def _fc_state_callback(self, message: State) -> None:
        """Store the latest MAVROS flight-controller state."""
        self._model.set_fc_state(
            connected=message.connected,
            armed=message.armed,
            mode=message.mode,
            timestamp=time.monotonic(),
        )

    def _safety_reason_callback(self, message: String) -> None:
        """Store the command safety supervisor's active stop reason."""
        self._model.set_supervisor_reason(message.data, time.monotonic())

    def _fc_gate_reason_callback(self, message: String) -> None:
        """Store the MAVROS adapter's active gate reason."""
        self._model.set_fc_gate_reason(message.data, time.monotonic())

    def _rc_aux_callback(self, message: String) -> None:
        """Store the optional RC auxiliary mode-selector state."""
        self._model.set_rc_aux_state(message.data, time.monotonic())

    def _publish_status(self) -> None:
        """Publish matching compact and standard diagnostic messages."""
        summary = self._model.snapshot(time.monotonic())

        compact = String()
        compact.data = json.dumps(
            summary.as_dict(), sort_keys=True, separators=(',', ':'))
        self._status_publisher.publish(compact)

        status = DiagnosticStatus()
        status.level = summary.level
        status.name = 'diy_autonomous_drone/status'
        status.message = summary.message
        status.hardware_id = 'companion_computer'
        status.values = [
            self._key_value('health', summary.health),
            self._key_value('autonomy_mode', summary.autonomy_mode),
            self._key_value('fc_connected', summary.fc_connected),
            self._key_value('fc_armed', summary.fc_armed),
            self._key_value('fc_mode', summary.fc_mode),
            self._key_value('tracking_state', summary.tracking_state),
            self._key_value('target_locked', summary.target_locked),
            self._key_value('rc_aux_state', summary.rc_aux_state),
            self._key_value(
                'safety_stop_reason', summary.safety_stop_reason),
        ]

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._diagnostic_publisher.publish(diagnostics)

    @staticmethod
    def _key_value(key: str, value) -> KeyValue:
        """Create one consistently formatted diagnostic field."""
        item = KeyValue()
        item.key = key
        if isinstance(value, bool):
            item.value = str(value).lower()
        else:
            item.value = str(value)
        return item


def main(args=None) -> None:
    """Run the passive status aggregator until ROS shuts down."""
    rclpy.init(args=args)
    node = StatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Status node interrupted by user.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
