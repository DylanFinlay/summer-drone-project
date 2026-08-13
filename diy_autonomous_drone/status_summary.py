"""Hardware-independent aggregation for operator-facing drone status."""

from dataclasses import dataclass
import math
from typing import Dict, Optional


DIAGNOSTIC_OK = 0
DIAGNOSTIC_WARN = 1
DIAGNOSTIC_ERROR = 2
DIAGNOSTIC_STALE = 3


@dataclass(frozen=True)
class StatusSummary:
    """One immutable summary of the latest flight-system observations."""

    level: int
    health: str
    message: str
    autonomy_mode: str
    fc_connected: bool
    fc_armed: bool
    fc_mode: str
    tracking_state: str
    target_locked: bool
    rc_aux_state: str
    safety_stop_reason: str

    def as_dict(self) -> Dict[str, object]:
        """Return stable fields suitable for JSON serialization."""
        return {
            'health': self.health,
            'autonomy_mode': self.autonomy_mode,
            'fc_connected': self.fc_connected,
            'fc_armed': self.fc_armed,
            'fc_mode': self.fc_mode,
            'tracking_state': self.tracking_state,
            'target_locked': self.target_locked,
            'rc_aux_state': self.rc_aux_state,
            'safety_stop_reason': self.safety_stop_reason,
        }


class DroneStatusModel:
    """Combine timestamped component reports without depending on ROS."""

    def __init__(
        self,
        input_timeout_sec: float,
        expect_fc_interface: bool = True,
        expect_tracking: bool = True,
        expect_rc_aux: bool = False,
    ) -> None:
        """Configure freshness requirements for enabled stack components."""
        timeout = float(input_timeout_sec)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('input_timeout_sec must be finite and positive')

        self._input_timeout = timeout
        self._expect_fc = bool(expect_fc_interface)
        self._expect_tracking = bool(expect_tracking)
        self._expect_rc_aux = bool(expect_rc_aux)

        self._autonomy_mode = 'unknown'
        self._autonomy_mode_time: Optional[float] = None
        self._tracking_state = 'unknown'
        self._tracking_state_time: Optional[float] = None
        self._fc_connected = False
        self._fc_armed = False
        self._fc_mode = 'unknown'
        self._fc_state_time: Optional[float] = None
        self._supervisor_reason = ''
        self._supervisor_reason_time: Optional[float] = None
        self._fc_gate_reason = ''
        self._fc_gate_reason_time: Optional[float] = None
        self._rc_aux_state = 'disabled'
        self._rc_aux_state_time: Optional[float] = None

    def set_autonomy_mode(self, value: str, timestamp: float) -> None:
        """Record the command generator's active mode."""
        self._autonomy_mode = str(value) or 'unknown'
        self._autonomy_mode_time = self._timestamp(timestamp)

    def set_tracking_state(self, value: str, timestamp: float) -> None:
        """Record the target-loss state reported by command generation."""
        self._tracking_state = str(value) or 'unknown'
        self._tracking_state_time = self._timestamp(timestamp)

    def set_fc_state(
        self,
        connected: bool,
        armed: bool,
        mode: str,
        timestamp: float,
    ) -> None:
        """Record connection, arming, and mode from MAVROS."""
        self._fc_connected = bool(connected)
        self._fc_armed = bool(armed)
        self._fc_mode = str(mode) or 'unknown'
        self._fc_state_time = self._timestamp(timestamp)

    def set_supervisor_reason(self, value: str, timestamp: float) -> None:
        """Record the independent command safety supervisor's stop reason."""
        self._supervisor_reason = str(value)
        self._supervisor_reason_time = self._timestamp(timestamp)

    def set_fc_gate_reason(self, value: str, timestamp: float) -> None:
        """Record the final MAVROS command gate's stop reason."""
        self._fc_gate_reason = str(value)
        self._fc_gate_reason_time = self._timestamp(timestamp)

    def set_rc_aux_state(self, value: str, timestamp: float) -> None:
        """Record the optional spare-channel mode-selector state."""
        self._rc_aux_state = str(value) or 'unknown'
        self._rc_aux_state_time = self._timestamp(timestamp)

    def snapshot(self, timestamp: float) -> StatusSummary:
        """Build a conservative summary from only fresh component inputs."""
        now = self._timestamp(timestamp)
        unavailable = []
        stop_reasons = []

        if self._expect_tracking:
            mode_fresh = self._is_fresh(self._autonomy_mode_time, now)
            tracking_fresh = self._is_fresh(
                self._tracking_state_time, now)
            autonomy_mode = self._autonomy_mode if mode_fresh else 'unknown'
            tracking_state = (
                self._tracking_state if tracking_fresh else 'unknown')
            if not mode_fresh:
                unavailable.append('autonomy mode')
            if not tracking_fresh:
                unavailable.append('tracking state')
        else:
            autonomy_mode = 'disabled'
            tracking_state = 'disabled'
            stop_reasons.append('tracking command generator disabled')

        rc_aux_state = 'disabled'
        if self._expect_rc_aux:
            rc_aux_fresh = self._is_fresh(self._rc_aux_state_time, now)
            if rc_aux_fresh:
                rc_aux_state = self._rc_aux_state
                if rc_aux_state.startswith('stale:'):
                    stop_reasons.append('RC auxiliary input stale')
                if rc_aux_state.endswith(':rejected'):
                    stop_reasons.append('RC auxiliary mode request rejected')
            else:
                rc_aux_state = 'unknown'
                unavailable.append('RC auxiliary state')

        fc_connected = False
        fc_armed = False
        fc_mode = 'unknown'
        if self._expect_fc:
            fc_fresh = self._is_fresh(self._fc_state_time, now)
            gate_fresh = self._is_fresh(self._fc_gate_reason_time, now)
            if fc_fresh:
                fc_connected = self._fc_connected
                fc_armed = self._fc_armed
                fc_mode = self._fc_mode
                if not fc_connected:
                    stop_reasons.append('flight controller disconnected')
            else:
                unavailable.append('flight controller state')
            if gate_fresh:
                if self._fc_gate_reason:
                    stop_reasons.append(
                        'flight-controller gate: %s'
                        % self._fc_gate_reason)
            else:
                unavailable.append('flight-controller gate')
        else:
            fc_mode = 'disabled'
            stop_reasons.append('flight-controller interface disabled')

        supervisor_fresh = self._is_fresh(
            self._supervisor_reason_time, now)
        if supervisor_fresh:
            if self._supervisor_reason:
                stop_reasons.append(
                    'safety supervisor: %s' % self._supervisor_reason)
        else:
            unavailable.append('safety supervisor')

        for name in unavailable:
            stop_reasons.append('%s unavailable' % name)

        target_locked = (
            autonomy_mode == 'active_track'
            and tracking_state == 'tracking'
        )
        safety_stop_reason = '; '.join(stop_reasons) or 'none'

        if unavailable:
            level = DIAGNOSTIC_STALE
            health = 'stale'
            message = 'status inputs stale or unavailable'
        elif self._expect_fc and not fc_connected:
            level = DIAGNOSTIC_ERROR
            health = 'error'
            message = 'flight controller disconnected'
        elif stop_reasons:
            level = DIAGNOSTIC_WARN
            health = 'warning'
            message = 'safety stop active'
        else:
            level = DIAGNOSTIC_OK
            health = 'ok'
            message = 'all enabled status inputs healthy'

        return StatusSummary(
            level=level,
            health=health,
            message=message,
            autonomy_mode=autonomy_mode,
            fc_connected=fc_connected,
            fc_armed=fc_armed,
            fc_mode=fc_mode,
            tracking_state=tracking_state,
            target_locked=target_locked,
            rc_aux_state=rc_aux_state,
            safety_stop_reason=safety_stop_reason,
        )

    def _is_fresh(self, then: Optional[float], now: float) -> bool:
        """Return whether one timestamp is present and within its timeout."""
        return then is not None and 0.0 <= now - then <= self._input_timeout

    @staticmethod
    def _timestamp(value: float) -> float:
        """Validate one timestamp used for freshness comparisons."""
        timestamp = float(value)
        if not math.isfinite(timestamp):
            raise ValueError('timestamp must be finite')
        return timestamp
