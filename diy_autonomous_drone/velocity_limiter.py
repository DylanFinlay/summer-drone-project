"""Time-based velocity acceleration limiting without ROS dependencies."""

import math
from typing import Tuple


Velocity = Tuple[float, float, float, float]


class VelocityLimiter:
    """Limit linear-vector and yaw-rate changes between commands."""

    def __init__(
        self,
        max_linear_acceleration: float,
        max_yaw_acceleration: float,
        max_dt: float,
    ) -> None:
        """Configure positive acceleration limits and elapsed-time cap."""
        max_linear_acceleration = float(max_linear_acceleration)
        max_yaw_acceleration = float(max_yaw_acceleration)
        max_dt = float(max_dt)
        if (
            not math.isfinite(max_linear_acceleration)
            or max_linear_acceleration <= 0.0
        ):
            raise ValueError('max_linear_acceleration must be positive')
        if (
            not math.isfinite(max_yaw_acceleration)
            or max_yaw_acceleration <= 0.0
        ):
            raise ValueError('max_yaw_acceleration must be positive')
        if not math.isfinite(max_dt) or max_dt <= 0.0:
            raise ValueError('max_dt must be positive')

        self._max_linear_acceleration = max_linear_acceleration
        self._max_yaw_acceleration = max_yaw_acceleration
        self._max_dt = max_dt
        self.reset()

    def reset(self) -> None:
        """Forget prior motion so the next command starts from rest."""
        self._current: Velocity = (0.0, 0.0, 0.0, 0.0)
        self._last_time = None

    @property
    def current(self) -> Velocity:
        """Return the most recently limited command."""
        return self._current

    def limit(self, desired: Velocity, timestamp: float) -> Velocity:
        """Return a command reachable within the elapsed time and limits."""
        desired = tuple(float(value) for value in desired)
        timestamp = float(timestamp)
        if len(desired) != 4:
            raise ValueError('desired velocity must contain x, y, z, and yaw')
        if not all(math.isfinite(value) for value in desired):
            raise ValueError('desired velocity must contain finite values')
        if not math.isfinite(timestamp):
            raise ValueError('timestamp must be finite')

        if self._last_time is None:
            self._last_time = timestamp
            return self._current

        elapsed = max(0.0, min(timestamp - self._last_time, self._max_dt))
        self._last_time = timestamp

        linear = self._limited_linear_vector(desired[:3], elapsed)
        yaw = self._approach(
            self._current[3],
            desired[3],
            self._max_yaw_acceleration * elapsed,
        )
        self._current = (linear[0], linear[1], linear[2], yaw)
        return self._current

    def _limited_linear_vector(
        self,
        desired: Tuple[float, float, float],
        elapsed: float,
    ) -> Tuple[float, float, float]:
        """Bound the magnitude of the three-axis velocity change."""
        delta = tuple(
            desired[index] - self._current[index]
            for index in range(3)
        )
        delta_magnitude = math.sqrt(sum(value * value for value in delta))
        allowed_change = self._max_linear_acceleration * elapsed
        if delta_magnitude <= allowed_change or delta_magnitude == 0.0:
            return desired

        scale = allowed_change / delta_magnitude
        return tuple(
            self._current[index] + delta[index] * scale
            for index in range(3)
        )

    @staticmethod
    def _approach(current: float, desired: float, max_change: float) -> float:
        """Move one scalar toward its target by at most ``max_change``."""
        delta = desired - current
        if abs(delta) <= max_change:
            return desired
        return current + math.copysign(max_change, delta)
