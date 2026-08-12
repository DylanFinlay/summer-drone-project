"""Filtering and deadband helpers for normalized target observations."""

import math
from typing import Optional, Tuple


TargetObservation = Tuple[float, float, float]


class TargetObservationFilter:
    """Apply exponential smoothing to target centre and box height."""

    def __init__(self, alpha: float) -> None:
        """Configure new-sample weight in the interval ``(0, 1]``."""
        alpha = float(alpha)
        if not math.isfinite(alpha) or not 0.0 < alpha <= 1.0:
            raise ValueError('alpha must be finite and in the interval (0, 1]')
        self._alpha = alpha
        self._current: Optional[TargetObservation] = None

    @property
    def current(self) -> Optional[TargetObservation]:
        """Return the current filtered observation, if initialized."""
        return self._current

    def reset(self) -> None:
        """Discard all earlier target observations."""
        self._current = None

    def update(self, observation: TargetObservation) -> TargetObservation:
        """Include one observation and return the new filtered value."""
        observation = tuple(float(value) for value in observation)
        if len(observation) != 3:
            raise ValueError('observation must contain x, y, and box height')
        if not all(math.isfinite(value) for value in observation):
            raise ValueError('observation must contain finite values')

        if self._current is None:
            self._current = observation
            return self._current

        retained_weight = 1.0 - self._alpha
        self._current = tuple(
            self._alpha * observation[index]
            + retained_weight * self._current[index]
            for index in range(3)
        )
        return self._current


def apply_continuous_deadband(error: float, width: float) -> float:
    """Suppress small errors without a discontinuity at the boundary."""
    error = float(error)
    width = float(width)
    if not math.isfinite(error):
        raise ValueError('error must be finite')
    if not math.isfinite(width) or width < 0.0:
        raise ValueError('deadband width must be finite and nonnegative')
    if abs(error) <= width:
        return 0.0
    return math.copysign(abs(error) - width, error)
