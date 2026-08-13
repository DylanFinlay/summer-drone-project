"""Pure, fail-closed RC auxiliary-channel mode selection."""

import math
from typing import Optional, Sequence

from diy_autonomous_drone.autonomy_modes import MODE_HOVER, VALID_MODES


POSITION_LOW = 'low'
POSITION_MIDDLE = 'middle'
POSITION_HIGH = 'high'


class RcAuxModeSelector:
    """Debounce a calibrated spare RC channel into ROS autonomy modes."""

    def __init__(
        self,
        channel: int,
        low_max_pwm: int,
        high_min_pwm: int,
        confirm_samples: int,
        timeout_sec: float,
        low_mode: str = MODE_HOVER,
        middle_mode: str = MODE_HOVER,
        high_mode: str = 'active_track',
    ) -> None:
        """Validate the channel, thresholds, mappings, and stale timeout."""
        self._channel = int(channel)
        self._low_max = int(low_max_pwm)
        self._high_min = int(high_min_pwm)
        self._confirm_samples = int(confirm_samples)
        self._timeout = float(timeout_sec)
        self._modes = {
            POSITION_LOW: str(low_mode),
            POSITION_MIDDLE: str(middle_mode),
            POSITION_HIGH: str(high_mode),
        }
        if self._channel <= 0:
            raise ValueError('RC auxiliary channel must be one-based')
        if self._low_max <= 0 or self._high_min <= self._low_max:
            raise ValueError('RC PWM thresholds must be positive and ordered')
        if self._confirm_samples <= 0:
            raise ValueError('RC confirmation sample count must be positive')
        if not math.isfinite(self._timeout) or self._timeout <= 0.0:
            raise ValueError('RC input timeout must be finite and positive')
        if any(mode not in VALID_MODES for mode in self._modes.values()):
            raise ValueError('RC auxiliary mappings must use valid modes')
        self._candidate: Optional[str] = None
        self._candidate_samples = 0
        self._confirmed: Optional[str] = None
        self._last_valid_time: Optional[float] = None

    @property
    def confirmed_position(self) -> Optional[str]:
        """Return the latest debounced switch position."""
        return self._confirmed

    def update(
        self, channels: Sequence[int], timestamp: float
    ) -> Optional[str]:
        """Consume one RCIn sample and return its confirmed requested mode."""
        now = self._timestamp(timestamp)
        index = self._channel - 1
        if index >= len(channels) or int(channels[index]) <= 0:
            self._clear_candidate()
            return self.requested_mode(now)

        position = self._position(int(channels[index]))
        self._last_valid_time = now
        if position != self._candidate:
            self._candidate = position
            self._candidate_samples = 1
        else:
            self._candidate_samples += 1
        if self._candidate_samples >= self._confirm_samples:
            self._confirmed = position
        return self.requested_mode(now)

    def requested_mode(self, timestamp: float) -> Optional[str]:
        """Return the confirmed mode, or hover after stale RC input."""
        now = self._timestamp(timestamp)
        if self.is_stale(now):
            return MODE_HOVER
        if self._confirmed is None or self._candidate != self._confirmed:
            return MODE_HOVER
        return self._modes[self._confirmed]

    def is_stale(self, timestamp: float) -> bool:
        """Return whether no sufficiently recent valid channel was seen."""
        now = self._timestamp(timestamp)
        return (
            self._last_valid_time is None
            or now - self._last_valid_time < 0.0
            or now - self._last_valid_time > self._timeout
        )

    def status(self, timestamp: float) -> str:
        """Return a compact operator-facing state string."""
        now = self._timestamp(timestamp)
        if self.is_stale(now):
            return 'stale:hover'
        if self._confirmed is None or self._candidate != self._confirmed:
            return 'confirming:hover'
        return '%s:%s' % (
            self._confirmed, self._modes[self._confirmed])

    def _position(self, pwm: int) -> str:
        """Map one calibrated PWM sample to a three-position switch."""
        if pwm <= self._low_max:
            return POSITION_LOW
        if pwm >= self._high_min:
            return POSITION_HIGH
        return POSITION_MIDDLE

    def _clear_candidate(self) -> None:
        """Discard an incomplete switch transition after malformed input."""
        self._candidate = None
        self._candidate_samples = 0

    @staticmethod
    def _timestamp(value: float) -> float:
        """Return a finite monotonic-style timestamp."""
        timestamp = float(value)
        if not math.isfinite(timestamp):
            raise ValueError('timestamp must be finite')
        return timestamp
