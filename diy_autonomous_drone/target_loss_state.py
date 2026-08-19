"""Hardware-independent target-loss state machine."""

from enum import Enum
import math
from typing import Optional


class TargetTrackingState(str, Enum):
    """Explicit command-generator states for active person tracking."""

    HOVER = 'hover'
    TRACKING = 'tracking'
    TEMPORARILY_LOST = 'temporarily_lost'


class TargetLossStateMachine:
    """Track target presence and expire a bounded reacquisition window."""

    def __init__(self, loss_grace_sec: float) -> None:
        """Configure the finite temporarily-lost grace period."""
        loss_grace_sec = float(loss_grace_sec)
        if not math.isfinite(loss_grace_sec) or loss_grace_sec <= 0.0:
            raise ValueError('loss_grace_sec must be finite and positive')
        self._loss_grace_sec = loss_grace_sec
        self.reset()

    @property
    def state(self) -> TargetTrackingState:
        """Return the current tracking state."""
        return self._state

    @property
    def lost_since(self) -> Optional[float]:
        """Return when temporary loss began, if currently applicable."""
        return self._lost_since

    def reset(self) -> TargetTrackingState:
        """Return to hover and discard any target-loss timer."""
        self._state = TargetTrackingState.HOVER
        self._lost_since: Optional[float] = None
        return self._state

    def target_seen(self) -> TargetTrackingState:
        """Enter tracking after receiving a valid target observation."""
        self._state = TargetTrackingState.TRACKING
        self._lost_since = None
        return self._state

    def target_missed(self, timestamp: float) -> TargetTrackingState:
        """Enter temporary loss once, without extending its deadline."""
        timestamp = self._finite_timestamp(timestamp)
        if self._state == TargetTrackingState.TRACKING:
            self._state = TargetTrackingState.TEMPORARILY_LOST
            self._lost_since = timestamp
        return self._state

    def update(self, timestamp: float) -> TargetTrackingState:
        """Expire temporary loss into hover when its grace time elapses."""
        timestamp = self._finite_timestamp(timestamp)
        if (
            self._state == TargetTrackingState.TEMPORARILY_LOST
            and self._lost_since is not None
            and timestamp - self._lost_since >= self._loss_grace_sec
        ):
            self.reset()
        return self._state

    @staticmethod
    def _finite_timestamp(timestamp: float) -> float:
        """Validate one monotonic timestamp supplied by the caller."""
        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError('timestamp must be finite')
        return timestamp
