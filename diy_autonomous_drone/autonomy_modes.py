"""Shared validation rules for live autonomy-mode selection."""

from typing import Optional


MODE_HOVER = 'hover'
MODE_ACTIVE_TRACK = 'active_track'
MODE_GESTURE_CONTROL = 'gesture_control'
VALID_MODES = frozenset({
    MODE_HOVER,
    MODE_ACTIVE_TRACK,
    MODE_GESTURE_CONTROL,
})


def normalized_mode(value: str) -> str:
    """Return the canonical lowercase representation of a mode name."""
    return str(value).strip().lower()


def mode_rejection_reason(
    requested_mode: str,
    gesture_enabled: bool,
) -> Optional[str]:
    """Return why a mode request is unsafe, or ``None`` when permitted."""
    if requested_mode not in VALID_MODES:
        return (
            'autonomy_mode must be one of: %s'
            % ', '.join(sorted(VALID_MODES))
        )
    if requested_mode == MODE_GESTURE_CONTROL and not gesture_enabled:
        return (
            'gesture_control is disabled; set enable_gesture_control=true '
            'before selecting it'
        )
    return None
