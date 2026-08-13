"""Pure command planning for the operator autonomy-mode tool."""

from dataclasses import dataclass
from typing import Optional, Tuple

from diy_autonomous_drone.autonomy_modes import (
    MODE_ACTIVE_TRACK,
    MODE_GESTURE_CONTROL,
    MODE_HOVER,
)


@dataclass(frozen=True)
class ParameterChange:
    """One ordered parameter update requested by an operator command."""

    name: str
    value: object


@dataclass(frozen=True)
class OperatorCommandPlan:
    """Safe ordered changes and optional zero burst for one command."""

    changes: Tuple[ParameterChange, ...]
    zero_burst: bool = False


def command_plan(command: str) -> Optional[OperatorCommandPlan]:
    """Return safe ordered effects for a supported operator command."""
    name = str(command).strip().lower()
    if name == 'status':
        return None
    if name == 'hover':
        return OperatorCommandPlan((
            ParameterChange('autonomy_mode', MODE_HOVER),
        ))
    if name == 'track':
        return OperatorCommandPlan((
            ParameterChange('autonomy_mode', MODE_ACTIVE_TRACK),
        ))
    if name == 'gesture':
        return OperatorCommandPlan((
            ParameterChange('enable_gesture_control', True),
            ParameterChange('autonomy_mode', MODE_GESTURE_CONTROL),
        ))
    if name == 'lock-gesture':
        return OperatorCommandPlan((
            ParameterChange('autonomy_mode', MODE_HOVER),
            ParameterChange('enable_gesture_control', False),
        ))
    if name == 'prepare-shutdown':
        return OperatorCommandPlan(
            changes=(ParameterChange('autonomy_mode', MODE_HOVER),),
            zero_burst=True,
        )
    raise ValueError(
        'command must be one of: status, hover, track, gesture, '
        'lock-gesture, prepare-shutdown'
    )
