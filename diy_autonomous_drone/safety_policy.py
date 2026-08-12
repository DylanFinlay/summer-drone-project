"""Pure, fail-closed policies shared by the ROS safety gate nodes."""

import math
from typing import Optional


def supervisor_fault_reason(
    command_age_sec: Optional[float],
    command_requests_motion: bool,
    target_age_sec: Optional[float],
    target_too_close: bool,
    watchdog_timeout_sec: float,
) -> Optional[str]:
    """Return why a raw command must stop before reaching the FC gate."""
    timeout = _positive_timeout(
        watchdog_timeout_sec, 'watchdog_timeout_sec')
    if command_age_sec is None:
        return 'waiting for first velocity command'
    if not _age_is_fresh(command_age_sec, timeout):
        return 'velocity command timeout'
    if command_requests_motion:
        if target_age_sec is None:
            return 'motion requested without a target observation'
        if not _age_is_fresh(target_age_sec, timeout):
            return 'target tracking timeout'
        if target_too_close:
            return 'target inside minimum safety distance'
    return None


def fc_authority_block_reason(
    state_age_sec: Optional[float],
    connected: bool,
    flight_mode: str,
    armed: bool,
    state_timeout_sec: float,
    require_guided_mode: bool,
    require_armed: bool,
) -> Optional[str]:
    """Return why current MAVROS/RC state cannot grant command authority."""
    timeout = _positive_timeout(state_timeout_sec, 'state_timeout_sec')
    if state_age_sec is None:
        return 'waiting for MAVROS state'
    if not _age_is_fresh(state_age_sec, timeout):
        return 'MAVROS state timeout'
    if not connected:
        return 'MAVROS is disconnected from the flight controller'
    normalized_mode = str(flight_mode).strip().upper()
    if require_guided_mode and normalized_mode != 'GUIDED':
        return 'flight mode is %s, not Guided' % (
            normalized_mode or 'UNKNOWN')
    if require_armed and not armed:
        return 'vehicle is disarmed'
    return None


def fc_command_gate_reason(
    authority_reason: Optional[str],
    command_age_sec: Optional[float],
    command_timeout_sec: float,
) -> Optional[str]:
    """Return the final reason a safety-approved command cannot pass."""
    timeout = _positive_timeout(command_timeout_sec, 'command_timeout_sec')
    if authority_reason is not None:
        return authority_reason
    if command_age_sec is None:
        return 'waiting for a new safety command'
    if not _age_is_fresh(command_age_sec, timeout):
        return 'safety command timeout'
    return None


def _age_is_fresh(age_sec: float, timeout_sec: float) -> bool:
    """Accept only finite, nonnegative ages inside the configured timeout."""
    age = float(age_sec)
    return math.isfinite(age) and 0.0 <= age <= timeout_sec


def _positive_timeout(value: float, name: str) -> float:
    """Validate one timeout instead of allowing a disabled safety gate."""
    timeout = float(value)
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError('%s must be finite and positive' % name)
    return timeout
