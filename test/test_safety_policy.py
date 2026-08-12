"""Behavioral tests for both independent command safety gates."""

import unittest

from diy_autonomous_drone.safety_policy import (
    fc_authority_block_reason,
    fc_command_gate_reason,
    supervisor_fault_reason,
)


class TestSafetySupervisorPolicy(unittest.TestCase):
    """Verify target, command, proximity, and invalid-time stops."""

    def _reason(
        self,
        command_age=0.1,
        moving=True,
        target_age=0.1,
        too_close=False,
    ):
        """Evaluate the supervisor with safe defaults."""
        return supervisor_fault_reason(
            command_age_sec=command_age,
            command_requests_motion=moving,
            target_age_sec=target_age,
            target_too_close=too_close,
            watchdog_timeout_sec=0.5,
        )

    def test_fresh_motion_with_target_passes(self):
        """Fresh motion and target observations leave the gate open."""
        self.assertIsNone(self._reason())

    def test_missing_and_stale_commands_stop(self):
        """The raw-command watchdog fails closed before and after input."""
        self.assertEqual(
            self._reason(command_age=None),
            'waiting for first velocity command',
        )
        self.assertEqual(
            self._reason(command_age=0.51),
            'velocity command timeout',
        )

    def test_motion_requires_a_fresh_target(self):
        """Nonzero tracking motion cannot outlive its observation."""
        self.assertEqual(
            self._reason(target_age=None),
            'motion requested without a target observation',
        )
        self.assertEqual(
            self._reason(target_age=0.51),
            'target tracking timeout',
        )

    def test_proximity_limit_stops_motion(self):
        """A target inside the configured limit produces a distinct stop."""
        self.assertEqual(
            self._reason(too_close=True),
            'target inside minimum safety distance',
        )

    def test_hover_does_not_require_a_target(self):
        """Zero velocity remains valid without perception observations."""
        self.assertIsNone(self._reason(moving=False, target_age=None))

    def test_invalid_clock_ages_fail_closed(self):
        """Clock reversals and non-finite ages cannot bypass watchdogs."""
        for invalid_age in (-0.1, float('nan'), float('inf')):
            with self.subTest(age=invalid_age):
                self.assertEqual(
                    self._reason(command_age=invalid_age),
                    'velocity command timeout',
                )


class TestFlightControllerSafetyPolicy(unittest.TestCase):
    """Verify MAVROS authority and final command freshness gating."""

    def _authority(
        self,
        state_age=0.1,
        connected=True,
        mode='GUIDED',
        armed=True,
    ):
        """Evaluate FC authority with safe defaults."""
        return fc_authority_block_reason(
            state_age_sec=state_age,
            connected=connected,
            flight_mode=mode,
            armed=armed,
            state_timeout_sec=2.0,
            require_guided_mode=True,
            require_armed=True,
        )

    def test_healthy_guided_armed_state_grants_authority(self):
        """Only a fresh connected Guided and armed state opens authority."""
        self.assertIsNone(self._authority())

    def test_missing_stale_and_invalid_state_stop(self):
        """Absent or invalid MAVROS timing never grants authority."""
        self.assertEqual(
            self._authority(state_age=None), 'waiting for MAVROS state')
        for invalid_age in (2.1, -0.1, float('nan'), float('inf')):
            with self.subTest(age=invalid_age):
                self.assertEqual(
                    self._authority(state_age=invalid_age),
                    'MAVROS state timeout',
                )

    def test_disconnection_stops(self):
        """A current disconnected state closes the command gate."""
        self.assertEqual(
            self._authority(connected=False),
            'MAVROS is disconnected from the flight controller',
        )

    def test_wrong_or_unknown_mode_stops(self):
        """RC-selected authority must be Guided when that gate is enabled."""
        self.assertEqual(
            self._authority(mode='LOITER'),
            'flight mode is LOITER, not Guided',
        )
        self.assertEqual(
            self._authority(mode=''),
            'flight mode is UNKNOWN, not Guided',
        )

    def test_disarmed_vehicle_stops(self):
        """An otherwise healthy but disarmed vehicle cannot accept motion."""
        self.assertEqual(
            self._authority(armed=False), 'vehicle is disarmed')

    def test_authority_reason_has_priority(self):
        """The final gate preserves the upstream FC authority reason."""
        reason = fc_command_gate_reason(
            authority_reason='vehicle is disarmed',
            command_age_sec=0.1,
            command_timeout_sec=0.5,
        )
        self.assertEqual(reason, 'vehicle is disarmed')

    def test_new_and_stale_safety_commands_stop(self):
        """Authority alone cannot replay an absent or expired command."""
        self.assertEqual(
            fc_command_gate_reason(None, None, 0.5),
            'waiting for a new safety command',
        )
        for invalid_age in (0.51, -0.1, float('nan'), float('inf')):
            with self.subTest(age=invalid_age):
                self.assertEqual(
                    fc_command_gate_reason(None, invalid_age, 0.5),
                    'safety command timeout',
                )

    def test_fresh_safety_command_passes(self):
        """Fresh commands pass only after FC authority is healthy."""
        self.assertIsNone(fc_command_gate_reason(None, 0.1, 0.5))

    def test_invalid_timeout_is_rejected(self):
        """Invalid configuration cannot silently disable a watchdog."""
        with self.assertRaises(ValueError):
            fc_command_gate_reason(None, 0.1, 0.0)


if __name__ == '__main__':
    unittest.main()
