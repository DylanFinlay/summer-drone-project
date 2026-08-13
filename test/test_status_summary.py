"""Behavioral tests for conservative drone status aggregation."""

import unittest

from diy_autonomous_drone.status_summary import (
    DIAGNOSTIC_ERROR,
    DIAGNOSTIC_OK,
    DIAGNOSTIC_STALE,
    DIAGNOSTIC_WARN,
    DroneStatusModel,
)


class TestDroneStatusModel(unittest.TestCase):
    """Verify summary levels, locks, reasons, and stale-input handling."""

    def setUp(self):
        """Create a model with a one-second input timeout."""
        self.model = DroneStatusModel(input_timeout_sec=1.0)

    def _set_healthy_inputs(self, timestamp=10.0):
        """Populate every required input with an open command path."""
        self.model.set_autonomy_mode('active_track', timestamp)
        self.model.set_tracking_state('tracking', timestamp)
        self.model.set_fc_state(True, True, 'GUIDED', timestamp)
        self.model.set_supervisor_reason('', timestamp)
        self.model.set_fc_gate_reason('', timestamp)

    def test_healthy_tracking_reports_target_lock(self):
        """Fresh open gates and tracking produce an OK locked summary."""
        self._set_healthy_inputs()

        summary = self.model.snapshot(10.5)

        self.assertEqual(summary.level, DIAGNOSTIC_OK)
        self.assertEqual(summary.health, 'ok')
        self.assertTrue(summary.target_locked)
        self.assertEqual(summary.safety_stop_reason, 'none')

    def test_hover_is_not_a_target_lock(self):
        """Hover remains healthy when all enabled safety gates are open."""
        self._set_healthy_inputs()
        self.model.set_autonomy_mode('hover', 10.0)
        self.model.set_tracking_state('hover', 10.0)

        summary = self.model.snapshot(10.5)

        self.assertEqual(summary.level, DIAGNOSTIC_OK)
        self.assertFalse(summary.target_locked)

    def test_both_gate_reasons_are_preserved(self):
        """Independent safety causes remain visible instead of masking."""
        self._set_healthy_inputs()
        self.model.set_supervisor_reason('target tracking timeout', 10.0)
        self.model.set_fc_gate_reason('vehicle is disarmed', 10.0)

        summary = self.model.snapshot(10.5)

        self.assertEqual(summary.level, DIAGNOSTIC_WARN)
        self.assertIn('target tracking timeout', summary.safety_stop_reason)
        self.assertIn('vehicle is disarmed', summary.safety_stop_reason)

    def test_disconnected_fc_is_an_error(self):
        """A fresh explicit disconnect is more severe than a closed gate."""
        self._set_healthy_inputs()
        self.model.set_fc_state(False, False, '', 10.0)
        self.model.set_fc_gate_reason(
            'MAVROS is disconnected from the flight controller', 10.0)

        summary = self.model.snapshot(10.5)

        self.assertEqual(summary.level, DIAGNOSTIC_ERROR)
        self.assertFalse(summary.fc_connected)
        self.assertIn('disconnected', summary.safety_stop_reason)

    def test_missing_or_expired_inputs_are_stale(self):
        """The summary never treats absent component reports as healthy."""
        initial = self.model.snapshot(0.0)
        self.assertEqual(initial.level, DIAGNOSTIC_STALE)
        self.assertEqual(initial.autonomy_mode, 'unknown')

        self._set_healthy_inputs(timestamp=10.0)
        expired = self.model.snapshot(11.1)
        self.assertEqual(expired.level, DIAGNOSTIC_STALE)
        self.assertFalse(expired.target_locked)
        self.assertIn('unavailable', expired.safety_stop_reason)

    def test_disabled_components_are_explicit(self):
        """Intentional feature toggles do not appear as missing heartbeats."""
        model = DroneStatusModel(
            input_timeout_sec=1.0,
            expect_fc_interface=False,
            expect_tracking=False,
        )
        model.set_supervisor_reason('', 10.0)

        summary = model.snapshot(10.5)

        self.assertEqual(summary.level, DIAGNOSTIC_WARN)
        self.assertEqual(summary.autonomy_mode, 'disabled')
        self.assertEqual(summary.fc_mode, 'disabled')
        self.assertIn('interface disabled', summary.safety_stop_reason)

    def test_invalid_timeout_is_rejected(self):
        """A nonpositive freshness period cannot disable stale detection."""
        with self.assertRaises(ValueError):
            DroneStatusModel(input_timeout_sec=0.0)

    def test_enabled_rc_aux_requires_fresh_state(self):
        """An expected spare-channel selector is visible and fail-closed."""
        model = DroneStatusModel(
            input_timeout_sec=1.0,
            expect_fc_interface=False,
            expect_tracking=False,
            expect_rc_aux=True,
        )
        model.set_supervisor_reason('', 10.0)
        self.assertEqual(model.snapshot(10.0).rc_aux_state, 'unknown')

        model.set_rc_aux_state('stale:hover', 10.0)
        summary = model.snapshot(10.5)
        self.assertEqual(summary.rc_aux_state, 'stale:hover')
        self.assertIn('RC auxiliary input stale', summary.safety_stop_reason)

    def test_nonfinite_timestamps_are_rejected(self):
        """Invalid time values cannot make status inputs permanently fresh."""
        with self.assertRaises(ValueError):
            self.model.set_autonomy_mode('hover', float('nan'))


if __name__ == '__main__':
    unittest.main()
