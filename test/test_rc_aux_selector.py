"""Tests for fail-closed RC auxiliary autonomy-mode selection."""

import unittest

from diy_autonomous_drone.rc_aux_selector import RcAuxModeSelector


class TestRcAuxModeSelector(unittest.TestCase):
    """Verify channel indexing, debounce, mapping, and stale behavior."""

    def setUp(self):
        """Create a channel-six selector with conservative mappings."""
        self.selector = RcAuxModeSelector(
            channel=6,
            low_max_pwm=1300,
            high_min_pwm=1700,
            confirm_samples=3,
            timeout_sec=0.5,
        )

    @staticmethod
    def channels(pwm):
        """Return a six-channel RC sample with one auxiliary value."""
        return [1500, 1500, 1000, 1500, 1000, pwm]

    def test_switch_position_requires_confirmation(self):
        """One noisy high sample cannot enable active tracking."""
        self.assertEqual(
            self.selector.update(self.channels(1900), 1.0), 'hover')
        self.assertEqual(
            self.selector.update(self.channels(1900), 1.1), 'hover')
        self.assertEqual(
            self.selector.update(self.channels(1900), 1.2),
            'active_track',
        )
        self.assertEqual(self.selector.confirmed_position, 'high')

    def test_position_transition_returns_to_hover_before_confirmation(self):
        """Leaving a moving position stops before the new switch settles."""
        for index in range(3):
            self.selector.update(self.channels(1900), 1.0 + index * 0.1)
        self.assertEqual(
            self.selector.update(self.channels(1100), 1.3), 'hover')
        self.assertEqual(self.selector.status(1.3), 'confirming:hover')

    def test_default_low_and_middle_positions_request_hover(self):
        """Only the high position enables tracking by default."""
        for pwm in (1100, 1100, 1100):
            requested = self.selector.update(self.channels(pwm), 2.0)
        self.assertEqual(requested, 'hover')

        for pwm in (1500, 1500, 1500):
            requested = self.selector.update(self.channels(pwm), 2.1)
        self.assertEqual(requested, 'hover')

    def test_stale_or_never_received_input_requests_hover(self):
        """RC loss cannot leave the previous autonomous mode selected."""
        self.assertEqual(self.selector.requested_mode(0.0), 'hover')
        for index in range(3):
            self.selector.update(self.channels(1900), index * 0.1)
        self.assertEqual(self.selector.requested_mode(0.3), 'active_track')
        self.assertEqual(self.selector.requested_mode(0.8), 'hover')
        self.assertEqual(self.selector.status(0.8), 'stale:hover')

    def test_missing_channel_does_not_refresh_input(self):
        """A short or invalid RCIn array eventually fails to hover."""
        for index in range(3):
            self.selector.update(self.channels(1900), index * 0.1)
        self.selector.update([1500] * 5, 0.3)
        self.assertEqual(self.selector.requested_mode(0.8), 'hover')

    def test_custom_gesture_mapping_is_supported_but_explicit(self):
        """A calibrated installation may later map high to gesture mode."""
        selector = RcAuxModeSelector(
            6, 1300, 1700, 2, 0.5,
            high_mode='gesture_control',
        )
        selector.update(self.channels(1900), 1.0)
        self.assertEqual(
            selector.update(self.channels(1900), 1.1),
            'gesture_control',
        )

    def test_invalid_configuration_is_rejected(self):
        """Unknown modes and ambiguous thresholds cannot silently run."""
        with self.assertRaises(ValueError):
            RcAuxModeSelector(0, 1300, 1700, 3, 0.5)
        with self.assertRaises(ValueError):
            RcAuxModeSelector(6, 1800, 1700, 3, 0.5)
        with self.assertRaises(ValueError):
            RcAuxModeSelector(
                6, 1300, 1700, 3, 0.5, high_mode='follow_me')


if __name__ == '__main__':
    unittest.main()
