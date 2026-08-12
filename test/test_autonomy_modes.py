"""Tests for autonomy-mode feature gates."""

import unittest

from diy_autonomous_drone.autonomy_modes import (
    MODE_ACTIVE_TRACK,
    MODE_GESTURE_CONTROL,
    MODE_HOVER,
    mode_rejection_reason,
    normalized_mode,
)


class TestAutonomyModes(unittest.TestCase):
    """Verify valid modes and the experimental gesture feature gate."""

    def test_normalizes_launch_input(self):
        """Launch-time input can be normalized before validation."""
        self.assertEqual(normalized_mode(' Active_Track '), MODE_ACTIVE_TRACK)

    def test_all_non_gesture_modes_are_available(self):
        """Hover and active tracking do not depend on the gesture toggle."""
        self.assertIsNone(mode_rejection_reason(MODE_HOVER, False))
        self.assertIsNone(
            mode_rejection_reason(MODE_ACTIVE_TRACK, False))

    def test_gesture_mode_requires_explicit_enable(self):
        """Gesture mode fails closed until its feature flag is enabled."""
        reason = mode_rejection_reason(MODE_GESTURE_CONTROL, False)
        self.assertIn('disabled', reason)
        self.assertIsNone(
            mode_rejection_reason(MODE_GESTURE_CONTROL, True))

    def test_unknown_mode_is_rejected(self):
        """Misspelled or unsupported modes cannot become active."""
        reason = mode_rejection_reason('follow_anyone', True)
        self.assertIn('must be one of', reason)


if __name__ == '__main__':
    unittest.main()
