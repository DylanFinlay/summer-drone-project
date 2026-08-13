"""Tests for safe operator-tool command planning."""

import unittest

from diy_autonomous_drone.operator_control import command_plan


class TestOperatorCommandPlan(unittest.TestCase):
    """Verify every command has deliberate ordered effects."""

    def _changes(self, command):
        """Return planned changes as simple name/value pairs."""
        return [
            (change.name, change.value)
            for change in command_plan(command).changes
        ]

    def test_status_has_no_mutating_plan(self):
        """Status inspection never changes mode or feature locks."""
        self.assertIsNone(command_plan('status'))

    def test_hover_and_track_change_only_autonomy_mode(self):
        """Normal MVP modes do not alter the gesture feature lock."""
        self.assertEqual(self._changes('hover'), [
            ('autonomy_mode', 'hover'),
        ])
        self.assertEqual(self._changes('track'), [
            ('autonomy_mode', 'active_track'),
        ])

    def test_gesture_unlock_precedes_mode_selection(self):
        """Gesture mode follows the node's required safe update order."""
        self.assertEqual(self._changes('gesture'), [
            ('enable_gesture_control', True),
            ('autonomy_mode', 'gesture_control'),
        ])

    def test_lock_gesture_returns_to_hover_first(self):
        """The experimental feature is never disabled while it is active."""
        self.assertEqual(self._changes('lock-gesture'), [
            ('autonomy_mode', 'hover'),
            ('enable_gesture_control', False),
        ])

    def test_prepare_shutdown_enters_hover_and_requests_zero_burst(self):
        """Orderly shutdown preparation combines state reset and zeros."""
        plan = command_plan('prepare-shutdown')
        self.assertEqual(self._changes('prepare-shutdown'), [
            ('autonomy_mode', 'hover'),
        ])
        self.assertTrue(plan.zero_burst)

    def test_unknown_command_is_rejected(self):
        """Typos cannot silently select a fallback mode."""
        with self.assertRaises(ValueError):
            command_plan('follow')


if __name__ == '__main__':
    unittest.main()
