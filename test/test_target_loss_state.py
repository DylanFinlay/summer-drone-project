"""Behavioral tests for explicit target-loss states."""

import unittest

from diy_autonomous_drone.target_loss_state import (
    TargetLossStateMachine,
    TargetTrackingState,
)


class TestTargetLossStateMachine(unittest.TestCase):
    """Verify acquisition, loss, reacquisition, and expiry behavior."""

    def setUp(self):
        """Create a state machine with a one-second grace window."""
        self.machine = TargetLossStateMachine(loss_grace_sec=1.0)

    def test_starts_in_hover(self):
        """No target can imply tracking at startup."""
        self.assertEqual(self.machine.state, TargetTrackingState.HOVER)

    def test_valid_target_enters_tracking(self):
        """A selected target explicitly activates tracking state."""
        self.assertEqual(
            self.machine.target_seen(), TargetTrackingState.TRACKING)

    def test_miss_enters_temporary_loss(self):
        """The first miss after tracking starts the grace window."""
        self.machine.target_seen()
        state = self.machine.target_missed(10.0)
        self.assertEqual(state, TargetTrackingState.TEMPORARILY_LOST)
        self.assertEqual(self.machine.lost_since, 10.0)

    def test_repeated_miss_does_not_extend_grace_window(self):
        """Repeated missing frames cannot postpone the hover deadline."""
        self.machine.target_seen()
        self.machine.target_missed(10.0)
        self.machine.target_missed(10.8)
        self.assertEqual(self.machine.lost_since, 10.0)
        self.assertEqual(
            self.machine.update(11.0), TargetTrackingState.HOVER)

    def test_reacquisition_within_grace_returns_to_tracking(self):
        """A fresh target can resume from rest before loss expires."""
        self.machine.target_seen()
        self.machine.target_missed(10.0)
        self.machine.update(10.9)
        self.assertEqual(
            self.machine.target_seen(), TargetTrackingState.TRACKING)
        self.assertIsNone(self.machine.lost_since)

    def test_expired_loss_returns_to_hover(self):
        """No reacquisition by the deadline produces stable hover."""
        self.machine.target_seen()
        self.machine.target_missed(10.0)
        self.assertEqual(
            self.machine.update(10.99),
            TargetTrackingState.TEMPORARILY_LOST,
        )
        self.assertEqual(
            self.machine.update(11.0), TargetTrackingState.HOVER)

    def test_miss_while_hovering_does_not_start_loss_timer(self):
        """Missing frames before acquisition leave the state in hover."""
        self.assertEqual(
            self.machine.target_missed(10.0), TargetTrackingState.HOVER)
        self.assertIsNone(self.machine.lost_since)

    def test_invalid_grace_period_is_rejected(self):
        """An invalid grace period cannot create an unbounded loss state."""
        with self.assertRaises(ValueError):
            TargetLossStateMachine(loss_grace_sec=0.0)

    def test_reset_clears_tracking_and_loss_time(self):
        """A mode change returns the state machine to a clean hover."""
        self.machine.target_seen()
        self.machine.target_missed(10.0)
        self.assertEqual(self.machine.reset(), TargetTrackingState.HOVER)
        self.assertIsNone(self.machine.lost_since)


if __name__ == '__main__':
    unittest.main()
