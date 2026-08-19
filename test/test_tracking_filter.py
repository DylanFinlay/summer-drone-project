"""Tests for target smoothing and continuous tracking deadbands."""

import unittest

from diy_autonomous_drone.tracking_filter import (
    apply_continuous_deadband,
    TargetObservationFilter,
)


class TestTargetObservationFilter(unittest.TestCase):
    """Verify filter initialization, smoothing, and safety reset."""

    def test_first_observation_is_not_biased_toward_zero(self):
        """Initialization preserves the first acquired target exactly."""
        target_filter = TargetObservationFilter(alpha=0.25)
        self.assertEqual(
            target_filter.update((0.4, -0.2, 0.5)),
            (0.4, -0.2, 0.5),
        )

    def test_exponential_smoothing_reduces_single_frame_jitter(self):
        """A sudden detection change contributes only its configured weight."""
        target_filter = TargetObservationFilter(alpha=0.25)
        target_filter.update((0.0, 0.0, 0.4))
        filtered = target_filter.update((0.4, -0.2, 0.8))
        self.assertAlmostEqual(filtered[0], 0.1)
        self.assertAlmostEqual(filtered[1], -0.05)
        self.assertAlmostEqual(filtered[2], 0.5)

    def test_alpha_one_disables_smoothing(self):
        """An alpha of one follows every valid observation directly."""
        target_filter = TargetObservationFilter(alpha=1.0)
        target_filter.update((0.0, 0.0, 0.4))
        self.assertEqual(
            target_filter.update((0.5, 0.1, 0.7)),
            (0.5, 0.1, 0.7),
        )

    def test_reset_prevents_old_target_leaking_into_reacquisition(self):
        """The first observation after target loss starts a new filter."""
        target_filter = TargetObservationFilter(alpha=0.25)
        target_filter.update((-0.8, 0.0, 0.2))
        target_filter.reset()
        self.assertIsNone(target_filter.current)
        self.assertEqual(
            target_filter.update((0.8, 0.0, 0.6)),
            (0.8, 0.0, 0.6),
        )

    def test_invalid_filter_inputs_are_rejected(self):
        """Invalid configuration and observations cannot reach control."""
        with self.assertRaises(ValueError):
            TargetObservationFilter(alpha=0.0)
        target_filter = TargetObservationFilter(alpha=0.5)
        with self.assertRaises(ValueError):
            target_filter.update((float('nan'), 0.0, 0.5))


class TestContinuousDeadband(unittest.TestCase):
    """Verify jitter suppression without an edge discontinuity."""

    def test_errors_inside_deadband_are_zero(self):
        """Small positive and negative detector errors are ignored."""
        self.assertEqual(apply_continuous_deadband(0.03, 0.04), 0.0)
        self.assertEqual(apply_continuous_deadband(-0.04, 0.04), 0.0)

    def test_error_outside_deadband_has_width_removed(self):
        """Output grows continuously from zero beyond either edge."""
        self.assertAlmostEqual(
            apply_continuous_deadband(0.10, 0.04), 0.06)
        self.assertAlmostEqual(
            apply_continuous_deadband(-0.10, 0.04), -0.06)

    def test_zero_width_preserves_error(self):
        """A zero-width deadband provides an explicit disable setting."""
        self.assertEqual(apply_continuous_deadband(-0.2, 0.0), -0.2)

    def test_invalid_deadband_is_rejected(self):
        """Negative or non-finite widths cannot silently disable control."""
        with self.assertRaises(ValueError):
            apply_continuous_deadband(0.1, -0.1)


if __name__ == '__main__':
    unittest.main()
