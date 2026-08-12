"""Behavioral tests for velocity acceleration limiting."""

import math
import unittest

from diy_autonomous_drone.velocity_limiter import VelocityLimiter


class TestVelocityLimiter(unittest.TestCase):
    """Verify bounded ramps, reversals, timing, and reset behavior."""

    def setUp(self):
        """Create a limiter with simple deterministic limits."""
        self.limiter = VelocityLimiter(
            max_linear_acceleration=1.0,
            max_yaw_acceleration=2.0,
            max_dt=0.2,
        )

    def test_first_command_starts_at_rest(self):
        """A fresh or reset limiter cannot jump directly into motion."""
        output = self.limiter.limit((1.0, 0.0, 0.0, 1.0), 10.0)
        self.assertEqual(output, (0.0, 0.0, 0.0, 0.0))

    def test_linear_vector_magnitude_is_limited(self):
        """Diagonal motion obeys one vector acceleration limit."""
        self.limiter.limit((1.0, 1.0, 0.0, 0.0), 0.0)
        output = self.limiter.limit((1.0, 1.0, 0.0, 0.0), 0.1)
        magnitude = math.sqrt(sum(value * value for value in output[:3]))
        self.assertAlmostEqual(magnitude, 0.1)
        self.assertAlmostEqual(output[0], output[1])

    def test_yaw_rate_has_an_independent_limit(self):
        """Yaw rate ramps using its configured angular acceleration."""
        self.limiter.limit((0.0, 0.0, 0.0, 1.0), 0.0)
        output = self.limiter.limit((0.0, 0.0, 0.0, 1.0), 0.1)
        self.assertAlmostEqual(output[3], 0.2)

    def test_direction_reversal_cannot_jump_through_zero(self):
        """A reversal consumes the same bounded velocity-change budget."""
        self.limiter.limit((1.0, 0.0, 0.0, 0.0), 0.0)
        self.limiter.limit((1.0, 0.0, 0.0, 0.0), 0.2)
        output = self.limiter.limit((-1.0, 0.0, 0.0, 0.0), 0.3)
        self.assertAlmostEqual(output[0], 0.1)

    def test_elapsed_time_is_capped_after_a_pause(self):
        """A delayed timer cannot spend an unbounded acceleration budget."""
        self.limiter.limit((1.0, 0.0, 0.0, 0.0), 0.0)
        output = self.limiter.limit((1.0, 0.0, 0.0, 0.0), 10.0)
        self.assertAlmostEqual(output[0], 0.2)

    def test_reset_requires_a_new_ramp_from_zero(self):
        """Safety resets discard the earlier velocity and timestamp."""
        self.limiter.limit((1.0, 0.0, 0.0, 0.0), 0.0)
        self.limiter.limit((1.0, 0.0, 0.0, 0.0), 0.2)
        self.limiter.reset()
        output = self.limiter.limit((1.0, 0.0, 0.0, 0.0), 0.3)
        self.assertEqual(output, (0.0, 0.0, 0.0, 0.0))

    def test_invalid_limits_are_rejected(self):
        """Invalid limits cannot silently disable safe behavior."""
        with self.assertRaises(ValueError):
            VelocityLimiter(0.0, 1.0, 0.1)
        with self.assertRaises(ValueError):
            VelocityLimiter(float('inf'), 1.0, 0.1)


if __name__ == '__main__':
    unittest.main()
