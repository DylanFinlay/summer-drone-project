"""Behavioral tests for conservative person-target selection."""

import unittest

from diy_autonomous_drone.target_selector import (
    BoundingBox,
    TargetSelector,
    intersection_over_union,
)


class TestBoundingBox(unittest.TestCase):
    """Verify bounding-box geometry and ROS observation conversion."""

    def test_normalized_pose(self):
        """Image centre and half-frame height normalize predictably."""
        box = BoundingBox(160.0, 120.0, 480.0, 360.0, 0.9)
        center_x, center_y, height = box.normalized_pose(640, 480)
        self.assertAlmostEqual(center_x, 0.0)
        self.assertAlmostEqual(center_y, 0.0)
        self.assertAlmostEqual(height, 0.5)

    def test_intersection_over_union(self):
        """Partially overlapping boxes produce the expected ratio."""
        first = BoundingBox(0.0, 0.0, 10.0, 10.0, 0.9)
        second = BoundingBox(5.0, 0.0, 15.0, 10.0, 0.8)
        self.assertAlmostEqual(
            intersection_over_union(first, second), 1.0 / 3.0)


class TestTargetSelector(unittest.TestCase):
    """Verify target confirmation, locking, ambiguity, and target loss."""

    def setUp(self):
        """Create a selector with compact values for deterministic tests."""
        self.selector = TargetSelector(
            acquire_confirm_frames=3,
            acquire_iou_threshold=0.3,
            lock_iou_threshold=0.15,
            ambiguity_iou_margin=0.05,
            max_missed_frames=2,
            require_single_person=True,
        )
        self.target = BoundingBox(250.0, 100.0, 390.0, 420.0, 0.9)

    def update(self, detections):
        """Update the selector using a 640 by 480 test frame."""
        return self.selector.update(detections, 640, 480)

    def acquire_target(self):
        """Submit the required number of stable acquisition frames."""
        self.assertIsNone(self.update([self.target]))
        self.assertIsNone(self.update([self.target]))
        selected = self.update([self.target])
        self.assertEqual(selected, self.target)
        self.assertTrue(self.selector.is_locked)

    def test_requires_consecutive_confirmation(self):
        """A single detection cannot immediately command motion."""
        self.assertIsNone(self.update([self.target]))
        self.assertFalse(self.selector.is_locked)

    def test_refuses_ambiguous_initial_scene(self):
        """Multiple people prevent default target acquisition."""
        second = BoundingBox(10.0, 100.0, 150.0, 420.0, 0.95)
        for _ in range(5):
            self.assertIsNone(self.update([self.target, second]))
        self.assertFalse(self.selector.is_locked)

    def test_preserves_locked_person_with_bystander(self):
        """A locked target remains selected when another person appears."""
        self.acquire_target()
        moved_target = BoundingBox(255.0, 102.0, 395.0, 422.0, 0.85)
        bystander = BoundingBox(20.0, 80.0, 160.0, 430.0, 0.99)
        selected = self.update([bystander, moved_target])
        self.assertEqual(selected, moved_target)

    def test_missing_target_stops_immediately_then_unlocks(self):
        """Missing frames return no output and eventually clear identity."""
        self.acquire_target()
        self.assertIsNone(self.update([]))
        self.assertTrue(self.selector.is_locked)
        self.assertIsNone(self.update([]))
        self.assertTrue(self.selector.is_locked)
        self.assertIsNone(self.update([]))
        self.assertFalse(self.selector.is_locked)

    def test_ambiguous_locked_scene_stops_for_that_frame(self):
        """Two similarly overlapping candidates cannot command movement."""
        self.acquire_target()
        first = BoundingBox(245.0, 100.0, 385.0, 420.0, 0.9)
        second = BoundingBox(255.0, 100.0, 395.0, 420.0, 0.9)
        self.assertIsNone(self.update([first, second]))
        self.assertTrue(self.selector.is_locked)

    def test_reacquisition_requires_fresh_confirmation(self):
        """A lost target cannot resume motion from one new detection."""
        self.acquire_target()
        for _ in range(3):
            self.assertIsNone(self.update([]))
        self.assertIsNone(self.update([self.target]))
        self.assertFalse(self.selector.is_locked)


if __name__ == '__main__':
    unittest.main()
