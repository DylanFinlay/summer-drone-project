"""Tests for conservative pose-based arm gesture recognition."""

import unittest

from diy_autonomous_drone.gesture_recognizer import (
    ArmGestureClassifier,
    GESTURE_DOWN,
    GESTURE_LEFT,
    GESTURE_NONE,
    GESTURE_RIGHT,
    GESTURE_UP,
    GestureDebouncer,
    PoseKeypoint,
    keypoints_from_values,
)


def pose(left_arm, right_arm, confidence=0.9):
    """Build COCO keypoints with configurable shoulder/elbow/wrist arms."""
    points = [PoseKeypoint(0.5, 0.1, confidence) for _ in range(17)]
    points[5], points[7], points[9] = (
        PoseKeypoint(x, y, confidence) for x, y in left_arm)
    points[6], points[8], points[10] = (
        PoseKeypoint(x, y, confidence) for x, y in right_arm)
    points[11] = PoseKeypoint(0.43, 0.60, confidence)
    points[12] = PoseKeypoint(0.57, 0.60, confidence)
    return points


class TestArmGestureClassifier(unittest.TestCase):
    """Verify large, unambiguous poses and fail-closed cases."""

    def setUp(self):
        """Create a classifier with ordinary confidence requirements."""
        self.classifier = ArmGestureClassifier(min_confidence=0.55)
        self.resting_left = ((0.43, 0.30), (0.42, 0.45), (0.42, 0.58))
        self.resting_right = ((0.57, 0.30), (0.58, 0.45), (0.58, 0.58))

    def test_both_arms_up(self):
        """Two clearly raised arms request upward movement."""
        keypoints = pose(
            ((0.43, 0.30), (0.40, 0.22), (0.38, 0.08)),
            ((0.57, 0.30), (0.60, 0.22), (0.62, 0.08)),
        )
        self.assertEqual(self.classifier.classify(keypoints), GESTURE_UP)

    def test_both_arms_down_and_out(self):
        """Down requires outward separation instead of a neutral stance."""
        keypoints = pose(
            ((0.43, 0.30), (0.35, 0.45), (0.27, 0.64)),
            ((0.57, 0.30), (0.65, 0.45), (0.73, 0.64)),
        )
        self.assertEqual(self.classifier.classify(keypoints), GESTURE_DOWN)

    def test_image_left_and_right_signals(self):
        """One horizontal arm plus one resting arm selects image direction."""
        left = pose(
            ((0.43, 0.30), (0.30, 0.30), (0.15, 0.30)),
            self.resting_right,
        )
        right = pose(
            self.resting_left,
            ((0.57, 0.30), (0.70, 0.30), (0.85, 0.30)),
        )
        self.assertEqual(self.classifier.classify(left), GESTURE_LEFT)
        self.assertEqual(self.classifier.classify(right), GESTURE_RIGHT)

    def test_neutral_or_low_confidence_pose_stops(self):
        """Natural rest and uncertain landmarks never request movement."""
        neutral = pose(self.resting_left, self.resting_right)
        uncertain = pose(self.resting_left, self.resting_right, confidence=0.2)
        self.assertEqual(self.classifier.classify(neutral), GESTURE_NONE)
        self.assertEqual(self.classifier.classify(uncertain), GESTURE_NONE)

    def test_missing_landmarks_stop(self):
        """Incomplete model output fails closed."""
        self.assertEqual(self.classifier.classify([]), GESTURE_NONE)


class TestGestureDebouncer(unittest.TestCase):
    """Verify confirmation and immediate release behavior."""

    def test_repeated_gesture_is_required(self):
        """Movement starts only after the configured consecutive frames."""
        debouncer = GestureDebouncer(confirm_frames=3)
        self.assertEqual(debouncer.update(GESTURE_UP), GESTURE_NONE)
        self.assertEqual(debouncer.update(GESTURE_UP), GESTURE_NONE)
        self.assertEqual(debouncer.update(GESTURE_UP), GESTURE_UP)

    def test_ambiguity_stops_immediately_and_resets_confirmation(self):
        """One no-gesture frame cancels movement and prior confirmation."""
        debouncer = GestureDebouncer(confirm_frames=2)
        debouncer.update(GESTURE_LEFT)
        self.assertEqual(debouncer.update(GESTURE_LEFT), GESTURE_LEFT)
        self.assertEqual(debouncer.update(GESTURE_NONE), GESTURE_NONE)
        self.assertEqual(debouncer.update(GESTURE_LEFT), GESTURE_NONE)

    def test_conversion_rejects_mismatched_model_values(self):
        """Tensor conversion cannot silently drop keypoint confidence."""
        with self.assertRaises(ValueError):
            keypoints_from_values([[0.1, 0.2]], [])


if __name__ == '__main__':
    unittest.main()
