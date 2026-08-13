"""Dependency-free arm-gesture classification and confirmation."""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple


GESTURE_NONE = 0
GESTURE_UP = 2
GESTURE_DOWN = 3
GESTURE_LEFT = 4
GESTURE_RIGHT = 5
VALID_GESTURES = frozenset({
    GESTURE_NONE,
    GESTURE_UP,
    GESTURE_DOWN,
    GESTURE_LEFT,
    GESTURE_RIGHT,
})


@dataclass(frozen=True)
class PoseKeypoint:
    """One normalized COCO pose keypoint and its confidence."""

    x: float
    y: float
    confidence: float


class ArmGestureClassifier:
    """Classify deliberately large arm poses from COCO keypoints."""

    LEFT_SHOULDER = 5
    RIGHT_SHOULDER = 6
    LEFT_ELBOW = 7
    RIGHT_ELBOW = 8
    LEFT_WRIST = 9
    RIGHT_WRIST = 10
    LEFT_HIP = 11
    RIGHT_HIP = 12

    def __init__(self, min_confidence: float = 0.55) -> None:
        """Set the minimum confidence required for every body landmark."""
        confidence = float(min_confidence)
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError('min_confidence must be between zero and one')
        self._min_confidence = confidence

    def classify(self, keypoints: Sequence[PoseKeypoint]) -> int:
        """Return one gesture ID, or zero when the pose is unsafe/unclear."""
        required_indices = (
            self.LEFT_SHOULDER,
            self.RIGHT_SHOULDER,
            self.LEFT_ELBOW,
            self.RIGHT_ELBOW,
            self.LEFT_WRIST,
            self.RIGHT_WRIST,
            self.LEFT_HIP,
            self.RIGHT_HIP,
        )
        if len(keypoints) <= max(required_indices):
            return GESTURE_NONE
        points = tuple(keypoints[index] for index in required_indices)
        if any(not self._usable(point) for point in points):
            return GESTURE_NONE

        left_shoulder, right_shoulder = points[0], points[1]
        left_elbow, right_elbow = points[2], points[3]
        left_wrist, right_wrist = points[4], points[5]
        left_hip, right_hip = points[6], points[7]

        shoulder_y = (left_shoulder.y + right_shoulder.y) / 2.0
        hip_y = (left_hip.y + right_hip.y) / 2.0
        torso = hip_y - shoulder_y
        if torso <= 0.05:
            return GESTURE_NONE

        arms = (
            (left_shoulder, left_elbow, left_wrist),
            (right_shoulder, right_elbow, right_wrist),
        )
        if all(self._arm_up(*arm, torso) for arm in arms):
            return GESTURE_UP
        if all(self._arm_down_and_out(*arm, torso) for arm in arms):
            return GESTURE_DOWN

        leftward = [self._arm_horizontal(*arm, torso, -1.0) for arm in arms]
        rightward = [self._arm_horizontal(*arm, torso, 1.0) for arm in arms]
        resting = [self._arm_resting(*arm, torso) for arm in arms]
        if (leftward[0] and resting[1]) or (leftward[1] and resting[0]):
            return GESTURE_LEFT
        if (rightward[0] and resting[1]) or (rightward[1] and resting[0]):
            return GESTURE_RIGHT
        return GESTURE_NONE

    def _usable(self, point: PoseKeypoint) -> bool:
        """Return whether one keypoint is finite and sufficiently certain."""
        return (
            math.isfinite(point.x)
            and math.isfinite(point.y)
            and math.isfinite(point.confidence)
            and point.confidence >= self._min_confidence
        )

    @staticmethod
    def _arm_up(
        shoulder: PoseKeypoint,
        elbow: PoseKeypoint,
        wrist: PoseKeypoint,
        torso: float,
    ) -> bool:
        """Return whether one arm is deliberately extended upward."""
        return (
            elbow.y < shoulder.y - 0.05 * torso
            and wrist.y < elbow.y - 0.15 * torso
        )

    @staticmethod
    def _arm_down_and_out(
        shoulder: PoseKeypoint,
        elbow: PoseKeypoint,
        wrist: PoseKeypoint,
        torso: float,
    ) -> bool:
        """Return whether one arm points down and away from the torso."""
        shoulder_mid_x = shoulder.x
        horizontal_offset = abs(wrist.x - shoulder_mid_x)
        return (
            elbow.y > shoulder.y + 0.25 * torso
            and wrist.y > elbow.y + 0.15 * torso
            and horizontal_offset > 0.25 * torso
        )

    @staticmethod
    def _arm_horizontal(
        shoulder: PoseKeypoint,
        elbow: PoseKeypoint,
        wrist: PoseKeypoint,
        torso: float,
        direction: float,
    ) -> bool:
        """Return whether one arm points horizontally in image direction."""
        elbow_offset = direction * (elbow.x - shoulder.x)
        wrist_offset = direction * (wrist.x - shoulder.x)
        return (
            elbow_offset > 0.20 * torso
            and wrist_offset > 0.50 * torso
            and abs(elbow.y - shoulder.y) < 0.30 * torso
            and abs(wrist.y - shoulder.y) < 0.35 * torso
        )

    @staticmethod
    def _arm_resting(
        shoulder: PoseKeypoint,
        elbow: PoseKeypoint,
        wrist: PoseKeypoint,
        torso: float,
    ) -> bool:
        """Return whether the other arm is clearly lowered, not signaling."""
        return (
            elbow.y > shoulder.y + 0.20 * torso
            and wrist.y > shoulder.y + 0.45 * torso
        )


class GestureDebouncer:
    """Require repeated agreement while stopping immediately on ambiguity."""

    def __init__(self, confirm_frames: int = 4) -> None:
        """Set the number of consecutive frames required for movement."""
        self._confirm_frames = int(confirm_frames)
        if self._confirm_frames <= 0:
            raise ValueError('confirm_frames must be positive')
        self.reset()

    def reset(self) -> None:
        """Clear both pending and confirmed gesture state."""
        self._candidate = GESTURE_NONE
        self._candidate_frames = 0
        self._confirmed = GESTURE_NONE

    def update(self, gesture: int) -> int:
        """Return a confirmed gesture, failing immediately to no movement."""
        observed = int(gesture)
        if observed not in VALID_GESTURES:
            raise ValueError('unknown gesture ID')
        if observed == GESTURE_NONE:
            self.reset()
            return GESTURE_NONE
        if observed != self._candidate:
            self._candidate = observed
            self._candidate_frames = 1
            self._confirmed = GESTURE_NONE
        else:
            self._candidate_frames += 1
        if self._candidate_frames >= self._confirm_frames:
            self._confirmed = observed
        return self._confirmed


def keypoints_from_values(
    coordinates: Sequence[Sequence[float]],
    confidences: Sequence[float],
) -> Tuple[PoseKeypoint, ...]:
    """Convert model values into validated immutable keypoint containers."""
    if len(coordinates) != len(confidences):
        raise ValueError('coordinate and confidence counts must match')
    return tuple(
        PoseKeypoint(float(point[0]), float(point[1]), float(confidence))
        for point, confidence in zip(coordinates, confidences)
    )
