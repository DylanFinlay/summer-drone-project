"""Hardware-independent target acquisition and identity-locking helpers."""

from dataclasses import dataclass
from math import hypot
from typing import Iterable, List, Optional, Tuple


@dataclass(frozen=True)
class BoundingBox:
    """One pixel-space object detection in ``x1, y1, x2, y2`` format."""

    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float

    @property
    def width(self) -> float:
        """Return the nonnegative box width."""
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        """Return the nonnegative box height."""
        return max(0.0, self.y2 - self.y1)

    @property
    def center(self) -> Tuple[float, float]:
        """Return the box centre in pixels."""
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    @property
    def area(self) -> float:
        """Return the nonnegative box area."""
        return self.width * self.height

    def normalized_pose(
        self, frame_width: int, frame_height: int
    ) -> Tuple[float, float, float]:
        """Return normalized centre X/Y and height for the ROS observation."""
        if frame_width <= 0 or frame_height <= 0:
            raise ValueError('Frame dimensions must be positive.')
        center_x, center_y = self.center
        normalized_x = 2.0 * center_x / frame_width - 1.0
        normalized_y = 2.0 * center_y / frame_height - 1.0
        normalized_height = self.height / frame_height
        return (
            _clamp(normalized_x, -1.0, 1.0),
            _clamp(normalized_y, -1.0, 1.0),
            _clamp(normalized_height, 0.0, 1.0),
        )


class TargetSelector:
    """Acquire one person conservatively and preserve its spatial identity."""

    def __init__(
        self,
        acquire_confirm_frames: int = 3,
        acquire_iou_threshold: float = 0.3,
        lock_iou_threshold: float = 0.15,
        ambiguity_iou_margin: float = 0.05,
        max_missed_frames: int = 5,
        require_single_person: bool = True,
    ) -> None:
        """Configure confirmation, association, and target-loss behavior."""
        self._acquire_confirm_frames = max(1, acquire_confirm_frames)
        self._acquire_iou_threshold = _unit_interval(acquire_iou_threshold)
        self._lock_iou_threshold = _unit_interval(lock_iou_threshold)
        self._ambiguity_iou_margin = max(0.0, ambiguity_iou_margin)
        self._max_missed_frames = max(0, max_missed_frames)
        self._require_single_person = require_single_person

        self._candidate_box: Optional[BoundingBox] = None
        self._candidate_hits = 0
        self._locked_box: Optional[BoundingBox] = None
        self._missed_frames = 0

    @property
    def is_locked(self) -> bool:
        """Return whether a target identity is currently retained."""
        return self._locked_box is not None

    def reset(self) -> None:
        """Forget all acquisition and tracking state."""
        self._candidate_box = None
        self._candidate_hits = 0
        self._locked_box = None
        self._missed_frames = 0

    def update(
        self,
        detections: Iterable[BoundingBox],
        frame_width: int,
        frame_height: int,
    ) -> Optional[BoundingBox]:
        """Return the safely associated target, or ``None`` for this frame."""
        valid = [box for box in detections if box.area > 0.0]
        if self._locked_box is not None:
            return self._update_locked_target(valid)
        return self._update_acquisition(valid, frame_width, frame_height)

    def _update_locked_target(
        self, detections: List[BoundingBox]
    ) -> Optional[BoundingBox]:
        """Associate detections with the existing target using box overlap."""
        ranked = sorted(
            ((intersection_over_union(self._locked_box, box), box)
             for box in detections),
            key=lambda item: item[0],
            reverse=True,
        )
        if not ranked or ranked[0][0] < self._lock_iou_threshold:
            return self._record_miss()

        best_iou, best_box = ranked[0]
        if len(ranked) > 1:
            second_iou = ranked[1][0]
            if best_iou - second_iou < self._ambiguity_iou_margin:
                return self._record_miss()

        self._locked_box = best_box
        self._missed_frames = 0
        return best_box

    def _update_acquisition(
        self,
        detections: List[BoundingBox],
        frame_width: int,
        frame_height: int,
    ) -> Optional[BoundingBox]:
        """Require a stable, unambiguous candidate before acquiring it."""
        if not detections or (
            self._require_single_person and len(detections) != 1
        ):
            self._clear_candidate()
            return None

        candidate = min(
            detections,
            key=lambda box: self._centre_distance(
                box, frame_width, frame_height),
        )
        if self._candidate_box is not None and intersection_over_union(
            self._candidate_box, candidate
        ) >= self._acquire_iou_threshold:
            self._candidate_hits += 1
        else:
            self._candidate_hits = 1
        self._candidate_box = candidate

        if self._candidate_hits < self._acquire_confirm_frames:
            return None

        self._locked_box = candidate
        self._missed_frames = 0
        self._clear_candidate()
        return candidate

    def _record_miss(self) -> None:
        """Count one unsafe association and clear an expired lock."""
        self._missed_frames += 1
        if self._missed_frames > self._max_missed_frames:
            self._locked_box = None
            self._missed_frames = 0
        return None

    def _clear_candidate(self) -> None:
        """Clear only the pending acquisition state."""
        self._candidate_box = None
        self._candidate_hits = 0

    @staticmethod
    def _centre_distance(
        box: BoundingBox, frame_width: int, frame_height: int
    ) -> float:
        """Return normalized distance from the image centre."""
        if frame_width <= 0 or frame_height <= 0:
            raise ValueError('Frame dimensions must be positive.')
        center_x, center_y = box.center
        return hypot(
            (center_x - frame_width / 2.0) / frame_width,
            (center_y - frame_height / 2.0) / frame_height,
        )


def intersection_over_union(first: BoundingBox, second: BoundingBox) -> float:
    """Return intersection-over-union for two axis-aligned boxes."""
    intersection_width = max(
        0.0, min(first.x2, second.x2) - max(first.x1, second.x1))
    intersection_height = max(
        0.0, min(first.y2, second.y2) - max(first.y1, second.y1))
    intersection = intersection_width * intersection_height
    union = first.area + second.area - intersection
    if union <= 0.0:
        return 0.0
    return intersection / union


def _clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a number to an inclusive range."""
    return max(lower, min(upper, float(value)))


def _unit_interval(value: float) -> float:
    """Clamp a configuration value to the unit interval."""
    return _clamp(value, 0.0, 1.0)
