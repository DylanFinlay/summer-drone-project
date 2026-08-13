"""Dependency-free recorded-video playback state for the vision node."""

from dataclasses import dataclass


@dataclass(frozen=True)
class FrameRead:
    """Result of reading one frame from a recorded-video capture."""

    success: bool
    frame: object
    restarted: bool


class VideoPlayback:
    """Read a capture once or loop it with an observable identity reset."""

    def __init__(
        self,
        capture,
        loop_enabled: bool,
        position_property,
    ) -> None:
        """Store an OpenCV-like capture and its frame-position property."""
        self._capture = capture
        self._loop_enabled = bool(loop_enabled)
        self._position_property = position_property

    def read(self) -> FrameRead:
        """Read one frame, rewinding once at EOF when looping is enabled."""
        success, frame = self._capture.read()
        if success and frame is not None:
            return FrameRead(True, frame, False)
        if not self._loop_enabled:
            return FrameRead(False, None, False)

        rewound = self._capture.set(self._position_property, 0)
        if not rewound:
            return FrameRead(False, None, False)
        success, frame = self._capture.read()
        if not success or frame is None:
            return FrameRead(False, None, False)
        return FrameRead(True, frame, True)
