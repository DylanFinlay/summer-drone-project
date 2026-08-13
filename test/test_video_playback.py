"""Tests for safe recorded-video EOF and looping behavior."""

import unittest

from diy_autonomous_drone.video_playback import VideoPlayback


class FakeCapture:
    """Provide deterministic OpenCV-like reads and rewinds."""

    def __init__(self, reads, rewind_succeeds=True):
        """Store scripted read results and rewind behavior."""
        self._reads = list(reads)
        self._rewind_succeeds = rewind_succeeds
        self.set_calls = []

    def read(self):
        """Return the next scripted capture result."""
        return self._reads.pop(0)

    def set(self, property_id, value):
        """Record one rewind request and report its configured result."""
        self.set_calls.append((property_id, value))
        return self._rewind_succeeds


class TestVideoPlayback(unittest.TestCase):
    """Verify normal frames, terminal EOF, and safe loop restarts."""

    def test_successful_frame_does_not_rewind(self):
        """Ordinary reads preserve continuous target-selector state."""
        capture = FakeCapture([(True, 'frame')])
        result = VideoPlayback(capture, True, 1).read()
        self.assertTrue(result.success)
        self.assertEqual(result.frame, 'frame')
        self.assertFalse(result.restarted)
        self.assertEqual(capture.set_calls, [])

    def test_non_looping_eof_is_a_failed_read(self):
        """EOF is exposed so the vision node can publish immediate loss."""
        capture = FakeCapture([(False, None)])
        result = VideoPlayback(capture, False, 1).read()
        self.assertFalse(result.success)
        self.assertFalse(result.restarted)
        self.assertEqual(capture.set_calls, [])

    def test_looping_eof_rewinds_and_marks_identity_boundary(self):
        """A successful loop tells vision to clear its previous target ID."""
        capture = FakeCapture([(False, None), (True, 'first-frame')])
        result = VideoPlayback(capture, True, 7).read()
        self.assertTrue(result.success)
        self.assertEqual(result.frame, 'first-frame')
        self.assertTrue(result.restarted)
        self.assertEqual(capture.set_calls, [(7, 0)])

    def test_failed_rewind_remains_a_failed_read(self):
        """A backend rewind failure cannot reuse an earlier video frame."""
        capture = FakeCapture([(False, None)], rewind_succeeds=False)
        result = VideoPlayback(capture, True, 7).read()
        self.assertFalse(result.success)
        self.assertFalse(result.restarted)

    def test_failed_first_frame_after_rewind_remains_failed(self):
        """An empty or corrupt looping file reports target loss."""
        capture = FakeCapture([(False, None), (False, None)])
        result = VideoPlayback(capture, True, 7).read()
        self.assertFalse(result.success)
        self.assertFalse(result.restarted)


if __name__ == '__main__':
    unittest.main()
