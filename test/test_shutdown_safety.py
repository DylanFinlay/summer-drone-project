"""Tests for best-effort repeated zero publication."""

import unittest

from diy_autonomous_drone.shutdown_safety import publish_zero_burst


class CapturingPublisher:
    """Capture all messages published by a shutdown burst."""

    def __init__(self):
        """Start with no messages."""
        self.messages = []

    def publish(self, message):
        """Record one message."""
        self.messages.append(message)


class TestPublishZeroBurst(unittest.TestCase):
    """Verify message freshness, counts, pacing, and validation."""

    def test_each_zero_is_a_fresh_message(self):
        """The burst does not repeatedly mutate or reuse one message."""
        publisher = CapturingPublisher()
        sleeps = []
        sent = publish_zero_burst(
            publisher,
            dict,
            count=5,
            interval_sec=0.025,
            sleeper=sleeps.append,
        )
        self.assertEqual(sent, 5)
        self.assertEqual(len(publisher.messages), 5)
        self.assertEqual(len({id(item) for item in publisher.messages}), 5)
        self.assertEqual(sleeps, [0.025] * 4)

    def test_zero_interval_does_not_sleep(self):
        """Tests and special callers can request an immediate burst."""
        publisher = CapturingPublisher()
        sleeps = []
        publish_zero_burst(
            publisher, dict, count=2, interval_sec=0.0,
            sleeper=sleeps.append)
        self.assertEqual(len(publisher.messages), 2)
        self.assertEqual(sleeps, [])

    def test_invalid_count_is_rejected(self):
        """A configured graceful stop must publish at least one zero."""
        with self.assertRaises(ValueError):
            publish_zero_burst(
                CapturingPublisher(), dict, 0, 0.1)

    def test_invalid_interval_is_rejected(self):
        """Negative or non-finite timing cannot create an invalid burst."""
        for interval in (-0.1, float('nan'), float('inf')):
            with self.subTest(interval=interval):
                with self.assertRaises(ValueError):
                    publish_zero_burst(
                        CapturingPublisher(), dict, 1, interval)


if __name__ == '__main__':
    unittest.main()
