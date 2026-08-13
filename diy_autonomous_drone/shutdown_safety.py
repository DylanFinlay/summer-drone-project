"""Best-effort repeated zero publication for orderly process shutdown."""

import math
import time


def publish_zero_burst(
    publisher,
    message_factory,
    count: int,
    interval_sec: float,
    sleeper=time.sleep,
) -> int:
    """Publish several fresh zero messages and return the sent count."""
    message_count = int(count)
    interval = float(interval_sec)
    if message_count <= 0:
        raise ValueError('shutdown zero count must be positive')
    if not math.isfinite(interval) or interval < 0.0:
        raise ValueError(
            'shutdown zero interval must be finite and nonnegative')

    for index in range(message_count):
        publisher.publish(message_factory())
        if index + 1 < message_count and interval > 0.0:
            sleeper(interval)
    return message_count
