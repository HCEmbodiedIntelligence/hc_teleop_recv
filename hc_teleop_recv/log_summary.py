"""Bounded, low-rate diagnostics; never retain individual input packets."""
from dataclasses import dataclass


@dataclass
class _Bucket:
    last_emit: float = float('-inf')
    pending: int = 0
    latest: str = ''


class LogSummary:
    def __init__(self, categories, interval=30.0):
        self.interval = interval
        self.buckets = {key: _Bucket() for key in categories}

    def record(self, category, detail):
        bucket = self.buckets[category]
        bucket.pending += 1
        bucket.latest = detail[:1024]

    def poll(self, now):
        messages = []
        for category, bucket in self.buckets.items():
            if bucket.pending and now - bucket.last_emit >= self.interval:
                messages.append((category, f'{category}: count_since_last_log={bucket.pending}; latest={bucket.latest}'))
                bucket.pending = 0
                bucket.last_emit = now
        return messages
