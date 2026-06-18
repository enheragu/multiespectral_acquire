import time
import collections


class FreqCounter:
    """Frequency counter keeping the last N message timestamps (adaptive window)."""

    def __init__(self, window_secs=5.0):  # window_secs kept for API compat, unused
        self._timestamps = collections.deque(maxlen=10)
        self._total = 0
        self._running = False

    def start(self):
        print("[FreqCounter::start] Init freq counter.")
        self._timestamps.clear()
        self._total = 0
        self._running = True

    def stop(self):
        self._running = False

    def tick(self):
        if not self._running:
            return
        self._timestamps.append(time.monotonic())
        self._total += 1

    def getFreq(self):
        if len(self._timestamps) < 2:
            return 0.0
        span = self._timestamps[-1] - self._timestamps[0]
        if span <= 0.0:
            return 0.0
        return (len(self._timestamps) - 1) / span

    def countItems(self):
        return self._total

    def __str__(self):
        return f"{self.getFreq():.2f}"
