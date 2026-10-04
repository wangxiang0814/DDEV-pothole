"""Freeze a shared straight-path reference from stationary observations."""
from collections import deque
import math

from .config import PATH_REFERENCE


class StaticPathReference:
    def __init__(self, *, window_s=PATH_REFERENCE.window_s,
                 period_s=PATH_REFERENCE.period_s, min_samples=PATH_REFERENCE.min_samples):
        if (not math.isfinite(window_s) or not math.isfinite(period_s) or
                min(window_s, period_s) <= 0 or min_samples < 2 or
                (min_samples - 1) * period_s > window_s):
            raise ValueError('invalid static path reference window')
        self.window_s, self.period_s, self.min_samples = window_s, period_s, min_samples
        self.samples = deque()
        self.last_s = None
        self.frozen = None

    @property
    def sample_count(self):
        return len(self.samples)

    def update(self, now_s, lateral_m, yaw_deg):
        if self.frozen is not None:
            return
        if not all(math.isfinite(v) for v in (now_s, lateral_m, yaw_deg)):
            raise ValueError('nonfinite path reference observation')
        if self.last_s is not None and now_s - self.last_s < self.period_s - 1e-8:
            return
        self.last_s = now_s
        self.samples.append((now_s, lateral_m, yaw_deg))
        while self.samples and self.samples[0][0] < now_s - self.window_s - 1e-8:
            self.samples.popleft()

    def freeze(self):
        if self.frozen is None:
            if self.sample_count < self.min_samples:
                raise ValueError('insufficient static reference samples')
            count = self.sample_count
            self.frozen = (sum(r[1] for r in self.samples) / count,
                           sum(r[2] for r in self.samples) / count)
        return self.frozen
