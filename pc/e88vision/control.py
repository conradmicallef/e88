"""Small control helpers shared by the autopilots."""
from __future__ import annotations

import collections
import math


def clamp(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


class PID:
    """PID with derivative on measurement, low-pass on D, and integral clamp."""

    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0,
                 i_limit: float = 0.2, out_limit: float = 1.0, d_tau: float = 0.1):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit, self.d_tau = i_limit, out_limit, d_tau
        self.reset()

    def reset(self):
        self.i = 0.0
        self.prev = None
        self.d = 0.0

    def update(self, error: float, dt: float, measurement_rate: float | None = None) -> float:
        dt = max(dt, 1e-3)
        self.i = clamp(self.i + error * dt * self.ki, self.i_limit)
        if measurement_rate is not None:
            raw_d = -measurement_rate
        else:
            raw_d = 0.0 if self.prev is None else (error - self.prev) / dt
        a = dt / (self.d_tau + dt)
        self.d += a * (raw_d - self.d)
        self.prev = error
        return clamp(self.kp * error + self.i + self.kd * self.d, self.out_limit)


class Slew:
    """Limits how fast a command may change (units per second)."""

    def __init__(self, rate: float):
        self.rate = rate
        self.v = 0.0

    def __call__(self, target: float, dt: float) -> float:
        step = self.rate * max(dt, 1e-3)
        self.v += clamp(target - self.v, step)
        return self.v

    def reset(self, v: float = 0.0):
        self.v = v


class DelayLine:
    """Returns the value that was pushed `delay` seconds ago (for lag compensation)."""

    def __init__(self, delay: float):
        self.delay = delay
        self.buf: collections.deque = collections.deque()

    def push(self, t: float, v):
        self.buf.append((t, v))

    def at(self, t: float, default=0.0):
        target = t - self.delay
        # drop entries superseded by a newer one that is still old enough
        while len(self.buf) > 1 and self.buf[1][0] <= target:
            self.buf.popleft()
        if self.buf and self.buf[0][0] <= target:
            return self.buf[0][1]
        return default


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
