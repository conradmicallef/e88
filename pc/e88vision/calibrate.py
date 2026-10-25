"""Works out how the bottom camera is mounted, by nudging the drone and watching
which way the floor moves. Run it once, hovering about 1 m over a textured floor.

Sequence: settle, roll right, settle, roll left (returns), pitch forward,
pitch back. The ground shift seen during each nudge gives the image direction
for 'right' and 'forward'; A is the inverse of that.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .flow import FlowMeasurement
from .link import Sticks

STEPS = [  # (name, roll, pitch, seconds, measure_as)
    ("settle", 0, 0, 1.5, None),
    ("roll right", +1, 0, 0.5, "roll+"),
    ("settle", 0, 0, 1.2, "roll+"),
    ("roll left", -1, 0, 0.5, "roll-"),
    ("settle", 0, 0, 1.5, "roll-"),
    ("pitch forward", 0, +1, 0.5, "pitch+"),
    ("settle", 0, 0, 1.2, "pitch+"),
    ("pitch back", 0, -1, 0.5, "pitch-"),
    ("settle", 0, 0, 1.5, "pitch-"),
]


class Calibrator:
    def __init__(self, nudge: float = 0.25):
        self.nudge = nudge
        self.i = 0
        self.t_step = None
        self.acc = {k: np.zeros(2) for k in ("roll+", "roll-", "pitch+", "pitch-")}
        self.done = False
        self.result: dict | None = None
        self.status = "calibration starting"

    def update(self, m: FlowMeasurement | None, t: float) -> Sticks:
        if self.done:
            return Sticks()
        if self.t_step is None:
            self.t_step = t
        name, r, p, dur, key = STEPS[self.i]
        if key and m is not None and m.ok:
            self.acc[key] += (m.dx, m.dy)
        if t - self.t_step >= dur:
            self.i += 1
            self.t_step = t
            if self.i >= len(STEPS):
                self.done = True
                self.result = self.solve()
                return Sticks()
        name, r, p, _, _ = STEPS[self.i]
        self.status = f"calibrating: {name}"
        return Sticks(roll=r * self.nudge, pitch=p * self.nudge)

    def solve(self) -> dict:
        right = (self.acc["roll+"] - self.acc["roll-"]) / 2.0   # image shift when moving right
        fwd = (self.acc["pitch+"] - self.acc["pitch-"]) / 2.0
        R = np.column_stack([right / (np.linalg.norm(right) + 1e-9),
                             fwd / (np.linalg.norm(fwd) + 1e-9)])
        cond = np.linalg.cond(R)
        ok = bool(cond < 4 and np.linalg.norm(right) > 0.02 and np.linalg.norm(fwd) > 0.02)
        A = np.linalg.inv(R) if cond < 1e6 else np.eye(2)
        self.status = ("calibration OK" if ok else
                       "calibration unclear - more texture/light, or a bigger nudge") + f" (cond {cond:.1f})"
        return {"A": A.round(3).tolist(), "ok": ok, "cond": float(cond),
                "right_shift": right.round(4).tolist(), "fwd_shift": fwd.round(4).tolist()}

    def save(self, path: str | Path = "calibration.json") -> None:
        if self.result:
            Path(path).write_text(json.dumps(self.result, indent=2))
