"""Follow a person with the front camera.

Detector: Ultralytics YOLO (nano, CPU) restricted to the `person` class, run
every few frames; an OpenCV tracker fills the frames in between.
Logic follows the approach published for the E88 Pro (Buff220 Human_Tracking.md):
ALIGN -> FORWARD -> ARRIVED, short-term PREDICT, then SEARCH or HOVER.
"""
from __future__ import annotations

import logging
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

from .control import Slew, clamp
from .link import Sticks

log = logging.getLogger("follow")


@dataclass
class FollowConfig:
    model: str = "yolo11n.pt"
    imgsz: int = 320            # small input keeps CPU inference fast
    conf: float = 0.4
    detect_every: int = 3       # run YOLO every N frames; track in between
    radius: float = 0.12        # follow circle, fraction of the shorter frame side
    release: float = 1.6        # FORWARD drops to ALIGN outside radius*release
    stop_height: float = 0.60   # box height (fraction of frame) meaning "close enough"
    kp_yaw: float = 0.9
    max_yaw: float = 0.35
    min_cmd: float = 0.10
    forward: float = 0.18       # pitch while approaching
    altitude: bool = True       # use throttle to keep the person's box vertically centred
    kp_thr: float = 0.5
    max_thr: float = 0.25
    confirm: int = 2            # frames needed to (re)acquire
    pred_window: float = 0.8
    pred_lead: float = 1.0
    pred_gain: float = 0.6
    pred_max_age: float = 1.5
    search: bool = True
    search_yaw: float = 0.10
    search_turn: float = 0.35
    search_pause: float = 1.0
    slew: float = 2.0


@dataclass
class Target:
    cx: float   # 0..1
    cy: float
    h: float    # box height / frame height
    box: tuple  # x, y, w, h in pixels


class PersonDetector:
    def __init__(self, cfg: FollowConfig):
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise SystemExit("Follow mode needs: pip install ultralytics") from e
        self.cfg = cfg
        self.model = YOLO(cfg.model)

    def __call__(self, img) -> list[tuple]:
        r = self.model.predict(img, imgsz=self.cfg.imgsz, conf=self.cfg.conf, classes=[0],
                               verbose=False, device="cpu")[0]
        out = []
        for b in r.boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = b[:4]
            out.append((int(x1), int(y1), int(x2 - x1), int(y2 - y1)))
        return out


def _make_tracker():
    for name in ("TrackerKCF_create", "TrackerCSRT_create", "TrackerMIL_create"):
        for mod in (cv2, getattr(cv2, "legacy", None)):
            if mod is not None and hasattr(mod, name):
                return getattr(mod, name)()
    return None


class Follower:
    def __init__(self, cfg: FollowConfig | None = None, detector=None):
        self.cfg = cfg or FollowConfig()
        self.detector = detector  # injectable for tests
        self.tracker = None
        self.frame_i = 0
        self.state = "LOST"
        self.sightings: deque = deque()
        self.seen_streak = 0
        self.last_seen_t = None
        self.search_dir = -1
        self.search_t0 = None
        self.slew_y, self.slew_p, self.slew_t = Slew(self.cfg.slew), Slew(self.cfg.slew), Slew(self.cfg.slew)
        self.target: Target | None = None
        self.predicted: tuple | None = None
        self.last_vx = 0.0   # remembered after sightings expire, for search direction
        self.last_x = 0.5

    # ---- perception ------------------------------------------------------
    def _perceive(self, img) -> Target | None:
        H, W = img.shape[:2]
        box = None
        run_det = self.frame_i % self.cfg.detect_every == 0 or self.tracker is None
        if run_det:
            if self.detector is None:
                self.detector = PersonDetector(self.cfg)
            boxes = self.detector(img)
            if boxes:
                box = max(boxes, key=lambda b: b[2] * b[3])   # largest = closest
                self.tracker = _make_tracker()
                if self.tracker is not None:
                    self.tracker.init(img, tuple(int(v) for v in box))
            else:
                self.tracker = None
        elif self.tracker is not None:
            ok, b = self.tracker.update(img)
            if ok:
                box = tuple(int(v) for v in b)
            else:
                self.tracker = None
        self.frame_i += 1
        if box is None:
            return None
        x, y, w, h = box
        return Target((x + w / 2) / W, (y + h / 2) / H, h / H, box)

    # ---- control ---------------------------------------------------------
    def update(self, img, t: float, dt: float, video_fps: float) -> Sticks:
        c = self.cfg
        H, W = img.shape[:2]
        tgt = self._perceive(img)
        self.target = tgt
        self.predicted = None
        aspect = min(W, H) / W   # radius is relative to the shorter side
        R = c.radius * aspect * 2  # in units where frame width = 2 (error is cx-0.5 -> *2)

        if tgt is not None:
            self.seen_streak += 1
            self.sightings.append((t, tgt.cx, tgt.cy, tgt.h))
            v = self._velocity()
            self.last_vx = v[0] if v else self.last_vx
            self.last_x = tgt.cx
        else:
            self.seen_streak = 0
        while self.sightings and t - self.sightings[0][0] > c.pred_window:
            self.sightings.popleft()

        # Slow video => gentler turns (full gain at >=15 fps).
        fps_gain = clamp(video_fps / 15.0, 1.0)
        fps_gain = max(fps_gain, 0.35)

        yaw = pitch = thr = 0.0
        if tgt is not None and (self.seen_streak >= c.confirm or self.state in ("ALIGN", "FORWARD", "ARRIVED")):
            self.last_seen_t = t
            self.search_t0 = None
            ex = (tgt.cx - 0.5) * 2          # -1..1, + = target right of centre
            ey = (tgt.cy - 0.5) * 2          # + = target below centre
            dist = abs(ex)
            if tgt.h >= c.stop_height:
                self.state = "ARRIVED"
            elif self.state == "FORWARD":
                if dist > R * c.release:
                    self.state = "ALIGN"
            else:
                self.state = "FORWARD" if dist <= R else "ALIGN"
            yaw = self._steer(ex, R, c.kp_yaw) * fps_gain
            if self.state == "FORWARD":
                pitch = c.forward * fps_gain
            if c.altitude:
                thr = clamp(-c.kp_thr * ey, c.max_thr) if abs(ey) > 0.15 else 0.0
        else:
            age = None if self.last_seen_t is None else t - self.last_seen_t
            pred = self._predict(t) if age is not None and age <= c.pred_max_age else None
            if pred is not None:
                self.state = "PREDICT"
                self.predicted = pred
                yaw = self._steer((pred[0] - 0.5) * 2, R, c.kp_yaw * c.pred_gain) * fps_gain
            elif c.search and self.last_seen_t is not None:
                self.state = "SEARCH"
                if self.search_t0 is None:
                    self.search_t0 = t
                    self.search_dir = self._last_direction()
                phase = (t - self.search_t0) % (c.search_turn + c.search_pause)
                yaw = self.search_dir * c.search_yaw if phase < c.search_turn else 0.0
            else:
                self.state = "LOST"

        return Sticks(roll=0.0,
                      pitch=self.slew_p(pitch, dt),
                      throttle=self.slew_t(thr, dt),
                      yaw=self.slew_y(clamp(yaw, c.max_yaw), dt))

    def _steer(self, ex: float, R: float, kp: float) -> float:
        if abs(ex) < 0.4 * R:
            return 0.0
        cmd = kp * ex
        if abs(cmd) < self.cfg.min_cmd:
            cmd = self.cfg.min_cmd * np.sign(ex)
        return clamp(cmd, self.cfg.max_yaw)

    def _velocity(self):
        if len(self.sightings) < 2:
            return None
        t0, x0, y0, _ = self.sightings[0]
        t1, x1, y1, _ = self.sightings[-1]
        if t1 - t0 < 0.15:
            return None
        return (x1 - x0) / (t1 - t0), (y1 - y0) / (t1 - t0), (t1, x1, y1)

    def _predict(self, t: float):
        v = self._velocity()
        if v is None:
            if self.sightings:
                _, x, y, _ = self.sightings[-1]
                return (x, y)
            return None
        vx, vy, (t1, x1, y1) = v
        lead = min(t - t1, self.cfg.pred_lead)
        return (clamp(x1 + vx * lead - 0.5, 0.5) + 0.5, clamp(y1 + vy * lead - 0.5, 0.5) + 0.5)

    def _last_direction(self) -> int:
        if abs(self.last_vx) >= 0.15:
            return 1 if self.last_vx > 0 else -1
        if abs(self.last_x - 0.5) > 0.1:
            return 1 if self.last_x > 0.5 else -1
        return -1

    def status(self) -> str:
        if self.target:
            return f"follow {self.state}  x {self.target.cx:.2f}  h {self.target.h:.2f}"
        return f"follow {self.state}"
