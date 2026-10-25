"""Frame-to-frame ground motion from the bottom camera.

Tracks corner features with pyramidal Lucas-Kanade, rejects outliers with a
forward-backward check, then fits a similarity transform (translation,
rotation, uniform scale) around the image centre with RANSAC.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class FlowMeasurement:
    dx: float          # image shift of the ground, in image widths (+ = right)
    dy: float          # image shift of the ground, in image widths (+ = down)
    rotation: float    # rad, + = ground turned clockwise in the image
    scale: float       # >1 = ground got bigger (camera moved closer)
    inliers: int
    tracked: int
    ok: bool


class FlowEstimator:
    def __init__(self, width: int = 320, max_corners: int = 200, min_inliers: int = 20,
                 fb_max_err: float = 1.0):
        self.width = width
        self.max_corners = max_corners
        self.min_inliers = min_inliers
        self.fb_max_err = fb_max_err
        self.prev_gray: np.ndarray | None = None
        self.prev_pts: np.ndarray | None = None
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.lk = dict(winSize=(21, 21), maxLevel=3,
                       criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))
        self.last_good_pts: np.ndarray | None = None  # for drawing

    def reset(self):
        self.prev_gray = None
        self.prev_pts = None

    def _prep(self, bgr: np.ndarray) -> np.ndarray:
        h, w = bgr.shape[:2]
        scale = self.width / w
        small = cv2.resize(bgr, (self.width, int(round(h * scale))), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY) if small.ndim == 3 else small
        return self.clahe.apply(gray)

    def _detect(self, gray: np.ndarray) -> np.ndarray | None:
        return cv2.goodFeaturesToTrack(gray, maxCorners=self.max_corners, qualityLevel=0.01,
                                       minDistance=8, blockSize=7)

    def update(self, bgr: np.ndarray) -> FlowMeasurement | None:
        gray = self._prep(bgr)
        if self.prev_gray is None or self.prev_pts is None or len(self.prev_pts) < self.min_inliers * 2:
            self.prev_gray, self.prev_pts = gray, self._detect(gray)
            return None

        p0 = self.prev_pts
        p1, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, p0, None, **self.lk)
        if p1 is None:
            self.reset()
            return None
        p0r, st2, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev_gray, p1, None, **self.lk)
        fb = np.linalg.norm((p0 - p0r).reshape(-1, 2), axis=1)
        good = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < self.fb_max_err)
        a = p0.reshape(-1, 2)[good]
        b = p1.reshape(-1, 2)[good]

        h, w = gray.shape
        centre = np.array([w / 2.0, h / 2.0], dtype=np.float32)
        meas = None
        if len(a) >= self.min_inliers:
            M, inl = cv2.estimateAffinePartial2D(a - centre, b - centre, method=cv2.RANSAC,
                                                 ransacReprojThreshold=1.5, maxIters=500,
                                                 confidence=0.99)
            n_in = int(inl.sum()) if inl is not None else 0
            if M is not None and n_in >= self.min_inliers:
                s = math.hypot(M[0, 0], M[1, 0])
                rot = math.atan2(M[1, 0], M[0, 0])
                meas = FlowMeasurement(dx=float(M[0, 2]) / w, dy=float(M[1, 2]) / w,
                                       rotation=rot, scale=s, inliers=n_in, tracked=len(a), ok=True)
                keep = inl.ravel().astype(bool)
                b = b[keep]
        if meas is None:
            meas = FlowMeasurement(0, 0, 0, 1, 0, len(a), ok=False)

        # Re-seed features when they thin out; otherwise keep tracking survivors.
        self.last_good_pts = b
        if len(b) < self.max_corners // 2:
            fresh = self._detect(gray)
            self.prev_pts = fresh if fresh is not None else b.reshape(-1, 1, 2)
        else:
            self.prev_pts = b.reshape(-1, 1, 2).astype(np.float32)
        self.prev_gray = gray
        return meas
