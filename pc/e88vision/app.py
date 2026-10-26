"""Main flight loop: video in -> chosen vision autopilot -> sticks out, with an
overlay window and a recorder."""
from __future__ import annotations

import json
import logging
import time
from pathlib import Path

import cv2
import numpy as np

from . import protocol as P
from .autoland import Autolander, AutolandConfig
from .calibrate import Calibrator
from .flow import FlowEstimator
from .follow import Follower, FollowConfig
from .link import DroneLink, Sticks
from .recorder import Recorder
from .stabilize import Stabiliser, StabiliserConfig
from .video import LatestFrameSource

log = logging.getLogger("app")

HOVER, STABILISE, FOLLOW, LAND, CALIBRATE = "HOVER", "STABILISE", "FOLLOW", "AUTOLAND", "CALIBRATE"
NEEDS_BOTTOM = {STABILISE, LAND, CALIBRATE}
FRAME_STALE_S = 0.4

HELP = ("T takeoff  L land  SPACE stop motors  0 hover  1 stabilise  2 follow  3 autoland  "
        "K calibrate mount  G gyro-cal (on ground)  C switch camera  Q quit")


def load_calibration(path="calibration.json"):
    p = Path(path)
    if p.exists():
        d = json.loads(p.read_text())
        log.info("loaded camera mount calibration %s (ok=%s)", d.get("A"), d.get("ok"))
        return d
    return None


class App:
    def __init__(self, source=P.RTSP_URL, dry_run=False, record=True, protocol=None,
                 hfov=90.0, start_mode=HOVER, realtime=True, follow_model="yolo11n.pt",
                 show=True, transport="auto"):
        self.dry_run = dry_run
        self.show = show
        cal = load_calibration()
        A = cal["A"] if cal else [[-1.0, 0.0], [0.0, 1.0]]
        self.link = DroneLink(protocol=protocol, dry_run=dry_run)
        self.src = LatestFrameSource(source, realtime=realtime, transport=transport)
        self.flow = FlowEstimator()
        self.stab = Stabiliser(StabiliserConfig(A=A))
        self.follow = Follower(FollowConfig(model=follow_model))
        self.land = Autolander(AutolandConfig(A=tuple(map(tuple, A)), hfov_deg=hfov))
        self.cal: Calibrator | None = None
        self.rec = Recorder() if record else None
        self.mode = HOVER
        self.bottom = False
        self.status = ""
        self.sticks = Sticks()
        self.pending_mode = start_mode
        self._flow_meas = None

    # ---- mode handling ---------------------------------------------------
    def set_mode(self, mode: str):
        if mode == self.mode:
            return
        want_bottom = mode in NEEDS_BOTTOM if mode != HOVER else self.bottom
        if want_bottom != self.bottom:
            self.switch_camera(want_bottom)
        self.mode = mode
        self.flow.reset(); self.stab.reset(); self.land.reset()
        if mode == CALIBRATE:
            self.cal = Calibrator()
        if self.rec:
            self.rec.event("mode", mode=mode)
        log.info("mode -> %s", mode)

    def switch_camera(self, bottom: bool):
        self.bottom = bottom
        self.link.camera(bottom)
        if not self.dry_run:
            self.src.reconnect()

    # ---- main loop -------------------------------------------------------
    def run(self):
        self.link.start()
        self.src.start()
        self.set_mode(self.pending_mode)
        last_seq, last_t = -1, time.monotonic()
        print(HELP)
        try:
            while True:
                fr = self.src.get()
                now = time.monotonic()
                if fr is None or fr.seq == last_seq:
                    if self.src.age() > FRAME_STALE_S:
                        self.link.set_sticks(Sticks())
                        self.status = "NO VIDEO - sticks neutral (drone holds itself)"
                        if self.show:
                            self._draw_waiting()
                    if self.src.ended:
                        break
                    if not self._keys(5):
                        break
                    continue
                dt = max(1e-3, min(0.2, now - last_t))
                last_seq, last_t = fr.seq, now
                self.step(fr.image, now, dt)
                if self.show and not self._keys(1, fr.image):
                    break
        finally:
            self.link.close()
            self.src.stop()
            if self.rec:
                self.rec.close()
                print(f"recording saved to {self.rec.dir}")
            if self.show:
                cv2.destroyAllWindows()

    def step(self, img: np.ndarray, t: float, dt: float) -> Sticks:
        s = Sticks()
        m = None
        if self.mode in (STABILISE, LAND, CALIBRATE):
            m = self.flow.update(img)
            self._flow_meas = m
        if self.mode == STABILISE:
            s = self.stab.update(m, t, dt)
            self.status = self.stab.status
        elif self.mode == CALIBRATE and self.cal:
            s = self.cal.update(m, t)
            self.status = self.cal.status
            if self.cal.done:
                self.cal.save()
                self.status = f"{self.cal.status} -> calibration.json saved; restart to use it"
                self.stab = Stabiliser(StabiliserConfig(A=self.cal.result["A"]))
                self.mode = HOVER
        elif self.mode == FOLLOW:
            s = self.follow.update(img, t, dt, self.src.fps)
            self.status = self.follow.status()
        elif self.mode == LAND:
            fallback = self.stab.update(m, t, dt)
            s, send_land = self.land.update(img, t, dt, fallback=fallback)
            if send_land:
                self.link.land()
                if self.rec:
                    self.rec.event("autoland_land_command")
            self.status = self.land.status()
        else:
            self.status = "HOVER - drone's own altitude hold, no vision"
        self.sticks = s
        self.link.set_sticks(s)
        if self.rec:
            self.rec.write(img, {"mode": self.mode, "bottom": self.bottom, "sticks": s.__dict__,
                                 "status": self.status, "fps": round(self.src.fps, 1),
                                 "flow": None if m is None else m.__dict__,
                                 "tx": self.link.stats.tx, "rx": self.link.stats.rx})
        return s

    # ---- UI --------------------------------------------------------------
    def _keys(self, wait_ms: int, img=None) -> bool:
        if not self.show:
            return True
        if img is not None:
            cv2.imshow("E88 vision", self._overlay(img))
        k = cv2.waitKey(wait_ms) & 0xFF
        if k in (ord("q"), 27):
            return False
        if k == ord(" "):
            self.link.emergency_stop(); self.set_mode(HOVER)
        elif k == ord("t"):
            self.link.takeoff()
        elif k == ord("l"):
            self.link.land(); self.set_mode(HOVER)
        elif k == ord("0"):
            self.set_mode(HOVER)
        elif k == ord("1"):
            self.set_mode(STABILISE)
        elif k == ord("2"):
            self.set_mode(FOLLOW)
        elif k == ord("3"):
            self.set_mode(LAND)
        elif k == ord("k"):
            self.set_mode(CALIBRATE)
        elif k == ord("g"):
            self.link.calibrate_gyro()
        elif k == ord("c"):
            self.switch_camera(not self.bottom)
        return True

    def _draw_waiting(self):
        img = np.zeros((360, 640, 3), np.uint8)
        cv2.putText(img, "waiting for video...", (180, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        cv2.putText(img, self.src.info[:70], (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 210, 255), 1)
        self._keys(1, img)

    def _overlay(self, img: np.ndarray) -> np.ndarray:
        o = img.copy()
        h, w = o.shape[:2]
        green, yellow, red, white = (80, 220, 80), (0, 210, 255), (60, 60, 255), (240, 240, 240)
        # mode-specific drawing
        if self.mode in (STABILISE, CALIBRATE, LAND) and self.flow.last_good_pts is not None:
            sx = w / self.flow.width
            for x, y in self.flow.last_good_pts[:150]:
                cv2.circle(o, (int(x * sx), int(y * sx)), 2, green, -1)
        if self.mode == FOLLOW:
            if self.follow.target:
                x, y, bw, bh = self.follow.target.box
                cv2.rectangle(o, (x, y), (x + bw, y + bh), green, 2)
            if self.follow.predicted:
                px, py = self.follow.predicted
                cv2.circle(o, (int(px * w), int(py * h)), 10, yellow, 2)
            R = int(self.follow.cfg.radius * min(w, h))
            cv2.circle(o, (w // 2, h // 2), R, white, 1)
        if self.mode == LAND and self.land.pose:
            cv2.aruco.drawDetectedMarkers(o, self.land.pose.corners)
        cv2.drawMarker(o, (w // 2, h // 2), white, cv2.MARKER_CROSS, 16, 1)
        # header
        cv2.rectangle(o, (0, 0), (w, 44), (0, 0, 0), -1)
        st = self.link.stats
        link_ok = self.dry_run or (st.rx > 0 and time.monotonic() - st.last_rx < 3)
        proto = "GL" if self.link.protocol == P.GL else "legacy"
        cam = "DOWN" if self.bottom else "FRONT"
        head = (f"{self.mode}  cam {cam}  {self.src.fps:4.1f} fps  "
                f"{'DRY RUN' if self.dry_run else ('link OK' if link_ok else 'no telemetry')}  {proto}"
                f"{'  id ' + str(st.device_id) if st.device_id is not None else ''}")
        cv2.putText(o, head, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, green if link_ok else yellow, 1)
        cv2.putText(o, self.status[:110], (8, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.45, white, 1)
        # stick bars
        s = self.sticks
        for i, (name, v) in enumerate((("R", s.roll), ("P", s.pitch), ("T", s.throttle), ("Y", s.yaw))):
            y0 = h - 70 + i * 16
            cv2.putText(o, name, (8, y0 + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, white, 1)
            cv2.rectangle(o, (24, y0), (124, y0 + 10), (90, 90, 90), 1)
            cv2.rectangle(o, (74, y0 + 1), (74 + int(v * 50), y0 + 9), yellow, -1)
        cv2.putText(o, "SPACE = stop motors   Q = quit", (w - 260, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, red, 1)
        return o
