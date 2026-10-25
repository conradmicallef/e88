"""Vision autoland on a printed ArUco pad, using the down camera.

The pad has four 70 mm markers around a 40 mm centre marker (see `pad.py`):
the big ones lock on from ~0.5-3 m, the small one keeps the lock in the last
few tens of centimetres. Every visible marker gives a full pose, so we get
*metric* height, horizontal offset, and the drone's tilt relative to the pad.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from .control import Slew, clamp
from .link import Sticks
from .pad import PAD_DICT, PAD_LAYOUT

# Local axis conventions:
#   camera frame: x right, y down (image), z out of the lens (towards the floor)
#   drone frame offsets: [right, forward] metres from drone to pad centre


@dataclass
class AutolandConfig:
    hfov_deg: float = 90.0       # down camera horizontal field of view (affects height scale)
    A: tuple = ((-1.0, 0.0), (0.0, 1.0))  # mount mapping from calibration.json (same as stabiliser)
    kp: float = 0.9              # stick per metre
    kd: float = 0.5              # stick per m/s
    max_tilt: float = 0.2
    descend: float = 0.30        # throttle stick while descending
    align_tol_m: float = 0.08    # must be this close (or 15% of height) to descend
    final_height_m: float = 0.25 # below this and centred -> send LAND
    lost_low_m: float = 0.5      # lose the pad below this -> LAND anyway
    lost_abort_s: float = 3.0    # lose it higher for this long -> give up (hover)
    slew: float = 1.5


@dataclass
class PadPose:
    height: float            # metres, camera above pad
    offset: np.ndarray       # [right, forward] metres, tilt-corrected
    tilt_deg: tuple          # (roll, pitch) of camera relative to the pad
    markers: list            # ids seen
    corners: list            # for drawing


def _detector():
    d = cv2.aruco.getPredefinedDictionary(PAD_DICT)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(d, params)


def _rot_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Smallest rotation taking unit vector a onto unit vector b."""
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b)
    v = np.cross(a, b); c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


class PadLocator:
    def __init__(self, hfov_deg: float = 90.0):
        self.hfov = math.radians(hfov_deg)
        self.det = _detector()
        self._K_for = None

    def _K(self, w: int, h: int) -> np.ndarray:
        if self._K_for != (w, h):
            f = (w / 2) / math.tan(self.hfov / 2)
            self.K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], dtype=np.float64)
            self._K_for = (w, h)
        return self.K

    def locate(self, img: np.ndarray, A: np.ndarray) -> PadPose | None:
        h, w = img.shape[:2]
        K = self._K(w, h)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        corners, ids, _ = self.det.detectMarkers(gray)
        if ids is None:
            return None
        # One pose for the whole pad from every visible corner: far steadier than
        # per-marker poses, which flip between two mirror solutions when seen head-on.
        obj, imgp, seen = [], [], []
        for c, i in zip(corners, ids.ravel()):
            spec = PAD_LAYOUT.get(int(i))
            if spec is None:
                continue
            s, x, y = spec["size_m"] / 2, spec["x_m"], spec["y_m"]
            obj += [[x - s, y + s, 0], [x + s, y + s, 0], [x + s, y - s, 0], [x - s, y - s, 0]]
            imgp += c.reshape(4, 2).tolist()
            seen.append(int(i))
        if not seen:
            return None
        obj = np.array(obj, dtype=np.float64)
        imgp = np.array(imgp, dtype=np.float64)
        if len(seen) == 1:
            ok, rvec, tvec = cv2.solvePnP(obj, imgp, K, None, flags=cv2.SOLVEPNP_IPPE)
        else:
            ok, rvec, tvec = cv2.solvePnP(obj, imgp, K, None, flags=cv2.SOLVEPNP_SQPNP)
            if ok:
                ok, rvec, tvec = cv2.solvePnP(obj, imgp, K, None, rvec, tvec, True,
                                              flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        p = tvec.ravel()      # pad centre in camera frame
        n = R[:, 2]           # pad normal in camera frame
        # Level the camera: rotate so the pad's normal points straight back up the lens.
        # (marker z points out of the marker, i.e. towards the camera, so -n is "down")
        Rl = _rot_between(-n, np.array([0.0, 0.0, 1.0]))
        pl = Rl @ p
        tilt_roll = math.degrees(math.atan2(n[0], -n[2]))
        tilt_pitch = math.degrees(math.atan2(-n[1], -n[2]))
        offset = -(np.asarray(A) @ pl[:2])   # image right/down -> drone right/forward
        return PadPose(height=float(pl[2]), offset=offset, tilt_deg=(tilt_roll, tilt_pitch),
                       markers=seen, corners=corners)


class Autolander:
    def __init__(self, cfg: AutolandConfig | None = None):
        self.cfg = cfg or AutolandConfig()
        self.A = np.array(self.cfg.A, dtype=float)
        self.loc = PadLocator(self.cfg.hfov_deg)
        self.slew_r, self.slew_p, self.slew_t = (Slew(self.cfg.slew) for _ in range(3))
        self.reset()

    def reset(self):
        self.state = "SEARCH"
        self.pose: PadPose | None = None
        self.prev_off = None
        self.vel = np.zeros(2)
        self.last_seen_t = None
        self.last_height = None
        self.land_sent = False

    def update(self, img, t: float, dt: float, fallback: Sticks | None = None) -> tuple[Sticks, bool]:
        """Returns (sticks, send_land_now)."""
        c = self.cfg
        if self.state == "LANDED":
            return Sticks(), False
        pose = self.loc.locate(img, self.A)
        self.pose = pose
        if pose is None:
            age = None if self.last_seen_t is None else t - self.last_seen_t
            if self.last_height is not None and self.last_height < c.lost_low_m and age is not None:
                return self._land()
            if age is not None and age > c.lost_abort_s:
                self.state = "LOST"
            elif self.last_seen_t is not None:
                self.state = "REACQUIRE"
            fb = fallback or Sticks()
            return Sticks(self.slew_r(fb.roll, dt), self.slew_p(fb.pitch, dt),
                          self.slew_t(0.0, dt), fb.yaw), False

        self.last_seen_t = t
        self.last_height = pose.height
        off = pose.offset
        if self.prev_off is not None:
            a = min(1.0, dt / 0.15)
            self.vel += a * ((off - self.prev_off) / max(dt, 1e-3) - self.vel)
        self.prev_off = off.copy()

        roll = clamp(c.kp * off[0] - c.kd * self.vel[0], c.max_tilt)
        pitch = clamp(c.kp * off[1] - c.kd * self.vel[1], c.max_tilt)
        err = float(np.linalg.norm(off))
        tol = max(c.align_tol_m, 0.15 * pose.height)
        if pose.height <= c.final_height_m and err <= tol:
            return self._land()
        if err <= tol:
            self.state = "DESCEND"
            thr = -c.descend
        else:
            self.state = "ALIGN"
            thr = 0.0
        return Sticks(self.slew_r(roll, dt), self.slew_p(pitch, dt), self.slew_t(thr, dt), 0.0), False

    def _land(self):
        self.state = "LANDED"
        send = not self.land_sent
        self.land_sent = True
        return Sticks(), send

    def status(self) -> str:
        p = self.pose
        if p is None:
            return f"land {self.state}  (pad not visible)"
        return (f"land {self.state}  h {p.height:.2f} m  off R{p.offset[0]:+.2f} F{p.offset[1]:+.2f} m  "
                f"tilt {p.tilt_deg[0]:+.0f}/{p.tilt_deg[1]:+.0f}°  ids {p.markers}")
