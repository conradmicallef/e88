"""Vision position hold from the bottom camera.

The drone already holds attitude and (barometric) altitude itself. This loop
adds what it lacks: holding *position* and *heading* over the floor, which is
what wind and gyro drift take away.

Image -> drone mapping comes from calibration (see `calibrate.py`): a 2x2
matrix A so that [right, forward] = A @ [dx, dy] for ground image shift.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .control import PID, DelayLine, Slew, clamp
from .flow import FlowMeasurement
from .link import Sticks


@dataclass
class StabiliserConfig:
    # [right, forward] = A @ [dx, dy]; default assumes image top = drone nose.
    A: list = field(default_factory=lambda: [[-1.0, 0.0], [0.0, 1.0]])
    # Image shift caused by tilt alone, per unit stick (image widths). From calibration.
    tilt_shift: float = 0.0
    video_delay_s: float = 0.25
    kp: float = 2.5          # stick per image-width of position error
    ki: float = 0.3
    kd: float = 1.2          # stick per (image-width / s) of velocity
    max_tilt: float = 0.25   # stick limit for roll/pitch (≈ the official "slow" gear)
    yaw_kp: float = 0.8      # stick per rad of heading error
    max_yaw: float = 0.3
    alt_kp: float = 0.0      # throttle per log-scale of height error (0 = trust baro)
    max_throttle: float = 0.3
    slew: float = 1.5        # stick units per second
    lost_reset_s: float = 1.0


class Stabiliser:
    """Feed one FlowMeasurement per frame; get stick commands back."""

    def __init__(self, cfg: StabiliserConfig | None = None):
        self.cfg = cfg or StabiliserConfig()
        c = self.cfg
        self.A = np.array(c.A, dtype=float)
        self.pid_r = PID(c.kp, c.ki, c.kd, out_limit=c.max_tilt)
        self.pid_p = PID(c.kp, c.ki, c.kd, out_limit=c.max_tilt)
        self.slew_r, self.slew_p = Slew(c.slew), Slew(c.slew)
        self.cmd_delay = DelayLine(c.video_delay_s)
        self.reset()

    def reset(self):
        self.pos = np.zeros(2)       # drone displacement [right, fwd] in image widths
        self.vel = np.zeros(2)
        self.heading = 0.0           # rad accumulated
        self.log_height = 0.0        # +ve = camera moved closer (descended)
        self.prev_tilt_cmd = np.zeros(2)
        self.last_ok_t = None
        self.status = "waiting for flow"
        for x in (self.pid_r, self.pid_p):
            x.reset()
        self.slew_r.reset(); self.slew_p.reset()

    def update(self, m: FlowMeasurement | None, t: float, dt: float) -> Sticks:
        c = self.cfg
        if m is None or not m.ok:
            # No trustworthy flow: let the drone's own hold take over, keep the anchor briefly.
            if self.last_ok_t is not None and t - self.last_ok_t > c.lost_reset_s:
                self.reset()
                self.status = "lost floor texture - anchor reset"
            else:
                self.status = "flow weak - holding sticks neutral"
            self.vel[:] = 0
            r = self.slew_r(0.0, dt); p = self.slew_p(0.0, dt)
            return Sticks(roll=r, pitch=p)
        self.last_ok_t = t

        # 1) Ground shift -> drone displacement this frame.
        d = self.A @ np.array([m.dx, m.dy])

        # 2) Remove the part caused by the drone tilting (camera swings with the body).
        #    The tilt we see now was commanded `video_delay_s` ago.
        tilt_now = np.array(self.cmd_delay.at(t, default=(0.0, 0.0)))
        d -= c.tilt_shift * (tilt_now - self.prev_tilt_cmd)
        self.prev_tilt_cmd = tilt_now

        self.pos += d
        a = min(1.0, dt / 0.15)
        self.vel += a * (d / max(dt, 1e-3) - self.vel)
        self.heading += m.rotation
        self.log_height += math.log(max(m.scale, 1e-3))

        roll = self.pid_r.update(-self.pos[0], dt, measurement_rate=self.vel[0])
        pitch = self.pid_p.update(-self.pos[1], dt, measurement_rate=self.vel[1])
        roll, pitch = self.slew_r(roll, dt), self.slew_p(pitch, dt)
        self.cmd_delay.push(t, (roll, pitch))

        yaw = clamp(-c.yaw_kp * self.heading, c.max_yaw)
        throttle = clamp(c.alt_kp * self.log_height, c.max_throttle)

        self.status = (f"hold  off R{self.pos[0]*100:+.1f} F{self.pos[1]*100:+.1f} %w  "
                       f"v {np.linalg.norm(self.vel)*100:.1f} %w/s  hdg {math.degrees(self.heading):+.0f}°  "
                       f"pts {m.inliers}")
        return Sticks(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)
