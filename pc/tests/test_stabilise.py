import numpy as np
from e88vision.flow import FlowMeasurement
from e88vision.stabilize import Stabiliser, StabiliserConfig
from e88vision.calibrate import Calibrator

def fm(dx=0, dy=0, rot=0, sc=1.0):
    return FlowMeasurement(dx, dy, rot, sc, 100, 120, True)

def sim(stab, steps=400, wind=(0.002, -0.001), dt=0.04, A=np.array([[-1, 0], [0, 1.0]]), lag=0):
    """Toy plant: drone velocity follows stick (first order), wind pushes it;
    camera sees ground shift = A^-1 @ displacement."""
    vel = np.zeros(2); pos = np.zeros(2); Ainv = np.linalg.inv(A); t = 0
    seen = [np.zeros(2)] * (lag + 1)   # video arrives `lag` frames late
    for _ in range(steps):
        seen.append(Ainv @ (vel * dt))
        s = stab.update(fm(*seen.pop(0)), t, dt)
        cmd = np.array([s.roll, s.pitch])
        vel += (cmd * 0.15 + np.array(wind) - vel) * min(1, dt / 0.4)
        pos += vel * dt; t += dt
    return pos, s

def test_holds_against_wind():
    stab = Stabiliser(StabiliserConfig(video_delay_s=0.0, slew=10))
    pos_open, _ = sim(Stabiliser(StabiliserConfig(kp=0, ki=0, kd=0)), wind=(0.02, -0.01))
    pos, s = sim(stab, wind=(0.02, -0.01))
    assert np.linalg.norm(pos) < 0.2 * np.linalg.norm(pos_open), (pos, pos_open)
    assert s.roll < 0 and s.pitch > 0   # leaning into the wind

def test_lost_flow_goes_neutral():
    stab = Stabiliser()
    s = stab.update(None, 0.0, 0.04)
    assert s.roll == 0 and s.pitch == 0

def test_heading_hold_sign():
    stab = Stabiliser()
    s = stab.update(fm(rot=0.2), 0, 0.04)
    assert s.yaw < 0

def test_calibration_recovers_mount():
    # Camera mounted rotated 90°: moving right shows ground moving up (-dy), forward shows +dx... etc.
    A_true = np.array([[0, -1.0], [1.0, 0]])
    Ainv = np.linalg.inv(A_true)
    cal = Calibrator(); t = 0.0; vel = np.zeros(2)
    while not cal.done:
        disp = vel * 0.04
        s = cal.update(fm(*(Ainv @ disp)), t)
        vel += (np.array([s.roll, s.pitch]) * 0.5 - vel) * 0.2
        t += 0.04
    A = np.array(cal.result["A"])
    assert cal.result["ok"], cal.result
    assert np.allclose(A, A_true, atol=0.15), A

def test_stable_with_video_lag():
    # 0.25 s of video delay (6 frames): must still converge, not oscillate.
    pos, _ = sim(Stabiliser(), steps=750, wind=(0.02, -0.01), lag=6)
    pos_open, _ = sim(Stabiliser(StabiliserConfig(kp=0, ki=0, kd=0)), steps=750, wind=(0.02, -0.01), lag=6)
    assert np.linalg.norm(pos) < 0.3 * np.linalg.norm(pos_open), (pos, pos_open)
