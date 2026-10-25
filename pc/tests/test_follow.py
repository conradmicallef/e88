import numpy as np
from e88vision.follow import Follower, FollowConfig

class FakeDet:
    def __init__(self): self.box = None
    def __call__(self, img): return [self.box] if self.box else []

def mk(det_every=1):
    det = FakeDet()
    f = Follower(FollowConfig(detect_every=det_every, slew=100), detector=det)
    return f, det

IMG = np.zeros((360, 640, 3), np.uint8)

def step(f, t, fps=25):
    return f.update(IMG, t, 0.04, fps)

def test_turns_toward_person_right():
    f, det = mk(); det.box = (520, 100, 60, 150)
    for i in range(3): s = step(f, i * 0.04)
    assert f.state == "ALIGN" and s.yaw > 0 and s.pitch == 0

def test_moves_forward_when_centred_and_stops_when_close():
    f, det = mk(); det.box = (300, 120, 40, 120)
    for i in range(3): s = step(f, i * 0.04)
    assert f.state == "FORWARD" and s.pitch > 0 and s.yaw == 0
    det.box = (290, 20, 60, 300)   # 83% of frame height
    s = step(f, 0.2)
    assert f.state == "ARRIVED" and s.pitch == 0

def test_predict_then_search_after_loss():
    f, det = mk()
    for i in range(10):            # person walking right across frame
        det.box = (200 + i * 20, 100, 50, 150); step(f, i * 0.04)
    det.box = None
    s = step(f, 0.5)
    assert f.state == "PREDICT" and s.yaw > 0
    s = step(f, 0.4 + 2.0)
    assert f.state == "SEARCH" and f.search_dir == 1

def test_slow_video_scales_down_turn():
    f1, d1 = mk(); f2, d2 = mk()
    d1.box = d2.box = (560, 100, 50, 150)
    for i in range(3):
        a = step(f1, i * 0.04, fps=25); b = step(f2, i * 0.04, fps=5)
    assert abs(b.yaw) < abs(a.yaw)
