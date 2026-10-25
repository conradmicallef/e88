import math, cv2, numpy as np
from e88vision.pad import render_pad, A4_M
from e88vision.autoland import PadLocator, Autolander, AutolandConfig

PAD = render_pad(dpi=150)
PX_PER_M = PAD.shape[1] / A4_M[0]

def view(height, right=0.0, fwd=0.0, roll_deg=0.0, hfov=90, W=640, H=360):
    """Render what a down camera sees: camera at (right, fwd) metres from pad centre,
    `height` above it, image top = drone nose = pad 'NOSE' side, optional roll tilt."""
    f = (W / 2) / math.tan(math.radians(hfov) / 2)
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1]])
    # pad point (x right, y up/nose) -> camera frame (x right, y down, z down to floor)
    r = math.radians(roll_deg)
    Rroll = np.array([[math.cos(r), 0, math.sin(r)], [0, 1, 0], [-math.sin(r), 0, math.cos(r)]])
    def project(px, py):
        pw = np.array([px - right, -(py - fwd), height])
        pc = Rroll.T @ pw
        uv = K @ pc
        return uv[:2] / uv[2]
    h, w = PAD.shape
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    corners_m = [(-A4_M[0]/2, A4_M[1]/2), (A4_M[0]/2, A4_M[1]/2), (A4_M[0]/2, -A4_M[1]/2), (-A4_M[0]/2, -A4_M[1]/2)]
    dst = np.float32([project(*c) for c in corners_m])
    M = cv2.getPerspectiveTransform(src, dst)
    floor = np.full((H, W), 150, np.uint8)
    out = cv2.warpPerspective(PAD, M, (W, H), dst=floor, borderMode=cv2.BORDER_TRANSPARENT)
    return cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)

A = np.array([[-1.0, 0], [0, 1.0]])

def test_metric_height_and_offset():
    p = PadLocator(90).locate(view(1.0, right=0.10, fwd=-0.05), A)
    assert p is not None and len(p.markers) >= 3
    assert abs(p.height - 1.0) < 0.05, p.height
    # pad is to the drone's LEFT (drone is 0.10 right of it) and AHEAD
    assert abs(p.offset[0] - (-0.10)) < 0.02 and abs(p.offset[1] - 0.05) < 0.02, p.offset

def test_close_range_uses_centre_marker():
    p = PadLocator(90).locate(view(0.12), A)
    assert p is not None and 0 in p.markers and abs(p.height - 0.12) < 0.02

def test_tilt_does_not_fool_offset():
    p = PadLocator(90).locate(view(1.0, roll_deg=8), A)
    assert p is not None
    assert abs(p.tilt_deg[0]) > 5            # tilt is seen...
    assert np.linalg.norm(p.offset) < 0.03   # ...but the drone is still right over the pad

def test_lander_aligns_then_descends_then_lands():
    L = Autolander(AutolandConfig(slew=100))
    s, land = L.update(view(1.0, right=0.3), 0.0, 0.04)
    assert L.state == "ALIGN" and s.roll < 0 and s.throttle == 0 and not land
    s, land = L.update(view(1.0, right=0.02), 0.04, 0.04)
    assert L.state == "DESCEND" and s.throttle < 0
    s, land = L.update(view(0.2), 0.08, 0.04)
    assert L.state == "LANDED" and land

def test_lost_low_lands_lost_high_hovers():
    L = Autolander(AutolandConfig(slew=100))
    L.update(view(0.4), 0.0, 0.04)
    blank = np.full((360, 640, 3), 150, np.uint8)
    s, land = L.update(blank, 0.04, 0.04)
    assert land
    L2 = Autolander(AutolandConfig(slew=100))
    L2.update(view(1.5, right=0.5), 0.0, 0.04)
    s, land = L2.update(blank, 0.04, 0.04)
    assert not land and L2.state == "REACQUIRE" and s.throttle == 0
