import math, cv2, numpy as np
from e88vision.flow import FlowEstimator

def texture(h=360, w=640, seed=1):
    rng = np.random.default_rng(seed)
    img = rng.integers(0, 255, (h // 4, w // 4), dtype=np.uint8)
    img = cv2.resize(img, (w, h), interpolation=cv2.INTER_CUBIC)
    img = cv2.GaussianBlur(img, (5, 5), 0)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

def warp(img, dx_px, dy_px, rot_deg, scale):
    h, w = img.shape[:2]
    M = cv2.getRotationMatrix2D((w / 2, h / 2), -rot_deg, scale)  # OpenCV: + angle = CCW
    M[0, 2] += dx_px; M[1, 2] += dy_px
    return cv2.warpAffine(img, M, (w, h), borderMode=cv2.BORDER_REFLECT)

def run(dx, dy, rot, sc):
    base = texture()
    est = FlowEstimator()
    assert est.update(base) is None
    m = est.update(warp(base, dx, dy, rot, sc))
    return m

def test_translation():
    m = run(12, -8, 0, 1.0)
    assert m.ok and m.inliers > 50
    assert abs(m.dx - 12 / 640) < 0.002 and abs(m.dy - (-8 / 640)) < 0.002

def test_rotation_and_scale():
    m = run(0, 0, 3.0, 1.04)
    assert m.ok
    assert abs(math.degrees(m.rotation) - 3.0) < 0.3, math.degrees(m.rotation)
    assert abs(m.scale - 1.04) < 0.005

def test_featureless_floor_rejected():
    est = FlowEstimator()
    flat = np.full((360, 640, 3), 128, np.uint8)
    est.update(flat)
    m = est.update(flat)
    assert m is None or not m.ok
