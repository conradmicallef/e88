"""Landing pad layout and printable A4 generator.

Layout (pad frame: x right, y up, metres from pad centre):
  id 0: 40 mm centre marker        (keeps lock in the last ~30 cm)
  id 1-4: 70 mm markers around it  (lock on from higher up)
"""
from __future__ import annotations

import cv2
import numpy as np

PAD_DICT = cv2.aruco.DICT_4X4_50

PAD_LAYOUT = {
    0: {"size_m": 0.040, "x_m": 0.000, "y_m": 0.000},
    1: {"size_m": 0.070, "x_m": -0.060, "y_m": 0.080},
    2: {"size_m": 0.070, "x_m": 0.060, "y_m": 0.080},
    3: {"size_m": 0.070, "x_m": -0.060, "y_m": -0.080},
    4: {"size_m": 0.070, "x_m": 0.060, "y_m": -0.080},
}

A4_M = (0.210, 0.297)


def render_pad(dpi: int = 300, layout=PAD_LAYOUT) -> np.ndarray:
    px_per_m = dpi / 0.0254
    W, H = int(round(A4_M[0] * px_per_m)), int(round(A4_M[1] * px_per_m))
    img = np.full((H, W), 255, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(PAD_DICT)
    cx, cy = W / 2, H / 2
    for mid, spec in layout.items():
        side = int(round(spec["size_m"] * px_per_m))
        m = cv2.aruco.generateImageMarker(d, mid, side)
        x0 = int(round(cx + spec["x_m"] * px_per_m - side / 2))
        y0 = int(round(cy - spec["y_m"] * px_per_m - side / 2))
        img[y0:y0 + side, x0:x0 + side] = m
    # Orientation hint + scale check line (100 mm) so a mis-scaled print is obvious.
    cv2.arrowedLine(img, (int(cx), int(H * 0.06)), (int(cx), int(H * 0.025)), 0, 6, tipLength=0.4)
    cv2.putText(img, "NOSE", (int(cx) + 25, int(H * 0.055)), cv2.FONT_HERSHEY_SIMPLEX, 1.6, 0, 3)
    L = int(round(0.100 * px_per_m)); y = int(H * 0.965)
    cv2.line(img, (int(cx - L / 2), y), (int(cx + L / 2), y), 0, 4)
    cv2.putText(img, "this line must measure 100 mm", (int(cx - L / 2), y - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
    return img


def save_pad(path_png: str = "landing_pad.png", path_pdf: str | None = "landing_pad.pdf", dpi: int = 300):
    img = render_pad(dpi)
    cv2.imwrite(path_png, img)
    if path_pdf:
        try:
            from PIL import Image
            Image.fromarray(img).save(path_pdf, "PDF", resolution=dpi)
        except ImportError:
            path_pdf = None
    return path_png, path_pdf
