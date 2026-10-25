"""End-to-end: the real App.step drives a toy drone over a rendered pad."""
import numpy as np
from test_autoland import view
from e88vision.app import App, LAND, STABILISE

def toy_flight(app, mode, h0=1.2, x0=0.35, y0=-0.25, steps=600, wind=(0.0, 0.0)):
    app.set_mode(mode)
    pos = np.array([x0, y0]); vel = np.zeros(2); h = h0; t = 0.0; dt = 0.04
    landed_at = None
    for i in range(steps):
        img = view(h, right=pos[0], fwd=pos[1])
        s = app.step(img, t, dt)
        vel += (np.array([s.roll, s.pitch]) * 0.6 + np.array(wind) - vel) * 0.1
        pos += vel * dt
        h = max(0.05, h + s.throttle * 0.5 * dt)
        t += dt
        if app.land.state == "LANDED" and landed_at is None:
            landed_at = (t, pos.copy(), h)
            break
    return landed_at, pos, h

def test_autoland_end_to_end():
    app = App(dry_run=True, record=False, show=False)
    landed, pos, h = toy_flight(app, LAND)
    assert landed is not None, (app.land.state, pos, h)
    t, p, hh = landed
    assert np.linalg.norm(p) < 0.12 and hh < 0.3, (t, p, hh)
