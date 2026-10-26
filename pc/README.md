# e88vision: vision autopilots for the E88 on a Windows PC

Three autopilots that use the drone's two cameras, running on your PC's CPU:

| Key | Mode | Camera | What it does |
|---|---|---|---|
| `1` | **Stabilise** | down | Optical flow over the floor holds position and heading against drift and wind |
| `2` | **Follow** | front | YOLO person detector + tracker. It yaws to keep you centred, approaches, and stops when close. It predicts briefly when you're lost, then searches. |
| `3` | **Autoland** | down | Finds the printed ArUco pad and measures real height, offset and tilt. It centres, descends, then sends LAND. |
| `0` | Hover | either | No vision; the drone's own altitude hold |

Other keys: `T` take off · `L` land · **`SPACE` stop motors** · `K` calibrate camera mount · `G` gyro-calibrate (on the ground) · `C` switch camera · `Q` quit.

Every flight is recorded (`recordings/<time>/video_001.avi`, a new file at each camera switch, + `log.jsonl`), so you can re-run any autopilot on it offline with `replay`.

## Setup (once)

1. Install **Python 3.11+** from python.org, ticking "Add to PATH".
2. In this folder, run:
   ```
   py -m pip install -r requirements.txt
   ```
3. Power the drone and join its Wi-Fi from Windows. The PC should get a `192.168.1.x` address.
4. The first time Python runs, **allow it through Windows Firewall on Public networks**. Drone networks usually show as Public, and the drone's replies are UDP. If nothing comes back, this is the usual cause.

## First flights, in this order

| Step | Command | Why |
|---|---|---|
| 1 | `py -m e88vision probe` | Logs everything the drone sends back for 15 s, without flying. Paste the summary to Claude: it tells us GL vs legacy, camera resolution, and anything new. |
| 2 | `py -m e88vision latency` | Point the front camera at the window's clock. Delay = left clock − clock in the video. Snap a phone photo to read both at once. |
| 3 | `py -m e88vision pad` | Writes `landing_pad.pdf`. Print it at **100% / actual size** on A4, check the 100 mm line, and tape it flat with NOSE towards the drone's front. |
| 4 | `py -m e88vision fly` → `T`, hover ~1 m over a rug, press `K` | Calibrates which way the down camera faces. The drone makes four small nudges, and the result is saved to `calibration.json`. Restart afterwards. |
| 5 | `py -m e88vision fly` → `T` → `1` | Stabilise. Push the drone gently by hand or walk a fan past it: it should drift back. |
| 6 | `py -m e88vision fly` → `T` → `3` | Autoland, starting about 1 m above the pad. |
| 7 | `py -m e88vision fly` → `T` → `2` | Follow. Stand 2–3 m in front of the drone. |

Use the first flights to test signs and behaviour **with the propellers off**, or in a big empty room. `SPACE` cuts the motors and the drone will drop.

## Testing without the drone

- `py -m e88vision fly --dry-run --source 0 --mode follow` runs follow on your **laptop webcam**, sending nothing.
- `py -m e88vision replay recordings\20261025-101500 --mode stabilise` re-runs an autopilot on a recorded flight.
- `py -m pytest tests` runs the test suite (link, flow, stabiliser, calibration, follow logic, pad pose, a closed-loop autoland sim).

## How it works and what to tune

- **Safety layers:**
  - Video frozen for more than 0.4 s → sticks neutral.
  - The link's dead-man → sticks neutral if the autopilot stops updating.
  - Weak floor texture → neutral and re-anchor.
  - Pad lost below 0.5 m → land; lost higher → hold position on optical flow.
  - Autopilot stick limits sit around the official "slow" gear.
- **Stabilise** (`stabilize.py`, `StabiliserConfig`):
  - `kp`/`kd` are stick per image-width of offset and velocity.
  - Raise `kd` if it overshoots, raise `kp` if it drifts back too slowly.
  - Image motion is relative to height: at 2 m the same drift looks half as big, so start at about 1 m.
- **Tilt.** When the drone leans, its camera swings, which looks like motion. `tilt_shift` subtracts that. It's 0 by default; raise it (0.02–0.1) if the stabiliser twitches back and forth.
- **Follow** (`follow.py`, `FollowConfig`):
  - `kp_yaw` and `max_yaw` set how hard it turns.
  - `stop_height` is how close it comes.
  - `detect_every` trades CPU for responsiveness. YOLO runs every 3rd frame at 320 px; a tracker fills the gaps.
- **Autoland** (`autoland.py`, `AutolandConfig`):
  - `--hfov` is the down camera's field of view. It only scales the height estimate; centring works either way.
  - `descend`, `align_tol_m` and `final_height_m` shape the descent.
- **Protocol** (`protocol.py`): RC-UFO family at `192.168.1.1`, UDP 7099, video `rtsp://192.168.1.1:7070/webcam`. GL or legacy control is chosen automatically from the drone's first status packet, or forced with `--protocol gl|legacy`.
