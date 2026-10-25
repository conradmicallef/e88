"""E88 / RC-UFO (cooingdv) UDP protocol: packet builders and telemetry parsing.

Sources: CraxCurl/RC-Swamp working.md, Buff220/e88pro-reverse-RC-FPV-,
marshallrichards/turbodrone docs/research/cooingdv.md.
"""
from __future__ import annotations

from dataclasses import dataclass

DRONE_IP = "192.168.1.1"
CONTROL_PORT = 7099
RTSP_URL = f"rtsp://{DRONE_IP}:7070/webcam"

HEARTBEAT = bytes([0x01, 0x01])
LEAVE_CONTROL = bytes([0x08, 0x01])   # sent when leaving flight control, not at start
CAMERA_FRONT = bytes([0x06, 0x01])
CAMERA_BOTTOM = bytes([0x06, 0x02])

LEGACY, GL = 10, 2

# Momentary / persistent commands (protocol-neutral names)
TAKEOFF, LAND, STOP, FLIP, GYRO = "takeoff", "land", "stop", "flip", "gyro"

_GL_IDS = set(range(90, 102)) | {82, 85, 88, 103}
RESOLUTION_BY_ID = {
    **{i: "480p" for i in (5, 26, 63, 68, 85)},
    **{i: "720p" for i in (9, 21, 27, 41, 64, 69, 88, 90)},
    **{i: "1080p" for i in (12, 23, 29, 43, 65, 70, 91)},
    **{i: "4K" for i in (30, 44, 71, 92, 94)},
}


def axis_byte(v: float, half: float = 127.0, trim: int = 0) -> int:
    """-1..1 -> 1..255 with 128 centre. `half` = max deflection in raw units."""
    v = max(-1.0, min(1.0, float(v)))
    return max(1, min(255, int(round(128 + v * half)) + trim))


def legacy_packet(r: int, p: int, t: int, y: int, cmds: set[str], headless: bool) -> bytes:
    f = 0
    f |= 0x01 if TAKEOFF in cmds else 0
    f |= 0x02 if LAND in cmds else 0
    f |= 0x04 if STOP in cmds else 0
    f |= 0x08 if FLIP in cmds else 0
    f |= 0x10 if headless else 0
    f |= 0x80 if GYRO in cmds else 0
    cs = r ^ p ^ t ^ y ^ f
    return bytes([0x03, 0x66, r, p, t, y, f, cs & 0xFF, 0x99])


def gl_packet(r: int, p: int, t: int, y: int, cmds: set[str], headless: bool,
              altitude_hold: bool = True) -> bytes:
    f1 = 0
    f1 |= 0x01 if (TAKEOFF in cmds or LAND in cmds) else 0
    f1 |= 0x02 if STOP in cmds else 0
    f1 |= 0x04 if GYRO in cmds else 0
    f1 |= 0x08 if FLIP in cmds else 0
    f2 = (0x01 if headless else 0) | (0x02 if altitude_hold else 0)
    inner = bytearray(20)
    inner[0], inner[1] = 0x66, 0x14
    inner[2:6] = bytes([r, p, t, y])
    inner[6], inner[7] = f1, f2
    inner[18] = (r ^ p ^ t ^ y ^ f1 ^ f2) & 0xFF
    inner[19] = 0x99
    return b"\x03" + bytes(inner)


@dataclass
class Telemetry:
    raw: bytes
    device_id: int | None  # None for GL control-state frames (start with 0x66)
    protocol: int | None   # GL or LEGACY; None if this packet can't tell
    resolution: str | None
    remote_button: str | None   # "photo" / "video" / None


def parse_telemetry(d: bytes) -> Telemetry | None:
    if not d:
        return None
    dev = d[0]
    if dev == 0x66:  # GL control-state echo, not a device ID
        return Telemetry(d, None, None, None, None)
    btn = None
    if len(d) > 2 and d[2] == 0x4D:
        btn = "photo"
    elif len(d) > 2 and d[2] == 0x58:
        btn = "video"
    return Telemetry(d, dev, GL if dev in _GL_IDS else LEGACY, RESOLUTION_BY_ID.get(dev), btn)
