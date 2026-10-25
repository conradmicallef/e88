"""UDP link to the drone: heartbeat, 25 Hz control loop, telemetry receiver."""
from __future__ import annotations

import logging
import socket
import threading
import time
from dataclasses import dataclass, field

from . import protocol as P

log = logging.getLogger("link")


@dataclass
class Sticks:
    roll: float = 0.0      # + right
    pitch: float = 0.0     # + forward
    throttle: float = 0.0  # + up (0 = hold altitude)
    yaw: float = 0.0       # + clockwise

    def clamp(self, lim: float) -> "Sticks":
        c = lambda v: max(-lim, min(lim, v))
        return Sticks(c(self.roll), c(self.pitch), c(self.throttle), c(self.yaw))


@dataclass
class LinkStats:
    tx: int = 0
    rx: int = 0
    last_rx: float = 0.0
    first_rx_hex: str = ""
    last_rx_hex: str = ""
    device_id: int | None = None
    resolution: str | None = None
    rx_kinds: dict = field(default_factory=dict)


class DroneLink:
    """Sends control packets every 40 ms. Sticks go neutral if not refreshed
    within `stale_s` (dead-man), so a crashed autopilot can't leave a stick held."""

    def __init__(self, host: str = P.DRONE_IP, port: int = P.CONTROL_PORT,
                 protocol: int | None = None, half_deflection: float = 127.0,
                 stale_s: float = 0.5, dry_run: bool = False, on_telemetry=None):
        self.addr = (host, port)
        self.forced_protocol = protocol
        self.protocol = protocol or P.LEGACY
        self.half = half_deflection
        self.stale_s = stale_s
        self.dry_run = dry_run
        self.on_telemetry = on_telemetry
        self.stats = LinkStats()
        self.headless = False
        self.last_packet = b""

        self._lock = threading.Lock()
        self._sticks = Sticks()
        self._sticks_t = 0.0
        self._pulses: dict[str, float] = {}
        self._lock_until = 0.0
        self._run = False
        self._sock: socket.socket | None = None
        self._threads: list[threading.Thread] = []

    # ---- lifecycle -------------------------------------------------------
    def start(self) -> "DroneLink":
        if not self.dry_run:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.settimeout(0.5)
        self._run = True
        self._raw(P.HEARTBEAT)  # also binds the socket so telemetry can come back
        for fn in (self._heartbeat_loop, self._control_loop, self._rx_loop):
            t = threading.Thread(target=fn, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def close(self) -> None:
        if not self._run:
            return
        self._raw(P.LEAVE_CONTROL)
        self._run = False
        for t in self._threads:
            t.join(timeout=1.0)
        if self._sock:
            self._sock.close()

    # ---- commands --------------------------------------------------------
    def set_sticks(self, s: Sticks) -> None:
        with self._lock:
            self._sticks, self._sticks_t = s, time.monotonic()

    def pulse(self, cmd: str, seconds: float = 1.0) -> None:
        with self._lock:
            self._pulses[cmd] = time.monotonic() + seconds

    def takeoff(self):
        self.pulse(P.TAKEOFF, 1.0)

    def land(self):
        self.pulse(P.LAND, 1.0)

    def emergency_stop(self):
        with self._lock:
            self._lock_until = time.monotonic() + 1.5
        self.pulse(P.STOP, 1.5)

    def calibrate_gyro(self):
        self.pulse(P.GYRO, 2.0)

    def camera(self, bottom: bool) -> None:
        self._raw(P.CAMERA_BOTTOM if bottom else P.CAMERA_FRONT)

    # ---- loops -----------------------------------------------------------
    def _raw(self, data: bytes) -> None:
        if self.dry_run or not self._sock:
            return
        try:
            self._sock.sendto(data, self.addr)
        except OSError as e:
            log.debug("send failed: %s", e)

    def build_packet(self, now: float | None = None) -> bytes:
        now = time.monotonic() if now is None else now
        with self._lock:
            s = self._sticks
            if now - self._sticks_t > self.stale_s or now < self._lock_until:
                s = Sticks()
            cmds = {c for c, until in self._pulses.items() if until > now}
        r = P.axis_byte(s.roll, self.half)
        p = P.axis_byte(s.pitch, self.half)
        t = P.axis_byte(s.throttle, 127.0)
        y = P.axis_byte(s.yaw, 127.0)
        if self.protocol == P.GL:
            return P.gl_packet(r, p, t, y, cmds, self.headless)
        return P.legacy_packet(r, p, t, y, cmds, self.headless)

    def _control_loop(self) -> None:
        nxt = time.monotonic()
        while self._run:
            pkt = self.build_packet()
            self.last_packet = pkt
            self._raw(pkt)
            self.stats.tx += 1
            nxt += 0.04
            time.sleep(max(0.0, nxt - time.monotonic()))

    def _heartbeat_loop(self) -> None:
        while self._run:
            time.sleep(1.0)
            self._raw(P.HEARTBEAT)

    def _rx_loop(self) -> None:
        while self._run:
            if self.dry_run or not self._sock:
                time.sleep(0.2)
                continue
            try:
                d, _ = self._sock.recvfrom(1024)
            except (socket.timeout, OSError):
                continue
            tel = P.parse_telemetry(d)
            if tel is None:
                continue
            st = self.stats
            st.rx += 1
            st.last_rx = time.monotonic()
            st.last_rx_hex = d.hex(" ")
            key = f"{len(d)}B:{d[0]:02x}"
            st.rx_kinds[key] = st.rx_kinds.get(key, 0) + 1
            if not st.first_rx_hex:
                st.first_rx_hex = st.last_rx_hex
                log.info("first telemetry packet: %s", st.first_rx_hex)
            if tel.device_id is not None:
                st.device_id, st.resolution = tel.device_id, tel.resolution
                if self.forced_protocol is None and tel.protocol and tel.protocol != self.protocol:
                    self.protocol = tel.protocol
                    log.info("protocol -> %s (device id %d)", "GL" if tel.protocol == P.GL else "legacy", tel.device_id)
            if tel.remote_button:
                self._raw(bytes([0x09, 1 if tel.remote_button == "photo" else 2]))
            if self.on_telemetry:
                self.on_telemetry(tel)
