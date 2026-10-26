"""Frame sources. All of them keep only the newest frame so the autopilot never
steers on a backlog of stale video."""
from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

log = logging.getLogger("video")

# FFmpeg options for the drone's RTSP (RTP/JPEG) stream. Each JPEG frame spans
# dozens of UDP packets, so losing or reordering one drops the whole frame:
#  - big socket buffer so a frame's burst of packets is never overflowed
#  - keep FFmpeg's normal reorder queue (setting it to 0 turns any reordered
#    packet into a "Missing packets; dropping frame")
#  - nobuffer/low_delay still avoid building a backlog of decoded frames
_BASE_OPTS = "fflags;nobuffer|flags;low_delay|buffer_size;4194304"
TRANSPORTS = ("udp", "tcp")


def rtsp_options(transport: str) -> str:
    return f"rtsp_transport;{transport}|{_BASE_OPTS}"


@dataclass
class Frame:
    image: np.ndarray
    t: float          # time.monotonic() when the frame arrived
    seq: int


class LatestFrameSource:
    """Background reader. `get()` returns the newest frame (or None).

    For RTSP, `transport="auto"` tries UDP first and falls back to TCP when a
    transport can't open, or opens but delivers no frames."""

    def __init__(self, url, realtime: bool = True, loop: bool = False, transport: str = "auto"):
        self.url = url
        self.realtime = realtime   # False for files: pace at file fps
        self.loop = loop
        self.transport = transport
        self._frame: Frame | None = None
        self._lock = threading.Lock()
        self._run = False
        self._seq = 0
        self._reopen = threading.Event()
        self._try = 0
        self._fails = 0   # consecutive opens that gave no frames
        self.fps = 0.0
        self.connected = False
        self.ended = False
        self.transport_in_use: str | None = None
        self.locked_transport: str | None = None   # first transport that delivered frames
        self.info = "opening video..."

    @property
    def is_rtsp(self) -> bool:
        return isinstance(self.url, str) and self.url.startswith("rtsp")

    def start(self) -> "LatestFrameSource":
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()
        return self

    def stop(self):
        self._run = False

    def reconnect(self):
        """Call after switching camera: the drone's RTSP session needs a restart."""
        self._reopen.set()

    def get(self) -> Frame | None:
        with self._lock:
            return self._frame

    def age(self) -> float:
        f = self.get()
        return float("inf") if f is None else time.monotonic() - f.t

    def _next_transport(self) -> str:
        if self.transport != "auto":
            return self.transport
        if self.locked_transport and self._fails < 2:
            return self.locked_transport
        t = TRANSPORTS[self._try % len(TRANSPORTS)]
        self._try += 1
        return t

    def _open(self):
        if isinstance(self.url, int):
            # DirectShow opens most webcams much faster than Windows' default backend.
            if os.name == "nt":
                cap = cv2.VideoCapture(self.url, cv2.CAP_DSHOW)
                if cap.isOpened():
                    return cap
                cap.release()
            return cv2.VideoCapture(self.url)
        if self.is_rtsp:
            tr = self._next_transport()
            self.transport_in_use = tr
            self.info = f"opening drone video over {tr.upper()}..."
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = rtsp_options(tr)
            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG,
                                   [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
                                    cv2.CAP_PROP_READ_TIMEOUT_MSEC, 3000])
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            return cap
        return cv2.VideoCapture(self.url)

    def _loop(self):
        times: list[float] = []
        while self._run:
            cap = self._open()
            if not cap.isOpened():
                self.connected = False
                self._fails += 1
                if self.is_rtsp:
                    self.info = f"{self.transport_in_use.upper()} would not open; retrying"
                    log.info("RTSP over %s did not open", self.transport_in_use)
                time.sleep(0.5)
                continue
            self.connected = True
            file_dt = 0.0
            if not self.realtime:
                fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
                file_dt = 1.0 / fps
            self._reopen.clear()
            opened_at = time.monotonic()
            got_any = False
            while self._run and not self._reopen.is_set():
                ok, img = cap.read()
                now = time.monotonic()
                if not ok:
                    break
                if not got_any and self.is_rtsp:
                    got_any = True
                    self._fails = 0
                    if self.locked_transport != self.transport_in_use:
                        self.locked_transport = self.transport_in_use
                        log.info("drone video working over %s", self.transport_in_use.upper())
                    self.info = f"video over {self.transport_in_use.upper()}"
                self._seq += 1
                with self._lock:
                    self._frame = Frame(img, now, self._seq)
                times.append(now)
                times = times[-30:]
                if len(times) > 1:
                    self.fps = (len(times) - 1) / (times[-1] - times[0] + 1e-9)
                if file_dt:
                    time.sleep(file_dt)
            if self.is_rtsp and not got_any and self._run and not self._reopen.is_set():
                self._fails += 1
                waited = time.monotonic() - opened_at
                log.info("RTSP over %s opened but gave no frames in %.1f s", self.transport_in_use, waited)
                self.info = f"{self.transport_in_use.upper()} gave no frames; trying the other transport"
            cap.release()
            self.connected = False
            if not self.realtime and not self._reopen.is_set():
                if self.loop:
                    continue
                self.ended = True
                return
            time.sleep(0.3)
