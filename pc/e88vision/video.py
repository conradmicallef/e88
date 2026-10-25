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

# Low-latency FFmpeg options for OpenCV's RTSP reader (must be set before opening).
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    "rtsp_transport;udp|fflags;nobuffer|flags;low_delay|max_delay;0|reorder_queue_size;0",
)


@dataclass
class Frame:
    image: np.ndarray
    t: float          # time.monotonic() when the frame arrived
    seq: int


class LatestFrameSource:
    """Background reader. `get()` returns the newest frame (or None)."""

    def __init__(self, url, realtime: bool = True, loop: bool = False):
        self.url = url
        self.realtime = realtime   # False for files: pace at file fps
        self.loop = loop
        self._frame: Frame | None = None
        self._lock = threading.Lock()
        self._run = False
        self._seq = 0
        self._reopen = threading.Event()
        self.fps = 0.0
        self.connected = False
        self.ended = False

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

    def _open(self):
        if isinstance(self.url, int):
            cap = cv2.VideoCapture(self.url)
        elif str(self.url).startswith("rtsp"):
            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            cap = cv2.VideoCapture(self.url)
        return cap

    def _loop(self):
        times: list[float] = []
        while self._run:
            cap = self._open()
            if not cap.isOpened():
                self.connected = False
                time.sleep(0.5)
                continue
            self.connected = True
            file_dt = 0.0
            if not self.realtime:
                fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
                file_dt = 1.0 / fps
            self._reopen.clear()
            while self._run and not self._reopen.is_set():
                ok, img = cap.read()
                if not ok:
                    break
                now = time.monotonic()
                self._seq += 1
                with self._lock:
                    self._frame = Frame(img, now, self._seq)
                times.append(now)
                times = times[-30:]
                if len(times) > 1:
                    self.fps = (len(times) - 1) / (times[-1] - times[0] + 1e-9)
                if file_dt:
                    time.sleep(file_dt)
            cap.release()
            self.connected = False
            if not self.realtime and not self._reopen.is_set():
                if self.loop:
                    continue
                self.ended = True
                return
            time.sleep(0.3)
