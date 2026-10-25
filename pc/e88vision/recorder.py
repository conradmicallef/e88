"""Records every processed frame plus what the autopilot saw and commanded,
so flights can be replayed and controllers re-tuned offline."""
from __future__ import annotations

import json
import time
from pathlib import Path

import cv2


class Recorder:
    def __init__(self, folder: str | Path = "recordings"):
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.dir = Path(folder) / stamp
        self.dir.mkdir(parents=True, exist_ok=True)
        self.video_path = self.dir / "video.avi"
        self._writer = None
        self._log = open(self.dir / "log.jsonl", "w", encoding="utf-8")
        self.t0 = time.monotonic()

    def write(self, image, record: dict) -> None:
        if self._writer is None:
            h, w = image.shape[:2]
            self._writer = cv2.VideoWriter(str(self.video_path),
                                           cv2.VideoWriter_fourcc(*"MJPG"), 25.0, (w, h))
        self._writer.write(image)
        record = {"t": round(time.monotonic() - self.t0, 4), **record}
        self._log.write(json.dumps(record, default=_jsonable) + "\n")

    def event(self, name: str, **kw) -> None:
        self._log.write(json.dumps({"t": round(time.monotonic() - self.t0, 4), "event": name, **kw},
                                   default=_jsonable) + "\n")
        self._log.flush()

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
        self._log.close()


def _jsonable(o):
    if hasattr(o, "__dict__"):
        return {k: v for k, v in o.__dict__.items() if not k.startswith("_")}
    try:
        return float(o)
    except Exception:
        return str(o)
