"""python -m e88vision <command>

  fly      fly with vision autopilots (press 1/2/3 in the window)
  probe    connect without flying and log everything the drone sends back
  latency  measure video delay (point the front camera at the screen)
  pad      write the printable landing pad (landing_pad.pdf / .png)
  replay   run an autopilot offline on a recorded flight
"""
from __future__ import annotations

import argparse
import logging
import sys
import time

from . import protocol as P


def _source(s: str):
    return int(s) if s.isdigit() else s


def cmd_fly(a):
    from .app import App, HOVER, STABILISE, FOLLOW, LAND
    proto = {"auto": None, "gl": P.GL, "legacy": P.LEGACY}[a.protocol]
    modes = {"hover": HOVER, "stabilise": STABILISE, "follow": FOLLOW, "autoland": LAND}
    App(source=_source(a.source), dry_run=a.dry_run, record=not a.no_record, protocol=proto,
        hfov=a.hfov, start_mode=modes[a.mode], follow_model=a.model, transport=a.transport).run()


def cmd_replay(a):
    from pathlib import Path
    from .app import App, STABILISE, FOLLOW, LAND
    path = Path(a.recording)
    video = path / "video.avi" if path.is_dir() else path
    modes = {"stabilise": STABILISE, "follow": FOLLOW, "autoland": LAND}
    App(source=str(video), dry_run=True, record=False, hfov=a.hfov, realtime=False,
        start_mode=modes[a.mode], follow_model=a.model).run()


def cmd_probe(a):
    from .link import DroneLink
    print(f"Talking to {P.DRONE_IP}:{P.CONTROL_PORT} for {a.seconds}s (sticks neutral, no takeoff)...")
    link = DroneLink().start()
    seen = set()
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < a.seconds:
            st = link.stats
            if st.last_rx_hex and st.last_rx_hex not in seen:
                seen.add(st.last_rx_hex)
                print(f"{time.monotonic() - t0:6.2f}s  rx {st.last_rx_hex}")
            time.sleep(0.02)
    finally:
        link.close()
    st = link.stats
    print("\nsummary")
    print(f"  sent {st.tx} packets, received {st.rx}")
    print(f"  packet kinds (length:first byte -> count): {st.rx_kinds}")
    if st.device_id is not None:
        print(f"  device id {st.device_id}  -> {'GL (21-byte)' if link.protocol == P.GL else 'legacy (9-byte)'} control,"
              f" camera {st.resolution or 'unknown resolution'}")
    elif st.rx == 0:
        print("  nothing came back: is this PC on the drone's Wi-Fi (192.168.1.x)? Is a firewall blocking UDP?")


def cmd_latency(a):
    import cv2
    import numpy as np
    from .video import LatestFrameSource
    from .link import DroneLink
    src_url = _source(a.source)
    link = DroneLink(dry_run=not str(src_url).startswith("rtsp")).start()  # heartbeat keeps the drone streaming
    src = LatestFrameSource(src_url, transport=a.transport).start()
    print("Point the drone's front camera at the clock on the left. Video delay = left clock - "
          "clock shown in the video. Take a phone photo of the window to read both at once. Q quits.")
    t0 = time.monotonic()
    try:
        while True:
            clock = np.zeros((360, 480, 3), np.uint8)
            ms = int(time.monotonic() * 1000) % 100000
            cv2.putText(clock, f"{ms:05d}", (40, 210), cv2.FONT_HERSHEY_SIMPLEX, 3.2, (255, 255, 255), 8)
            fr = src.get()
            if fr is not None:
                vid = cv2.resize(fr.image, (480, 360))
                note = f"{fr.image.shape[1]}x{fr.image.shape[0]}  {src.fps:4.1f} fps  {src.info}"
            else:
                vid = np.zeros((360, 480, 3), np.uint8)
                note = f"{src.info}  ({time.monotonic() - t0:.0f} s)"
            cv2.putText(vid, note, (8, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            cv2.imshow("latency: clock | drone video", np.hstack([clock, vid]))
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        fr = src.get()
        print(f"video: {src.info}; " + (f"{fr.image.shape[1]}x{fr.image.shape[0]} at {src.fps:.1f} fps"
                                      if fr is not None else "no frames received"))
        src.stop()
        link.close()
        cv2.destroyAllWindows()


def cmd_pad(a):
    from .pad import save_pad
    png, pdf = save_pad()
    print(f"wrote {png}" + (f" and {pdf}" if pdf else " (pip install pillow for the PDF)"))
    print("Print at 100% / 'actual size' on A4, check the 100 mm line, tape it flat, NOSE towards the drone's front.")


def main(argv=None):
    p = argparse.ArgumentParser(prog="e88vision", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-v", "--verbose", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True)

    f = sub.add_parser("fly", help="fly with vision autopilots")
    f.add_argument("--source", default=P.RTSP_URL, help="RTSP url, webcam index (0) or video file")
    f.add_argument("--dry-run", action="store_true", help="send nothing to the drone (test with a webcam/file)")
    f.add_argument("--no-record", action="store_true")
    f.add_argument("--protocol", choices=["auto", "gl", "legacy"], default="auto")
    f.add_argument("--hfov", type=float, default=90.0, help="down camera horizontal field of view, degrees")
    f.add_argument("--mode", choices=["hover", "stabilise", "follow", "autoland"], default="hover")
    f.add_argument("--model", default="yolo11n.pt", help="YOLO model for follow mode")
    f.add_argument("--transport", choices=["auto", "udp", "tcp"], default="auto", help="RTSP transport")
    f.set_defaults(fn=cmd_fly)

    r = sub.add_parser("replay", help="run an autopilot on a recorded flight (nothing is sent)")
    r.add_argument("recording", help="recordings/<stamp> folder or a video file")
    r.add_argument("--mode", choices=["stabilise", "follow", "autoland"], required=True)
    r.add_argument("--hfov", type=float, default=90.0)
    r.add_argument("--model", default="yolo11n.pt")
    r.set_defaults(fn=cmd_replay)

    pr = sub.add_parser("probe", help="log what the drone sends back")
    pr.add_argument("--seconds", type=float, default=15)
    pr.set_defaults(fn=cmd_probe)

    la = sub.add_parser("latency", help="measure video delay")
    la.add_argument("--source", default=P.RTSP_URL)
    la.add_argument("--transport", choices=["auto", "udp", "tcp"], default="auto", help="RTSP transport")
    la.set_defaults(fn=cmd_latency)

    pa = sub.add_parser("pad", help="write the printable landing pad")
    pa.set_defaults(fn=cmd_pad)

    a = p.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    try:
        a.fn(a)
    except KeyboardInterrupt:
        print("stopped")


if __name__ == "__main__":
    sys.exit(main())
