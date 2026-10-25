# E88

Tools for flying an E88 / E88 Pro Wi-Fi drone (RC-UFO family, `192.168.1.1`) from your own software.

| Folder | What | Status |
|---|---|---|
| [`pc/`](pc/) | **e88vision**: Windows/Python vision autopilots (optical-flow stabilise, person follow, ArUco autoland), with recording, replay, latency and protocol probe tools | Main line of work |
| [`ios/`](ios/) | iPhone manual controller (SwiftUI), built on GitHub Actions and delivered through TestFlight | Parked; may become the field front-end later |

Protocol research, covering what the drone accepts and what it sends back: the *E88 Drone: Feedback Protocol & Vision Autopilot Research* doc.

The E88 sends no flight telemetry: no battery, attitude, altitude or position. Everything the autopilots know comes from the two cameras.
