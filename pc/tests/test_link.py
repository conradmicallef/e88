import socket, threading, time
from e88vision import protocol as P
from e88vision.link import DroneLink, Sticks

def test_packets_match_spec_examples():
    # neutral legacy: 03 66 80 80 80 80 00 00 99
    assert P.legacy_packet(128, 128, 128, 128, set(), False).hex() == "036680808080000099"
    gl = P.gl_packet(128, 128, 128, 128, set(), False)
    assert len(gl) == 21 and gl[:3] == b"\x03\x66\x14" and gl[-1] == 0x99
    assert gl[19] == (128 ^ 128 ^ 128 ^ 128 ^ 0 ^ 2)   # altitude-hold bit in F2

def test_telemetry_detects_gl_and_ignores_status_echo():
    assert P.parse_telemetry(bytes([90, 0, 0])).protocol == P.GL
    assert P.parse_telemetry(bytes([9, 0, 0])).protocol == P.LEGACY
    assert P.parse_telemetry(bytes([0x66, 0, 1])).protocol is None

def test_dead_man_neutralises_stale_sticks():
    link = DroneLink(dry_run=True)
    link.set_sticks(Sticks(roll=1.0))
    now = time.monotonic()
    assert link.build_packet(now)[2] == 255
    assert link.build_packet(now + 1.0)[2] == 128

def test_loopback_with_fake_drone():
    drone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); drone.bind(("127.0.0.1", 0))
    drone.settimeout(2)
    link = DroneLink(host="127.0.0.1", port=drone.getsockname()[1]).start()
    got = []
    d, addr = drone.recvfrom(64); got.append(d)          # first: heartbeat
    drone.sendto(bytes([91, 0, 0, 0, 0]), addr)          # pretend to be a GL 1080p drone
    deadline = time.time() + 2
    while time.time() < deadline:
        d, _ = drone.recvfrom(64); got.append(d)
        if len(d) == 21: break
    link.close()
    assert got[0] == P.HEARTBEAT
    assert link.protocol == P.GL and link.stats.resolution == "1080p"
    assert any(len(x) == 21 for x in got)
