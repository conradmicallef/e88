import json, numpy as np, cv2
from e88vision.recorder import Recorder

def test_camera_switch_starts_new_segment(tmp_path):
    r = Recorder(tmp_path)
    for i in range(5): r.write(np.zeros((384, 640, 3), np.uint8), {"i": i})
    for i in range(5): r.write(np.zeros((360, 640, 3), np.uint8), {"i": i})
    r.close()
    vids = sorted(r.dir.glob("video_*.avi"))
    assert [v.name for v in vids] == ["video_001.avi", "video_002.avi"]
    for v, h in zip(vids, (384, 360)):
        cap = cv2.VideoCapture(str(v)); n = 0
        while cap.read()[0]: n += 1
        assert n == 5 and int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) == h
    segs = [json.loads(l).get("seg") for l in open(r.dir / "log.jsonl") if '"i"' in l]
    assert segs == [1] * 5 + [2] * 5
