#!/usr/bin/env python3
"""camera_check.py - can Gazebo keep real-time speed with the down camera running?  Also saves one picture to look at.

Runs in Ubuntu while the camera world flies.    python3 camera_check.py seconds picture.png result.json
"""
import json
import sys
import threading
import time
import urllib.request

sys.path[:0] = ["/usr/lib/python3/dist-packages", "/usr/local/lib/python3/dist-packages"]
import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node

secs = float(sys.argv[1])
picture, out = sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None
rtf, frames = [], []
last = {}
fc = {}


def on_image(m):
    frames.append(time.monotonic())
    last["m"] = m


node = Node()
node.subscribe(Image, "/X3/down_camera/image", on_image)
node.subscribe(WorldStatistics, "/stats", lambda m: rtf.append(m.real_time_factor))


def telemetry():
    while True:
        try:
            with urllib.request.urlopen("http://127.0.0.1:8770/events", timeout=5) as r:
                for line in r:
                    if line.startswith(b"data:"):
                        fc.update(json.loads(line[5:]))
        except Exception:
            time.sleep(0.5)


threading.Thread(target=telemetry, daemon=True).start()
until = time.monotonic() + 120
while fc.get("stateName") != "FLYING" and time.monotonic() < until:
    time.sleep(0.2)
time.sleep(5)
rtf.clear()
frames.clear()
time.sleep(secs)
fps = len(frames) / secs
r = sorted(rtf)
result = {"seconds": secs, "rtf_mean": round(sum(r) / max(1, len(r)), 3), "rtf_10pct": round(r[len(r) // 10], 3) if r else 0, "camera_fps": round(fps, 1),
          "imu_hz": fc.get("imuHz"), "state": fc.get("stateName"), "tilt": [fc.get("roll"), fc.get("pitch")], "alt": fc.get("alt")}
print(result, flush=True)
m = last.get("m")
if m is not None:
    img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
    cv2.imwrite(picture, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    result["image_size"] = [m.width, m.height]
    print("picture saved", m.width, m.height, flush=True)
else:
    print("no picture arrived", flush=True)
if out:
    json.dump(result, open(out, "w"), indent=1)
