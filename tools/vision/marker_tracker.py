#!/usr/bin/env python3
"""marker_tracker.py - looks through Gazebo's down camera for the landing marker, and tells the flight controller where it is.

Runs in Ubuntu beside Gazebo and bridge_plus (run_sitl.sh --camera or run_hil.sh --camera).  It reads the same picture a real camera
would give, finds the ArUco marker with OpenCV, and works out how far north and east of the drone it is. Nothing here uses Gazebo's
true position.  The flight controller gets it as CODE_VISION (firmware/AeroFC_R4/src/core/flight_core.h) and does the flying.
"""
import json
import math
import sys
import threading
import time
import urllib.request

sys.path[:0] = ["/usr/lib/python3/dist-packages", "/usr/local/lib/python3/dist-packages"]
import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

MARKER_ID = 7
MARKER_M = 0.8            # the black square's side (make_camera_world.py)
HFOV_RAD = 1.047          # the camera's horizontal field of view
CODE_VISION = 24
SEND_HZ = 10.0

fc = {}
latest = {"image": None, "seq": 0}
detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())


def on_image(m):
    latest["image"] = m
    latest["seq"] += 1


def telemetry():
    while True:
        try:
            with urllib.request.urlopen("http://127.0.0.1:8770/events", timeout=5) as r:
                for line in r:
                    if line.startswith(b"data:"):
                        fc.update(json.loads(line[5:]))
        except Exception:
            time.sleep(0.5)


def send(north, east, seen):
    body = f"{CODE_VISION},{north:.3f},{east:.3f},{1 if seen else 0}\n".encode()
    try:
        urllib.request.urlopen(urllib.request.Request("http://127.0.0.1:8770/control", data=body, method="POST"), timeout=0.5).read()
    except Exception:
        pass


def find_marker(m, roll, pitch, yaw):
    """Returns (metres north, metres east of the drone, height the picture shows) or None.  Angles in degrees."""
    img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
    grey = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = detector.detectMarkers(grey)
    if ids is None:
        return None
    for c, i in zip(corners, ids.flatten()):
        if i != MARKER_ID:
            continue
        pts = c[0]
        side = np.mean([np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)])
        if side < 8:
            return None
        f = (m.width / 2) / math.tan(HFOV_RAD / 2)
        height = f * MARKER_M / side
        u, v = pts.mean(axis=0)
        # top of the picture = towards the nose, right of the picture = the drone's right
        forward = -(v - m.height / 2) / f * height
        right = (u - m.width / 2) / f * height
        # a tilted drone's camera looks to one side: take that out
        forward += height * math.tan(math.radians(pitch))
        right -= height * math.tan(math.radians(roll))
        a = math.radians(yaw)
        return forward * math.cos(a) - right * math.sin(a), forward * math.sin(a) + right * math.cos(a), height
    return None


def main():
    node = Node()
    node.subscribe(Image, "/X3/down_camera/image", on_image)
    threading.Thread(target=telemetry, daemon=True).start()
    print("[marker_tracker] looking for the landing marker", flush=True)
    seen_seq = 0
    last_seen = False
    period = 1.0 / SEND_HZ
    while True:
        t0 = time.monotonic()
        if latest["seq"] != seen_seq and latest["image"] is not None:
            seen_seq = latest["seq"]
            found = find_marker(latest["image"], fc.get("roll", 0.0), fc.get("pitch", 0.0), fc.get("yaw", 0.0) - fc.get("yawOffset", 0.0))
            if found:
                send(found[0], found[1], True)
            if bool(found) != last_seen:
                last_seen = bool(found)
                print(f"[marker_tracker] {'marker in view' if found else 'marker out of view'}", flush=True)
        time.sleep(max(0.0, period - (time.monotonic() - t0)))


if __name__ == "__main__":
    main()
