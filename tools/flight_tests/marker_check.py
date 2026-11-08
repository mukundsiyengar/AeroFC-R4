#!/usr/bin/env python3
"""marker_check.py - the red pole in Gazebo: move it the way Gazebo's move tool does, ask bridge_plus where it is, fly there.

Runs in Ubuntu while Gazebo and bridge_plus fly.    python3 marker_check.py north east result.json
"""
import json
import math
import subprocess
import sys
import threading
import time
import urllib.request

sys.path[:0] = ["/usr/lib/python3/dist-packages", "/usr/local/lib/python3/dist-packages"]
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node

north, east = float(sys.argv[1]), float(sys.argv[2])
truth = [0.0, 0.0, 0.0]
fc = {}
node = Node()
node.subscribe(Odometry, "/model/X3/odometry", lambda m: truth.__setitem__(slice(None), [m.pose.position.x, m.pose.position.y, m.pose.position.z]))


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
time.sleep(4)
out = subprocess.run(["gz", "service", "-s", "/world/quadcopter/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req",
                      f'name: "target_marker", position: {{x: {east}, y: {north}, z: 0}}'], capture_output=True, text=True).stdout
print("moved the pole:", out.strip(), flush=True)
time.sleep(1)
m = json.loads(urllib.request.urlopen("http://127.0.0.1:8770/marker", timeout=15).read())
print("bridge says the pole is at", m, flush=True)
body = f"23,{m['north'] - fc['homeN']},{m['east'] - fc['homeE']},3\n".encode()
for _ in range(3):
    urllib.request.urlopen(urllib.request.Request("http://127.0.0.1:8770/control", data=body, method="POST"), timeout=1).read()
time.sleep(2)
until = time.monotonic() + 60
while fc.get("navKind") != 0 and time.monotonic() < until:
    time.sleep(0.1)
time.sleep(6)
miss = math.hypot(truth[0] - east, truth[1] - north)
print(f"the drone is {miss:.3f} m from the pole", flush=True)
result = {"pass": abs(m["north"] - north) < 0.01 and abs(m["east"] - east) < 0.01 and miss < 0.2, "miss_m": round(miss, 3), "marker": m}
print("PASS" if result["pass"] else "FAIL", result, flush=True)
if len(sys.argv) > 3:
    json.dump(result, open(sys.argv[3], "w"), indent=1)
