#!/usr/bin/env python3
"""brake_check.py - run in Ubuntu while Gazebo and bridge_plus fly.  Holds a stick for 3 s, then lets go, and measures with
Gazebo's true position how far the drone goes before it stops, and how fast Gazebo really runs.

    python3 brake_check.py [repeats] [result.json]
    python3 brake_check.py watch seconds [result.json]     someone else flies (the page): only watches each stick movement
"""
import json
import math
import sys
import threading
import time
import urllib.request

sys.path[:0] = ["/usr/lib/python3/dist-packages", "/usr/local/lib/python3/dist-packages"]
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node

pos = [0.0, 0.0, 0.0]
vel = [0.0, 0.0]
rtf = []
fc = {}
stick = {"roll": 0.0, "pitch": 0.0}


def on_odom(m):
    pos[:] = [m.pose.position.x, m.pose.position.y, m.pose.position.z]
    vel[:] = [m.twist.linear.x, m.twist.linear.y]


def on_stats(m):
    rtf.append(m.real_time_factor)


node = Node()
node.subscribe(Odometry, "/model/X3/odometry", on_odom)
node.subscribe(WorldStatistics, "/stats", on_stats)


def pilot():
    while True:
        try:
            body = f"2.5,{stick['roll']},{stick['pitch']},0\n".encode()
            urllib.request.urlopen(urllib.request.Request("http://127.0.0.1:8770/control", data=body, method="POST"), timeout=1).read()
        except Exception:
            pass
        time.sleep(0.04)


def telemetry():
    while True:
        try:
            with urllib.request.urlopen("http://127.0.0.1:8770/events", timeout=5) as r:
                for line in r:
                    if line.startswith(b"data:"):
                        fc.update(json.loads(line[5:]))
        except Exception:
            time.sleep(0.5)


WATCH = len(sys.argv) > 1 and sys.argv[1] == "watch"
if not WATCH:
    threading.Thread(target=pilot, daemon=True).start()
threading.Thread(target=telemetry, daemon=True).start()


def watch(seconds, path):
    """Each time a stick moves and returns: how far the drone went after that, in Gazebo's own truth."""
    rows = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if abs(fc.get("rcRoll", 0.0)) + abs(fc.get("rcPitch", 0.0)) < 0.1 or fc.get("stateName") != "FLYING":
            time.sleep(0.02)
            continue
        start = list(pos)
        rtf.clear()
        t0 = time.monotonic()
        while abs(fc.get("rcRoll", 0.0)) + abs(fc.get("rcPitch", 0.0)) >= 0.02 and time.monotonic() < end:
            time.sleep(0.02)
        at_release = math.hypot(pos[0] - start[0], pos[1] - start[1])
        speed = math.hypot(*vel)
        t1 = time.monotonic()
        worst = 0.0
        rates = []
        while math.hypot(*vel) > 0.2 and time.monotonic() - t1 < 25:
            worst = max(worst, abs(math.hypot(*vel) - math.hypot(fc.get("velN", 0.0), fc.get("velE", 0.0))))
            rates.append(fc.get("pilotHz", 0))
            time.sleep(0.05)
        r = {"held_s": round(t1 - t0, 2), "at_release_m": round(at_release, 2), "speed_at_release": round(speed, 2), "stop_s": round(time.monotonic() - t1, 2),
             "coast_m": round(math.hypot(pos[0] - start[0], pos[1] - start[1]) - at_release, 2), "worst_speed_estimate_error": round(worst, 2),
             "real_time_factor": round(sum(rtf) / max(1, len(rtf)), 2), "lowest_rtf": round(min(rtf), 2) if rtf else None, "pilotHz": min(rates) if rates else None}
        rows.append(r)
        print(r, flush=True)
        if path:
            json.dump(rows, open(path, "w"), indent=1)


if WATCH:
    watch(float(sys.argv[2]), sys.argv[3] if len(sys.argv) > 3 else None)
    sys.exit(0)


def wait_hold(secs=25):
    until = time.monotonic() + secs
    while time.monotonic() < until:
        if fc.get("stateName") == "FLYING" and fc.get("posStateName") == "HOLDING" and math.hypot(*vel) < 0.15:
            return True
        time.sleep(0.05)
    return False


repeats = int(sys.argv[1]) if len(sys.argv) > 1 else 4
out = []
until = time.monotonic() + 120
while fc.get("stateName") != "FLYING" and time.monotonic() < until:
    time.sleep(0.2)
time.sleep(4)
for i in range(repeats):
    key = ("pitch", -0.6) if i % 2 == 0 else ("roll", 0.6)
    if not wait_hold():
        print("never settled", fc.get("stateName"), fc.get("posStateName"), vel)
        break
    start = list(pos)
    rtf.clear()
    t0 = time.monotonic()
    est = []
    stick[key[0]] = key[1]
    time.sleep(3.0)
    stick[key[0]] = 0.0
    at_release = math.hypot(pos[0] - start[0], pos[1] - start[1])
    speed_release = math.hypot(*vel)
    t1 = time.monotonic()
    worst_gap = 0.0
    while math.hypot(*vel) > 0.2 and time.monotonic() - t1 < 25:
        est.append((math.hypot(*vel), math.hypot(fc.get("velN", 0.0), fc.get("velE", 0.0))))
        worst_gap = max(worst_gap, abs(est[-1][0] - est[-1][1]))
        time.sleep(0.05)
    stop_s = time.monotonic() - t1
    total = math.hypot(pos[0] - start[0], pos[1] - start[1])
    row = {"stick": key[0], "at_release_m": round(at_release, 2), "speed_at_release": round(speed_release, 2), "stop_s": round(stop_s, 2),
           "total_m": round(total, 2), "coast_m": round(total - at_release, 2), "worst_speed_estimate_error": round(worst_gap, 2),
           "real_time_factor": round(sum(rtf) / max(1, len(rtf)), 2), "pilotHz": fc.get("pilotHz")}
    out.append(row)
    print(row, flush=True)
if len(sys.argv) > 2:
    json.dump(out, open(sys.argv[2], "w"), indent=1)
