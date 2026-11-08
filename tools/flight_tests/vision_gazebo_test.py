#!/usr/bin/env python3
"""vision_gazebo_test.py - phases 8 and 9 in the real Gazebo, judged by its true positions.

  Phase 8: the marker position the camera works out, against where the pad really is, hovering and while moving
  Phase 9: lines up and lands on the pad; marker lost on the way down; asked to land with no marker in view

Runs in Ubuntu with the camera world and the tracker running (vision_gazebo_run.sh).    python3 vision_gazebo_test.py result.json
"""
import importlib.util
import json
import math
import os
import subprocess
import sys
import threading
import time
import urllib.request

sys.path[:0] = ["/usr/lib/python3/dist-packages", "/usr/local/lib/python3/dist-packages"]
from gz.msgs10.image_pb2 import Image
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("marker_tracker", os.path.join(HERE, "..", "vision", "marker_tracker.py"))
tracker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracker)

PAD_N, PAD_E = 4.0, 6.0
GOTO, NAV_CANCEL, MARKER_LAND, TAKEOFF = 23, 28, 29, 12
truth = {"x": 0.0, "y": 0.0, "z": 0.0}
fc = {}
results = {}
image = {"m": None}
node = Node()
node.subscribe(Odometry, "/model/X3/odometry", lambda m: truth.update(x=m.pose.position.x, y=m.pose.position.y, z=m.pose.position.z))
node.subscribe(Image, "/X3/down_camera/image", lambda m: image.update(m=m))


def telemetry():
    while True:
        try:
            with urllib.request.urlopen("http://127.0.0.1:8770/events", timeout=5) as r:
                for line in r:
                    if line.startswith(b"data:"):
                        fc.update(json.loads(line[5:]))
        except Exception:
            time.sleep(0.5)


def post(thr, roll=0.0, pitch=0.0, yaw=0.0):
    body = f"{thr},{roll},{pitch},{yaw}\n".encode()
    try:
        urllib.request.urlopen(urllib.request.Request("http://127.0.0.1:8770/control", data=body, method="POST"), timeout=1).read()
    except Exception:
        pass


def code(c, r=0.0, p=0.0, y=0.0):
    for _ in range(3):
        post(c, r, p, y)


def wait(test, secs, what):
    until = time.monotonic() + secs
    while time.monotonic() < until:
        if fc and test():
            return time.monotonic() - (until - secs)
        time.sleep(0.02)
    raise TimeoutError(f"{what} (state {fc.get('stateName')}, nav {fc.get('navKindName')} {fc.get('navStageName')}, reason {fc.get('reasonName')}, height {truth['z']:.2f})")


def check(name, ok, detail):
    results[name] = {"pass": bool(ok), "detail": detail}
    print(("PASS " if ok else "FAIL ") + name + ": " + str(detail), flush=True)


def go(n, e, h):
    """n, e in Gazebo's frame (it starts at the origin)."""
    code(GOTO, n - fc.get("homeN", 0.0), e - fc.get("homeE", 0.0), h)
    wait(lambda: fc.get("navKind") == 1, 3, "the trip to start")
    wait(lambda: fc.get("navKind") == 0 and fc.get("stateName") == "FLYING", 60, "the trip to end")
    time.sleep(3)


def estimate_errors(secs):
    """How far the camera's idea of the pad is from the real pad, using Gazebo's true position of the drone."""
    errs, heights = [], []
    until = time.monotonic() + secs
    while time.monotonic() < until:
        m = image["m"]
        if m is not None:
            f = tracker.find_marker(m, fc.get("roll", 0.0), fc.get("pitch", 0.0), fc.get("yaw", 0.0) - fc.get("yawOffset", 0.0))
            if f:
                errs.append(math.hypot(truth["y"] + f[0] - PAD_N, truth["x"] + f[1] - PAD_E))
                heights.append(f[2] - (truth["z"] - 0.05))
        time.sleep(0.1)
    return errs, heights


def phase8():
    for h in (3.0, 6.0):
        go(PAD_N, PAD_E, h)
        errs, hs = estimate_errors(6)
        worst = max(errs) if errs else 99
        check(f"Phase 8: hovering over the pad at {h:.0f} m: the camera's position of the pad is right", bool(errs) and worst < 0.2,
              f"{len(errs)} pictures, worst {worst:.3f} m off, height from the picture off by {max(abs(x) for x in hs) if hs else 99:.3f} m")
    # known motion: sliding past the pad at 4 m
    go(PAD_N - 1.5, PAD_E - 1.5, 4.0)
    code(GOTO, PAD_N + 1.5 - fc.get("homeN", 0.0), PAD_E + 1.5 - fc.get("homeE", 0.0), 4.0)
    errs, _ = estimate_errors(5)
    worst = max(errs) if errs else 99
    check("Phase 8: moving past the pad: the camera still has it right", len(errs) >= 5 and worst < 0.5, f"{len(errs)} pictures, worst {worst:.3f} m off")
    wait(lambda: fc.get("navKind") == 0, 30, "the slide to end")
    time.sleep(2)
    # no marker in view: nothing must be reported
    go(0.0, 0.0, 4.0)
    time.sleep(2)
    check("Phase 8: with no marker in the picture, nothing is reported", not fc.get("markerSeen"), f"seen: {fc.get('markerSeen')}")


def land_on_pad(n, e, h):
    go(n, e, h)
    wait(lambda: fc.get("markerSeen"), 5, "the camera to see the marker")
    code(MARKER_LAND)
    wait(lambda: fc.get("navKind") == 4, 3, "the marker landing to start")
    wait(lambda: fc.get("stateName") == "DISARMED", 60, "it to land")
    time.sleep(1)
    return math.hypot(truth["y"] - PAD_N, truth["x"] - PAD_E), fc.get("reasonName")


def take_off():
    time.sleep(2)
    code(TAKEOFF)
    wait(lambda: fc.get("stateName") == "FLYING", 25, "it to take off")
    time.sleep(3)


def phase9():
    worst = 0.0
    for n, e, h in ((PAD_N + 1.0, PAD_E + 1.0, 5.0), (PAD_N - 1.5, PAD_E + 0.5, 6.0), (PAD_N + 0.5, PAD_E - 0.6, 3.0)):
        miss, why = land_on_pad(n, e, h)
        worst = max(worst, miss)
        print(f"   from {n - PAD_N:+.1f}, {e - PAD_E:+.1f} m at {h:.0f} m: landed {miss:.3f} m from the centre, reason {why}", flush=True)
        check(f"Phase 9: from {n - PAD_N:+.1f}, {e - PAD_E:+.1f} m, {h:.0f} m up: lands on the pad", miss < 0.25 and why == "landed", f"{miss:.3f} m from the centre, {why}")
        take_off()
    # the marker is lost half way down: holds, then lands normally
    go(PAD_N + 0.5, PAD_E, 6.0)
    wait(lambda: fc.get("markerSeen"), 5, "the camera to see the marker")
    code(MARKER_LAND)
    wait(lambda: fc.get("navStage") == 7, 25, "it to start coming down")
    wait(lambda: truth["z"] < 4.5, 15, "it to come down a bit")
    subprocess.run(["pkill", "-f", "marker_tracker.py"], capture_output=True)
    h0 = truth["z"]
    t = wait(lambda: fc.get("stateName") == "LANDING", 8, "it to give up on the marker")
    held = abs(truth["z"] - h0)
    check("Phase 9: marker lost on the way down: holds its height, then lands normally", held < 0.5 and 2.0 < t < 5.0, f"waited {t:.1f} s, height moved {held:.2f} m")
    wait(lambda: fc.get("stateName") == "DISARMED", 40, "it to land")
    check("Phase 9: and it lands", fc.get("reasonName") == "landed", fc.get("reasonName"))
    # asked to land with no marker in view (the tracker has gone too): nothing happens
    take_off()
    code(MARKER_LAND)
    time.sleep(2)
    check("Phase 9: no marker in view: the command is refused", fc.get("stateName") == "FLYING" and fc.get("navKind") == 0, fc.get("stateName"))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else None
    threading.Thread(target=telemetry, daemon=True).start()
    until = time.monotonic() + 120
    while fc.get("stateName") != "FLYING" and time.monotonic() < until:
        time.sleep(0.2)
    time.sleep(4)
    for stage in (phase8, phase9):
        print(f"=== {stage.__name__} {time.strftime('%X')}", flush=True)
        try:
            stage()
        except TimeoutError as e:
            check(f"{stage.__name__} ran to the end", False, f"timed out waiting for {e}")
    bad = [k for k, v in results.items() if not v["pass"]]
    print(f"{len(results) - len(bad)} of {len(results)} pass", flush=True)
    if out:
        json.dump({"date": time.strftime("%Y-%m-%d %H:%M"), "results": results}, open(out, "w"), indent=1)


main()
