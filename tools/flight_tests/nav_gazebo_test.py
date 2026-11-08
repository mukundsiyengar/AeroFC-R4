#!/usr/bin/env python3
"""nav_gazebo_test.py - return to home, go-to, missions, RC loss and hold detection in the real Gazebo.

Runs in Ubuntu while Gazebo and bridge_plus fly (see nav_gazebo_run.sh), with the software flight controller or the real board.
It sends what a pilot would send, and judges by Gazebo's own true position, never by what the flight controller says about itself.

    python3 nav_gazebo_test.py result.json [quick]        quick = four directions and five targets instead of eight and twenty
    NAV_STAGES="hold detection,go to" python3 ...         only these stages
"""
import json
import math
import os
import random
import subprocess
import sys
import threading
import time
import urllib.request

sys.path[:0] = ["/usr/lib/python3/dist-packages", "/usr/local/lib/python3/dist-packages"]
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node

CONTROL = "http://127.0.0.1:8770/control"
TOOL = os.path.expanduser("~/hil_test_r4/arduino_hil/control_input.py")      # the organisers' file, unchanged
RTH, GOTO, WAYPOINT, MISSION_START, MISSION_CLEAR, NAV_CANCEL, TAKEOFF, NORMAL_SPEED = 22, 23, 25, 26, 27, 28, 12, 18
GROUND_Z = 0.05     # Gazebo's height of the drone sitting on the ground

truth = {"x": 0.0, "y": 0.0, "z": 0.0, "vx": 0.0, "vy": 0.0, "seen": False}
fc = {}
results = {}
send_lock = threading.Lock()


def on_odom(m):
    truth.update(x=m.pose.position.x, y=m.pose.position.y, z=m.pose.position.z, vx=m.twist.linear.x, vy=m.twist.linear.y, seen=True)


node = Node()
node.subscribe(Odometry, "/model/X3/odometry", on_odom)


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
        urllib.request.urlopen(urllib.request.Request(CONTROL, data=body, method="POST"), timeout=1).read()
    except Exception:
        pass


class Pilot(threading.Thread):
    """The stream a pilot's page sends: 25 packets a second, or fewer, or broken, or none."""

    def __init__(self):
        super().__init__(daemon=True)
        self.on = False
        self.hz = 25.0
        self.broken = False
        self.roll = self.pitch = self.yaw = 0.0

    def run(self):
        while True:
            if self.on:
                with send_lock:
                    post(2.5, "nan" if self.broken else self.roll, self.pitch, self.yaw)
            time.sleep(1.0 / self.hz)


pilot = Pilot()


def code(c, roll=0.0, pitch=0.0, yaw=0.0):
    """Three copies with nothing between them, as the page does."""
    with send_lock:
        for _ in range(3):
            post(c, roll, pitch, yaw)


def wait(test, secs, what):
    until = time.monotonic() + secs
    while time.monotonic() < until:
        if fc and test():
            return time.monotonic() - (until - secs)
        time.sleep(0.02)
    raise TimeoutError(f"{what} (state {fc.get('stateName')}, nav {fc.get('navKindName')} {fc.get('navStageName')}, reason {fc.get('reasonName')}, "
                       f"position {truth['x']:.1f}, {truth['y']:.1f}, {truth['z']:.1f})")


def check(name, ok, detail):
    results[name] = {"pass": bool(ok), "detail": detail}
    print(("PASS " if ok else "FAIL ") + name + ": " + str(detail), flush=True)


def height():
    return truth["z"] - GROUND_Z


def from_home(home):
    return math.hypot(truth["x"] - home[0], truth["y"] - home[1])


def arrived(n=None):
    wait(lambda: fc.get("navKind") == GOTO_KIND, 3, "the trip to start")
    return wait(lambda: fc.get("navKind") == 0 and fc.get("stateName") == "FLYING", 100, "the trip to end")


GOTO_KIND, MISSION_KIND, RTH_KIND = 1, 2, 3
HOVER_STAGE, CLIMB_STAGE, CRUISE_STAGE = 5, 1, 3


def go(north, east, h):
    code(GOTO, north, east, h)
    return arrived()


def hovering(secs=70):
    return wait(lambda: fc.get("navKind") == RTH_KIND and fc.get("navStage") == HOVER_STAGE, secs, "the hover at home")


def take_back():
    """Stream on, stick moved: the pilot has it again."""
    pilot.on = True
    time.sleep(0.5)
    pilot.pitch = -0.5
    wait(lambda: fc.get("stateName") == "FLYING", 3, "the pilot to take over")
    pilot.pitch = 0.0
    wait(lambda: fc.get("posStateName") == "HOLDING", 15, "position hold")


def settle(secs=12):
    """Judge where it ends up, not where it is while still arriving: until Gazebo says it has really stopped."""
    good = 0
    until = time.monotonic() + secs
    while time.monotonic() < until and good < 40:
        still = abs(truth["vx"]) < 0.03 and abs(truth["vy"]) < 0.03
        good = good + 1 if still else 0
        time.sleep(0.05)


def measure_hover(home, secs):
    settle()
    d = h = 0.0
    until = time.monotonic() + secs
    while time.monotonic() < until:
        d = max(d, from_home(home))
        h = max(h, abs(height() - 2.0))
        time.sleep(0.05)
    return d, h


def wrench(z):
    subprocess.run(["gz", "topic", "-t", "/world/quadcopter/wrench/persistent", "--msgtype", "gz.msgs.EntityWrench", "-p",
                    f'entity: {{name: "X3", type: MODEL}}, wrench: {{force: {{z: {z}}}}}'], capture_output=True, timeout=15)


def wrench_clear():
    subprocess.run(["gz", "topic", "-t", "/world/quadcopter/wrench/clear", "--msgtype", "gz.msgs.Entity", "-p", 'name: "X3", type: MODEL'],
                   capture_output=True, timeout=15)


def test_their_tool(home):
    """Before any pilot stream: the organisers' own tool, unchanged, with the number codes."""
    subprocess.run(["python3", "-B", TOOL, "--throttle", "23", "--roll", "10", "--pitch", "5", "--yaw", "3", "--duration", "0.2"], capture_output=True, timeout=20)
    arrived()
    settle()
    miss = math.hypot(truth["x"] - home[0] - 5, truth["y"] - home[1] - 10)
    check("their tool: --throttle 23 goes to 10 m north, 5 m east, 3 m up", miss < 0.2 and abs(height() - 3) < 0.2, f"missed by {miss:.3f} m, height {height():.2f} m")
    subprocess.run(["python3", "-B", TOOL, "--throttle", "22", "--duration", "0.2"], capture_output=True, timeout=20)
    wait(lambda: fc.get("navKind") == RTH_KIND, 3, "RTH to start")
    hovering()
    d, h = measure_hover(home, 5)
    check("their tool: --throttle 22 returns home and hovers at 2 m", d < 0.5 and h < 0.1, f"{d:.3f} m from home, {h:.3f} m from 2 m")
    time.sleep(3)
    check("their tool: its silence doesn't count as a lost link", not fc.get("linkLost"), fc.get("linkLost"))


def test_rth_directions(home, dirs):
    worst_d = worst_h = 0.0
    worst_lost = slowest = 0.0
    bad = []
    take_back()
    for n, e in dirs:
        go(n, e, 3)
        pilot.on = False
        lost = wait(lambda: fc.get("navKind") == RTH_KIND, 6, "RC loss to start RTH")
        trip = hovering()
        d, h = measure_hover(home, 8)
        worst_d, worst_h = max(worst_d, d), max(worst_h, h)
        worst_lost, slowest = max(worst_lost, lost), max(slowest, trip)
        print(f"   from {n}, {e}: lost after {lost:.2f} s, trip {trip:.1f} s, {d:.3f} m from home, {h:.3f} m from 2 m", flush=True)
        if d >= 0.5 or h >= 0.1 or not 1.5 < lost < 3.5:
            bad.append((n, e))
        pilot.on = True
        time.sleep(3)
        if fc.get("navKind") != RTH_KIND or fc.get("linkLost"):
            bad.append("the link coming back changed something")
        take_back()
    check(f"RC lost from {len(dirs)} directions, 30 m away: home within 0.5 m, 2 m within 0.1 m", not bad,
          f"worst {worst_d:.3f} m from home, {worst_h:.3f} m from 2 m, lost after at most {worst_lost:.2f} s, slowest trip {slowest:.1f} s {bad or ''}")


def test_heights(home):
    go(20, 0, 1.5)
    start = (truth["x"], truth["y"])
    code(RTH)
    wait(lambda: fc.get("navStage") == CLIMB_STAGE, 3, "the climb")
    side = 0.0
    while fc.get("navStage") == CLIMB_STAGE:
        side = max(side, math.hypot(truth["x"] - start[0], truth["y"] - start[1]))
        time.sleep(0.05)
    check("low: climbs to 5 m before moving", side < 1.0 and abs(height() - 5) < 0.4, f"moved {side:.2f} m sideways, height {height():.2f} m after the climb")
    hovering()
    take_back()
    go(25, 0, 12)
    code(RTH)
    wait(lambda: fc.get("navKind") == RTH_KIND, 3, "RTH")
    lowest = 99.0
    while fc.get("navStage") not in (4, 5):
        lowest = min(lowest, height())
        time.sleep(0.05)
    check("high: stays at its height until it is over home", lowest > 11.5, f"lowest {lowest:.2f} m on the way")
    hovering()
    d, h = measure_hover(home, 5)
    check("then comes down to 2 m over home", d < 0.5 and h < 0.1, f"{d:.3f} m from home, {h:.3f} m from 2 m")
    take_back()


def test_loss_conditions():
    pilot.hz = 3.0
    t = wait(lambda: fc.get("navKind") == RTH_KIND, 10, "too few packets to start RTH")
    check("3 packets a second count as a lost link", True, f"RTH after {t:.1f} s")
    pilot.hz = 25.0
    wait(lambda: not fc.get("linkLost"), 6, "the link to come back")
    time.sleep(1)
    check("the rate comes back: it keeps going home", fc.get("navKind") == RTH_KIND, fc.get("navKindName"))
    take_back()
    pilot.broken = True
    t = wait(lambda: fc.get("navKind") == RTH_KIND, 8, "broken packets to start RTH")
    check("packets that aren't numbers count as a lost link", fc.get("pilotBad", 0) > 5, f"RTH after {t:.1f} s, {fc.get('pilotBad')} bad packets")
    pilot.broken = False
    wait(lambda: not fc.get("linkLost"), 4, "the link to come back")
    take_back()
    # the stream gone and a single command sent: it says nothing about the link
    pilot.on = False
    wait(lambda: fc.get("navKind") == RTH_KIND, 6, "RC loss to start RTH")
    code(NORMAL_SPEED)
    came_back = False
    until = time.monotonic() + 3
    while time.monotonic() < until:
        came_back = came_back or not fc.get("linkLost")
        time.sleep(0.02)
    check("a single command doesn't bring a lost link back", not came_back and fc.get("navKind") == RTH_KIND, f"link came back: {came_back}, {fc.get('navKindName')}")
    take_back()


def test_hold(home):
    code(RTH)
    hovering()
    time.sleep(35)
    check("35 s at home with nobody holding it: keeps hovering", fc.get("navStage") == HOVER_STAGE and fc.get("stateName") == "AUTO", fc.get("stateName"))
    for name, force in (("pulled down", -6), ("pushed up", 6)):
        wrench(force)
        t = wait(lambda: fc.get("stateName") == "DISARMED", 6, f"{name}: the motors to stop")
        check(f"{name} with {abs(force)} N: motors stop within 2 s", t < 2.0 and fc.get("reasonName") == "held", f"{t:.2f} s, reason {fc.get('reasonName')}")
        wrench_clear()
        wait(lambda: abs(truth["vx"]) < 1 and height() < 0.3, 15, "it to reach the ground")
        time.sleep(2)
        code(TAKEOFF)
        wait(lambda: fc.get("stateName") == "FLYING", 15, "it to take off again")
        time.sleep(3)
        code(RTH)
        hovering()
        time.sleep(4)
    code(NAV_CANCEL)
    wait(lambda: fc.get("stateName") == "FLYING", 3, "NAV_CANCEL")


def test_goto(home, count):
    rng = random.Random(7)
    worst = worst_h = 0.0
    for i in range(count):
        a, r = rng.uniform(0, 2 * math.pi), rng.uniform(5, 50)
        n, e, h = r * math.cos(a), r * math.sin(a), rng.uniform(2, 10)
        go(n, e, h)
        on_arrival = math.hypot(truth["x"] - home[0] - e, truth["y"] - home[1] - n)
        settle()
        miss = math.hypot(truth["x"] - home[0] - e, truth["y"] - home[1] - n)
        worst, worst_h = max(worst, miss), max(worst_h, abs(height() - h))
        print(f"   target {i + 1}: {n:.1f}, {e:.1f}, {h:.1f} m: {on_arrival:.3f} m away when it said arrived, then missed by {miss:.3f} m, height by {abs(height() - h):.3f} m", flush=True)
    check(f"{count} random targets up to 50 m away: within 0.2 m of each", worst < 0.2 and worst_h < 0.2, f"worst {worst:.3f} m, height {worst_h:.3f} m")


def test_mission(home):
    points = [(10, 0, 3), (10, 10, 5), (0, 10, 3), (-5, 5, 4)]
    code(MISSION_CLEAR)
    time.sleep(0.2)
    with send_lock:
        for i, (n, e, h) in enumerate(points):
            for _ in range(3):
                post(WAYPOINT, n, e, h + 1000 * i)
    time.sleep(0.5)
    code(MISSION_START)
    wait(lambda: fc.get("navKind") == MISSION_KIND, 3, "the mission to start")
    last = 0
    misses = []
    while fc.get("navKind") == MISSION_KIND:
        if fc.get("wpIndex", 0) != last:
            n, e, _ = points[last]
            misses.append(math.hypot(truth["x"] - home[0] - e, truth["y"] - home[1] - n))
            last = fc.get("wpIndex", 0)
        time.sleep(0.02)
    n, e, _ = points[last] if last < len(points) else points[-1]
    misses.append(math.hypot(truth["x"] - home[0] - e, truth["y"] - home[1] - n))
    check("mission: visits the four points in order", len(misses) == 4 and max(misses) < 0.3, [round(m, 3) for m in misses])
    check("mission: then goes home by itself", fc.get("navKind") == RTH_KIND, fc.get("navKindName"))
    hovering()
    d, h = measure_hover(home, 5)
    check("mission: ends hovering at home", d < 0.5 and h < 0.1, f"{d:.3f} m from home, {h:.3f} m from 2 m")
    take_back()


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else None
    quick = len(sys.argv) > 2 and sys.argv[2] == "quick"
    threading.Thread(target=telemetry, daemon=True).start()
    pilot.start()
    until = time.monotonic() + 120
    while not truth["seen"] and time.monotonic() < until:
        time.sleep(0.1)
    home = (truth["x"], truth["y"])
    print("home in Gazebo:", home, flush=True)
    stages = [
        ("their tool", lambda: test_their_tool(home)),
        ("RTH directions", lambda: test_rth_directions(home, [(30, 0), (0, 30), (-30, 0), (0, -30)] if quick else
                                                         [(30, 0), (21, 21), (0, 30), (-21, 21), (-30, 0), (-21, -21), (0, -30), (21, -21)])),
        ("heights", lambda: test_heights(home)),
        ("RC loss conditions", test_loss_conditions),
        ("hold detection", lambda: test_hold(home)),
        ("go to", lambda: test_goto(home, 5 if quick else 20)),
        ("mission", lambda: test_mission(home)),
    ]
    wait(lambda: fc.get("stateName") == "FLYING", 120, "the first take-off")
    time.sleep(3)
    if os.environ.get("NAV_STAGES"):
        wanted = os.environ["NAV_STAGES"].split(",")
        stages = [s for w in wanted for s in stages if s[0] == w]
        pilot.on = True
    for name, stage in stages:
        print(f"=== {name} {time.strftime('%X')}", flush=True)
        try:
            stage()
        except TimeoutError as e:
            check(f"{name}: ran to the end", False, f"timed out waiting for {e}")
            wrench_clear()
            pilot.on = True
            pilot.hz = 25.0
            pilot.broken = False
            pilot.roll = pilot.pitch = pilot.yaw = 0.0
            time.sleep(1)
            if fc.get("stateName") != "FLYING":
                code(NAV_CANCEL)
                time.sleep(1)
                if fc.get("stateName") == "DISARMED":
                    time.sleep(3)
                    code(TAKEOFF)
                try:
                    wait(lambda: fc.get("stateName") == "FLYING", 25, "flying again")
                except TimeoutError:
                    print("could not recover: stopping", flush=True)
                    break
    bad = [k for k, v in results.items() if not v["pass"]]
    print(f"{len(results) - len(bad)} of {len(results)} pass", flush=True)
    if out:
        json.dump({"date": time.strftime("%Y-%m-%d %H:%M"), "imuStale": fc.get("imuStale"), "imuTimeBad": fc.get("imuTimeBad"), "results": results}, open(out, "w"), indent=1)


main()
