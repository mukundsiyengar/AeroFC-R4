#!/usr/bin/env python3
"""validate_gazebo_test.py - phase 12: the complete system working together for a long time.

A mixed flight of every kind of flying one after the other: trips, sticks, climbs, turns, return to home, missions and a lost link. All the
way through it records timing and motor outputs, and judges the flying by Gazebo's own true position. After it, the safety commands again.
Power can't be measured from here: that takes a USB meter on the board (docs/13).

Runs in Ubuntu while Gazebo and bridge_plus fly, started by validate_gazebo_run.sh.    python3 validate_gazebo_test.py result.json [minutes]
"""
import json
import math
import random
import statistics
import sys
import threading
import time
import urllib.request

sys.path[:0] = ["/usr/lib/python3/dist-packages", "/usr/local/lib/python3/dist-packages"]
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node

TAKEOFF, LAND, KILL, RTH, GOTO, WAYPOINT, MISSION_START, MISSION_CLEAR, NAV_CANCEL = 12, 13, 16, 22, 23, 25, 26, 27, 28
GROUND_Z = 0.05
truth = {"x": 0.0, "y": 0.0, "z": 0.0, "vx": 0.0, "vy": 0.0}
fc = {}
results = {}
stream = {"on": True, "thr": 2.5, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
takeoff = {"x": 0.0, "y": 0.0}
lock = threading.Lock()
rec = {"tilt": 0.0, "speed": 0.0, "motorLow": 1.0, "motorHigh": 0.0, "badMotor": 0, "pinned": 0.0, "pinnedWorst": 0.0, "loop": [], "imu": [],
       "pilot": [], "seen": set(), "samples": 0, "hoverSpread": []}
node = Node()
node.subscribe(Odometry, "/model/X3/odometry", lambda m: truth.update(x=m.pose.position.x, y=m.pose.position.y, z=m.pose.position.z, vx=m.twist.linear.x, vy=m.twist.linear.y))


def telemetry():
    armed = False
    while True:
        try:
            with urllib.request.urlopen("http://127.0.0.1:8770/events", timeout=5) as r:
                for line in r:
                    if line.startswith(b"data:"):
                        fc.update(json.loads(line[5:]))
                        if fc.get("armed") and not armed:
                            takeoff.update(x=truth["x"], y=truth["y"])
                        armed = fc.get("armed")
        except Exception:
            time.sleep(0.5)


def post(path, body):
    try:
        urllib.request.urlopen(urllib.request.Request("http://127.0.0.1:8770" + path, data=body.encode(), method="POST"), timeout=2).read()
    except Exception:
        pass


def pilot():
    while True:
        if stream["on"]:
            with lock:
                post("/control", f"{stream['thr']},{stream['roll']},{stream['pitch']},{stream['yaw']}\n")
        time.sleep(0.04)


def code(c, r=0.0, p=0.0, y=0.0):
    with lock:
        for _ in range(3):
            post("/control", f"{c},{r},{p},{y}\n")


def recorder():
    """Everything that must hold all the way through."""
    next_second = time.monotonic()
    while True:
        time.sleep(0.05)
        if not fc.get("airborne"):
            rec["pinned"] = 0.0
            continue
        rec["samples"] += 1
        rec["seen"].add((fc.get("stateName"), fc.get("reasonName")))
        rec["tilt"] = max(rec["tilt"], abs(fc.get("roll", 0.0)), abs(fc.get("pitch", 0.0)))
        rec["speed"] = max(rec["speed"], math.hypot(truth["vx"], truth["vy"]))
        m = fc.get("motors", [0.0, 0.0, 0.0, 0.0])
        if not all(isinstance(v, (int, float)) and 0.0 <= v <= 1.0 for v in m):
            rec["badMotor"] += 1
        rec["motorLow"] = min(rec["motorLow"], *m)
        rec["motorHigh"] = max(rec["motorHigh"], *m)
        rec["pinned"] = rec["pinned"] + 0.05 if min(m) <= 0.01 or max(m) >= 0.99 else 0.0
        rec["pinnedWorst"] = max(rec["pinnedWorst"], rec["pinned"])
        if time.monotonic() >= next_second:
            next_second += 1.0
            rec["loop"].append(fc.get("loopUs", 0))
            rec["imu"].append(fc.get("imuHz", 0))
            rec["pilot"].append(fc.get("pilotHz", 0))


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


def from_home():
    return truth["y"] - takeoff["y"], truth["x"] - takeoff["x"]


def holding():
    wait(lambda: fc.get("stateName") == "FLYING" and fc.get("posStateName") == "HOLDING", 25, "position hold")


def nav_stop():
    if fc.get("stateName") == "AUTO":
        code(NAV_CANCEL)
        wait(lambda: fc.get("stateName") == "FLYING", 3, "the trip to stop")
    holding()


class Flight:
    def __init__(self):
        self.misses = []
        self.home = []
        self.points = []
        self.counts = {}
        self.rng = random.Random(12)

    def done(self, what):
        self.counts[what] = self.counts.get(what, 0) + 1

    def trip(self):
        n, e, h = self.rng.uniform(-25, 25), self.rng.uniform(-25, 25), self.rng.uniform(3, 8)
        code(GOTO, n, e, h)
        wait(lambda: fc.get("navKind") == 1, 3, "the trip to start")
        wait(lambda: fc.get("navKind") == 0 and fc.get("stateName") == "FLYING", 60, "the trip to end")
        time.sleep(1)
        tn, te = from_home()
        self.misses.append(math.hypot(tn - n, te - e))
        self.done("trips")

    def sticks(self):
        roll, pitch = self.rng.choice([(0.0, -0.6), (0.0, 0.6), (0.6, 0.0), (-0.6, 0.0), (0.5, -0.5)])
        stream.update(roll=roll, pitch=pitch)
        time.sleep(self.rng.uniform(1.5, 3.0))
        stream.update(roll=0.0, pitch=0.0)
        holding()
        self.done("stick moves")

    def climb(self):
        stream["thr"] = 3.0
        time.sleep(2.0)
        stream["thr"] = 2.0
        time.sleep(1.5)
        stream["thr"] = 2.5
        holding()
        self.done("climbs and descents")

    def spin(self):
        stream["yaw"] = self.rng.choice([-0.6, 0.6])
        time.sleep(4.0)
        stream["yaw"] = 0.0
        holding()
        self.done("turns")

    def home_hover(self):
        code(RTH)
        wait(lambda: fc.get("navKind") == 3, 3, "RTH to start")
        wait(lambda: fc.get("navStage") == 5, 60, "the hover at home")
        time.sleep(2)
        self.home.append(math.hypot(*from_home()))
        nav_stop()
        self.done("returns home")

    def mission(self):
        code(MISSION_CLEAR)
        pts = [(self.rng.uniform(-15, 15), self.rng.uniform(-15, 15), self.rng.uniform(3, 6)) for _ in range(3)]
        for i, (n, e, h) in enumerate(pts):
            post("/control", f"{WAYPOINT},{n},{e},{h + 1000 * i}\n")
            time.sleep(0.05)
        code(MISSION_START)
        wait(lambda: fc.get("navKind") == 2, 3, "the mission to start")
        wait(lambda: fc.get("navKind") == 3, 90, "the mission to finish")
        wait(lambda: fc.get("navStage") == 5, 60, "the hover at home after the mission")
        time.sleep(2)
        self.home.append(math.hypot(*from_home()))
        nav_stop()
        self.done("missions")

    def link_drop(self):
        stream["on"] = False
        wait(lambda: fc.get("linkLost") and fc.get("navKind") == 3, 5, "the lost link to start RTH")
        stream["on"] = True
        wait(lambda: not fc.get("linkLost"), 5, "the link to come back")
        time.sleep(2)
        still_home = fc.get("navKind") == 3
        nav_stop()
        self.done("lost links" if still_home else "lost links that stopped the flight home")

    def run(self, minutes):
        actions = [self.trip, self.sticks, self.trip, self.climb, self.spin, self.home_hover, self.trip, self.link_drop, self.mission]
        until = time.monotonic() + minutes * 60.0
        i = 0
        failed = None
        while time.monotonic() < until:
            act = actions[i % len(actions)]
            i += 1
            try:
                act()
            except TimeoutError as e:
                failed = f"{act.__name__}: timed out waiting for {e}"
                break
        return failed


def pct(values, q):
    v = sorted(values)
    return v[min(len(v) - 1, int(len(v) * q))] if v else 0


def judge(flight, failed, minutes):
    check("Phase 12: the whole mixed flight, without one failure", failed is None, failed or f"{minutes} minutes: " + ", ".join(f"{v} {k}" for k, v in flight.counts.items()))
    bad = sorted({f"{s} {r}" for s, r in rec["seen"] if s in ("FAILSAFE", "LANDING") or r not in ("-", "pilot link")})
    check("Phase 12: no failsafe, no landing that nobody asked for", not bad, ", ".join(bad) if bad else f"{rec['samples']} samples in the air")
    worst_trip = max(flight.misses, default=99.0)
    check("Phase 12: every trip arrives within 0.5 m", worst_trip < 0.5, f"{len(flight.misses)} trips, worst {worst_trip:.3f} m")
    worst_home = max(flight.home, default=99.0)
    check("Phase 12: every return home arrives within 0.5 m", worst_home < 0.5, f"{len(flight.home)} returns, worst {worst_home:.3f} m")
    check("Phase 12: never tilts or speeds out of control", rec["tilt"] < 35 and rec["speed"] < 15, f"worst tilt {rec['tilt']:.1f} deg, top speed {rec['speed']:.1f} m/s")
    check("Phase 12: motor outputs always valid", rec["badMotor"] == 0 and rec["samples"] > 0, f"{rec['badMotor']} invalid of {rec['samples']}, between {rec['motorLow']:.3f} and {rec['motorHigh']:.3f}")
    check("Phase 12: no motor stuck at its limit for over 1 s", rec["pinnedWorst"] <= 1.0, f"longest {rec['pinnedWorst']:.2f} s")
    loop = rec["loop"]
    check("Phase 12: one control step takes under 10 ms", loop and max(loop) < 10000, f"typical {pct(loop, 0.5)} us, worst {max(loop, default=0)} us (sensors come every 20 ms)")
    imu = rec["imu"]
    check("Phase 12: sensor rate near 50 a second", imu and statistics.median(imu) >= 40, f"typical {statistics.median(imu) if imu else 0} a second, lowest {min(imu, default=0)}")
    pilots = rec["pilot"]
    check("Phase 12: pilot stream arrives", pilots and statistics.median(pilots) >= 15, f"typical {statistics.median(pilots) if pilots else 0} a second")
    try:
        s = json.loads(urllib.request.urlopen("http://127.0.0.1:8770/stats", timeout=3).read())
    except Exception as e:
        s = None
        check("Phase 12: control delay", False, f"no figures from the bridge: {e}")
    if s:
        n = max(1, s["flightPackets"])
        check("Phase 12: control delay stays under the 80 ms limit", s["over80"] / n < 0.001,
              f"{s['over80']} of {s['flightPackets']} motor packets over 80 ms, worst {s['worstMs']:.0f} ms, average {s['sumMs'] / n:.1f} ms")
        check("Phase 12: delays of seconds are Ubuntu's clock moving, not the link", s["clockJumped"] <= s["wallJumps"] * 3 + 3,
              f"{s['clockJumped']} motor packets looked over 1 s late, and the clock moved {s['wallJumps']} times")


def safety():
    nav_stop()
    code(LAND)
    t = wait(lambda: fc.get("stateName") == "DISARMED", 60, "it to land")
    check("Phase 12 safety: LAND comes down and stops the motors", fc.get("reasonName") == "landed" and not fc.get("crashed"), f"after {t:.1f} s, reason {fc.get('reasonName')}")
    time.sleep(4)
    code(TAKEOFF)
    wait(lambda: fc.get("stateName") == "FLYING", 25, "it to take off again")
    holding()
    code(KILL)
    t = wait(lambda: max(fc.get("motors", [1.0])) == 0.0, 2, "the motors to stop")
    check("Phase 12 safety: KILL stops the motors at once", t < 0.5 and fc.get("reasonName") == "KILL", f"after {t:.2f} s, reason {fc.get('reasonName')}")
    wait(lambda: truth["z"] - GROUND_Z < 0.3, 15, "it to reach the ground")
    time.sleep(4)
    code(TAKEOFF)
    t = wait(lambda: fc.get("stateName") == "FLYING", 25, "it to take off after the KILL")
    holding()
    check("Phase 12 safety: flies again after a KILL", True, f"flying after {t:.1f} s")
    stream["on"] = False
    t = wait(lambda: fc.get("navKind") == 3, 5, "the lost link to start RTH")
    check("Phase 12 safety: a lost link still starts the flight home", t < 3, f"after {t:.1f} s")
    stream["on"] = True


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else None
    minutes = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    for job in (telemetry, pilot, recorder):
        threading.Thread(target=job, daemon=True).start()
    until = time.monotonic() + 120
    while fc.get("stateName") != "FLYING" and time.monotonic() < until:
        time.sleep(0.2)
    time.sleep(4)
    flight = Flight()
    failed = flight.run(minutes)
    judge(flight, failed, minutes)
    print(f"=== safety {time.strftime('%X')}", flush=True)
    try:
        safety()
    except TimeoutError as e:
        check("Phase 12 safety: ran to the end", False, f"timed out waiting for {e}")
    bad = [k for k, v in results.items() if not v["pass"]]
    print(f"{len(results) - len(bad)} of {len(results)} pass", flush=True)
    if out:
        json.dump({"date": time.strftime("%Y-%m-%d %H:%M"), "results": results}, open(out, "w"), indent=1)


main()
