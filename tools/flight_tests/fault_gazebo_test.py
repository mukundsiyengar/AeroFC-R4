#!/usr/bin/env python3
"""fault_gazebo_test.py - phase 11 in the real Gazebo: break things on purpose and check the expected response.

Faults go into the sensor values inside bridge_plus (POST /fault), so the organisers' file on disk stays as it is. Everything is judged
by Gazebo's true position.  Runs in Ubuntu while Gazebo and bridge_plus fly (fault_gazebo_run.sh).   python3 fault_gazebo_test.py result.json
"""
import csv
import glob
import json
import math
import os
import sys
import threading
import time
import urllib.request

sys.path[:0] = ["/usr/lib/python3/dist-packages", "/usr/local/lib/python3/dist-packages"]
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node

GOTO, RTH, NAV_CANCEL, TAKEOFF, LAND = 23, 22, 28, 12, 13
GROUND_Z = 0.05
truth = {"x": 0.0, "y": 0.0, "z": 0.0, "vx": 0.0, "vy": 0.0}
fc = {}
results = {}
stream = {"on": True, "pitch": 0.0}
takeoff = {"x": 0.0, "y": 0.0}      # where it really was when last armed: its "home" and "north, east" for a trip start from there
lock = threading.Lock()
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


def control(thr, r=0.0, p=0.0, y=0.0):
    post("/control", f"{thr},{r},{p},{y}\n")


def pilot():
    while True:
        if stream["on"]:
            with lock:
                control(2.5, 0.0, stream["pitch"], 0.0)
        time.sleep(0.04)


def code(c, r=0.0, p=0.0, y=0.0):
    with lock:
        for _ in range(3):
            control(c, r, p, y)


def fault(name):
    post("/fault", name)


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


def from_home():
    return math.hypot(truth["x"] - takeoff["x"], truth["y"] - takeoff["y"])


def speed():
    return math.hypot(truth["vx"], truth["vy"])


def watch(secs, done=lambda: False):
    """For this long, or until done: the furthest it got from where it started, and the top speed."""
    p0 = (truth["x"], truth["y"])
    far = top = 0.0
    until = time.monotonic() + secs
    while time.monotonic() < until and not done():
        far = max(far, math.hypot(truth["x"] - p0[0], truth["y"] - p0[1]))
        top = max(top, speed())
        time.sleep(0.05)
    return far, top


def next_trip_is_right(what):
    """Whatever the last fault left behind: the next trip must go where it is told, at the normal speed."""
    fly_again()
    n0, e0 = truth["y"] - takeoff["y"], truth["x"] - takeoff["x"]
    code(GOTO, n0 + 10, e0, 4.0)
    wait(lambda: fc.get("navKind") == 1, 3, "the next trip to start")
    _, top = watch(40, lambda: fc.get("navKind") == 0)
    time.sleep(2)
    miss = math.hypot(truth["y"] - takeoff["y"] - n0 - 10, truth["x"] - takeoff["x"] - e0)
    check(f"after {what}: the next trip goes where it is told", miss < 0.5 and top < 6 and fc.get("stateName") == "FLYING", f"{miss:.2f} m off, top speed {top:.1f} m/s, state {fc.get('stateName')}, reason {fc.get('reasonName')}")


def fly_again():
    """Back to a healthy flight over the start, whatever the last test left."""
    fault("none")
    stream.update(on=True, pitch=0.0)
    time.sleep(1)
    if fc.get("stateName") in ("LANDING", "FAILSAFE"):
        wait(lambda: fc.get("stateName") == "DISARMED", 60, "it to finish landing")
    if fc.get("stateName") == "DISARMED":
        wait(lambda: height() < 0.3, 20, "it to reach the ground")
        time.sleep(4)
        code(TAKEOFF)
        wait(lambda: fc.get("stateName") == "FLYING", 25, "it to take off again")
        time.sleep(4)
    elif fc.get("stateName") == "AUTO":
        code(NAV_CANCEL)
        wait(lambda: fc.get("stateName") == "FLYING", 3, "the trip to stop")
    wait(lambda: fc.get("posStateName") == "HOLDING", 25, "position hold")


def go(n, e, h):
    code(GOTO, n, e, h)
    wait(lambda: fc.get("navKind") == 1, 3, "the trip to start")
    wait(lambda: fc.get("navKind") == 0 and fc.get("stateName") == "FLYING", 60, "the trip to end")
    time.sleep(2)


def link_loss_at_each_stage(home):
    for stage_no, name in ((1, "CLIMB"), (2, "TURN"), (3, "CRUISE"), (4, "DESCEND"), (5, "HOVER")):
        fly_again()
        go(20, 0, 1.5 if stage_no == 1 else 5.0)
        code(RTH)
        wait(lambda: fc.get("navKind") == 3 and fc.get("navStage") >= min(stage_no, 5), 40, f"stage {name}")
        stream["on"] = False
        wait(lambda: fc.get("linkLost"), 4, "the loss to be noticed")
        wait(lambda: fc.get("navStage") == 5, 60, "the hover at home")
        time.sleep(5)
        d = math.hypot(truth["x"] - home[0], truth["y"] - home[1])
        check(f"RC lost during {name}: it carries on home", d < 0.5 and abs(height() - 2) < 0.2 and fc.get("stateName") == "AUTO", f"{d:.3f} m from home, height {height():.2f} m")
        stream["on"] = True
        time.sleep(1)


def gps_freeze_in_cruise():
    fly_again()
    go(30, 0, 4.0)
    code(RTH)
    wait(lambda: fc.get("navStage") == 3, 20, "the cruise")
    time.sleep(3)
    p0 = (truth["x"], truth["y"])
    fault("gps_freeze")
    t = wait(lambda: fc.get("stateName") == "FAILSAFE", 6, "it to give up navigating")
    check("GPS frozen during the cruise: stops navigating, says why", fc.get("reasonName") == "no navigation", f"after {t:.2f} s, reason {fc.get('reasonName')}")
    wait(lambda: fc.get("stateName") == "DISARMED", 40, "it to land")
    d = math.hypot(truth["x"] - p0[0], truth["y"] - p0[1])
    check("and lands where it is, upright", fc.get("reasonName") != "CRASH" and d < 25, f"{d:.1f} m from where it happened, reason {fc.get('reasonName')}")


def gps_jump_hover():
    fly_again()
    p0 = (truth["x"], truth["y"])
    jumps = fc.get("gpsJumps", 0)
    far = 0.0
    tilt = 0.0

    def stay(secs):
        nonlocal far, tilt
        until = time.monotonic() + secs
        while time.monotonic() < until:
            far = max(far, math.hypot(truth["x"] - p0[0], truth["y"] - p0[1]))
            tilt = max(tilt, abs(fc.get("roll", 0.0)), abs(fc.get("pitch", 0.0)))
            time.sleep(0.05)

    fault("gps_jump")
    stay(10)
    state = fc.get("stateName")
    check("GPS jumps 50 m: the drone stays where it is", far < 0.5 and tilt < 10 and state == "FLYING", f"moved at most {far:.2f} m, tilt {tilt:.1f} deg, state {state}, {fc.get('gpsJumps', 0) - jumps} jump seen")
    fault("none")
    stay(6)
    check("GPS jumps back: it still stays where it is", far < 0.5 and tilt < 10 and fc.get("stateName") == "FLYING", f"moved at most {far:.2f} m, tilt {tilt:.1f} deg, {fc.get('gpsJumps', 0) - jumps} jumps seen")


def gps_jump_on_the_way_home():
    fly_again()
    go(25, 0, 5.0)
    code(RTH)
    wait(lambda: fc.get("navStage") == 3, 20, "the cruise")
    time.sleep(2)
    fault("gps_jump")
    wait(lambda: fc.get("navStage") == 5 or fc.get("stateName") != "AUTO", 40, "the hover at home")
    time.sleep(3)
    d = from_home()
    check("GPS jumps 50 m on the way home: it still gets there", d < 0.5 and fc.get("stateName") == "AUTO", f"{d:.3f} m from home, state {fc.get('stateName')}, height {height():.2f} m")
    fault("none")
    time.sleep(5)
    d = from_home()
    check("and stays there when the GPS jumps back", d < 0.5 and fc.get("stateName") == "AUTO", f"{d:.3f} m from home, state {fc.get('stateName')}")


def compass(which):
    """A fault during the flight home. It must get home at the normal speed, or land saying why; it never gets away."""
    fly_again()
    go(25, 0, 4.0)
    code(RTH)
    wait(lambda: fc.get("navStage") == 3, 20, "the cruise")
    time.sleep(1)
    start = from_home()
    fault(which)
    far_home = start
    top = 0.0
    until = time.monotonic() + 60
    while time.monotonic() < until and fc.get("navStage") != 5 and fc.get("stateName") != "DISARMED":
        far_home = max(far_home, from_home())
        top = max(top, speed())
        time.sleep(0.05)
    time.sleep(3)
    state = fc.get("stateName")
    d = from_home()
    home_ok = state == "AUTO" and d < 0.5
    landed_ok = state == "DISARMED" and fc.get("reasonName") == "no navigation"
    check(f"compass fault {which} on the way home: gets home, or lands and says why", (home_ok or landed_ok) and far_home <= start + 2 and top < 6,
          f"{start:.1f} m from home when it happened, at most {far_home:.1f} m, {d:.2f} m at the end, top speed {top:.1f} m/s, state {state}, reason {fc.get('reasonName')}, compass used {fc.get('compass')}")
    fault("none")
    next_trip_is_right(f"compass fault {which}")


def compass_turns_wrong():
    """Something magnetic nearby: it reads like a compass but turns further and further wrong, and then comes right all at once."""
    fly_again()
    p0 = (truth["x"], truth["y"])
    n0, e0 = truth["y"] - takeoff["y"], truth["x"] - takeoff["x"]
    code(GOTO, n0 + 40, e0, 4.0)
    wait(lambda: fc.get("navStage") == 3, 20, "the cruise")
    fault("mag_turn")
    _, top = watch(18, lambda: fc.get("stateName") == "DISARMED")
    fault("none")
    far, top2 = watch(25, lambda: fc.get("stateName") == "DISARMED" or fc.get("navKind") == 0)
    d = math.hypot(truth["x"] - p0[0], truth["y"] - p0[1])
    check("compass turning wrong, then right again: it never gets away", d < 60 and max(top, top2) < 8 and fc.get("reasonName") != "CRASH",
          f"{d:.1f} m from where the 40 m trip started, top speed {max(top, top2):.1f} m/s, state {fc.get('stateName')}, reason {fc.get('reasonName')}, heading offset {fc.get('yawOffset', 0):.0f} deg")
    next_trip_is_right("a compass turning wrong")


def baro_freeze_hover():
    fly_again()
    z0 = truth["z"]
    fault("baro_freeze")
    worst = 0.0
    until = time.monotonic() + 15
    while time.monotonic() < until:
        worst = max(worst, abs(truth["z"] - z0))
        time.sleep(0.05)
    fault("none")
    check("barometer frozen for 15 s while hovering: holds its height", worst < 0.5, f"moved at most {worst:.2f} m")


def gyro_bias_hover():
    fly_again()
    p0 = (truth["x"], truth["y"])
    fault("gyro_bias")
    far = 0.0
    tilt = 0.0
    until = time.monotonic() + 20
    while time.monotonic() < until:
        far = max(far, math.hypot(truth["x"] - p0[0], truth["y"] - p0[1]))
        tilt = max(tilt, abs(fc.get("roll", 0.0)))
        time.sleep(0.05)
    state = fc.get("stateName")
    fault("none")
    check("gyro drift 3 deg/s switched on in flight: recovers", far < 3 and state == "FLYING", f"moved at most {far:.2f} m, state {state}")


def imu_freeze_hover():
    fly_again()
    z0 = truth["z"]
    fault("imu_freeze")
    try:
        t = wait(lambda: fc.get("stateName") == "DISARMED", 6, "the motors to stop")
        check("IMU stuck in flight (the world goes on): motors stop, says why", fc.get("reasonName") == "sensors", f"after {t:.2f} s, reason {fc.get('reasonName')}, height {height():.2f} m")
    except TimeoutError:
        check("IMU stuck in flight (the world goes on): motors stop, says why", False, f"still {fc.get('stateName')} after 6 s, height {height():.2f} m, was {z0 - GROUND_Z:.2f} m")
    fault("none")


def faults_are_in_the_log():
    """Phase 10: what happened must be there to read afterwards, in the log bridge_plus writes."""
    name = max(glob.glob(os.path.expanduser("~/hil_test_r4/logs/aerofc_status_*.csv")), key=os.path.getmtime)
    with open(name) as f:
        rows = list(csv.DictReader(f))
    why = {r["reason"] for r in rows}
    no_compass = sum(1 for r in rows if r["compass"] == "0" and r["armed"] == "1")
    jumps = max(int(r["gps_jumps"]) for r in rows)
    lost = sum(1 for r in rows if r["link_lost"] == "1")
    check("Phase 10: the faults are in the flight log", {"no navigation", "sensors"} <= why and no_compass > 0 and jumps >= 2 and lost > 0,
          f"{len(rows)} rows: reasons {sorted(why - {'-'})}, {no_compass} rows flying with no compass, {jumps} GPS jumps, {lost} rows with the RC link lost")


def garbage():
    fly_again()
    p0 = (truth["x"], truth["y"], truth["z"])
    before = fc.get("crcErrors", 0)
    far = 0.0
    for _ in range(40):
        post("/garbage", "120")
        far = max(far, math.hypot(truth["x"] - p0[0], truth["y"] - p0[1]), abs(truth["z"] - p0[2]))
        time.sleep(0.2)
    time.sleep(1)
    check("damaged bytes on the line for 8 s: thrown away, flight not disturbed", far < 0.3 and fc.get("stateName") == "FLYING", f"moved at most {far:.3f} m, {fc.get('crcErrors', 0) - before} packets failed their check, state {fc.get('stateName')}")


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else None
    threading.Thread(target=telemetry, daemon=True).start()
    threading.Thread(target=pilot, daemon=True).start()
    until = time.monotonic() + 120
    while fc.get("stateName") != "FLYING" and time.monotonic() < until:
        time.sleep(0.2)
    time.sleep(4)
    home = (truth["x"], truth["y"])
    for name, stage in (("garbage", garbage), ("barometer", baro_freeze_hover), ("gyro", gyro_bias_hover), ("link loss", lambda: link_loss_at_each_stage(home)),
                        ("gps freeze", gps_freeze_in_cruise), ("gps jump", gps_jump_hover), ("gps jump on the way home", gps_jump_on_the_way_home),
                        ("compass stuck", lambda: compass("mag_freeze")), ("compass zero", lambda: compass("mag_zero")), ("compass turning wrong", compass_turns_wrong),
                        ("imu", imu_freeze_hover)):
        print(f"=== {name} {time.strftime('%X')}", flush=True)
        try:
            stage()
        except TimeoutError as e:
            check(f"{name}: ran to the end", False, f"timed out waiting for {e}")
    fault("none")
    faults_are_in_the_log()
    bad = [k for k, v in results.items() if not v["pass"]]
    print(f"{len(results) - len(bad)} of {len(results)} pass", flush=True)
    if out:
        json.dump({"date": time.strftime("%Y-%m-%d %H:%M"), "results": results}, open(out, "w"), indent=1)


main()
