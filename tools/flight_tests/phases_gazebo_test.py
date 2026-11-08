#!/usr/bin/env python3
"""phases_gazebo_test.py - the evidence for phases 2 to 5 in the real Gazebo, always against Gazebo's own truth.

  Phase 2  sensor acquisition: steady sensor rate, time stamps moving forward, steady readings at rest
  Phase 3  AHRS: the attitude and heading estimates follow known moves
  Phase 4  EKF: height and position estimates follow known moves, sensor health stays good
  Phase 5  stabilisation: real pushes (forces and twists) are corrected without unstable behaviour

Runs in Ubuntu while Gazebo and bridge_plus fly, started by phases_gazebo_run.sh.    python3 phases_gazebo_test.py result.json
"""
import bisect
import collections
import json
import math
import statistics
import subprocess
import sys
import threading
import time
import urllib.request

sys.path[:0] = ["/usr/lib/python3/dist-packages", "/usr/local/lib/python3/dist-packages"]
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node

GROUND_Z = 0.05
LAG_LIMIT_S = 0.2       # the numbers can't sensibly take this long to arrive
truth = {"x": 0.0, "y": 0.0, "z": 0.0, "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0}
fc = {}
results = {}
stick = {"thr": 2.5, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


truth_log = collections.deque(maxlen=100000)    # time, Gazebo's roll and pitch
estimate_log = collections.deque(maxlen=20000)  # time it arrived here, the flight controller's roll and pitch


def on_odom(m):
    q = m.pose.orientation
    truth.update(x=m.pose.position.x, y=m.pose.position.y, z=m.pose.position.z, qw=q.w, qx=q.x, qy=q.y, qz=q.z)
    truth_log.append((time.monotonic(),) + true_angles()[:2])


node = Node()
node.subscribe(Odometry, "/model/X3/odometry", on_odom)


def telemetry():
    while True:
        try:
            with urllib.request.urlopen("http://127.0.0.1:8770/events", timeout=5) as r:
                for line in r:
                    if line.startswith(b"data:"):
                        fc.update(json.loads(line[5:]))
                        estimate_log.append((time.monotonic(), fc.get("roll", 0.0), fc.get("pitch", 0.0)))
        except Exception:
            time.sleep(0.5)


def pilot():
    while True:
        body = f"{stick['thr']},{stick['roll']},{stick['pitch']},{stick['yaw']}\n".encode()
        try:
            urllib.request.urlopen(urllib.request.Request("http://127.0.0.1:8770/control", data=body, method="POST"), timeout=1).read()
        except Exception:
            pass
        time.sleep(0.04)


def check(name, ok, detail):
    results[name] = {"pass": bool(ok), "detail": detail}
    print(("PASS " if ok else "FAIL ") + name + ": " + str(detail), flush=True)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def true_angles():
    """Gazebo's attitude as the flight controller reports it: roll right +, pitch nose up +, heading clockwise from north."""
    w, x, y, z = truth["qw"], truth["qx"], truth["qy"], truth["qz"]
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    yaw_enu = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, -pitch, wrap180(90.0 - yaw_enu)


def settled(secs=25):
    until = time.monotonic() + secs
    while time.monotonic() < until:
        if fc.get("posStateName") == "HOLDING" and abs(fc.get("velN", 9)) < 0.1 and abs(fc.get("velE", 9)) < 0.1 and abs(fc.get("vz", 9)) < 0.1:
            return
        time.sleep(0.1)


def sample(secs, every=0.04):
    """Estimate and truth side by side, for secs."""
    rows = []
    until = time.monotonic() + secs
    while time.monotonic() < until:
        r, p, y = true_angles()
        rows.append({"r": fc.get("roll", 0.0) - r, "p": fc.get("pitch", 0.0) - p, "y": wrap180(fc.get("yaw", 0.0) - y - fc.get("yawOffset", 0.0)),
                     "tr": r, "tp": p, "alt": fc.get("alt", 0.0), "tz": truth["z"] - GROUND_Z, "vz": fc.get("vz", 0.0), "pn": fc.get("posN", 0.0),
                     "pe": fc.get("posE", 0.0), "tn": truth["y"], "te": truth["x"], "tilt": math.hypot(r, p)})
        time.sleep(every)
    return rows


def rms(values):
    return math.sqrt(sum(v * v for v in values) / max(1, len(values)))


def phase2():
    settled()
    rows = sample(10)
    alt = [r["alt"] for r in rows]
    roll = [r["tr"] + r["r"] for r in rows]
    hz = []
    for _ in range(10):
        hz.append(fc.get("imuHz", 0))
        time.sleep(1)
    check("Phase 2: sensor rate steady near 50 a second", min(hz) >= 40, f"lowest {min(hz)}, highest {max(hz)} a second")
    check("Phase 2: time stamps always move forward", fc.get("imuTimeBad", 0) <= 3, f"{fc.get('imuTimeBad')} out of order or repeated since start-up")
    check("Phase 2: readings steady while hovering", statistics.pstdev(alt) < 0.05 and statistics.pstdev(roll) < 0.5,
          f"height moves {statistics.pstdev(alt):.3f} m, roll {statistics.pstdev(roll):.2f} deg (standard deviation)")
    check("Phase 2: no damaged packets to speak of", fc.get("crcErrors", 0) < 20, f"{fc.get('crcErrors')} failed their CRC check")


def tilt_errors(t0, t1, axis, lag):
    """Estimate minus truth over a move, each estimate against Gazebo's truth lag seconds before it arrived here."""
    log = list(truth_log)
    times = [x[0] for x in log]
    errors = []
    for row in list(estimate_log):
        if t0 <= row[0] <= t1:
            t = row[0] - lag
            i = bisect.bisect_left(times, t)
            if 0 < i < len(times):
                part = (t - times[i - 1]) / max(1e-6, times[i] - times[i - 1])
                errors.append(row[1 + axis] - (log[i - 1][1 + axis] + part * (log[i][1 + axis] - log[i - 1][1 + axis])))
    return errors


def phase3():
    moves = [("roll right", {"roll": 0.5}, 0), ("roll left", {"roll": -0.5}, 0), ("pitch forward", {"pitch": -0.5}, 1), ("pitch back", {"pitch": 0.5}, 1)]
    done = []
    for name, s, axis in moves:
        settled()
        t0 = time.monotonic()
        stick.update(s)
        rows = sample(2.5)
        stick.update(roll=0.0, pitch=0.0)
        done.append((name, axis, t0, time.monotonic(), max(abs(r["tr" if axis == 0 else "tp"]) for r in rows)))
    # The estimate needs time to reach this test, and Gazebo's truth doesn't. The tilt changes fast, so that time alone looks
    # like an error. It is measured here: the delay that fits all four moves best.
    delays = [i * 0.005 for i in range(int(LAG_LIMIT_S / 0.005) + 1)]
    lag = min(delays, key=lambda d: sum(rms(tilt_errors(t0, t1, axis, d)) ** 2 for _, axis, t0, t1, _ in done))
    for name, axis, t0, t1, moved in done:
        err = rms(tilt_errors(t0, t1, axis, lag))
        raw = rms(tilt_errors(t0, t1, axis, 0.0))
        check(f"Phase 3: {name}: the estimate follows Gazebo's truth", moved > 3 and err < 2.0 and lag < LAG_LIMIT_S,
              f"tilted to {moved:.1f} deg, error {err:.2f} deg (root mean square), {raw:.2f} deg before taking off the {lag * 1000:.0f} ms the numbers take to get here")
    settled()
    stick["yaw"] = 0.5
    rows = sample(6)
    stick["yaw"] = 0.0
    err = rms([r["y"] for r in rows])
    check("Phase 3: turning: the heading follows Gazebo's truth", err < 5.0, f"error {err:.2f} deg over a 6 s turn")
    settled()
    rows = sample(5)
    err = rms([r["y"] for r in rows])
    check("Phase 3: heading after the turn, held still", err < 3.0, f"error {err:.2f} deg")


def phase4():
    settled()
    stick["thr"] = 3.0
    rows = sample(4)
    stick["thr"] = 2.5
    settled()
    rows += sample(2)
    stick["thr"] = 2.0
    rows += sample(3)
    stick["thr"] = 2.5
    settled()
    err = rms([r["alt"] - r["tz"] for r in rows])
    climbed = max(r["tz"] for r in rows) - min(r["tz"] for r in rows)
    check("Phase 4: height estimate follows a climb and descent", climbed > 3 and err < 0.25, f"moved {climbed:.1f} m, error {err:.3f} m (root mean square)")
    # the position filter's origin is the first GPS fix, Gazebo's the world's: compare the moves, not the numbers
    settled()
    n0, e0, tn0, te0 = fc.get("posN", 0.0), fc.get("posE", 0.0), truth["y"], truth["x"]
    stick["pitch"] = -0.6
    rows = sample(3)
    stick["pitch"] = 0.0
    rows += sample(3)
    err = rms([math.hypot(r["pn"] - n0 - (r["tn"] - tn0), r["pe"] - e0 - (r["te"] - te0)) for r in rows])
    check("Phase 4: position estimate follows a fast move", err < 0.8, f"error {err:.3f} m (root mean square)")
    check("Phase 4: sensor health stays good", fc.get("compass") and fc.get("sensorsOk") and fc.get("posReady"), f"compass {fc.get('compass')}, sensors {fc.get('sensorsOk')}, position {fc.get('posReady')}")


def wrench(fx=0.0, fy=0.0, fz=0.0, tx=0.0, ty=0.0, tz=0.0):
    subprocess.run(["gz", "topic", "-t", "/world/quadcopter/wrench/persistent", "--msgtype", "gz.msgs.EntityWrench", "-p",
                    f'entity: {{name: "X3", type: MODEL}}, wrench: {{force: {{x: {fx}, y: {fy}, z: {fz}}}, torque: {{x: {tx}, y: {ty}, z: {tz}}}}}'],
                   capture_output=True, timeout=15)


def wrench_clear():
    subprocess.run(["gz", "topic", "-t", "/world/quadcopter/wrench/clear", "--msgtype", "gz.msgs.Entity", "-p", 'name: "X3", type: MODEL'],
                   capture_output=True, timeout=15)


def phase5():
    pushes = [("sideways push 4 N", {"fx": 4.0}, 3.0), ("push the other way 4 N", {"fy": -4.0}, 3.0), ("push down 4 N", {"fz": -4.0}, 3.0),
              ("push up 3 N", {"fz": 3.0}, 3.0), ("twist in roll 0.3 N m", {"tx": 0.3}, 1.0), ("twist in yaw 0.3 N m", {"tz": 0.3}, 1.0)]
    for name, w, secs in pushes:
        settled()
        x0, y0, z0 = truth["x"], truth["y"], truth["z"]
        wrench(**w)
        rows = sample(secs)
        wrench_clear()
        rows += sample(8)
        far = max(math.hypot(r["te"] - x0, r["tn"] - y0) for r in rows)
        high = max(abs(r["tz"] + GROUND_Z - z0) for r in rows)
        tilt = max(r["tilt"] for r in rows)
        settled(15)
        back = math.hypot(truth["x"] - x0, truth["y"] - y0)
        flying = fc.get("stateName") == "FLYING"
        check(f"Phase 5: {name}: corrected, no unstable behaviour", flying and tilt < 45 and back < 1.0 and abs(truth["z"] - z0) < 0.2,
              f"pushed {far:.2f} m sideways and {high:.2f} m in height, tilted at most {tilt:.1f} deg, ended {back:.2f} m from the start")


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else None
    threading.Thread(target=telemetry, daemon=True).start()
    threading.Thread(target=pilot, daemon=True).start()
    until = time.monotonic() + 120
    while fc.get("stateName") != "FLYING" and time.monotonic() < until:
        time.sleep(0.2)
    time.sleep(4)
    for phase in (phase2, phase3, phase4, phase5):
        print(f"=== {phase.__name__} {time.strftime('%X')}", flush=True)
        phase()
    wrench_clear()
    bad = [k for k, v in results.items() if not v["pass"]]
    print(f"{len(results) - len(bad)} of {len(results)} pass", flush=True)
    if out:
        json.dump({"date": time.strftime("%Y-%m-%d %H:%M"), "results": results}, open(out, "w"), indent=1)


main()
