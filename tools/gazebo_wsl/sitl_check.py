#!/usr/bin/env python3
"""sitl_check.py - automatic flight test in the REAL Gazebo (run it while run_sitl.sh or run_hil.sh is running).

It reads the flight controller's STATUS telemetry from bridge_plus.py (http://localhost:8770/events) and flies with
pilot packets (POST /control, 25 per second like the simulator page):

  1. auto take-off to 1.5 m, hover with position hold
  2. forward with the stick for 3 s, let go -> must brake and then hold its spot
  3. climb with full stick: with the height limit ON (Max alt 20) it must stop at 20 m
  4. RC link lost (pilot packets stop) -> failsafe must land on the spot and disarm (reason "pilot link")

    ./run_sitl.sh -- --set "Alt limit=1" --set "Max alt=20"      (window 1)
    python3 sitl_check.py                                          (window 2)
    python3 sitl_check.py --long 10                                adds 10 real minutes of flying around after step 2:
                                                                   afterwards it must still hold its spot
"""
import argparse
import json
import math
import os
import random
import sys
import threading
import time
import urllib.request

URL = "http://localhost:" + os.environ.get("AEROFC_HTTP_PORT", "8770")
status = {}
status_lock = threading.Lock()
pilot = {"thr": 2.5, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
stream_on = threading.Event()
results = []


def events():
    while True:
        try:
            with urllib.request.urlopen(URL + "/events", timeout=5) as r:
                for raw in r:
                    line = raw.decode(errors="replace").strip()
                    if line.startswith("data: "):
                        s = json.loads(line[6:])
                        with status_lock:
                            status.clear()
                            status.update(s)
                            status["_t"] = time.time()
        except Exception:
            time.sleep(0.5)


def post(thr, roll, pitch, yaw):
    body = ("%.3f,%.3f,%.3f,%.3f\n" % (thr, roll, pitch, yaw)).encode()
    try:
        urllib.request.urlopen(urllib.request.Request(URL + "/control", data=body, method="POST"), timeout=1).read()
    except Exception:
        pass


def pilot_stream():
    while True:
        if stream_on.is_set():
            post(pilot["thr"], pilot["roll"], pilot["pitch"], pilot["yaw"])
        time.sleep(0.04)


def st():
    with status_lock:
        return dict(status)


def wait(cond, timeout, every=None):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = st()
        if every:
            every(s)
        if s and cond(s):
            return time.time() - t0
        time.sleep(0.05)
    return None


def run_for(seconds, every):
    t0 = time.time()
    while time.time() - t0 < seconds:
        every(st())
        time.sleep(0.05)


def long_flight(minutes):
    """Keeps moving the sticks like a pilot: hands off, flying around, turning."""
    rnd = random.Random(1)
    end = time.time() + minutes * 60
    worst_offset = [0.0]
    started = time.time()
    next_note = started + 60
    while time.time() < end:
        a = rnd.random()
        if a < 0.45:
            pilot.update(pitch=0.0, roll=0.0, yaw=0.0)
            duration = 3 + rnd.random() * 12
        elif a < 0.65:
            pilot.update(pitch=(rnd.random() - 0.5) * 2, roll=(rnd.random() - 0.5) * 2, yaw=0.0)
            duration = 1 + rnd.random() * 5
        elif a < 0.85:
            pilot.update(pitch=rnd.random() - 0.5, roll=rnd.random() - 0.5, yaw=(rnd.random() - 0.5) * 1.5)
            duration = 1 + rnd.random() * 6
        else:
            pilot.update(pitch=0.0, roll=0.0, yaw=(rnd.random() - 0.5) * 2)
            duration = 1 + rnd.random() * 4
        run_for(min(duration, max(0.0, end - time.time())),
                lambda s: worst_offset.__setitem__(0, max(worst_offset[0], abs(s.get("yawOffset", 0.0)))))
        if time.time() > next_note:
            next_note += 60
            s = st()
            print("  %.0f min: %s, position %.0f / %.0f m, heading offset %.1f deg" % (
                (time.time() - started) / 60, s.get("posStateName"), s.get("posN", 0), s.get("posE", 0), s.get("yawOffset", 0)), flush=True)
    pilot.update(pitch=0.0, roll=0.0, yaw=0.0)
    return worst_offset[0]


def check(ok, text, value=""):
    results.append((bool(ok), text, value))
    print("%s  %s  %s" % ("PASS" if ok else "FAIL", text, value), flush=True)


def main():
    ap = argparse.ArgumentParser(description="Automatic flight test in the real Gazebo")
    ap.add_argument("--long", type=float, default=0.0, metavar="MIN", help="fly around for this many minutes after step 2")
    args = ap.parse_args()
    threading.Thread(target=events, daemon=True).start()
    threading.Thread(target=pilot_stream, daemon=True).start()
    print("waiting for STATUS telemetry from bridge_plus.py ...", flush=True)
    if wait(lambda s: s.get("version") == 2, 60) is None:
        sys.exit("no STATUS v2 telemetry on %s/events - is run_sitl.sh (or run_hil.sh) running?" % URL)
    # a simulator page connected to the same bridge is a second pilot: its keys and its packets would spoil every check
    time.sleep(3)
    if st().get("pilotHz", 0) > 3:
        sys.exit("Another pilot is sending packets (%d per second): close the simulator page first." % st().get("pilotHz", 0))
    stream_on.set()

    # 1. take-off and hover
    t = wait(lambda s: s["stateName"] == "FLYING" and abs(s["alt"] - 1.5) < 0.15, 30)
    check(t is not None, "auto take-off to 1.5 m", "%.1f s" % (t or -1))
    time.sleep(3)
    s0 = st()
    check(s0.get("posActive"), "position hold active (GPS + compass ready)", s0.get("posStateName"))
    worst = [0.0]
    run_for(6, lambda s: worst.__setitem__(0, max(worst[0], s.get("holdDist", 9))))
    check(worst[0] < 0.3, "hover: stays within 0.3 m of its spot", "%.2f m" % worst[0])

    # 2. forward and release
    pilot["pitch"] = -0.6
    vmax = [0.0]
    run_for(3, lambda s: vmax.__setitem__(0, max(vmax[0], math.hypot(s.get("velN", 0), s.get("velE", 0)))))
    pilot["pitch"] = 0.0
    p_rel = st()
    check(vmax[0] > 2.0, "forward stick (0.6 = 3 m/s in NORMAL) moves it", "%.2f m/s" % vmax[0])
    t = wait(lambda s: s.get("posStateName") == "HOLDING", 8)
    check(t is not None, "let go: brakes and switches to HOLDING", "%.1f s" % (t or -1))
    time.sleep(1.0)
    s1 = st()
    worst = [0.0]
    run_for(6, lambda s: worst.__setitem__(0, max(worst[0], math.hypot(s["posN"] - s1["posN"], s["posE"] - s1["posE"]))))
    check(worst[0] < 0.3, "then holds its spot (no drift)", "%.2f m, stopped %.1f m after release" %
          (worst[0], math.hypot(s1["posN"] - p_rel["posN"], s1["posE"] - p_rel["posE"])))

    # 2b. long flight: it must not get worse with time
    if args.long > 0:
        print("flying around for %.0f minutes ..." % args.long, flush=True)
        offset = long_flight(args.long)
        t = wait(lambda s: s.get("posStateName") == "HOLDING", 15)
        time.sleep(3.0)
        s1 = st()
        worst = [0.0]
        run_for(20, lambda s: worst.__setitem__(0, max(worst[0], math.hypot(s["posN"] - s1["posN"], s["posE"] - s1["posE"]))))
        check(t is not None and worst[0] < 0.5, "after %.0f min of flying it still holds its spot" % args.long, "%.2f m" % worst[0])
        check(offset < 5.0, "heading offset stayed small during the long flight", "worst %.1f deg" % offset)
        check(st().get("stateName") == "FLYING" and st().get("posActive"), "still flying with position hold active")

    # 3. height limit
    s = st()
    if s.get("altLimit"):
        pilot["thr"] = 3.0                       # full climb stick
        peak = [0.0]
        run_for(16, lambda s: peak.__setitem__(0, max(peak[0], s.get("alt", 0))))
        s = st()
        check(abs(peak[0] - s["maxAlt"]) < 0.6 and s.get("atCeiling"),
              "height limit %d m: stops there with full climb stick" % s["maxAlt"], "highest %.2f m" % peak[0])
        pilot["thr"] = 2.5
        time.sleep(2)
    else:
        print("(height limit OFF: start run_sitl.sh with -- --set \"Alt limit=1\" --set \"Max alt=20\" to test it)")

    # 4. RC link lost
    p_cut = st()
    stream_on.clear()
    t_fs = wait(lambda s: s.get("failsafe"), 6)
    check(t_fs is not None, "RC link cut: failsafe starts", "after %.1f s" % (t_fs or -1))
    t_land = wait(lambda s: not s.get("armed"), 90)
    s = st()
    d = math.hypot(s["posN"] - p_cut["posN"], s["posE"] - p_cut["posE"])
    check(t_land is not None and s.get("reasonName") == "pilot link" and d < 1.0,
          "failsafe lands on the spot and disarms (reason pilot link)",
          "%.1f s from %.1f m, %.2f m from where the link was lost" % (t_land or -1, p_cut.get("alt", 0), d))

    n = sum(1 for r in results if r[0])
    print("\n%d / %d checks passed in the real Gazebo" % (n, len(results)), flush=True)
    sys.exit(0 if n == len(results) else 1)


if __name__ == "__main__":
    main()
