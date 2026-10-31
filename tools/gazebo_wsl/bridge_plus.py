#!/usr/bin/env python3
"""bridge_plus.py - runs the OFFICIAL hil_bridge_v1.py unchanged, plus two small additions for practice at home:

  * STATUS (0x25) telemetry from the flight controller is decoded, logged to CSV and streamed to the browser
    ground station (Server-Sent Events) for the live plots the problem statement asks for
  * the browser ground station can send pilot input (keyboard / gamepad) with HTTP POST /control

The official file is read, ONE line is hooked (where it ignores non-motor packets) and then executed, so every
other behaviour (handshake, 50 Hz sensors, failsafe, motor scaling, CSV log) is exactly the organisers' code.

    cd ~/hil_test/arduino_hil
    python3 bridge_plus.py --port /dev/ttyACM0          # real Arduino
    python3 bridge_plus.py                              # fake / socat link (fc_sim), like the official bridge
Then in the simulator page (Windows): More connections -> Relay http://127.0.0.1:8770 -> "Gazebo in WSL".
"""
import json
import os
import queue
import re
import struct
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
OFFICIAL = os.path.join(HERE, "hil_bridge_v1.py")
HTTP_PORT = int(os.environ.get("AEROFC_HTTP_PORT", "8770"))
STATES = ["DISARMED", "IDLE", "TAKEOFF", "FLYING", "LANDING", "FAILSAFE"]
REASONS = ["-", "user", "pilot link", "sensors", "estimator", "CRASH", "STOP", "KILL", "idle", "landed", "geofence"]
SPEED_MODES = ["ECO", "NORMAL", "SPORT"]
POS_STATES = ["OFF", "NOT READY", "MOVING", "BRAKING", "HOLDING", "FAILED"]
CSV_HEADER = ("time,state,mode,armed,rc_thr,rc_roll,rc_pitch,rc_yaw,roll,pitch,yaw,alt,alt_target,vz,m1,m2,m3,m4,"
              "speed_mode,pos_state,pos_n,pos_e,vel_n,vel_e,hold_dist,failsafe,reason,heading_offset\n")

clients = []
clients_lock = threading.Lock()
bridge_globals = {}
csv_file = None
pilot_sent = 0         # pilot commands passed on to the flight controller since the last report
pilot_refused = 0      # pilot commands that arrived before the flight controller was connected
send_error_shown = False
delays_ms = []         # how long the flight controller took to answer each sensor burst, since the last report


def name(table, i):
    return table[i] if i < len(table) else "?"


def decode_status(p):
    """STATUS v2 (64 bytes, firmware 2.x) and v1 (48 bytes, firmware 1.x) - same fields as simulator/js/protocol.js"""
    if len(p) < 48 or p[0] not in (1, 2):
        return None
    i16 = lambda o: struct.unpack_from("<h", p, o)[0]
    u16 = lambda o: struct.unpack_from("<H", p, o)[0]
    f32 = lambda o: struct.unpack_from("<f", p, o)[0]
    f = p[3]
    s = {
        "state": p[1], "stateName": name(STATES, p[1]), "mode": p[2],
        "modeName": "MANUAL" if p[2] == 1 else "ALT HOLD",
        "armed": bool(f & 1), "airborne": bool(f & 2), "compass": bool(f & 4), "headingLock": bool(f & 8),
        "failsafe": bool(f & 16), "crashed": bool(f & 32), "pilotLink": bool(f & 64), "sensorsOk": bool(f & 128),
        "roll": i16(4) / 100, "pitch": i16(6) / 100, "yaw": u16(8) / 100,
    }
    if p[0] == 2 and len(p) >= 64:
        f2 = p[50]
        s.update({
            "version": 2,
            "alt": f32(10), "altTarget": f32(14), "vz": i16(18) / 100,
            "rcRoll": i16(20) / 1000, "rcPitch": i16(22) / 1000, "rcYaw": i16(24) / 1000, "rcThr": i16(26) / 1000,
            "motors": [p[28] / 250, p[29] / 250, p[30] / 250, p[31] / 250],
            "rollSp": i16(32) / 100, "pitchSp": i16(34) / 100, "headingTarget": u16(36) / 100,
            "loopUs": u16(38), "imuHz": p[40], "reason": p[41], "reasonName": name(REASONS, p[41]),
            "crcErrors": u16(42), "hoverCmd": u16(44) / 10000, "collective": i16(46) / 10000, "pilotHz": p[48],
            "calKind": p[49] >> 4, "calProgress": (p[49] & 15) / 15,
            "posActive": bool(f2 & 1), "posReady": bool(f2 & 2), "posOn": bool(f2 & 4), "altLimit": bool(f2 & 8),
            "atCeiling": bool(f2 & 16), "linkLost": bool(f2 & 32), "gpsAid": bool(f2 & 64), "stickAngle": bool(f2 & 128),
            "posState": p[51] & 15, "posStateName": name(POS_STATES, p[51] & 15),
            "speedMode": p[51] >> 4, "speedModeName": name(SPEED_MODES, p[51] >> 4),
            "posN": i16(52) / 10, "posE": i16(54) / 10, "velN": i16(56) / 100, "velE": i16(58) / 100,
            "holdDist": u16(60) / 100, "maxAlt": u16(62), "yawOffset": i16(64) / 100 if len(p) >= 66 else 0.0,
        })
        return s
    if p[0] != 1:
        return None
    s.update({
        "version": 1,
        "alt": i16(10) / 100, "altTarget": i16(12) / 100, "vz": i16(14) / 100,
        "rcRoll": i16(16) / 1000, "rcPitch": i16(18) / 1000, "rcYaw": i16(20) / 1000, "rcThr": i16(22) / 1000,
        "motors": [p[24] / 250, p[25] / 250, p[26] / 250, p[27] / 250],
        "rollSp": i16(28) / 100, "pitchSp": i16(30) / 100, "headingTarget": u16(32) / 100,
        "loopUs": u16(34), "imuHz": p[36], "reason": p[37], "reasonName": name(REASONS, p[37]),
        "crcErrors": u16(38), "hoverCmd": u16(40) / 10000, "baroAlt": i16(42) / 100, "pilotHz": p[44],
        "calKind": p[45] >> 4, "calProgress": (p[45] & 15) / 15, "collective": i16(46) / 10000,
        "posActive": False, "posReady": False, "posOn": False, "posState": 0, "posStateName": "-",
        "speedMode": 1, "speedModeName": "-", "posN": 0, "posE": 0, "velN": 0, "velE": 0, "holdDist": 0, "maxAlt": 0,
    })
    return s


def on_status(payload):
    s = decode_status(bytes(payload))
    if not s:
        return
    if csv_file:
        csv_file.write("%.3f,%s,%s,%d,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%s,%s,%.1f,%.1f,%.2f,%.2f,%.2f,%d,%s,%.2f\n" % (
            time.time(), s["stateName"], s["modeName"], s["armed"], s["rcThr"], s["rcRoll"], s["rcPitch"], s["rcYaw"],
            s["roll"], s["pitch"], s["yaw"], s["alt"], s["altTarget"], s["vz"], ",".join("%.3f" % m for m in s["motors"]),
            s["speedModeName"], s["posStateName"], s["posN"], s["posE"], s["velN"], s["velE"], s["holdDist"],
            s["failsafe"], s["reasonName"], s.get("yawOffset", 0.0)))
    msg = json.dumps(s)
    with clients_lock:
        for q in list(clients):
            if q.qsize() < 50:
                q.put(msg)


def on_motors(decoded):
    """A motor packet echoes the time stamp of the sensor packet it answers, so this is the real control delay."""
    delays_ms.append((time.time_ns() - decoded["timestamp"]) / 1e6)


def delay_report():
    """The drone is steady up to about 80 ms of control delay, so show how far away from that this link is."""
    while True:
        time.sleep(15.0)
        d = sorted(delays_ms)
        delays_ms.clear()
        if d:
            warn = "  ABOVE THE 80 ms LIMIT: close other programs" if d[-1] > 80.0 else ""
            print(f"[bridge_plus] control delay: typical {d[len(d) // 2]:.0f} ms, worst {d[-1]:.0f} ms, "
                  f"{len(d) / 15.0:.0f} motor packets per second{warn}", flush=True)


def pilot_report():
    """Says in this window whether the page's commands arrive, so a silent link can't go unnoticed during a flight."""
    global pilot_sent, pilot_refused
    last = None
    last_rate = 0
    silent = 0.0
    while True:
        time.sleep(2.0)
        sent, refused, pilot_sent, pilot_refused = pilot_sent, pilot_refused, 0, 0
        rate = round(sent / 2.0)
        if rate > 40:
            state, text = "two", f"{rate} pilot commands per second: two pilots? Close the other simulator page."
        elif sent:
            state, text = "ok", f"pilot commands from the page: OK, about {rate} per second"
        elif refused:
            state, text = "early", "the page is sending, but the flight controller is not connected yet"
        elif bridge_globals.get("running"):
            state, text = "none", ("NO pilot commands from the page. It must be in Gazebo mode: "
                                   "launcher 3 or 4 opens it so, or press More connections > Gazebo in WSL (relay).")
        else:
            continue
        silent = silent + 2.0 if state == "none" else 0.0
        if state != last or silent >= 30.0 or abs(rate - last_rate) > 5:
            print(f"[bridge_plus] {text}", flush=True)
            last = state
            last_rate = rate
            silent = 0.0


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def do_OPTIONS(self):
        self.send_response(204)
        self.cors()
        self.end_headers()

    def do_GET(self):
        if self.path.startswith("/events"):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.cors()
            self.end_headers()
            q = queue.Queue()
            with clients_lock:
                clients.append(q)
            try:
                while True:
                    try:
                        msg = q.get(timeout=1.0)
                        self.wfile.write(("data: " + msg + "\n\n").encode())
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                with clients_lock:
                    clients.remove(q)
            return
        self.send_response(200)
        self.cors()
        self.end_headers()
        self.wfile.write(b"AeroFC bridge_plus running\n")

    def do_POST(self):
        global pilot_sent, pilot_refused, send_error_shown
        n = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(n).decode(errors="replace").strip()
        ok = False
        if self.path.startswith("/control") and bridge_globals.get("running"):
            try:
                thr, roll, pitch, yaw = [float(v) for v in body.split(",")[:4]]
                g = bridge_globals
                g["send_control_packet"](g["CONTROL_INPUT"], g["pack_control_input"](thr, roll, pitch, yaw))
                ok = True
                pilot_sent += 1
            except Exception as err:
                ok = False
                if not send_error_shown:
                    send_error_shown = True
                    print(f"[bridge_plus] could not pass a pilot command on: {err}", flush=True)
        elif self.path.startswith("/control"):
            pilot_refused += 1
        self.send_response(200 if ok else 409)
        self.cors()
        self.end_headers()


def main():
    global csv_file
    src = open(OFFICIAL, encoding="utf-8").read()
    pattern = re.compile(r'^([ \t]*)if decoded\["type"\] != ACTUATOR_COMMAND:\n', re.M)
    m = pattern.search(src)
    if not m:
        sys.exit("hil_bridge_v1.py has changed - hook not found. Run the official bridge instead.")
    ind = m.group(1)
    src = (src[:m.start()] + f'{ind}if decoded["type"] == 0x25:\n{ind}    _aerofc_status(decoded["payload"])\n'
           + f'{ind}elif decoded["type"] == ACTUATOR_COMMAND:\n{ind}    _aerofc_motors(decoded)\n' + src[m.start():])

    os.makedirs(os.path.expanduser("~/hil_test/logs"), exist_ok=True)
    name = os.path.expanduser("~/hil_test/logs/aerofc_status_%s.csv" % datetime.now().strftime("%Y%m%d_%H%M%S"))
    csv_file = open(name, "w", buffering=1)
    csv_file.write(CSV_HEADER)
    print(f"[bridge_plus] STATUS telemetry log: {name}", flush=True)

    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[bridge_plus] ground station link on http://127.0.0.1:{HTTP_PORT}  (/events, POST /control)", flush=True)
    threading.Thread(target=pilot_report, daemon=True).start()
    threading.Thread(target=delay_report, daemon=True).start()

    sys.path.insert(0, HERE)
    bridge_globals.update({"__name__": "__main__", "__file__": OFFICIAL, "_aerofc_status": on_status, "_aerofc_motors": on_motors})
    sys.argv = [OFFICIAL] + sys.argv[1:]
    exec(compile(src, OFFICIAL, "exec"), bridge_globals)


if __name__ == "__main__":
    main()
