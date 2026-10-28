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
Then in the simulator page (Windows): More connections -> Relay http://localhost:8770 -> "Gazebo in WSL".
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

clients = []
clients_lock = threading.Lock()
bridge_globals = {}
csv_file = None


def decode_status(p):
    if len(p) < 48 or p[0] != 1:
        return None
    i16 = lambda o: struct.unpack_from("<h", p, o)[0]
    u16 = lambda o: struct.unpack_from("<H", p, o)[0]
    f = p[3]
    return {
        "state": p[1], "stateName": STATES[p[1]] if p[1] < len(STATES) else "?", "mode": p[2],
        "modeName": "MANUAL" if p[2] == 1 else "ALT HOLD",
        "armed": bool(f & 1), "airborne": bool(f & 2), "compass": bool(f & 4), "headingLock": bool(f & 8),
        "failsafe": bool(f & 16), "crashed": bool(f & 32), "pilotLink": bool(f & 64), "sensorsOk": bool(f & 128),
        "roll": i16(4) / 100, "pitch": i16(6) / 100, "yaw": u16(8) / 100,
        "alt": i16(10) / 100, "altTarget": i16(12) / 100, "vz": i16(14) / 100,
        "rcRoll": i16(16) / 1000, "rcPitch": i16(18) / 1000, "rcYaw": i16(20) / 1000, "rcThr": i16(22) / 1000,
        "motors": [p[24] / 250, p[25] / 250, p[26] / 250, p[27] / 250],
        "rollSp": i16(28) / 100, "pitchSp": i16(30) / 100, "headingTarget": u16(32) / 100,
        "loopUs": u16(34), "imuHz": p[36], "reason": p[37], "reasonName": REASONS[p[37]] if p[37] < len(REASONS) else "?",
        "crcErrors": u16(38), "hoverCmd": u16(40) / 10000, "baroAlt": i16(42) / 100, "pilotHz": p[44],
        "calKind": p[45] >> 4, "calProgress": (p[45] & 15) / 15, "collective": i16(46) / 10000,
    }


def on_status(payload):
    s = decode_status(bytes(payload))
    if not s:
        return
    if csv_file:
        csv_file.write("%.3f,%s,%s,%d,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n" % (
            time.time(), s["stateName"], s["modeName"], s["armed"], s["rcThr"], s["rcRoll"], s["rcPitch"], s["rcYaw"],
            s["roll"], s["pitch"], s["yaw"], s["alt"], s["altTarget"], s["vz"], ",".join("%.3f" % m for m in s["motors"])))
    msg = json.dumps(s)
    with clients_lock:
        for q in list(clients):
            if q.qsize() < 50:
                q.put(msg)


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
        n = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(n).decode(errors="replace").strip()
        ok = False
        if self.path.startswith("/control") and bridge_globals.get("running"):
            try:
                thr, roll, pitch, yaw = [float(v) for v in body.split(",")[:4]]
                g = bridge_globals
                g["send_control_packet"](g["CONTROL_INPUT"], g["pack_control_input"](thr, roll, pitch, yaw))
                ok = True
            except Exception:
                ok = False
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
    src = src[:m.start()] + f'{ind}if decoded["type"] == 0x25:\n{ind}    _aerofc_status(decoded["payload"])\n' + src[m.start():]

    os.makedirs(os.path.expanduser("~/hil_test/logs"), exist_ok=True)
    name = os.path.expanduser("~/hil_test/logs/aerofc_status_%s.csv" % datetime.now().strftime("%Y%m%d_%H%M%S"))
    csv_file = open(name, "w", buffering=1)
    csv_file.write("time,state,mode,armed,rc_thr,rc_roll,rc_pitch,rc_yaw,roll,pitch,yaw,alt,alt_target,vz,m1,m2,m3,m4\n")
    print(f"[bridge_plus] STATUS telemetry log: {name}", flush=True)

    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[bridge_plus] ground station link on http://localhost:{HTTP_PORT}  (/events, POST /control)", flush=True)

    sys.path.insert(0, HERE)
    bridge_globals.update({"__name__": "__main__", "__file__": OFFICIAL, "_aerofc_status": on_status})
    sys.argv = [OFFICIAL] + sys.argv[1:]
    exec(compile(src, OFFICIAL, "exec"), bridge_globals)


if __name__ == "__main__":
    main()
