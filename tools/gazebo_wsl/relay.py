#!/usr/bin/env python3
"""relay.py - lets the browser ground station (keyboard / gamepad on Windows) fly the drone in Gazebo when the
OFFICIAL, unmodified hil_bridge_v1.py is running.  It receives pilot lines over HTTP and writes them into the
bridge's control pipe /tmp/hil_control_input, exactly like the official control_input.py does.

    python3 relay.py            (in the Ubuntu window, while hil_bridge_v1.py runs)
Then in the simulator page: More connections -> Relay http://127.0.0.1:8770 -> "Gazebo in WSL".
(Live plots need bridge_plus.py instead of the official bridge; relay.py only carries the pilot input.)
"""
import errno
import os
import stat
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PIPE = "/tmp/hil_control_input"
PORT = int(os.environ.get("AEROFC_HTTP_PORT", "8770"))
lock = threading.Lock()
fd = None
fd_ino = None


def write_line(line):
    """Writes one line into the FIFO; reopens it when the bridge re-created it.  Never blocks."""
    global fd, fd_ino
    with lock:
        try:
            st = os.stat(PIPE)
            if not stat.S_ISFIFO(st.st_mode):
                return False
            if fd is not None and fd_ino != st.st_ino:
                os.close(fd)
                fd = None
            if fd is None:
                fd = os.open(PIPE, os.O_WRONLY | os.O_NONBLOCK)   # fails with ENXIO if the bridge is not reading
                fd_ino = st.st_ino
            os.write(fd, line.encode())
            return True
        except OSError as e:
            if fd is not None and e.errno in (errno.EPIPE, errno.EBADF):
                try:
                    os.close(fd)
                except OSError:
                    pass
                fd = None
            return False


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
        if self.path.startswith("/events"):   # no telemetry here: keep the browser's EventSource quiet
            self.send_response(204)
            self.cors()
            self.end_headers()
            return
        self.send_response(200)
        self.cors()
        self.end_headers()
        self.wfile.write(b"AeroFC relay running\n")

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(n).decode(errors="replace").strip()
        parts = body.split(",")
        ok = False
        if self.path.startswith("/control") and len(parts) >= 4:
            try:
                vals = [float(v) for v in parts[:4]]
                ok = write_line("%.4f,%.4f,%.4f,%.4f\n" % tuple(vals))
            except ValueError:
                ok = False
        self.send_response(200 if ok else 409)
        self.cors()
        self.end_headers()


if __name__ == "__main__":
    print(f"AeroFC relay: http://127.0.0.1:{PORT}/control  ->  {PIPE}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
