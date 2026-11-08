"""serve.py - the simulator's local web server (like `python -m http.server`, but the browser never keeps old copies:
after you rebuild the firmware or update the files, a page reload always gets the new ones).

It also reads the flying keys for the page while a Gazebo window is the active window (Windows only). A web page gets keys
only after it has been clicked, and this way nobody has to click it. Only which flying keys are down goes to the page, at
/keys. Nothing is stored, and with any other program active nothing is read.

    python tools/serve.py [port] [url]    (START_SIMULATOR.bat does this; default port 8790; url = page to open)
"""
import functools
import http.server
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import threading
import time
import webbrowser

# Windows key code -> the name simulator/js/pilot_input.js uses for it (e.key in lower case).  Not Space: Gazebo's window also
# gets the key, and there Space pauses the simulation, so KILL is Backspace in that window.
KEY_NAMES = {
    0x57: "w", 0x41: "a", 0x53: "s", 0x44: "d", 0x51: "q", 0x45: "e", 0x52: "r", 0x46: "f",
    0x26: "arrowup", 0x28: "arrowdown", 0x25: "arrowleft", 0x27: "arrowright",
    0x08: "backspace", 0x0D: "enter", 0x54: "t", 0x4C: "l", 0x4D: "m", 0x31: "1", 0x32: "2", 0x33: "3",
    0x47: "g", 0x50: "p", 0x4B: "k", 0x10: "shift", 0x48: "h",
}
# Gazebo's 3D view (Windows shows Ubuntu's windows through msrdc.exe) and the launchers' black windows.
# The title alone isn't enough: an editor or File Explorer can have "gazebo" in its title too.
GAZEBO_PROGRAMS = {"msrdc.exe", "conhost.exe", "openconsole.exe", "windowsterminal.exe", "cmd.exe"}
READ_S = 0.02            # how often the keys are read
HEARTBEAT_S = 0.25       # a message at least this often, so the page notices if this server stops


def is_gazebo_window(title, program):
    return "gazebo" in title.lower() and program.lower() in GAZEBO_PROGRAMS


class KeyWatcher(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        import ctypes
        from ctypes import wintypes
        self.ctypes = ctypes
        self.user32 = ctypes.WinDLL("user32")
        self.kernel32 = ctypes.WinDLL("kernel32")
        self.user32.GetForegroundWindow.restype = wintypes.HWND
        self.user32.GetWindowTextW.argtypes = (wintypes.HWND, wintypes.LPWSTR, ctypes.c_int)
        self.user32.GetWindowThreadProcessId.argtypes = (wintypes.HWND, ctypes.POINTER(wintypes.DWORD))
        self.user32.GetAsyncKeyState.restype = ctypes.c_short
        self.kernel32.OpenProcess.restype = wintypes.HANDLE
        self.kernel32.OpenProcess.argtypes = (wintypes.DWORD, wintypes.BOOL, wintypes.DWORD)
        self.kernel32.QueryFullProcessImageNameW.argtypes = (wintypes.HANDLE, wintypes.DWORD, wintypes.LPWSTR, ctypes.POINTER(wintypes.DWORD))
        self.kernel32.CloseHandle.argtypes = (wintypes.HANDLE,)
        self.DWORD = wintypes.DWORD
        self.changed = threading.Condition()
        self.message = json.dumps({"active": False, "keys": []})

    def active_window(self):
        """The active window's title and the file name of its program."""
        ctypes = self.ctypes
        hwnd = self.user32.GetForegroundWindow()
        if not hwnd:
            return "", ""
        title = ctypes.create_unicode_buffer(256)
        self.user32.GetWindowTextW(hwnd, title, 256)
        pid = self.DWORD(0)
        self.user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
        process = self.kernel32.OpenProcess(0x1000, False, pid.value)     # 0x1000: may only ask for its name
        if not process:
            return title.value, ""
        path = ctypes.create_unicode_buffer(600)
        size = self.DWORD(600)
        ok = self.kernel32.QueryFullProcessImageNameW(process, 0, path, ctypes.byref(size))
        self.kernel32.CloseHandle(process)
        return title.value, path.value.rsplit("\\", 1)[-1] if ok else ""

    def read(self):
        active = is_gazebo_window(*self.active_window())
        keys = sorted(n for k, n in KEY_NAMES.items() if active and self.user32.GetAsyncKeyState(k) & 0x8000)
        return json.dumps({"active": active, "keys": keys})

    def run(self):
        while True:
            message = self.read()
            with self.changed:
                if message != self.message:
                    self.message = message
                    self.changed.notify_all()
            time.sleep(READ_S)


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    watcher = None

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/keys" and self.watcher:
            self.stream_keys()
        else:
            super().do_GET()

    def stream_keys(self):
        w = self.watcher
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.end_headers()
        sent = None
        try:
            while True:
                with w.changed:
                    w.changed.wait_for(lambda: w.message != sent, timeout=HEARTBEAT_S)
                    sent = w.message
                self.wfile.write(f"data: {sent}\n\n".encode())
                self.wfile.flush()
        except OSError:                  # the page went away
            self.close_connection = True


class Server(http.server.ThreadingHTTPServer):
    allow_reuse_address = False      # on Windows "reuse" would let a second copy share the port


def open_page(url):
    """Edge or Chrome is needed for "Connect Arduino" (Web Serial), so Edge first.  The page gets its own window and its own
    settings folder, because these switches only work on a separate Edge: normally Edge slows a page down once it is covered or
    minimised, and a key that has been let go would keep flying the drone for a second or more."""
    if not url:
        return
    folder = pathlib.Path(os.environ.get("LOCALAPPDATA", tempfile.gettempdir())) / "AeroFC_Edge_R4"
    switches = "--no-first-run --no-default-browser-check --disable-background-timer-throttling " \
               "--disable-renderer-backgrounding --disable-backgrounding-occluded-windows"
    if subprocess.call(f'start "" msedge --app="{url}" --user-data-dir="{folder}" {switches}', shell=True, stderr=subprocess.DEVNULL) != 0:
        webbrowser.open(url)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8790
    url = sys.argv[2] if len(sys.argv) > 2 else ""
    root = pathlib.Path(__file__).resolve().parent.parent
    handler = functools.partial(NoCacheHandler, directory=str(root))
    try:
        srv = Server(("127.0.0.1", port), handler)
    except OSError:
        print(f"Port {port} is busy: the simulator server is probably already running (that is fine).", flush=True)
        open_page(url)
        return
    if sys.platform == "win32":
        NoCacheHandler.watcher = KeyWatcher()
        NoCacheHandler.watcher.start()
    with srv:
        print(f"AeroFC simulator on http://localhost:{port}/simulator/index.html  (close this window to stop)", flush=True)
        open_page(url)      # the server already listens here, so the page can't arrive too early
        srv.serve_forever()


if __name__ == "__main__":
    main()
