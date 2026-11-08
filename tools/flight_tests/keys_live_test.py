"""keys_live_test.py - the keys with the real launcher, the real Gazebo window and the real Edge page, never clicking the page.

    python tools/flight_tests/keys_live_test.py sitl [result.json]     launcher 4: no board
    python tools/flight_tests/keys_live_test.py board [result.json]    launcher 3: the real R4 Minima (it must be on Windows, not attached)

It presses keys with Windows' keybd_event and reads the result from the flight controller's own telemetry (bridge_plus, port
8770) and from Gazebo's true position.  Before every key it checks the active window is still the expected one, so if someone
starts using the PC it stops instead of typing into their window.
"""
import ctypes
import json
import pathlib
import subprocess
import sys
import threading
import time
import urllib.request
from ctypes import wintypes

ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
user32 = ctypes.WinDLL("user32")
kernel32 = ctypes.WinDLL("kernel32")
user32.GetForegroundWindow.restype = wintypes.HWND
user32.SetForegroundWindow.argtypes = (wintypes.HWND,)
user32.ShowWindow.argtypes = (wintypes.HWND, ctypes.c_int)
user32.IsWindowVisible.argtypes = (wintypes.HWND,)
user32.GetWindowTextW.argtypes = (wintypes.HWND, wintypes.LPWSTR, ctypes.c_int)
user32.GetWindowThreadProcessId.argtypes = (wintypes.HWND, ctypes.POINTER(wintypes.DWORD))
user32.GetWindowRect.argtypes = (wintypes.HWND, ctypes.POINTER(wintypes.RECT))
user32.MoveWindow.argtypes = (wintypes.HWND, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, wintypes.BOOL)
kernel32.OpenProcess.restype = wintypes.HANDLE
kernel32.OpenProcess.argtypes = (wintypes.DWORD, wintypes.BOOL, wintypes.DWORD)
kernel32.QueryFullProcessImageNameW.argtypes = (wintypes.HANDLE, wintypes.DWORD, wintypes.LPWSTR, ctypes.POINTER(wintypes.DWORD))
kernel32.CloseHandle.argtypes = (wintypes.HANDLE,)
user32.keybd_event.argtypes = (wintypes.BYTE, wintypes.BYTE, wintypes.DWORD, ctypes.c_void_p)
ENUM = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
VK = {"w": 0x57, "a": 0x41, "s": 0x53, "d": 0x44, "t": 0x54, "l": 0x4C, "p": 0x50, " ": 0x20, "2": 0x32, "1": 0x31, "backspace": 0x08,
      "q": 0x51, "e": 0x45, "r": 0x52, "f": 0x46, "g": 0x47, "k": 0x4B, "h": 0x48}
VK_ALT, KEYUP, SW_MINIMIZE, SW_RESTORE, SW_MAXIMIZE = 0x12, 2, 6, 9, 3

status = {}
status_at = [0.0]
results = {}
trace = None        # optional record of the flight: python keys_live_test.py sitl result.json trace.csv
gz_partition = ""


class Stop(Exception):
    pass


def windows():
    found = []

    def one(hwnd, _):
        if not user32.IsWindowVisible(hwnd):
            return True
        t = ctypes.create_unicode_buffer(256)
        user32.GetWindowTextW(hwnd, t, 256)
        pid = wintypes.DWORD(0)
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
        p = kernel32.OpenProcess(0x1000, False, pid.value)
        name = ""
        if p:
            b = ctypes.create_unicode_buffer(600)
            s = wintypes.DWORD(600)
            if kernel32.QueryFullProcessImageNameW(p, 0, b, ctypes.byref(s)):
                name = b.value.rsplit("\\", 1)[-1]
            kernel32.CloseHandle(p)
        r = wintypes.RECT()
        user32.GetWindowRect(hwnd, ctypes.byref(r))
        found.append({"hwnd": hwnd, "title": t.value, "program": name, "area": (r.right - r.left) * (r.bottom - r.top)})
        return True

    user32.EnumWindows(ENUM(one), 0)
    return found


def find(program, word):
    w = [x for x in windows() if x["program"].lower() == program and word.lower() in x["title"].lower()]
    return max(w, key=lambda x: x["area"])["hwnd"] if w else None


def activate(hwnd):
    for _ in range(5):
        user32.keybd_event(VK_ALT, 0, 0, None)        # Windows lets a program change the active window only after a key press
        user32.ShowWindow(hwnd, SW_RESTORE)
        user32.SetForegroundWindow(hwnd)
        user32.keybd_event(VK_ALT, 0, KEYUP, None)
        time.sleep(0.5)
        if user32.GetForegroundWindow() == hwnd:
            return
    raise Stop("could not make that window active")


def active_title():
    h = user32.GetForegroundWindow()
    w = [x for x in windows() if x["hwnd"] == h]
    return f"{w[0]['program']}: {w[0]['title']}" if w else "no window"


def press(key, hold, window):
    if user32.GetForegroundWindow() != window:
        raise Stop(f"the active window changed to {active_title()}: stopping so no key goes into it")
    user32.keybd_event(VK[key], 0, 0, None)
    time.sleep(hold)
    user32.keybd_event(VK[key], 0, KEYUP, None)


def mark(text):
    if trace:
        trace.write(f"# {time.monotonic():.2f} {text}\n")
        trace.flush()


def telemetry():
    while True:
        try:
            with urllib.request.urlopen("http://127.0.0.1:8770/events", timeout=5) as r:
                for line in r:
                    if line.startswith(b"data:"):
                        status.update(json.loads(line[5:]))
                        status_at[0] = time.monotonic()
                        if trace:
                            trace.write("%.2f,%s,%.2f,%.2f,%.1f,%.1f,%.2f,%.2f,%s,%d,%.1f,%.2f\n" % (
                                status_at[0], status.get("stateName"), status.get("rcRoll", 0), status.get("rcPitch", 0), status.get("posN", 0),
                                status.get("posE", 0), status.get("velN", 0), status.get("velE", 0), status.get("posStateName"),
                                status.get("pilotHz", 0), status.get("yaw", 0), status.get("holdDist", 0)))
                            trace.flush()
        except Exception:
            time.sleep(0.5)


def wsl(cmd, timeout=30):
    try:
        r = subprocess.run(["wsl.exe", "-d", "Ubuntu-22.04", "--", "bash", "-lc", gz_partition + cmd], capture_output=True, timeout=timeout,
                           creationflags=subprocess.CREATE_NO_WINDOW)     # a console window would take the focus
    except subprocess.TimeoutExpired:
        raise Stop(f"no answer from Ubuntu to: {cmd[:50]}")
    return r.stdout.decode(errors="replace").replace("\0", "")


def truth():
    """Gazebo's own position of the drone (x, y, z in metres)."""
    t = wsl("gz topic -e -n 1 -t /model/X3/odometry 2>/dev/null | grep -A3 -m1 position")
    v = {}
    for line in t.splitlines():
        p = line.strip().split(":")
        if len(p) == 2 and p[0] in "xyz":
            v[p[0]] = float(p[1])
    return (v.get("x", 0.0), v.get("y", 0.0), v.get("z", 0.0))


def moved(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def wait_for(test, secs, what):
    until = time.monotonic() + secs
    while time.monotonic() < until:
        if status and test(status):
            return
        time.sleep(0.1)
    print("windows:", [(w["program"], w["title"]) for w in windows() if w["title"]], flush=True)
    raise Stop(f"timed out waiting for {what}; last: { {k: status.get(k) for k in ('stateName', 'pilotHz', 'alt', 'posStateName')} }")


def extreme(key, secs):
    """The largest value of a telemetry field over the next secs, and the lowest command rate."""
    best = 0.0
    hz = 255
    until = time.monotonic() + secs
    while time.monotonic() < until:
        v = status.get(key, 0.0)
        if abs(v) > abs(best):
            best = v
        hz = min(hz, status.get("pilotHz", 0))
        time.sleep(0.03)
    return best, hz


def check(name, ok, detail):
    results[name] = {"pass": bool(ok), "detail": detail}
    print(("PASS " if ok else "FAIL ") + name + ": " + str(detail), flush=True)


def fly_with_key(key, stick, gazebo, hold=3.0):
    """Holds a key in the window.  Returns (largest stick seen, metres moved in Gazebo, s until the stick centred after letting go,
    commands a second while it was held)."""
    wait_for(lambda s: s.get("posStateName") == "HOLDING", 15, "position hold")
    a = truth()
    got = []
    t = threading.Thread(target=lambda: got.append(extreme(stick, hold)))
    t.start()
    mark(f"{key} down")
    press(key, hold, gazebo)
    mark(f"{key} up")
    up = time.monotonic()
    while abs(status.get(stick, 0.0)) > 0.02 and time.monotonic() - up < 10:
        time.sleep(0.02)
    released = time.monotonic() - up
    t.join()
    wait_for(lambda s: s.get("posStateName") == "HOLDING", 15, "position hold after the key")
    b = truth()
    mark(f"{key} done, Gazebo {a} -> {b}")
    return got[0][0], moved(a, b), released, got[0][1]


def check_key(name, res, want_sign):
    stick, dist, released, hz = res
    ok = stick * want_sign > 0.3 and dist > 1.0 and released < 0.5 and hz >= 10
    check(name, ok, f"stick {stick:.2f}, moved {dist:.2f} m, centred {released:.2f} s after letting go, {hz} commands a second")


def settled():
    """Position hold has really stopped the drone: a test of "nothing moves" must not catch it still arriving."""
    good = 0
    until = time.monotonic() + 60
    while time.monotonic() < until:
        fine = status.get("posStateName") == "HOLDING" and status.get("holdDist", 9) < 0.4 and abs(status.get("velN", 9)) < 0.2 and abs(status.get("velE", 9)) < 0.2
        good = good + 1 if fine else 0
        if good >= 10:
            return
        time.sleep(0.1)
    print("note: the drone was still moving slowly after 60 s", {k: status.get(k) for k in ("posStateName", "holdDist", "velN", "velE")}, flush=True)


def paused():
    return "paused: true" in wsl("gz topic -e -n 1 -t /stats 2>/dev/null | grep paused")


def check_toggle(name, window):
    """Four presses of P: each must switch position hold exactly once."""
    states = [status["posOn"]]
    for _ in range(4):
        press("p", 0.15, window)
        time.sleep(1.2)
        states.append(status["posOn"])
    check(name, all(a != b for a, b in zip(states, states[1:])), states)


def run_tests(kind):
    global gz_partition
    if kind == "board":
        gz_partition = "export GZ_PARTITION=hil_test_r4; "      # run_hil.sh runs Gazebo in this partition
    launcher = "3_FLY_ARDUINO_in_Gazebo.bat" if kind == "board" else "4_FLY_SOFTWARE_FC_in_Gazebo.bat"
    subprocess.run(["cmd", "/c", "start", "", str(ROOT / "gazebo" / launcher)], check=True)
    threading.Thread(target=telemetry, daemon=True).start()
    wait_for(lambda s: s.get("stateName") == "FLYING" and s.get("pilotHz", 0) >= 10, 120, "the drone flying, with the page sending")
    check("page sends with nobody clicking it", True, f"{status['pilotHz']} commands a second, height {status['alt']:.2f} m")
    time.sleep(3)
    gz = None
    for _ in range(30):
        gz = find("msrdc.exe", "gazebo")
        if gz:
            break
        time.sleep(1)
    print("windows named Gazebo or AeroFC:", [(w["program"], w["title"]) for w in windows() if "gazebo" in w["title"].lower() or "aerofc" in w["title"].lower()], flush=True)
    if not gz:
        raise Stop("the Gazebo window was not found")
    edge = find("msedge.exe", "aerofc")

    activate(gz)
    check_key("Gazebo active: W flies forward, Edge never clicked", fly_with_key("w", "rcPitch", gz), -1)

    if edge:
        user32.ShowWindow(edge, SW_MINIMIZE)
        time.sleep(2)
        activate(gz)
        check_key("Edge minimised: D flies right", fly_with_key("d", "rcRoll", gz), 1)
        check_toggle("Edge minimised: P switches exactly once per press", gz)
        user32.ShowWindow(edge, SW_RESTORE)
        time.sleep(1)
    else:
        check("Edge minimised: D flies right", False, "the Edge window was not found")

    # Edge fully behind a normal-size Gazebo window (maximising Gazebo slows it down, see rtf_matrix.py)
    if edge:
        r = wintypes.RECT()
        user32.GetWindowRect(gz, ctypes.byref(r))
        user32.MoveWindow(edge, r.left + 60, r.top + 60, 400, 300, True)
        time.sleep(1)
    activate(gz)
    time.sleep(3)
    check_key("Edge behind Gazebo: S flies back", fly_with_key("s", "rcPitch", gz), 1)
    check_toggle("Gazebo active: P switches exactly once per press", gz)
    press("1", 0.2, gz)
    time.sleep(1.0)
    eco = status["speedMode"]
    press("2", 0.2, gz)
    time.sleep(1.0)
    check("Gazebo active: 1 and 2 change the speed mode", eco == 0 and status["speedMode"] == 1, f"{eco}, then {status['speedMode']}")

    other = subprocess.Popen([sys.executable, "-I", "-c", "import tkinter as t; r = t.Tk(); r.title('KEYTEST'); r.geometry('300x100'); r.mainloop()"])
    time.sleep(2)
    w = next((x["hwnd"] for x in windows() if x["title"] == "KEYTEST"), None)
    if not w:
        other.kill()
        raise Stop("the other window was not found")
    settled()
    activate(w)
    a = truth()
    got = []
    t = threading.Thread(target=lambda: got.append(extreme("rcPitch", 3.0)))
    t.start()
    press("s", 3.0, w)
    t.join()
    dist = moved(a, truth())
    check("another program active: S doesn't reach the drone", abs(got[0][0]) < 0.05, f"stick {got[0][0]:.2f} (moved {dist:.2f} m while settling)")
    other.kill()
    time.sleep(1)

    if edge:
        activate(edge)
        time.sleep(1)
        check_toggle("Edge active: P switches exactly once per press", edge)
        settled()
        check_key("Edge active: W flies forward, as it always did", fly_with_key("w", "rcPitch", edge), -1)

    # the server stops while a key is held: nobody can see it go up, so the stick must centre
    activate(gz)
    wait_for(lambda s: s.get("posStateName") == "HOLDING", 15, "position hold")
    if user32.GetForegroundWindow() != gz:
        raise Stop(f"the active window changed to {active_title()}: stopping so no key goes into it")
    user32.keybd_event(VK["w"], 0, 0, None)
    time.sleep(1.5)
    during = status.get("rcPitch", 0.0)
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    "Get-CimInstance Win32_Process -Filter \"Name like 'python%'\" | Where-Object { $_.CommandLine -match 'serve\\.py 8790' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }"],
                   capture_output=True, creationflags=subprocess.CREATE_NO_WINDOW)
    t0 = time.monotonic()
    centred = None
    while time.monotonic() - t0 < 5:
        if abs(status.get("rcPitch", 0.0)) < 0.02:
            centred = time.monotonic() - t0
            break
        time.sleep(0.03)
    user32.keybd_event(VK["w"], 0, KEYUP, None)
    check("server stopped with W held: the stick centres", during < -0.3 and centred is not None, f"stick {during:.2f}, centred after {centred if centred is None else round(centred, 2)} s")
    subprocess.Popen([sys.executable, "-I", str(ROOT / "tools" / "serve.py"), "8790"], cwd=ROOT, creationflags=subprocess.CREATE_NO_WINDOW)
    time.sleep(6)                     # the page reconnects by itself
    activate(gz)
    check_key("server back: keys work again by themselves", fly_with_key("a", "rcRoll", gz), -1)

    for k in ("q", "e", "r", "f", "g", "g", "g", "k", "k"):     # none of the other flying keys may upset Gazebo either
        press(k, 0.2, gz)
        time.sleep(0.5)
    check("Gazebo active: Q, E, R, F, G, K don't pause Gazebo", not paused(), "running")
    settled()

    press("h", 0.2, gz)
    wait_for(lambda s: s.get("navKind") == 3, 3, "H to start the flight home")
    check("Gazebo active: H starts the flight home", True, f"{status.get('stateName')} {status.get('navKindName')} {status.get('navStageName')}")
    time.sleep(2)
    press("w", 0.6, gz)
    wait_for(lambda s: s.get("navKind") == 0 and s.get("stateName") == "FLYING", 3, "a key to take over")
    check("and a flying key takes over", True, status.get("stateName"))
    settled()

    press("l", 0.2, gz)
    wait_for(lambda s: s.get("stateName") == "DISARMED", 40, "landing")
    check("Gazebo active: L lands", status.get("reasonName") == "landed", status.get("reasonName"))
    time.sleep(2)
    press("t", 0.2, gz)
    wait_for(lambda s: s.get("stateName") == "FLYING" and s.get("alt", 0) > 0.0, 30, "take-off")
    time.sleep(3)
    press("backspace", 0.2, gz)
    wait_for(lambda s: s.get("stateName") == "DISARMED", 5, "KILL")
    check("Gazebo active: T takes off, Backspace stops the motors", status.get("reasonName") == "KILL", status.get("reasonName"))
    check("Backspace doesn't pause Gazebo", not paused(), "running")
    # a fact worth knowing, not a thing we control: Gazebo's own Space key pauses the simulation.  Afterwards it runs again.
    press(" ", 0.2, gz)
    time.sleep(1)
    now_paused = paused()
    press(" ", 0.2, gz)
    time.sleep(1)
    print(f"Gazebo's own Space key: paused {now_paused}, pressed again: paused {paused()}", flush=True)


def stop_page():
    """Only the page's own Edge (its own settings folder, see tools/serve.py), never the user's Edge, and the page's web server."""
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    "Get-CimInstance Win32_Process | Where-Object { ($_.Name -eq 'msedge.exe' -and $_.CommandLine -match 'AeroFC_Edge_R4' -and $_.CommandLine -notmatch '--type=')"
                    " -or ($_.Name -like 'python*' -and $_.CommandLine -match 'serve\\.py 8790') } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }"],
                   capture_output=True, creationflags=subprocess.CREATE_NO_WINDOW)


def stop_all(kind):
    wsl("~/hil_test_r4/arduino_hil/stop_all.sh >/dev/null 2>&1; pkill -f 'gz sim' ; true", 40)
    stop_page()
    for title in ("Gazebo +*", "AeroFC*"):
        subprocess.run(["taskkill", "/f", "/fi", f"WINDOWTITLE eq {title}"], capture_output=True, creationflags=subprocess.CREATE_NO_WINDOW)


def main():
    global trace
    kind = sys.argv[1] if len(sys.argv) > 1 else "sitl"
    out = sys.argv[2] if len(sys.argv) > 2 else None
    if len(sys.argv) > 3:
        trace = open(sys.argv[3], "w", encoding="utf-8")
    stop_page()
    time.sleep(1)
    try:
        run_tests(kind)
    except Stop as e:
        check("the test ran to the end", False, str(e))
    finally:
        for k in VK.values():                # never leave a key down
            user32.keybd_event(k, 0, KEYUP, None)
        stop_all(kind)
    bad = [k for k, v in results.items() if not v["pass"]]
    print(f"{len(results) - len(bad)} of {len(results)} pass", flush=True)
    if out:
        pathlib.Path(out).write_text(json.dumps({"kind": kind, "date": time.strftime("%Y-%m-%d %H:%M"), "results": results}, indent=1), encoding="utf-8")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
