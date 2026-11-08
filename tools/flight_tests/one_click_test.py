"""one_click_test.py - START_ALL.bat and STOP_ALL.bat, exactly as a double-click runs them.

    python tools/flight_tests/one_click_test.py result.json [camera]

The R4 Minima must be on Windows, not attached to Ubuntu. Nothing here presses a key.
"""
import json
import os
import pathlib
import subprocess
import sys
import threading
import time
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
HIDE = subprocess.CREATE_NO_WINDOW
status = {}
results = {}


def telemetry():
    while True:
        try:
            with urllib.request.urlopen("http://127.0.0.1:8770/events", timeout=5) as r:
                for line in r:
                    if line.startswith(b"data:"):
                        status.update(json.loads(line[5:]))
        except Exception:
            time.sleep(0.5)


def check(name, ok, detail):
    results[name] = {"pass": bool(ok), "detail": detail}
    print(("PASS " if ok else "FAIL ") + name + ": " + str(detail), flush=True)


def usb_state():
    out = subprocess.run(["usbipd", "list"], capture_output=True, text=True, creationflags=HIDE).stdout
    line = next((x for x in out.splitlines() if "2341:0069" in x), "")
    return "attached" if "Attached" in line else ("shared" if "Shared" in line else (line.strip() or "not plugged in"))


def page_server_up():
    try:
        urllib.request.urlopen("http://127.0.0.1:8790/", timeout=2).read(1)
        return True
    except Exception:
        return False


def ubuntu(cmd):
    r = subprocess.run(["wsl.exe", "-d", "Ubuntu-22.04", "--", "bash", "-lc", cmd], capture_output=True, timeout=30, creationflags=HIDE)
    return r.stdout.decode(errors="replace").replace("\0", "").strip()


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else None
    camera = len(sys.argv) > 2 and sys.argv[2] == "camera"
    label = "START_ALL camera" if camera else "START_ALL"
    plugged = usb_state() != "not plugged in"
    subprocess.run(["cmd", "/c", "start", "", str(ROOT / "START_ALL.bat")] + (["camera"] if camera else []), check=True)
    threading.Thread(target=telemetry, daemon=True).start()
    t0 = time.monotonic()
    while status.get("stateName") != "FLYING" and time.monotonic() - t0 < 180:
        time.sleep(0.5)
    took = time.monotonic() - t0
    check(f"{label}: the drone is flying with nothing else done", status.get("stateName") == "FLYING", f"after {took:.0f} s, height {status.get('alt', 0):.2f} m")
    time.sleep(8)
    check(f"{label}: the page sends without being clicked", status.get("pilotHz", 0) >= 10, f"{status.get('pilotHz')} commands a second")
    check(f"{label}: position hold is on", status.get("posStateName") == "HOLDING", status.get("posStateName"))
    if plugged:
        check(f"{label}: it is flying on the real board", usb_state() == "attached", usb_state())
    if camera:
        ok = False
        for _ in range(30):
            if "marker_tracker" in ubuntu("pgrep -af marker_tracker || true"):
                ok = True
                break
            time.sleep(1)
        check(f"{label}: the camera's marker finder is running", ok, ok)

    subprocess.run(["cmd", "/c", str(ROOT / "STOP_ALL.bat")], capture_output=True, timeout=120)
    time.sleep(3)
    left = ubuntu("pgrep -af 'gz sim|bridge_plus|aerofc_sitl|marker_tracker' | grep -v pgrep || true")
    check(f"{label}, then STOP_ALL: Gazebo and the bridge have stopped", not left, left or "none left")
    check(f"{label}, then STOP_ALL: the page's server has stopped", not page_server_up(), "stopped" if not page_server_up() else "still running")
    if plugged:
        check(f"{label}, then STOP_ALL: the board is back on Windows", usb_state() == "shared", usb_state())
    bad = [k for k, v in results.items() if not v["pass"]]
    print(f"{len(results) - len(bad)} of {len(results)} pass", flush=True)
    if out:
        pathlib.Path(out).write_text(json.dumps({"date": time.strftime("%Y-%m-%d %H:%M"), "results": results}, indent=1), encoding="utf-8")
    os._exit(1 if bad else 0)


main()
