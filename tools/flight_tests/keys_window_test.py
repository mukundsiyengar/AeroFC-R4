"""keys_window_test.py - which windows may fly the drone (tools/serve.py).  Run: python tools/flight_tests/keys_window_test.py"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from serve import KEY_NAMES, is_gazebo_window

CASES = [
    ("Gazebo Sim", "msrdc.exe", True),
    ("Gazebo Sim (Ubuntu-22.04)", "MSRDC.EXE", True),
    ("Gazebo + your Arduino (close this window to stop)", "conhost.exe", True),
    ("Gazebo + AeroFC software flight controller (close this window to stop)", "WindowsTerminal.exe", True),
    ("3_FLY_ARDUINO_in_Gazebo.bat - Notepad", "notepad.exe", False),
    ("gazebo - File Explorer", "explorer.exe", False),
    ("Gazebo documentation - Microsoft Edge", "msedge.exe", False),
    ("AeroFC Simulator - Microsoft Edge", "msedge.exe", False),
    ("Command Prompt", "cmd.exe", False),
    ("", "", False),
]
bad = 0
for title, program, want in CASES:
    got = is_gazebo_window(title, program)
    print(("ok  " if got == want else "FAIL"), repr(title), program, "->", got)
    bad += got != want
want_keys = set("wasdqerftlm123gpkh") | {"backspace", "enter", "shift", "arrowup", "arrowdown", "arrowleft", "arrowright"}
if set(KEY_NAMES.values()) != want_keys:
    print("FAIL keys", set(KEY_NAMES.values()) ^ want_keys)
    bad += 1
print("all pass" if not bad else f"{bad} failed")
sys.exit(1 if bad else 0)
