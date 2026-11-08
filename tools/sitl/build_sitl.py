"""Compiles the firmware core (the very same files that go to the Arduino) into a Linux program for WSL / Ubuntu:
tools/sitl/aerofc_sitl  (static x86_64 binary, no libraries needed).

    python tools/sitl/build_sitl.py

Uses the zig compiler from pip (`pip install ziglang`), like tools/build_wasm.py.
"""
import os
import pathlib
import subprocess
import sys

root = pathlib.Path(__file__).resolve().parent.parent.parent
core = root / "firmware" / "AeroFC_R4" / "src" / "core"
main = root / "tools" / "sitl" / "aerofc_sitl.cpp"
out = root / "tools" / "sitl" / "aerofc_sitl"

candidates = [
    pathlib.Path(os.environ.get("AEROFC_ZIG_PYTHON", "no such file")),
    root / ".venv" / "Scripts" / "python.exe",
    pathlib.Path(sys.executable),
]
py = next((str(c) for c in candidates if c.exists()), sys.executable)

cache = pathlib.Path(os.environ.get("LOCALAPPDATA", str(root))) / "aerofc_zig_cache"
env = dict(os.environ, ZIG_LOCAL_CACHE_DIR=str(cache / "local"), ZIG_GLOBAL_CACHE_DIR=str(cache / "global"))

# the UI (OLED menu, font, graphics) is not needed without a display
skip = {"ui_menu.cpp", "gfx.cpp"}
srcs = sorted(str(p) for p in core.glob("*.cpp") if p.name not in skip) + [str(main)]
cmd = [py, "-m", "ziglang", "c++", "-target", "x86_64-linux-musl", "-O2", "-s", "-static", "-std=c++14",
       "-fno-exceptions", "-fno-rtti", "-Wall", "-Wextra", "-Wdouble-promotion", "-Wno-unused-parameter",
       "-o", str(out)] + srcs
r = subprocess.run(cmd, capture_output=True, text=True, env=env)
if r.stdout.strip() or r.stderr.strip():
    print(r.stdout, r.stderr)
if r.returncode != 0 or not out.exists():
    sys.exit("SITL build FAILED")
print("built", out, out.stat().st_size, "bytes")
