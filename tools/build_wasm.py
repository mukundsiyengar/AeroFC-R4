"""Compiles the firmware core (the very same files that go to the Arduino) into simulator/fc.wasm.

    python tools/build_wasm.py

Uses the zig compiler from pip (`pip install ziglang`); it looks for it in this project's .venv and in the
older FlightController\\.venv.  Nothing native is loaded by the simulator: the browser runs the .wasm file.
"""
import os
import pathlib
import subprocess
import sys

root = pathlib.Path(__file__).resolve().parent.parent
core = root / "firmware" / "AeroFC_R4_v2" / "src" / "core"
api = root / "tools" / "wasm" / "fc_wasm_api.cpp"
out = root / "simulator" / "fc.wasm"

candidates = [
    root / ".venv" / "Scripts" / "python.exe",
    root.parent / "FlightController" / ".venv" / "Scripts" / "python.exe",
    pathlib.Path(sys.executable),
]
py = next((str(c) for c in candidates if c.exists()), sys.executable)

# zig keeps a cache; OneDrive locks files inside synced folders, so keep it in LOCALAPPDATA
cache = pathlib.Path(os.environ.get("LOCALAPPDATA", str(root))) / "aerofc_zig_cache"
env = dict(os.environ, ZIG_LOCAL_CACHE_DIR=str(cache / "local"), ZIG_GLOBAL_CACHE_DIR=str(cache / "global"))

srcs = sorted(str(p) for p in core.glob("*.cpp")) + [str(api)]
cmd = [py, "-m", "ziglang", "cc", "-x", "c++", "-target", "wasm32-wasi", "-mexec-model=reactor", "-O2", "-s", "-std=c++14",
       "-fno-exceptions", "-fno-rtti", "-Wall", "-Wextra", "-Wdouble-promotion", "-Wno-unused-parameter",
       "-o", str(out)] + srcs
r = subprocess.run(cmd, capture_output=True, text=True, env=env)
if r.stdout.strip() or r.stderr.strip():
    print(r.stdout, r.stderr)
if r.returncode != 0 or not out.exists():
    sys.exit("wasm build FAILED")
print("built", out, out.stat().st_size, "bytes")
