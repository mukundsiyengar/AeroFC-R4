"""run_all_tests.py - every automatic test, one after the other, then a dated pass or fail report.  RUN_ALL_TESTS.bat runs this.

    python tools/run_all_tests.py [quick] [keys] [noboard] [boardonly]
      quick      fewer flights
      keys       also keys_live_test.py, which types and switches windows: nobody may use the PC during it
      noboard    leave the real board out, even if it is plugged in
      boardonly  only the flights on the real board
"""
import json
import os
import pathlib
import re
import subprocess
import sys
import tempfile
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
DATA = ROOT / "docs" / "test_results"
WSL_ROOT = "/mnt/" + str(ROOT)[0].lower() + str(ROOT)[2:].replace("\\", "/")
HIDE = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
stamp = time.strftime("%Y-%m-%d_%H%M")
args = set(sys.argv[1:])
sections = []


def wsl(command, timeout):
    r = subprocess.run(["wsl.exe", "-d", "Ubuntu-22.04", "--", "bash", "-lc", command], capture_output=True, timeout=timeout, creationflags=HIDE)
    return r.stdout.decode(errors="replace").replace("\0", "")


def section(name, rows):
    """rows: (check, passed, what was measured)"""
    sections.append((name, rows))
    bad = sum(1 for r in rows if not r[1])
    print(f"   {name}: {len(rows) - bad} of {len(rows)} pass", flush=True)
    for r in rows:
        if not r[1]:
            print(f"     FAIL {r[0]}: {r[2]}", flush=True)


def from_json(path):
    if not path.exists():
        return [("the test ran", False, "it wrote no result")]
    d = json.loads(path.read_text(encoding="utf-8"))
    if "results" not in d:
        return [("fly to the red pole in Gazebo", d["pass"], f"stopped {d['miss_m']} m from it")]
    return [(k, v["pass"], str(v["detail"])) for k, v in d["results"].items()]


def browser_tests():
    print("Browser tests...", flush=True)
    subprocess.run(["cmd", "/c", str(ROOT / "tools" / "run_tests_headless.bat")], cwd=ROOT, capture_output=True, creationflags=HIDE)
    html = pathlib.Path(tempfile.gettempdir()) / "aerofc_test_result.html"
    out = subprocess.run([sys.executable, "-I", str(ROOT / "tools" / "summarize_test_dom.py"), str(html)], capture_output=True, text=True, encoding="utf-8", errors="replace").stdout
    rows = []
    for line in out.splitlines():
        m = re.match(r"\[(PASS|FAIL)\] (.*)", line.strip())
        if m:
            rows.append((m.group(2), m.group(1) == "PASS", ""))
    section("Browser tests: the same flight code in a replica", rows or [("the tests ran", False, out[:200])])


def window_rule():
    print("Which windows may fly the drone...", flush=True)
    r = subprocess.run([sys.executable, "-I", str(ROOT / "tools" / "flight_tests" / "keys_window_test.py")], capture_output=True, text=True)
    section("Keys: which windows count", [("window rule", r.returncode == 0, r.stdout.strip().splitlines()[-1] if r.stdout.strip() else "")])


def board_present():
    r = subprocess.run(["usbipd", "list"], capture_output=True, text=True, creationflags=HIDE)
    return "2341:0069" in r.stdout


def attach():
    return subprocess.run(["usbipd", "attach", "--wsl", "--hardware-id", "2341:0069"], capture_output=True, creationflags=HIDE).returncode == 0


def detach():
    subprocess.run(["usbipd", "detach", "--hardware-id", "2341:0069"], capture_output=True, creationflags=HIDE)


TITLES = {"phases": "phases 2 to 5", "marker": "the red pole", "nav": "return to home, go-to, missions, RC loss, holding it",
          "vision": "phases 8 and 9: the camera, landing on the marker", "fault": "phase 11: faults put in on purpose",
          "validate": "phase 12: the whole system, a long mixed flight"}


def gazebo_tests(kind, label):
    for name, script, extra, limit in (("phases", "phases_gazebo_run.sh", "", 1000), ("marker", "marker_run.sh", "", 300),
                                       ("nav", "nav_gazebo_run.sh", " quick" if "quick" in args else "", 2500),
                                       ("vision", "vision_gazebo_run.sh", "", 1000), ("fault", "fault_gazebo_run.sh", "", 1600),
                                       ("validate", "validate_gazebo_run.sh", " 3" if "quick" in args else "", 1300)):
        print(f"{label}: {name}...", flush=True)
        result = DATA / f"{name}_{kind}_{stamp}.json"
        try:
            wsl(f"bash {WSL_ROOT}/tools/flight_tests/{script} {kind} {WSL_ROOT}/docs/test_results/{result.name}{extra}", limit)
        except subprocess.TimeoutExpired:
            pass
        section(f"{label}: {TITLES[name]}", from_json(result))


def keys_test(kind, label):
    print(f"{label}: keys (don't use the PC)...", flush=True)
    result = DATA / f"keys_{kind}_{stamp}.json"
    subprocess.run([sys.executable, "-I", str(ROOT / "tools" / "flight_tests" / "keys_live_test.py"), kind, str(result)], capture_output=True)
    section(f"{label}: keys with no click on the page", from_json(result))


def report():
    total = sum(len(r) for _, r in sections)
    bad = sum(1 for _, rows in sections for r in rows if not r[1])
    lines = [f"# Test report {stamp.replace('_', ' ')}", "", f"**{'ALL PASS' if not bad else f'{bad} FAILED'}: {total - bad} of {total} checks.**", ""]
    for name, rows in sections:
        lines += [f"## {name}", "", "| Check | Result | Measured |", "|---|---|---|"]
        lines += [f"| {r[0]} | {'pass' if r[1] else '**FAIL**'} | {r[2]} |" for r in rows]
        lines.append("")
    path = DATA / f"report_{stamp}.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    print(f"\n{'ALL PASS' if not bad else f'{bad} FAILED'}: {total - bad} of {total} checks.\nReport: {path}", flush=True)
    return bad


def main():
    DATA.mkdir(exist_ok=True)
    subprocess.Popen(["wsl.exe", "-d", "Ubuntu-22.04", "--", "sleep", "18000"], creationflags=HIDE)
    time.sleep(2)
    print(wsl(f"cd {WSL_ROOT} && sed -i 's/\\r$//' tools/gazebo_wsl/*.sh tools/flight_tests/*.sh && bash tools/gazebo_wsl/install_tools.sh", 60).strip(), flush=True)
    if "boardonly" not in args:
        window_rule()
        browser_tests()
        gazebo_tests("sitl", "Real Gazebo, no board")
        if "keys" in args:
            keys_test("sitl", "Real Gazebo, no board")
    if "noboard" not in args and board_present():
        if attach():
            time.sleep(5)
            gazebo_tests("board", "Real Gazebo, real R4 Minima")
            if "keys" in args:
                keys_test("board", "Real Gazebo, real R4 Minima")
            detach()
        else:
            section("Real Gazebo, real R4 Minima", [("the board could be given to Ubuntu", False, "usbipd attach failed")])
    elif "noboard" not in args:
        section("Real Gazebo, real R4 Minima", [("the board was plugged in", False, "not found, so nothing was flown on it")])
    sys.exit(1 if report() else 0)


if __name__ == "__main__":
    main()
