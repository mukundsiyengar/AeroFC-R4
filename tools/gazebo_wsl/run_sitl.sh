#!/usr/bin/env bash
# run_sitl.sh - fly the AeroFC firmware WITHOUT the Arduino in the real Gazebo:
#   socat virtual serial link + Gazebo + aerofc_sitl (our firmware as a Linux program) + bridge_plus.py
#
#   ./run_sitl.sh                 Gazebo headless (fastest)
#   ./run_sitl.sh --gui           with the Gazebo 3D window
#   ./run_sitl.sh --gui -- --set "Alt limit=1" --set "Max alt=20"      settings for the flight controller
#
# Fly it from the simulator page on Windows (More connections -> Gazebo in WSL), or test it with ./sitl_check.py.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
ROOT="$(cd .. && pwd)"
GUI=""
CAMERA=""
SITL_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui) GUI="--gui" ;;
    --camera) CAMERA=1 ;;
    --) shift; SITL_ARGS=("$@"); break ;;
  esac
  shift
done
[[ -x ./aerofc_sitl ]] || { echo "aerofc_sitl missing: run 1_UPDATE_tools_in_Ubuntu.bat on Windows first"; exit 1; }

"$ROOT/stop_hil.sh" >/dev/null 2>&1 || true
pkill -f "aerofc_sitl" 2>/dev/null || true
cleanup() {
  pkill -f "marker_tracker.py" 2>/dev/null || true
  pkill -f "aerofc_sitl" 2>/dev/null || true
  pkill -f "bridge_plus.py" 2>/dev/null || true
  "$ROOT/stop_hil.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# --camera: the world with the down camera and the landing pad, and the program that looks for the pad
if [[ -n "$CAMERA" ]]; then
  export AEROFC_WORLD="$ROOT/gz_betaflight_bridge/worlds/quadcopter_cam.sdf"
  (sleep 8; exec python3 -u "$HOME/hil_test_r4/arduino_hil/marker_tracker.py") &
fi
"$ROOT/start_serial.sh" -b
"$ROOT/start_gazebo.sh" $GUI -b
echo "== starting the AeroFC firmware (SITL) on fc_fake"
./aerofc_sitl --port fc_fake ${SITL_ARGS[@]+"${SITL_ARGS[@]}"} &
sleep 0.5
echo "== starting bridge_plus.py on fc_sim (live plots + pilot on http://127.0.0.1:8770)"
python3 bridge_plus.py
