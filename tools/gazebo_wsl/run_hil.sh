#!/usr/bin/env bash
# run_hil.sh - starts Gazebo + the bridge (with AeroFC telemetry) for your real Arduino, in one command.
#
#   ./run_hil.sh                      Arduino on /dev/ttyACM0, Gazebo headless (fastest, recommended in WSL)
#   ./run_hil.sh --gui                same, with the Gazebo 3D window
#   ./run_hil.sh --port /dev/ttyACM1  other serial port
#   ./run_hil.sh --official           use the organisers' unmodified hil_bridge_v1.py (no live plots)
#
# Plug the Arduino in (and attach it to WSL with usbipd) BEFORE running this: the bridge says HELLO only once.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
ROOT="$(cd .. && pwd)"
PORT=/dev/ttyACM0
GUI=""
BRIDGE=bridge_plus.py
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui) GUI="--gui" ;;
    --port) PORT="$2"; shift ;;
    --official) BRIDGE=hil_bridge_v1.py ;;
  esac
  shift
done
[[ -e "$PORT" ]] || { echo "No $PORT. In Windows PowerShell (admin): usbipd attach --wsl --busid <id>  (see docs/08)"; exit 1; }

export GZ_PARTITION=hil_test
export GZ_SIM_RESOURCE_PATH="$ROOT/gz_betaflight_bridge/models:${GZ_SIM_RESOURCE_PATH:-}"
cleanup() { pkill -f "gz sim" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

echo "== starting Gazebo ($ROOT/gz_betaflight_bridge/worlds/quadcopter.sdf)"
"$ROOT/start_gazebo.sh" $GUI -b
sleep 1
echo "== starting $BRIDGE on $PORT"
python3 "$BRIDGE" --port "$PORT"
