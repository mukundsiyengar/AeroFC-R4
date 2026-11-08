#!/usr/bin/env bash
# run_event.sh <sitl|board> <fast|slow> <gui|headless> <full|hover> <result.json> - their files only, as in their README
SCRATCH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FC="$1"; LINK="$2"; VIEW="$3"; MODE="$4"; OUT="$5"
ROOT="$HOME/hil_clean/uav-hil-simulator-main"
if [[ ! -x "$ROOT/start_gazebo.sh" ]]; then
  mkdir -p "$HOME/hil_clean"
  [[ -f "$AEROFC_SIM_ZIP" ]] || { echo "Set AEROFC_SIM_ZIP to the zip of the simulator, so this test can use a clean copy."; exit 1; }
  python3 -c "import zipfile; zipfile.ZipFile('$AEROFC_SIM_ZIP').extractall('$HOME/hil_clean')"
  [[ -d "$ROOT" ]] || { echo "zip layout differs:"; ls "$HOME/hil_clean"; exit 1; }
  chmod +x "$ROOT"/*.sh
fi
cd "$ROOT"
stop_all() {
  pkill -f "event_test.py" 2>/dev/null
  pkill -f "hil_bridge_v1" 2>/dev/null
  pkill -f "aerofc_sitl" 2>/dev/null; pkill -f "fake_arduino" 2>/dev/null
  pkill -f "gz topic" 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
  pkill -f "socat" 2>/dev/null
  sleep 1
  rm -f arduino_hil/fc_sim arduino_hil/fc_fake /tmp/hil_control_input
}
stop_all
trap stop_all EXIT
GUI=""; [[ "$VIEW" == "gui" ]] && GUI="--gui"
./start_gazebo.sh $GUI -b
PORT=""
if [[ "$FC" == "sitl" ]]; then
  ./start_serial.sh -b
  sleep 0.5
  "$HOME/hil_test_r4/arduino_hil/aerofc_sitl" --port "$ROOT/arduino_hil/fc_fake" > /tmp/ev_sitl.log 2>&1 &
  sleep 0.5
elif [[ "$FC" == "fake" ]]; then
  ./start_serial.sh -b
  sleep 0.5
  (cd arduino_hil && exec python3 -u fake_arduino_v1.py > /tmp/ev_fake.log 2>&1) &
  sleep 1.0
else
  PORT="--port /dev/ttyACM0"
fi
cd arduino_hil
if [[ "$LINK" == "slow" ]]; then
  # their file, unedited: the Gazebo Python bindings simply can't be imported, as on a laptop without them
  python3 -u -c "import sys, runpy; sys.modules['gz.transport13'] = None; sys.argv = ['hil_bridge_v1.py'] + sys.argv[1:]; runpy.run_path('hil_bridge_v1.py', run_name='__main__')" $PORT > /tmp/ev_bridge.log 2>&1 &
else
  python3 -u hil_bridge_v1.py $PORT > /tmp/ev_bridge.log 2>&1 &
fi
python3 "$SCRATCH/event_test.py" "$ROOT" "$OUT" "$MODE"
echo "--- bridge says"
grep -vE "^Dispatched" /tmp/ev_bridge.log | tail -14
grep -c "^Dispatched" /tmp/ev_bridge.log
