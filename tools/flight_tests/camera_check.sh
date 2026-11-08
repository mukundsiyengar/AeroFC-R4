#!/usr/bin/env bash
# camera_check.sh [sitl|board] seconds picture.png result.json - the camera world with no 3D window: does Gazebo keep up?
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KIND="${1:-sitl}"
RUN=run_sitl.sh
if [[ "$KIND" == "board" ]]; then
  RUN=run_hil.sh
  export GZ_PARTITION=hil_test_r4
fi
export AEROFC_WORLD="$HOME/hil_test_r4/gz_betaflight_bridge/worlds/quadcopter_cam.sdf"
cd ~/hil_test_r4/arduino_hil
./stop_all.sh >/dev/null 2>&1
sleep 2
./$RUN > /tmp/camera_run.log 2>&1 &
RUN_PID=$!
timeout 240 python3 -u "$HERE/camera_check.py" "$2" "$3" "$4" 2>&1 | cut -c1-240
kill $RUN_PID 2>/dev/null
./stop_all.sh >/dev/null 2>&1
grep -E "control delay" /tmp/camera_run.log | tail -2 | cut -c1-110
