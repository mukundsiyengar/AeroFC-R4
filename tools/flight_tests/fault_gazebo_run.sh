#!/usr/bin/env bash
# fault_gazebo_run.sh [sitl|board] result.json - Gazebo with no 3D window and the flight controller, then the checks for phase 11.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KIND="${1:-sitl}"
OUT="$2"
RUN=run_sitl.sh
if [[ "$KIND" == "board" ]]; then
  RUN=run_hil.sh
  export GZ_PARTITION=hil_test_r4
fi
cd ~/hil_test_r4/arduino_hil
./stop_all.sh >/dev/null 2>&1
sleep 2
./$RUN > /tmp/fault_run.log 2>&1 &
RUN_PID=$!
timeout 1500 python3 -u "$HERE/fault_gazebo_test.py" "$OUT" 2>&1 | cut -c1-220
kill $RUN_PID 2>/dev/null
./stop_all.sh >/dev/null 2>&1
