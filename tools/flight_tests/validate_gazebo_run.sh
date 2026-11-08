#!/usr/bin/env bash
# validate_gazebo_run.sh [sitl|board] result.json [minutes] - Gazebo with no 3D window and the flight controller, then the checks for phase 12.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KIND="${1:-sitl}"
OUT="$2"
MINUTES="${3:-10}"
RUN=run_sitl.sh
if [[ "$KIND" == "board" ]]; then
  RUN=run_hil.sh
  export GZ_PARTITION=hil_test_r4
fi
cd ~/hil_test_r4/arduino_hil
./stop_all.sh >/dev/null 2>&1
sleep 2
./$RUN > /tmp/validate_run.log 2>&1 &
RUN_PID=$!
timeout 1200 python3 -u "$HERE/validate_gazebo_test.py" "$OUT" "$MINUTES" 2>&1 | cut -c1-220
kill $RUN_PID 2>/dev/null
./stop_all.sh >/dev/null 2>&1
