#!/usr/bin/env bash
# nav_gazebo_run.sh [sitl|board] result.json [quick] - starts Gazebo with no 3D window and the flight controller, runs the
# navigation test, and stops everything.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KIND="${1:-sitl}"
OUT="$2"
QUICK="$3"
RUN=run_sitl.sh
if [[ "$KIND" == "board" ]]; then
  RUN=run_hil.sh
  export GZ_PARTITION=hil_test_r4
fi
cd ~/hil_test_r4/arduino_hil
./stop_all.sh >/dev/null 2>&1
sleep 2
./$RUN > /tmp/nav_run.log 2>&1 &
RUN_PID=$!
timeout 2400 python3 -u "$HERE/nav_gazebo_test.py" "$OUT" $QUICK 2>&1 | cut -c1-220
kill $RUN_PID 2>/dev/null
./stop_all.sh >/dev/null 2>&1
grep -E "control delay" /tmp/nav_run.log | tail -3 | cut -c1-120
