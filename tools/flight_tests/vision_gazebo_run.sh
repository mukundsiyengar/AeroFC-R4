#!/usr/bin/env bash
# vision_gazebo_run.sh [sitl|board] result.json - the camera world with no 3D window, the tracker, then the checks for phases 8 and 9.
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
./$RUN --camera > /tmp/vision_run.log 2>&1 &
RUN_PID=$!
timeout 900 python3 -u "$HERE/vision_gazebo_test.py" "$OUT" 2>&1 | cut -c1-220
kill $RUN_PID 2>/dev/null
pkill -f marker_tracker.py
./stop_all.sh >/dev/null 2>&1
grep -E "marker_tracker|control delay" /tmp/vision_run.log | tail -6 | cut -c1-110
