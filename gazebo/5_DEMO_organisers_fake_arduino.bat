@echo off
title Organisers' demo: Gazebo + fake Arduino (close this window to stop)
rem The organisers' own smoke test. Their fake Arduino takes off to 1 m by itself and drifts: that is their demo code,
rem not your flight controller. After 20 s a second window opens with their keyboard control
rem (click into it before pressing keys):
rem   w/s forward/back   a/d left/right   q/e turn   u/j throttle up/down   SPACE neutral   x exit
start "Organisers' keyboard control - click here, then press keys" wsl -d Ubuntu-22.04 -- bash -lc "sleep 20; cd ~/hil_test/arduino_hil && python3 control_input.py --interactive"
wsl -d Ubuntu-22.04 -- bash -lc "cd ~/hil_test && ./start_hil.sh --gui"
pause
