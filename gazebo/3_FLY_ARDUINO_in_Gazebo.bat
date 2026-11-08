@echo off
title Gazebo + your Arduino (close this window to stop)
rem Starts Gazebo (3D window) + the bridge with live telemetry for your real Arduino, and opens the simulator page
rem in "Gazebo" mode: fly with keyboard / gamepad there, see the plots, test the RC link loss.
rem Needs 2_ARDUINO_to_Ubuntu.bat running first.
start "AeroFC simulator page" /min cmd /c ""%~dp0..\START_SIMULATOR.bat" gazebo"
wsl -d Ubuntu-22.04 -- bash -lc "cd ~/hil_test_r4/arduino_hil && ./run_hil.sh --gui %AEROFC_ARGS%"
echo.
echo  Gazebo stopped.
pause
