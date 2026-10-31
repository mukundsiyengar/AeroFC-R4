@echo off
title Gazebo + AeroFC software flight controller (close this window to stop)
rem No Arduino needed: the same firmware code runs as a Linux program (SITL) and flies the real Gazebo drone.
rem The simulator page opens in "Gazebo" mode: fly with keyboard / gamepad, live plots, RC link test.
start "AeroFC simulator page" /min cmd /c ""%~dp0..\START_SIMULATOR.bat" gazebo"
wsl -d Ubuntu-22.04 -- bash -lc "cd ~/hil_test/arduino_hil && ./run_sitl.sh --gui"
echo.
echo  Gazebo stopped.
pause
