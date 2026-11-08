@echo off
title Copy the AeroFC-R4 tools into Ubuntu
rem Copies bridge_plus, the software flight controller (SITL) and the start scripts to ~/hil_test_r4/arduino_hil.
rem Run it once, and again after you update this folder.
wsl -d Ubuntu-22.04 --cd "%~dp0.." -- bash tools/gazebo_wsl/install_tools.sh
echo.
pause
