@echo off
title Copy the AeroFC v3 tools into Ubuntu
rem Copies bridge_plus, the software flight controller (SITL) and the start scripts to ~/hil_test/arduino_hil.
rem Run it once, and again after you update the AeroFC_R4_v3 folder.
wsl -d Ubuntu-22.04 --cd "%~dp0.." -- bash tools/gazebo_wsl/install_tools.sh
echo.
pause
