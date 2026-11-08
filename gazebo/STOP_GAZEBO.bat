@echo off
title Stop Gazebo
rem Stops everything the launchers started inside Ubuntu (Gazebo, bridges, software FC, virtual serial link).
wsl -d Ubuntu-22.04 -- bash -lc "~/hil_test_r4/arduino_hil/stop_all.sh"
echo.
echo  Everything stopped.
timeout /t 3 >nul
