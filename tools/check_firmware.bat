@echo off
rem Compiles the firmware for the Arduino UNO R4 Minima (no board needed) and shows flash / RAM use.
rem Uses the arduino-cli that comes with Arduino IDE 2.
setlocal
set CLI="%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
if not exist %CLI% set CLI=arduino-cli
%CLI% compile --fqbn arduino:renesas_uno:minima --warnings default --build-path "%LOCALAPPDATA%\aerofc_build_r4" "%~dp0..\firmware\AeroFC_R4"
if errorlevel 1 (
  echo.
  echo COMPILE FAILED
  exit /b 1
)
echo.
echo COMPILE OK - you can upload firmware\AeroFC_R4\AeroFC_R4.ino from the Arduino IDE.
