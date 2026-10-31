@echo off
title Upload the AeroFC firmware to the Arduino R4 Minima
rem Compiles and uploads, no Arduino IDE window needed. Press Stop on the simulator page first (it uses the COM port).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\upload_firmware.ps1"
if errorlevel 1 echo. & echo UPLOAD FAILED: see the message above.
echo.
pause
