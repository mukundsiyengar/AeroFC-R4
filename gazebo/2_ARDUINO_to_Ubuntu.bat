@echo off
title Arduino R4 -^> Ubuntu (keep this window open while you fly)
rem Gives the Arduino UNO R4 Minima's USB port to Ubuntu (WSL), so the Gazebo bridge can talk to it.
rem While it is in Ubuntu, the Arduino IDE on Windows cannot use it: run 6_ARDUINO_back_to_Windows.bat to upload again.
where usbipd >nul 2>nul || (echo usbipd is not installed: run 0_INSTALL_usbipd_once.bat first & pause & exit /b)
echo.
echo  Plug in the Arduino (flight controller firmware already uploaded), then press a key.
pause >nul
rem keep Ubuntu running in the background (usbipd needs it)
start "Ubuntu (keeps WSL running)" /min wsl -d Ubuntu-22.04 -- sleep 86400
echo  Windows asks for permission to share the USB port - say Yes.
powershell -NoProfile -Command "Start-Process usbipd -ArgumentList 'bind','--hardware-id','2341:0069' -Verb RunAs -Wait"
usbipd list
echo.
echo  Attaching the Arduino to Ubuntu (it re-attaches by itself if you unplug it).
echo  Keep this window open. In Ubuntu the board is /dev/ttyACM0.
echo  Next: double-click 3_FLY_ARDUINO_in_Gazebo.bat
echo.
usbipd attach --wsl --hardware-id 2341:0069 --auto-attach
pause
