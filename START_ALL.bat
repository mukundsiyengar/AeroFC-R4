@echo off
title AeroFC-R4: starting everything
rem One double-click: gives the R4 Minima to Ubuntu if it is plugged in, then starts Gazebo, the bridge and the page.
rem With no board plugged in, the same flight code flies as a program on this PC. STOP_ALL.bat stops everything.
rem Needs once: gazebo\0_INSTALL_usbipd_once.bat. The first time, Windows also asks for permission to share the USB port.
rem   START_ALL          the normal world
rem   START_ALL camera   with the down camera and the landing pad (it uses more of the PC)
set AEROFC_ARGS=
if /i "%~1"=="camera" set AEROFC_ARGS=--camera
cd /d "%~dp0"
echo.
echo  AeroFC-R4
echo  -----------
echo  Copying the tools to Ubuntu...
start "Ubuntu (keeps WSL running)" /min wsl -d Ubuntu-22.04 -- sleep 86400
wsl -d Ubuntu-22.04 --cd "%~dp0." -- bash tools/gazebo_wsl/install_tools.sh
if errorlevel 1 (echo  The tools could not be copied: see docs\08. & pause & exit /b 1)

where usbipd >nul 2>nul || goto software
usbipd list 2>nul | findstr /c:"2341:0069" >nul || goto software

echo  R4 Minima found. Giving it to Ubuntu...
usbipd list 2>nul | findstr /c:"2341:0069" | findstr /c:"Not shared" >nul && (
  echo  Windows asks for permission to share the USB port: say Yes.
  powershell -NoProfile -Command "Start-Process usbipd -ArgumentList 'bind','--hardware-id','2341:0069' -Verb RunAs -Wait"
)
start "R4 Minima to Ubuntu (keep open)" /min usbipd attach --wsl --hardware-id 2341:0069 --auto-attach
wsl -d Ubuntu-22.04 -- bash -c "for i in $(seq 1 30); do [ -e /dev/ttyACM0 ] && exit 0; sleep 1; done; exit 1"
if errorlevel 1 (echo  The board did not appear in Ubuntu. Unplug it, plug it back in, and try again. & pause & exit /b 1)
echo  Starting Gazebo, the bridge and the page for the real board. The OLED says PC:RUNNING when they are connected.
start "" "%~dp0gazebo\3_FLY_ARDUINO_in_Gazebo.bat"
goto done

:software
echo  No R4 Minima found: the same flight code flies as a program on this PC.
start "" "%~dp0gazebo\4_FLY_SOFTWARE_FC_in_Gazebo.bat"

:done
echo.
echo  The drone takes off by itself. Fly with the keys while the Gazebo window or the page is active.
echo  This window can be closed. To stop everything: STOP_ALL.bat
timeout /t 15 >nul
