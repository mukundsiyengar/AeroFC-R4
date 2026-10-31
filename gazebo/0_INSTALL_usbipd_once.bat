@echo off
title Install usbipd-win (only once)
echo.
echo  Installs usbipd-win. It lets Ubuntu (WSL) use the Arduino's USB port.
echo  Windows asks for permission - say Yes.
echo.
winget install --exact --id dorssel.usbipd-win --accept-source-agreements --accept-package-agreements
echo.
echo  Done. If it asks you to restart Windows, do that once.
pause
