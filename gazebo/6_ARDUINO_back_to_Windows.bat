@echo off
title Arduino R4 -^> back to Windows
rem Gives the Arduino back to Windows (for uploading with the Arduino IDE).
rem stop the auto-attach of 2_ARDUINO_to_Ubuntu.bat, then detach
taskkill /im usbipd.exe /f >nul 2>nul
usbipd detach --hardware-id 2341:0069
echo.
echo  Done: the Arduino is back on Windows. Close the "Arduino R4 -> Ubuntu" window if it is still open.
pause
