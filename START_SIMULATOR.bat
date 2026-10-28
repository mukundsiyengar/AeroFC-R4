@echo off
title AeroFC simulator (keep this window open)
rem Starts a small local web server for the simulator and opens it in Microsoft Edge.
rem (Edge or Chrome is needed for "Connect Arduino": they support Web Serial.)
cd /d "%~dp0"
set PORT=8765
set URL=http://localhost:%PORT%/simulator/index.html
echo.
echo  AeroFC-R4 simulator
echo  -------------------
echo  Opening %URL%
echo  Close this window to stop the simulator.
echo.
start "" msedge "%URL%" 2>nul || start "" "%URL%"
where python >nul 2>nul && (python -m http.server %PORT% --bind 127.0.0.1 & goto :eof)
where py >nul 2>nul && (py -m http.server %PORT% --bind 127.0.0.1 & goto :eof)
echo Python was not found. Install Python 3 from https://www.python.org/downloads/ (tick "Add to PATH").
pause
