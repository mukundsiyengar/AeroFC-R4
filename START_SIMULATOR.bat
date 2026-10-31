@echo off
title AeroFC v3 simulator (keep this window open)
rem Starts a small local web server for the simulator and opens it in Microsoft Edge.
rem (Edge or Chrome is needed for "Connect Arduino": they support Web Serial.)
rem v3 uses port 8767, so it never mixes with v2 (port 8766) or v1 (port 8765).
rem
rem   START_SIMULATOR          the page asks how to fly
rem   START_SIMULATOR gazebo   the page connects to Gazebo by itself (the launchers in gazebo\ do this)
rem A plain word, not "?mode=gazebo": a batch file splits its arguments at "=", and the page would lose its mode.
cd /d "%~dp0"
set PORT=8767
set URL=http://localhost:%PORT%/simulator/index.html
if /i "%~1"=="gazebo" set URL=%URL%?mode=gazebo
echo.
echo  AeroFC-R4 v3 simulator
echo  ----------------------
echo  Opening %URL%
echo  Close this window to stop the simulator.
echo.
rem serve.py opens the page itself, once the server is ready to answer it
where python >nul 2>nul && (python tools\serve.py %PORT% "%URL%" & goto :eof)
where py >nul 2>nul && (py tools\serve.py %PORT% "%URL%" & goto :eof)
echo Python was not found. Install Python 3 from https://www.python.org/downloads/ (tick "Add to PATH").
pause
