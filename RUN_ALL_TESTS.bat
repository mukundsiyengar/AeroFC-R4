@echo off
title AeroFC-R4: all tests (about 100 minutes with the board)
rem Runs every automatic test, and writes a dated pass or fail report in docs\test_results. You can use the PC meanwhile.
rem   RUN_ALL_TESTS          everything, with the real R4 Minima if it is plugged in and has this firmware
rem   RUN_ALL_TESTS quick    fewer flights: about 80 minutes with the board
rem   RUN_ALL_TESTS noboard  leave the board out: about half the time
rem   RUN_ALL_TESTS keys     also the keyboard test, which types and switches windows: don't use the PC during it
cd /d "%~dp0"
where python >nul 2>nul || (echo Python was not found. Install it from https://www.python.org/downloads/ & pause & exit /b 1)
python -I tools\run_all_tests.py %*
echo.
pause
