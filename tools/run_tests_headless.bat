@echo off
rem Runs the automated flight tests in a hidden Edge window and prints PASS / FAIL.
rem (Same tests as simulator\tests.html, which you can also open in the browser.)
setlocal
cd /d "%~dp0.."
set PORT=8766
start "aerofc-test-server" /min python -m http.server %PORT% --bind 127.0.0.1
timeout /t 2 /nobreak >nul
set EDGE="%ProgramFiles(x86)%\Microsoft\Edge\Application\msedge.exe"
if not exist %EDGE% set EDGE="%ProgramFiles%\Microsoft\Edge\Application\msedge.exe"
%EDGE% --headless=new --disable-gpu --no-first-run --user-data-dir="%TEMP%\aerofc_edge_test" --virtual-time-budget=180000 --dump-dom "http://localhost:%PORT%/simulator/tests.html" > "%TEMP%\aerofc_test_result.html" 2>nul
taskkill /fi "WINDOWTITLE eq aerofc-test-server*" /f >nul 2>nul
python -I tools\summarize_test_dom.py "%TEMP%\aerofc_test_result.html"
