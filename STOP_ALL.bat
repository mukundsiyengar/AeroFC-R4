@echo off
title AeroFC-R4: stopping everything
rem Stops Gazebo, the bridge and the page's web server, and gives the R4 Minima back to Windows.
echo  Stopping Gazebo and the bridge...
wsl -d Ubuntu-22.04 -- bash -lc "~/hil_test_r4/arduino_hil/stop_all.sh >/dev/null 2>&1; pkill -f 'gz sim'; true"
taskkill /f /fi "WINDOWTITLE eq R4 Minima to Ubuntu*" >nul 2>nul
taskkill /f /fi "WINDOWTITLE eq Gazebo +*" >nul 2>nul
taskkill /f /fi "WINDOWTITLE eq AeroFC-R4 simulator*" >nul 2>nul
where usbipd >nul 2>nul && usbipd detach --hardware-id 2341:0069 >nul 2>nul
powershell -NoProfile -Command "Get-CimInstance Win32_Process | Where-Object { ($_.Name -like 'python*' -and $_.CommandLine -match 'serve\.py 8790') -or ($_.Name -eq 'msedge.exe' -and $_.CommandLine -match 'AeroFC_Edge_R4' -and $_.CommandLine -notmatch '--type=') } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }"
echo  Done. The board is back on Windows.
timeout /t 4 >nul
