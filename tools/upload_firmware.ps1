# upload_firmware.ps1 - compiles the firmware and uploads it to the Arduino UNO R4 Minima (UPLOAD_FIRMWARE.bat runs this).
# The board is first asked to go to its bootloader with a "1200 baud touch" (the same as double-pressing RESET),
# because the Arduino IDE's way sometimes ends with "get_status: LIBUSB_ERROR_TIMEOUT".
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$sketch = Join-Path $root "firmware\AeroFC_R4_v3"
$build = Join-Path $env:LOCALAPPDATA "aerofc_build_v3"
$cli = Join-Path $env:LOCALAPPDATA "Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
if (-not (Test-Path $cli)) { $cli = "arduino-cli" }

function Get-R4 { Get-CimInstance Win32_PnPEntity | Where-Object { $_.DeviceID -match 'VID_2341&PID_(0069|0369)' } }
function Test-Bootloader { [bool](Get-R4 | Where-Object { $_.DeviceID -match 'PID_0369' }) }

Write-Host "`n[1/3] Compiling ..."
$ErrorActionPreference = "Continue"     # the tools print normal messages on stderr
& $cli compile --fqbn arduino:renesas_uno:minima --warnings default --build-path $build $sketch 2>&1 |
    Select-String "Sketch uses|Global variables|error" | ForEach-Object { "      $($_.Line)" }
$bin = Join-Path $build "AeroFC_R4_v3.ino.bin"
if ($LASTEXITCODE -ne 0 -or -not (Test-Path $bin)) { throw "Compile failed." }
$ErrorActionPreference = "Stop"

Write-Host "`n[2/3] Asking the board to go to its bootloader ..."
if (-not (Get-R4)) { throw "No Arduino R4 found. Plug it in. (If it is attached to Ubuntu, run gazebo\6_ARDUINO_back_to_Windows.bat.)" }
if (Test-Bootloader) {
    Write-Host "      Already in the bootloader."
} else {
    $name = (Get-R4 | Where-Object { $_.Name -match '\(COM\d+\)' } | Select-Object -First 1).Name
    if ($name -notmatch '\((COM\d+)\)') { throw "The board has no COM port. Double-press its RESET button, then run this again." }
    $com = $Matches[1]
    Write-Host "      Board on $com"
    try {
        $p = New-Object System.IO.Ports.SerialPort $com, 1200
        $p.DtrEnable = $false
        $p.Open()
        Start-Sleep -Milliseconds 300
        $p.Close()
    } catch [System.UnauthorizedAccessException] {
        throw "$com is in use. Press Stop on the simulator page and close the Serial Monitor, then run this again."
    } catch {
        # normal: the port disappears the moment the board restarts into its bootloader
    }
    $deadline = (Get-Date).AddSeconds(8)
    while (-not (Test-Bootloader) -and (Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 200 }
    if (-not (Test-Bootloader)) { throw "The board did not go to its bootloader. Double-press its RESET button (the LED breathes), then run this again." }
}

Write-Host "`n[3/3] Uploading ..."
$dfu = Get-ChildItem (Join-Path $env:LOCALAPPDATA "Arduino15\packages\arduino\tools\dfu-util") -Recurse -Filter dfu-util.exe |
    Sort-Object FullName | Select-Object -Last 1
if (-not $dfu) { throw "dfu-util not found. Install the board package 'Arduino UNO R4 Boards' in the Arduino IDE." }
$ErrorActionPreference = "Continue"
& $dfu.FullName --device 0x2341:0x0369 -D $bin -a0 -Q 2>&1 | Select-String "Download done|error|Error" | ForEach-Object { "      $_" }
if ($LASTEXITCODE -ne 0) { throw "Upload failed. Double-press RESET and run this again." }
$ErrorActionPreference = "Stop"

$deadline = (Get-Date).AddSeconds(8)
while (((Test-Bootloader) -or -not (Get-R4)) -and (Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 200 }
if (Test-Bootloader) { throw "Uploaded, but the board stayed in its bootloader. Press RESET once." }
Write-Host "`nDONE: the board runs the new firmware. Its OLED SYSTEM page should say Reset: upload/USB.`n"
