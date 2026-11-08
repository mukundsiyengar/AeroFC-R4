# 8. The real Gazebo simulator on Windows (WSL + Ubuntu 22.04)

The organisers' simulator runs on **Ubuntu 22.04 + Gazebo Harmonic**. On Windows it runs inside WSL as **Ubuntu-22.04**. Every command names `Ubuntu-22.04` explicitly, because your default WSL distribution may be a different Ubuntu that Gazebo Harmonic doesn't support.

**The simulator itself isn't in this repository.** Get it first: see [Set it up for the first time](#set-it-up-for-the-first-time) at the end of this page. The tools work on their own copy in `~/hil_test_r4`, so your simulator folder is never changed.

**The quickest way:** after set-up, double-click `START_ALL.bat`. It gives the board to Ubuntu if it is plugged in, then starts Gazebo, the bridge and the page. With no board, the same flight code flies as a program on the PC. `STOP_ALL.bat` stops everything.

## One step at a time: double-click files in `gazebo\`

| File | What it does | When |
|---|---|---|
| `0_INSTALL_usbipd_once.bat` | Installs **usbipd-win** (lets Ubuntu use the Arduino's USB port). | Once, before using the real board in Gazebo |
| `1_UPDATE_tools_in_Ubuntu.bat` | Copies the tools (bridge_plus, software FC, scripts) into `~/hil_test_r4/arduino_hil`. | Once now, and after every update of this folder |
| `2_ARDUINO_to_Ubuntu.bat` | Gives the plugged-in Arduino to Ubuntu (asks for permission once). **Keep its window open.** | Before flying the real board in Gazebo |
| `3_FLY_ARDUINO_in_Gazebo.bat` | Gazebo 3D window + bridge for **your Arduino**, and opens the simulator page in Gazebo mode (keyboard / gamepad, live plots, RC test). | Fly the board in the real Gazebo |
| `4_FLY_SOFTWARE_FC_in_Gazebo.bat` | Same, but **without the board**: the same firmware code runs as a Linux program (SITL). | Practise in the real Gazebo any time |
| `5_DEMO_organisers_fake_arduino.bat` | The organisers' own smoke test plus their keyboard tool in a second window. | Only to see their demo |
| `6_ARDUINO_back_to_Windows.bat` | Gives the Arduino back to Windows (Arduino IDE uploads). | After flying in Gazebo |
| `STOP_GAZEBO.bat` | Stops everything in Ubuntu (Gazebo, bridges, SITL). | If something is left running |

**Typical session with the real board:**
1. Upload the firmware (Arduino IDE).
2. Double-click `2_ARDUINO_to_Ubuntu.bat` → press a key → Yes.
3. Double-click `3_FLY_ARDUINO_in_Gazebo.bat`.
4. Gazebo opens, the drone takes off to 1.5 m, and the page shows live plots.
5. **Check the banner at the top of the page.** It must say **Commands reach the flight controller**. Then fly with the keyboard: **the keys work while the Gazebo window, its black launcher window or the page is the active window, with no click on the page.** The page must stay open, but it can be covered or minimised. With any other program active, the keys do nothing to the drone.
6. Close the Gazebo window (or Ctrl+C in the black window), then double-click `6_ARDUINO_back_to_Windows.bat`.

**Without the board:** double-click `4_FLY_SOFTWARE_FC_in_Gazebo.bat`. That's all.

The page connects to the bridge by itself (it retries every few seconds while Gazebo starts). If a page was already open, its banner says Gazebo is running and offers **Connect to Gazebo**.

**Do my commands reach the drone?** Two places tell you:

| Where | What you want to see | If not |
|---|---|---|
| Banner at the top of the page | Green: `Commands reach the flight controller: 25 per second` (the board's own count; with Gazebo's 3D window on a busy PC it can be about 17, which is fine) | It says what is wrong and what to do |
| Black Ubuntu window | `[bridge_plus] pilot commands from the page: OK` | `NO pilot commands from the page` means no page is in Gazebo mode |

The black window also prints `control delay: typical 5 ms, worst 64 ms` (our values). Above about 80 ms the drone gets unsteady, and it warns you: close other programs.

**Close extra pages:** only one page may fly. Two pages show `Two pilots` in the banner.

**Three things to know about the Gazebo window:**
- **KILL is Backspace there.** Space is Gazebo's own pause key, and it freezes the simulation.
- **Don't maximise it.** On the PC used for testing, Gazebo runs at about 77 % of real time when maximised, about 94 % at its normal size while active, and about 99 % when another window is active. Below real time the drone brakes worse: with Gazebo active it coasted 3.3 to 5.5 m after letting go, against about 2.5 m with Gazebo not active. For the smoothest flight, make the page the active window and leave Gazebo visible beside it.
- **T and R also change Gazebo's mouse tool.** That does nothing to the drone unless you then drag it with the mouse. Press Esc to go back.

The keys come from the page's small web server (`tools/serve.py`), which reads only the flying keys and stores nothing. The page opens in its own Edge window so that Edge doesn't slow it down when it is covered. If you close that window, the commands stop and the drone treats it as a lost RC link.

## The same with PowerShell commands (copy and paste)

Open **PowerShell** in this project's folder, and paste one block at a time.

**Open an Ubuntu 22.04 window** (to type Linux commands yourself):
```powershell
wsl -d Ubuntu-22.04
```
Type `exit` to leave it.

**Copy the tools into Ubuntu** (same as launcher 1):
```powershell
wsl -d Ubuntu-22.04 --cd "$PWD" -- bash tools/gazebo_wsl/install_tools.sh
```

**Fly the software FC in Gazebo, no board** (same as launcher 4). Start the page first:
```powershell
Start-Process ".\START_SIMULATOR.bat" -ArgumentList "gazebo"
```
Then Gazebo:
```powershell
wsl -d Ubuntu-22.04 -- bash -lc "cd ~/hil_test_r4/arduino_hil && ./run_sitl.sh --gui"
```
Stop with **Ctrl+C** in that window.

**Give the Arduino to Ubuntu** (same as launcher 2; first install usbipd once: `winget install --exact --id dorssel.usbipd-win`). In PowerShell **as Administrator**, once per board:
```powershell
usbipd bind --hardware-id 2341:0069
```
Then in a normal PowerShell, keep it open:
```powershell
usbipd attach --wsl --hardware-id 2341:0069 --auto-attach
```

**Fly the Arduino in Gazebo** (same as launcher 3), in a second PowerShell:
```powershell
wsl -d Ubuntu-22.04 -- bash -lc "cd ~/hil_test_r4/arduino_hil && ./run_hil.sh --gui"
```

**Arduino back to Windows:** press Ctrl+C in the `usbipd attach` window, then:
```powershell
usbipd detach --hardware-id 2341:0069
```

**Organisers' smoke test** (their fake Arduino), window 1:
```powershell
wsl -d Ubuntu-22.04 -- bash -lc "cd ~/hil_test_r4 && ./start_hil.sh --gui"
```
Window 2 (their keyboard tool; click into this window before pressing keys):
```powershell
wsl -d Ubuntu-22.04 -- bash -lc "cd ~/hil_test_r4/arduino_hil && python3 control_input.py --interactive"
```

**Stop everything:**
```powershell
wsl -d Ubuntu-22.04 -- bash -lc "~/hil_test_r4/arduino_hil/stop_all.sh"
```

## Why the smoke-test drone "moved on its own"

`start_hil.sh` runs the organisers' **fake Arduino** (`fake_arduino_v1.py`), not your flight controller. It takes off to about 1 m by itself, and its small roll/yaw corrections push the wrong way, so it drifts and turns even when nothing is pressed. Its keys only work in **their keyboard window** (`control_input.py --interactive`):
- w/s, a/d, q/e: pitch, roll, yaw in steps of 0.1 (they stay until SPACE)
- u/j: throttle
- SPACE: neutral
- x: exit

Your own flight controller (launchers 3 and 4) holds its position when nothing is pressed.

## Check that it works (automatic test in the real Gazebo)

With launcher 4 running (or `./run_sitl.sh -- --set "Alt limit=1" --set "Max alt=20"` to include the height-limit check), in a second PowerShell:
```powershell
wsl -d Ubuntu-22.04 -- bash -lc "cd ~/hil_test_r4/arduino_hil && python3 sitl_check.py"
```
It flies take-off, hover, forward + let go, the height limit and an RC-link loss, and prints PASS/FAIL. Our result: **9/9**. Add `--long 10` for 10 minutes of flying around (real R4 results: docs/11). **Close the simulator page first:** an open page is a second pilot, and the script refuses to start.

## Important: Gazebo's compass

In this Gazebo (gz-sim 8), the magnetometer's "north" points **90.8° east of the GPS north**. Gazebo uses its world magnetic model in the wrong axis order and ignores the world file's `<magnetic_field>`.
- The firmware's **Mag decl** is set to **90.8** for this, so heading and GPS agree from the start.
- If a different Gazebo version behaves differently, the flight controller learns the difference from GPS during the first move (tested up to 180°).

## Notes

- **Logs:**
  - `~/hil_test_r4/logs/aerofc_status_*.csv`: our telemetry, including position-hold columns.
  - `hil_flight_*.csv`: the official bridge's own log.
- **Slow PC:** without `--gui`, Gazebo runs headless (faster).
- **Running the organisers' unmodified bridge** with the board: `./run_hil.sh --official`. There are no live plots then. Fly with `control_input.py`, or run `python3 relay.py` to use our page as the joystick.
- **`HELLO_ACK timeout`:** the board wasn't attached or ready. Check launcher 2's window, then start launcher 3 again.
- **The software FC (SITL)** is `~/hil_test_r4/arduino_hil/aerofc_sitl`. Settings: `./aerofc_sitl --list`. Change one: `./run_sitl.sh -- --set "Speed=SPORT"`.

## Set it up for the first time

1. Download the simulator: [uav-hil-simulator](https://github.com/Pramod-Gurav/uav-hil-simulator). Unzip it anywhere.
2. On Windows, install Ubuntu 22.04 and open it:
   ```powershell
   wsl --install -d Ubuntu-22.04
   ```
3. In Ubuntu, from this project's folder, give the setup script the simulator's folder:
   ```bash
   bash tools/gazebo_wsl/setup_gazebo_ubuntu2204.sh /path/to/uav-hil-simulator
   ```
   It asks for your Ubuntu password once and takes 10 to 20 minutes, with about 1 GB of downloads. On Windows, a folder such as `C:\sim` is `/mnt/c/sim` in Ubuntu.
4. Copy our tools in: `gazebo\1_UPDATE_tools_in_Ubuntu.bat`, or `bash tools/gazebo_wsl/install_tools.sh` on Linux.
5. On Windows, install usbipd once: `gazebo\0_INSTALL_usbipd_once.bat`.

