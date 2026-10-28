# 8. The real Gazebo simulator on Windows (WSL + Ubuntu 22.04)

The organisers' simulator is validated on **Ubuntu 22.04 + Gazebo Harmonic**. Your WSL already has Ubuntu 26.04, which Gazebo Harmonic doesn't support, so add Ubuntu 22.04 next to it; nothing is removed. Do this before the event so you can test against the *real* Gazebo once.

## 1. Install Ubuntu 22.04 (once, about 10 minutes)

Open **PowerShell** and run:

```powershell
wsl --install -d Ubuntu-22.04
```

It asks for a new Ubuntu user name and password; remember the password.

## 2. Install Gazebo and copy the simulator (once, 10–20 minutes)

In the **Ubuntu 22.04** window:

```bash
bash AeroFC_R4/tools/gazebo_wsl/setup_gazebo_ubuntu2204.sh
```

It installs the following, copies `uav-hil-simulator-main` to `~/hil_test`, and adds our helper scripts:
- `gz-harmonic`
- the Gazebo Python bindings (`python3-gz-transport13`, `python3-gz-msgs10`); without them the official bridge publishes motor commands very slowly
- `socat` and pyserial

Close the window and open a new Ubuntu 22.04 window afterwards.

## 3. Smoke test with the organisers' fake Arduino (no board)

```bash
cd ~/hil_test
./start_hil.sh --gui
```

You should see Gazebo with the X3 drone and the bridge printing `HELLO_ACK received … HIL session started`. Stop with Ctrl+C (`./stop_hil.sh` cleans up). On some PCs the GUI is slow in WSL; headless (`./start_hil.sh`) is fine for testing.

## 4. Give Ubuntu the Arduino (usbipd-win)

In **PowerShell as Administrator**, once:

```powershell
winget install usbipd
```

Every time you plug the board in:

```powershell
usbipd list                              # find the BUSID of the Arduino (e.g. 2-3)
usbipd bind --busid 2-3                  # only the first time for this board
usbipd attach --wsl --busid 2-3
```

In Ubuntu, `ls /dev/ttyACM*` must now show `/dev/ttyACM0`.

While the board is attached to WSL, Windows (Arduino IDE, simulator page) can't use it. Run `usbipd detach --busid 2-3` to give it back.

## 5. Fly the board in the real Gazebo

```bash
cd ~/hil_test/arduino_hil
./run_hil.sh --gui        # Gazebo + bridge_plus (the official bridge plus live telemetry), board on /dev/ttyACM0
```

- **Pilot input and live plots from Windows:**
  1. Start `START_SIMULATOR.bat`.
  2. Open **More connections**, keep Relay = `http://localhost:8770`, and press **Gazebo in WSL (relay)**.
  3. Keyboard and gamepad now fly the Gazebo drone, and the plots show the FC's STATUS telemetry.
- **Exactly the organisers' setup:**
  1. Run `./run_hil.sh --official`. This is their unmodified `hil_bridge_v1.py`.
  2. Fly with their tool, `python3 control_input.py --interactive`, or run `python3 relay.py` to use our page as the joystick.
  3. There are no live plots in this case. Use the OLED, or a USB-TTL adapter on D1 with "Telemetry only".
- A CSV of the STATUS telemetry is written to `~/hil_test/logs/aerofc_status_*.csv`. The official bridge writes its own `hil_flight_*.csv` too.

## Notes

- **Run Gazebo headless if the PC is slow.** The firmware uses the packets' timestamps, but a Gazebo running slower than real time still makes everything slower.
- If the bridge says `HELLO_ACK timeout`, the board wasn't ready or the port was wrong. Restart the bridge; the firmware answers HELLO at any time.
- The world the organisers' scripts load is `gz_betaflight_bridge/worlds/quadcopter.sdf` (maximum rotor speed 1000 rad/s, Motor max 1.00). The other `quadcopter.sdf` in the repository root has no sensor plugins and can't run HIL.
