# 12. Hand-over sheet for the organisers' laptop

Everything runs on the Arduino. Nothing of ours is installed on the laptop: only the organisers' simulator from GitHub is used.

## Start

1. Plug in the R4, with its OLED and buttons attached. Wait 5 s.
2. Terminal 1:
   ```bash
   cd ~/hil_test && ./start_gazebo.sh --gui
   ```
3. Terminal 2:
   ```bash
   cd ~/hil_test && python3 arduino_hil/hil_bridge_v1.py --port /dev/ttyACM0
   ```
4. Terminal 3:
   ```bash
   cd ~/hil_test/arduino_hil && python3 control_input.py --interactive
   ```

If you use `export GZ_PARTITION=hil_test`, do it in terminals 1 and 2.

**Healthy start:**
- The bridge prints `HELLO_ACK received`, `HIL session started` and `Gazebo Transport: Native (High Perf)`.
- The drone takes off to 1.5 m by itself, about 5 s after the bridge starts, and holds its spot.
- The OLED's first page says `PC:RUNNING`.

## Fly (click on terminal 3 first)

| Key | Action |
|---|---|
| `w` / `s` | forward / back |
| `a` / `d` | left / right |
| `q` / `e` | turn left / right |
| `u` / `j` | throttle up / down (leaves altitude hold) |
| `SPACE` | sticks centred, altitude hold: brakes and holds its spot |
| `x` | exit the tool |

Each press moves that stick by 0.1, and it stays there until `SPACE`.

## Land and take off again

| Way | What to do |
|---|---|
| Keyboard tool | Press `j` twice. It comes down, and the motors stop by themselves after touch-down. To fly again, press `SPACE`, then `u`, then `SPACE` at the height you want. |
| Command | `python3 control_input.py --throttle 13 --duration 0.2` lands gently. `--throttle 12` takes off to 1.5 m. |
| OLED buttons | Hold `SELECT` on the dashboard. |

Timed commands also work, for example `python3 control_input.py --pitch -0.2 --duration 2.0` flies forward for 2 s.

## Live plots and data on the OLED

Press `UP` once from the first page for **PLOTS**: RC input, attitude and altitude, live. `DOWN` steps through STATUS, ATTITUDE, ALTITUDE, POSITION, MOTORS, RC INPUT, SENSORS and SYSTEM.

Their bridge also logs altitude and motors to a CSV file (the path is printed at its start).

## If something goes wrong

| What you see | Fix |
|---|---|
| `HELLO_ACK timeout` | The R4 wasn't ready. Wait 5 s and start the bridge again. |
| `Could not open serial port` | Check `ls /dev/ttyACM*`. If it's permission denied: `sudo usermod -a -G dialout $USER`, then log in again. |
| `Gazebo Transport: Subprocess Fallback`, and `BRIDGE TOO SLOW` on the OLED; the drone stays on the ground | The bridge is in its slow mode, with about one motor update a second. No flight controller can fly on that, so the R4 refuses to take off. Fix: `sudo apt install python3-gz-transport13 python3-gz-msgs10`, then start the bridge again. |
| Keys do nothing | Click on terminal 3. Keys go to the window you clicked last. |
| USB came out: the bridge stops with an error | Plug it back in, wait 5 s and start the bridge again. It takes off by itself. |
| Start again | Press Ctrl+C in terminals 1 and 2, then repeat steps 2 to 4. The R4 can stay plugged in. |
