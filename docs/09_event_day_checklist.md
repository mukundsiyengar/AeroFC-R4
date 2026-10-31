# 9. Event-day checklist

## Night before

- [ ] `tools\check_firmware.bat` → COMPILE OK. Upload the final firmware.
- [ ] `tools\run_tests_headless.bat` → ALL PASS 35/35. Screenshot `simulator/tests.html` for your presentation.
- [ ] Fly the board in the Windows simulator: take-off, roll/pitch/yaw, climb, let go (position hold), RC link test, land. Record a CSV log ("Record log").
- [ ] Fly once in the real Gazebo: `gazebo\4_FLY_SOFTWARE_FC_in_Gazebo.bat` (no board), then with the board (launchers 2 and 3). See docs/08.
- [ ] **MENU → Storage → Save settings**, and check "Boot: saved" after a power cycle.
- [ ] With the Arduino IDE open, the OLED keeps running and SYSTEM shows **Reset: power-on** (the v1 reset loop is gone).
- [ ] Charge the laptop. Pack the USB-C data cable, a spare cable, the board with OLED/buttons, the gamepad, and the USB-TTL adapter if you have one.

## At the organisers' PC

0. **Close our simulator page** if it is open and connected to the same bridge: it would be a second pilot.
1. **Plug the Arduino in first**, then they start Gazebo and the bridge (`hil_bridge_v1.py --port /dev/ttyACM0`). The bridge says HELLO only once at start-up.
2. OLED: **PC: HANDSHAKE**, then **RUNNING**. The bridge prints `HELLO_ACK received … HIL session started`.
3. The drone **takes off to 1.5 m by itself** and hovers (altitude-hold take-off, as the problem statement asks).
4. Manoeuvres with their joystick or `control_input.py`:
   - roll right, pitch forward/back, yaw both ways
   - the drone holds height and heading when the sticks are centred
   - **If forward/back is reversed** with their tool: **MENU → Input → Inv pitch → ON** (and roll/yaw likewise). No upload needed.
5. Let go of the sticks: it brakes and **holds its position** (POSITION page: HOLDING, distance in cm).
6. Show the OLED pages: ATTITUDE, ALTITUDE, POSITION, MOTORS, RC INPUT, SYSTEM (sensor rate, µs per step, CRC errors 0). Hold BACK on a menu line to show the built-in help.
7. Land: there is no "land" key in their tool. Use **MENU → Flight → Land** on the board, or hold SELECT on the dashboard (it lands if flying).

## Live plots on event day ("real-time plotting of RC inputs, attitude, altitude")

| Option | What you need |
|---|---|
| A. Our simulator page as plotter | **USB-TTL adapter**: R4 **D1 → adapter RX**, GND → GND. In the page: More connections → **Telemetry only** → pick the adapter's port. Works with the organisers' unmodified bridge. |
| B. Their PC runs our `bridge_plus.py` | If they allow it: the same as their bridge plus live plots in the browser (docs/08). |
| C. No adapter | The OLED shows live attitude, altitude, RC input and motors. Their bridge also writes a CSV (`~/hil_test/logs/hil_flight_*.csv`) with altitude and motors. |

## What to say to the judges (2 minutes)

- **Sensing:** the simulated IMU, baro, compass and GPS arrive over USB. A sensor profile emulates a 16-bit IMU's sensitivity. Calibration covers gyro, level, compass and baro.
- **Estimation:**
  - error-state EKF with gyro-bias estimation
  - accelerometer and compass updates; the compass is yaw-only, so it can't tilt the horizon
  - GPS-aided correction of acceleration errors
  - 3-state altitude Kalman filter
- **Control:**
  - cascaded angle → rate PID with anti-windup and D filtering
  - altitude → climb rate → thrust with hover learning and tilt compensation
  - heading hold
  - thrust-linearized mixer with saturation priority
- **Position hold:**
  - GPS/IMU Kalman filter, and a position → speed → tilt cascade that learns the wind.
  - Brakes and holds its spot within centimetres, and leans into 8 m/s wind.
  - Learns a wrong compass in flight.
  - Speed modes ECO / NORMAL / SPORT.
- **Safety:**
  - arming checks
  - RC-loss failsafe (land on the spot or hover)
  - crash detection
  - watchdog (only while armed)
  - optional height limit up to 1 km
- **Interoperability:** byte-identical to the official protocol (tested against packets made by their own code). Works with their unmodified bridge and keyboard tool.
- **Validation:** 35 automated closed-loop scenarios with metrics, a 2-hour endurance run, and the real R4 in the real Gazebo (over 10 minutes of flying, then holds within 0.14 m) with the same firmware code built for Linux (docs/11).
