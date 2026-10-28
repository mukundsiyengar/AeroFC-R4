# 9. Event-day checklist

## Night before

- [ ] `tools\check_firmware.bat` → COMPILE OK. Upload the final firmware.
- [ ] `tools\run_tests_headless.bat` → ALL PASS 12/12. Screenshot `simulator/tests.html` for your presentation.
- [ ] Fly the board in the Windows simulator: take-off, roll/pitch/yaw, climb, land. Record a CSV log ("Record log").
- [ ] If you did the WSL setup, fly once in the real Gazebo (docs/08).
- [ ] **MENU → Storage → Save settings**, and check "Boot: saved" after a power cycle.
- [ ] Charge the laptop. Pack the USB-C data cable, a spare cable, the board with OLED/buttons, the gamepad, and the USB-TTL adapter if you have one.

## At the organisers' PC

1. **Plug the Arduino in first**, then they start Gazebo and the bridge (`hil_bridge_v1.py --port /dev/ttyACM0`). The bridge says HELLO only once at start-up.
2. OLED: **PC: HANDSHAKE**, then **RUNNING**. The bridge prints `HELLO_ACK received … HIL session started`.
3. The drone **takes off to 1.5 m by itself** and hovers (altitude-hold take-off, as the problem statement asks).
4. Manoeuvres with their joystick or `control_input.py`:
   - roll right, pitch forward/back, yaw both ways
   - the drone holds height and heading when the sticks are centred
   - **If forward/back is reversed** with their tool: **MENU → Input → Inv pitch → ON** (and roll/yaw likewise). No upload needed.
5. Show the OLED pages: ATTITUDE, ALTITUDE, MOTORS, RC INPUT, SYSTEM (sensor rate, µs per step, CRC errors 0).
6. Land: there is no "land" key in their tool. Use **MENU → Flight → Land** on the board, or hold SELECT on the dashboard (it lands if flying).

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
- **Safety:** arming checks, failsafe landing, crash detection, watchdog, ceiling.
- **Interoperability:** byte-identical to the official protocol (tested against packets made by their own code). Works with their unmodified bridge and keyboard tool.
- **Validation:** 12 automated closed-loop scenarios with metrics (docs/11), including robustness to delay, sensor rate, inertia, noise, bias and wind.
